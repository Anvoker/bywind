//! Pull GFS UGRD/VGRD-at-10m messages from NOAA's AWS Open Data bucket
//! into a concatenated GRIB2 stream.
//!
//! The output is a regular GRIB2 file the `grib2` loader already
//! understands, so the bake / encode / search pipeline downstream is
//! unchanged. For each frame timestamp in `[start, end)` stepped by
//! `interval_h` hours, this module resolves the GFS cycle (latest
//! 00/06/12/18z at or before the timestamp) and forecast hour (0..=5)
//! that produces that frame, fetches the `.idx` sidecar from
//! `s3://noaa-gfs-bdp-pds`, locates the UGRD / VGRD-at-10m byte ranges,
//! and `HTTP Range`-fetches just those into the writer. Total transfer
//! is a couple of MB per frame instead of the ~500 MB full-file size.
//!
//! ## Cycle / forecast-hour mapping
//!
//! GFS publishes a new cycle every 6 hours. Each cycle ships `f000`
//! (analysis), `f001`..`f005` (1- to 5-hour short-range forecasts), then
//! longer leads we don't pull. So with the default `interval_h = 1` we
//! get seamless 1-hour cadence: cycle N's `f000`..`f005` followed by
//! cycle N+1's `f000` for the next hour.
//!
//! Any `interval_h` where every frame's forecast hour lands in `[0, 5]`
//! works: 1 (cadence 1h, uses f000..f005 of every cycle), 2 (cadence 2h,
//! uses f000/f002/f004 of every cycle), 3 (cadence 3h, uses f000/f003),
//! and 6 (cadence 6h, only `f000`s = analyses). 4 and 5 produce a forecast
//! hour outside [0, 5] on the second frame and are rejected up front.
//!
//! ## Errors
//!
//! Failures from individual frame fetches don't abort: the caller's
//! `progress` callback receives a `Skipped` event with the reason and the
//! loop continues. The terminal error is reserved for situations that
//! can't be recovered — invalid spec, I/O failure on the *writer*, or
//! every frame failing.

use std::io::{Read as _, Write};
use std::ops::ControlFlow;
use std::path::Path;
use std::time::Duration;

use chrono::{
    DateTime, Datelike as _, NaiveDate, NaiveDateTime, NaiveTime, TimeDelta, TimeZone as _,
    Timelike as _, Utc,
};

/// NOAA's public S3 bucket mirror, addressed via HTTPS rather than the
/// `s3://` protocol so we can use plain `ureq` without an AWS SDK.
const BUCKET: &str = "https://noaa-gfs-bdp-pds.s3.amazonaws.com";
/// Hours between successive GFS cycles (00z, 06z, 12z, 18z).
const CYCLE_HOURS: i64 = 6;
/// Highest forecast hour we pull from a cycle. Cycle N's `f005` is
/// followed by cycle N+1's `f000` one hour later, so f005 is the last
/// frame we need before the next cycle is published.
const MAX_FORECAST_HOUR: u32 = 5;

/// Inputs for [`fetch_to_grib2`]. All fields are required — callers compute
/// `end` themselves from a duration or the current UTC time as needed.
#[derive(Debug, Clone)]
pub struct FetchSpec {
    /// First frame timestamp (inclusive). Must land on a GFS cycle hour
    /// (00, 06, 12, or 18 UTC); the bucket has no analyses at other
    /// hours.
    pub start: DateTime<Utc>,
    /// Frame timestamp upper bound (exclusive). Must be strictly after
    /// `start`.
    pub end: DateTime<Utc>,
    /// Hours between successive frames. Must be one of 1, 2, 3, or 6.
    pub interval_h: u32,
}

/// One callback notification from [`fetch_to_grib2`]. Caller decides
/// whether to print to stderr, push into a UI progress bar, etc.
#[derive(Debug, Clone)]
pub enum FetchProgress {
    /// A frame's byte ranges have been appended to the writer.
    Fetched {
        /// 1-based frame number in this run.
        idx: u32,
        /// Total frames the spec expanded to.
        total: u32,
        /// Frame's wall-clock UTC timestamp.
        timestamp: DateTime<Utc>,
        /// Bytes pulled for this frame (compressed GRIB2 messages).
        bytes: u64,
    },
    /// A frame's `.idx` or message data could not be retrieved; the
    /// frame is missing from the output. The loop continues with the
    /// next frame.
    Skipped {
        idx: u32,
        total: u32,
        timestamp: DateTime<Utc>,
        reason: String,
    },
}

/// Hard errors from [`fetch_to_grib2`]. Per-frame failures are not
/// returned here — they're delivered through `progress` as
/// `FetchProgress::Skipped` instead.
#[derive(Debug)]
#[non_exhaustive]
pub enum FetchError {
    /// `start` isn't a 6-hour cycle boundary.
    StartNotOnCycle { hour: u32 },
    /// `end <= start`.
    BadWindow,
    /// `interval_h` not in `{1, 2, 3, 6}`.
    BadInterval { interval_h: u32 },
    /// The cycle-iteration loop walked off the end of the supported
    /// `[0, 5]` forecast-hour range. Currently only reachable through
    /// internal bugs since [`FetchError::BadInterval`] catches the
    /// known offending values up front.
    ForecastHourOutOfRange { forecast_hour: u32 },
    /// I/O failure on the output writer. Errors from the network side
    /// land in `FetchProgress::Skipped` instead.
    Io(std::io::Error),
    /// Every frame was skipped — likely a dead bucket or an entirely
    /// out-of-archive window. Carries the count for diagnostics.
    NoFramesFetched { skipped: u32 },
}

impl std::fmt::Display for FetchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::StartNotOnCycle { hour } => write!(
                f,
                "start hour {hour} is not a GFS cycle (must be 00, 06, 12, or 18 UTC)",
            ),
            Self::BadWindow => f.write_str("end must be strictly after start"),
            Self::BadInterval { interval_h } => write!(
                f,
                "interval_h={interval_h} would require a forecast hour > 5; \
                 use one of 1, 2, 3, or 6",
            ),
            Self::ForecastHourOutOfRange { forecast_hour } => write!(
                f,
                "internal error: forecast hour {forecast_hour} exceeds f005",
            ),
            Self::Io(e) => write!(f, "I/O error: {e}"),
            Self::NoFramesFetched { skipped } => write!(
                f,
                "no frames fetched successfully ({skipped} attempts skipped)",
            ),
        }
    }
}

impl std::error::Error for FetchError {}

impl From<std::io::Error> for FetchError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

/// Pull every frame the spec describes from the GFS bucket and write the
/// concatenated GRIB2 stream into `out`.
///
/// `progress` is invoked once per frame (success or skip) so callers
/// can render a progress UI; returning [`ControlFlow::Break`] from the
/// callback ends the fetch early, treats the partial output as success,
/// and returns [`FetchStats`] describing what was actually transferred.
///
/// # Errors
///
/// Returns [`FetchError::StartNotOnCycle`] / [`FetchError::BadWindow`] /
/// [`FetchError::BadInterval`] for invalid specs;
/// [`FetchError::Io`] when writing to `out` fails;
/// [`FetchError::NoFramesFetched`] when every frame was skipped (or the
/// loop was cancelled before any frame succeeded). Per-frame network
/// failures are emitted as [`FetchProgress::Skipped`] and do not
/// terminate the loop.
pub fn fetch_to_grib2<W: Write>(
    spec: &FetchSpec,
    out: &mut W,
    mut progress: impl FnMut(FetchProgress) -> ControlFlow<()>,
) -> Result<FetchStats, FetchError> {
    validate_spec(spec)?;

    let agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(120)))
        .user_agent("bywind-fetch/0.1")
        .build()
        .new_agent();

    let total = expected_frame_count(spec);
    let mut fetched: u32 = 0;
    let mut skipped: u32 = 0;
    let mut total_bytes: u64 = 0;
    let mut idx: u32 = 0;

    let mut frame_t = spec.start;
    while frame_t < spec.end {
        idx += 1;
        let (cycle, forecast_hour) = resolve_cycle(frame_t)?;

        let yyyymmdd = format!("{:04}{:02}{:02}", cycle.year(), cycle.month(), cycle.day());
        let hh = format!("{:02}", cycle.hour());
        let f_url =
            format!("{BUCKET}/gfs.{yyyymmdd}/{hh}/atmos/gfs.t{hh}z.pgrb2.0p25.f{forecast_hour:03}");
        let idx_url = format!("{f_url}.idx");

        let idx_text = match fetch_text(&agent, &idx_url) {
            Ok(s) => s,
            Err(e) => {
                let cf = progress(FetchProgress::Skipped {
                    idx,
                    total,
                    timestamp: frame_t,
                    reason: format!("idx unavailable: {e}"),
                });
                skipped += 1;
                if cf.is_break() {
                    break;
                }
                frame_t += TimeDelta::hours(i64::from(spec.interval_h));
                continue;
            }
        };
        let ranges = parse_idx_for_10m_wind(&idx_text);
        if ranges.len() < 2 {
            let cf = progress(FetchProgress::Skipped {
                idx,
                total,
                timestamp: frame_t,
                reason: format!("UGRD/VGRD@10m not found in idx (got {} hits)", ranges.len()),
            });
            skipped += 1;
            if cf.is_break() {
                break;
            }
            frame_t += TimeDelta::hours(i64::from(spec.interval_h));
            continue;
        }

        let mut frame_bytes: u64 = 0;
        let mut failed: Option<String> = None;
        for (offset, end) in &ranges {
            let range_header = match end {
                Some(e) => format!("bytes={}-{}", offset, e - 1),
                None => format!("bytes={offset}-"),
            };
            match fetch_range(&agent, &f_url, &range_header, out) {
                Ok(n) => frame_bytes += n,
                Err(e) => {
                    failed = Some(format!("range fetch failed: {e}"));
                    break;
                }
            }
        }
        let cf = if let Some(reason) = failed {
            skipped += 1;
            progress(FetchProgress::Skipped {
                idx,
                total,
                timestamp: frame_t,
                reason,
            })
        } else {
            fetched += 1;
            total_bytes += frame_bytes;
            progress(FetchProgress::Fetched {
                idx,
                total,
                timestamp: frame_t,
                bytes: frame_bytes,
            })
        };
        if cf.is_break() {
            break;
        }
        frame_t += TimeDelta::hours(i64::from(spec.interval_h));
    }

    if fetched == 0 {
        return Err(FetchError::NoFramesFetched { skipped });
    }
    Ok(FetchStats {
        fetched,
        skipped,
        total_bytes,
    })
}

/// Summary of one [`fetch_to_grib2`] run, returned on success.
#[derive(Debug, Clone, Copy)]
pub struct FetchStats {
    pub fetched: u32,
    pub skipped: u32,
    pub total_bytes: u64,
}

fn validate_spec(spec: &FetchSpec) -> Result<(), FetchError> {
    let hour = spec.start.hour();
    if u32::try_from(CYCLE_HOURS).is_ok_and(|c| hour % c != 0) {
        return Err(FetchError::StartNotOnCycle { hour });
    }
    if spec.end <= spec.start {
        return Err(FetchError::BadWindow);
    }
    // The supported intervals are exactly those where every frame's
    // forecast-hour offset stays in [0, 5]. Equivalently: `interval`
    // divides 6 (1, 2, 3, 6), so consecutive frames either stay within
    // the same cycle (offset += interval, capped at 5) or wrap to the
    // next cycle (offset = 0).
    if !matches!(spec.interval_h, 1 | 2 | 3 | 6) {
        return Err(FetchError::BadInterval {
            interval_h: spec.interval_h,
        });
    }
    Ok(())
}

fn expected_frame_count(spec: &FetchSpec) -> u32 {
    let total_hours = (spec.end - spec.start).num_hours().max(0);
    let interval = i64::from(spec.interval_h).max(1);
    // ceil division so the first uncovered frame past the window doesn't
    // get counted; matches the half-open `[start, end)` loop in the
    // body.
    u32::try_from(total_hours.div_euclid(interval)).unwrap_or(u32::MAX)
}

fn resolve_cycle(frame_t: DateTime<Utc>) -> Result<(DateTime<Utc>, u32), FetchError> {
    let cycle_hour = frame_t.hour() / (CYCLE_HOURS as u32) * (CYCLE_HOURS as u32);
    let cycle = frame_t
        .with_hour(cycle_hour)
        .expect("cycle hour <= frame hour <= 23");
    let forecast_hour = frame_t.hour() - cycle_hour;
    if forecast_hour > MAX_FORECAST_HOUR {
        return Err(FetchError::ForecastHourOutOfRange { forecast_hour });
    }
    Ok((cycle, forecast_hour))
}

/// One byte range from a `.idx` file: `(start_offset, end_offset)`.
/// `end_offset` is `None` for the final message in the file (the HTTP
/// Range request becomes open-ended `bytes=N-`).
type ByteRange = (u64, Option<u64>);

/// Parse a GFS-style `.idx` and return the byte ranges spanning every
/// UGRD@10m and VGRD@10m message it lists. `.idx` lines look like:
///
/// ```text
/// 1:0:d=2026030100:HGT:surface:anl:
/// 2:5482:d=2026030100:UGRD:10 m above ground:anl:
/// 3:11034:d=2026030100:VGRD:10 m above ground:anl:
/// ```
///
/// Field 0 = 1-indexed message number; field 1 = byte offset; field 3 =
/// short name; field 4 = level. The end offset of message *i* is the
/// start offset of message *i+1*; the last message is open-ended.
fn parse_idx_for_10m_wind(idx: &str) -> Vec<ByteRange> {
    let parsed: Vec<(u64, &str, &str)> = idx
        .lines()
        .filter_map(|line| {
            let mut parts = line.split(':');
            let _msg_num = parts.next()?;
            let offset: u64 = parts.next()?.parse().ok()?;
            let _date = parts.next()?;
            let var = parts.next()?;
            let level = parts.next()?;
            Some((offset, var, level))
        })
        .collect();

    let mut out = Vec::new();
    for (i, (offset, var, level)) in parsed.iter().enumerate() {
        let is_wind = matches!(*var, "UGRD" | "VGRD") && *level == "10 m above ground";
        if !is_wind {
            continue;
        }
        let end = parsed.get(i + 1).map(|next| next.0);
        out.push((*offset, end));
    }
    out
}

fn fetch_text(agent: &ureq::Agent, url: &str) -> Result<String, Box<dyn std::error::Error>> {
    let mut resp = agent.get(url).call()?;
    Ok(resp.body_mut().read_to_string()?)
}

/// Parse `YYYYMMDD` (hour defaults to 00z) or `YYYYMMDDHH` into a
/// `DateTime<Utc>`.
///
/// The shape matches the GFS bucket's own `YYYYMMDD/HH` split, which is
/// also what `bywind-cli fetch` and `bywind-viz`'s fetch dialog accept
/// on input.
///
/// # Errors
///
/// Returns a [`WhenParseError`] for inputs that aren't 8 or 10 ASCII
/// digits, or whose date / hour components are out of range.
pub fn parse_yyyymmddhh(s: &str) -> Result<DateTime<Utc>, WhenParseError> {
    let (date_part, hour) = match s.len() {
        8 => (s, 0u32),
        10 => {
            let (d, h) = s.split_at(8);
            let hour = h.parse::<u32>().map_err(|source| WhenParseError::BadHour {
                hour: h.to_owned(),
                source,
            })?;
            (d, hour)
        }
        _ => {
            return Err(WhenParseError::BadLength {
                len: s.len(),
                value: s.to_owned(),
            });
        }
    };
    let date = NaiveDate::parse_from_str(date_part, "%Y%m%d").map_err(|source| {
        WhenParseError::BadDate {
            date: date_part.to_owned(),
            source,
        }
    })?;
    let time =
        NaiveTime::from_hms_opt(hour, 0, 0).ok_or(WhenParseError::HourOutOfRange { hour })?;
    Ok(Utc.from_utc_datetime(&NaiveDateTime::new(date, time)))
}

/// Failure modes from [`parse_yyyymmddhh`]. Each carries enough context
/// to render a user-actionable message without re-deriving from the
/// input.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum WhenParseError {
    /// Input wasn't 8 or 10 characters long.
    BadLength { len: usize, value: String },
    /// Date component (`YYYYMMDD`) didn't parse.
    BadDate {
        date: String,
        source: chrono::ParseError,
    },
    /// Hour component wasn't a base-10 integer.
    BadHour {
        hour: String,
        source: std::num::ParseIntError,
    },
    /// Hour component was numeric but outside `0..=23`.
    HourOutOfRange { hour: u32 },
}

impl std::fmt::Display for WhenParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BadLength { len, value } => write!(
                f,
                "expected YYYYMMDD or YYYYMMDDHH, got `{value}` ({len} chars)",
            ),
            Self::BadDate { date, source } => write!(f, "invalid date `{date}`: {source}"),
            Self::BadHour { hour, source } => write!(f, "invalid hour `{hour}`: {source}"),
            Self::HourOutOfRange { hour } => write!(f, "invalid hour {hour}"),
        }
    }
}

impl std::error::Error for WhenParseError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::BadDate { source, .. } => Some(source),
            Self::BadHour { source, .. } => Some(source),
            Self::BadLength { .. } | Self::HourOutOfRange { .. } => None,
        }
    }
}

/// Read a GRIB2 file, decode it, and re-encode it as `.wcav`.
///
/// Reads from `grib2_path`, decodes via the GRIB2 loader, writes the
/// resulting [`crate::TimedWindMap`] to `wcav_path`, and returns the
/// in-memory map so the caller doesn't need to re-decode the file it
/// just wrote.
///
/// Used by the two `fetch` frontends after [`fetch_to_grib2`] has
/// streamed the GFS messages to a staging GRIB2 — they delete the
/// staging file themselves once this returns. The function only deals
/// with the two named files; staging-path naming and cleanup are the
/// caller's concern.
///
/// # Errors
///
/// Returns [`TranscodeError::OpenGrib2`] if `grib2_path` can't be
/// opened, [`TranscodeError::DecodeGrib2`] / [`TranscodeError::CreateWcav`]
/// / [`TranscodeError::EncodeWcav`] for the corresponding pipeline
/// steps.
pub fn transcode_grib2_to_wcav(
    grib2_path: &Path,
    wcav_path: &Path,
) -> Result<crate::TimedWindMap, TranscodeError> {
    use std::fs::File;
    use std::io::{BufReader, BufWriter};

    let reader =
        BufReader::new(
            File::open(grib2_path).map_err(|source| TranscodeError::OpenGrib2 {
                path: grib2_path.to_path_buf(),
                source,
            })?,
        );
    let map = crate::TimedWindMap::from_grib2_reader(reader, 1, None).map_err(|source| {
        TranscodeError::DecodeGrib2 {
            path: grib2_path.to_path_buf(),
            source,
        }
    })?;
    let writer =
        BufWriter::new(
            File::create(wcav_path).map_err(|source| TranscodeError::CreateWcav {
                path: wcav_path.to_path_buf(),
                source,
            })?,
        );
    crate::wind_av1::encode(&map, writer, crate::wind_av1::EncodeParams::default()).map_err(
        |source| TranscodeError::EncodeWcav {
            path: wcav_path.to_path_buf(),
            source,
        },
    )?;
    Ok(map)
}

/// Failure modes from [`transcode_grib2_to_wcav`].
#[derive(Debug)]
#[non_exhaustive]
pub enum TranscodeError {
    /// Could not open the input GRIB2 file.
    OpenGrib2 {
        path: std::path::PathBuf,
        source: std::io::Error,
    },
    /// Could not decode the GRIB2 stream.
    DecodeGrib2 {
        path: std::path::PathBuf,
        source: crate::grib2::LoadError,
    },
    /// Could not create the output `.wcav` file.
    CreateWcav {
        path: std::path::PathBuf,
        source: std::io::Error,
    },
    /// Could not encode the `.wcav` payload.
    EncodeWcav {
        path: std::path::PathBuf,
        source: crate::wind_av1::EncodeError,
    },
}

impl std::fmt::Display for TranscodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::OpenGrib2 { path, source } => {
                write!(f, "opening {}: {source}", path.display())
            }
            Self::DecodeGrib2 { path, source } => {
                write!(f, "decoding GRIB2 from {}: {source}", path.display())
            }
            Self::CreateWcav { path, source } => {
                write!(f, "creating {}: {source}", path.display())
            }
            Self::EncodeWcav { path, source } => {
                write!(f, "encoding wind_av1 at {}: {source}", path.display())
            }
        }
    }
}

impl std::error::Error for TranscodeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::OpenGrib2 { source, .. } | Self::CreateWcav { source, .. } => Some(source),
            Self::DecodeGrib2 { source, .. } => Some(source),
            Self::EncodeWcav { source, .. } => Some(source),
        }
    }
}

fn fetch_range(
    agent: &ureq::Agent,
    url: &str,
    range_header: &str,
    out: &mut impl Write,
) -> Result<u64, Box<dyn std::error::Error>> {
    let mut resp = agent.get(url).header("Range", range_header).call()?;
    let mut reader = resp.body_mut().as_reader();
    // Heap-allocate the read buffer — 64 KiB exceeds the per-thread
    // stack-frame size clippy flags.
    let mut buf = vec![0u8; 64 * 1024];
    let mut total: u64 = 0;
    loop {
        let n = reader.read(&mut buf)?;
        if n == 0 {
            break;
        }
        out.write_all(&buf[..n])?;
        total += n as u64;
    }
    Ok(total)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(h: u32) -> DateTime<Utc> {
        DateTime::<Utc>::from_timestamp(1_739_404_800 + i64::from(h) * 3600, 0)
            .expect("constant epoch is representable")
    }

    #[test]
    fn expected_frame_count_is_ceil_division_of_half_open_window() {
        // Window [00z, 07z) at 1 h cadence → 7 frames (00..=06).
        let spec = FetchSpec {
            start: at(0),
            end: at(7),
            interval_h: 1,
        };
        assert_eq!(expected_frame_count(&spec), 7);
        // Window [00z, 12z) at 6 h cadence → 2 frames (00, 06).
        let spec = FetchSpec {
            start: at(0),
            end: at(12),
            interval_h: 6,
        };
        assert_eq!(expected_frame_count(&spec), 2);
    }

    #[test]
    fn validate_rejects_non_cycle_start() {
        let spec = FetchSpec {
            start: at(1),
            end: at(10),
            interval_h: 1,
        };
        assert!(matches!(
            validate_spec(&spec),
            Err(FetchError::StartNotOnCycle { hour: 1 })
        ));
    }

    #[test]
    fn validate_rejects_unsupported_interval() {
        let spec = FetchSpec {
            start: at(0),
            end: at(24),
            interval_h: 4,
        };
        assert!(matches!(
            validate_spec(&spec),
            Err(FetchError::BadInterval { interval_h: 4 })
        ));
    }

    #[test]
    fn validate_rejects_end_before_start() {
        let spec = FetchSpec {
            start: at(6),
            end: at(0),
            interval_h: 1,
        };
        assert!(matches!(validate_spec(&spec), Err(FetchError::BadWindow)));
    }

    #[test]
    fn resolve_cycle_maps_offset_hours_to_forecast_indices() {
        // 00z → cycle 00z, f000.
        let (cycle, fh) = resolve_cycle(at(0)).expect("0h is in range");
        assert_eq!(cycle.hour(), 0);
        assert_eq!(fh, 0);
        // 05z → cycle 00z, f005.
        let (cycle, fh) = resolve_cycle(at(5)).expect("5h is in range");
        assert_eq!(cycle.hour(), 0);
        assert_eq!(fh, 5);
        // 06z → cycle 06z, f000 (new cycle).
        let (cycle, fh) = resolve_cycle(at(6)).expect("6h is in range");
        assert_eq!(cycle.hour(), 6);
        assert_eq!(fh, 0);
    }

    #[test]
    fn idx_parser_extracts_only_10m_wind_ranges() {
        let idx = "\
            1:0:d=2026030100:HGT:surface:anl:\n\
            2:5482:d=2026030100:UGRD:10 m above ground:anl:\n\
            3:11034:d=2026030100:VGRD:10 m above ground:anl:\n\
            4:16500:d=2026030100:TMP:2 m above ground:anl:\n\
            5:22000:d=2026030100:UGRD:10 m above ground:anl:\n\
        ";
        let ranges = parse_idx_for_10m_wind(idx);
        // Three matches: UGRD@10m, VGRD@10m (open-ended? no — followed
        // by TMP), the second UGRD@10m (followed by nothing in this
        // sample → open-ended).
        assert_eq!(ranges.len(), 3);
        assert_eq!(ranges[0], (5482, Some(11034)));
        assert_eq!(ranges[1], (11034, Some(16500)));
        assert_eq!(ranges[2], (22000, None));
    }
}
