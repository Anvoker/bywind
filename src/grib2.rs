//! GRIB2 → [`TimedWindMap`] loader.
//!
//! Reads UGRD/VGRD pairs at 10 m above ground from a GRIB2 stream, projects
//! the lat/lon grid into Cartesian metres via an equirectangular projection
//! anchored at the bounding-box centre, and assembles one [`WindMap`] frame
//! per absolute UTC time. Each frame's time = its message's reference time
//! (cycle init, from section 1) + the forecast-time offset (from section 4),
//! which lets concatenated multi-cycle files (e.g. one analysis per cycle
//! over 30+ days) produce one frame per cycle rather than collapsing onto
//! `forecast_time = 0`.

use std::collections::{BTreeMap, HashSet};
use std::io::{Read, Seek};

use grib::LatLons as _;
use grib::codetables::Code;
use grib::codetables::grib2::Table4_4;
use grib::def::grib2::RefTime;
use rayon::prelude::*;

use crate::wind_map::{TimedWindMap, WindMap};
use crate::{WeatherRow, WindSample};

/// GRIB2 discipline 0 = Meteorological products.
const DISCIPLINE_METEOROLOGICAL: u8 = 0;
/// GRIB2 Table 4.1, discipline 0, category 2 = Momentum.
const PARAM_CATEGORY_MOMENTUM: u8 = 2;
/// GRIB2 Table 4.2, momentum, parameter 2 = U-component of wind.
const PARAM_NUMBER_UGRD: u8 = 2;
/// GRIB2 Table 4.2, momentum, parameter 3 = V-component of wind.
const PARAM_NUMBER_VGRD: u8 = 3;
/// GRIB2 Table 4.5, surface type 103 = Specified height level above ground.
const SURFACE_HEIGHT_ABOVE_GROUND: u8 = 103;
/// Target altitude (metres). Standard meteorological "surface wind".
const TARGET_HEIGHT_METRES: f64 = 10.0;
/// Tolerance when matching the height-above-ground level (metres). Some
/// producers store 10.0 with a different scale factor that round-trips
/// imperfectly; 0.5 m is well below any plausible alternate level.
const HEIGHT_TOLERANCE_METRES: f64 = 0.5;

/// Inclusive lat/lon rectangle for GRIB2 loading, in degrees.
///
/// Field order is lat-first to match the GRIB2 spec; for the canonical
/// lon-first bbox used by routing / search / map bounds, see
/// [`swarmkit_sailing::spherical::LonLatBbox`] (re-exported as
/// [`crate::LonLatBbox`]).
///
/// Used to filter raw GRIB2 grid points before projection — typical use is
/// restricting a global file to a region of interest (e.g. the North
/// Atlantic) so only that subset reaches the spatial index. Filtering
/// preserves grid uniformity: the result is still a regular sub-grid in
/// lat/lon, hence in the projected plane, hence still takes the fast grid
/// path in `WindMap::new`.
#[derive(Clone, Copy, Debug)]
pub struct Grib2Bbox {
    pub lat_min: f32,
    pub lat_max: f32,
    pub lon_min: f32,
    pub lon_max: f32,
}

impl Grib2Bbox {
    fn contains(self, lat: f32, lon: f32) -> bool {
        lat >= self.lat_min && lat <= self.lat_max && lon >= self.lon_min && lon <= self.lon_max
    }
}

#[derive(Debug)]
#[non_exhaustive]
pub enum LoadError {
    /// Underlying GRIB2 parse / decode failure.
    Grib(grib::GribError),
    /// The file contained no UGRD/VGRD pair we could pair up at any forecast
    /// time. Either the file has no surface wind, or every time step was
    /// missing one component.
    NoFrames,
}

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Grib(e) => write!(f, "GRIB2 error: {e}"),
            Self::NoFrames => write!(
                f,
                "GRIB2 file contained no complete UGRD/VGRD pair at 10 m above ground",
            ),
        }
    }
}

impl std::error::Error for LoadError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Grib(e) => Some(e),
            Self::NoFrames => None,
        }
    }
}

impl From<grib::GribError> for LoadError {
    fn from(e: grib::GribError) -> Self {
        Self::Grib(e)
    }
}

/// Eastward (U) and northward (V) decoded value arrays at a single forecast
/// time. Lat/lons are not stored here — they're identical across every
/// submessage in a single GRIB2 file (single model run, single grid) and
/// are cached once in [`GridCache`].
#[derive(Default)]
struct Pair {
    u: Option<Vec<f32>>,
    v: Option<Vec<f32>>,
}

/// Per-file grid metadata captured once on the first matching submessage and
/// reused for every subsequent frame, avoiding per-frame `latlons()` calls
/// and per-frame `compute_kept_axes` `HashSet` builds — both of which scale
/// `O(grid_size)` and dominate load time on global grids.
struct GridCache {
    latlons: Vec<(f32, f32)>,
    /// `Some` only when `stride > 1`; lazily computed once and shared across
    /// all frames.
    kept_axes: Option<(HashSet<u32>, HashSet<u32>)>,
}

/// Submessage queued for parallel decoding. The decoder owns its compressed
/// section bytes (sect5/6/7) and the grid-point count, so it can be moved
/// across threads without borrowing from the parent `Grib2` reader.
struct PendingSubmessage {
    absolute_t: i64,
    is_u: bool,
    decoder: grib::Grib2SubmessageDecoder,
}

impl TimedWindMap {
    /// Build a [`TimedWindMap`] from a GRIB2 stream.
    ///
    /// Selects UGRD (parameter 0/2/2) and VGRD (parameter 0/2/3) at
    /// 10 m above ground (surface type 103). Forecast times whose unit
    /// the loader doesn't recognise are skipped, as are time steps that
    /// have only one of U/V — both are logged at `warn` level. Returns
    /// [`crate::grib2::LoadError::NoFrames`] if no time step ends up with both
    /// components.
    ///
    /// Lat/lon grid points are projected to (x, y) metres using an
    /// equirectangular projection anchored at the centre of the bounding
    /// box of the first complete frame's grid. Subsequent frames are
    /// assumed to share that grid; this is true for any single GRIB2
    /// file produced by a single model run.
    ///
    /// `stride` decimates the grid by keeping every Nth unique latitude
    /// and Nth unique longitude. `1` (or `0`) keeps every point. Larger
    /// values reduce memory and rendering cost at the cost of fidelity:
    /// e.g. `stride = 4` on a global GFS 0.25° grid (1440×721) yields
    /// 360×181 ≈ 65k points instead of 1M.
    ///
    /// `bbox` restricts ingest to a lat/lon rectangle. Points outside
    /// the box are dropped before projection — useful for keeping only
    /// the region you'll actually route through when the source file
    /// (e.g. an AWS-archived global GFS) covers the whole planet.
    /// `None` keeps every point.
    ///
    /// # Errors
    /// Returns [`LoadError::Grib`] if the underlying GRIB2 parser fails,
    /// or [`crate::grib2::LoadError::NoFrames`] if the file contains no UGRD/VGRD pair
    /// that can be matched at any forecast time.
    pub fn from_grib2_reader<R: Read + Seek>(
        reader: R,
        stride: usize,
        bbox: Option<Grib2Bbox>,
    ) -> Result<Self, LoadError> {
        let stride = stride.max(1);
        let total_start = std::time::Instant::now();
        let parse_start = std::time::Instant::now();
        let grib2 = grib::from_reader(reader)?;
        let parse_elapsed = parse_start.elapsed();

        // Phase 1 (serial): walk the submessage stream, filter to UGRD/VGRD
        // at 10 m, and build owned decoders for the matches. Decoders own
        // their compressed section bytes, so once Phase 1 ends we have no
        // further dependency on the parent reader and can fan out.
        // The lat/lon grid (and its bbox/stride-filtered axis sets) is the
        // same for every submessage in a single model run — capture it on
        // the first match and reuse for all subsequent frames.
        let phase1_start = std::time::Instant::now();
        let mut pending: Vec<PendingSubmessage> = Vec::new();
        let mut cache: Option<GridCache> = None;

        for (_index, sub) in &grib2 {
            if sub.indicator().discipline != DISCIPLINE_METEOROLOGICAL {
                continue;
            }
            let pd = sub.prod_def();
            let Some(category) = pd.parameter_category() else {
                continue;
            };
            let Some(number) = pd.parameter_number() else {
                continue;
            };
            if category != PARAM_CATEGORY_MOMENTUM {
                continue;
            }
            let is_u = number == PARAM_NUMBER_UGRD;
            let is_v = number == PARAM_NUMBER_VGRD;
            if !is_u && !is_v {
                continue;
            }

            let Some((first_surface, _second_surface)) = pd.fixed_surfaces() else {
                continue;
            };
            if first_surface.surface_type != SURFACE_HEIGHT_ABOVE_GROUND {
                continue;
            }
            if (first_surface.value() - TARGET_HEIGHT_METRES).abs() > HEIGHT_TOLERANCE_METRES {
                continue;
            }

            let Some(forecast_time) = pd.forecast_time() else {
                continue;
            };
            let Some(forecast_seconds) =
                forecast_unit_to_seconds(&forecast_time.unit, forecast_time.value)
            else {
                log::warn!(
                    "GRIB2: skipping submessage with unsupported forecast-time unit {:?}",
                    forecast_time.unit,
                );
                continue;
            };
            let ref_time = sub.identification().ref_time_unchecked();
            let Some(ref_seconds) = ref_time_to_unix_seconds(&ref_time) else {
                log::warn!("GRIB2: skipping submessage with invalid reference time {ref_time:?}");
                continue;
            };
            let absolute_t = ref_seconds.saturating_add(i64::from(forecast_seconds));

            if cache.is_none() {
                let latlons: Vec<(f32, f32)> = match sub.latlons() {
                    Ok(iter) => iter.collect(),
                    Err(e) => {
                        log::warn!("GRIB2: skipping submessage at t={absolute_t}s ({e})");
                        continue;
                    }
                };
                let kept_axes = (stride > 1).then(|| compute_kept_axes(&latlons, stride, bbox));
                cache = Some(GridCache { latlons, kept_axes });
            }

            let decoder = match grib::Grib2SubmessageDecoder::from(sub) {
                Ok(d) => d,
                Err(e) => {
                    log::warn!("GRIB2: skipping submessage at t={absolute_t}s ({e})");
                    continue;
                }
            };

            pending.push(PendingSubmessage {
                absolute_t,
                is_u,
                decoder,
            });
        }

        let phase1_elapsed = phase1_start.elapsed();
        let pending_count = pending.len();

        let Some(cache) = cache else {
            return Err(LoadError::NoFrames);
        };

        // Phase 2 (parallel): decompress every queued submessage's value
        // array on the rayon thread pool. `dispatch().collect()` is the
        // dominant cost on large grids — for a 1.2 GB file with hundreds
        // of frames this is where the wall-clock savings come from.
        let phase2_start = std::time::Instant::now();
        let decoded = decode_pending_parallel(pending, cache.latlons.len());
        let phase2_elapsed = phase2_start.elapsed();
        let decoded_count = decoded.len();

        // Phase 3 (parallel): bucket decoded U/V arrays by absolute time
        // and assemble frames. The grid cache short-circuits per-frame
        // `compute_kept_axes` and per-row HashSet construction.
        let phase3_start = std::time::Instant::now();
        let (times, frames, build_rows_total, wind_map_new_total) =
            assemble_frames_parallel(decoded, &cache, bbox);
        let phase3_elapsed = phase3_start.elapsed();

        if frames.is_empty() {
            return Err(LoadError::NoFrames);
        }

        let step_seconds = compute_median_step_seconds(&times);
        let mut map = Self::new(frames, step_seconds);
        // The GRIB2 messages carry their reference time + forecast
        // offsets per submessage, summed into `absolute_t` above. We
        // surface that as the dataset's UTC range so downstream tools
        // (CLI `info`, GUI Time section, `.wcav` v2 header) can show
        // "what real-world period does this file cover".
        if let (Some(&min_unix), Some(&max_unix)) = (times.first(), times.last())
            && let (Some(start), Some(end)) = (
                chrono::DateTime::<chrono::Utc>::from_timestamp(min_unix, 0),
                chrono::DateTime::<chrono::Utc>::from_timestamp(max_unix, 0),
            )
        {
            map = map.with_time_range(start, end);
        }

        log_grib2_load_summary(&LoadSummary {
            total_elapsed: total_start.elapsed(),
            parse_elapsed,
            phase1_elapsed,
            phase2_elapsed,
            phase3_elapsed,
            grid_size: cache.latlons.len(),
            frame_count: map.frame_count(),
            pending_count,
            decoded_count,
            build_rows_total,
            wind_map_new_total,
        });

        Ok(map)
    }
}

/// Per-load timing / counter bundle for [`log_grib2_load_summary`].
/// One field per metric the summary line surfaces.
struct LoadSummary {
    total_elapsed: std::time::Duration,
    parse_elapsed: std::time::Duration,
    phase1_elapsed: std::time::Duration,
    phase2_elapsed: std::time::Duration,
    phase3_elapsed: std::time::Duration,
    grid_size: usize,
    frame_count: usize,
    pending_count: usize,
    decoded_count: usize,
    build_rows_total: std::time::Duration,
    wind_map_new_total: std::time::Duration,
}

/// Phase 2: parallel-decompress every pending submessage on the rayon
/// pool. Returns `(absolute_t, is_u, values)` triples — caller buckets
/// them by time and pairs U with V. Submessages whose decode fails, or
/// whose value count disagrees with the shared grid, are logged and
/// dropped; the load proceeds with whatever survives.
fn decode_pending_parallel(
    pending: Vec<PendingSubmessage>,
    expected_len: usize,
) -> Vec<(i64, bool, Vec<f32>)> {
    pending
        .into_par_iter()
        .filter_map(|p| {
            let values: Vec<f32> = match p.decoder.dispatch() {
                Ok(iter) => iter.collect(),
                Err(e) => {
                    log::warn!(
                        "GRIB2: skipping submessage at t={}s ({:?})",
                        p.absolute_t,
                        e,
                    );
                    return None;
                }
            };
            if values.len() != expected_len {
                log::warn!(
                    "GRIB2: values ({}) / grid ({}) length mismatch at t={}s, skipping",
                    values.len(),
                    expected_len,
                    p.absolute_t,
                );
                return None;
            }
            Some((p.absolute_t, p.is_u, values))
        })
        .collect()
}

/// Phase 3: bucket the decoded `(t, is_u, values)` triples into U/V
/// pairs by time, then parallel-build one `WindMap` per complete pair.
/// `BTreeMap` iteration is ascending-by-time, and `rayon::collect()`
/// preserves input order, so the output frames are sorted by absolute
/// time. The returned `build_rows_total` / `wind_map_new_total` are
/// CPU-time sums across rayon workers (not wall-clock); their ratio
/// against the caller's phase-3 wall-clock duration indicates parallel
/// utilisation.
fn assemble_frames_parallel(
    decoded: Vec<(i64, bool, Vec<f32>)>,
    cache: &GridCache,
    bbox: Option<Grib2Bbox>,
) -> (
    Vec<i64>,
    Vec<WindMap>,
    std::time::Duration,
    std::time::Duration,
) {
    use std::sync::atomic::{AtomicU64, Ordering};

    let mut by_time: BTreeMap<i64, Pair> = BTreeMap::new();
    for (absolute_t, is_u, values) in decoded {
        let entry = by_time.entry(absolute_t).or_default();
        if is_u {
            entry.u = Some(values);
        } else {
            entry.v = Some(values);
        }
    }

    let by_time_vec: Vec<(i64, Pair)> = by_time.into_iter().collect();
    let build_rows_total = AtomicU64::new(0);
    let wind_map_new_total = AtomicU64::new(0);
    let frames_and_times: Vec<(i64, WindMap)> = by_time_vec
        .into_par_iter()
        .filter_map(|(absolute_t, pair)| {
            let (Some(u), Some(v)) = (pair.u, pair.v) else {
                log::warn!("GRIB2: incomplete UGRD/VGRD pair at t={absolute_t}s, skipping",);
                return None;
            };
            let br_start = std::time::Instant::now();
            let rows = build_rows(&u, &v, &cache.latlons, bbox, cache.kept_axes.as_ref());
            build_rows_total.fetch_add(br_start.elapsed().as_nanos() as u64, Ordering::Relaxed);
            if rows.is_empty() {
                log::warn!(
                    "GRIB2: no points survived bbox/stride filter at t={absolute_t}s, skipping",
                );
                return None;
            }
            let wm_start = std::time::Instant::now();
            let map = WindMap::new(rows);
            wind_map_new_total.fetch_add(wm_start.elapsed().as_nanos() as u64, Ordering::Relaxed);
            Some((absolute_t, map))
        })
        .collect();
    let (times, frames): (Vec<i64>, Vec<WindMap>) = frames_and_times.into_iter().unzip();
    let build_rows_total =
        std::time::Duration::from_nanos(build_rows_total.load(Ordering::Relaxed));
    let wind_map_new_total =
        std::time::Duration::from_nanos(wind_map_new_total.load(Ordering::Relaxed));
    (times, frames, build_rows_total, wind_map_new_total)
}

/// Median inter-frame gap (seconds) of `times`, or `1.0` when there's
/// only one frame. The median rather than the first gap so a single
/// missing cycle (e.g. a corrupt file in the middle of a long download)
/// doesn't dilate the whole timeline.
fn compute_median_step_seconds(times: &[i64]) -> f32 {
    if times.len() < 2 {
        // Single frame: the value is unused (no interpolation possible)
        // but TimedWindMap::new requires step_seconds > 0.
        return 1.0;
    }
    let mut gaps: Vec<i64> = times
        .windows(2)
        .map(|w| w[1] - w[0])
        .filter(|&g| g > 0)
        .collect();
    gaps.sort_unstable();
    let median = gaps.get(gaps.len() / 2).copied().unwrap_or(1);
    median as f32
}

/// Emit the GRIB2 load summary to both `log::info!` and a best-effort
/// temp-dir log file. Two sinks because `bywind-viz` on Windows runs
/// as `windows_subsystem = "windows"` in release, which detaches
/// stderr — so `log::info!` would be invisible exactly when the user
/// wants to diagnose a slow release-mode load. The file always works;
/// `log::info!` is kept for terminal users (`RUST_LOG=bywind=info`)
/// and headless examples.
fn log_grib2_load_summary(s: &LoadSummary) {
    let summary = format!(
        "GRIB2 load: total={:.2}s (parse={:.2}s, phase1={:.2}s, phase2={:.2}s, phase3={:.2}s) \
         grid={} frames={} pending={} decoded={} \
         build_rows_cpu_sum={:.2}s wind_map_new_cpu_sum={:.2}s\n",
        s.total_elapsed.as_secs_f64(),
        s.parse_elapsed.as_secs_f64(),
        s.phase1_elapsed.as_secs_f64(),
        s.phase2_elapsed.as_secs_f64(),
        s.phase3_elapsed.as_secs_f64(),
        s.grid_size,
        s.frame_count,
        s.pending_count,
        s.decoded_count,
        s.build_rows_total.as_secs_f64(),
        s.wind_map_new_total.as_secs_f64(),
    );
    log::info!("{}", summary.trim_end());
    let log_path = std::env::temp_dir().join("bywind-grib2-load.log");
    if let Ok(mut f) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&log_path)
    {
        use std::io::Write as _;
        // Best-effort debug log; ignore write failures.
        drop(f.write_all(summary.as_bytes()));
    }
}

/// Convert a GRIB2 section-1 reference time to seconds since the Unix
/// epoch (1970-01-01T00:00:00 UTC), proleptic Gregorian. Uses Howard
/// Hinnant's `days_from_civil` algorithm so we don't pull in `chrono`
/// just for the date arithmetic.
///
/// Returns `None` for syntactically invalid times (the GRIB API marks
/// the value as "unchecked", and missing fields show up as 0xFF).
fn ref_time_to_unix_seconds(rt: &RefTime) -> Option<i64> {
    let days = days_from_civil(i32::from(rt.year), rt.month, rt.day)?;
    if rt.hour > 23 || rt.minute > 59 || rt.second > 60 {
        return None;
    }
    Some(
        days * 86_400
            + i64::from(rt.hour) * 3_600
            + i64::from(rt.minute) * 60
            + i64::from(rt.second),
    )
}

/// Days from 1970-01-01 to (y, m, d) in the proleptic Gregorian calendar,
/// after Howard Hinnant ("chrono-Compatible Low-Level Date Algorithms").
/// Returns `None` for `m` outside `1..=12` or `d` outside `1..=31`; out-of-
/// range combinations within those bounds (e.g. February 30) round-trip
/// to a nearby valid date, which is acceptable for this use case where
/// invalid GRIB metadata should mostly be skipped upstream anyway.
fn days_from_civil(y: i32, m: u8, d: u8) -> Option<i64> {
    if !(1..=12).contains(&m) || !(1..=31).contains(&d) {
        return None;
    }
    let y = if m <= 2 { y - 1 } else { y };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = (y - era * 400) as i64; // 0..=399
    let m = i64::from(m);
    let d = i64::from(d);
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + d - 1; // 0..=365
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    Some(i64::from(era) * 146_097 + doe - 719_468)
}

/// Convert a GRIB2 Table 4.4 time-unit + value into seconds. Returns `None`
/// for units we don't translate (months/years/decades/centuries are not
/// fixed-length so are ambiguous; "Missing" is meaningless here).
fn forecast_unit_to_seconds(unit: &Code<Table4_4, u8>, value: u32) -> Option<u32> {
    let Code::Name(unit) = unit else { return None };
    let multiplier: u32 = match unit {
        Table4_4::Second => 1,
        Table4_4::Minute => 60,
        Table4_4::Hour => 3600,
        Table4_4::ThreeHours => 3 * 3600,
        Table4_4::SixHours => 6 * 3600,
        Table4_4::TwelveHours => 12 * 3600,
        Table4_4::Day => 86_400,
        Table4_4::Month
        | Table4_4::Year
        | Table4_4::Decade
        | Table4_4::Normal
        | Table4_4::Century
        | Table4_4::Missing => return None,
    };
    value.checked_mul(multiplier)
}

/// Pair the U and V samples elementwise (they share grid order with
/// `latlons`) and emit [`WeatherRow`]s with `(lon, lat)` coordinates.
///
/// `kept_axes`, when `Some`, is a precomputed `(kept_lat_bits,
/// kept_lon_bits)` filter from [`compute_kept_axes`] — passing it in
/// (rather than recomputing per call) is what lets the GRIB2 loader avoid
/// rebuilding `HashSets` for every frame in a multi-frame file.
fn build_rows(
    u_values: &[f32],
    v_values: &[f32],
    latlons: &[(f32, f32)],
    bbox: Option<Grib2Bbox>,
    kept_axes: Option<&(HashSet<u32>, HashSet<u32>)>,
) -> Vec<WeatherRow> {
    let mut rows = Vec::with_capacity(latlons.len());
    for i in 0..latlons.len() {
        let (lat, lon) = latlons[i];
        if let Some(b) = bbox
            && !b.contains(lat, lon)
        {
            continue;
        }
        if let Some((kept_lats, kept_lons)) = kept_axes
            && (!kept_lats.contains(&lat.to_bits()) || !kept_lons.contains(&lon.to_bits()))
        {
            continue;
        }
        let uc = u_values[i];
        let vc = v_values[i];
        if !uc.is_finite() || !vc.is_finite() {
            // GRIB2 missing-value bitmap encodes gaps as NaN. Skip them so
            // WindMap's spatial index doesn't see degenerate samples; IDW
            // will fall back to the surrounding grid cells when queried.
            continue;
        }
        let speed = uc.hypot(vc);
        // Meteorological "from" direction: degrees clockwise from north,
        // i.e. the bearing the wind is coming FROM. Frame-independent —
        // matches `WindSample::direction`.
        let direction = (270.0 - vc.atan2(uc).to_degrees()).rem_euclid(360.0);
        rows.push(WeatherRow {
            lon,
            lat,
            sample: WindSample { speed, direction },
        });
    }
    rows
}

/// Pick every Nth unique latitude and every Nth unique longitude (sorted
/// ascending) and return their f32 bit-patterns as `HashSet`s for O(1)
/// membership tests during the per-row scan. Keys are bit-patterns rather
/// than `f32` so we can hash them; lat/lon values come unmodified from the
/// GRIB2 reader so equality is byte-exact.
///
/// The unique-axis collection is done over the bbox-filtered point set so
/// stride counts apply to the *kept* region, not the original global grid.
/// E.g. stride 2 on a 60°-wide bbox keeps 121 of 241 unique lons rather
/// than 720 of 1440.
fn compute_kept_axes(
    latlons: &[(f32, f32)],
    stride: usize,
    bbox: Option<Grib2Bbox>,
) -> (HashSet<u32>, HashSet<u32>) {
    let cmp = |a: &f32, b: &f32| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal);
    let in_bbox = |lat: f32, lon: f32| -> bool { bbox.is_none_or(|b| b.contains(lat, lon)) };
    let mut unique_lats: Vec<f32> = latlons
        .iter()
        .filter(|(lat, lon)| in_bbox(*lat, *lon))
        .map(|p| p.0)
        .collect();
    unique_lats.sort_by(cmp);
    unique_lats.dedup();
    let mut unique_lons: Vec<f32> = latlons
        .iter()
        .filter(|(lat, lon)| in_bbox(*lat, *lon))
        .map(|p| p.1)
        .collect();
    unique_lons.sort_by(cmp);
    unique_lons.dedup();
    let kept_lats = unique_lats
        .into_iter()
        .step_by(stride)
        .map(f32::to_bits)
        .collect();
    let kept_lons = unique_lons
        .into_iter()
        .step_by(stride)
        .map(f32::to_bits)
        .collect();
    (kept_lats, kept_lons)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forecast_unit_to_seconds_handles_common_units() {
        let hour = Code::Name(Table4_4::Hour);
        assert_eq!(forecast_unit_to_seconds(&hour, 0), Some(0));
        assert_eq!(forecast_unit_to_seconds(&hour, 6), Some(21_600));
        let three_hours = Code::Name(Table4_4::ThreeHours);
        assert_eq!(forecast_unit_to_seconds(&three_hours, 4), Some(43_200));
        let day = Code::Name(Table4_4::Day);
        assert_eq!(forecast_unit_to_seconds(&day, 2), Some(172_800));
    }

    #[test]
    fn forecast_unit_to_seconds_rejects_calendar_units() {
        let month = Code::Name(Table4_4::Month);
        assert_eq!(forecast_unit_to_seconds(&month, 1), None);
        let missing = Code::Name(Table4_4::Missing);
        assert_eq!(forecast_unit_to_seconds(&missing, 1), None);
        let unknown: Code<Table4_4, u8> = Code::Num(99);
        assert_eq!(forecast_unit_to_seconds(&unknown, 1), None);
    }

    /// Helper: build the kept-axes filter that production callers cache once
    /// across frames. Returns `None` when stride is ≤ 1, matching the loader.
    fn kept(
        latlons: &[(f32, f32)],
        stride: usize,
        bbox: Option<Grib2Bbox>,
    ) -> Option<(HashSet<u32>, HashSet<u32>)> {
        (stride > 1).then(|| compute_kept_axes(latlons, stride, bbox))
    }

    #[test]
    fn build_rows_bbox_drops_points_outside_rectangle() {
        // 3×3 grid; bbox keeps the central column and the upper two rows
        // → 2×1 = 2 points kept (rows at lat=1 and lat=2, lon=1).
        let mut latlons = Vec::new();
        for j in 0..3 {
            for i in 0..3 {
                latlons.push((j as f32, i as f32));
            }
        }
        let values = vec![1.0_f32; 9];
        let bbox = Grib2Bbox {
            lat_min: 1.0,
            lat_max: 2.0,
            lon_min: 1.0,
            lon_max: 1.0,
        };
        let rows = build_rows(&values, &values, &latlons, Some(bbox), None);
        assert_eq!(rows.len(), 2);
    }

    #[test]
    fn build_rows_bbox_and_stride_compose() {
        // 5×5 grid lat ∈ {0..4}, lon ∈ {0..4}. Bbox keeps lat ∈ [1, 3] and
        // lon ∈ [1, 3] → 3×3. Stride 2 over that subset keeps every 2nd
        // unique value: lat ∈ {1, 3}, lon ∈ {1, 3} → 2×2 = 4 points.
        let mut latlons = Vec::new();
        for j in 0..5 {
            for i in 0..5 {
                latlons.push((j as f32, i as f32));
            }
        }
        let values = vec![1.0_f32; 25];
        let bbox = Grib2Bbox {
            lat_min: 1.0,
            lat_max: 3.0,
            lon_min: 1.0,
            lon_max: 3.0,
        };
        let kept_axes = kept(&latlons, 2, Some(bbox));
        let rows = build_rows(&values, &values, &latlons, Some(bbox), kept_axes.as_ref());
        assert_eq!(rows.len(), 4);
    }

    #[test]
    fn build_rows_round_trips_north_wind() {
        // Wind blowing FROM the north (i.e. southward). u=0, v=-10 means the
        // wind vector points south, which is "north wind" in met convention.
        // Direction encoding is frame-independent (compass from-bearing) so
        // it survives the projection-removal change unchanged.
        let latlons = vec![(45.0, 0.0)];
        let u = vec![0.0_f32];
        let v = vec![-10.0_f32];
        let rows = build_rows(&u, &v, &latlons, None, None);
        assert_eq!(rows.len(), 1);
        assert!((rows[0].sample.speed - 10.0).abs() < 1e-4);
        assert!((rows[0].sample.direction - 0.0).abs() < 1e-3);
    }

    #[test]
    fn build_rows_round_trips_east_wind() {
        // Wind blowing FROM the east. u=-10, v=0 means vector points west.
        let latlons = vec![(45.0, 0.0)];
        let u = vec![-10.0_f32];
        let v = vec![0.0_f32];
        let rows = build_rows(&u, &v, &latlons, None, None);
        assert!((rows[0].sample.direction - 90.0).abs() < 1e-3);
    }

    #[test]
    fn build_rows_skips_nan_samples() {
        let latlons = vec![(45.0, 0.0), (45.5, 0.0)];
        let u = vec![f32::NAN, 5.0];
        let v = vec![0.0, 0.0];
        let rows = build_rows(&u, &v, &latlons, None, None);
        assert_eq!(rows.len(), 1);
    }

    #[test]
    fn build_rows_emits_lon_lat_directly() {
        // Coordinates flow through verbatim: latlons input is `(lat, lon)`,
        // the row stores them as `(lon, lat)`.
        let latlons = vec![(45.0_f32, -10.0_f32), (60.0, 5.0)];
        let u = vec![0.0_f32; 2];
        let v = vec![0.0_f32; 2];
        let rows = build_rows(&u, &v, &latlons, None, None);
        assert_eq!(rows.len(), 2);
        assert!((rows[0].lon - (-10.0)).abs() < 1e-6);
        assert!((rows[0].lat - 45.0).abs() < 1e-6);
        assert!((rows[1].lon - 5.0).abs() < 1e-6);
        assert!((rows[1].lat - 60.0).abs() < 1e-6);
    }

    #[test]
    fn build_rows_stride_keeps_every_nth_unique_axis_value() {
        // 4×4 grid with lat ∈ {0,1,2,3}, lon ∈ {10,11,12,13}. Stride 2 must
        // keep lat ∈ {0,2} and lon ∈ {10,12} → 2×2 = 4 rows.
        let mut latlons = Vec::new();
        for j in 0..4 {
            for i in 0..4 {
                latlons.push((j as f32, 10.0 + i as f32));
            }
        }
        let values = vec![1.0_f32; 16];
        let kept_axes = kept(&latlons, 2, None);
        let rows = build_rows(&values, &values, &latlons, None, kept_axes.as_ref());
        assert_eq!(rows.len(), 4);
        let mut lats: Vec<f32> = rows.iter().map(|r| r.lat).collect();
        let mut lons: Vec<f32> = rows.iter().map(|r| r.lon).collect();
        lats.sort_by(|a, b| a.partial_cmp(b).unwrap());
        lats.dedup();
        lons.sort_by(|a, b| a.partial_cmp(b).unwrap());
        lons.dedup();
        assert_eq!(lats, vec![0.0, 2.0]);
        assert_eq!(lons, vec![10.0, 12.0]);
    }

    #[test]
    fn build_rows_stride_one_is_no_op() {
        let latlons: Vec<(f32, f32)> = (0..9).map(|k| (k as f32, k as f32)).collect();
        let values = vec![1.0_f32; 9];
        let no_stride = build_rows(&values, &values, &latlons, None, None);
        let stride_zero = build_rows(&values, &values, &latlons, None, None);
        assert_eq!(no_stride.len(), 9);
        assert_eq!(stride_zero.len(), 9);
    }

    #[test]
    fn days_from_civil_matches_known_anchors() {
        // Unix epoch by definition.
        assert_eq!(days_from_civil(1970, 1, 1), Some(0));
        // Y2K: 1970-01-01 + 30 years and 1 day across the leap-year cycle.
        // Independently: 30 years * 365 + 8 leap days (72,76,80,84,88,92,96, plus 2000 is leap but not yet) = 10957.
        assert_eq!(days_from_civil(2000, 1, 1), Some(10_957));
        // 2000-12-31: 2000-01-01 + 365 days (2000 is a leap year, but Dec 31 is day 366 from Jan 1, i.e. +365).
        assert_eq!(days_from_civil(2000, 12, 31), Some(10_957 + 365));
        // Pre-epoch.
        assert_eq!(days_from_civil(1969, 12, 31), Some(-1));
        assert_eq!(days_from_civil(1969, 1, 1), Some(-365));
    }

    #[test]
    fn ref_time_to_unix_seconds_combines_date_and_time() {
        let rt = RefTime::new(1970, 1, 1, 0, 0, 0);
        assert_eq!(ref_time_to_unix_seconds(&rt), Some(0));
        let rt = RefTime::new(1970, 1, 1, 1, 30, 15);
        assert_eq!(ref_time_to_unix_seconds(&rt), Some(3600 + 30 * 60 + 15));
        // 24-hour shift = 86400 s.
        let rt = RefTime::new(1970, 1, 2, 0, 0, 0);
        assert_eq!(ref_time_to_unix_seconds(&rt), Some(86_400));
        // Two cycles 6h apart on the same day.
        let a = ref_time_to_unix_seconds(&RefTime::new(2026, 3, 1, 0, 0, 0)).unwrap();
        let b = ref_time_to_unix_seconds(&RefTime::new(2026, 3, 1, 6, 0, 0)).unwrap();
        assert_eq!(b - a, 21_600);
    }

    #[test]
    fn ref_time_to_unix_seconds_rejects_invalid_clock() {
        let bad_hour = RefTime::new(2026, 3, 1, 25, 0, 0);
        assert_eq!(ref_time_to_unix_seconds(&bad_hour), None);
        let bad_minute = RefTime::new(2026, 3, 1, 0, 60, 0);
        assert_eq!(ref_time_to_unix_seconds(&bad_minute), None);
        let bad_month = RefTime::new(2026, 13, 1, 0, 0, 0);
        assert_eq!(ref_time_to_unix_seconds(&bad_month), None);
    }
}
