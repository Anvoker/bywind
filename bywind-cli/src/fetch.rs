//! `fetch` subcommand: pull a wind-map window from NOAA's public GFS S3
//! bucket and write it directly to disk.
//!
//! Wraps `bywind::fetch` and adds the user-facing concerns:
//!
//! * Date parsing (`YYYYMMDD` or `YYYYMMDDHH`, hour defaults to 00z).
//! * Progress reporting to stderr.
//! * Output-format dispatch: `.grib2` streams the concatenated GRIB2
//!   messages straight to the requested path; `.wcav` writes the GRIB2
//!   into a sibling `*.grib2.tmp` file, decodes it via `bywind::io::load`,
//!   re-encodes via `bywind::wind_av1::encode`, and deletes the tmp on
//!   success. The user only ever sees the final `.wcav`.

use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::time::Instant;

use anyhow::{Context as _, Result, anyhow};
use bywind::{
    fetch::{FetchProgress, FetchSpec, fetch_to_grib2, parse_yyyymmddhh, transcode_grib2_to_wcav},
    io::Format,
};
use chrono::{DateTime, Utc};

use crate::error::AppError;

#[derive(clap::Args, Debug)]
pub struct FetchArgs {
    /// Window start (UTC, inclusive). Format `YYYYMMDD` (hour defaults
    /// to 00z) or `YYYYMMDDHH`. Must be a GFS cycle hour
    /// (00, 06, 12, or 18 UTC).
    pub start: String,
    /// Window end (UTC, exclusive). Same format as `<start>`.
    pub end: String,
    /// Output path. Format inferred from extension (`.grib2` or
    /// `.wcav`).
    #[arg(long, short = 'o')]
    pub out: PathBuf,
    /// Hours between successive frames. Must be one of 1, 2, 3, or 6.
    /// Default 1 (seamless 1-hour cadence using each cycle's
    /// f000..f005 forecast hours).
    #[arg(long, value_name = "N", default_value_t = 1)]
    pub interval_h: u32,
}

pub fn run(args: &FetchArgs) -> Result<(), AppError> {
    let start = parse_yyyymmddhh(&args.start)
        .map_err(|e| anyhow!(e))
        .context("--start")?;
    let end = parse_yyyymmddhh(&args.end)
        .map_err(|e| anyhow!(e))
        .context("--end")?;
    let spec = FetchSpec {
        start,
        end,
        interval_h: args.interval_h,
    };

    let out_fmt = Format::from_path(&args.out).context("output path")?;

    eprintln!(
        "fetch: {} → {} (interval {} h) → {}",
        format_when(start),
        format_when(end),
        args.interval_h,
        args.out.display(),
    );

    let t0 = Instant::now();
    let stats = match out_fmt {
        Format::Grib2 => fetch_direct(&spec, &args.out)?,
        Format::WindAv1 => fetch_then_encode_wcav(&spec, &args.out)?,
    };
    eprintln!(
        "\ndone: {} frames, {} skipped, {} MB in {:.1}s",
        stats.fetched,
        stats.skipped,
        stats.total_bytes / (1024 * 1024),
        t0.elapsed().as_secs_f64(),
    );
    Ok(())
}

fn fetch_direct(spec: &FetchSpec, out: &Path) -> Result<bywind::fetch::FetchStats> {
    let file = File::create(out).with_context(|| format!("creating {}", out.display()))?;
    let mut writer = BufWriter::new(file);
    let stats = fetch_to_grib2(spec, &mut writer, log_progress)?;
    Ok(stats)
}

fn fetch_then_encode_wcav(spec: &FetchSpec, out: &Path) -> Result<bywind::fetch::FetchStats> {
    let staging = out.with_extension("grib2.tmp");
    let stats = {
        let file =
            File::create(&staging).with_context(|| format!("creating {}", staging.display()))?;
        let mut writer = BufWriter::new(file);
        fetch_to_grib2(spec, &mut writer, log_progress)?
    };

    eprintln!("\nencoding {} as wind_av1...", out.display());
    let t0 = Instant::now();
    transcode_grib2_to_wcav(&staging, out).map_err(anyhow::Error::from)?;
    eprintln!("  encoded in {:.1}s", t0.elapsed().as_secs_f64());

    // Best-effort cleanup: the encode succeeded so the staged GRIB2 is
    // no longer useful, but leaving a stale `.grib2.tmp` doesn't break
    // anything if removal fails (e.g. AV / file lock on Windows).
    if let Err(e) = std::fs::remove_file(&staging) {
        eprintln!(
            "note: failed to delete staging file {}: {e}",
            staging.display()
        );
    }
    Ok(stats)
}

fn log_progress(ev: FetchProgress) -> std::ops::ControlFlow<()> {
    match ev {
        FetchProgress::Fetched {
            idx,
            total,
            timestamp,
            bytes,
        } => eprintln!(
            "[{idx:3}/{total:3}] {} ok ({} KB)",
            format_when(timestamp),
            bytes / 1024,
        ),
        FetchProgress::Skipped {
            idx,
            total,
            timestamp,
            reason,
        } => eprintln!(
            "[{idx:3}/{total:3}] {} skipped: {reason}",
            format_when(timestamp),
        ),
    }
    std::ops::ControlFlow::Continue(())
}

fn format_when(t: DateTime<Utc>) -> String {
    t.format("%Y-%m-%d %H:%M UTC").to_string()
}

impl From<bywind::fetch::FetchError> for AppError {
    fn from(e: bywind::fetch::FetchError) -> Self {
        use bywind::fetch::FetchError;
        match e {
            // Forecast-hour overrun is unreachable under the current
            // `BadInterval` guard; if it ever fires, the spec validator
            // missed a case and that's an internal bug.
            FetchError::ForecastHourOutOfRange { .. } => Self::internal(anyhow!(e)),
            // Valid spec, S3 returned nothing useful — same shape as
            // "search ran but produced no route".
            FetchError::NoFramesFetched { .. } => Self::no_result(anyhow!(e)),
            // Spec / I/O / network problems (and any future
            // `non_exhaustive` variant) are user-actionable; default
            // BadInput class via the `From<anyhow::Error>` impl.
            _ => Self::from(anyhow!(e)),
        }
    }
}
