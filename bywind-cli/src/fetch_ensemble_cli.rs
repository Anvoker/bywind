//! `fetch-ensemble` subcommand: pull GEFS ensemble members from
//! NOAA's S3 bucket and write one `.wcav` per member into the output
//! directory.
//!
//! Sibling of [`crate::fetch`]; same date parsing, same interval
//! validation, same per-frame progress reporting. The orchestration
//! loops over members and reuses `transcode_grib2_to_wcav` per
//! member to land each as a self-contained AV1-encoded wind map.

use std::fs::{File, create_dir_all};
use std::io::BufWriter;
use std::path::PathBuf;
use std::time::Instant;

use anyhow::{Context as _, Result, anyhow};
use bywind::fetch::{FetchProgress, FetchSpec, parse_yyyymmddhh, transcode_grib2_to_wcav};
use bywind::fetch_ensemble::{fetch_member_to_grib2, stride_sample_members};
use chrono::{DateTime, Utc};

use crate::error::AppError;

#[derive(clap::Args, Debug)]
pub struct FetchEnsembleArgs {
    /// Window start (UTC, inclusive). Format `YYYYMMDD` (hour defaults
    /// to 00z) or `YYYYMMDDHH`. Must be a GEFS cycle hour
    /// (00, 06, 12, or 18 UTC).
    pub start: String,
    /// Window end (UTC, exclusive). Same format as `<start>`.
    pub end: String,
    /// Output directory. Created if missing. One `.wcav` per member
    /// lands here (`gec00.wcav`, `gep03.wcav`, …).
    #[arg(long, short = 'o')]
    pub out: PathBuf,
    /// Hours between successive frames. Must be one of 1, 2, 3, or 6.
    /// Default 3 — GEFS's `pgrb2sp25` subset publishes at 3-hourly
    /// cadence (f000, f003, ...), not hourly like GFS, so 1 / 2
    /// return 404s on every other frame.
    #[arg(long, value_name = "N", default_value_t = 3)]
    pub interval_h: u32,
    /// Number of ensemble members to pull (stride-sampled from the
    /// 31-member control+perturbed set). Default 10.
    #[arg(long, value_name = "K", default_value_t = 10)]
    pub members: usize,
}

pub fn run(args: &FetchEnsembleArgs) -> Result<(), AppError> {
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

    let members = stride_sample_members(args.members);
    if members.is_empty() {
        return Err(AppError::from(anyhow!(
            "--members must be > 0 (got {})",
            args.members
        )));
    }

    create_dir_all(&args.out).with_context(|| format!("creating {}", args.out.display()))?;

    eprintln!(
        "fetch-ensemble: {} → {} (interval {} h), {} members → {}",
        format_when(start),
        format_when(end),
        args.interval_h,
        members.len(),
        args.out.display(),
    );

    let total_start = Instant::now();
    let mut total_fetched: u32 = 0;
    let mut total_skipped: u32 = 0;
    let mut total_bytes: u64 = 0;
    let mut succeeded_members: usize = 0;

    // Sequential per-member fetch. Within each member the frames are
    // network-bound serial requests anyway, so members-in-parallel
    // would mainly stress the bandwidth without changing wallclock
    // by much. Worth revisiting if S3 stops being the bottleneck.
    for (m_idx, member) in members.iter().enumerate() {
        let prefix = member.filename_prefix();
        eprintln!(
            "\n[{}/{}] member {} — pulling...",
            m_idx + 1,
            members.len(),
            prefix
        );
        let wcav_path = args.out.join(format!("{prefix}.wcav"));
        let staging = wcav_path.with_extension("grib2.tmp");
        let stats = {
            let file = File::create(&staging)
                .with_context(|| format!("creating {}", staging.display()))?;
            let mut writer = BufWriter::new(file);
            match fetch_member_to_grib2(&spec, *member, &mut writer, log_progress) {
                Ok(s) => s,
                Err(e) => {
                    // One member failing shouldn't abort the whole pull;
                    // log and continue.
                    eprintln!("  member {prefix} failed: {e}");
                    drop(std::fs::remove_file(&staging));
                    continue;
                }
            }
        };
        eprintln!(
            "  fetched {} frames ({} skipped, {} KB), encoding wind_av1...",
            stats.fetched,
            stats.skipped,
            stats.total_bytes / 1024,
        );
        let t0 = Instant::now();
        match transcode_grib2_to_wcav(&staging, &wcav_path) {
            Ok(_) => {
                eprintln!(
                    "  encoded in {:.1}s → {}",
                    t0.elapsed().as_secs_f64(),
                    wcav_path.display()
                );
                total_fetched += stats.fetched;
                total_skipped += stats.skipped;
                total_bytes += stats.total_bytes;
                succeeded_members += 1;
            }
            Err(e) => {
                eprintln!("  encode failed: {e}");
            }
        }
        if let Err(e) = std::fs::remove_file(&staging) {
            eprintln!(
                "  note: failed to delete staging {}: {e}",
                staging.display()
            );
        }
    }

    if succeeded_members == 0 {
        return Err(AppError::no_result(anyhow!(
            "no members fetched successfully across {} attempts",
            members.len()
        )));
    }

    eprintln!(
        "\ndone: {succeeded_members}/{} members, {total_fetched} frames, \
         {total_skipped} skipped, {} MB in {:.1}s",
        members.len(),
        total_bytes / (1024 * 1024),
        total_start.elapsed().as_secs_f64(),
    );
    Ok(())
}

fn log_progress(ev: FetchProgress) -> std::ops::ControlFlow<()> {
    match ev {
        FetchProgress::Fetched {
            idx,
            total,
            timestamp,
            bytes,
        } => eprintln!(
            "    [{idx:3}/{total:3}] {} ok ({} KB)",
            format_when(timestamp),
            bytes / 1024,
        ),
        FetchProgress::Skipped {
            idx,
            total,
            timestamp,
            reason,
        } => eprintln!(
            "    [{idx:3}/{total:3}] {} skipped: {reason}",
            format_when(timestamp),
        ),
    }
    std::ops::ControlFlow::Continue(())
}

fn format_when(t: DateTime<Utc>) -> String {
    t.format("%Y-%m-%d %H:%M UTC").to_string()
}
