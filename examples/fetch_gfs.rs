//! Thin shim around `bywind::fetch` kept for the original CLI shape
//! (`fetch_gfs <YYYYMMDD[HH]> <cycles> <out.grib2> [frames-per-cycle]`).
//! The supported flow now lives in the `bywind-cli fetch` subcommand;
//! this example is left in place so old shell snippets keep working
//! against the same code path.
//!
//! Usage:
//!   cargo run --release --example `fetch_gfs` -- <YYYYMMDD[HH]> <cycles> <out.grib2> [frames-per-cycle]
//!
//! Examples:
//!   # 150 cycles × 1 = 150 frames at 6 h cadence (900 h):
//!   cargo run --release --example `fetch_gfs` -- 20260301 150 atlantic.grib2
//!
//!   # 28 cycles × 6 = 168 frames at 1 h cadence (7 days):
//!   cargo run --release --example `fetch_gfs` -- 20260315 28 weekly.grib2 6
//!
//! `frames-per-cycle` is reinterpreted as the cadence in hours:
//!   1 (legacy default) → `interval_h` = 6 (analyses only, 6 h cadence).
//!   6 → `interval_h` = 1 (seamless 1 h cadence).
//!   2, 3 → `interval_h` = 3, 2 (other allowed combinations).
//!   anything else → rejected (the library's allowed set is `{1, 2, 3, 6}`).

#![allow(
    clippy::print_stdout,
    clippy::print_stderr,
    reason = "example binaries use stdout / stderr as their entire UI."
)]

use std::fs::File;
use std::io::BufWriter;

use bywind::fetch::{FetchProgress, FetchSpec, fetch_to_grib2};
use chrono::{NaiveDate, NaiveDateTime, NaiveTime, TimeDelta, TimeZone as _, Utc};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (start_str, cycles_str, out_path, frames_per_cycle) = match args.as_slice() {
        [s, c, o] => (s, c, o, 1u32),
        [s, c, o, f] => (s, c, o, f.parse::<u32>()?),
        _ => {
            eprintln!("usage: fetch_gfs <YYYYMMDD[HH]> <cycles> <out.grib2> [frames-per-cycle]",);
            eprintln!("  start              cycle init time, hour defaults to 00z");
            eprintln!("  cycles             number of 6-hour cycles to fetch");
            eprintln!("  out                output path (existing file is truncated)");
            eprintln!("  frames-per-cycle   1..=6, default 1 (analyses only).");
            eprintln!("                     6 gives seamless 1 h cadence.");
            std::process::exit(2);
        }
    };
    let start = parse_start(start_str)?;
    let cycles: u32 = cycles_str.parse()?;
    if cycles == 0 {
        return Err("cycles must be > 0".into());
    }
    let interval_h = match frames_per_cycle {
        1 => 6,
        2 => 3,
        3 => 2,
        6 => 1,
        other => {
            return Err(format!("frames-per-cycle {other} not supported; use 1, 2, 3, or 6").into());
        }
    };
    let total_hours = i64::from(cycles) * 6;
    let end = start + TimeDelta::hours(total_hours);

    let spec = FetchSpec {
        start,
        end,
        interval_h,
    };
    let mut out = BufWriter::new(File::create(out_path)?);
    let stats = fetch_to_grib2(&spec, &mut out, |ev| {
        match ev {
            FetchProgress::Fetched {
                idx,
                total,
                timestamp,
                bytes,
            } => eprintln!(
                "[{idx:3}/{total:3}] {} ok ({} KB)",
                timestamp.format("%Y-%m-%d %H:%M UTC"),
                bytes / 1024,
            ),
            FetchProgress::Skipped {
                idx,
                total,
                timestamp,
                reason,
            } => eprintln!(
                "[{idx:3}/{total:3}] {} skipped: {reason}",
                timestamp.format("%Y-%m-%d %H:%M UTC"),
            ),
        }
        std::ops::ControlFlow::Continue(())
    })?;
    eprintln!(
        "\ndone: {} frames, {} skipped, {} MB written to {out_path}",
        stats.fetched,
        stats.skipped,
        stats.total_bytes / (1024 * 1024),
    );
    Ok(())
}

fn parse_start(s: &str) -> Result<chrono::DateTime<Utc>, Box<dyn std::error::Error>> {
    let (date_part, hour) = match s.len() {
        8 => (s, 0u32),
        10 => {
            let (d, h) = s.split_at(8);
            (d, h.parse::<u32>()?)
        }
        _ => return Err("date must be YYYYMMDD or YYYYMMDDHH".into()),
    };
    let date = NaiveDate::parse_from_str(date_part, "%Y%m%d")?;
    let time = NaiveTime::from_hms_opt(hour, 0, 0).ok_or("invalid hour")?;
    Ok(Utc.from_utc_datetime(&NaiveDateTime::new(date, time)))
}
