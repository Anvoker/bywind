//! `info` subcommand: print metadata about a wind map.
//!
//! Loads the map (any supported format, auto-detected by extension), then
//! prints format / frame count / step seconds / total duration / bbox /
//! sample count to stderr. Useful when you need a quick "what's actually
//! in this file?" before deciding to feed it to `search` or `convert`.

use std::path::Path;
use std::time::Instant;

use anyhow::{Context as _, anyhow};
use bywind::{MapBounds, fmt::format_duration_breakdown, io::Format};

use crate::error::AppError;

pub fn run(map_path: &Path) -> Result<(), AppError> {
    let fmt = Format::from_path(map_path)
        .with_context(|| format!("inferring format of {}", map_path.display()))?;

    eprintln!("loading {}...", map_path.display());
    let load_start = Instant::now();
    let map = bywind::io::load(map_path, 1, None)
        .with_context(|| format!("loading wind map from {}", map_path.display()))?;
    let load_dur = load_start.elapsed();

    let frame_count = map.frame_count();
    let step_seconds = f64::from(map.step_seconds());
    // Duration is from frame 0 to the start of frame N — `frame_count` steps
    // between frame 0 and frame N=frame_count, but the conventional "loaded
    // duration" includes both endpoints, so spans `(frame_count - 1)` steps.
    let total_seconds = step_seconds * (frame_count.saturating_sub(1) as f64);

    let bounds = MapBounds::from_wind_map(&map).ok_or_else(|| {
        AppError::no_result(anyhow!("wind map {} has no rows", map_path.display()))
    })?;

    let samples_per_frame = map.frame(0).map(|f| f.rows().len()).unwrap_or(0);

    eprintln!();
    eprintln!("=== Wind map ===");
    eprintln!("Path:           {}", map_path.display());
    eprintln!("Format:         {} ({})", fmt.name(), describe_format(fmt));
    eprintln!("Loaded in:      {:.2}s", load_dur.as_secs_f64());
    eprintln!("Frames:         {frame_count}");
    eprintln!("Step:           {step_seconds} s");
    eprintln!(
        "Total duration: {} ({} frames × {} s)",
        format_duration_breakdown(total_seconds),
        frame_count,
        step_seconds,
    );
    let bbox = bounds.bbox;
    eprintln!(
        "Bounding box:   lon {:.3}..{:.3}  lat {:.3}..{:.3}",
        bbox.lon_min, bbox.lon_max, bbox.lat_min, bbox.lat_max,
    );
    if bounds.lon_wraps() {
        eprintln!("                (wraps the antimeridian: lon_min > lon_max)");
    }
    // Real-world UTC range when the dataset carries it. Sources:
    // GRIB2 (read from message reference time + forecast offsets),
    // `wind_av1` v2 files (read from the header's two `i64` slots).
    // Synthetic generators and v1 `wind_av1` files report "unknown".
    match map.time_range() {
        Some((start, end)) => eprintln!(
            "Time range:     {} → {}",
            start.format("%Y-%m-%d %H:%M UTC"),
            end.format("%Y-%m-%d %H:%M UTC"),
        ),
        None => eprintln!("Time range:     unknown"),
    }
    eprintln!("Samples / frame: {samples_per_frame}");

    Ok(())
}

/// Short prose label for the format's defining feature.
fn describe_format(fmt: Format) -> &'static str {
    match fmt {
        Format::Grib2 => "WMO GRIB2, parallel decode via the `grib` crate",
        Format::WindAv1 => "bywind::wind_av1, AV1 near-lossless (header + IVF)",
    }
}
