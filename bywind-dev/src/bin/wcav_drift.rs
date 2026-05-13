//! Per-cell drift comparison between a source GRIB2 and a target
//! `.wcav` — both wind speed (m/s) and wind direction (degrees,
//! angular distance). Speed-bucketed because direction error is
//! dominated by low-speed cells where small `(u, v)` perturbations
//! produce huge angle swings.
//!
//! Usage:
//!   cargo run --release --example wcav_drift -- <source.grib2> <target.wcav>

#![allow(
    clippy::print_stdout,
    clippy::print_stderr,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::uninlined_format_args,
    clippy::needless_range_loop,
    clippy::too_many_lines,
    clippy::doc_markdown,
    clippy::panic_in_result_fn,
    rustdoc::missing_crate_level_docs,
    reason = "throwaway measurement scaffolding"
)]

use std::path::PathBuf;
use std::time::Instant;

/// Smallest signed-angle distance between two compass bearings (degrees).
fn angular_diff_deg(a: f32, b: f32) -> f32 {
    let d = ((a - b + 540.0) % 360.0) - 180.0;
    d.abs()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let source: PathBuf = args
        .next()
        .expect("usage: <source.grib2> <target.wcav>")
        .into();
    let target: PathBuf = args
        .next()
        .expect("usage: <source.grib2> <target.wcav>")
        .into();

    eprintln!("loading {} ...", source.display());
    let t0 = Instant::now();
    let src_map = bywind::io::load(&source, 1, None)?;
    eprintln!(
        "  loaded {} frames in {:.2}s",
        src_map.frame_count(),
        t0.elapsed().as_secs_f64(),
    );

    eprintln!("loading {} ...", target.display());
    let t0 = Instant::now();
    let tgt_map = bywind::io::load(&target, 1, None)?;
    eprintln!(
        "  loaded {} frames in {:.2}s",
        tgt_map.frame_count(),
        t0.elapsed().as_secs_f64(),
    );

    if src_map.frame_count() != tgt_map.frame_count() {
        return Err(format!(
            "frame count mismatch: source {} vs target {}",
            src_map.frame_count(),
            tgt_map.frame_count(),
        )
        .into());
    }

    // Speed buckets — direction drift varies enormously across them.
    // Bucket cell into the SOURCE's speed (so the bucketing isn't
    // affected by the codec's drift).
    let buckets: &[(f32, &str)] = &[
        (0.5, "[0, 0.5)"),
        (1.0, "[0.5, 1.0)"),
        (2.0, "[1.0, 2.0)"),
        (5.0, "[2.0, 5.0)"),
        (10.0, "[5.0, 10.0)"),
        (20.0, "[10.0, 20.0)"),
        (f32::INFINITY, "[20.0, inf)"),
    ];
    // Speeds are stored in knots; convert to m/s for the bucket
    // boundaries above so the labels are in physical units the user
    // actually thinks in.
    const KNOTS_TO_MS: f32 = 1852.0 / 3600.0;

    let mut bucket_counts = vec![0u64; buckets.len()];
    let mut bucket_sum_diff = vec![0.0_f64; buckets.len()];
    let mut bucket_max_diff = vec![0.0_f32; buckets.len()];

    let mut total_dir_sum: f64 = 0.0;
    let mut total_dir_max: f32 = 0.0;
    let mut total_speed_sum: f64 = 0.0;
    let mut total_speed_max: f32 = 0.0;
    let mut n_cells: u64 = 0;

    eprintln!("comparing per-cell ...");
    for f in 0..src_map.frame_count() {
        let src_rows = src_map.frames()[f].rows();
        let tgt_rows = tgt_map.frames()[f].rows();
        assert_eq!(
            src_rows.len(),
            tgt_rows.len(),
            "frame {f} row count mismatch"
        );

        for (src, tgt) in src_rows.iter().zip(tgt_rows.iter()) {
            let speed_diff = (src.sample.speed - tgt.sample.speed).abs();
            let dir_diff = angular_diff_deg(src.sample.direction, tgt.sample.direction);

            total_speed_sum += f64::from(speed_diff);
            if speed_diff > total_speed_max {
                total_speed_max = speed_diff;
            }
            total_dir_sum += f64::from(dir_diff);
            if dir_diff > total_dir_max {
                total_dir_max = dir_diff;
            }

            let speed_ms = src.sample.speed * KNOTS_TO_MS;
            let bucket = buckets.iter().position(|(hi, _)| speed_ms < *hi).unwrap();
            bucket_counts[bucket] += 1;
            bucket_sum_diff[bucket] += f64::from(dir_diff);
            if dir_diff > bucket_max_diff[bucket] {
                bucket_max_diff[bucket] = dir_diff;
            }

            n_cells += 1;
        }
    }

    println!("\nOverall ({} cells)", n_cells);
    println!(
        "  mean |Δspeed|     = {:.4} kt",
        total_speed_sum / n_cells as f64
    );
    println!("  max  |Δspeed|     = {:.4} kt", total_speed_max);
    println!(
        "  mean |Δdirection| = {:.3}°",
        total_dir_sum / n_cells as f64
    );
    println!("  max  |Δdirection| = {:.3}°", total_dir_max);

    println!("\nDirection drift by source-speed bucket (m/s):");
    println!(
        "  {:<14}  {:>12}  {:>10}  {:>10}",
        "bucket", "cells", "mean", "max"
    );
    for (b, (_, label)) in buckets.iter().enumerate() {
        if bucket_counts[b] == 0 {
            continue;
        }
        let mean = bucket_sum_diff[b] / bucket_counts[b] as f64;
        println!(
            "  {:<14}  {:>12}  {:>9.3}°  {:>9.3}°",
            label, bucket_counts[b], mean, bucket_max_diff[b],
        );
    }

    Ok(())
}
