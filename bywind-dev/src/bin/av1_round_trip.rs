//! End-to-end verification for the `wind_av1` Rust pipeline.
//!
//! Loads a GRIB2 source, encodes via `bywind::wind_av1::encode`,
//! decodes via `bywind::wind_av1::decode`, and reports drift between
//! source samples and the decoded samples cell-by-cell. Used to
//! verify that the production codec matches the POC's drift figures
//! and to spot regressions after rav1d / format changes.
//!
//! Usage:
//!   `cargo run --release --example av1_round_trip -- <grib2> [<frames>]`
//!
//! Frames defaults to "all". Writes the intermediate `.wcav` to a
//! temp file alongside the source so manual inspection is easy.

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

use std::fs::{self, File};
use std::io::{BufReader, BufWriter};
use std::path::PathBuf;
use std::time::Instant;

use bywind::wind_av1::{self, EncodeParams};

fn sample_to_uv(s: &bywind::WindSample) -> (f32, f32) {
    let theta = (270.0 - s.direction).to_radians();
    let speed = if s.speed.is_finite() { s.speed } else { 0.0 };
    (speed * theta.cos(), speed * theta.sin())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let grib: PathBuf = args
        .next()
        .expect("usage: <grib2> [<frames>] [<quantizer>]")
        .into();
    let frames_cap: Option<usize> = args.next().and_then(|s| s.parse().ok());
    let quantizer: u8 = args.next().and_then(|s| s.parse().ok()).unwrap_or(80);
    eprintln!("rav1e quantizer = {quantizer}");

    eprintln!("loading {} ...", grib.display());
    let t0 = Instant::now();
    let mut map = bywind::io::load(&grib, 1, None)?;
    eprintln!(
        "  loaded {} frames in {:.2}s",
        map.frame_count(),
        t0.elapsed().as_secs_f64()
    );

    if let Some(cap) = frames_cap
        && cap < map.frame_count()
    {
        map = bywind::TimedWindMap::new(
            map.frames().iter().take(cap).cloned().collect(),
            map.step_seconds(),
        );
        eprintln!("  truncated to {} frames", map.frame_count());
    }

    let layout = map.frames()[0]
        .grid_layout()
        .ok_or("source frame is not on a regular grid")?;
    let nx = layout.nx;
    let ny = layout.ny;
    let n_frames = map.frame_count();
    eprintln!("  grid: {nx} × {ny}");

    let wcav_path = grib.with_extension("av1roundtrip.wcav");
    eprintln!("encoding to {} ...", wcav_path.display());
    let t0 = Instant::now();
    {
        let writer = BufWriter::new(File::create(&wcav_path)?);
        wind_av1::encode(
            &map,
            writer,
            EncodeParams {
                quantizer,
                speed_preset: 6,
            },
        )?;
    }
    let wcav_size = fs::metadata(&wcav_path)?.len();
    eprintln!(
        "  encoded in {:.2}s ({} bytes, {:.2} MB)",
        t0.elapsed().as_secs_f64(),
        wcav_size,
        wcav_size as f64 / 1048576.0,
    );

    eprintln!("decoding ...");
    let t0 = Instant::now();
    let decoded = {
        let reader = BufReader::new(File::open(&wcav_path)?);
        wind_av1::decode(reader)?
    };
    eprintln!("  decoded in {:.2}s", t0.elapsed().as_secs_f64());

    if decoded.frame_count() != n_frames {
        return Err(format!(
            "frame count mismatch: source {} vs decoded {}",
            n_frames,
            decoded.frame_count(),
        )
        .into());
    }

    eprintln!("comparing ...");
    let mut total_err: f64 = 0.0;
    let mut max_err: f32 = 0.0;
    let mut n_cells: u64 = 0;
    let thresholds = [0.1_f32, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0];
    let mut buckets: [u64; 8] = [0; 8];
    for f in 0..n_frames {
        let src_rows = map.frames()[f].rows();
        let dec_rows = decoded.frames()[f].rows();
        assert_eq!(
            src_rows.len(),
            dec_rows.len(),
            "frame {f} row count mismatch"
        );
        for (s, d) in src_rows.iter().zip(dec_rows.iter()) {
            let (su, sv) = sample_to_uv(&s.sample);
            let (du, dv) = sample_to_uv(&d.sample);
            // Compare per-component; the worst per-cell drift is
            // max(|u_src - u_dec|, |v_src - v_dec|), which is what we
            // budgeted in m/s.
            let eu = (su - du).abs();
            let ev = (sv - dv).abs();
            let err = eu.max(ev);
            total_err += f64::from(err);
            if err > max_err {
                max_err = err;
            }
            for (i, &t) in thresholds.iter().enumerate() {
                if err >= t {
                    buckets[i] += 1;
                }
            }
            n_cells += 1;
        }
    }
    let mean_err = total_err / n_cells as f64;
    println!("rust-round-trip results:");
    println!(
        "  wcav size:     {} bytes ({:.2} MB)",
        wcav_size,
        wcav_size as f64 / 1048576.0
    );
    println!("  cells:         {n_cells}");
    println!("  mean drift:    {:.4} m/s", mean_err);
    println!("  max drift:     {:.4} m/s", max_err);
    println!("  cells with |err| ≥ threshold (out of {n_cells}):");
    for (i, &t) in thresholds.iter().enumerate() {
        let count = buckets[i];
        if count == 0 {
            continue;
        }
        println!(
            "    ≥ {:>5} m/s: {:>12} ({:.4}%)",
            t,
            count,
            count as f64 / n_cells as f64 * 100.0,
        );
    }
    Ok(())
}
