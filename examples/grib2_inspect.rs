//! Diagnostic dump: walk a GRIB2 file, print every UGRD/VGRD-at-10m message
//! we'd accept, then for the first frame report whether its projected (x, y)
//! grid is detectable as a uniform grid (the criterion `WindMap::new` uses
//! before deciding between the grid path and the kd-tree fallback).
//!
//! Run: `cargo run --example grib2_inspect -- <path/to/file.grib2>`

// Examples are command-line tools — printing the diagnostic dump to
// stdout is the entire user interface, so silence the workspace-level
// `print_stdout` lint here.
#![allow(
    clippy::print_stdout,
    reason = "example binaries use stdout as their entire UI."
)]

use std::collections::BTreeMap;
use std::fs::File;
use std::io::BufReader;

use bywind::TimedWindMap;
use grib::LatLons as _;
use grib::codetables::Code;
use grib::codetables::grib2::Table4_4;

const DISCIPLINE_METEOROLOGICAL: u8 = 0;
const PARAM_CATEGORY_MOMENTUM: u8 = 2;
const PARAM_NUMBER_UGRD: u8 = 2;
const PARAM_NUMBER_VGRD: u8 = 3;
const SURFACE_HEIGHT_ABOVE_GROUND: u8 = 103;
const TARGET_HEIGHT_METRES: f64 = 10.0;
const HEIGHT_TOLERANCE_METRES: f64 = 0.5;
const METRES_PER_DEGREE: f32 = 111_320.0;

/// Holds one accepted UGRD or VGRD submessage's grid + values.
struct AcceptedFrame {
    latlons: Vec<(f32, f32)>,
    values: Vec<f32>,
}

fn unit_to_seconds(unit: &Code<Table4_4, u8>, value: u32) -> Option<u32> {
    let Code::Name(unit) = unit else { return None };
    let mul: u32 = match unit {
        Table4_4::Second => 1,
        Table4_4::Minute => 60,
        Table4_4::Hour => 3600,
        Table4_4::ThreeHours => 3 * 3600,
        Table4_4::SixHours => 6 * 3600,
        Table4_4::TwelveHours => 12 * 3600,
        Table4_4::Day => 86_400,
        _ => return None,
    };
    value.checked_mul(mul)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .expect("usage: grib2_inspect <path>");
    println!("opening {path}");
    let file = File::open(&path)?;
    let reader = BufReader::new(file);
    let grib2 = grib::from_reader(reader)?;

    // (forecast_time_seconds, is_u) → AcceptedFrame
    let mut accepted: BTreeMap<(u32, bool), AcceptedFrame> = BTreeMap::new();
    let mut total_msgs = 0usize;

    for (idx, sub) in &grib2 {
        total_msgs += 1;
        if sub.indicator().discipline != DISCIPLINE_METEOROLOGICAL {
            continue;
        }
        let pd = sub.prod_def();
        let Some(cat) = pd.parameter_category() else {
            continue;
        };
        let Some(num) = pd.parameter_number() else {
            continue;
        };
        if cat != PARAM_CATEGORY_MOMENTUM {
            continue;
        }
        let is_u = num == PARAM_NUMBER_UGRD;
        let is_v = num == PARAM_NUMBER_VGRD;
        if !is_u && !is_v {
            continue;
        }
        let Some((first, _)) = pd.fixed_surfaces() else {
            continue;
        };
        if first.surface_type != SURFACE_HEIGHT_ABOVE_GROUND {
            continue;
        }
        if (first.value() - TARGET_HEIGHT_METRES).abs() > HEIGHT_TOLERANCE_METRES {
            continue;
        }
        let Some(ft) = pd.forecast_time() else {
            continue;
        };
        let Some(t) = unit_to_seconds(&ft.unit, ft.value) else {
            continue;
        };

        let latlons: Vec<(f32, f32)> = match sub.latlons() {
            Ok(it) => it.collect(),
            Err(e) => {
                println!("  msg {idx:?}: latlons err: {e}");
                continue;
            }
        };
        let decoder = match grib::Grib2SubmessageDecoder::from(sub) {
            Ok(d) => d,
            Err(e) => {
                println!("  msg {idx:?}: decoder err: {e}");
                continue;
            }
        };
        let values: Vec<f32> = match decoder.dispatch() {
            Ok(it) => it.collect(),
            Err(e) => {
                println!("  msg {idx:?}: dispatch err: {e:?}");
                continue;
            }
        };

        let finite = values.iter().filter(|v| v.is_finite()).count();
        println!(
            "  msg {idx:?}: t={}s  {}  points={}  finite={}",
            t,
            if is_u { "UGRD" } else { "VGRD" },
            values.len(),
            finite,
        );
        accepted.insert((t, is_u), AcceptedFrame { latlons, values });
    }

    println!("\ntotal submessages in file: {total_msgs}");
    println!("accepted UGRD/VGRD@10m messages: {}", accepted.len());

    let Some(((t0, _), frame)) = accepted.iter().find(|((_, is_u), _)| *is_u) else {
        println!("no UGRD found; aborting projection-grid analysis");
        return Ok(());
    };
    let latlons = &frame.latlons;
    let values = &frame.values;
    println!("\nanalysing first UGRD frame at t={t0}s");

    // Lat/lon ranges
    let (lat_min, lat_max) = latlons
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(mn, mx), &(lat, _)| {
            (mn.min(lat), mx.max(lat))
        });
    let (lon_min, lon_max) = latlons
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(mn, mx), &(_, lon)| {
            (mn.min(lon), mx.max(lon))
        });
    println!(
        "  raw grid: {} points, lat ∈ [{lat_min}, {lat_max}], lon ∈ [{lon_min}, {lon_max}]",
        latlons.len(),
    );

    let mut unique_lats: Vec<f32> = latlons.iter().map(|p| p.0).collect();
    unique_lats.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    unique_lats.dedup();
    let mut unique_lons: Vec<f32> = latlons.iter().map(|p| p.1).collect();
    unique_lons.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    unique_lons.dedup();
    println!(
        "  unique lats: {}, unique lons: {}, product = {} (vs total {})",
        unique_lats.len(),
        unique_lons.len(),
        unique_lats.len() * unique_lons.len(),
        latlons.len(),
    );

    // Project as the loader would
    let lat0 = (lat_min + lat_max) * 0.5;
    let lon0 = (lon_min + lon_max) * 0.5;
    let cos_lat0 = lat0.to_radians().cos();
    println!("  projection origin: lat0={lat0}, lon0={lon0}, cos_lat0={cos_lat0}");

    let xs: Vec<f32> = latlons
        .iter()
        .map(|&(_, lon)| (lon - lon0) * cos_lat0 * METRES_PER_DEGREE)
        .collect();
    let ys: Vec<f32> = latlons
        .iter()
        .map(|&(lat, _)| (lat - lat0) * METRES_PER_DEGREE)
        .collect();

    let mut ux = xs.clone();
    ux.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    ux.dedup();
    let mut uy = ys.clone();
    uy.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    uy.dedup();
    println!(
        "  projected: unique xs={}, unique ys={}, product={}",
        ux.len(),
        uy.len(),
        ux.len() * uy.len(),
    );

    // After dropping NaN values (what the loader currently does)
    let finite_count = values.iter().filter(|v| v.is_finite()).count();
    println!("  finite values: {finite_count}");
    println!(
        "  loader currently emits {} rows (drops {} NaN samples)",
        finite_count,
        latlons.len() - finite_count,
    );

    // Detect uniform spacing
    if ux.len() >= 2 {
        let step = ux[1] - ux[0];
        let max_dev = ux
            .windows(2)
            .map(|w| (w[1] - w[0] - step).abs())
            .fold(0.0_f32, f32::max);
        println!(
            "  x step: {step}, max deviation from uniform: {max_dev}, tol = {}",
            step.abs() * 1e-3
        );
    }
    if uy.len() >= 2 {
        let step = uy[1] - uy[0];
        let max_dev = uy
            .windows(2)
            .map(|w| (w[1] - w[0] - step).abs())
            .fold(0.0_f32, f32::max);
        println!(
            "  y step: {step}, max deviation from uniform: {max_dev}, tol = {}",
            step.abs() * 1e-3
        );
    }

    println!("\nrunning the actual loader (stride=8) to verify multi-cycle handling");
    let f = File::open(&path)?;
    let f = BufReader::new(f);
    match TimedWindMap::from_grib2_reader(f, 8, None) {
        Ok(map) => {
            println!(
                "  loader produced {} frames, step_seconds = {}",
                map.frame_count(),
                map.step_seconds(),
            );
        }
        Err(e) => {
            println!("  loader failed: {e}");
        }
    }

    Ok(())
}
