//! `convert` subcommand: bake a slow-to-parse GRIB2 file into the
//! compact [`bywind::wind_av1`] AV1 near-lossless format. The only
//! conversion direction `bywind` supports today; the in/out dispatch
//! lives behind [`Format::from_path`] so adding a new format later
//! just means a new match arm.

use std::fs::File;
use std::io::BufWriter;
use std::path::Path;
use std::time::Instant;

use anyhow::{Context as _, Result, anyhow};
use bywind::{Grib2Bbox, TimedWindMap, io::Format, wind_av1};

use crate::error::AppError;
use crate::parsing::parse_n_floats;

pub fn run(
    input: &Path,
    out: &Path,
    grib_stride: Option<usize>,
    grib_bbox: Option<&str>,
) -> Result<(), AppError> {
    let in_fmt = Format::from_path(input).context("input path")?;
    let out_fmt = Format::from_path(out).context("output path")?;

    if matches!(out_fmt, Format::Grib2) {
        return Err(anyhow!("GRIB2 output is not supported — bywind has no GRIB2 writer").into());
    }
    if in_fmt == out_fmt {
        return Err(anyhow!("input and output are both {}; nothing to do", in_fmt.name()).into());
    }

    let bbox = grib_bbox.map(parse_bbox).transpose()?;
    let stride = grib_stride.unwrap_or(1);

    eprintln!("loading {} as {}...", input.display(), in_fmt.name());
    let load_start = Instant::now();
    let map = bywind::io::load(input, stride, bbox)
        .with_context(|| format!("loading wind map from {}", input.display()))?;
    eprintln!(
        "  loaded in {:.2}s: {} frame(s), step = {} s",
        load_start.elapsed().as_secs_f64(),
        map.frame_count(),
        map.step_seconds(),
    );

    eprintln!("writing {} as {}...", out.display(), out_fmt.name());
    let write_start = Instant::now();
    match out_fmt {
        Format::WindAv1 => write_wcav(out, &map)?,
        Format::Grib2 => {
            return Err(anyhow!("GRIB2 output is not supported (already rejected above)").into());
        }
    }
    eprintln!("  wrote in {:.2}s", write_start.elapsed().as_secs_f64());
    Ok(())
}

/// Parse a `lat_min,lon_min,lat_max,lon_max` string into a `Grib2Bbox`.
fn parse_bbox(s: &str) -> Result<Grib2Bbox> {
    let [lat_min, lon_min, lat_max, lon_max]: [f32; 4] = parse_n_floats(
        s,
        ["lat_min", "lon_min", "lat_max", "lon_max"],
        "--grib-bbox",
    )?;
    Ok(Grib2Bbox {
        lat_min,
        lon_min,
        lat_max,
        lon_max,
    })
}

fn write_wcav(path: &Path, map: &TimedWindMap) -> Result<()> {
    let file = File::create(path).with_context(|| format!("creating {}", path.display()))?;
    let writer = BufWriter::new(file);
    wind_av1::encode(map, writer, wind_av1::EncodeParams::default())
        .context("encoding wind_av1 via rav1e")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verifies the wrapper's component → struct-field mapping. The
    /// arity / whitespace / non-numeric edge cases live in
    /// `crate::parsing::tests` so they aren't repeated per-flag.
    #[test]
    fn parse_bbox_maps_components_to_lat_lon_bbox_fields() {
        let b = parse_bbox("30,-60,40,10").expect("valid bbox should parse");
        assert!((b.lat_min - 30.0).abs() < 1e-6);
        assert!((b.lon_min + 60.0).abs() < 1e-6);
        assert!((b.lat_max - 40.0).abs() < 1e-6);
        assert!((b.lon_max - 10.0).abs() < 1e-6);
    }
}
