//! AV1 near-lossless codec for [`crate::TimedWindMap`] data on a regular
//! lat/lon grid.
//!
//! Encodes the time-varying `(u_east, v_north)` field as a 10-bit
//! 4:4:4 AV1 bitstream via the pure-Rust `rav1e` encoder, and decodes
//! it back with the matching `rav1d` decoder. No external tooling
//! (no ffmpeg / libaom build) involved on either side. The temporal +
//! spatial prediction inside AV1 is considerably more aggressive than
//! a generic compressor would manage on the same data, so the on-disk
//! artifact for a 720-hour global hourly GFS lands around **25 MB** —
//! small enough to `include_bytes!` into the GUI binary as a default
//! dataset.
//!
//! ## Pixel packing
//!
//! Each cell carries two scalars, `u_east` and `v_north`. We quantize
//! them linearly into 10-bit unsigned pixels:
//!
//! * `pixel = round((value + QUANT_MIN) / QUANT_RANGE * QUANT_LEVELS)`
//! * `value = pixel / QUANT_LEVELS * QUANT_RANGE - QUANT_MIN`
//!
//! with `QUANT_MIN = -60.0`, `QUANT_RANGE = 120.0`, `QUANT_LEVELS = 1023`.
//! `±60 m/s` brackets every observed surface wind (the strongest cyclonic
//! gusts in operational forecasts top out around 50 m/s), and 1024 buckets
//! over 120 m/s gives ~0.12 m/s per bucket — well under the AV1 codec
//! drift, so the pre-codec quantization is essentially noise-free.
//!
//! The 4:4:4 layout puts `u` on the Y plane, `v` on the Cb plane, and a
//! constant midpoint sentinel on the Cr plane. AV1's cross-component
//! prediction can use Cb when encoding Cr — feeding it a flat sentinel
//! collapses Cr's bitrate to almost nothing. Full-range (`color_range pc`)
//! is used both ways so the encoder doesn't clip to TV range.
//!
//! ## File layout
//!
//! The header has two on-disk versions:
//!
//! * **v1** (44-byte header, no datetime fields). The encoder no
//!   longer emits v1 but the decoder still reads it for backward
//!   compatibility with files produced before the v2 bump.
//! * **v2** (60-byte header, adds `start_unix_seconds` /
//!   `end_unix_seconds` at offsets 44 and 52). Both fields are
//!   `i64::MIN` when the dataset doesn't carry a UTC range
//!   (synthetic generators, hand-rolled wind maps).
//!
//! | Offset | Size | Field          | Notes |
//! |-------:|-----:|----------------|-------|
//! |      0 |    8 | `magic`        | `b"WCAV\0\0\0\0"` |
//! |      8 |    4 | `version`      | `u32`, current `2`; readers also accept `1` |
//! |     12 |    4 | `origin_lon`   | `f32`, longitude of cell `i=0` (degrees) |
//! |     16 |    4 | `origin_lat`   | `f32`, latitude of cell `j=0` (degrees) |
//! |     20 |    4 | `step_lon`     | `f32`, degrees per cell along longitude |
//! |     24 |    4 | `step_lat`     | `f32`, degrees per cell along latitude |
//! |     28 |    4 | `nx`           | `u32`, longitude cell count |
//! |     32 |    4 | `ny`           | `u32`, latitude cell count |
//! |     36 |    4 | `frame_count`  | `u32`, number of time frames |
//! |     40 |    4 | `step_seconds` | `f32`, seconds between frames |
//! |     44 |    8 | `start_unix`   | `i64` LE Unix seconds (v2 only); `i64::MIN` = unknown |
//! |     52 |    8 | `end_unix`     | `i64` LE Unix seconds (v2 only); `i64::MIN` = unknown |
//! |     60 |    — | IVF stream     | AV1 frames (1 OBU per IVF frame, framerate=1) |
//!
//! ## Latitude axis convention
//!
//! GRIB2's column-major source layout puts `j = 0` at the southernmost
//! latitude — natural for the spec but upside-down for AV1 (which
//! expects pixel row 0 at the top of the frame). The encoder flips the
//! latitude axis on the way in and the decoder reverses the flip on the
//! way out, so the codec is invisible to the caller. Inside the AV1
//! stream, pixel row 0 is the highest latitude.

#[cfg(not(target_arch = "wasm32"))]
mod decoder;
#[cfg(not(target_arch = "wasm32"))]
mod encoder;
#[cfg(not(target_arch = "wasm32"))]
mod ivf;
#[cfg(not(target_arch = "wasm32"))]
mod rav1d_wrap;

#[cfg(not(target_arch = "wasm32"))]
pub use decoder::{DecodeError, decode};
#[cfg(not(target_arch = "wasm32"))]
pub use encoder::{EncodeError, EncodeParams, encode};

use crate::wind_map::GridLayout;

/// File magic prefix; same in every version.
pub const MAGIC: [u8; 8] = *b"WCAV\0\0\0\0";
/// Current on-disk format version. The encoder always emits this; the
/// decoder also accepts older versions for backward compatibility.
pub const VERSION: u32 = 2;
/// Bytes the v1 header occupies, before the IVF payload starts. Kept
/// as a constant because the decoder still walks v1 files produced
/// before the v2 bump.
pub const HEADER_BYTES_V1: usize = 44;
/// Bytes the v2 header occupies, before the IVF payload starts. v2
/// adds two `i64` UTC timestamps at offsets 44 and 52.
pub const HEADER_BYTES_V2: usize = 60;
/// Bytes the *current* header occupies. Always equal to the latest
/// version's size.
pub const HEADER_BYTES: usize = HEADER_BYTES_V2;
/// Sentinel value for "the dataset's UTC time range is unknown" in
/// the v2 header's `start_unix` / `end_unix` slots. Distinct from
/// `0` (the actual Unix epoch) and `-1` (one second before epoch).
pub const UNKNOWN_TIME_SENTINEL: i64 = i64::MIN;

/// Quantization origin: physical value (m/s) that maps to pixel `0`.
pub const QUANT_MIN: f32 = -60.0;
/// Quantization span: full physical range covered by the pixel space.
pub const QUANT_RANGE: f32 = 120.0;
/// Number of distinct pixel levels above zero — for 10-bit unsigned
/// this is `2^10 - 1 = 1023`. Stored as f32 because every arithmetic
/// site uses it as a multiplier rather than an index count.
pub const QUANT_LEVELS_F32: f32 = 1023.0;
/// Cr-plane sentinel: the constant midpoint pixel value, chosen so the
/// encoder doesn't see any signal in Cr at all.
pub const CR_SENTINEL: u16 = 512;

/// Quantize a physical value (m/s) to a 10-bit pixel, saturating at the ends.
///
/// The clamp guards against cyclonic outliers above the `±60 m/s`
/// modelled range — the small fraction of cells that saturate still
/// round-trip to whatever pixel value we picked, just with a clipped
/// magnitude.
pub fn quantize(v: f32) -> u16 {
    let q = ((v - QUANT_MIN) / QUANT_RANGE * QUANT_LEVELS_F32).round();
    q.clamp(0.0, QUANT_LEVELS_F32) as u16
}

/// Inverse of [`quantize`]. Lossless to the resolution of one pixel
/// (`QUANT_RANGE / QUANT_LEVELS_F32 ≈ 0.117 m/s`); AV1's reconstruction
/// drift dominates this in practice.
pub fn dequantize(q: u16) -> f32 {
    f32::from(q) / QUANT_LEVELS_F32 * QUANT_RANGE + QUANT_MIN
}

/// `(speed, direction)` → `(u_east, v_north)`.
///
/// Mirrors the convention `grib2` and `wind_codec` use: `direction` is
/// a "from"-bearing in degrees clockwise from north, so the wind
/// vector points the opposite way.
pub fn sample_to_uv(s: &crate::WindSample) -> (f32, f32) {
    let theta = (270.0 - s.direction).to_radians();
    let speed = if s.speed.is_finite() { s.speed } else { 0.0 };
    (speed * theta.cos(), speed * theta.sin())
}

/// Inverse of [`sample_to_uv`], with the same convention.
pub fn uv_to_sample(u: f32, v: f32) -> crate::WindSample {
    let speed = u.hypot(v);
    let direction = if speed == 0.0 {
        0.0
    } else {
        (270.0 - v.atan2(u).to_degrees()).rem_euclid(360.0)
    };
    crate::WindSample { speed, direction }
}

/// Layout-equality check for encoder validation.
///
/// All frames in a `TimedWindMap` must share one grid. `f32` fields
/// are compared by bit pattern so the test is exact (no NaN ambiguity)
/// and serialises cleanly through the header.
pub fn grid_matches(a: &GridLayout, b: &GridLayout) -> bool {
    a.nx == b.nx
        && a.ny == b.ny
        && a.origin_x.to_bits() == b.origin_x.to_bits()
        && a.origin_y.to_bits() == b.origin_y.to_bits()
        && a.step_x.to_bits() == b.step_x.to_bits()
        && a.step_y.to_bits() == b.step_y.to_bits()
}

/// Pixels per cell after packing both U and V into 10-bit values stored
/// in 16-bit little-endian containers. Exported because the encoder and
/// IVF parser both reason about it.
pub const BYTES_PER_PIXEL: usize = 2;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quantize_round_trip_lossless_within_one_bucket() {
        // Spot-check several values across the supported range. Each
        // round-trip error is bounded by half a pixel (~0.06 m/s).
        let cases = [-50.0_f32, -1.0, 0.0, 0.5, 12.34, 49.9];
        for v in cases {
            let q = quantize(v);
            let back = dequantize(q);
            let err = (v - back).abs();
            assert!(err < 0.07, "{v} → {q} → {back}, err = {err}");
        }
    }

    #[test]
    fn quantize_clamps_extreme_values() {
        // Cyclonic outliers above the modelled range saturate at the
        // pixel ceiling rather than wrapping.
        assert_eq!(quantize(200.0), QUANT_LEVELS_F32 as u16);
        assert_eq!(quantize(-200.0), 0);
    }

    #[test]
    fn sample_uv_round_trip() {
        // direction = 45° from north, speed = 10 → wind blows toward
        // 45° + 180° = 225° (southwest), so u and v are both negative
        // with equal magnitude.
        let s = crate::WindSample {
            speed: 10.0,
            direction: 45.0,
        };
        let (u, v) = sample_to_uv(&s);
        let back = uv_to_sample(u, v);
        assert!((back.speed - s.speed).abs() < 1e-4);
        // wrap-safe direction diff
        let d = ((back.direction - s.direction + 540.0) % 360.0) - 180.0;
        assert!(d.abs() < 1e-3);
    }
}
