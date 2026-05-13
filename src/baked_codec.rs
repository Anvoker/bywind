//! Compact binary codec for [`BakedWindMap`].
//!
//! Lets the CLI cache a baked wind map across runs (`--save-baked` /
//! `--load-baked`) so hyperparameter sweeps over the same map skip
//! the (often multi-second) bake step.
//!
//! ## File layout
//!
//! All multi-byte fields are little-endian.
//!
//! | Offset | Size | Field             | Notes |
//! |-------:|-----:|-------------------|-------|
//! |      0 |    8 | `magic`           | `b"WBAKED01"` |
//! |      8 |    4 | `version`         | `u32`, currently `1` |
//! |     12 |    8 | `nx`              | `u64`, longitude cell count |
//! |     20 |    8 | `ny`              | `u64`, latitude cell count |
//! |     28 |    8 | `nt`              | `u64`, frame count |
//! |     36 |    8 | `x_min`           | `f64`, deg lon of cell `(0, *)` |
//! |     44 |    8 | `y_min`           | `f64`, deg lat of cell `(*, 0)` |
//! |     52 |    8 | `step`            | `f64`, deg per cell along both axes |
//! |     60 |    8 | `t_step_seconds`  | `f64`, seconds between frames |
//! |     68 |    8 | `coord_scale`     | `f64`, divisor applied at sample-time |
//! |     76 |    — | zstd stream       | `nx × ny × nt × 16 bytes` of grid data |
//!
//! Decompressed payload: the same row-major-by-`(iy, ix)` and time-innermost
//! layout `BakedWindMap` uses internally — `grid[(iy * nx + ix) * nt + it]`.
//! Each cell is two `f64` LE values: `u_east` then `v_north` (m/s).
//!
//! No quantization: the baked grid is full-precision `f64`, so we lean on
//! zstd to compress it. Typical regional bake (60° × 30° × 720h at 0.25°
//! ~= 332 MB raw) compresses to roughly 100–150 MB at level 11.

use std::io::{self, Read, Write};

use swarmkit_sailing::spherical::Wind;

use crate::wind_map::BakedWindMap;

const MAGIC: [u8; 8] = *b"WBAKED01";
const VERSION: u32 = 1;
/// Sweet-spot zstd level for "compress once, load many times". Same value
/// `wind_codec` settled on after the Phase 6 perf pass.
const ZSTD_LEVEL: i32 = 11;
const HEADER_BYTES: usize = 76;
const BYTES_PER_CELL: usize = 16;

#[derive(Debug)]
#[non_exhaustive]
pub enum EncodeError {
    Io(io::Error),
}

impl std::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "I/O error: {e}"),
        }
    }
}

impl std::error::Error for EncodeError {}

impl From<io::Error> for EncodeError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

#[derive(Debug)]
#[non_exhaustive]
pub enum DecodeError {
    BadMagic,
    UnsupportedVersion(u32),
    BadDimensions { nx: u64, ny: u64, nt: u64 },
    Io(io::Error),
}

impl std::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BadMagic => f.write_str("not a bywind baked-cache file (bad magic)"),
            Self::UnsupportedVersion(v) => write!(f, "unsupported baked-cache version: {v}"),
            Self::BadDimensions { nx, ny, nt } => {
                write!(f, "invalid grid dimensions: nx={nx}, ny={ny}, nt={nt}")
            }
            Self::Io(e) => write!(f, "I/O error: {e}"),
        }
    }
}

impl std::error::Error for DecodeError {}

impl From<io::Error> for DecodeError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

/// Encode `map` to `writer` (header + zstd-compressed payload).
///
/// # Errors
/// Returns [`EncodeError::Io`] if the writer or zstd stream fails on
/// any header or payload write.
pub fn encode<W: Write>(map: &BakedWindMap, writer: W) -> Result<(), EncodeError> {
    let mut writer = writer;
    write_header(&mut writer, map)?;

    let mut zenc = zstd::stream::write::Encoder::new(writer, ZSTD_LEVEL)?;
    let workers: u32 = std::thread::available_parallelism()
        .ok()
        .and_then(|n| u32::try_from(n.get()).ok())
        .unwrap_or(1);
    if let Err(e) = zenc.multithread(workers) {
        log::warn!("zstd multithread setup failed (continuing single-threaded): {e}");
    }

    // The grid is contiguous `Vec<Wind>` with the
    // `(iy * nx + ix) * nt + it` ordering; we serialise in storage order.
    let mut buf = [0u8; BYTES_PER_CELL];
    for v in &map.grid {
        buf[..8].copy_from_slice(&v.east_mps.to_le_bytes());
        buf[8..].copy_from_slice(&v.north_mps.to_le_bytes());
        zenc.write_all(&buf)?;
    }
    zenc.finish()?;
    Ok(())
}

/// Decode a `BakedWindMap` from `reader`.
///
/// # Errors
/// Returns [`DecodeError::BadMagic`] if the file's magic bytes don't
/// match; [`DecodeError::UnsupportedVersion`] if the version field is
/// unknown; [`DecodeError::BadDimensions`] if `nx * ny * nt` overflows
/// `usize`; or [`DecodeError::Io`] for any underlying read or
/// zstd-decompression failure.
pub fn decode<R: Read>(reader: R) -> Result<BakedWindMap, DecodeError> {
    let mut reader = reader;
    let header = read_header(&mut reader)?;

    let total_cells = (header.nx as usize)
        .checked_mul(header.ny as usize)
        .and_then(|p| p.checked_mul(header.nt as usize))
        .ok_or(DecodeError::BadDimensions {
            nx: header.nx,
            ny: header.ny,
            nt: header.nt,
        })?;
    let payload_bytes =
        total_cells
            .checked_mul(BYTES_PER_CELL)
            .ok_or(DecodeError::BadDimensions {
                nx: header.nx,
                ny: header.ny,
                nt: header.nt,
            })?;

    let mut zdec = zstd::stream::read::Decoder::new(reader)?;
    // Decompress the whole payload into a buffer in one shot — much faster
    // than 2-byte-per-cell reads, and the buffer is the same size we'll
    // need for the resulting grid anyway.
    let mut bytes = vec![0u8; payload_bytes];
    zdec.read_exact(&mut bytes)?;

    let mut grid: Vec<Wind> = Vec::with_capacity(total_cells);
    for chunk in bytes.chunks_exact(BYTES_PER_CELL) {
        // `chunks_exact` guarantees each `chunk` has length
        // `BYTES_PER_CELL` (= 16), so the 8-byte sub-slices passed to
        // `read_f64_le` always have the required length.
        grid.push(Wind::new(
            read_f64_le(&chunk[..8]),
            read_f64_le(&chunk[8..]),
        ));
    }

    Ok(BakedWindMap {
        grid,
        nx: header.nx as usize,
        ny: header.ny as usize,
        nt: header.nt as usize,
        x_min: header.x_min,
        y_min: header.y_min,
        step: header.step,
        t_step_seconds: header.t_step_seconds,
        // `crossfade_seconds` is not part of the on-disk schema yet —
        // derive it from `t_step_seconds` using the same 5-frame rule
        // `TimedWindMap::new` applies. If we ever want per-map
        // customisation persisted across bakes, bump the on-disk
        // version and store it explicitly.
        crossfade_seconds: 5.0 * header.t_step_seconds,
        coord_scale: header.coord_scale,
    })
}

struct Header {
    nx: u64,
    ny: u64,
    nt: u64,
    x_min: f64,
    y_min: f64,
    step: f64,
    t_step_seconds: f64,
    coord_scale: f64,
}

fn write_header<W: Write>(writer: &mut W, map: &BakedWindMap) -> io::Result<()> {
    let mut buf = [0u8; HEADER_BYTES];
    buf[0..8].copy_from_slice(&MAGIC);
    buf[8..12].copy_from_slice(&VERSION.to_le_bytes());
    buf[12..20].copy_from_slice(&(map.nx as u64).to_le_bytes());
    buf[20..28].copy_from_slice(&(map.ny as u64).to_le_bytes());
    buf[28..36].copy_from_slice(&(map.nt as u64).to_le_bytes());
    buf[36..44].copy_from_slice(&map.x_min.to_le_bytes());
    buf[44..52].copy_from_slice(&map.y_min.to_le_bytes());
    buf[52..60].copy_from_slice(&map.step.to_le_bytes());
    buf[60..68].copy_from_slice(&map.t_step_seconds.to_le_bytes());
    buf[68..76].copy_from_slice(&map.coord_scale.to_le_bytes());
    writer.write_all(&buf)
}

fn read_header<R: Read>(reader: &mut R) -> Result<Header, DecodeError> {
    let mut buf = [0u8; HEADER_BYTES];
    reader.read_exact(&mut buf)?;
    if buf[0..8] != MAGIC {
        return Err(DecodeError::BadMagic);
    }
    let version = read_u32_le(&buf[8..12]);
    if version != VERSION {
        return Err(DecodeError::UnsupportedVersion(version));
    }
    Ok(Header {
        nx: read_u64_le(&buf[12..20]),
        ny: read_u64_le(&buf[20..28]),
        nt: read_u64_le(&buf[28..36]),
        x_min: read_f64_le(&buf[36..44]),
        y_min: read_f64_le(&buf[44..52]),
        step: read_f64_le(&buf[52..60]),
        t_step_seconds: read_f64_le(&buf[60..68]),
        coord_scale: read_f64_le(&buf[68..76]),
    })
}

fn read_u32_le(slice: &[u8]) -> u32 {
    let bytes: [u8; 4] = slice.try_into().expect("4-byte slice required");
    u32::from_le_bytes(bytes)
}

fn read_u64_le(slice: &[u8]) -> u64 {
    let bytes: [u8; 8] = slice.try_into().expect("8-byte slice required");
    u64::from_le_bytes(bytes)
}

fn read_f64_le(slice: &[u8]) -> f64 {
    let bytes: [u8; 8] = slice.try_into().expect("8-byte slice required");
    f64::from_le_bytes(bytes)
}
