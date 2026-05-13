//! `wind_av1` decoder: reads our 44-byte header, walks the IVF stream,
//! feeds frames into `rav1d` via [`super::rav1d_wrap::Decoder`], and
//! reconstructs a [`TimedWindMap`].
//!
//! Mirrors `wind_codec::decode`: same `Read`-based input, same
//! `TimedWindMap` output, same column-major frame layout
//! (`cell_idx = i * ny + j`), so swapping `.wc1` for `.wcav` in
//! `io::load` is a one-line change in the caller.

use std::io::{self, Read};

use crate::wind_map::GridLayout;
use crate::{TimedWindMap, WeatherRow, WindMap};

use super::ivf::IvfReader;
use super::rav1d_wrap::{Decoder, DecoderError};
use super::{
    HEADER_BYTES_V1, HEADER_BYTES_V2, MAGIC, UNKNOWN_TIME_SENTINEL, dequantize, uv_to_sample,
};

#[derive(Debug)]
#[non_exhaustive]
pub enum DecodeError {
    /// Magic bytes don't match — the file isn't a `wind_av1` file.
    BadMagic,
    /// Version field is something this build doesn't understand.
    UnsupportedVersion(u32),
    /// Header had `nx < 2`, `ny < 2`, or zero frames.
    BadDimensions {
        nx: u32,
        ny: u32,
        frame_count: u32,
    },
    /// Header step values were non-finite or non-positive.
    BadStep {
        step_lon: f32,
        step_lat: f32,
    },
    /// IVF stream ended before the header's `frame_count` pictures had
    /// been decoded. Indicates corruption or truncation.
    Truncated {
        expected: u32,
        got: u32,
    },
    /// Decoded picture dimensions don't match the header grid.
    DimensionMismatch {
        header_nx: usize,
        header_ny: usize,
        pic_w: usize,
        pic_h: usize,
    },
    Av1(DecoderError),
    Io(io::Error),
}

impl std::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BadMagic => f.write_str("not a wind_av1 file (bad magic)"),
            Self::UnsupportedVersion(v) => write!(f, "unsupported wind_av1 version: {v}"),
            Self::BadDimensions {
                nx,
                ny,
                frame_count,
            } => {
                write!(
                    f,
                    "invalid grid dimensions: nx={nx}, ny={ny}, frame_count={frame_count}"
                )
            }
            Self::BadStep { step_lon, step_lat } => {
                write!(
                    f,
                    "invalid grid step: step_lon={step_lon}, step_lat={step_lat}"
                )
            }
            Self::Truncated { expected, got } => {
                write!(
                    f,
                    "IVF stream truncated: expected {expected} frames, decoded {got}"
                )
            }
            Self::DimensionMismatch {
                header_nx,
                header_ny,
                pic_w,
                pic_h,
            } => {
                write!(
                    f,
                    "AV1 picture {pic_w}x{pic_h} doesn't match header grid {header_nx}x{header_ny}",
                )
            }
            Self::Av1(e) => write!(f, "AV1 decode: {e}"),
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

impl From<DecoderError> for DecodeError {
    fn from(e: DecoderError) -> Self {
        Self::Av1(e)
    }
}

/// Decode a `wind_av1` artifact from `reader` into a [`TimedWindMap`].
///
/// # Errors
/// Returns [`DecodeError::BadMagic`] if the file's leading bytes don't
/// match `WCAV\0\0\0\0`; [`DecodeError::UnsupportedVersion`] for a
/// future-version on-disk format; [`DecodeError::BadDimensions`] /
/// [`DecodeError::BadStep`] for malformed header fields;
/// [`DecodeError::Truncated`] if the IVF stream ends before the
/// declared frame count; [`DecodeError::DimensionMismatch`] if the
/// AV1 picture dimensions disagree with the header;
/// [`DecodeError::Av1`] for any rav1d-side failure; or
/// [`DecodeError::Io`] for any reader I/O failure.
pub fn decode<R: Read>(reader: R) -> Result<TimedWindMap, DecodeError> {
    let mut reader = reader;
    let header = read_header(&mut reader)?;
    let layout = GridLayout {
        origin_x: header.origin_lon,
        origin_y: header.origin_lat,
        step_x: header.step_lon,
        step_y: header.step_lat,
        nx: header.nx as usize,
        ny: header.ny as usize,
    };

    let mut ivf = IvfReader::open(reader)?;
    let mut av1 = Decoder::new()?;
    let mut frames: Vec<WindMap> = Vec::with_capacity(header.frame_count as usize);

    // Feed-then-drain loop. On `-EAGAIN` from send_data we pull
    // pictures until the decoder accepts the same bytes; on EOF we
    // drain any remaining pictures still in the pipeline.
    while let Some(packet) = ivf.read_frame()? {
        let mut accepted = av1.feed(&packet)?;
        while !accepted {
            match av1.next_picture()? {
                Some(pic) => collect_picture(&pic, &layout, &mut frames)?,
                // Decoder is empty but still refusing input — shouldn't
                // happen with a well-formed dav1d stream; bail rather
                // than spin.
                None => break,
            }
            accepted = av1.feed(&packet)?;
        }
        while let Some(pic) = av1.next_picture()? {
            collect_picture(&pic, &layout, &mut frames)?;
        }
    }
    // Flush the trailing pictures that were still in dav1d's pipeline
    // when the input ended.
    while let Some(pic) = av1.next_picture()? {
        collect_picture(&pic, &layout, &mut frames)?;
    }

    if frames.len() as u32 != header.frame_count {
        return Err(DecodeError::Truncated {
            expected: header.frame_count,
            got: frames.len() as u32,
        });
    }
    let mut map = TimedWindMap::new(frames, header.step_seconds);
    if let Some((s, e)) = header.time_range {
        map = map.with_time_range(s, e);
    }
    Ok(map)
}

struct Header {
    origin_lon: f32,
    origin_lat: f32,
    step_lon: f32,
    step_lat: f32,
    nx: u32,
    ny: u32,
    frame_count: u32,
    step_seconds: f32,
    /// `Some` only when the file is v2 *and* both endpoints round-trip
    /// to valid `chrono::DateTime<Utc>` values (i.e. neither is the
    /// `UNKNOWN_TIME_SENTINEL` and both are in chrono's representable
    /// range). v1 files always decode to `None` here.
    time_range: Option<(chrono::DateTime<chrono::Utc>, chrono::DateTime<chrono::Utc>)>,
}

fn read_u32_le(buf: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(
        buf[offset..offset + 4]
            .try_into()
            .expect("slice is exactly 4 bytes"),
    )
}

fn read_f32_le(buf: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes(
        buf[offset..offset + 4]
            .try_into()
            .expect("slice is exactly 4 bytes"),
    )
}

fn read_i64_le(buf: &[u8], offset: usize) -> i64 {
    i64::from_le_bytes(
        buf[offset..offset + 8]
            .try_into()
            .expect("slice is exactly 8 bytes"),
    )
}

fn read_header<R: Read>(reader: &mut R) -> Result<Header, DecodeError> {
    // Read the v1-sized prefix first; v2 just adds two `i64` slots at
    // offsets 44 and 52. Branching on `version` lets us walk both
    // file shapes from a single decoder.
    let mut buf = [0u8; HEADER_BYTES_V2];
    reader.read_exact(&mut buf[..HEADER_BYTES_V1])?;
    if buf[0..8] != MAGIC {
        return Err(DecodeError::BadMagic);
    }
    let version = read_u32_le(&buf, 8);
    let time_range = match version {
        1 => None,
        2 => {
            reader.read_exact(&mut buf[HEADER_BYTES_V1..HEADER_BYTES_V2])?;
            let start_unix = read_i64_le(&buf, 44);
            let end_unix = read_i64_le(&buf, 52);
            decode_time_range(start_unix, end_unix)
        }
        _ => return Err(DecodeError::UnsupportedVersion(version)),
    };
    let origin_lon = read_f32_le(&buf, 12);
    let origin_lat = read_f32_le(&buf, 16);
    let step_lon = read_f32_le(&buf, 20);
    let step_lat = read_f32_le(&buf, 24);
    let nx = read_u32_le(&buf, 28);
    let ny = read_u32_le(&buf, 32);
    let frame_count = read_u32_le(&buf, 36);
    let step_seconds = read_f32_le(&buf, 40);

    if nx < 2 || ny < 2 || frame_count == 0 {
        return Err(DecodeError::BadDimensions {
            nx,
            ny,
            frame_count,
        });
    }
    if !step_lon.is_finite() || !step_lat.is_finite() || step_lon <= 0.0 || step_lat <= 0.0 {
        return Err(DecodeError::BadStep { step_lon, step_lat });
    }
    Ok(Header {
        origin_lon,
        origin_lat,
        step_lon,
        step_lat,
        nx,
        ny,
        frame_count,
        step_seconds,
        time_range,
    })
}

/// Convert a pair of Unix-seconds slots from the v2 header into a
/// `(DateTime, DateTime)` pair, or `None` if either slot is the
/// "unknown" sentinel or outside chrono's representable range.
fn decode_time_range(
    start_unix: i64,
    end_unix: i64,
) -> Option<(chrono::DateTime<chrono::Utc>, chrono::DateTime<chrono::Utc>)> {
    if start_unix == UNKNOWN_TIME_SENTINEL || end_unix == UNKNOWN_TIME_SENTINEL {
        return None;
    }
    let start = chrono::DateTime::<chrono::Utc>::from_timestamp(start_unix, 0)?;
    let end = chrono::DateTime::<chrono::Utc>::from_timestamp(end_unix, 0)?;
    Some((start, end))
}

/// Convert one decoded `Dav1dPicture` into a `WindMap` and push it
/// onto the frames vector. Verifies the picture's format and
/// dimensions against the header; reverses the latitude flip applied
/// at encode time.
fn collect_picture(
    pic: &super::rav1d_wrap::DecodedPicture,
    layout: &GridLayout,
    frames: &mut Vec<WindMap>,
) -> Result<(), DecodeError> {
    pic.check_format()?;
    let w = pic.width();
    let h = pic.height();
    if w != layout.nx || h != layout.ny {
        return Err(DecodeError::DimensionMismatch {
            header_nx: layout.nx,
            header_ny: layout.ny,
            pic_w: w,
            pic_h: h,
        });
    }
    let mut rows = Vec::with_capacity(w * h);
    // Pre-fill column-major slots. Outer i = lon, inner j = lat starting
    // from the southernmost (j = 0) to match the encoder's source axis.
    for i in 0..layout.nx {
        let lon = layout.origin_x + (i as f32) * layout.step_x;
        for j in 0..layout.ny {
            let lat = layout.origin_y + (j as f32) * layout.step_y;
            rows.push(WeatherRow {
                lon,
                lat,
                sample: crate::WindSample {
                    speed: 0.0,
                    direction: 0.0,
                },
            });
        }
    }
    // Fill from the AV1 pixel rows (north-at-top, so pixel row 0 is the
    // highest latitude — reverse to land at `j = ny-1`).
    for j_screen in 0..h {
        let j_src = layout.ny - 1 - j_screen;
        let y_row = pic.plane_row(0, j_screen);
        let cb_row = pic.plane_row(1, j_screen);
        for i in 0..w {
            let u = dequantize(y_row[i]);
            let v = dequantize(cb_row[i]);
            let cell_idx = i * layout.ny + j_src;
            rows[cell_idx].sample = uv_to_sample(u, v);
        }
    }
    frames.push(WindMap::from_grid(rows, *layout));
    Ok(())
}
