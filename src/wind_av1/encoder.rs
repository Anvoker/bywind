//! `wind_av1` encoder. Drives the pure-Rust `rav1e` AV1 encoder against
//! packed `yuv444p10le` frames and writes the resulting AV1 OBUs into
//! an IVF container directly after our 44-byte header.
//!
//! No external tooling involved — rav1e is a `Cargo.toml` dependency
//! like any other. Encoding is offline one-shot work so the trade vs
//! a more aggressive C encoder is fine.

use std::io::{self, Write};
use std::sync::Arc;

use rav1e::Config;
use rav1e::Context;
use rav1e::EncoderConfig;
use rav1e::Frame;
use rav1e::InvalidConfig;
use rav1e::data::{EncoderStatus, Packet};
use rav1e::prelude::ChromaSampling;
use rav1e::prelude::PixelRange;
use rav1e::prelude::Rational;

use crate::TimedWindMap;
use crate::wind_map::GridLayout;

use super::{
    CR_SENTINEL, HEADER_BYTES, MAGIC, UNKNOWN_TIME_SENTINEL, VERSION, grid_matches, quantize,
    sample_to_uv,
};

/// Codec tuning for the rav1e encoder.
///
/// Defaults were picked after a sweep against the previous libaom
/// CRF 10 baseline (see the pre-release roadmap's Phase 1.1 section).
///
/// `quantizer` is rav1e's base quantizer, scale 0..=255 (NOT libaom's
/// 0..=63 CRF — same semantic of "higher = lossier", different scale).
/// `speed_preset` is rav1e's 0..=10 speed setting (0 slowest/best,
/// 10 fastest/worst).
#[derive(Clone, Copy, Debug)]
pub struct EncodeParams {
    /// rav1e base quantizer (0..=255).
    pub quantizer: u8,
    /// rav1e speed preset (0..=10).
    pub speed_preset: u8,
}

impl Default for EncodeParams {
    fn default() -> Self {
        // Picked from a 168h GFS sweep against the previous libaom
        // CRF 10 baseline (5.6 MB, 0.38 m/s mean drift). Interpolating
        // between q=20 (0.31 drift, 10.5 MB) and q=40 (0.46 drift,
        // 3.7 MB) puts q=30 at ~0.39 m/s drift / ~7 MB — drift parity
        // with libaom, size ~25% over. The bundled 720h artifact lands
        // around 30 MB at this setting, comfortably inside the
        // "small enough to include_bytes!" budget.
        Self {
            quantizer: 30,
            speed_preset: 6,
        }
    }
}

#[derive(Debug)]
#[non_exhaustive]
pub enum EncodeError {
    /// `TimedWindMap` had zero frames.
    Empty,
    /// A frame is not stored on a regular grid (e.g. it's a kd-tree
    /// map because [`crate::WindMap::new`] didn't see uniform spacing).
    NonGridFrame {
        frame: usize,
    },
    /// Two frames disagree on grid layout. Codec assumes the entire
    /// `TimedWindMap` shares one `(origin, step, nx, ny)`.
    InconsistentGrid {
        frame: usize,
    },
    /// A dimension or frame count didn't fit in the on-disk `u32`
    /// field. `what` names the offending field.
    Overflow {
        what: &'static str,
        value: usize,
    },
    /// `rav1e` rejected the requested `EncoderConfig`. Most common
    /// causes: invalid `width`/`height` combinations, invalid
    /// quantizer/preset values.
    InvalidConfig(InvalidConfig),
    /// `rav1e` returned a hard error from `send_frame`/`receive_packet`.
    /// The wrapped status carries the diagnostic; everything except
    /// `Encoded`/`NeedMoreData`/`LimitReached` is propagated here.
    Rav1e(EncoderStatus),
    Io(io::Error),
}

impl std::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Empty => f.write_str("TimedWindMap has no frames"),
            Self::NonGridFrame { frame } => write!(f, "frame {frame} is not on a regular grid"),
            Self::InconsistentGrid { frame } => {
                write!(f, "frame {frame}'s grid layout differs from frame 0")
            }
            Self::Overflow { what, value } => {
                write!(
                    f,
                    "{what} = {value} doesn't fit in u32 (on-disk format limit)"
                )
            }
            Self::InvalidConfig(e) => write!(f, "rav1e rejected the encoder config: {e:?}"),
            Self::Rav1e(s) => write!(f, "rav1e encoder error: {s:?}"),
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

impl From<InvalidConfig> for EncodeError {
    fn from(e: InvalidConfig) -> Self {
        Self::InvalidConfig(e)
    }
}

/// Encode `map` to `writer`.
///
/// The map must be non-empty, every frame must be on a regular grid,
/// and all frames must share the same grid layout.
///
/// # Errors
/// - [`EncodeError::Empty`] if `map` has zero frames.
/// - [`EncodeError::NonGridFrame`] / [`EncodeError::InconsistentGrid`]
///   for irregular-grid inputs.
/// - [`EncodeError::Overflow`] if `nx`, `ny`, or the frame count
///   exceeds `u32`.
/// - [`EncodeError::InvalidConfig`] if rav1e rejects the encoder
///   config for the input dimensions.
/// - [`EncodeError::Rav1e`] for any rav1e-side encoder failure.
/// - [`EncodeError::Io`] for any underlying writer failure.
pub fn encode<W: Write>(
    map: &TimedWindMap,
    writer: W,
    params: EncodeParams,
) -> Result<(), EncodeError> {
    let frames = map.frames();
    if frames.is_empty() {
        return Err(EncodeError::Empty);
    }
    let layout = frames[0]
        .grid_layout()
        .ok_or(EncodeError::NonGridFrame { frame: 0 })?;
    for (idx, frame) in frames.iter().enumerate().skip(1) {
        let l = frame
            .grid_layout()
            .ok_or(EncodeError::NonGridFrame { frame: idx })?;
        if !grid_matches(&layout, &l) {
            return Err(EncodeError::InconsistentGrid { frame: idx });
        }
    }

    let nx_u32 = u32::try_from(layout.nx).ok().ok_or(EncodeError::Overflow {
        what: "nx",
        value: layout.nx,
    })?;
    let ny_u32 = u32::try_from(layout.ny).ok().ok_or(EncodeError::Overflow {
        what: "ny",
        value: layout.ny,
    })?;
    let frame_count_u32 = u32::try_from(frames.len())
        .ok()
        .ok_or(EncodeError::Overflow {
            what: "frame_count",
            value: frames.len(),
        })?;

    let mut writer = writer;
    let (start_unix, end_unix) = match map.time_range() {
        Some((s, e)) => (s.timestamp(), e.timestamp()),
        None => (UNKNOWN_TIME_SENTINEL, UNKNOWN_TIME_SENTINEL),
    };
    write_header(
        &mut writer,
        &layout,
        nx_u32,
        ny_u32,
        frame_count_u32,
        map.step_seconds(),
        start_unix,
        end_unix,
    )?;

    encode_payload(&mut writer, frames, &layout, frame_count_u32, &params)
}

fn encode_payload<W: Write>(
    writer: &mut W,
    frames: &[crate::WindMap],
    layout: &GridLayout,
    frame_count: u32,
    params: &EncodeParams,
) -> Result<(), EncodeError> {
    let nx = layout.nx;
    let ny = layout.ny;

    // IVF container — width/height fit in u16 because rav1e itself
    // caps frame dimensions well below that.
    let width_u16 = u16::try_from(nx).ok().ok_or(EncodeError::Overflow {
        what: "nx (IVF)",
        value: nx,
    })?;
    let height_u16 = u16::try_from(ny).ok().ok_or(EncodeError::Overflow {
        what: "ny (IVF)",
        value: ny,
    })?;
    super::ivf::write_file_header(writer, width_u16, height_u16, 1, 1, frame_count)?;

    // Encoder config: 10-bit 4:4:4 full-range, matching what the
    // decoder expects (and what the libaom + ffmpeg pipeline produced
    // before this commit).
    let mut enc_config = EncoderConfig::with_speed_preset(params.speed_preset);
    enc_config.width = nx;
    enc_config.height = ny;
    enc_config.bit_depth = 10;
    enc_config.chroma_sampling = ChromaSampling::Cs444;
    enc_config.pixel_range = PixelRange::Full;
    enc_config.quantizer = params.quantizer as usize;
    enc_config.time_base = Rational { num: 1, den: 1 };

    let cfg = Config::new().with_encoder_config(enc_config);
    let mut ctx: Context<u16> = cfg.new_context()?;

    let cr_plane_u16 = vec![CR_SENTINEL; nx * ny];
    let mut y_plane_u16 = vec![0u16; nx * ny];
    let mut cb_plane_u16 = vec![0u16; nx * ny];
    let mut pts: u64 = 0;

    for source in frames {
        pack_into_planes(&mut y_plane_u16, &mut cb_plane_u16, source.rows(), nx, ny);
        let mut frame: Frame<u16> = ctx.new_frame();
        copy_u16_into_plane(&mut frame.planes[0], &y_plane_u16, nx);
        copy_u16_into_plane(&mut frame.planes[1], &cb_plane_u16, nx);
        copy_u16_into_plane(&mut frame.planes[2], &cr_plane_u16, nx);
        ctx.send_frame(Arc::new(frame))
            .map_err(EncodeError::Rav1e)?;
        drain_packets(&mut ctx, writer, &mut pts)?;
    }
    ctx.flush();
    drain_packets(&mut ctx, writer, &mut pts)?;
    Ok(())
}

/// Pull every packet rav1e currently has buffered and mux each into
/// the IVF container. Retries on `Encoded` (encoder did internal work
/// without producing a packet — common in lookahead mode), exits on
/// `NeedMoreData` (send more frames) or `LimitReached` (encoding
/// complete after `flush`).
fn drain_packets<W: Write>(
    ctx: &mut Context<u16>,
    writer: &mut W,
    pts: &mut u64,
) -> Result<(), EncodeError> {
    loop {
        match ctx.receive_packet() {
            Ok(Packet { data, .. }) => {
                super::ivf::write_frame(writer, &data, *pts)?;
                *pts += 1;
            }
            // `Encoded` means "I did internal work, try again". The
            // distinction from `NeedMoreData` matters because in
            // lookahead mode rav1e can return `Encoded` many times
            // before emitting a single packet; bailing here would
            // drop everything until the next `send_frame` call.
            Err(EncoderStatus::Encoded) => {}
            Err(EncoderStatus::NeedMoreData | EncoderStatus::LimitReached) => {
                return Ok(());
            }
            Err(other) => return Err(EncodeError::Rav1e(other)),
        }
    }
}

#[expect(
    clippy::too_many_arguments,
    reason = "private header serializer; one arg per on-disk field reads cleaner than bundling"
)]
fn write_header<W: Write>(
    writer: &mut W,
    layout: &GridLayout,
    nx: u32,
    ny: u32,
    frame_count: u32,
    step_seconds: f32,
    start_unix: i64,
    end_unix: i64,
) -> io::Result<()> {
    let mut buf = [0u8; HEADER_BYTES];
    buf[0..8].copy_from_slice(&MAGIC);
    buf[8..12].copy_from_slice(&VERSION.to_le_bytes());
    buf[12..16].copy_from_slice(&layout.origin_x.to_le_bytes());
    buf[16..20].copy_from_slice(&layout.origin_y.to_le_bytes());
    buf[20..24].copy_from_slice(&layout.step_x.to_le_bytes());
    buf[24..28].copy_from_slice(&layout.step_y.to_le_bytes());
    buf[28..32].copy_from_slice(&nx.to_le_bytes());
    buf[32..36].copy_from_slice(&ny.to_le_bytes());
    buf[36..40].copy_from_slice(&frame_count.to_le_bytes());
    buf[40..44].copy_from_slice(&step_seconds.to_le_bytes());
    buf[44..52].copy_from_slice(&start_unix.to_le_bytes());
    buf[52..60].copy_from_slice(&end_unix.to_le_bytes());
    writer.write_all(&buf)
}

/// Fill the `y` and `cb` planes with one frame's quantized `(u, v)`
/// pixel values. Latitude is flipped on the way in so pixel row 0 is
/// the highest latitude (the AV1 video convention).
fn pack_into_planes(
    y_out: &mut [u16],
    cb_out: &mut [u16],
    rows: &[crate::WeatherRow],
    nx: usize,
    ny: usize,
) {
    debug_assert_eq!(rows.len(), nx * ny, "rows must cover the full grid");
    for j_screen in 0..ny {
        // GRIB column-major has `j = 0` at the southernmost lat;
        // we put the northernmost lat at the top of the video frame.
        let j_src = ny - 1 - j_screen;
        for i in 0..nx {
            let cell_idx = i * ny + j_src;
            let (u, v) = sample_to_uv(&rows[cell_idx].sample);
            let pix = j_screen * nx + i;
            y_out[pix] = quantize(u);
            cb_out[pix] = quantize(v);
        }
    }
}

/// Copy `width × height` u16 pixels (`source` is row-major,
/// `source.len() == width * height`) into a rav1e `Plane<u16>`.
/// rav1e's planes are padded for SIMD alignment, so we route through
/// the plane's `copy_from_raw_u8` helper which handles per-row
/// striding.
fn copy_u16_into_plane(plane: &mut rav1e::prelude::Plane<u16>, source: &[u16], width: usize) {
    let stride_bytes = width * std::mem::size_of::<u16>();
    let raw: &[u8] = bytemuck_le_u16_slice(source);
    plane.copy_from_raw_u8(raw, stride_bytes, std::mem::size_of::<u16>());
}

/// Reinterpret a `&[u16]` as `&[u8]` with little-endian byte order.
/// On every target Rust runs on today host endianness is LE, so this
/// is a zero-copy view; the assert keeps us honest if that ever stops
/// being the case.
fn bytemuck_le_u16_slice(values: &[u16]) -> &[u8] {
    // We don't depend on bytemuck for this single site; the cast is
    // safe because u16 → u8 is layout-compatible and the resulting
    // bytes carry whatever endianness the host uses.
    const _: () = assert!(
        cfg!(target_endian = "little"),
        "wind_av1 encoder assumes little-endian host (file format is little-endian regardless)",
    );
    // SAFETY: `&[u16]` and `&[u8]` are layout-compatible — same
    // alignment is not required for the resulting `&[u8]` (alignment
    // 1), and `&[u16]` has alignment ≥ 1. The resulting slice has
    // exactly `values.len() * 2` bytes covering the same memory.
    #[expect(unsafe_code, reason = "zero-copy u16 → u8 view for rav1e plane fill")]
    unsafe {
        std::slice::from_raw_parts(values.as_ptr().cast::<u8>(), std::mem::size_of_val(values))
    }
}
