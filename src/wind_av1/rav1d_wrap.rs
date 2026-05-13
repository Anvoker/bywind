//! Minimal safe wrapper around `rav1d`'s C-shaped public API.
//!
//! `rav1d`'s only `pub` surface is its `dav1d_*` extern "C" functions
//! (the `rav1d_*` Rust-shaped equivalents are `pub(crate)`), so the
//! wrapper has to deal with raw pointers, `Option<NonNull<_>>`
//! arguments, and `Dav1dResult(c_int)` return codes. This module
//! encapsulates that ugliness into three simple methods —
//! [`Decoder::new`], [`Decoder::feed`], and [`Decoder::next_picture`] —
//! plus a [`DecodedPicture`] that exposes plane bytes via safe slices.
//!
//! The `unsafe` blocks here are kept narrow and individually
//! documented; correctness rests on rav1d upholding the documented
//! dav1d C ABI, which is straightforward push-data-pull-pictures.

#![allow(
    unsafe_code,
    reason = "rav1d's only Rust API is unsafe extern \"C\" functions; this module wraps them safely"
)]

use std::ptr::NonNull;
use std::slice;

use rav1d::include::dav1d::data::Dav1dData;
use rav1d::include::dav1d::dav1d::Dav1dContext;
use rav1d::include::dav1d::dav1d::Dav1dSettings;
use rav1d::include::dav1d::headers::DAV1D_PIXEL_LAYOUT_I444;
use rav1d::include::dav1d::picture::Dav1dPicture;
// The `dav1d_*` extern functions are exported through the (unusually
// named) `src::lib` submodule rather than the crate root. The names
// match the canonical libdav1d C API; see the rav1d source's `lib.rs`
// at `src/lib.rs`.
use rav1d::src::lib::dav1d_close;
use rav1d::src::lib::dav1d_data_create;
use rav1d::src::lib::dav1d_default_settings;
use rav1d::src::lib::dav1d_get_picture;
use rav1d::src::lib::dav1d_open;
use rav1d::src::lib::dav1d_picture_unref;
use rav1d::src::lib::dav1d_send_data;

/// Wrapper around [`Dav1dContext`]: owns the decoder state and closes
/// it on drop.
pub struct Decoder {
    ctx: Option<Dav1dContext>,
}

#[derive(Debug)]
#[non_exhaustive]
pub enum DecoderError {
    /// `dav1d_open` returned non-zero (`Dav1dResult(c_int)` is negative
    /// errno on failure).
    Open(i32),
    /// `dav1d_send_data` returned a hard error (not `-EAGAIN`).
    SendData(i32),
    /// `dav1d_get_picture` returned a hard error (not `-EAGAIN`).
    GetPicture(i32),
    /// `dav1d_data_create` returned a null pointer.
    DataAllocFailed,
    /// Decoded picture was something other than the 10-bit 4:4:4 layout
    /// the encoder produced — encoder / decoder format drift.
    UnexpectedFormat { bpc: i32, layout: u32 },
}

impl std::fmt::Display for DecoderError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Open(c) => write!(f, "dav1d_open failed: errno {c}"),
            Self::SendData(c) => write!(f, "dav1d_send_data failed: errno {c}"),
            Self::GetPicture(c) => write!(f, "dav1d_get_picture failed: errno {c}"),
            Self::DataAllocFailed => f.write_str("dav1d_data_create returned null"),
            Self::UnexpectedFormat { bpc, layout } => {
                write!(
                    f,
                    "decoder produced unexpected pixel format (bpc={bpc}, layout={layout})"
                )
            }
        }
    }
}

impl std::error::Error for DecoderError {}

/// dav1d signals "need to drain pictures before more data fits" via
/// `-EAGAIN`. The value differs per platform (11 on Linux/Windows,
/// 35 on macOS) — sourced from `libc` to stay correct cross-target.
fn eagain_code() -> i32 {
    -libc::EAGAIN
}

impl Decoder {
    /// Open a fresh decoder with `dav1d_default_settings`. The default
    /// thread count picks up the host's available parallelism via
    /// `dav1d_default_settings`, which is fine for an offline one-shot
    /// decode.
    pub fn new() -> Result<Self, DecoderError> {
        // SAFETY: `Dav1dSettings` has no Default impl. A zeroed struct
        // is a valid input here because `dav1d_default_settings`
        // overwrites every field with library-chosen defaults before
        // any of them is observed.
        let mut settings: Dav1dSettings = unsafe { std::mem::zeroed() };
        // SAFETY: `settings` is a valid mutable reference; the function
        // writes through the pointer and returns.
        unsafe { dav1d_default_settings(NonNull::from(&mut settings)) };

        let mut ctx: Option<Dav1dContext> = None;
        // SAFETY: Both pointers refer to valid local stack variables;
        // `dav1d_open` writes the context handle through `c_out` and
        // reads settings through the second pointer.
        let result = unsafe {
            dav1d_open(
                Some(NonNull::from(&mut ctx)),
                Some(NonNull::from(&mut settings)),
            )
        };
        if result.0 != 0 {
            return Err(DecoderError::Open(result.0));
        }
        Ok(Self { ctx })
    }

    /// Submit one AV1 access unit (an IVF frame's payload bytes) to the
    /// decoder.
    ///
    /// Returns:
    /// - `Ok(true)` if the data was accepted in full.
    /// - `Ok(false)` for `-EAGAIN` — the caller must drain at least one
    ///   picture via [`Self::next_picture`] and call `feed` again with
    ///   the same bytes.
    /// - `Err(_)` for any other hard error.
    pub fn feed(&mut self, bytes: &[u8]) -> Result<bool, DecoderError> {
        let mut data = Dav1dData::default();
        // SAFETY: `data` is a valid mutable reference; `dav1d_data_create`
        // either fills the struct with a fresh buffer of the requested
        // size and returns its pointer, or returns null. On null we
        // bail before reading or writing through the pointer.
        let ptr = unsafe { dav1d_data_create(Some(NonNull::from(&mut data)), bytes.len()) };
        if ptr.is_null() {
            return Err(DecoderError::DataAllocFailed);
        }
        // SAFETY: `ptr` is the start of a freshly allocated `bytes.len()`
        // buffer owned by dav1d (lifetime tied to `data`'s ref-counted
        // buffer). Writing exactly `bytes.len()` bytes into it is safe
        // and the source slice is disjoint.
        unsafe {
            std::ptr::copy_nonoverlapping(bytes.as_ptr(), ptr, bytes.len());
        }

        // SAFETY: `self.ctx` came from a successful `dav1d_open` and
        // hasn't been closed (close is in `Drop`). `data` references the
        // buffer dav1d just allocated. On success, dav1d takes over the
        // buffer's reference count via `data`'s internal CArc; on
        // EAGAIN, ownership stays with `data` and the buffer is freed
        // when `data` is dropped at end of scope.
        let result = unsafe { dav1d_send_data(self.ctx, Some(NonNull::from(&mut data))) };
        match result.0 {
            0 => Ok(true),
            x if x == eagain_code() => Ok(false),
            err => Err(DecoderError::SendData(err)),
        }
    }

    /// Try to pull one decoded picture out of the decoder. Returns
    /// `Ok(None)` for `-EAGAIN` (no picture ready; feed more data).
    pub fn next_picture(&mut self) -> Result<Option<DecodedPicture>, DecoderError> {
        let mut pic = Dav1dPicture::default();
        // SAFETY: `self.ctx` is alive (see `feed`). `pic` is a valid
        // mutable reference; dav1d either fills it with plane pointers
        // it now owns (released by `dav1d_picture_unref` on drop) or
        // returns `-EAGAIN` and leaves it default-initialised.
        let result = unsafe { dav1d_get_picture(self.ctx, Some(NonNull::from(&mut pic))) };
        match result.0 {
            0 => Ok(Some(DecodedPicture { pic })),
            x if x == eagain_code() => Ok(None),
            err => Err(DecoderError::GetPicture(err)),
        }
    }
}

impl Drop for Decoder {
    fn drop(&mut self) {
        if self.ctx.is_some() {
            // SAFETY: `self.ctx` came from a successful `dav1d_open`
            // and we haven't already closed it (we're inside `Drop`,
            // called at most once per instance).
            unsafe { dav1d_close(Some(NonNull::from(&mut self.ctx))) };
        }
    }
}

/// One decoded picture. Holds the underlying `Dav1dPicture` and
/// releases it via `dav1d_picture_unref` on drop. All plane access goes
/// through the safe `plane_row` helper rather than letting callers
/// touch the raw pointers.
pub struct DecodedPicture {
    pic: Dav1dPicture,
}

impl DecodedPicture {
    pub fn width(&self) -> usize {
        self.pic.p.w as usize
    }
    pub fn height(&self) -> usize {
        self.pic.p.h as usize
    }

    /// Verify the picture matches the 10-bit 4:4:4 layout the encoder
    /// emits. Round-trip safety hinges on this — if libaom-av1 ever
    /// transcodes our content to a different layout we want a clear
    /// error rather than misinterpreted pixels.
    pub fn check_format(&self) -> Result<(), DecoderError> {
        if self.pic.p.bpc == 10 && self.pic.p.layout == DAV1D_PIXEL_LAYOUT_I444 {
            Ok(())
        } else {
            Err(DecoderError::UnexpectedFormat {
                bpc: self.pic.p.bpc,
                layout: self.pic.p.layout,
            })
        }
    }

    /// Return one row of plane `i` (0=Y, 1=Cb, 2=Cr) as a `u16` slice.
    /// 10-bit pixels are stored little-endian in 16-bit containers;
    /// each row's stride may be larger than `width * 2` for SIMD
    /// alignment, so we read exactly `width` u16s starting at the
    /// plane's per-row stride offset.
    ///
    /// # Panics
    /// Panics if the plane pointer is null (shouldn't happen for a
    /// valid `Dav1dPicture`; debugged against in the wrapper) or if
    /// `row >= height`.
    pub fn plane_row(&self, plane: usize, row: usize) -> &[u16] {
        debug_assert!(plane < 3, "plane index out of range");
        assert!(row < self.height(), "row {row} >= height {}", self.height());
        let stride_bytes = if plane == 0 {
            self.pic.stride[0] as usize
        } else {
            self.pic.stride[1] as usize
        };
        let ptr = self.pic.data[plane]
            .expect("decoded plane pointer is null")
            .as_ptr()
            .cast::<u8>();
        // SAFETY: dav1d allocates a `stride_bytes * height`-byte plane
        // contiguously. The pointer is valid until `dav1d_picture_unref`
        // runs in our `Drop` — borrowing `&self` keeps us alive until
        // then. `row * stride_bytes` is inside that allocation because
        // we asserted `row < height` above.
        let row_ptr = unsafe { ptr.add(row * stride_bytes).cast::<u16>() };
        // SAFETY: `row_ptr` now points at the start of one row of
        // `width` 10-bit pixels, each in a u16 container = `width * 2`
        // bytes, all inside the plane allocation. dav1d aligns each
        // row to at least 2 bytes for 10-bit planes, so the u16 reads
        // are aligned.
        unsafe { slice::from_raw_parts(row_ptr, self.width()) }
    }
}

impl Drop for DecodedPicture {
    fn drop(&mut self) {
        // SAFETY: `self.pic` came from a successful `dav1d_get_picture`
        // and we haven't already unref'd it (we're inside `Drop`).
        unsafe { dav1d_picture_unref(Some(NonNull::from(&mut self.pic))) };
    }
}
