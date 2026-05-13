//! Tiny IVF muxer + demuxer. IVF is a lightweight container for AV1
//! bitstreams — 32-byte file header followed by length-prefixed
//! frames. We use it as our on-disk container because rav1e emits one
//! AV1 OBU packet per `send_frame`, and wrapping each packet in a
//! 12-byte IVF frame header gives us self-delimiting frame boundaries
//! for the rav1d decoder without inventing a custom layout.
//!
//! Spec: <https://wiki.multimedia.cx/index.php/IVF>
//!
//! ```text
//! File header (32 bytes):
//!   00..04  signature       "DKIF"
//!   04..06  version         u16 LE (currently 0)
//!   06..08  header length   u16 LE (32)
//!   08..12  codec fourcc    "AV01" for AV1
//!   12..14  width           u16 LE
//!   14..16  height          u16 LE
//!   16..20  framerate num   u32 LE
//!   20..24  framerate den   u32 LE
//!   24..28  frame count     u32 LE
//!   28..32  reserved        4 bytes
//!
//! Per-frame header (12 bytes):
//!   00..04  frame size      u32 LE
//!   04..12  timestamp       u64 LE
//! ```

use std::io::{self, Read, Write};

/// Magic prefix at the start of every IVF file.
pub const FILE_MAGIC: [u8; 4] = *b"DKIF";
/// IVF file header length.
pub const FILE_HEADER_BYTES: usize = 32;
/// Per-frame header length.
pub const FRAME_HEADER_BYTES: usize = 12;
/// `FourCC` marker for AV1 video streams in the IVF file header.
pub const AV1_FOURCC: [u8; 4] = *b"AV01";

/// Write the 32-byte IVF file header. `frame_count` is the number of
/// frames that will follow; some readers ignore the value but a few
/// (e.g. ffmpeg's `-f ivf` decoder) honour it for progress reporting,
/// so we set it honestly.
pub fn write_file_header<W: Write>(
    writer: &mut W,
    width: u16,
    height: u16,
    fps_num: u32,
    fps_den: u32,
    frame_count: u32,
) -> io::Result<()> {
    let mut buf = [0u8; FILE_HEADER_BYTES];
    buf[0..4].copy_from_slice(&FILE_MAGIC);
    // bytes 4..6 = version (currently 0), already zero.
    buf[6..8].copy_from_slice(&(FILE_HEADER_BYTES as u16).to_le_bytes());
    buf[8..12].copy_from_slice(&AV1_FOURCC);
    buf[12..14].copy_from_slice(&width.to_le_bytes());
    buf[14..16].copy_from_slice(&height.to_le_bytes());
    buf[16..20].copy_from_slice(&fps_num.to_le_bytes());
    buf[20..24].copy_from_slice(&fps_den.to_le_bytes());
    buf[24..28].copy_from_slice(&frame_count.to_le_bytes());
    // bytes 28..32 reserved, already zero.
    writer.write_all(&buf)
}

/// Write a 12-byte per-frame header followed by `frame_bytes`. `pts`
/// (presentation timestamp) is opaque to our pipeline; we tick it
/// frame-by-frame so any IVF tool that displays timestamps shows a
/// sensible sequence.
pub fn write_frame<W: Write>(writer: &mut W, frame_bytes: &[u8], pts: u64) -> io::Result<()> {
    let mut header = [0u8; FRAME_HEADER_BYTES];
    let size = u32::try_from(frame_bytes.len())
        .ok()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "IVF frame > 4 GiB"))?;
    header[0..4].copy_from_slice(&size.to_le_bytes());
    header[4..12].copy_from_slice(&pts.to_le_bytes());
    writer.write_all(&header)?;
    writer.write_all(frame_bytes)
}

/// Streaming IVF reader. Pulls one AV1 access unit at a time off the
/// underlying reader. Stops at EOF (signalled by `read_frame` returning
/// `Ok(None)`); short reads or a malformed magic produce
/// [`io::ErrorKind::InvalidData`].
pub struct IvfReader<R> {
    reader: R,
}

impl<R: Read> IvfReader<R> {
    /// Consume and verify the IVF file header, leaving the reader
    /// positioned at the first per-frame header.
    pub fn open(mut reader: R) -> io::Result<Self> {
        let mut buf = [0u8; FILE_HEADER_BYTES];
        reader.read_exact(&mut buf)?;
        if buf[0..4] != FILE_MAGIC {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("IVF: missing DKIF magic (got {:02x?})", &buf[0..4],),
            ));
        }
        Ok(Self { reader })
    }

    /// Read the next frame's bytes (decoded body, no IVF headers).
    /// Returns `Ok(None)` on clean EOF.
    pub fn read_frame(&mut self) -> io::Result<Option<Vec<u8>>> {
        let mut header = [0u8; FRAME_HEADER_BYTES];
        if self.reader.read(&mut header[..1])? == 0 {
            // Clean EOF before any frame header byte — the file simply
            // had no further frames after the last one we returned.
            return Ok(None);
        }
        self.reader.read_exact(&mut header[1..])?;
        let size = u32::from_le_bytes([header[0], header[1], header[2], header[3]]) as usize;
        let mut frame = vec![0u8; size];
        self.reader.read_exact(&mut frame)?;
        Ok(Some(frame))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    /// Construct a synthetic IVF stream with two frames carrying the
    /// supplied payloads, then verify `IvfReader` walks them in order
    /// and stops cleanly at EOF.
    #[test]
    fn round_trip_two_frames() {
        let mut buf = Vec::new();
        buf.extend_from_slice(&FILE_MAGIC); // signature
        buf.extend_from_slice(&0u16.to_le_bytes()); // version
        buf.extend_from_slice(&32u16.to_le_bytes()); // header length
        buf.extend_from_slice(b"AV01");
        buf.extend_from_slice(&64u16.to_le_bytes()); // width
        buf.extend_from_slice(&64u16.to_le_bytes()); // height
        buf.extend_from_slice(&1u32.to_le_bytes()); // fps num
        buf.extend_from_slice(&1u32.to_le_bytes()); // fps den
        buf.extend_from_slice(&2u32.to_le_bytes()); // frame count
        buf.extend_from_slice(&0u32.to_le_bytes()); // reserved
        for (i, body) in [b"FRAME-ONE".as_slice(), b"BODY-2"].iter().enumerate() {
            buf.extend_from_slice(&(body.len() as u32).to_le_bytes());
            buf.extend_from_slice(&(i as u64).to_le_bytes());
            buf.extend_from_slice(body);
        }
        let mut r = IvfReader::open(Cursor::new(buf)).expect("open");
        assert_eq!(
            r.read_frame().unwrap().as_deref(),
            Some(b"FRAME-ONE".as_slice())
        );
        assert_eq!(
            r.read_frame().unwrap().as_deref(),
            Some(b"BODY-2".as_slice())
        );
        assert!(
            r.read_frame().unwrap().is_none(),
            "clean EOF after last frame"
        );
    }

    #[test]
    fn rejects_bad_magic() {
        let mut buf = vec![0u8; FILE_HEADER_BYTES];
        buf[0..4].copy_from_slice(b"NOPE");
        let err = IvfReader::open(Cursor::new(buf)).err().expect("bad magic");
        assert_eq!(err.kind(), io::ErrorKind::InvalidData);
    }
}
