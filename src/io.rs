//! Wind-map loaders dispatched by file extension.
//!
//! Used by `bywind-viz`'s file dialogs and by `bywind-cli`'s `convert` /
//! `search` / `info` subcommands. Centralises GRIB2 / `wind_av1`
//! loader plumbing so consumers don't reimplement the per-format
//! boilerplate.
//!
//! Gated on non-`wasm32` because the `wind_av1` decoder pulls in the
//! rav1d AV1 decoder, which doesn't cross-compile to wasm cleanly.
//! `bywind-viz`'s wasm build has no file loading anyway (no native
//! file dialog), so this gate doesn't lose any consumer.

use std::fs::File;
use std::io::BufReader;
use std::path::{Path, PathBuf};

use crate::{Grib2Bbox, TimedWindMap, grib2, wind_av1};

/// Wind-map file formats supported by `bywind`.
///
/// Detection is by extension only — magic-byte sniffing would catch
/// mis-extensioned files but isn't worth the complexity for tools where
/// the user controls file names.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Format {
    /// GRIB2 (read-only). Extensions `.grib2` / `.grb2` / `.grib`.
    Grib2,
    /// AV1 near-lossless `bywind::wind_av1`. Extension `.wcav`.
    WindAv1,
}

impl Format {
    /// Resolve a format from a file path's extension.
    ///
    /// # Errors
    /// Returns [`LoadError::UnknownExtension`] if the extension is
    /// missing or not a recognised wind-map suffix.
    pub fn from_path(path: &Path) -> Result<Self, LoadError> {
        let ext = path
            .extension()
            .and_then(|e| e.to_str())
            .map(str::to_ascii_lowercase);
        match ext.as_deref() {
            Some("grib2" | "grb2" | "grib") => Ok(Self::Grib2),
            Some("wcav") => Ok(Self::WindAv1),
            _ => Err(LoadError::UnknownExtension(path.to_path_buf())),
        }
    }

    /// Human-friendly format label for log output.
    pub fn name(self) -> &'static str {
        match self {
            Self::Grib2 => "GRIB2",
            Self::WindAv1 => "wind_av1",
        }
    }
}

/// Errors surfaced by [`load`] and the per-format loaders.
#[derive(Debug)]
#[non_exhaustive]
pub enum LoadError {
    /// Path's extension didn't match any supported wind-map format.
    UnknownExtension(PathBuf),
    /// A loader option was supplied that doesn't apply to the chosen
    /// format (e.g. `grib_stride > 1` with a `wind_av1` input).
    /// Surfaced as a hard error so the caller notices the knob is being
    /// silently dropped.
    OptionNotApplicable(&'static str),
    /// Filesystem error opening the input file.
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    /// Decoding the GRIB2 stream failed.
    Grib(grib2::LoadError),
    /// Decoding the AV1 `wind_av1` stream failed.
    WindAv1(wind_av1::DecodeError),
}

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownExtension(p) => write!(
                f,
                "cannot infer wind-map format from extension of {}: \
                 known extensions are .grib2 / .grb2 / .grib / .wcav",
                p.display(),
            ),
            Self::OptionNotApplicable(msg) => f.write_str(msg),
            Self::Io { path, source } => {
                write!(f, "I/O error on {}: {source}", path.display())
            }
            Self::Grib(e) => write!(f, "{e}"),
            Self::WindAv1(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for LoadError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            Self::Grib(e) => Some(e),
            Self::WindAv1(e) => Some(e),
            _ => None,
        }
    }
}

impl From<grib2::LoadError> for LoadError {
    fn from(e: grib2::LoadError) -> Self {
        Self::Grib(e)
    }
}

impl From<wind_av1::DecodeError> for LoadError {
    fn from(e: wind_av1::DecodeError) -> Self {
        Self::WindAv1(e)
    }
}

/// Load a wind map from any supported format, dispatching by file
/// extension.
///
/// `grib_stride` and `grib_bbox` apply only to GRIB2 input; any
/// non-default value with a non-GRIB2 path returns
/// [`LoadError::OptionNotApplicable`].
///
/// # Errors
/// Returns any variant of [`LoadError`]: [`LoadError::UnknownExtension`]
/// for unsupported file types, [`LoadError::OptionNotApplicable`] for
/// GRIB-only options paired with a non-GRIB input, plus the format-
/// specific errors documented on [`load_grib2`] and [`load_wcav`].
pub fn load(
    path: &Path,
    grib_stride: usize,
    grib_bbox: Option<Grib2Bbox>,
) -> Result<TimedWindMap, LoadError> {
    let fmt = Format::from_path(path)?;
    if !matches!(fmt, Format::Grib2) && (grib_stride > 1 || grib_bbox.is_some()) {
        return Err(LoadError::OptionNotApplicable(
            "grib_stride / grib_bbox apply only to GRIB2 input",
        ));
    }
    match fmt {
        Format::Grib2 => load_grib2(path, grib_stride, grib_bbox),
        Format::WindAv1 => load_wcav(path),
    }
}

/// Decode a GRIB2 file.
///
/// `stride` decimates lat / lon (1 = keep every sample); `bbox` clips
/// to a lat / lon window before decoding. Both are passed through to
/// [`TimedWindMap::from_grib2_reader`] unchanged.
///
/// # Errors
/// Returns [`LoadError::Io`] if the file can't be opened, or
/// [`LoadError::Grib`] / [`crate::grib2::LoadError::NoFrames`] from the
/// GRIB2 parser.
pub fn load_grib2(
    path: &Path,
    stride: usize,
    bbox: Option<Grib2Bbox>,
) -> Result<TimedWindMap, LoadError> {
    let file = File::open(path).map_err(|source| LoadError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let reader = BufReader::new(file);
    Ok(TimedWindMap::from_grib2_reader(reader, stride, bbox)?)
}

/// Decode a `bywind::wind_av1` file (AV1 near-lossless).
///
/// # Errors
/// Returns [`LoadError::Io`] if the file can't be opened, or
/// [`LoadError::WindAv1`] for any decoding failure (bad header,
/// rav1d-side error, or truncated IVF stream).
pub fn load_wcav(path: &Path) -> Result<TimedWindMap, LoadError> {
    let file = File::open(path).map_err(|source| LoadError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let reader = BufReader::new(file);
    Ok(wind_av1::decode(reader)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_from_path_detects_known_extensions() {
        assert_eq!(
            Format::from_path(Path::new("a.grib2")).unwrap(),
            Format::Grib2
        );
        assert_eq!(
            Format::from_path(Path::new("a.GRB2")).unwrap(),
            Format::Grib2
        );
        assert_eq!(
            Format::from_path(Path::new("a.grib")).unwrap(),
            Format::Grib2
        );
        assert_eq!(
            Format::from_path(Path::new("a.wcav")).unwrap(),
            Format::WindAv1
        );
    }

    #[test]
    fn format_from_path_rejects_unknown_extensions() {
        assert!(matches!(
            Format::from_path(Path::new("a.json")),
            Err(LoadError::UnknownExtension(_)),
        ));
        assert!(matches!(
            Format::from_path(Path::new("a.csv")),
            Err(LoadError::UnknownExtension(_)),
        ));
        assert!(matches!(
            Format::from_path(Path::new("noext")),
            Err(LoadError::UnknownExtension(_)),
        ));
    }

    #[test]
    fn format_name_labels_match_module_doc() {
        assert_eq!(Format::Grib2.name(), "GRIB2");
        assert_eq!(Format::WindAv1.name(), "wind_av1");
    }

    #[test]
    fn load_rejects_grib_options_with_wcav_input() {
        // Note: `unwrap_err` would require `TimedWindMap: Debug` (which it
        // isn't); pattern-match instead so the test stays self-contained.
        let bbox = Grib2Bbox {
            lat_min: 0.0,
            lon_min: 0.0,
            lat_max: 1.0,
            lon_max: 1.0,
        };
        assert!(matches!(
            load(Path::new("a.wcav"), 1, Some(bbox)),
            Err(LoadError::OptionNotApplicable(_)),
        ));
        assert!(matches!(
            load(Path::new("a.wcav"), 2, None),
            Err(LoadError::OptionNotApplicable(_)),
        ));
    }
}
