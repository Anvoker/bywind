//! Pull GEFS UGRD/VGRD-at-10 m messages from NOAA's AWS Open Data
//! bucket, one GRIB2 stream per ensemble member.
//!
//! Sibling of [`crate::fetch`] (deterministic GFS) — same byte-range
//! trick, same `.idx`-guided pull, same `[start, end)`-stepped frame
//! iteration. The differences live entirely in the URL pattern (GEFS
//! bucket + per-member `gec00 / gepNN` filename component) and the
//! orchestration (one frame stream per member, optional parallel
//! fan-out).
//!
//! ## GEFS path pattern
//!
//! ```text
//! https://noaa-gefs-pds.s3.amazonaws.com/
//!     gefs.YYYYMMDD/HH/atmos/pgrb2sp25/
//!         gec00.tHHz.pgrb2s.0p25.fFFF        (control member)
//!         gep01.tHHz.pgrb2s.0p25.fFFF
//!         ...
//!         gep30.tHHz.pgrb2s.0p25.fFFF        (perturbed members)
//! ```
//!
//! `pgrb2sp25` is the 0.25° selected-fields subset (the one that
//! contains UGRD/VGRD-at-10 m). Cycle / forecast-hour structure
//! matches GFS exactly, so the validated [`crate::fetch::FetchSpec`]
//! transfers verbatim — only the URL builder changes.

use std::ops::ControlFlow;

use chrono::{DateTime, Datelike as _, Timelike as _, Utc};

use crate::fetch::{FetchError, FetchProgress, FetchSpec, FetchStats, fetch_to_grib2_with_url};

/// NOAA's public GEFS S3 bucket mirror (HTTPS so `ureq` can talk to it
/// without an AWS SDK).
const GEFS_BUCKET: &str = "https://noaa-gefs-pds.s3.amazonaws.com";

/// GEFS member identifier. `idx = 0` is the control member (`gec00`);
/// `1..=30` are the perturbed members (`gep01..gep30`). Stored as
/// `u8` because the bucket caps at 30.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GefsMember(pub u8);

impl GefsMember {
    /// Control member (`gec00`).
    pub const CONTROL: Self = Self(0);

    /// `true` iff `idx` is in the valid GEFS range `[0, 30]`.
    pub fn is_valid(self) -> bool {
        self.0 <= 30
    }

    /// Bucket filename prefix: `gec00` for control, `gep{01..30}`
    /// otherwise.
    pub fn filename_prefix(self) -> String {
        if self.0 == 0 {
            "gec00".to_owned()
        } else {
            format!("gep{:02}", self.0)
        }
    }
}

/// Stride-sampled subset of the full GEFS membership down to `k`
/// members.
///
/// Used by the CLI's `--members N` flag to pick a representative
/// subset rather than taking the first N. Stride sampling preserves
/// the spread of perturbations rather than biasing toward the first-
/// defined perturbation directions. The control member (`gec00`) is
/// always included as the first element so the sampled set's mean
/// stays anchored.
///
/// - `k = 1` → control only.
/// - `k = 2..=31` → control + `(k - 1)` stride-sampled perturbed
///   members chosen from `gep01..gep30`.
/// - `k > 31` → clamped to 31 (the full ensemble).
pub fn stride_sample_members(k: usize) -> Vec<GefsMember> {
    if k == 0 {
        return Vec::new();
    }
    let k = k.min(31);
    let mut out = Vec::with_capacity(k);
    out.push(GefsMember::CONTROL);
    if k == 1 {
        return out;
    }
    let perturbed = (k - 1) as f64;
    for i in 0..(k - 1) {
        // Stride across [1, 30]. `i + 0.5` so the samples are
        // distributed around the midpoint of each stride bucket
        // rather than at the start.
        let idx = ((i as f64 + 0.5) * 30.0 / perturbed).floor() as u8 + 1;
        out.push(GefsMember(idx.min(30)));
    }
    out
}

/// Build the GEFS frame URL for the given cycle, forecast hour, and
/// member.
///
/// Visible for unit testing — the loop in [`fetch_member_to_grib2`]
/// uses this through the URL builder it passes to
/// `crate::fetch::fetch_to_grib2_with_url`.
pub fn gefs_url_for_frame(cycle: DateTime<Utc>, forecast_hour: u32, member: GefsMember) -> String {
    let yyyymmdd = format!("{:04}{:02}{:02}", cycle.year(), cycle.month(), cycle.day());
    let hh = format!("{:02}", cycle.hour());
    let prefix = member.filename_prefix();
    format!(
        "{GEFS_BUCKET}/gefs.{yyyymmdd}/{hh}/atmos/pgrb2sp25/\
         {prefix}.t{hh}z.pgrb2s.0p25.f{forecast_hour:03}"
    )
}

/// Pull one ensemble member's frame stream into `out`. Same shape as
/// [`crate::fetch::fetch_to_grib2`] but pinned to the GEFS bucket and
/// the supplied member.
///
/// # Errors
/// Same as [`crate::fetch::fetch_to_grib2`].
pub fn fetch_member_to_grib2<W: std::io::Write>(
    spec: &FetchSpec,
    member: GefsMember,
    out: &mut W,
    progress: impl FnMut(FetchProgress) -> ControlFlow<()>,
) -> Result<FetchStats, FetchError> {
    fetch_to_grib2_with_url(spec, out, progress, move |cycle, fh| {
        gefs_url_for_frame(cycle, fh, member)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_member_filename_is_gec00() {
        assert_eq!(GefsMember::CONTROL.filename_prefix(), "gec00");
        assert_eq!(GefsMember(0).filename_prefix(), "gec00");
    }

    #[test]
    fn perturbed_members_zero_pad_to_two_digits() {
        assert_eq!(GefsMember(1).filename_prefix(), "gep01");
        assert_eq!(GefsMember(9).filename_prefix(), "gep09");
        assert_eq!(GefsMember(10).filename_prefix(), "gep10");
        assert_eq!(GefsMember(30).filename_prefix(), "gep30");
    }

    #[test]
    fn member_validity_is_bounded_at_thirty() {
        assert!(GefsMember(0).is_valid());
        assert!(GefsMember(30).is_valid());
        assert!(!GefsMember(31).is_valid());
    }

    #[test]
    fn stride_sample_zero_returns_empty() {
        assert!(stride_sample_members(0).is_empty());
    }

    #[test]
    fn stride_sample_one_returns_only_control() {
        assert_eq!(stride_sample_members(1), vec![GefsMember::CONTROL]);
    }

    #[test]
    fn stride_sample_default_ten_spans_the_perturbed_range() {
        let s = stride_sample_members(10);
        assert_eq!(s.len(), 10);
        assert_eq!(s[0], GefsMember::CONTROL);
        // Perturbed members should be sorted and span [1, 30].
        let perturbed: Vec<u8> = s[1..].iter().map(|m| m.0).collect();
        assert!(perturbed.iter().all(|&i| (1..=30).contains(&i)));
        assert!(perturbed.windows(2).all(|w| w[0] < w[1]));
        assert!(perturbed.first().copied().unwrap_or(0) <= 5);
        assert!(perturbed.last().copied().unwrap_or(0) >= 25);
    }

    #[test]
    fn stride_sample_clamps_above_thirty_one() {
        assert_eq!(stride_sample_members(40).len(), 31);
    }

    #[test]
    fn url_construction_matches_gefs_bucket_layout() {
        use chrono::TimeZone as _;
        let cycle = Utc.with_ymd_and_hms(2026, 5, 20, 6, 0, 0).unwrap();
        let url = gefs_url_for_frame(cycle, 3, GefsMember(7));
        assert_eq!(
            url,
            "https://noaa-gefs-pds.s3.amazonaws.com/gefs.20260520/06/atmos/pgrb2sp25/\
             gep07.t06z.pgrb2s.0p25.f003",
        );
    }

    #[test]
    fn url_for_control_member_uses_gec00_prefix() {
        use chrono::TimeZone as _;
        let cycle = Utc.with_ymd_and_hms(2026, 1, 1, 0, 0, 0).unwrap();
        let url = gefs_url_for_frame(cycle, 0, GefsMember::CONTROL);
        assert!(url.contains("/gec00.t00z.pgrb2s.0p25.f000"));
    }
}
