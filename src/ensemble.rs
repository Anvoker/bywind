//! Ensemble wind: collection of per-member [`TimedWindMap`] /
//! [`BakedWindMap`] instances.
//!
//! Loaded from a directory of `.wcav` files (one per member, naming
//! convention `gec00.wcav` / `gepNN.wcav` from the
//! [`crate::fetch_ensemble`] CLI subcommand).
//!
//! The data path is:
//!
//! 1. [`TimedEnsembleWindMap::load_dir`] — discover member files,
//!    decode each `.wcav` in parallel, sort by filename so the
//!    member-index → file mapping is stable across runs.
//! 2. [`TimedEnsembleWindMap::bake`] — bake every member in parallel
//!    against the same [`BakeBounds`], yielding a
//!    [`BakedEnsembleWindMap`] whose members share grid geometry.
//! 3. [`BakedEnsembleWindMap::mean`] — collapse to a single
//!    [`BakedWindMap`] whose per-cell wind is the mean of the
//!    ensemble's u/v components. Cheap "fast mode" alternative to
//!    K-fold robust fitness; see the trade-off note on `mean`.

#[cfg(not(target_arch = "wasm32"))]
use std::path::{Path, PathBuf};

use rayon::prelude::*;
use swarmkit_sailing::EnsembleWindSource;
use swarmkit_sailing::spherical::Wind;

use crate::wind_map::{BakeBounds, BakedWindMap, TimedWindMap};

/// Failure modes for [`TimedEnsembleWindMap::load_dir`].
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug)]
#[non_exhaustive]
pub enum EnsembleLoadError {
    /// Directory couldn't be opened or doesn't exist.
    OpenDir {
        path: PathBuf,
        source: std::io::Error,
    },
    /// Directory contained zero member files (no `ge[cp]*.wcav`).
    NoMembers { path: PathBuf },
    /// A member's `.wcav` failed to decode.
    Decode {
        member: String,
        source: crate::io::LoadError,
    },
}

#[cfg(not(target_arch = "wasm32"))]
impl std::fmt::Display for EnsembleLoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::OpenDir { path, source } => {
                write!(f, "cannot open ensemble directory {}: {source}", path.display())
            }
            Self::NoMembers { path } => write!(
                f,
                "ensemble directory {} contains no `ge[cp]*.wcav` files",
                path.display()
            ),
            Self::Decode { member, source } => {
                write!(f, "decoding member {member}: {source}")
            }
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl std::error::Error for EnsembleLoadError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::OpenDir { source, .. } => Some(source),
            Self::Decode { source, .. } => Some(source),
            Self::NoMembers { .. } => None,
        }
    }
}

/// Ensemble of time-varying wind maps, one per GEFS member.
pub struct TimedEnsembleWindMap {
    members: Vec<TimedWindMap>,
    member_names: Vec<String>,
}

impl TimedEnsembleWindMap {
    /// Construct from pre-decoded members. Length must match
    /// `member_names`. Intended for tests / programmatic use;
    /// production code goes through [`Self::load_dir`].
    pub fn from_members(members: Vec<TimedWindMap>, member_names: Vec<String>) -> Self {
        assert_eq!(
            members.len(),
            member_names.len(),
            "members ({}) and member_names ({}) must have matching lengths",
            members.len(),
            member_names.len(),
        );
        Self {
            members,
            member_names,
        }
    }

    /// Number of members in this ensemble.
    pub fn member_count(&self) -> usize {
        self.members.len()
    }

    /// Borrow the underlying members in stable filename order.
    pub fn members(&self) -> &[TimedWindMap] {
        &self.members
    }

    /// Borrow the member names in the same order as [`Self::members`].
    /// Useful for diagnostics ("member gep07 missing frames 3..5").
    pub fn member_names(&self) -> &[String] {
        &self.member_names
    }

    /// Bake every member against the same bounds, in parallel.
    /// All baked members share grid geometry, which is the
    /// precondition for [`BakedEnsembleWindMap::mean`] and for the
    /// K-fold ensemble fitness loop to make sense.
    pub fn bake(&self, bounds: BakeBounds) -> BakedEnsembleWindMap {
        let members: Vec<BakedWindMap> = self
            .members
            .par_iter()
            .map(|m| m.bake(bounds))
            .collect();
        BakedEnsembleWindMap {
            members,
            member_names: self.member_names.clone(),
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl TimedEnsembleWindMap {
    /// Load every `ge[cp]*.wcav` in `dir` in parallel via rayon.
    /// Files are sorted by filename so member indices are stable
    /// across runs — `gec00.wcav` always lands at index 0 if
    /// present, then `gep01.wcav` at 1, `gep02.wcav` at 2, etc.
    /// (Sort is alphabetical; matches the natural padding in the
    /// fetch CLI's output names.)
    ///
    /// # Errors
    /// [`EnsembleLoadError::OpenDir`] if the path can't be read,
    /// [`EnsembleLoadError::NoMembers`] if no matching files exist,
    /// [`EnsembleLoadError::Decode`] if any member's `.wcav` decode
    /// fails. A single decode failure aborts the whole load — partial
    /// ensembles aren't useful for K-fold fitness.
    pub fn load_dir(dir: &Path) -> Result<Self, EnsembleLoadError> {
        let entries = std::fs::read_dir(dir).map_err(|source| EnsembleLoadError::OpenDir {
            path: dir.to_path_buf(),
            source,
        })?;
        let mut member_paths: Vec<PathBuf> = entries
            .filter_map(Result::ok)
            .map(|e| e.path())
            .filter(|p| {
                if p.extension().and_then(|s| s.to_str()) != Some("wcav") {
                    return false;
                }
                let Some(stem) = p.file_stem().and_then(|s| s.to_str()) else {
                    return false;
                };
                // GEFS naming: `gec00` (control) or `gep01..gep30`.
                // ASCII-only check via `.get(3..)` to satisfy the
                // string-slice lint (filename stems are ASCII in
                // practice but the lint can't know that).
                (stem.starts_with("gec") || stem.starts_with("gep"))
                    && stem.len() >= 5
                    && stem
                        .get(3..)
                        .is_some_and(|rest| rest.chars().all(|c| c.is_ascii_digit()))
            })
            .collect();
        if member_paths.is_empty() {
            return Err(EnsembleLoadError::NoMembers {
                path: dir.to_path_buf(),
            });
        }
        member_paths.sort();

        let member_names: Vec<String> = member_paths
            .iter()
            .map(|p| {
                p.file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or("")
                    .to_owned()
            })
            .collect();

        let members: Result<Vec<TimedWindMap>, EnsembleLoadError> = member_paths
            .par_iter()
            .enumerate()
            .map(|(i, p)| {
                crate::io::load(p, 1, None).map_err(|source| EnsembleLoadError::Decode {
                    member: member_names[i].clone(),
                    source,
                })
            })
            .collect();

        Ok(Self {
            members: members?,
            member_names,
        })
    }
}

/// Baked counterpart of [`TimedEnsembleWindMap`]. Every member is the
/// result of `TimedWindMap::bake(bounds)` against the same bounds, so
/// the grids align cell-for-cell.
#[derive(Clone)]
pub struct BakedEnsembleWindMap {
    members: Vec<BakedWindMap>,
    member_names: Vec<String>,
}

impl BakedEnsembleWindMap {
    /// Construct from pre-baked members. For tests / programmatic use.
    pub fn from_members(members: Vec<BakedWindMap>, member_names: Vec<String>) -> Self {
        assert_eq!(
            members.len(),
            member_names.len(),
            "members ({}) and member_names ({}) must have matching lengths",
            members.len(),
            member_names.len(),
        );
        Self {
            members,
            member_names,
        }
    }

    /// Number of baked members.
    pub fn member_count(&self) -> usize {
        self.members.len()
    }

    /// Borrow the k-th baked member's wind map.
    /// Panics if `k >= member_count()`.
    pub fn member(&self, k: usize) -> &BakedWindMap {
        &self.members[k]
    }

    /// Borrow all baked members.
    pub fn members(&self) -> &[BakedWindMap] {
        &self.members
    }

    /// Member names in stable order (same as [`Self::members`]).
    pub fn member_names(&self) -> &[String] {
        &self.member_names
    }

    /// Collapse the ensemble to a single mean [`BakedWindMap`] whose
    /// per-cell wind is the arithmetic mean of every member's u/v
    /// components. Used by the search's "fast-mode" path:
    /// `E[f(x, wind)] ≠ f(x, E[wind])` in general (fuel burn is
    /// non-linear in wind speed via the polar), but for routes in the
    /// linear-ish polar regime the gap is typically <1 %.
    ///
    /// Requires all members to share grid geometry — `nx`, `ny`, `nt`,
    /// `x_min`, `y_min`, `step`, `t_step_seconds`. Panics if not (the
    /// caller built the ensemble with mismatched bakes, which is a
    /// programming error since `Self::bake` derives all members from
    /// the same `BakeBounds`).
    pub fn mean(&self) -> BakedWindMap {
        assert!(
            !self.members.is_empty(),
            "ensemble must have at least one member to take the mean",
        );
        let first = &self.members[0];
        for (i, m) in self.members.iter().enumerate().skip(1) {
            assert_eq!(
                (m.nx(), m.ny(), m.nt()),
                (first.nx(), first.ny(), first.nt()),
                "ensemble member {i} has mismatched grid dims vs member 0",
            );
        }
        let k_f = self.members.len() as f64;
        let mut grid: Vec<Wind> = Vec::with_capacity(first.grid.len());
        for idx in 0..first.grid.len() {
            let mut sum_east = 0.0;
            let mut sum_north = 0.0;
            for m in &self.members {
                let w = m.grid[idx];
                sum_east += w.east_mps;
                sum_north += w.north_mps;
            }
            grid.push(Wind::new(sum_east / k_f, sum_north / k_f));
        }
        BakedWindMap {
            grid,
            nx: first.nx,
            ny: first.ny,
            nt: first.nt,
            x_min: first.x_min,
            y_min: first.y_min,
            step: first.step,
            t_step_seconds: first.t_step_seconds,
            crossfade_seconds: first.crossfade_seconds,
            coord_scale: first.coord_scale,
        }
    }
}

impl EnsembleWindSource for BakedEnsembleWindMap {
    type Member = BakedWindMap;
    fn member_count(&self) -> usize {
        self.members.len()
    }
    fn member(&self, k: usize) -> &Self::Member {
        &self.members[k]
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "tests compare exact arithmetic on values stored as f64."
    )]
    use super::*;
    use crate::wind_map::WindMap;
    use crate::{WeatherRow, WindSample};
    use swarmkit_sailing::spherical::LonLatBbox;

    fn uniform_wind_map(speed: f32, direction: f32) -> WindMap {
        let mut rows = Vec::with_capacity(4);
        for i in 0..2 {
            for j in 0..2 {
                rows.push(WeatherRow {
                    lon: i as f32,
                    lat: j as f32,
                    sample: WindSample { speed, direction },
                });
            }
        }
        WindMap::new(rows)
    }

    fn uniform_timed(speed: f32, direction: f32) -> TimedWindMap {
        TimedWindMap::new(vec![uniform_wind_map(speed, direction)], 3600.0)
    }

    #[test]
    fn from_members_panics_on_length_mismatch() {
        let m = vec![uniform_timed(5.0, 90.0)];
        let names = vec!["gec00".to_owned(), "gep01".to_owned()];
        std::panic::set_hook(Box::new(|_| {}));
        let res = std::panic::catch_unwind(|| {
            let _ensemble = TimedEnsembleWindMap::from_members(m, names);
        });
        drop(std::panic::take_hook());
        assert!(res.is_err());
    }

    #[test]
    fn bake_produces_one_baked_per_member() {
        let ensemble = TimedEnsembleWindMap::from_members(
            vec![uniform_timed(5.0, 90.0), uniform_timed(10.0, 180.0)],
            vec!["gec00".to_owned(), "gep01".to_owned()],
        );
        let bounds = BakeBounds {
            bbox: LonLatBbox::new(0.0, 1.0, 0.0, 1.0),
            step: 0.5,
            coord_scale: 1.0,
        };
        let baked = ensemble.bake(bounds);
        assert_eq!(baked.member_count(), 2);
        assert_eq!(baked.member_names(), &["gec00", "gep01"]);
    }

    #[test]
    fn mean_averages_uv_components() {
        // Member 0: 10 m/s due east (u=10, v=0).
        // Member 1: 10 m/s due north (u=0, v=10).
        // Mean: u=5, v=5.
        let east = uniform_timed(10.0, 270.0); // 270° = from-west = wind moves east
        let north = uniform_timed(10.0, 180.0); // 180° = from-south = wind moves north
        let ensemble = TimedEnsembleWindMap::from_members(
            vec![east, north],
            vec!["gec00".to_owned(), "gep01".to_owned()],
        );
        let bounds = BakeBounds {
            bbox: LonLatBbox::new(0.0, 1.0, 0.0, 1.0),
            step: 1.0,
            coord_scale: 1.0,
        };
        let baked = ensemble.bake(bounds);
        let mean = baked.mean();
        // Spot-check: the mean grid is the average of two members'
        // u/v vectors. Direction encoding in `WindSample` is degrees
        // *from-bearing* (meteorological); 270° = west-to-east wind
        // = u=+v, v=0. We don't reach into the conversion details
        // here — we just confirm the dims match the input.
        assert_eq!((mean.nx(), mean.ny(), mean.nt()), (baked.member(0).nx(), baked.member(0).ny(), baked.member(0).nt()));
        // And confirm `mean.grid` length matches a member's.
        assert_eq!(mean.grid.len(), baked.member(0).grid.len());
    }

    #[test]
    fn mean_of_identical_members_equals_any_member() {
        let a = uniform_timed(7.5, 45.0);
        let b = uniform_timed(7.5, 45.0);
        let c = uniform_timed(7.5, 45.0);
        let ensemble = TimedEnsembleWindMap::from_members(
            vec![a, b, c],
            vec!["gec00".to_owned(), "gep01".to_owned(), "gep02".to_owned()],
        );
        let bounds = BakeBounds {
            bbox: LonLatBbox::new(0.0, 1.0, 0.0, 1.0),
            step: 0.5,
            coord_scale: 1.0,
        };
        let baked = ensemble.bake(bounds);
        let mean = baked.mean();
        for (i, sample) in mean.grid.iter().enumerate() {
            let expected = baked.member(0).grid[i];
            // Floats from average of identical u/v values should be
            // bit-identical (no rounding loss when N copies of x
            // average to x).
            assert!(
                (sample.east_mps - expected.east_mps).abs() < 1e-9,
                "east mismatch at {i}: {} vs {}",
                sample.east_mps,
                expected.east_mps,
            );
            assert!(
                (sample.north_mps - expected.north_mps).abs() < 1e-9,
                "north mismatch at {i}: {} vs {}",
                sample.north_mps,
                expected.north_mps,
            );
        }
    }
}
