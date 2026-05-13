//! Persistent representation of a single search solution.
//!
//! Captures the displayed gbest path plus the metadata needed to
//! reproduce or reason about the search that produced it; the wind map
//! is saved separately because it is large and reused across many
//! solutions.

use swarmkit::{Evolution, Particle};
use swarmkit_sailing::{Floats, Path, PathXY, Time, Topology};

use crate::config::topology_serde;
use crate::route::{RouteEvolution, WaypointCount};
use crate::waypoint_match;

/// Serializable snapshot of a search result.
///
/// Format is whatever the caller chooses (the GUI uses pretty-printed
/// JSON via `serde_json`); bywind only owns the schema and the round-
/// trip into a [`RouteEvolution`].
///
/// The fields after `iter_time` were added later — `seed` and `topology`
/// for reproducibility, and the three `path_kick_*` fields once those
/// became tunable. All carry `#[serde(default)]` so older saved files
/// (which only have the original 11 fields) still parse, picking up
/// `SearchConfig::default()`-equivalent values for the missing ones.
#[derive(serde::Serialize, serde::Deserialize, Clone, Debug)]
pub struct SavedSolution {
    /// Waypoint count of the saved path. Must match a [`WaypointCount`]
    /// variant when reloading.
    pub n: usize,
    /// Longitude (degrees) per waypoint; length `n`.
    pub xs: Vec<f64>,
    /// Latitude (degrees) per waypoint; length `n`.
    pub ys: Vec<f64>,
    /// Cumulative arrival time at each waypoint (seconds); length `n`.
    pub ts: Vec<f64>,
    /// Final gbest fitness for this path. Higher is better.
    pub best_fit: f64,
    /// Time weight that produced this solution.
    pub time_weight: f64,
    /// Fuel weight that produced this solution.
    pub fuel_weight: f64,
    /// Outer-PSO particle count at search time.
    pub particles_space: usize,
    /// Inner time-PSO particle count at search time.
    pub particles_time: usize,
    /// Outer-PSO iteration count at search time.
    pub iter_space: usize,
    /// Inner time-PSO iteration count at search time.
    pub iter_time: usize,
    /// RNG seed used when this solution was produced. `None` means the
    /// run drew fresh OS entropy and isn't reproducible by seed alone.
    #[serde(default)]
    pub seed: Option<u64>,
    /// Outer-PSO topology used. Default = `Topology::default()` (gbest).
    #[serde(default, with = "topology_serde")]
    pub topology: Topology,
    /// Path-kick mover knobs. Default to `SearchSettings::default()`
    /// equivalents when missing from older saved files.
    #[serde(default = "default_path_kick_probability")]
    pub path_kick_probability: f64,
    #[serde(default = "default_path_kick_gamma_0_fraction")]
    pub path_kick_gamma_0_fraction: f64,
    #[serde(default = "default_path_kick_gamma_min_fraction")]
    pub path_kick_gamma_min_fraction: f64,
}

fn default_path_kick_probability() -> f64 {
    crate::config::SearchConfig::default().path_kick_probability
}
fn default_path_kick_gamma_0_fraction() -> f64 {
    crate::config::SearchConfig::default().path_kick_gamma_0_fraction
}
fn default_path_kick_gamma_min_fraction() -> f64 {
    crate::config::SearchConfig::default().path_kick_gamma_min_fraction
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Older saved-solution JSON (pre-`seed`/`topology`/`path_kick_*`) must
    /// still parse, with the missing fields filled from `SearchConfig`'s
    /// defaults so reload doesn't crash on files produced before the schema
    /// extension.
    #[test]
    fn old_format_round_trips_with_defaults() {
        let json = r#"{
            "n": 5,
            "xs": [0.0, 1.0, 2.0, 3.0, 4.0],
            "ys": [0.0, 0.5, 1.0, 1.5, 2.0],
            "ts": [0.0, 100.0, 200.0, 300.0, 400.0],
            "best_fit": -1234.5,
            "time_weight": 1.0,
            "fuel_weight": 10.0,
            "particles_space": 40,
            "particles_time": 40,
            "iter_space": 40,
            "iter_time": 30
        }"#;
        let saved: SavedSolution = serde_json::from_str(json).expect("old format must parse");
        assert_eq!(saved.n, 5);
        assert_eq!(saved.seed, None);
        assert_eq!(saved.topology, Topology::default());
        let cfg_default = crate::config::SearchConfig::default();
        assert!((saved.path_kick_probability - cfg_default.path_kick_probability).abs() < 1e-12);
        assert!(
            (saved.path_kick_gamma_0_fraction - cfg_default.path_kick_gamma_0_fraction).abs()
                < 1e-12,
        );
        assert!(
            (saved.path_kick_gamma_min_fraction - cfg_default.path_kick_gamma_min_fraction).abs()
                < 1e-12,
        );
    }
}

/// Failure modes when reconstructing a search result from a
/// deserialized [`SavedSolution`].
///
/// Distinct from JSON parse errors, which surface as `serde_json::Error`
/// (or whatever the caller's chosen format) at the caller's I/O layer.
#[derive(Debug)]
#[non_exhaustive]
pub enum LoadError {
    /// `n` doesn't match any compile-time `WaypointCount` variant.
    UnsupportedWaypointCount(usize),
    /// `xs`/`ys`/`ts` slice lengths don't all equal `n`.
    LengthMismatch {
        n: usize,
        xs: usize,
        ys: usize,
        ts: usize,
    },
    /// `try_from` failed converting the slices to fixed-size arrays.
    /// Unreachable after the length check, but surfaced rather than
    /// panicked just in case.
    PathConversion(usize),
}

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnsupportedWaypointCount(n) => {
                write!(f, "Unsupported waypoint count {n} in saved solution")
            }
            Self::LengthMismatch { n, xs, ys, ts } => write!(
                f,
                "Saved solution arrays do not match n={n}: xs={xs}, ys={ys}, ts={ts}",
            ),
            Self::PathConversion(n) => write!(
                f,
                "Internal error converting saved arrays of length {n} to fixed-size path",
            ),
        }
    }
}

impl std::error::Error for LoadError {}

impl SavedSolution {
    /// Validate `self` and assemble a single-iteration [`RouteEvolution`]
    /// containing the saved gbest path. Pure: no I/O, no global state.
    ///
    /// # Errors
    /// Returns [`LoadError::UnsupportedWaypointCount`] if `n` isn't a
    /// known [`WaypointCount`] variant; [`LoadError::LengthMismatch`] if
    /// `xs`/`ys`/`ts` lengths disagree with `n`; or
    /// [`LoadError::PathConversion`] if the slice-to-array conversion
    /// fails (unreachable after the length check, surfaced rather than
    /// panicked just in case).
    pub fn to_route_evolution(&self) -> Result<(WaypointCount, RouteEvolution), LoadError> {
        let wc =
            WaypointCount::from_usize(self.n).ok_or(LoadError::UnsupportedWaypointCount(self.n))?;

        if self.xs.len() != self.n || self.ys.len() != self.n || self.ts.len() != self.n {
            return Err(LoadError::LengthMismatch {
                n: self.n,
                xs: self.xs.len(),
                ys: self.ys.len(),
                ts: self.ts.len(),
            });
        }

        // Inside the const-generic branch the size checks above are already
        // guaranteed (self.{xs,ys,ts}.len() == self.n == N), but try_from
        // is the only way to get a `[f64; N]` from a slice. The Err branch
        // is unreachable for well-formed data — surface it just in case.
        let route_evolution = waypoint_match!(wc, N, wrap, {
            let (Ok(xs_arr), Ok(ys_arr), Ok(ts_arr)) = (
                <[f64; N]>::try_from(self.xs.as_slice()),
                <[f64; N]>::try_from(self.ys.as_slice()),
                <[f64; N]>::try_from(self.ts.as_slice()),
            ) else {
                return Err(LoadError::PathConversion(self.n));
            };
            let path = Path::<N> {
                xy: PathXY(Floats(xs_arr), Floats(ys_arr)),
                t: Time(Floats(ts_arr)),
            };
            let particle = Particle::<Path<N>> {
                pos: path,
                vel: Path::default(),
                fit: self.best_fit,
                best_pos: path,
                best_fit: self.best_fit,
            };
            wrap(Evolution::from_frames(vec![vec![particle]]))
        });

        Ok((wc, route_evolution))
    }
}
