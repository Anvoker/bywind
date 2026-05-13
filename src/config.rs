//! Parameter-bag configs shared between consumers.
//!
//! Canonical schema for GUI panels, CLI flags, and test fixtures;
//! consumer-specific wrappers (clap overrides, eframe persistence)
//! live in the consumer crates.

use swarmkit_sailing::{Boat, DEFAULT_STEP_DISTANCE_FRACTION, SearchSettings, Topology};

use crate::route::WaypointCount;

/// Serde adapter for [`Topology`].
///
/// `Topology` lives in `swarmkit-sailing` without a serde dep; this
/// round-trips through its `FromStr` / `Display`. Use as
/// `#[serde(with = "topology_serde")]` on a `Topology` field, or
/// `topology_serde::option` on an `Option<Topology>` override.
pub mod topology_serde {
    use serde::{Deserialize as _, Deserializer, Serializer};
    use std::str::FromStr as _;
    use swarmkit_sailing::Topology;

    /// # Errors
    /// Forwards any error from `s`.
    pub fn serialize<S: Serializer>(t: &Topology, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(t.as_str())
    }

    /// # Errors
    /// `D::Error::custom` on unknown variant; otherwise forwards from `d`.
    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Topology, D::Error> {
        let s = String::deserialize(d)?;
        Topology::from_str(&s).map_err(serde::de::Error::custom)
    }

    pub mod option {
        use serde::{Deserialize as _, Deserializer, Serializer};
        use std::str::FromStr as _;
        use swarmkit_sailing::Topology;

        /// # Errors
        /// Forwards any error from `s`.
        pub fn serialize<S: Serializer>(t: &Option<Topology>, s: S) -> Result<S::Ok, S::Error> {
            match t {
                Some(t) => s.serialize_some(t.as_str()),
                None => s.serialize_none(),
            }
        }

        /// # Errors
        /// `D::Error::custom` on unknown variant; otherwise forwards from `d`.
        pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Option<Topology>, D::Error> {
            let opt: Option<String> = Option::deserialize(d)?;
            opt.map(|s| Topology::from_str(&s).map_err(serde::de::Error::custom))
                .transpose()
        }
    }
}

/// One hour. Re-exported because consumers (GUI panel, CLI flags) need
/// it as a default for new maps.
pub const DEFAULT_FRAME_STEP_SECONDS: f32 = 3600.0;

const DEFAULT_FRAME_COUNT: usize = 12;
const DEFAULT_GEN_SIZE: f32 = 10.0;
const DEFAULT_GEN_DENSITY: f32 = 0.5;
const DEFAULT_GEN_SPEED_MIN: f32 = 0.0;
const DEFAULT_GEN_SPEED_MAX: f32 = 149.0;

// `land_weight = 50.0`: one metre over land costs 50 seconds of travel
// time, ~500× what a continent-crossing shortcut (~0.1 s/m saved) could
// recover. Strong bias against any coastline-grazing waypoint.
const DEFAULT_TIME_WEIGHT: f64 = 1.0;
const DEFAULT_FUEL_WEIGHT: f64 = 10.0;
const DEFAULT_LAND_WEIGHT: f64 = 50.0;

const DEFAULT_PARTICLES_SPACE: usize = 80;
const DEFAULT_PARTICLES_TIME: usize = 40;
const DEFAULT_ITER_SPACE: usize = 60;
const DEFAULT_ITER_TIME: usize = 30;

const DEFAULT_INERTIA: f64 = 0.33;
const DEFAULT_COGNITIVE_COEFF: f64 = 2.0;
const DEFAULT_SOCIAL_COEFF: f64 = 2.0;

const DEFAULT_PATH_KICK_PROBABILITY: f64 = 0.1;
const DEFAULT_PATH_KICK_GAMMA_0_FRACTION: f64 = 0.05;
const DEFAULT_PATH_KICK_GAMMA_MIN_FRACTION: f64 = 0.005;

const DEFAULT_BAKE_STEP_DEG: f64 = 0.25;
const DEFAULT_SDF_RESOLUTION_DEG: f64 = 0.5;
const DEFAULT_RANGE_K: usize = 8;
const DEFAULT_K_MCR: usize = 8;

// Mid-size marine diesel, MCR = 1 MW. Matches `Boat::default()`.
const DEFAULT_BOAT_MCR_KW: f64 = 1000.0;
const DEFAULT_BOAT_K: f64 = 4000.0;
const DEFAULT_BOAT_POLAR_C: f64 = 1.5;
const DEFAULT_BOAT_POLAR_SIN_POWER: f64 = 1.0;
const DEFAULT_BOAT_FUEL_A: f64 = 0.0875;
const DEFAULT_BOAT_FUEL_B: f64 = -0.0555;
const DEFAULT_BOAT_FUEL_C: f64 = 0.0347;

/// Inputs to `TimedWindMap::generate_random`.
#[derive(serde::Deserialize, serde::Serialize, Clone, Copy, Debug)]
#[serde(default)]
pub struct GenerateConfig {
    pub size_x: f32,
    pub size_y: f32,
    pub density: f32,
    pub frame_count: usize,
    pub step_seconds: f32,
    pub speed_min: f32,
    pub speed_max: f32,
}

impl Default for GenerateConfig {
    fn default() -> Self {
        Self {
            size_x: DEFAULT_GEN_SIZE,
            size_y: DEFAULT_GEN_SIZE,
            density: DEFAULT_GEN_DENSITY,
            frame_count: DEFAULT_FRAME_COUNT,
            step_seconds: DEFAULT_FRAME_STEP_SECONDS,
            speed_min: DEFAULT_GEN_SPEED_MIN,
            speed_max: DEFAULT_GEN_SPEED_MAX,
        }
    }
}

/// Inputs to the sailing-search PSO.
#[derive(serde::Deserialize, serde::Serialize, Clone, Copy, Debug)]
#[serde(default)]
pub struct SearchConfig {
    /// How many waypoints the path has (endpoints included).
    pub waypoint_count: WaypointCount,

    /// `fitness = -(time*time_weight + fuel*fuel_weight + land_metres*land_weight)`.
    /// `land_weight = 0` disables the soft landmass-avoidance penalty.
    pub time_weight: f64,
    /// See [`Self::time_weight`].
    pub fuel_weight: f64,
    /// See [`Self::time_weight`].
    pub land_weight: f64,

    /// Outer-PSO particle count (route shape).
    pub particles_space: usize,
    /// Inner time-PSO particle count (per-waypoint timing).
    pub particles_time: usize,
    /// Outer-PSO iteration count.
    pub iter_space: usize,
    /// Inner time-PSO iteration count.
    pub iter_time: usize,

    /// `inertia` weights prior velocity; `cognitive_coeff` pulls toward
    /// pbest; `social_coeff` toward gbest. Shared by outer (space) and
    /// inner (time) PSOs.
    pub inertia: f64,
    /// See [`Self::inertia`].
    pub cognitive_coeff: f64,
    /// See [`Self::inertia`].
    pub social_coeff: f64,

    /// Path-shape kick lets the search jump into tacking topologies
    /// that gradient-style PSO can't reach. `path_kick_probability` is
    /// per-particle, per-iteration; the two gammas are initial and
    /// floor magnitudes as fractions of the route's straight-line
    /// length, cosine-decayed across iterations.
    pub path_kick_probability: f64,
    /// See [`Self::path_kick_probability`].
    pub path_kick_gamma_0_fraction: f64,
    /// See [`Self::path_kick_probability`].
    pub path_kick_gamma_min_fraction: f64,

    /// `None` (default) draws fresh OS entropy; `Some(s)` makes the run
    /// reproducible.
    pub seed: Option<u64>,

    /// Wind / landmass integration substep, as a fraction of the route
    /// bbox diagonal. Default 0.01.
    pub step_distance_fraction: f64,

    /// Wind-grid cell size (degrees) when baking the search grid.
    /// Default 0.25°. Grown automatically if the map would overflow
    /// the per-axis cell cap.
    pub bake_step_deg: f64,

    /// Landmass SDF cell size (degrees). Default 0.5°. Each distinct
    /// value pays a one-shot ~sub-second build.
    pub sdf_resolution_deg: f64,

    /// Inner time-PSO lookup-table samples along the departure-time
    /// axis. Default 8. Must be ≥ 2.
    pub range_k: usize,

    /// Inner time-PSO lookup-table samples along the `mcr_01` throttle
    /// axis. Default 8. Must be ≥ 2.
    pub k_mcr: usize,

    /// Round-trips through TOML / JSON as the lowercase variant name.
    #[serde(with = "topology_serde")]
    pub topology: Topology,
}

impl Default for SearchConfig {
    fn default() -> Self {
        Self {
            waypoint_count: WaypointCount::default(),
            time_weight: DEFAULT_TIME_WEIGHT,
            fuel_weight: DEFAULT_FUEL_WEIGHT,
            land_weight: DEFAULT_LAND_WEIGHT,
            particles_space: DEFAULT_PARTICLES_SPACE,
            particles_time: DEFAULT_PARTICLES_TIME,
            iter_space: DEFAULT_ITER_SPACE,
            iter_time: DEFAULT_ITER_TIME,
            inertia: DEFAULT_INERTIA,
            cognitive_coeff: DEFAULT_COGNITIVE_COEFF,
            social_coeff: DEFAULT_SOCIAL_COEFF,
            path_kick_probability: DEFAULT_PATH_KICK_PROBABILITY,
            path_kick_gamma_0_fraction: DEFAULT_PATH_KICK_GAMMA_0_FRACTION,
            path_kick_gamma_min_fraction: DEFAULT_PATH_KICK_GAMMA_MIN_FRACTION,
            seed: None,
            step_distance_fraction: DEFAULT_STEP_DISTANCE_FRACTION,
            bake_step_deg: DEFAULT_BAKE_STEP_DEG,
            sdf_resolution_deg: DEFAULT_SDF_RESOLUTION_DEG,
            range_k: DEFAULT_RANGE_K,
            k_mcr: DEFAULT_K_MCR,
            topology: Topology::default(),
        }
    }
}

impl SearchConfig {
    pub fn to_search_settings(&self) -> SearchSettings {
        SearchSettings {
            particle_count_space: self.particles_space,
            particle_count_time: self.particles_time,
            max_iteration_space: self.iter_space,
            max_iteration_time: self.iter_time,
            inertia: self.inertia,
            cognitive_coeff: self.cognitive_coeff,
            social_coeff: self.social_coeff,
            path_kick_probability: self.path_kick_probability,
            path_kick_gamma_0_fraction: self.path_kick_gamma_0_fraction,
            path_kick_gamma_min_fraction: self.path_kick_gamma_min_fraction,
            seed: self.seed,
            range_k: self.range_k,
            k_mcr: self.k_mcr,
            topology: self.topology,
            ..SearchSettings::default()
        }
    }

    /// Reject configs that would crash the PSO or produce nonsense
    /// output. Cheap; a 30-second search that crashes on iter 0
    /// because `particles_space = 0` is strictly worse than a 1 ms
    /// reject at submit time.
    ///
    /// # Errors
    /// Returns the first field that violates a rule.
    pub fn validate(&self) -> Result<(), ValidationError> {
        for (field, w) in [
            ("time_weight", self.time_weight),
            ("fuel_weight", self.fuel_weight),
            ("land_weight", self.land_weight),
        ] {
            ValidationError::require_finite(field, w)?;
            ValidationError::require_non_negative(field, w)?;
        }
        for (field, n) in [
            ("particles_space", self.particles_space),
            ("particles_time", self.particles_time),
            ("iter_space", self.iter_space),
            ("iter_time", self.iter_time),
        ] {
            ValidationError::require_at_least_one(field, n)?;
        }
        for (field, v) in [
            ("inertia", self.inertia),
            ("cognitive_coeff", self.cognitive_coeff),
            ("social_coeff", self.social_coeff),
        ] {
            ValidationError::require_finite(field, v)?;
        }
        // Out-of-range path-kick values either crash deep inside
        // swarmkit or silently rescale into nonsense — reject up front.
        ValidationError::require_finite("path_kick_probability", self.path_kick_probability)?;
        ValidationError::require_in_unit_interval(
            "path_kick_probability",
            self.path_kick_probability,
        )?;
        for (field, v) in [
            (
                "path_kick_gamma_0_fraction",
                self.path_kick_gamma_0_fraction,
            ),
            (
                "path_kick_gamma_min_fraction",
                self.path_kick_gamma_min_fraction,
            ),
        ] {
            ValidationError::require_finite(field, v)?;
            ValidationError::require_non_negative(field, v)?;
        }
        if self.path_kick_gamma_min_fraction > self.path_kick_gamma_0_fraction {
            return Err(ValidationError {
                field: "path_kick_gamma_min_fraction",
                message: format!(
                    "must be ≤ path_kick_gamma_0_fraction (got {} > {})",
                    self.path_kick_gamma_min_fraction, self.path_kick_gamma_0_fraction,
                ),
            });
        }
        // Tier-1 precision knobs. Bounds match the Advanced Settings
        // UI clamps; tighter-than-`> 0` so a `1e-300` typo can't blow
        // up the substep loop or the SDF allocation.
        for (field, v) in [
            ("step_distance_fraction", self.step_distance_fraction),
            ("bake_step_deg", self.bake_step_deg),
            ("sdf_resolution_deg", self.sdf_resolution_deg),
        ] {
            ValidationError::require_finite(field, v)?;
            ValidationError::require_positive(field, v)?;
        }
        if !(1e-6..=1.0).contains(&self.step_distance_fraction) {
            return Err(ValidationError {
                field: "step_distance_fraction",
                message: format!(
                    "must be in [1e-6, 1.0], got {}",
                    self.step_distance_fraction,
                ),
            });
        }
        if !(1e-3..=10.0).contains(&self.bake_step_deg) {
            return Err(ValidationError {
                field: "bake_step_deg",
                message: format!("must be in [1e-3, 10.0], got {}", self.bake_step_deg),
            });
        }
        // 0.001° here would build a 360k × 180k cell mask (64 GiB).
        if !(0.05..=5.0).contains(&self.sdf_resolution_deg) {
            return Err(ValidationError {
                field: "sdf_resolution_deg",
                message: format!("must be in [0.05, 5.0], got {}", self.sdf_resolution_deg,),
            });
        }
        // 256 × 256 × f64 × 60 segs × 40 particles ≈ 1.25 GiB — the
        // ceiling where the worker still boots.
        for (field, n) in [("range_k", self.range_k), ("k_mcr", self.k_mcr)] {
            if !(2..=256).contains(&n) {
                return Err(ValidationError {
                    field,
                    message: format!("must be in [2, 256], got {n}"),
                });
            }
        }
        Ok(())
    }
}

/// Mirrors `swarmkit_sailing::Boat`'s fields except `mcr_kw` is in kW
/// (vs. watts in the swarmkit type — converted in [`Self::to_boat`]).
#[derive(serde::Deserialize, serde::Serialize, Clone, Copy, Debug)]
#[serde(default)]
pub struct BoatConfig {
    /// Maximum continuous engine rating (kW).
    pub mcr_kw: f64,
    /// Hydrodynamic drag coefficient in `P = 0.5 · k · v³`.
    pub k: f64,
    /// Polar curve scale in `speed = polar_c · (1 + |sin φ|^polar_sin_power) / 2 · TWS`.
    pub polar_c: f64,
    /// Polar sharpness exponent on `|sin φ|`.
    pub polar_sin_power: f64,
    /// Cubic SFC coefficients in `mcr_01 · (fuel_a + fuel_b·mcr_01 + fuel_c·mcr_01²)`.
    pub fuel_a: f64,
    /// See [`Self::fuel_a`].
    pub fuel_b: f64,
    /// See [`Self::fuel_a`].
    pub fuel_c: f64,
}

impl Default for BoatConfig {
    fn default() -> Self {
        Self {
            mcr_kw: DEFAULT_BOAT_MCR_KW,
            k: DEFAULT_BOAT_K,
            polar_c: DEFAULT_BOAT_POLAR_C,
            polar_sin_power: DEFAULT_BOAT_POLAR_SIN_POWER,
            fuel_a: DEFAULT_BOAT_FUEL_A,
            fuel_b: DEFAULT_BOAT_FUEL_B,
            fuel_c: DEFAULT_BOAT_FUEL_C,
        }
    }
}

impl BoatConfig {
    /// Converts `mcr_kw` → watts for the swarmkit-side `Boat`.
    pub fn to_boat(&self) -> Boat {
        Boat {
            mcr: self.mcr_kw * 1000.0,
            k: self.k,
            polar_c: self.polar_c,
            polar_sin_power: self.polar_sin_power,
            fuel_a: self.fuel_a,
            fuel_b: self.fuel_b,
            fuel_c: self.fuel_c,
        }
    }

    /// Reject NaN/Inf in any field, or a non-positive `mcr_kw` (which
    /// would make every throttle-to-power conversion produce zero force).
    /// Errors are `boat.`-prefixed so TOML callers can locate the key.
    ///
    /// # Errors
    /// Returns the first non-finite field or non-positive `mcr_kw`.
    pub fn validate(&self) -> Result<(), ValidationError> {
        for (field, v) in [
            ("boat.mcr_kw", self.mcr_kw),
            ("boat.k", self.k),
            ("boat.polar_c", self.polar_c),
            ("boat.polar_sin_power", self.polar_sin_power),
            ("boat.fuel_a", self.fuel_a),
            ("boat.fuel_b", self.fuel_b),
            ("boat.fuel_c", self.fuel_c),
        ] {
            ValidationError::require_finite(field, v)?;
        }
        ValidationError::require_positive("boat.mcr_kw", self.mcr_kw)?;
        Ok(())
    }
}

/// A single field in a [`BoatConfig`] or [`SearchConfig`] failed its
/// range check. `field` matches the struct path (with `boat.` prefix
/// for `BoatConfig` fields) so TOML callers can locate the source key.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ValidationError {
    pub field: &'static str,
    pub message: String,
}

impl ValidationError {
    fn require_finite(field: &'static str, value: f64) -> Result<(), Self> {
        if value.is_finite() {
            Ok(())
        } else {
            Err(Self {
                field,
                message: format!("must be finite, got {value}"),
            })
        }
    }

    fn require_non_negative(field: &'static str, value: f64) -> Result<(), Self> {
        if value >= 0.0 {
            Ok(())
        } else {
            Err(Self {
                field,
                message: format!("must be non-negative, got {value}"),
            })
        }
    }

    fn require_positive(field: &'static str, value: f64) -> Result<(), Self> {
        if value > 0.0 {
            Ok(())
        } else {
            Err(Self {
                field,
                message: format!("must be > 0, got {value}"),
            })
        }
    }

    fn require_at_least_one(field: &'static str, value: usize) -> Result<(), Self> {
        if value >= 1 {
            Ok(())
        } else {
            Err(Self {
                field,
                message: format!("must be at least 1, got {value}"),
            })
        }
    }

    fn require_in_unit_interval(field: &'static str, value: f64) -> Result<(), Self> {
        if (0.0..=1.0).contains(&value) {
            Ok(())
        } else {
            Err(Self {
                field,
                message: format!("must be in [0, 1], got {value}"),
            })
        }
    }
}

impl std::fmt::Display for ValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.field, self.message)
    }
}

impl std::error::Error for ValidationError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn search_config_default_validates_clean() {
        SearchConfig::default()
            .validate()
            .expect("default must validate");
    }

    #[test]
    fn boat_config_default_validates_clean() {
        BoatConfig::default()
            .validate()
            .expect("default must validate");
    }

    #[test]
    fn search_config_rejects_negative_weight() {
        let cfg = SearchConfig {
            time_weight: -1.0,
            ..SearchConfig::default()
        };
        let err = cfg.validate().expect_err("negative weight");
        assert_eq!(err.field, "time_weight");
    }

    #[test]
    fn search_config_rejects_non_finite_weight() {
        let cfg = SearchConfig {
            fuel_weight: f64::NAN,
            ..SearchConfig::default()
        };
        let err = cfg.validate().expect_err("NaN weight");
        assert_eq!(err.field, "fuel_weight");
    }

    #[test]
    fn search_config_rejects_zero_particles() {
        let cfg = SearchConfig {
            particles_space: 0,
            ..SearchConfig::default()
        };
        let err = cfg.validate().expect_err("zero particles");
        assert_eq!(err.field, "particles_space");
    }

    #[test]
    fn search_config_rejects_zero_iterations() {
        let cfg = SearchConfig {
            iter_time: 0,
            ..SearchConfig::default()
        };
        let err = cfg.validate().expect_err("zero iterations");
        assert_eq!(err.field, "iter_time");
    }

    #[test]
    fn search_config_rejects_non_finite_coefficient() {
        let cfg = SearchConfig {
            inertia: f64::INFINITY,
            ..SearchConfig::default()
        };
        let err = cfg.validate().expect_err("infinite inertia");
        assert_eq!(err.field, "inertia");
    }

    #[test]
    fn boat_config_rejects_zero_mcr() {
        let cfg = BoatConfig {
            mcr_kw: 0.0,
            ..BoatConfig::default()
        };
        let err = cfg.validate().expect_err("zero mcr");
        assert_eq!(err.field, "boat.mcr_kw");
    }

    #[test]
    fn boat_config_rejects_negative_mcr() {
        let cfg = BoatConfig {
            mcr_kw: -1.0,
            ..BoatConfig::default()
        };
        let err = cfg.validate().expect_err("negative mcr");
        assert_eq!(err.field, "boat.mcr_kw");
    }

    #[test]
    fn boat_config_rejects_non_finite_polar() {
        let cfg = BoatConfig {
            polar_c: f64::NAN,
            ..BoatConfig::default()
        };
        let err = cfg.validate().expect_err("NaN polar_c");
        assert_eq!(err.field, "boat.polar_c");
    }

    #[test]
    fn validation_error_display_contains_field_and_message() {
        let cfg = SearchConfig {
            time_weight: -2.5,
            ..SearchConfig::default()
        };
        let err = cfg.validate().unwrap_err();
        let s = err.to_string();
        assert!(s.contains("time_weight"));
        assert!(s.contains("-2.5"));
    }

    #[test]
    fn search_config_rejects_tiny_step_distance_fraction() {
        let cfg = SearchConfig {
            step_distance_fraction: 1e-300,
            ..SearchConfig::default()
        };
        let err = cfg.validate().expect_err("tiny step_distance_fraction");
        assert_eq!(err.field, "step_distance_fraction");
    }

    #[test]
    fn search_config_rejects_too_large_step_distance_fraction() {
        let cfg = SearchConfig {
            step_distance_fraction: 2.0,
            ..SearchConfig::default()
        };
        let err = cfg.validate().expect_err("step_distance_fraction > 1.0");
        assert_eq!(err.field, "step_distance_fraction");
    }

    #[test]
    fn search_config_rejects_tiny_bake_step_deg() {
        let cfg = SearchConfig {
            bake_step_deg: 1e-9,
            ..SearchConfig::default()
        };
        let err = cfg.validate().expect_err("bake_step_deg too small");
        assert_eq!(err.field, "bake_step_deg");
    }

    #[test]
    fn search_config_rejects_huge_sdf_resolution() {
        let cfg = SearchConfig {
            sdf_resolution_deg: 100.0,
            ..SearchConfig::default()
        };
        let err = cfg.validate().expect_err("sdf_resolution_deg too coarse");
        assert_eq!(err.field, "sdf_resolution_deg");
    }

    #[test]
    fn search_config_rejects_tiny_sdf_resolution() {
        let cfg = SearchConfig {
            sdf_resolution_deg: 0.001,
            ..SearchConfig::default()
        };
        let err = cfg.validate().expect_err("sdf_resolution_deg too fine");
        assert_eq!(err.field, "sdf_resolution_deg");
    }

    #[test]
    fn search_config_rejects_huge_range_k() {
        let cfg = SearchConfig {
            range_k: 10_000,
            ..SearchConfig::default()
        };
        let err = cfg.validate().expect_err("range_k > 256");
        assert_eq!(err.field, "range_k");
    }

    #[test]
    fn search_config_rejects_huge_k_mcr() {
        let cfg = SearchConfig {
            k_mcr: 10_000,
            ..SearchConfig::default()
        };
        let err = cfg.validate().expect_err("k_mcr > 256");
        assert_eq!(err.field, "k_mcr");
    }
}
