//! TOML scenario files: a portable description of "what to search".
//!
//! Three optional sections — `[run]`, `[boat]`, `[search]` — each with
//! all-optional fields. Used by `bywind-cli` to drive `search` /
//! `tune-trial` runs and by `bywind-viz` to load the same files into the
//! GUI for visualisation. Multiple files merge left-to-right; the consumer
//! layers the result onto its own defaults (`SearchConfig::default()` etc.)
//! before applying.

use std::path::{Path, PathBuf};

use crate::{BoatConfig, SearchConfig, Topology};

/// Top-level TOML schema. Every section is optional; empty / missing sections
/// just contribute no overrides.
#[derive(Default, Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct CliConfigFile {
    pub run: RunOverrides,
    pub boat: BoatOverrides,
    pub search: SearchOverrides,
    pub tune: TuneOverrides,
}

/// Run-level fields: map / start / end / bounds / waypoints / weights.
///
/// These correspond 1:1 to top-level `bywind-cli` flags. Every field is
/// `Option` so the same shape works for partial TOML files and for
/// clap-flag overrides built in `main.rs`. On serialization, `None`
/// fields are omitted so a round-trip stays minimal.
#[derive(Default, Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct RunOverrides {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub map: Option<PathBuf>,
    /// `[lon, lat]`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub start: Option<[f64; 2]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub end: Option<[f64; 2]>,
    /// `[lon_min, lat_min, lon_max, lat_max]` — the order most natural for
    /// SW/NE bbox descriptions. Converted to `MapBounds::clamp_to`'s
    /// `(lon_min, lon_max, lat_min, lat_max)` shape at apply time.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bounds: Option<[f64; 4]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub waypoints: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub time_weight: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fuel_weight: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub land_weight: Option<f64>,
}

/// Boat polar / fuel parameters. No CLI flags map to these directly — only
/// the `[boat]` section of a TOML config can set them.
#[derive(Default, Clone, Copy, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct BoatOverrides {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mcr_kw: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub k: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub polar_c: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub polar_sin_power: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fuel_a: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fuel_b: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fuel_c: Option<f64>,
}

/// PSO tuning.
///
/// Like `BoatOverrides`, no CLI flags — only the `[search]` TOML section.
/// Weights and waypoint count live in `[run]` (where their CLI flags map)
/// so this struct stays focused on the inner-loop knobs.
#[derive(Default, Clone, Copy, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct SearchOverrides {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub particles_space: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub particles_time: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub iter_space: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub iter_time: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inertia: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cognitive_coeff: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub social_coeff: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path_kick_probability: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path_kick_gamma_0_fraction: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path_kick_gamma_min_fraction: Option<f64>,
    /// Optional RNG seed for deterministic search runs. `None` (the
    /// default) draws fresh OS entropy. Used by the PSO-tuning study.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub seed: Option<u64>,
    /// Outer-loop topology (e.g. `"gbest"`). `None` keeps the default
    /// from `SearchConfig`. See [`crate::Topology`].
    #[serde(
        default,
        with = "crate::config::topology_serde::option",
        skip_serializing_if = "Option::is_none"
    )]
    pub topology: Option<Topology>,
}

/// Per-coefficient declaration for the `[tune]` section.
///
/// A scalar like `inertia = 0.328` parses as [`Self::Pinned`]; a table like
/// `inertia = { min = 0.0, max = 1.2 }` parses as [`Self::Range`]. The
/// tuning study uses [`Self::Pinned`] to drop a dimension from the search
/// space (every trial reuses the value) and [`Self::Range`] to feed the
/// optimiser a `[low, high]` interval.
#[derive(Clone, Copy, Debug, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(untagged)]
pub enum TuneSlot {
    Pinned(f64),
    Range { min: f64, max: f64 },
}

impl TuneSlot {
    /// Reject obviously-broken slots: NaN/∞ values, inverted ranges, or
    /// negative coefficients (PSO theory permits negative values, but
    /// they're nonsensical for the sailing search and almost certainly
    /// indicate a typo). `name` identifies the slot in the returned error.
    ///
    /// # Errors
    /// Returns the first failure as a [`TuneValidationError`].
    pub fn validate(&self, name: &str) -> Result<(), TuneValidationError> {
        match *self {
            Self::Pinned(value) => {
                if !value.is_finite() {
                    return Err(TuneValidationError::NonFinitePinned {
                        name: name.to_owned(),
                        value,
                    });
                }
                if value < 0.0 {
                    return Err(TuneValidationError::NegativePinned {
                        name: name.to_owned(),
                        value,
                    });
                }
            }
            Self::Range { min, max } => {
                if !min.is_finite() || !max.is_finite() {
                    return Err(TuneValidationError::NonFiniteRange {
                        name: name.to_owned(),
                        min,
                        max,
                    });
                }
                if min < 0.0 {
                    return Err(TuneValidationError::NegativeRangeMin {
                        name: name.to_owned(),
                        min,
                    });
                }
                if min >= max {
                    return Err(TuneValidationError::InvertedRange {
                        name: name.to_owned(),
                        min,
                        max,
                    });
                }
            }
        }
        Ok(())
    }
}

/// Errors produced by [`TuneSlot::validate`] / [`TuneOverrides::validate`].
///
/// Each variant carries the slot name (e.g. `"inertia"`) so a single error
/// pinpoints both the failing slot and the offending value.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum TuneValidationError {
    /// A pinned value was NaN or ±∞.
    NonFinitePinned { name: String, value: f64 },
    /// A pinned value was finite but negative.
    NegativePinned { name: String, value: f64 },
    /// At least one of `min` / `max` was NaN or ±∞.
    NonFiniteRange { name: String, min: f64, max: f64 },
    /// `min` was finite but negative.
    NegativeRangeMin { name: String, min: f64 },
    /// `min >= max` — the range is empty or inverted.
    InvertedRange { name: String, min: f64, max: f64 },
}

impl std::fmt::Display for TuneValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFinitePinned { name, value } => {
                write!(f, "tune.{name}: pinned value must be finite, got {value}")
            }
            Self::NegativePinned { name, value } => write!(
                f,
                "tune.{name}: pinned value must be non-negative, got {value}",
            ),
            Self::NonFiniteRange { name, min, max } => write!(
                f,
                "tune.{name}: range bounds must be finite, got [{min}, {max}]",
            ),
            Self::NegativeRangeMin { name, min } => {
                write!(f, "tune.{name}: range min must be non-negative, got {min}")
            }
            Self::InvertedRange { name, min, max } => write!(
                f,
                "tune.{name}: range min={min} must be strictly less than max={max}",
            ),
        }
    }
}

impl std::error::Error for TuneValidationError {}

/// Search-space declaration for `bywind-cli tune`.
///
/// Each PSO coefficient is independently either pinned (every trial uses
/// the same value) or varied over a `{ min, max }` range that the TPE
/// sampler explores. Missing fields fall through to the tune subcommand's
/// built-in defaults — you can specify just one knob without disturbing
/// the others.
///
/// `path_kick_decay_ratio` is a tune-only virtual knob: the trial-side
/// computes `path_kick_gamma_min_fraction = path_kick_gamma_0_fraction *
/// decay_ratio` so the `min ≤ max` invariant of the cosine schedule is
/// intrinsic to the parameterisation. It does not appear in
/// `SearchSettings` directly.
#[derive(Default, Clone, Copy, Debug, serde::Deserialize, serde::Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct TuneOverrides {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inertia: Option<TuneSlot>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cognitive_coeff: Option<TuneSlot>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub social_coeff: Option<TuneSlot>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path_kick_probability: Option<TuneSlot>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path_kick_gamma_0_fraction: Option<TuneSlot>,
    /// Virtual: `gamma_min = gamma_0 * decay_ratio`. ∈ [0, 1] by convention.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path_kick_decay_ratio: Option<TuneSlot>,
}

impl TuneOverrides {
    pub fn merge_from(&mut self, other: Self) {
        if other.inertia.is_some() {
            self.inertia = other.inertia;
        }
        if other.cognitive_coeff.is_some() {
            self.cognitive_coeff = other.cognitive_coeff;
        }
        if other.social_coeff.is_some() {
            self.social_coeff = other.social_coeff;
        }
        if other.path_kick_probability.is_some() {
            self.path_kick_probability = other.path_kick_probability;
        }
        if other.path_kick_gamma_0_fraction.is_some() {
            self.path_kick_gamma_0_fraction = other.path_kick_gamma_0_fraction;
        }
        if other.path_kick_decay_ratio.is_some() {
            self.path_kick_decay_ratio = other.path_kick_decay_ratio;
        }
    }

    /// Validate every populated slot. Stops at the first failure.
    ///
    /// # Errors
    /// Returns the first slot's [`TuneValidationError`]; see
    /// [`TuneSlot::validate`].
    pub fn validate(&self) -> Result<(), TuneValidationError> {
        for (name, slot) in [
            ("inertia", &self.inertia),
            ("cognitive_coeff", &self.cognitive_coeff),
            ("social_coeff", &self.social_coeff),
            ("path_kick_probability", &self.path_kick_probability),
            (
                "path_kick_gamma_0_fraction",
                &self.path_kick_gamma_0_fraction,
            ),
            ("path_kick_decay_ratio", &self.path_kick_decay_ratio),
        ] {
            if let Some(s) = slot {
                s.validate(name)?;
            }
        }
        Ok(())
    }
}

/// Errors produced by [`CliConfigFile::from_path`]. Wraps the path so callers
/// can render a useful message without re-decorating.
#[derive(Debug)]
#[non_exhaustive]
pub enum LoadError {
    Read {
        path: PathBuf,
        source: std::io::Error,
    },
    Parse {
        path: PathBuf,
        source: toml::de::Error,
    },
}

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Read { path, source } => {
                write!(f, "reading config {}: {source}", path.display())
            }
            Self::Parse { path, source } => {
                write!(f, "parsing config {}: {source}", path.display())
            }
        }
    }
}

impl std::error::Error for LoadError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Read { source, .. } => Some(source),
            Self::Parse { source, .. } => Some(source),
        }
    }
}

impl CliConfigFile {
    /// Load and parse a TOML config file. Failures wrap the path for context.
    ///
    /// # Errors
    /// Returns [`LoadError::Read`] if the file cannot be opened, or
    /// [`LoadError::Parse`] if the TOML is malformed or contains unknown
    /// fields.
    pub fn from_path(path: &Path) -> Result<Self, LoadError> {
        let text = std::fs::read_to_string(path).map_err(|source| LoadError::Read {
            path: path.to_path_buf(),
            source,
        })?;
        toml::from_str(&text).map_err(|source| LoadError::Parse {
            path: path.to_path_buf(),
            source,
        })
    }

    /// Render this scenario as a pretty-printed TOML document. Mirrors
    /// [`Self::from_path`] for the save direction so consumers don't need
    /// to depend on the `toml` crate themselves.
    ///
    /// # Errors
    /// Returns the underlying [`toml::ser::Error`] if a value can't be
    /// represented (e.g. NaN — TOML has no NaN literal). All-finite
    /// inputs always succeed.
    pub fn to_toml_string(&self) -> Result<String, toml::ser::Error> {
        toml::to_string_pretty(self)
    }

    /// In-place merge: any `Some` field in `other` overrides this struct's
    /// value. Sections compose recursively.
    pub fn merge_from(&mut self, other: Self) {
        self.run.merge_from(other.run);
        self.boat.merge_from(other.boat);
        self.search.merge_from(other.search);
        self.tune.merge_from(other.tune);
    }
}

impl RunOverrides {
    pub fn merge_from(&mut self, other: Self) {
        if other.map.is_some() {
            self.map = other.map;
        }
        if other.start.is_some() {
            self.start = other.start;
        }
        if other.end.is_some() {
            self.end = other.end;
        }
        if other.bounds.is_some() {
            self.bounds = other.bounds;
        }
        if other.waypoints.is_some() {
            self.waypoints = other.waypoints;
        }
        if other.time_weight.is_some() {
            self.time_weight = other.time_weight;
        }
        if other.fuel_weight.is_some() {
            self.fuel_weight = other.fuel_weight;
        }
        if other.land_weight.is_some() {
            self.land_weight = other.land_weight;
        }
    }
}

impl BoatOverrides {
    pub fn merge_from(&mut self, other: Self) {
        if other.mcr_kw.is_some() {
            self.mcr_kw = other.mcr_kw;
        }
        if other.k.is_some() {
            self.k = other.k;
        }
        if other.polar_c.is_some() {
            self.polar_c = other.polar_c;
        }
        if other.polar_sin_power.is_some() {
            self.polar_sin_power = other.polar_sin_power;
        }
        if other.fuel_a.is_some() {
            self.fuel_a = other.fuel_a;
        }
        if other.fuel_b.is_some() {
            self.fuel_b = other.fuel_b;
        }
        if other.fuel_c.is_some() {
            self.fuel_c = other.fuel_c;
        }
    }

    /// Layer this struct's `Some` fields onto `cfg`, leaving `None` fields
    /// at whatever value `cfg` already has.
    pub fn apply_to(&self, cfg: &mut BoatConfig) {
        if let Some(v) = self.mcr_kw {
            cfg.mcr_kw = v;
        }
        if let Some(v) = self.k {
            cfg.k = v;
        }
        if let Some(v) = self.polar_c {
            cfg.polar_c = v;
        }
        if let Some(v) = self.polar_sin_power {
            cfg.polar_sin_power = v;
        }
        if let Some(v) = self.fuel_a {
            cfg.fuel_a = v;
        }
        if let Some(v) = self.fuel_b {
            cfg.fuel_b = v;
        }
        if let Some(v) = self.fuel_c {
            cfg.fuel_c = v;
        }
    }
}

impl SearchOverrides {
    pub fn merge_from(&mut self, other: Self) {
        if other.particles_space.is_some() {
            self.particles_space = other.particles_space;
        }
        if other.particles_time.is_some() {
            self.particles_time = other.particles_time;
        }
        if other.iter_space.is_some() {
            self.iter_space = other.iter_space;
        }
        if other.iter_time.is_some() {
            self.iter_time = other.iter_time;
        }
        if other.inertia.is_some() {
            self.inertia = other.inertia;
        }
        if other.cognitive_coeff.is_some() {
            self.cognitive_coeff = other.cognitive_coeff;
        }
        if other.social_coeff.is_some() {
            self.social_coeff = other.social_coeff;
        }
        if other.path_kick_probability.is_some() {
            self.path_kick_probability = other.path_kick_probability;
        }
        if other.path_kick_gamma_0_fraction.is_some() {
            self.path_kick_gamma_0_fraction = other.path_kick_gamma_0_fraction;
        }
        if other.path_kick_gamma_min_fraction.is_some() {
            self.path_kick_gamma_min_fraction = other.path_kick_gamma_min_fraction;
        }
        if other.seed.is_some() {
            self.seed = other.seed;
        }
        if other.topology.is_some() {
            self.topology = other.topology;
        }
    }

    /// Layer this struct's `Some` fields onto `cfg`'s PSO-tuning fields.
    /// Does not touch `time_weight` / `fuel_weight` / `land_weight` /
    /// `waypoint_count` — those come from `RunOverrides`.
    pub fn apply_to(&self, cfg: &mut SearchConfig) {
        if let Some(v) = self.particles_space {
            cfg.particles_space = v;
        }
        if let Some(v) = self.particles_time {
            cfg.particles_time = v;
        }
        if let Some(v) = self.iter_space {
            cfg.iter_space = v;
        }
        if let Some(v) = self.iter_time {
            cfg.iter_time = v;
        }
        if let Some(v) = self.inertia {
            cfg.inertia = v;
        }
        if let Some(v) = self.cognitive_coeff {
            cfg.cognitive_coeff = v;
        }
        if let Some(v) = self.social_coeff {
            cfg.social_coeff = v;
        }
        if let Some(v) = self.path_kick_probability {
            cfg.path_kick_probability = v;
        }
        if let Some(v) = self.path_kick_gamma_0_fraction {
            cfg.path_kick_gamma_0_fraction = v;
        }
        if let Some(v) = self.path_kick_gamma_min_fraction {
            cfg.path_kick_gamma_min_fraction = v;
        }
        if let Some(v) = self.seed {
            cfg.seed = Some(v);
        }
        if let Some(v) = self.topology {
            cfg.topology = v;
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "tests assert on bit-exact constants pulled out of TuneValidationError variants."
    )]
    use super::*;

    #[test]
    fn empty_toml_parses_to_default() {
        let cfg: CliConfigFile = toml::from_str("").expect("empty TOML");
        assert!(cfg.run.start.is_none());
        assert!(cfg.boat.mcr_kw.is_none());
        assert!(cfg.search.inertia.is_none());
    }

    #[test]
    fn unknown_field_rejected() {
        let err = toml::from_str::<CliConfigFile>("nonsense = 1\n")
            .expect_err("unknown top-level fields should be rejected");
        assert!(err.to_string().contains("nonsense"));
    }

    #[test]
    fn run_section_partial_parse() {
        let cfg: CliConfigFile = toml::from_str(
            r#"
            [run]
            start = [35.0, -50.0]
            waypoints = 20
            "#,
        )
        .expect("valid TOML");
        assert_eq!(cfg.run.start, Some([35.0, -50.0]));
        assert_eq!(cfg.run.waypoints, Some(20));
        assert!(cfg.run.end.is_none());
        assert!(cfg.run.bounds.is_none());
    }

    #[test]
    fn merge_overrides_some_fields_only() {
        let base: CliConfigFile = toml::from_str(
            r#"
            [run]
            start = [1.0, 2.0]
            waypoints = 10
            "#,
        )
        .expect("base TOML");
        let on_top: CliConfigFile = toml::from_str(
            r#"
            [run]
            waypoints = 20
            end = [3.0, 4.0]
            "#,
        )
        .expect("override TOML");
        let mut merged = base;
        merged.merge_from(on_top);
        // `start` survives from base, `waypoints` is replaced, `end` is added.
        assert_eq!(merged.run.start, Some([1.0, 2.0]));
        assert_eq!(merged.run.waypoints, Some(20));
        assert_eq!(merged.run.end, Some([3.0, 4.0]));
    }

    #[test]
    fn boat_apply_to_only_writes_some_fields() {
        let overrides: BoatOverrides = toml::from_str("mcr_kw = 5000.0\n").expect("valid");
        let mut cfg = BoatConfig::default();
        let baseline = cfg;
        overrides.apply_to(&mut cfg);
        assert!((cfg.mcr_kw - 5000.0).abs() < 1e-9);
        // Untouched fields stay at default.
        assert!((cfg.k - baseline.k).abs() < 1e-9);
        assert!((cfg.polar_c - baseline.polar_c).abs() < 1e-9);
    }

    #[test]
    fn serialize_omits_none_fields() {
        // Only fields with `Some(_)` should appear in the output; `None`
        // fields must not show up as `field = null` (which `toml` doesn't
        // even support) or as bare keys (which would break round-trip).
        let cfg = CliConfigFile {
            run: RunOverrides {
                start: Some([1.0, 2.0]),
                ..Default::default()
            },
            ..Default::default()
        };
        let s = toml::to_string(&cfg).expect("serialize");
        assert!(s.contains("start"));
        assert!(!s.contains("end"));
        assert!(!s.contains("waypoints"));
        assert!(!s.contains("mcr_kw"));
    }

    #[test]
    fn round_trip_through_serialization_preserves_values() {
        let original = CliConfigFile {
            run: RunOverrides {
                start: Some([-122.42, 37.77]),
                end: Some([139.77, 35.68]),
                bounds: Some([-130.0, 30.0, 140.0, 50.0]),
                waypoints: Some(40),
                time_weight: Some(1.0),
                fuel_weight: Some(10.0),
                land_weight: Some(50.0),
                ..Default::default()
            },
            boat: BoatOverrides {
                mcr_kw: Some(1234.5),
                ..Default::default()
            },
            search: SearchOverrides {
                inertia: Some(0.328),
                cognitive_coeff: Some(1.6),
                social_coeff: Some(0.85),
                ..Default::default()
            },
            tune: TuneOverrides::default(),
        };
        let s = toml::to_string(&original).expect("serialize");
        let parsed: CliConfigFile = toml::from_str(&s).expect("re-parse");
        assert_eq!(parsed.run.start, original.run.start);
        assert_eq!(parsed.run.end, original.run.end);
        assert_eq!(parsed.run.bounds, original.run.bounds);
        assert_eq!(parsed.run.waypoints, original.run.waypoints);
        assert_eq!(parsed.boat.mcr_kw, original.boat.mcr_kw);
        assert_eq!(parsed.search.inertia, original.search.inertia);
        // Fields that were None should still be None after round-trip.
        assert!(parsed.boat.k.is_none());
        assert!(parsed.search.seed.is_none());
    }

    #[test]
    fn tune_section_parses_pinned_and_range_forms() {
        let cfg: CliConfigFile = toml::from_str(
            r#"
            [tune]
            inertia = 0.328
            cognitive_coeff = { min = 0.0, max = 4.0 }
            social_coeff = { min = 0.5, max = 3.0 }
            "#,
        )
        .expect("valid TOML");
        assert_eq!(cfg.tune.inertia, Some(TuneSlot::Pinned(0.328)));
        assert_eq!(
            cfg.tune.cognitive_coeff,
            Some(TuneSlot::Range { min: 0.0, max: 4.0 }),
        );
        assert_eq!(
            cfg.tune.social_coeff,
            Some(TuneSlot::Range { min: 0.5, max: 3.0 }),
        );
    }

    #[test]
    fn tune_section_partial_fields_are_independent() {
        // Only one knob set — the other two fall through to None so the
        // CLI's defaults still apply.
        let cfg: CliConfigFile = toml::from_str("[tune]\ninertia = 0.5\n").expect("valid");
        assert_eq!(cfg.tune.inertia, Some(TuneSlot::Pinned(0.5)));
        assert!(cfg.tune.cognitive_coeff.is_none());
        assert!(cfg.tune.social_coeff.is_none());
    }

    #[test]
    fn tune_validate_rejects_inverted_range() {
        let slot = TuneSlot::Range { min: 1.0, max: 0.0 };
        let err = slot.validate("inertia").expect_err("inverted");
        assert!(
            matches!(
                &err,
                TuneValidationError::InvertedRange { name, min, max }
                if name == "inertia" && *min == 1.0 && *max == 0.0,
            ),
            "got: {err}",
        );
    }

    #[test]
    fn tune_validate_rejects_non_finite_pinned() {
        let err = TuneSlot::Pinned(f64::NAN)
            .validate("inertia")
            .expect_err("NaN");
        assert!(
            matches!(
                &err,
                TuneValidationError::NonFinitePinned { name, value }
                if name == "inertia" && value.is_nan(),
            ),
            "got: {err}",
        );
    }

    #[test]
    fn tune_validate_rejects_negative_pinned() {
        let err = TuneSlot::Pinned(-0.1)
            .validate("social_coeff")
            .expect_err("negative");
        assert!(
            matches!(
                &err,
                TuneValidationError::NegativePinned { name, value }
                if name == "social_coeff" && *value == -0.1,
            ),
            "got: {err}",
        );
    }

    #[test]
    fn tune_overrides_merge_replaces_some_fields() {
        let mut base = TuneOverrides {
            inertia: Some(TuneSlot::Pinned(0.2)),
            cognitive_coeff: Some(TuneSlot::Range { min: 0.0, max: 4.0 }),
            ..Default::default()
        };
        let on_top = TuneOverrides {
            inertia: Some(TuneSlot::Range { min: 0.0, max: 1.0 }),
            ..Default::default()
        };
        base.merge_from(on_top);
        // inertia replaced, cognitive_coeff preserved.
        assert_eq!(base.inertia, Some(TuneSlot::Range { min: 0.0, max: 1.0 }));
        assert_eq!(
            base.cognitive_coeff,
            Some(TuneSlot::Range { min: 0.0, max: 4.0 }),
        );
    }

    #[test]
    fn tune_section_parses_path_kick_fields() {
        let cfg: CliConfigFile = toml::from_str(
            r#"
            [tune]
            path_kick_probability = { min = 0.0, max = 0.5 }
            path_kick_gamma_0_fraction = { min = 0.0, max = 0.20 }
            path_kick_decay_ratio = { min = 0.0, max = 1.0 }
            "#,
        )
        .expect("valid TOML");
        assert_eq!(
            cfg.tune.path_kick_probability,
            Some(TuneSlot::Range { min: 0.0, max: 0.5 }),
        );
        assert_eq!(
            cfg.tune.path_kick_gamma_0_fraction,
            Some(TuneSlot::Range {
                min: 0.0,
                max: 0.20
            }),
        );
        assert_eq!(
            cfg.tune.path_kick_decay_ratio,
            Some(TuneSlot::Range { min: 0.0, max: 1.0 }),
        );
    }

    #[test]
    fn search_apply_to_writes_path_kick_fields() {
        let overrides: SearchOverrides = toml::from_str(
            "path_kick_probability = 0.25\n\
             path_kick_gamma_0_fraction = 0.1\n\
             path_kick_gamma_min_fraction = 0.01\n",
        )
        .expect("valid");
        let mut cfg = SearchConfig::default();
        overrides.apply_to(&mut cfg);
        assert!((cfg.path_kick_probability - 0.25).abs() < 1e-12);
        assert!((cfg.path_kick_gamma_0_fraction - 0.1).abs() < 1e-12);
        assert!((cfg.path_kick_gamma_min_fraction - 0.01).abs() < 1e-12);
    }

    #[test]
    fn search_apply_to_does_not_touch_weights() {
        let overrides: SearchOverrides =
            toml::from_str("particles_space = 200\ninertia = 0.5\n").expect("valid");
        let mut cfg = SearchConfig::default();
        let baseline = cfg;
        overrides.apply_to(&mut cfg);
        assert_eq!(cfg.particles_space, 200);
        assert!((cfg.inertia - 0.5).abs() < 1e-9);
        // Weights are RunOverrides territory; SearchOverrides::apply_to leaves them.
        assert!((cfg.time_weight - baseline.time_weight).abs() < 1e-9);
        assert!((cfg.fuel_weight - baseline.fuel_weight).abs() < 1e-9);
        assert!((cfg.land_weight - baseline.land_weight).abs() < 1e-9);
    }
}
