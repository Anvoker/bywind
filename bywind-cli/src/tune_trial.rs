//! `tune-trial` subcommand. Per-trial worker for the PSO-tuning study.
//!
//! Reads a JSON trial spec from stdin, runs the sailing search across
//! `routes × seeds` with the supplied PSO coefficients, and emits
//! aggregate JSON on stdout. Reading stdin / writing stdout makes it
//! trivial to drive from a Rust BO loop (Phase 3) or any other
//! orchestrator. Crash isolation is by subprocess: a degenerate config
//! that panics doesn't kill the study.
//!
//! Trial spec (stdin):
//! ```json
//! {
//!   "params":  { "inertia": 0.5, "cognitive_coeff": 1.6, "social_coeff": 1.4 },
//!   "seeds":   [42, 1337, 9001],
//!   "routes":  ["coastal-detour", "archipelago", "short-easy", "mid-pacific"],
//!   "routes_dir": "profiling/tuning"
//! }
//! ```
//!
//! All fields except `seeds` and `routes` are optional. Missing param
//! fields fall through to `SearchConfig::default()`. `routes_dir`
//! defaults to `profiling/tuning`.
//!
//! Output (stdout): see [`TrialResult`].

use std::io::{self, Read as _, Write as _};
use std::path::PathBuf;
use std::time::Instant;

use anyhow::{Context as _, Result, anyhow};
use bywind::{
    BakedWindMap, BoatConfig, LonLatBbox, MapBounds, SearchConfig, SearchWeights, baked_codec,
    run_search_blocking_with_baked,
};
use serde::{Deserialize, Serialize};

// Note: `TrialSpec` and `TrialParams` are `Serialize` *and* `Deserialize`
// because the `tune` subcommand (`crate::tune`) builds and serialises
// them before piping into a `tune-trial` subprocess. Both ends of the
// pipe live in this same binary, so sharing the type avoids drift.

use crate::error::AppError;
use bywind::scenario::CliConfigFile;

/// Default location of the per-route TOML configs and baked grids.
/// Layout: `<routes_dir>/<slug>/{config.toml,baked.bk1}` plus an
/// optional shared `<routes_dir>/_search.toml` for sizing overrides
/// (particles / iters) common to every route in the study.
const DEFAULT_ROUTES_DIR: &str = "profiling/tuning";

#[derive(Serialize, Deserialize, Debug)]
#[serde(deny_unknown_fields)]
pub struct TrialSpec {
    /// PSO coefficient overrides applied on top of `SearchConfig::default()`
    /// and the route's TOML. Any field omitted falls through to the layered
    /// default.
    #[serde(default)]
    pub params: TrialParams,
    /// Seeds run per route. Each seed is one full search per route.
    pub seeds: Vec<u64>,
    /// Route slugs (directory names under `routes_dir`).
    pub routes: Vec<String>,
    /// Override of the routes directory. Defaults to `profiling/tuning`.
    #[serde(default)]
    pub routes_dir: Option<PathBuf>,
}

/// Subset of `SearchConfig` that the tuner can vary. The three GBest
/// velocity coefficients plus the path-kick mover knobs; particle /
/// iter counts, init/baseline shares, and Cauchy mutation are held at
/// the values supplied by `_search.toml` plus each route's `config.toml`
/// so every trial sees the same compute budget. The tuner side computes
/// `path_kick_gamma_min_fraction = path_kick_gamma_0_fraction × decay_ratio`
/// before serialising — `decay_ratio` is purely a tune-side
/// parameterisation, never appears here.
#[derive(Serialize, Deserialize, Debug, Default)]
#[serde(deny_unknown_fields)]
pub struct TrialParams {
    pub inertia: Option<f64>,
    pub cognitive_coeff: Option<f64>,
    pub social_coeff: Option<f64>,
    pub path_kick_probability: Option<f64>,
    pub path_kick_gamma_0_fraction: Option<f64>,
    pub path_kick_gamma_min_fraction: Option<f64>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct TrialResult {
    /// Mean of `per_route[i].fitness_mean` across routes. Raw — the
    /// orchestrator is expected to apply per-route baseline
    /// normalisation before optimising. Useful as a sanity number.
    pub fitness_mean: f64,
    /// Standard deviation of `per_route[i].fitness_mean` across routes.
    pub fitness_std: f64,
    pub wall_seconds: f64,
    pub per_route: Vec<RouteResult>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct RouteResult {
    pub slug: String,
    /// Mean fitness across this route's seeds. Closer to 0 = better
    /// (fitness is `-(time*w_t + fuel*w_f + land*w_l)`).
    pub fitness_mean: f64,
    pub fitness_std: f64,
    pub seeds: Vec<SeedResult>,
    pub wall_seconds: f64,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SeedResult {
    pub seed: u64,
    pub fitness: f64,
    pub wall_seconds: f64,
}

pub fn run() -> Result<(), AppError> {
    let mut input = String::new();
    io::stdin()
        .read_to_string(&mut input)
        .context("reading trial spec from stdin")?;
    let spec: TrialSpec = serde_json::from_str(&input).context("parsing trial spec JSON")?;

    if spec.seeds.is_empty() {
        return Err(AppError::from(anyhow!(
            "`seeds` must contain at least one value"
        )));
    }
    if spec.routes.is_empty() {
        return Err(AppError::from(anyhow!(
            "`routes` must contain at least one slug"
        )));
    }

    let routes_dir = spec
        .routes_dir
        .clone()
        .unwrap_or_else(|| PathBuf::from(DEFAULT_ROUTES_DIR));
    let shared_search_path = routes_dir.join("_search.toml");

    let trial_start = Instant::now();
    let mut per_route = Vec::with_capacity(spec.routes.len());
    for slug in &spec.routes {
        per_route.push(run_route(slug, &routes_dir, &shared_search_path, &spec)?);
    }
    let wall_seconds = trial_start.elapsed().as_secs_f64();

    let route_means: Vec<f64> = per_route.iter().map(|r| r.fitness_mean).collect();
    let fitness_mean = mean(&route_means);
    let fitness_std = stddev(&route_means, fitness_mean);

    let result = TrialResult {
        fitness_mean,
        fitness_std,
        wall_seconds,
        per_route,
    };
    let stdout = io::stdout();
    let mut writer = stdout.lock();
    serde_json::to_writer_pretty(&mut writer, &result).context("writing result JSON")?;
    writer
        .write_all(b"\n")
        .context("writing trailing newline")?;
    Ok(())
}

/// Run all seeds on a single route. The baked grid is loaded once and
/// reused across seeds (moved in / out of `run_search_blocking_with_baked`
/// each iteration to satisfy its by-value contract).
fn run_route(
    slug: &str,
    routes_dir: &std::path::Path,
    shared_search_path: &std::path::Path,
    spec: &TrialSpec,
) -> Result<RouteResult, AppError> {
    let route_start = Instant::now();
    let route_dir = routes_dir.join(slug);
    let route_config_path = route_dir.join("config.toml");
    let baked_path = route_dir.join("baked.bk1");

    // Layer: defaults → shared `_search.toml` → route's `config.toml`.
    let mut cfg = CliConfigFile::default();
    if shared_search_path.exists() {
        cfg.merge_from(CliConfigFile::from_path(shared_search_path).map_err(anyhow::Error::from)?);
    }
    cfg.merge_from(CliConfigFile::from_path(&route_config_path).map_err(anyhow::Error::from)?);

    let start = cfg
        .run
        .start
        .ok_or_else(|| AppError::from(anyhow!("route `{slug}` has no [run].start in its TOML")))?;
    let end = cfg
        .run
        .end
        .ok_or_else(|| AppError::from(anyhow!("route `{slug}` has no [run].end in its TOML")))?;

    let mut boat_cfg = BoatConfig::default();
    cfg.boat.apply_to(&mut boat_cfg);
    boat_cfg.validate().map_err(anyhow::Error::from)?;

    let mut search_cfg = SearchConfig::default();
    cfg.search.apply_to(&mut search_cfg);
    if let Some(n) = cfg.run.waypoints {
        search_cfg.waypoint_count = bywind::WaypointCount::from_usize(n).ok_or_else(|| {
            AppError::from(anyhow!(
                "route `{slug}` has unsupported waypoint count {n} (must be 5/8/10/15/20/30/40/50/60)",
            ))
        })?;
    }
    if let Some(w) = cfg.run.time_weight {
        search_cfg.time_weight = w;
    }
    if let Some(w) = cfg.run.fuel_weight {
        search_cfg.fuel_weight = w;
    }
    if let Some(w) = cfg.run.land_weight {
        search_cfg.land_weight = w;
    }
    if let Some(v) = spec.params.inertia {
        search_cfg.inertia = v;
    }
    if let Some(v) = spec.params.cognitive_coeff {
        search_cfg.cognitive_coeff = v;
    }
    if let Some(v) = spec.params.social_coeff {
        search_cfg.social_coeff = v;
    }
    if let Some(v) = spec.params.path_kick_probability {
        search_cfg.path_kick_probability = v;
    }
    if let Some(v) = spec.params.path_kick_gamma_0_fraction {
        search_cfg.path_kick_gamma_0_fraction = v;
    }
    if let Some(v) = spec.params.path_kick_gamma_min_fraction {
        search_cfg.path_kick_gamma_min_fraction = v;
    }
    search_cfg.validate().map_err(anyhow::Error::from)?;

    let weights = SearchWeights {
        time_weight: search_cfg.time_weight,
        fuel_weight: search_cfg.fuel_weight,
        land_weight: search_cfg.land_weight,
    };

    let mut baked = read_baked(&baked_path)?;
    let map_bounds = map_bounds_from_baked(&baked);
    // The route's user bounds (if any) were already applied at bake time,
    // so the baked grid's extent IS the search domain.
    let route_bounds = map_bounds.to_route_bounds_with_step_fraction(
        start.into(),
        end.into(),
        search_cfg.step_distance_fraction,
    );

    let mut seed_results = Vec::with_capacity(spec.seeds.len());
    for &seed in &spec.seeds {
        let seed_start = Instant::now();
        search_cfg.seed = Some(seed);
        let result = run_search_blocking_with_baked(
            baked,
            route_bounds,
            search_cfg.waypoint_count,
            search_cfg.to_search_settings(),
            boat_cfg.to_boat(),
            weights,
            search_cfg.sdf_resolution_deg,
        )
        .map_err(|e| AppError::no_result(anyhow!("route `{slug}` seed {seed} — {e}",)))?;
        let evolution = result.route_evolution;
        baked = result.baked; // reclaim ownership for the next seed

        let last = evolution.iter_count().saturating_sub(1);
        let fitness = evolution
            .gbest_at(last)
            .ok_or_else(|| {
                AppError::internal(anyhow!("route `{slug}` seed {seed} produced no iterations",))
            })?
            .best_fit;
        seed_results.push(SeedResult {
            seed,
            fitness,
            wall_seconds: seed_start.elapsed().as_secs_f64(),
        });
    }

    let fits: Vec<f64> = seed_results.iter().map(|s| s.fitness).collect();
    let fitness_mean = mean(&fits);
    let fitness_std = stddev(&fits, fitness_mean);

    Ok(RouteResult {
        slug: slug.to_owned(),
        fitness_mean,
        fitness_std,
        seeds: seed_results,
        wall_seconds: route_start.elapsed().as_secs_f64(),
    })
}

fn read_baked(path: &std::path::Path) -> Result<BakedWindMap, AppError> {
    let file = std::fs::File::open(path).with_context(|| format!("opening {}", path.display()))?;
    let reader = std::io::BufReader::new(file);
    let baked =
        baked_codec::decode(reader).with_context(|| format!("decoding {}", path.display()))?;
    Ok(baked)
}

/// Reconstruct a `MapBounds` from a `BakedWindMap`'s grid extent. Mirrors
/// the helper in `search.rs` (which is private there); duplicated so the
/// tune-trial subcommand stays self-contained.
fn map_bounds_from_baked(baked: &BakedWindMap) -> MapBounds {
    let step = baked.step();
    let nx_steps = baked.nx().saturating_sub(1) as f64;
    let ny_steps = baked.ny().saturating_sub(1) as f64;
    MapBounds {
        bbox: LonLatBbox::new(
            baked.x_min(),
            baked.x_min() + nx_steps * step,
            baked.y_min(),
            baked.y_min() + ny_steps * step,
        ),
    }
}

fn mean(xs: &[f64]) -> f64 {
    if xs.is_empty() {
        return f64::NAN;
    }
    xs.iter().sum::<f64>() / xs.len() as f64
}

/// Population standard deviation (divides by N, not N-1). Matches what
/// the orchestrator's per-route baseline aggregation will use.
fn stddev(xs: &[f64], mean: f64) -> f64 {
    if xs.len() < 2 {
        return 0.0;
    }
    let variance: f64 = xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / xs.len() as f64;
    variance.sqrt()
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "tests rely on bit-exact comparisons of constant or stored f32/f64 values."
    )]
    use super::*;

    #[test]
    fn parse_trial_spec_minimal() {
        let json = r#"{"seeds":[1,2],"routes":["short-easy"]}"#;
        let spec: TrialSpec = serde_json::from_str(json).expect("valid");
        assert_eq!(spec.seeds, vec![1, 2]);
        assert_eq!(spec.routes, vec!["short-easy"]);
        assert!(spec.params.inertia.is_none());
        assert!(spec.routes_dir.is_none());
    }

    #[test]
    fn parse_trial_spec_full() {
        let json = r#"{
            "params": {"inertia":0.5,"cognitive_coeff":1.6,"social_coeff":1.4},
            "seeds": [42],
            "routes": ["short-easy", "archipelago"],
            "routes_dir": "profiling/tuning"
        }"#;
        let spec: TrialSpec = serde_json::from_str(json).expect("valid");
        assert_eq!(spec.params.inertia, Some(0.5));
        assert_eq!(spec.params.cognitive_coeff, Some(1.6));
        assert_eq!(spec.params.social_coeff, Some(1.4));
        assert_eq!(
            spec.routes_dir.as_deref(),
            Some(std::path::Path::new("profiling/tuning"))
        );
    }

    #[test]
    fn parse_trial_spec_rejects_unknown_field() {
        let json = r#"{"seeds":[1],"routes":["x"],"nonsense":42}"#;
        let err = serde_json::from_str::<TrialSpec>(json).expect_err("unknown field");
        assert!(err.to_string().contains("nonsense"), "got: {err}");
    }

    #[test]
    fn mean_and_stddev_match_textbook_values() {
        let xs = [-1000.0, -1100.0, -900.0];
        let m = mean(&xs);
        assert!((m - -1000.0).abs() < 1e-9);
        let s = stddev(&xs, m);
        // Population sd: sqrt(((-100)^2 + 0 + 100^2) / 3) = sqrt(20000/3) ≈ 81.65.
        assert!((s - 81.6496580927726).abs() < 1e-6, "got {s}");
    }

    #[test]
    fn stddev_handles_empty_and_single_element() {
        assert_eq!(stddev(&[], 0.0), 0.0);
        assert_eq!(stddev(&[42.0], 42.0), 0.0);
    }
}
