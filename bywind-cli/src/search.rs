//! `search` subcommand. Loads a wind map, runs `bywind::run_search_blocking`
//! against a config built up from defaults + TOML files + CLI flags, and
//! writes the gbest path as a `SavedSolution` JSON document.
//!
//! Configuration resolution order, lowest to highest precedence:
//!
//! 1. Compile-time defaults from `bywind::config::{BoatConfig,
//!    SearchConfig}::default()` — same as the GUI.
//! 2. `--config <toml>` files in the order given on the command line.
//!    Multiple files merge field-by-field; later wins.
//! 3. CLI flags (`--start`, `--bounds`, `--time-weight`, etc.).
//!
//! `<MAP>`, `--start`, and `--end` are hard-required, but any of the three
//! can come from the `[run]` section of a config file instead of the CLI.

use std::fs::File;
use std::io::{self, BufWriter, Write as _};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context as _, Result, anyhow};
use bywind::{
    BakedWindMap, BenchmarkRoute, BoatConfig, LonLatBbox, MapBounds, RouteBounds, RouteEvolution,
    SavedSolution, SearchConfig, SearchResult, SearchWeights, SegmentMetrics, WaypointCount,
    baked_codec, derive_route_bbox, format_bbox_flag, gbest_segment_metrics, landmass_grid,
    run_search_blocking, run_search_blocking_with_baked,
};

use bywind::fmt::{format_duration_breakdown, format_fuel_auto, format_land_km, format_pso_delta};

use crate::display::print_segment_table;
use crate::error::AppError;
use crate::parsing::parse_n_floats;
use bywind::scenario::{CliConfigFile, RunOverrides};

/// Flat bag of clap-parsed flags forwarded by `main.rs`. Strings are kept
/// in their CLI form (e.g. `"35.0,-50.0"`) and parsed inside `run` so the
/// "did the user pass anything?" question is uniformly answered by `Option`.
///
/// Derives `clap::Args` so the parent `Command::Search` variant can hold
/// this struct directly via `#[command(flatten)]`-style embedding, keeping
/// the field set (and the per-field doc comments that become `--help`
/// text) in one place.
#[derive(clap::Args, Debug)]
pub struct SearchArgs {
    /// Wind map path (any supported format). Optional if a `--config`
    /// file sets `[run].map`.
    pub map: Option<PathBuf>,
    /// Start point as `lon,lat` (degrees). Optional if a `--config`
    /// file sets `[run].start`.
    #[arg(long, value_name = "LON,LAT")]
    pub start: Option<String>,
    /// End point as `lon,lat` (degrees). Optional if a `--config`
    /// file sets `[run].end`.
    #[arg(long, value_name = "LON,LAT")]
    pub end: Option<String>,
    /// Search domain as `lon_min,lat_min,lon_max,lat_max`. Defaults to
    /// the wind map's full extent.
    #[arg(long, value_name = "LON_MIN,LAT_MIN,LON_MAX,LAT_MAX")]
    pub bounds: Option<String>,
    /// Waypoint count. Must be one of 5, 8, 10, 15, 20, 30, 40, 50, 60.
    #[arg(long, value_name = "N")]
    pub waypoints: Option<usize>,
    /// Fitness weight on travel time.
    #[arg(long, value_name = "F")]
    pub time_weight: Option<f64>,
    /// Fitness weight on fuel.
    #[arg(long, value_name = "F")]
    pub fuel_weight: Option<f64>,
    /// Fitness weight on land penalty.
    #[arg(long, value_name = "F")]
    pub land_weight: Option<f64>,
    /// RNG seed for a deterministic search run. Reproduces the same
    /// swarm trajectory given identical inputs. Used by the PSO-tuning
    /// study; for everyday use, leave unset and the search draws fresh
    /// OS entropy.
    #[arg(long, value_name = "N")]
    pub seed: Option<u64>,
    /// Outer-loop search topology. `gbest` (default) is the
    /// single-swarm global-best PSO; `niched` keeps each baseline
    /// cohort's bests separate; `ring` / `von_neumann` are local-best
    /// PSO variants where each particle's social attractor is the
    /// best of its index-neighbours (preserving diversity at the
    /// cost of slower convergence). Validated by clap's
    /// `PossibleValuesParser` against `Topology::ALL`; `resolve_config`
    /// maps it to a `Topology` via `FromStr`. `None` keeps whatever
    /// `SearchConfig` / TOML resolved to.
    #[arg(
        long,
        value_name = "NAME",
        value_parser = clap::builder::PossibleValuesParser::new(
            bywind::Topology::ALL.iter().map(|t| t.as_str()).collect::<Vec<_>>()
        ),
    )]
    pub topology: Option<String>,
    /// TOML config file. Repeatable; later files override earlier ones;
    /// CLI flags override the merged result.
    #[arg(long = "config", value_name = "TOML")]
    pub configs: Vec<PathBuf>,
    /// Output JSON path. If omitted, the JSON `SavedSolution` is written
    /// to stdout. The human-readable summary always goes to stderr.
    #[arg(long, short = 'o')]
    pub out: Option<PathBuf>,
    /// After baking the wind map, write the baked grid to this path so
    /// later `--load-baked` runs can reuse it. Useful for hyperparameter
    /// sweeps where every run otherwise re-bakes the same map.
    #[arg(long, value_name = "PATH")]
    pub save_baked: Option<PathBuf>,
    /// Skip the wind-map load and bake; read a previously
    /// `--save-baked`-written file instead. When given, `<MAP>` may be
    /// omitted (the bake's bbox is derived from the cache file).
    #[arg(long, value_name = "PATH", conflicts_with = "save_baked")]
    pub load_baked: Option<PathBuf>,
}

pub fn run(args: &SearchArgs) -> Result<(), AppError> {
    let resolved = resolve_config(args)?;
    let ResolvedConfig {
        map_path,
        start,
        end,
        bounds,
        boat_cfg,
        search_cfg,
    } = resolved;

    // Decide between bake-from-wind-map and load-baked-from-cache up front.
    // The fresh-bake path retains the loaded `TimedWindMap` for the actual
    // bake; the load-baked path goes straight to a `BakedWindMap`. Both
    // produce a `MapBounds` so the `--bounds` clamp + route-bounds
    // construction happens uniformly downstream.
    let source = if let Some(baked_path) = &args.load_baked {
        WindSource::Cached(read_baked_with_log(baked_path)?)
    } else {
        let map_path = map_path.as_deref().ok_or_else(|| {
            AppError::from(anyhow!(
                "no map specified (pass <MAP>, set [run].map in a --config file, or use --load-baked <PATH>)",
            ))
        })?;
        WindSource::Fresh(load_wind_map_with_log(map_path)?)
    };
    // If the user didn't pass `--bounds`, derive a sensible bbox from
    // the endpoints + landmass and log it (in human-readable form and
    // as a copy-pasteable `--bbox` flag) so subsequent runs can pin the
    // exact same area for reproducibility.
    let bounds = match bounds {
        Some(b) => Some(b),
        None => derive_and_log_bounds(start.into(), end.into(), source.map_bounds()),
    };
    let map_bounds = clamp_with_bounds(source.map_bounds(), bounds)?;
    let route_bounds = map_bounds.to_route_bounds_with_step_fraction(
        start.into(),
        end.into(),
        search_cfg.step_distance_fraction,
    );

    let weights = SearchWeights {
        time_weight: search_cfg.time_weight,
        fuel_weight: search_cfg.fuel_weight,
        land_weight: search_cfg.land_weight,
    };
    eprintln!(
        "running search: {} waypoints, {}x{} space PSO, {}x{} time PSO...",
        search_cfg.waypoint_count.as_usize(),
        search_cfg.particles_space,
        search_cfg.iter_space,
        search_cfg.particles_time,
        search_cfg.iter_time,
    );
    let total_start = Instant::now();
    let SearchResult {
        route_evolution,
        route_bounds: route_bounds_out,
        baked,
        boat,
        benchmark,
        bake_duration: bake_dur,
        search_duration: search_dur,
    } = with_progress_watcher(|| {
        execute_search(
            source,
            &map_bounds,
            route_bounds,
            &boat_cfg,
            &search_cfg,
            weights,
            args.save_baked.as_deref(),
        )
    })?;
    let total_dur = total_start.elapsed();

    let last_iter = route_evolution.iter_count().saturating_sub(1);
    let segment_stats = gbest_segment_metrics(
        &route_evolution,
        last_iter,
        &boat,
        &baked,
        route_bounds_out.step_distance_max,
    )
    .ok_or_else(|| AppError::internal(anyhow!("search produced no iterations")))?;

    let saved = build_saved_solution(&route_evolution, &search_cfg)?;
    write_solution(&saved, args.out.as_deref())?;
    print_summary(
        &saved,
        &segment_stats,
        benchmark.as_ref(),
        bake_dur,
        search_dur,
        total_dur,
    );

    Ok(())
}

/// Inputs ready for the search itself, after merging defaults / TOML / CLI
/// flags and validating ranges. `map_path` is `None` when `--load-baked` is
/// in play and there's no wind map to load.
struct ResolvedConfig {
    map_path: Option<PathBuf>,
    start: [f64; 2],
    end: [f64; 2],
    bounds: Option<[f64; 4]>,
    boat_cfg: BoatConfig,
    search_cfg: SearchConfig,
}

/// Run `f` with a side thread that prints "still searching..." every five
/// seconds to stderr until `f` returns. Gives the user liveness feedback
/// during long searches without an upstream callback hook in
/// `swarmkit-sailing::search`. The watcher uses `park_timeout` so the
/// shutdown is immediate when `f` returns rather than waiting out a final
/// sleep cycle.
fn with_progress_watcher<R>(f: impl FnOnce() -> R) -> R {
    let stop = Arc::new(AtomicBool::new(false));
    let stop_watcher = Arc::clone(&stop);
    let watcher = std::thread::spawn(move || {
        let start = Instant::now();
        let interval = Duration::from_secs(5);
        while !stop_watcher.load(Ordering::Relaxed) {
            std::thread::park_timeout(interval);
            // Re-check after waking so a near-instant search doesn't print
            // a spurious "still searching" message right before completing.
            if !stop_watcher.load(Ordering::Relaxed) {
                eprintln!(
                    "  still searching... {:.0}s elapsed",
                    start.elapsed().as_secs_f64(),
                );
            }
        }
    });
    let result = f();
    stop.store(true, Ordering::Relaxed);
    watcher.thread().unpark();
    // Watcher thread can't panic — we only park/unpark and read an atomic.
    drop(watcher.join());
    result
}

/// What the user pointed us at: a fresh wind map to bake, or a previously
/// baked grid loaded from disk. The retained data avoids reloading the
/// wind map a second time when we hand it to `run_search_blocking`.
enum WindSource {
    Fresh(bywind::TimedWindMap),
    Cached(BakedWindMap),
}

impl WindSource {
    fn map_bounds(&self) -> MapBounds {
        match self {
            // For the fresh path we fall back to a sentinel-like default
            // *only* if `from_wind_map` produced None; that case is caught
            // immediately downstream via `is_non_degenerate`. (The
            // `expect` here would only fire on an empty wind map, which
            // also fails `is_non_degenerate` — we'd produce a clearer
            // error there.)
            Self::Fresh(map) => MapBounds::from_wind_map(map).unwrap_or(MapBounds {
                bbox: LonLatBbox::new(0.0, 0.0, 0.0, 0.0),
            }),
            Self::Cached(baked) => map_bounds_from_baked(baked),
        }
    }
}

fn load_wind_map_with_log(map_path: &Path) -> Result<bywind::TimedWindMap, AppError> {
    eprintln!("loading {}...", map_path.display());
    let load_start = Instant::now();
    let map = bywind::io::load(map_path, 1, None)
        .with_context(|| format!("loading wind map from {}", map_path.display()))?;
    eprintln!(
        "  loaded in {:.2}s: {} frame(s), step = {} s",
        load_start.elapsed().as_secs_f64(),
        map.frame_count(),
        map.step_seconds(),
    );
    Ok(map)
}

fn read_baked_with_log(baked_path: &Path) -> Result<BakedWindMap, AppError> {
    eprintln!("loading baked cache from {}...", baked_path.display());
    let load_start = Instant::now();
    let file = std::fs::File::open(baked_path)
        .with_context(|| format!("opening {}", baked_path.display()))?;
    let reader = std::io::BufReader::new(file);
    let baked = baked_codec::decode(reader)
        .with_context(|| format!("decoding baked cache from {}", baked_path.display()))?;
    eprintln!(
        "  loaded in {:.2}s: {} × {} × {} grid",
        load_start.elapsed().as_secs_f64(),
        baked.nx(),
        baked.ny(),
        baked.nt(),
    );
    Ok(baked)
}

/// Derive an auto-bbox from the endpoints + landmass when the user
/// didn't pass `--bounds`. Logs the result both as human-readable text
/// and as a copy-pasteable `--bounds` flag so subsequent runs (e.g. a
/// `--save-baked`/`--load-baked` sweep) can pin the exact same area.
/// Returns the derived bbox in the same `[lon_min, lat_min, lon_max,
/// lat_max]` shape that `--bounds` parses into; `None` if the auto
/// derivation fails (currently only when origin == destination).
fn derive_and_log_bounds(
    start: (f64, f64),
    end: (f64, f64),
    map_bounds: MapBounds,
) -> Option<[f64; 4]> {
    let derived = derive_route_bbox(start, end, landmass_grid(), Some(map_bounds))?;
    let flag = format_bbox_flag(derived);
    let b = derived.bbox;
    eprintln!(
        "auto-bounds: lon [{:.4}, {:.4}] lat [{:.4}, {:.4}]  (--bounds={})",
        b.lon_min, b.lon_max, b.lat_min, b.lat_max, flag,
    );
    Some([b.lon_min, b.lat_min, b.lon_max, b.lat_max])
}

/// Apply a user-supplied `--bounds` clamp on top of the map-derived
/// `MapBounds`. Errors out if the result is degenerate (e.g. the user's
/// box doesn't intersect the map at all, or the wind map was empty).
fn clamp_with_bounds(
    map_bounds: MapBounds,
    bounds: Option<[f64; 4]>,
) -> Result<MapBounds, AppError> {
    let clamped = match bounds {
        Some([lon_min, lat_min, lon_max, lat_max]) => {
            // `MapBounds::clamp_to` expects (lon_min, lon_max, lat_min, lat_max);
            // our schema is the more familiar [SW lon/lat, NE lon/lat] form.
            map_bounds.clamp_to(Some((lon_min, lon_max, lat_min, lat_max)))
        }
        None => map_bounds,
    };
    if !clamped.is_non_degenerate() {
        let b = clamped.bbox;
        return Err(AppError::no_result(anyhow!(
            "search bounds are degenerate after intersecting with the wind map ({:.3}..{:.3} lon, {:.3}..{:.3} lat) — check --bounds vs the map's extent",
            b.lon_min,
            b.lon_max,
            b.lat_min,
            b.lat_max,
        )));
    }
    Ok(clamped)
}

/// Reconstruct a `MapBounds` from a `BakedWindMap`'s grid extent.
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

fn write_baked_with_log(path: &Path, baked: &BakedWindMap) -> Result<(), AppError> {
    eprintln!("saving baked cache to {}...", path.display());
    let save_start = Instant::now();
    let file =
        std::fs::File::create(path).with_context(|| format!("creating {}", path.display()))?;
    let writer = std::io::BufWriter::new(file);
    baked_codec::encode(baked, writer)
        .with_context(|| format!("encoding baked cache to {}", path.display()))?;
    eprintln!("  saved in {:.2}s", save_start.elapsed().as_secs_f64());
    Ok(())
}

/// Run the search via the appropriate bywind entry point depending on the
/// `WindSource` variant. Saves the baked grid when `save_baked` is set
/// (only meaningful on the fresh-bake path).
fn execute_search(
    source: WindSource,
    map_bounds: &MapBounds,
    route_bounds: RouteBounds,
    boat_cfg: &BoatConfig,
    search_cfg: &SearchConfig,
    weights: SearchWeights,
    save_baked: Option<&Path>,
) -> Result<SearchResult, AppError> {
    match source {
        WindSource::Cached(baked) => run_search_blocking_with_baked(
            baked,
            route_bounds,
            search_cfg.waypoint_count,
            search_cfg.to_search_settings(),
            boat_cfg.to_boat(),
            weights,
            search_cfg.sdf_resolution_deg,
        )
        .map_err(|e| AppError::no_result(anyhow!("{e}"))),
        WindSource::Fresh(map) => {
            let bake_bounds = map_bounds.to_bake_bounds(search_cfg.bake_step_deg);
            let result = run_search_blocking(
                &map,
                bake_bounds,
                route_bounds,
                search_cfg.waypoint_count,
                search_cfg.to_search_settings(),
                boat_cfg.to_boat(),
                weights,
                search_cfg.sdf_resolution_deg,
            )
            .map_err(|e| AppError::no_result(anyhow!("{e}")))?;
            if let Some(save_path) = save_baked {
                write_baked_with_log(save_path, &result.baked)?;
            }
            Ok(result)
        }
    }
}

/// Layer config-file overrides + CLI-flag overrides onto compile-time
/// defaults, then validate. Returns either a fully-resolved config or an
/// error explaining what's missing or out of range.
fn resolve_config(args: &SearchArgs) -> Result<ResolvedConfig> {
    let mut cfg = CliConfigFile::default();
    for path in &args.configs {
        cfg.merge_from(CliConfigFile::from_path(path).map_err(anyhow::Error::from)?);
    }
    cfg.run.merge_from(cli_run_overrides(args)?);

    // `<MAP>` is required from somewhere unless the caller passes
    // `--load-baked`. We can't see that flag from here, so resolve `map_path`
    // as `Option` and let `run` decide whether the absence is an error.
    let map_path = cfg.run.map.clone();
    let start = cfg.run.start.ok_or_else(|| {
        anyhow!("no start point specified (pass --start LON,LAT or set [run].start)")
    })?;
    let end = cfg
        .run
        .end
        .ok_or_else(|| anyhow!("no end point specified (pass --end LON,LAT or set [run].end)"))?;

    let mut boat_cfg = BoatConfig::default();
    cfg.boat.apply_to(&mut boat_cfg);

    let mut search_cfg = SearchConfig::default();
    cfg.search.apply_to(&mut search_cfg);
    if let Some(n) = cfg.run.waypoints {
        search_cfg.waypoint_count = WaypointCount::from_usize(n).ok_or_else(|| {
            anyhow!("waypoint count {n} is not supported (must be one of 5, 8, 10, 15, 20, 30, 40, 50, 60)")
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
    // `--seed` lives in `[search]` in TOML, not `[run]`, but it's still
    // a CLI flag for ergonomics — applied last so it overrides both
    // defaults and TOML.
    if let Some(s) = args.seed {
        search_cfg.seed = Some(s);
    }
    // Same pattern for `--topology`. clap's `PossibleValuesParser` has
    // already validated the string against `Topology::ALL`, so the
    // parse won't fail in practice; `?` would surface a misconfigured
    // CLI rather than panic if it ever did.
    if let Some(t) = args.topology.as_deref() {
        search_cfg.topology = t.parse().map_err(anyhow::Error::from)?;
    }

    boat_cfg.validate().map_err(anyhow::Error::from)?;
    search_cfg.validate().map_err(anyhow::Error::from)?;

    Ok(ResolvedConfig {
        map_path,
        start,
        end,
        bounds: cfg.run.bounds,
        boat_cfg,
        search_cfg,
    })
}

/// Build a `RunOverrides` from the clap-parsed CLI flags. String fields are
/// parsed here so callers see the same `Option<typed>` shape as TOML files.
fn cli_run_overrides(args: &SearchArgs) -> Result<RunOverrides> {
    Ok(RunOverrides {
        map: args.map.clone(),
        start: args
            .start
            .as_deref()
            .map(|s| parse_lonlat(s, "--start"))
            .transpose()?,
        end: args
            .end
            .as_deref()
            .map(|s| parse_lonlat(s, "--end"))
            .transpose()?,
        bounds: args.bounds.as_deref().map(parse_bounds_4).transpose()?,
        waypoints: args.waypoints,
        time_weight: args.time_weight,
        fuel_weight: args.fuel_weight,
        land_weight: args.land_weight,
    })
}

/// Parse a `lon,lat` string into a `[lon, lat]` array. The resolved
/// config carries `[f64; 2]` shape end-to-end, so we return an array
/// rather than a tuple.
fn parse_lonlat(s: &str, flag: &str) -> Result<[f64; 2]> {
    parse_n_floats(s, ["lon", "lat"], flag)
}

/// Parse a `lon_min,lat_min,lon_max,lat_max` string into a `[f64; 4]`.
fn parse_bounds_4(s: &str) -> Result<[f64; 4]> {
    parse_n_floats(s, ["lon_min", "lat_min", "lon_max", "lat_max"], "--bounds")
}

/// Extract the gbest path at the final iteration and pair it with the
/// search params from `search_cfg` for serialisation.
fn build_saved_solution(
    route_evolution: &RouteEvolution,
    search_cfg: &SearchConfig,
) -> Result<SavedSolution, AppError> {
    let last_iter = route_evolution.iter_count().saturating_sub(1);
    let gbest = route_evolution
        .gbest_at(last_iter)
        .ok_or_else(|| AppError::internal(anyhow!("search produced no iterations")))?;
    Ok(SavedSolution {
        n: gbest.xs.len(),
        xs: gbest.xs.to_vec(),
        ys: gbest.ys.to_vec(),
        ts: gbest.ts.to_vec(),
        best_fit: gbest.best_fit,
        time_weight: search_cfg.time_weight,
        fuel_weight: search_cfg.fuel_weight,
        particles_space: search_cfg.particles_space,
        particles_time: search_cfg.particles_time,
        iter_space: search_cfg.iter_space,
        iter_time: search_cfg.iter_time,
        seed: search_cfg.seed,
        topology: search_cfg.topology,
        path_kick_probability: search_cfg.path_kick_probability,
        path_kick_gamma_0_fraction: search_cfg.path_kick_gamma_0_fraction,
        path_kick_gamma_min_fraction: search_cfg.path_kick_gamma_min_fraction,
    })
}

/// Write `SavedSolution` as pretty-printed JSON. Goes to `--out` if provided,
/// otherwise to stdout (so `bywind-cli search ... > result.json` works
/// without flags).
fn write_solution(saved: &SavedSolution, out: Option<&Path>) -> Result<()> {
    if let Some(path) = out {
        let file = File::create(path).with_context(|| format!("creating {}", path.display()))?;
        let mut writer = BufWriter::new(file);
        serde_json::to_writer_pretty(&mut writer, saved)
            .with_context(|| format!("writing {}", path.display()))?;
        writer.write_all(b"\n")?;
        writer.flush()?;
    } else {
        let stdout = io::stdout();
        let mut writer = stdout.lock();
        serde_json::to_writer_pretty(&mut writer, saved).context("writing solution to stdout")?;
        writer.write_all(b"\n")?;
    }
    Ok(())
}

fn print_summary(
    saved: &SavedSolution,
    segment_stats: &[SegmentMetrics],
    benchmark: Option<&BenchmarkRoute>,
    bake_dur: std::time::Duration,
    search_dur: std::time::Duration,
    total_dur: std::time::Duration,
) {
    let total_time: f64 = segment_stats.iter().map(|m| m.time).sum();
    let total_fuel: f64 = segment_stats.iter().map(|m| m.fuel).sum();
    let total_land_metres: f64 = segment_stats.iter().map(|m| m.land_metres).sum();

    eprintln!();
    eprintln!("=== Summary ===");
    eprintln!("Waypoints:  {}", saved.n);
    eprintln!("Total time: {}", format_duration_breakdown(total_time));
    eprintln!("Total fuel: {}", format_fuel_auto(total_fuel));
    eprintln!("Total land: {}", format_land_km(total_land_metres));
    eprintln!("Fitness:    {:.4}", saved.best_fit);
    eprintln!("Bake:       {:.2}s", bake_dur.as_secs_f64());
    eprintln!("Search:     {:.2}s", search_dur.as_secs_f64());
    eprintln!("Total time elapsed: {:.2}s", total_dur.as_secs_f64());

    eprintln!();
    eprintln!("=== Segments ===");
    print_segment_table(segment_stats);

    eprintln!();
    if let Some(b) = benchmark {
        eprintln!("=== Benchmark (A* sea path + time PSO) ===");
        eprintln!(
            "Bench time: {}  ({})",
            format_duration_breakdown(b.total_time),
            format_pso_delta(total_time, b.total_time, false),
        );
        eprintln!(
            "Bench fuel: {}  ({})",
            format_fuel_auto(b.total_fuel),
            format_pso_delta(total_fuel, b.total_fuel, false),
        );
        eprintln!(
            "Bench land: {}  ({})",
            format_land_km(b.total_land_metres),
            format_pso_delta(total_land_metres, b.total_land_metres, false),
        );
        eprintln!(
            "Bench fit:  {:.4}  ({})",
            b.fitness,
            format_pso_delta(saved.best_fit, b.fitness, true),
        );
    } else {
        eprintln!(
            "(no A* benchmark — endpoints may be landlocked or no sea path inside the bounds)"
        );
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "tests rely on bit-exact comparisons of constant or stored f32/f64 values."
    )]
    use super::*;

    // Parse-time edge cases (arity, whitespace, non-numeric) live in
    // `crate::parsing::tests`. The wrappers below are thin enough that
    // a single round-trip per shape is sufficient regression coverage.
    #[test]
    fn parse_lonlat_returns_two_element_array() {
        let r = parse_lonlat("35.0,-50.0", "--start").expect("valid");
        assert_eq!(r, [35.0, -50.0]);
    }

    #[test]
    fn parse_bounds_4_returns_four_element_array() {
        let r = parse_bounds_4("-60,25,10,45").expect("valid");
        assert_eq!(r, [-60.0, 25.0, 10.0, 45.0]);
    }
}
