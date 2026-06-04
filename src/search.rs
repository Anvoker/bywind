//! Pure, blocking entry points for the sailing search and time-only PSO.
//! Callers wrap them in their preferred concurrency model (GUI: worker
//! thread + mpsc; CLI: main thread).

use swarmkit::FitCalc as _;
use swarmkit_sailing::{
    Boat, EnsembleSailboatFitCalc, LandmassSource, Path, PathBaseline, RobustObjective,
    RouteBounds, SailboatFitCalc, SeaPathBias, SearchSettings, get_segment_fuel_and_time,
    get_segment_land_metres, reoptimize_times, search_with_progress, walk_segments_against_wind,
    weighted_fitness,
};

use crate::config::EnsembleMode;
use crate::ensemble::BakedEnsembleWindMap;
use crate::landmass::{landmass_grid_at_resolution, landmass_grid_two_tier};
use crate::metrics::{SegmentMetrics, compute_segment_metrics};
use crate::route::{BenchmarkRoute, RouteEvolution, WaypointCount, debug_assert_path_no_nans};
use crate::wind_map::{BakeBounds, BakedWindMap, TimedWindMap};
use crate::{route_evolution_match, waypoint_match};

/// Bake-grid cell size in degrees of lon / lat. 0.25° matches typical
/// GFS resolution; the bake-bounds builder grows this past the
/// requested value if needed to stay under the per-axis cell cap.
pub const BAKE_STEP: f64 = 0.25;

/// PSO outputs + per-phase timings. `bake_duration` is
/// `Duration::ZERO` for the pre-baked entry point.
pub struct SearchResult {
    pub route_evolution: RouteEvolution,
    pub route_bounds: RouteBounds,
    pub baked: BakedWindMap,
    pub boat: Boat,
    pub benchmark: Option<BenchmarkRoute>,
    pub bake_duration: std::time::Duration,
    pub search_duration: std::time::Duration,
    /// `Some` for ensemble searches: K per-member evaluations of the
    /// converged gbest path. `None` for single-deterministic.
    pub ensemble: Option<EnsembleSpread>,
}

/// Per-member spread of the converged gbest path's metrics — one path,
/// K winds.
///
/// For each member the gbest `xy` is held fixed and the segment-time
/// array `t` is re-optimised against that member's wind via
/// [`reoptimize_times`]. The resulting `(time, fuel)` is what each
/// member's optimal scheduling of the same spatial route would look
/// like. Narrow spread → robust route; wide spread → brittle.
///
/// Per-member time-reopt is needed because
/// [`walk_segments_against_wind`] sums `seg.segment_time` from the
/// path's `t` array, not from the wind — without it, every member
/// shares `time_s = sum(gbest.t)` and the time axis of the spread
/// collapses to a single value.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct EnsembleSpread {
    pub per_member: Vec<MemberMetrics>,
}

/// One ensemble member's optimal-schedule evaluation of the converged
/// gbest route.
///
/// See [`EnsembleSpread`] for the reopt protocol. `land_m` is
/// shared across members (wind-independent) but stored per-member for
/// symmetry with the aggregate display layer.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct MemberMetrics {
    /// Filename stem of the source `.wcav`, e.g. `"gec00"` or
    /// `"gep08"`. Lets the UI show "worst-case: gep23, best-case:
    /// gec00" without making the user count indices.
    pub name: String,
    /// Total travel time in seconds for this member: the sum of the
    /// per-segment durations chosen by the per-member time-reopt
    /// against this member's wind.
    pub time_s: f64,
    /// Fuel consumed in kg.
    pub fuel_kg: f64,
    /// Over-land distance in metres. Wind-independent; same across
    /// every member by construction. Replicated for display
    /// uniformity.
    pub land_m: f64,
    /// Per-member fitness via `weighted_fitness(time, fuel, land,
    /// time_weight, fuel_weight, land_weight)`. Negated cost — higher
    /// is better.
    pub fitness: f64,
}

impl EnsembleSpread {
    /// `(mean, min, max, stddev)` of `field` across members.
    /// Returns `(NaN, NaN, NaN, NaN)` for an empty ensemble (which
    /// shouldn't happen — `Self` is constructed with K ≥ 1).
    pub fn stats<F: Fn(&MemberMetrics) -> f64>(&self, field: F) -> (f64, f64, f64, f64) {
        let k = self.per_member.len();
        if k == 0 {
            return (f64::NAN, f64::NAN, f64::NAN, f64::NAN);
        }
        let values: Vec<f64> = self.per_member.iter().map(&field).collect();
        let mean = values.iter().sum::<f64>() / k as f64;
        let min = values.iter().copied().fold(f64::INFINITY, f64::min);
        let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let var = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / k as f64;
        (mean, min, max, var.sqrt())
    }
}

/// One realization run.
///
/// An independent PSO search against a single ensemble member treated
/// as ground truth. The artifact is a gbest route specific to that
/// realization — distinct from the main search's single converged
/// gbest aggregated across the whole ensemble.
///
/// See [`run_realizations`] for how K of these are produced from a
/// baked ensemble.
pub struct RealizationRun {
    /// Source member's `.wcav` filename stem (e.g. `"gec00"` or
    /// `"gep07"`).
    pub name: String,
    /// Full evolution from this realization's PSO search.
    pub route_evolution: RouteEvolution,
    /// Per-segment metrics for the final-iteration gbest of this
    /// realization's search, computed against the source member's
    /// baked wind at search-completion time.
    pub segment_stats: Vec<SegmentMetrics>,
    /// Final-iteration gbest `best_fit` (negated cost; higher is
    /// better).
    pub fitness: f64,
}

/// Run K independent realization searches against a baked ensemble.
///
/// One per member, treated as if its wind were ground truth. Each
/// iteration uses `base_seed.wrapping_add(k as u64)` as the search
/// seed so identically-initialised swarms don't collapse near-
/// identical winds to bit-identical routes.
///
/// Sequential rather than rayon-parallel: each inner search is
/// already internally parallel via rayon's global pool, so nesting
/// would oversubscribe cores. On K=3 GEFS this is ~3× the wallclock
/// of a single search; on K=31 (full GEFS) it's ~31×.
///
/// First failure short-circuits the whole cohort (returns the error)
/// so an infeasible-for-one-realization wind doesn't quietly publish
/// a partial overlay.
///
/// # Errors
/// Forwards the first [`SearchError`] returned by the inner
/// [`run_search_blocking_with_baked`] call.
#[expect(
    clippy::too_many_arguments,
    reason = "matches the inner search-blocking signature one-for-one; \
              bundling args would just shuffle them around"
)]
pub fn run_realizations(
    ensemble: &BakedEnsembleWindMap,
    route_bounds: RouteBounds,
    waypoint_count: WaypointCount,
    mut search_settings: SearchSettings,
    base_seed: u64,
    ship: Boat,
    weights: SearchWeights,
    sdf_resolution_deg: f64,
    fine_sdf_resolution_deg: Option<f64>,
) -> Result<Vec<RealizationRun>, SearchError> {
    let names = ensemble.member_names().to_vec();
    let mut runs = Vec::with_capacity(names.len());
    // Per-realization runs don't emit progress upward: each one
    // computes its own benchmark, which is meaningful per-realization
    // but doesn't map onto the main-search "live bench overlay" UX.
    // Bench is the *main* search's reference; realization overlays
    // would just stomp on it.
    let mut no_progress = |_: SearchProgressEvent| {};
    for (k, name) in names.iter().enumerate() {
        search_settings.seed = Some(base_seed.wrapping_add(k as u64));
        let member_baked = ensemble.member(k).clone();
        let result = run_search_blocking_with_baked(
            WindInput::single(member_baked),
            route_bounds,
            waypoint_count,
            search_settings,
            ship,
            weights,
            sdf_resolution_deg,
            fine_sdf_resolution_deg,
            &mut no_progress,
        )?;
        // Pre-compute segment metrics + final fitness against the
        // realization's wind while we still own the baked grid. The
        // search returns ownership of `result.baked` and it's about
        // to be dropped at the end of this iteration; sampling it
        // here is the only chance to capture per-realization stats
        // without re-baking later.
        let (segment_stats, fitness) = route_evolution_match!(&result.route_evolution, |evo| {
            let frames = evo.frames();
            let last = frames
                .last()
                .expect("search returns at least one iteration");
            let best = last
                .iter()
                .max_by(|a, b| {
                    a.best_fit
                        .partial_cmp(&b.best_fit)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .expect("each iteration has at least one particle");
            let stats = compute_segment_metrics(
                &result.boat,
                &result.baked,
                best.best_pos,
                route_bounds.step_distance_max,
            );
            (stats, best.best_fit)
        });
        runs.push(RealizationRun {
            name: name.clone(),
            route_evolution: result.route_evolution,
            segment_stats,
            fitness,
        });
    }
    Ok(runs)
}

/// Failure modes the blocking search entry points can return. Enum
/// shape lets future variants land without breaking the public
/// `Result` signature.
#[derive(Debug, Clone, PartialEq)]
pub enum SearchError {
    /// Every particle converged with non-finite gbest fitness — every
    /// candidate xy in the bbox had at least one physically untraversable
    /// segment (pole-lock, dead calm against the transit direction,
    /// over-restrictive `RouteBounds`). `best_fit` retains the
    /// non-finite gbest for diagnostics.
    NoFeasibleRoute { best_fit: f64 },
}

impl std::fmt::Display for SearchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoFeasibleRoute { best_fit } => write!(
                f,
                "search produced no feasible route (best_fit = {best_fit}) — \
                 every candidate path had at least one segment the boat can't \
                 traverse in the given wind. Try widening the route bounds, \
                 relaxing the boat polar, or moving the endpoints out of any \
                 pole-locked region.",
            ),
        }
    }
}

impl std::error::Error for SearchError {}

/// Fitness weights for `SailboatFitCalc`. Grouped so the entry points
/// don't take three loose `f64`s.
#[derive(Clone, Copy, Debug)]
pub struct SearchWeights {
    pub time_weight: f64,
    pub fuel_weight: f64,
    pub land_weight: f64,
}

/// A* sea path → N-waypoint sample → time-PSO over fixed xy. `None`
/// when A* can't find a sea path (landlocked endpoints / bbox
/// excludes all water).
fn compute_benchmark<const N: usize, WS, LS>(
    ship: &Boat,
    wind_source: &WS,
    landmass: &LS,
    bounds: RouteBounds,
    fit_calc: &SailboatFitCalc<'_, N, Boat, WS, LS>,
    settings: SearchSettings,
) -> Option<BenchmarkRoute>
where
    WS: swarmkit_sailing::WindSource,
    LS: LandmassSource,
{
    let polyline = landmass.find_sea_path(
        bounds.origin,
        bounds.destination,
        &bounds,
        SeaPathBias::None,
    )?;

    // Land-respecting sampler: keeps consecutive-pair chords off land
    // even when `--waypoints` is too small for uniform arc-length to
    // follow coastline detours. PSO init uses the looser sampler — it
    // wants room for perpendicular kicks.
    let baseline = PathBaseline::<N>::from_polyline_land_respecting(&polyline, &bounds, landmass);

    // `t` is unused; `reoptimize_times` seeds fresh segment times from
    // the segment-range cache it builds internally.
    let mut path = Path::default();
    for i in 0..N {
        path.xy.0[i] = baseline.positions[i].lon;
        path.xy.1[i] = baseline.positions[i].lat;
    }

    let optimized = reoptimize_times(fit_calc, settings, path);

    let segment_metrics = get_segment_fuel_and_time(
        ship,
        wind_source,
        optimized,
        fit_calc.departure_time,
        fit_calc.step_distance_max,
    );
    let total_time: f64 = segment_metrics.iter().map(|(_, _, t)| *t).sum();
    let total_fuel: f64 = segment_metrics.iter().map(|(_, fuel, _)| *fuel).sum();
    // A* is land-aware but the straight-line chords between sampled
    // waypoints can still clip a coastline; sum them as a UI sanity
    // check (a correct benchmark reads zero).
    let total_land_metres: f64 = (0..N - 1)
        .map(|i| {
            let a = optimized.lat_lon(i);
            let b = optimized.lat_lon(i + 1);
            get_segment_land_metres(landmass, a, b, fit_calc.step_distance_max)
        })
        .sum();
    let fitness = fit_calc.calculate_fit(optimized);

    let waypoints: Vec<(f64, f64)> = (0..N)
        .map(|i| (optimized.xy.0[i], optimized.xy.1[i]))
        .collect();

    Some(BenchmarkRoute {
        waypoints,
        total_time,
        total_fuel,
        total_land_metres,
        fitness,
    })
}

/// Full sailing search.
///
/// Bakes `wind_map`, runs the const-generic-N PSO via
/// `waypoint_match!`, and computes the A*+time-PSO benchmark for
/// comparison. Pass [`crate::SDF_RESOLUTION_DEG`] for the default
/// landmass grid.
///
/// # Errors
/// [`SearchError::NoFeasibleRoute`] when the PSO converges with non-finite
/// gbest fitness (every candidate xy infeasible).
#[expect(
    clippy::too_many_arguments,
    reason = "Nine first-class inputs the caller picks independently; \
              a struct would just relocate the destructuring."
)]
pub fn run_search_blocking(
    wind_map: &TimedWindMap,
    bake_bounds: BakeBounds,
    route_bounds: RouteBounds,
    waypoint_count: WaypointCount,
    search_settings: SearchSettings,
    ship: Boat,
    weights: SearchWeights,
    sdf_resolution_deg: f64,
    fine_sdf_resolution_deg: Option<f64>,
    progress: &mut dyn FnMut(SearchProgressEvent),
) -> Result<SearchResult, SearchError> {
    progress(SearchProgressEvent::Phase(SearchPhase::Baking));
    let bake_start = std::time::Instant::now();
    let baked = wind_map.bake(bake_bounds);
    let bake_duration = bake_start.elapsed();
    let mut result = run_search_blocking_with_baked(
        WindInput::single(baked),
        route_bounds,
        waypoint_count,
        search_settings,
        ship,
        weights,
        sdf_resolution_deg,
        fine_sdf_resolution_deg,
        progress,
    )?;
    // Inner call zero-init'd `bake_duration`; patch in the real value.
    result.bake_duration = bake_duration;
    Ok(result)
}

/// In-flight progress notifications emitted by [`run_search_blocking`]
/// and [`run_search_blocking_with_baked`] over the course of a search.
///
/// A search worker can install a `&mut dyn FnMut(SearchProgressEvent)`
/// closure to receive these events as soon as the corresponding work
/// finishes — letting the UI paint partial results before the
/// terminal `SearchResult` lands. Today the variants cover phase
/// transitions and the early benchmark route; per-iteration gbest
/// updates may join later.
///
/// Variants are `#[non_exhaustive]` so new event kinds can land
/// without breaking existing match arms; consumers should always
/// include a default arm.
#[non_exhaustive]
#[derive(Clone, Debug)]
pub enum SearchProgressEvent {
    /// Search has transitioned into a new phase. Fires once per
    /// transition so consumers can update a status label without
    /// polling internal state. See [`SearchPhase`] for the meaning of
    /// each variant.
    Phase(SearchPhase),
    /// A* + time-PSO benchmark route, emitted **before** the main PSO
    /// starts. The benchmark is the cheap reference the user compares
    /// the main result against; surfacing it early lets the UI draw
    /// the dashed bench overlay while the search is still running. Not
    /// emitted when A* couldn't find a sea path (landlocked endpoints
    /// / bbox excludes all water); the search continues but there's
    /// no benchmark to compare against in that case.
    BenchmarkReady(BenchmarkRoute),
    /// One outer PSO iteration finished. `gbest_xs` / `gbest_ys` /
    /// `gbest_ts` carry the current best particle's `(lon, lat, t)`
    /// arrays — one entry per waypoint. Lets the UI paint the route
    /// as it converges instead of waiting for the terminal
    /// [`SearchResult`]. Fires once per outer iteration, so the total
    /// over a search equals `search_settings.max_iteration_space`.
    ///
    /// Const-generic `N` is type-erased into [`Vec<f64>`] at the event
    /// boundary so the variant is shape-stable across waypoint counts.
    Iteration {
        /// Zero-based outer iteration index that just completed.
        iter_idx: usize,
        /// Total outer iterations the search will run (the
        /// `max_iteration_space` from `SearchSettings`).
        total_iters: usize,
        /// `(lon, lat)` per waypoint and segment durations in seconds
        /// — see [`crate::route::RouteEvolution`] for the same layout
        /// in the terminal result.
        gbest_xs: Vec<f64>,
        gbest_ys: Vec<f64>,
        gbest_ts: Vec<f64>,
        /// Current best fitness (negated cost — higher is better).
        best_fit: f64,
    },
}

/// Coarse phase markers for the search pipeline.
///
/// Emitted by the `Phase` variant of [`SearchProgressEvent`] so the
/// UI can swap a generic "searching…" indicator for a specific
/// "baking…" / "computing benchmark…" label as each chunk of work
/// begins. `#[non_exhaustive]` so future intermediate phases (e.g.
/// an `EnsembleSpread` step) can join without breaking match arms.
#[non_exhaustive]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SearchPhase {
    /// Decoding wind data from disk. Only fires from consumers that
    /// own the I/O layer (the viz's ensemble worker) — the core
    /// search functions assume their inputs are already in memory.
    Loading,
    /// Rasterising a [`TimedWindMap`] (or K of them, for ensembles)
    /// into the search-side regular grid. Fires from [`run_search_blocking`]
    /// for the single-deterministic case and from the viz's ensemble
    /// worker for the K-member case (the latter also covers the
    /// `mean()` reduction that runs before the K-fold search starts).
    Baking,
    /// Computing the A* sea path + time-PSO refinement that becomes
    /// the [`BenchmarkRoute`] reference overlay. Always followed by
    /// [`SearchProgressEvent::BenchmarkReady`] when A* succeeds;
    /// followed directly by [`Self::RunningPso`] when it fails.
    Benchmark,
    /// Main particle-swarm optimisation. Today the only sub-event
    /// during this phase is `Phase(RunningPso)` itself at start;
    /// per-iteration gbest events may join later.
    RunningPso,
}

impl SearchPhase {
    /// Short lowercase label suitable for an in-progress status line
    /// (e.g. next to a spinner). Matches the existing fetch-dialog
    /// "fetching…" / "encoding…" tone.
    pub fn label(self) -> &'static str {
        match self {
            Self::Loading => "loading wind data…",
            Self::Baking => "baking wind grid…",
            Self::Benchmark => "computing benchmark…",
            Self::RunningPso => "searching…",
        }
    }
}

/// Per-search wind input: a single deterministic baked map, or an
/// ensemble of K baked members alongside a precomputed mean used for
/// the benchmark and fast-mode shortcut.
pub enum WindInput {
    /// One pre-baked wind map. Existing single-deterministic path.
    Single(BakedWindMap),
    /// Ensemble of K members + their pre-computed mean.
    Ensemble {
        ensemble: BakedEnsembleWindMap,
        mean: BakedWindMap,
        mode: EnsembleMode,
    },
}

impl WindInput {
    /// Convenience: wrap a single baked map.
    pub fn single(baked: BakedWindMap) -> Self {
        Self::Single(baked)
    }
    /// Convenience: wrap an ensemble in `Full` mode (computes the
    /// mean once and stashes it for the bench).
    pub fn ensemble_full(ensemble: BakedEnsembleWindMap) -> Self {
        let mean = ensemble.mean();
        Self::Ensemble {
            ensemble,
            mean,
            mode: EnsembleMode::Full,
        }
    }
    /// Convenience: wrap an ensemble in `FastMean` mode (computes the
    /// mean once and runs the search against it).
    pub fn ensemble_fast_mean(ensemble: BakedEnsembleWindMap) -> Self {
        let mean = ensemble.mean();
        Self::Ensemble {
            ensemble,
            mean,
            mode: EnsembleMode::FastMean,
        }
    }
}

/// Variant taking a pre-baked wind field. Used by `bywind-cli`'s
/// `--load-baked` flag for hyperparameter sweeps over the same map.
/// `bake_duration` in the result is `Duration::ZERO`.
///
/// # Errors
/// [`SearchError::NoFeasibleRoute`] when the PSO converges with non-finite
/// gbest fitness.
#[expect(
    clippy::too_many_arguments,
    reason = "Eight first-class inputs the caller picks independently; \
              the dispatch on `fine_sdf_resolution_deg` happens here so \
              callers stay single-line."
)]
pub fn run_search_blocking_with_baked(
    wind: WindInput,
    route_bounds: RouteBounds,
    waypoint_count: WaypointCount,
    search_settings: SearchSettings,
    ship: Boat,
    weights: SearchWeights,
    sdf_resolution_deg: f64,
    fine_sdf_resolution_deg: Option<f64>,
    progress: &mut dyn FnMut(SearchProgressEvent),
) -> Result<SearchResult, SearchError> {
    let search_start = std::time::Instant::now();
    // Resolve the wind input into one of two physical search modes:
    // single-baked or ensemble-with-mean-for-bench. FastMean folds
    // into single-baked by using the precomputed mean as the only
    // wind map.
    match wind {
        WindInput::Single(baked)
        | WindInput::Ensemble {
            mean: baked,
            mode: EnsembleMode::FastMean,
            ..
        } => match fine_sdf_resolution_deg {
            None => {
                let land = landmass_grid_at_resolution(sdf_resolution_deg);
                run_search_inner(
                    baked,
                    route_bounds,
                    waypoint_count,
                    search_settings,
                    ship,
                    weights,
                    land,
                    search_start,
                    progress,
                )
            }
            Some(fine) => {
                let land = landmass_grid_two_tier(sdf_resolution_deg, fine);
                run_search_inner(
                    baked,
                    route_bounds,
                    waypoint_count,
                    search_settings,
                    ship,
                    weights,
                    land,
                    search_start,
                    progress,
                )
            }
        },
        WindInput::Ensemble {
            ensemble,
            mean,
            mode: EnsembleMode::Full,
        } => match fine_sdf_resolution_deg {
            None => {
                let land = landmass_grid_at_resolution(sdf_resolution_deg);
                run_search_inner_ensemble(
                    ensemble,
                    mean,
                    route_bounds,
                    waypoint_count,
                    search_settings,
                    ship,
                    weights,
                    land,
                    search_start,
                    progress,
                )
            }
            Some(fine) => {
                let land = landmass_grid_two_tier(sdf_resolution_deg, fine);
                run_search_inner_ensemble(
                    ensemble,
                    mean,
                    route_bounds,
                    waypoint_count,
                    search_settings,
                    ship,
                    weights,
                    land,
                    search_start,
                    progress,
                )
            }
        },
    }
}

/// Generic search driver shared by the single-tier and two-tier code
/// paths in [`run_search_blocking_with_baked`]. Parameterised over the
/// concrete [`LandmassSource`] so the same body works for either
/// `&LandmassGrid` or `&TwoTierLandmass`.
#[expect(
    clippy::too_many_arguments,
    reason = "the outer entry point partitions inputs by ownership; \
              bundling them here would add a struct just to relocate them."
)]
#[expect(
    clippy::panic_in_result_fn,
    reason = "debug_assertions-only NaN guards catch upstream bugs before \
              they corrupt downstream rendering; release builds compile \
              them out."
)]
fn run_search_inner<LS: LandmassSource>(
    baked: BakedWindMap,
    route_bounds: RouteBounds,
    waypoint_count: WaypointCount,
    search_settings: SearchSettings,
    ship: Boat,
    weights: SearchWeights,
    land: &LS,
    search_start: std::time::Instant,
    progress: &mut dyn FnMut(SearchProgressEvent),
) -> Result<SearchResult, SearchError> {
    let (route_evolution, boat, benchmark, best_fit) = waypoint_match!(waypoint_count, N, wrap, {
        let fit_calc = SailboatFitCalc::<N, _, _, _> {
            time_weight: weights.time_weight,
            fuel_weight: weights.fuel_weight,
            land_weight: weights.land_weight,
            departure_time: 0.0,
            step_distance_max: route_bounds.step_distance_max,
            ship: &ship,
            wind_source: &baked,
            landmass: land,
        };
        // Compute the benchmark first so the UI can paint the A*-based
        // reference overlay before the main PSO kicks off. The bench
        // doesn't depend on the PSO result; it's an independent A* sea
        // path + time-PSO refinement. Emitting the event drops the
        // user-visible "search is alive" latency from "wait for the
        // full result" to "wait for A* + time-PSO" (typically <2 s).
        progress(SearchProgressEvent::Phase(SearchPhase::Benchmark));
        let benchmark = compute_benchmark::<N, _, _>(
            &ship,
            &baked,
            land,
            route_bounds,
            &fit_calc,
            search_settings,
        );
        if let Some(b) = &benchmark {
            progress(SearchProgressEvent::BenchmarkReady(b.clone()));
        }
        progress(SearchProgressEvent::Phase(SearchPhase::RunningPso));
        let total_iters = search_settings.max_iteration_space;
        let mut on_iter = |idx: usize, snapshot: &swarmkit::Best<Path<N>>| {
            progress(SearchProgressEvent::Iteration {
                iter_idx: idx,
                total_iters,
                gbest_xs: snapshot.best_pos.xy.0.0.to_vec(),
                gbest_ys: snapshot.best_pos.xy.1.0.to_vec(),
                gbest_ts: snapshot.best_pos.t.0.0.to_vec(),
                best_fit: snapshot.best_fit,
            });
        };
        let (gbest, evolution) = search_with_progress::<N, _, _, _, _>(
            &ship,
            &baked,
            land,
            route_bounds,
            &fit_calc,
            search_settings,
            &mut on_iter,
        );
        if cfg!(debug_assertions) {
            assert!(!gbest.best_fit.is_nan(), "NaN in gbest: best_fit");
            debug_assert_path_no_nans(&gbest.best_pos, "gbest.best_pos");
            for (iter_idx, particles) in evolution.frames().iter().enumerate() {
                for (p_idx, particle) in particles.iter().enumerate() {
                    assert!(
                        !particle.best_fit.is_nan(),
                        "NaN in evolution[{iter_idx}][{p_idx}]: best_fit",
                    );
                    debug_assert_path_no_nans(
                        &particle.best_pos,
                        &format!("evolution[{iter_idx}][{p_idx}].best_pos"),
                    );
                }
            }
        }
        (wrap(evolution), ship, benchmark, gbest.best_fit)
    });
    if !best_fit.is_finite() {
        return Err(SearchError::NoFeasibleRoute { best_fit });
    }
    let search_duration = search_start.elapsed();
    Ok(SearchResult {
        route_evolution,
        route_bounds,
        baked,
        boat,
        benchmark,
        bake_duration: std::time::Duration::ZERO,
        search_duration,
        ensemble: None,
    })
}

/// Ensemble counterpart of [`run_search_inner`]. PSO drives an
/// [`EnsembleSailboatFitCalc`] over the K-member ensemble; the
/// benchmark route still uses the existing single-wind
/// [`SailboatFitCalc`] against the precomputed mean wind map (cheaper
/// than K-fold inside the inner time-PSO, and the benchmark is a
/// reference, not the source of truth for ensemble fitness).
///
/// `SearchResult.baked` carries the mean wind map so the GUI's
/// per-segment metrics renderer (which queries one wind source) has
/// something concrete to consult. Callers who need access to the raw
/// ensemble can call the search through a higher-level API in a
/// future revision; the current pipeline only needs the mean for
/// display.
#[expect(
    clippy::too_many_arguments,
    reason = "outer dispatch partitions ownership; bundling would just relocate"
)]
#[expect(
    clippy::panic_in_result_fn,
    reason = "debug-only NaN guards catch upstream bugs"
)]
#[expect(
    clippy::needless_pass_by_value,
    reason = "ensemble is taken by value to make the dispatch's ownership \
              transfer explicit at the call site; reference plus drop() \
              clutter would obscure that. Cost: one extra move of an \
              owned struct that's about to be dropped anyway."
)]
fn run_search_inner_ensemble<LS: LandmassSource>(
    ensemble: BakedEnsembleWindMap,
    mean: BakedWindMap,
    route_bounds: RouteBounds,
    waypoint_count: WaypointCount,
    search_settings: SearchSettings,
    ship: Boat,
    weights: SearchWeights,
    land: &LS,
    search_start: std::time::Instant,
    progress: &mut dyn FnMut(SearchProgressEvent),
) -> Result<SearchResult, SearchError> {
    let (route_evolution, boat, benchmark, best_fit, ensemble_spread) =
        waypoint_match!(waypoint_count, N, wrap, {
            let ensemble_fit_calc = EnsembleSailboatFitCalc::<N, _, _, _> {
                time_weight: weights.time_weight,
                fuel_weight: weights.fuel_weight,
                land_weight: weights.land_weight,
                departure_time: 0.0,
                step_distance_max: route_bounds.step_distance_max,
                ship: &ship,
                wind_ensemble: &ensemble,
                landmass: land,
                robust_objective: RobustObjective::Mean,
            };
            // Benchmark uses single-wind fit calc against the mean — a
            // "mean-wind reference" the user can read alongside the
            // ensemble PSO result. Compute it first so the UI can paint
            // the dashed bench overlay before the K-fold PSO begins;
            // K-fold can take meaningful wallclock at K=31, and showing
            // the bench up-front lets the user see the search is alive.
            progress(SearchProgressEvent::Phase(SearchPhase::Benchmark));
            let bench_fit_calc = SailboatFitCalc::<N, _, _, _> {
                time_weight: weights.time_weight,
                fuel_weight: weights.fuel_weight,
                land_weight: weights.land_weight,
                departure_time: 0.0,
                step_distance_max: route_bounds.step_distance_max,
                ship: &ship,
                wind_source: &mean,
                landmass: land,
            };
            let benchmark = compute_benchmark::<N, _, _>(
                &ship,
                &mean,
                land,
                route_bounds,
                &bench_fit_calc,
                search_settings,
            );
            if let Some(b) = &benchmark {
                progress(SearchProgressEvent::BenchmarkReady(b.clone()));
            }
            // PSO uses ensemble K-fold; baselines / boundary repulsion
            // query the mean wind (single representative). The init code
            // only needs `WindSource::sample_wind`, not per-member
            // fitness — using the mean here keeps the baselines stable.
            progress(SearchProgressEvent::Phase(SearchPhase::RunningPso));
            let total_iters = search_settings.max_iteration_space;
            let mut on_iter = |idx: usize, snapshot: &swarmkit::Best<Path<N>>| {
                progress(SearchProgressEvent::Iteration {
                    iter_idx: idx,
                    total_iters,
                    gbest_xs: snapshot.best_pos.xy.0.0.to_vec(),
                    gbest_ys: snapshot.best_pos.xy.1.0.to_vec(),
                    gbest_ts: snapshot.best_pos.t.0.0.to_vec(),
                    best_fit: snapshot.best_fit,
                });
            };
            let (gbest, evolution) = search_with_progress::<N, _, _, _, _>(
                &ship,
                &mean,
                land,
                route_bounds,
                &ensemble_fit_calc,
                search_settings,
                &mut on_iter,
            );
            if cfg!(debug_assertions) {
                assert!(!gbest.best_fit.is_nan(), "NaN in gbest: best_fit");
                debug_assert_path_no_nans(&gbest.best_pos, "gbest.best_pos");
                for (iter_idx, particles) in evolution.frames().iter().enumerate() {
                    for (p_idx, particle) in particles.iter().enumerate() {
                        assert!(
                            !particle.best_fit.is_nan(),
                            "NaN in evolution[{iter_idx}][{p_idx}]: best_fit",
                        );
                        debug_assert_path_no_nans(
                            &particle.best_pos,
                            &format!("evolution[{iter_idx}][{p_idx}].best_pos"),
                        );
                    }
                }
            }
            // Per-member spread of the converged gbest path's metrics: hold
            // the spatial geometry `xy` fixed and time-reopt `t` against
            // each member's wind, then walk the reopt'd path for `(time,
            // fuel)`.
            // Without per-member time-reopt the time axis is constant —
            // `walk_segments_against_wind` integrates segment durations
            // from `path.t`, not the wind, so a fixed-path replay gives
            // every member the same `time_s = sum(gbest.t)`. Land penalty
            // is wind-independent — compute once and replicate.
            let gbest_pos = gbest.best_pos;
            let land_m: f64 = (0..N - 1)
                .map(|i| {
                    let a = gbest_pos.lat_lon(i);
                    let b = gbest_pos.lat_lon(i + 1);
                    get_segment_land_metres(land, a, b, route_bounds.step_distance_max)
                })
                .sum();
            let per_member: Vec<MemberMetrics> = (0..ensemble.member_count())
                .map(|k| {
                    let member = ensemble.member(k);
                    let member_fit_calc = SailboatFitCalc::<N, _, _, _> {
                        time_weight: weights.time_weight,
                        fuel_weight: weights.fuel_weight,
                        land_weight: weights.land_weight,
                        departure_time: 0.0,
                        step_distance_max: route_bounds.step_distance_max,
                        ship: &ship,
                        wind_source: member,
                        landmass: land,
                    };
                    let reopt_path = reoptimize_times(&member_fit_calc, search_settings, gbest_pos);
                    let (t, f) = walk_segments_against_wind(
                        &ship,
                        member,
                        reopt_path,
                        0.0,
                        route_bounds.step_distance_max,
                    );
                    let fitness = weighted_fitness(
                        t,
                        f,
                        land_m,
                        weights.time_weight,
                        weights.fuel_weight,
                        weights.land_weight,
                    );
                    MemberMetrics {
                        name: ensemble.member_names()[k].clone(),
                        time_s: t,
                        fuel_kg: f,
                        land_m,
                        fitness,
                    }
                })
                .collect();
            let ensemble_spread = EnsembleSpread { per_member };
            (
                wrap(evolution),
                ship,
                benchmark,
                gbest.best_fit,
                ensemble_spread,
            )
        });
    if !best_fit.is_finite() {
        return Err(SearchError::NoFeasibleRoute { best_fit });
    }
    let search_duration = search_start.elapsed();
    Ok(SearchResult {
        route_evolution,
        route_bounds,
        baked: mean,
        boat,
        benchmark,
        bake_duration: std::time::Duration::ZERO,
        search_duration,
        ensemble: Some(ensemble_spread),
    })
}

/// Time-only PSO. Re-optimises `path.t` with `path.xy` held fixed.
/// Pass [`crate::SDF_RESOLUTION_DEG`] for the default landmass grid.
/// `fine_sdf_resolution_deg = Some(f)` opts into the two-tier landmass.
#[expect(
    clippy::too_many_arguments,
    reason = "Eight first-class inputs the caller picks independently."
)]
pub fn run_time_reopt_blocking<const N: usize>(
    baked: &BakedWindMap,
    route_bounds: RouteBounds,
    settings: SearchSettings,
    ship: &Boat,
    fixed_path: Path<N>,
    weights: SearchWeights,
    sdf_resolution_deg: f64,
    fine_sdf_resolution_deg: Option<f64>,
) -> Path<N> {
    match fine_sdf_resolution_deg {
        None => {
            let land = landmass_grid_at_resolution(sdf_resolution_deg);
            run_time_reopt_inner(
                baked,
                route_bounds,
                settings,
                ship,
                fixed_path,
                weights,
                land,
            )
        }
        Some(fine) => {
            let land = landmass_grid_two_tier(sdf_resolution_deg, fine);
            run_time_reopt_inner(
                baked,
                route_bounds,
                settings,
                ship,
                fixed_path,
                weights,
                land,
            )
        }
    }
}

fn run_time_reopt_inner<const N: usize, LS: LandmassSource>(
    baked: &BakedWindMap,
    route_bounds: RouteBounds,
    settings: SearchSettings,
    ship: &Boat,
    fixed_path: Path<N>,
    weights: SearchWeights,
    land: &LS,
) -> Path<N> {
    let fit_calc = SailboatFitCalc {
        time_weight: weights.time_weight,
        fuel_weight: weights.fuel_weight,
        land_weight: weights.land_weight,
        departure_time: 0.0,
        step_distance_max: route_bounds.step_distance_max,
        ship,
        wind_source: baked,
        landmass: land,
    };
    reoptimize_times(&fit_calc, settings, fixed_path)
}
