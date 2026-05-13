//! Pure, blocking entry points for the sailing search and time-only PSO.
//! Callers wrap them in their preferred concurrency model (GUI: worker
//! thread + mpsc; CLI: main thread).

use swarmkit::FitCalc as _;
use swarmkit_sailing::{
    Boat, LandmassSource, Path, PathBaseline, RouteBounds, SailboatFitCalc, SeaPathBias,
    SearchSettings, get_segment_fuel_and_time, get_segment_land_metres, reoptimize_times, search,
};

use crate::landmass::landmass_grid_at_resolution;
use crate::route::{BenchmarkRoute, RouteEvolution, WaypointCount, debug_assert_path_no_nans};
use crate::waypoint_match;
use crate::wind_map::{BakeBounds, BakedWindMap, TimedWindMap};

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
    reason = "Eight first-class inputs the caller picks independently; \
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
) -> Result<SearchResult, SearchError> {
    let bake_start = std::time::Instant::now();
    let baked = wind_map.bake(bake_bounds);
    let bake_duration = bake_start.elapsed();
    let mut result = run_search_blocking_with_baked(
        baked,
        route_bounds,
        waypoint_count,
        search_settings,
        ship,
        weights,
        sdf_resolution_deg,
    )?;
    // Inner call zero-init'd `bake_duration`; patch in the real value.
    result.bake_duration = bake_duration;
    Ok(result)
}

/// Variant taking a pre-baked wind field. Used by `bywind-cli`'s
/// `--load-baked` flag for hyperparameter sweeps over the same map.
/// `bake_duration` in the result is `Duration::ZERO`.
///
/// # Errors
/// [`SearchError::NoFeasibleRoute`] when the PSO converges with non-finite
/// gbest fitness.
#[expect(
    clippy::panic_in_result_fn,
    reason = "debug-only `cfg!(debug_assertions)` asserts catch NaN \
              before it corrupts downstream rendering; release builds \
              compile them out."
)]
pub fn run_search_blocking_with_baked(
    baked: BakedWindMap,
    route_bounds: RouteBounds,
    waypoint_count: WaypointCount,
    search_settings: SearchSettings,
    ship: Boat,
    weights: SearchWeights,
    sdf_resolution_deg: f64,
) -> Result<SearchResult, SearchError> {
    let search_start = std::time::Instant::now();
    let land = landmass_grid_at_resolution(sdf_resolution_deg);
    // `best_fit` rides out of the macro so the `NoFeasibleRoute`
    // check below doesn't need to know the const-generic N.
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
        let (gbest, evolution) = search::<N, _, _, _, _>(
            &ship,
            &baked,
            land,
            route_bounds,
            &fit_calc,
            search_settings,
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
        let benchmark = compute_benchmark::<N, _, _>(
            &ship,
            &baked,
            land,
            route_bounds,
            &fit_calc,
            search_settings,
        );
        (wrap(evolution), ship, benchmark, gbest.best_fit)
    });
    // Non-finite gbest = every particle hit a physically-infeasible
    // segment (pole-lock, unsailable, etc. → `-INFINITY` propagated
    // through `TimeBoundary`). `is_finite` rejects `±INF` and `NaN`.
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
    })
}

/// Time-only PSO. Re-optimises `path.t` with `path.xy` held fixed.
/// Pass [`crate::SDF_RESOLUTION_DEG`] for the default landmass grid.
pub fn run_time_reopt_blocking<const N: usize>(
    baked: &BakedWindMap,
    route_bounds: RouteBounds,
    settings: SearchSettings,
    ship: &Boat,
    fixed_path: Path<N>,
    weights: SearchWeights,
    sdf_resolution_deg: f64,
) -> Path<N> {
    let land = landmass_grid_at_resolution(sdf_resolution_deg);
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
