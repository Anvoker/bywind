//! Pure, blocking entry points for the sailing search and time-only PSO.
//! Callers wrap them in their preferred concurrency model (GUI: worker
//! thread + mpsc; CLI: main thread).

use swarmkit::FitCalc as _;
use swarmkit_sailing::{
    Boat, EnsembleSailboatFitCalc, LandmassSource, Path, PathBaseline, RobustObjective,
    RouteBounds, SailboatFitCalc, SeaPathBias, SearchSettings, get_segment_fuel_and_time,
    get_segment_land_metres, reoptimize_times, search,
};

use crate::ensemble::BakedEnsembleWindMap;
use crate::landmass::{landmass_grid_at_resolution, landmass_grid_two_tier};
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
) -> Result<SearchResult, SearchError> {
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
    )?;
    // Inner call zero-init'd `bake_duration`; patch in the real value.
    result.bake_duration = bake_duration;
    Ok(result)
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

/// Aggregation strategy for ensemble fitness.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum EnsembleMode {
    /// `K`-fold per-particle fitness reduced via [`RobustObjective`]
    /// (currently `Mean`). Benchmark route uses the precomputed mean
    /// wind for the time-PSO refinement (cheaper than K-fold inside
    /// the inner loop; documents the bench as "mean-wind reference").
    Full,
    /// Skip the K-fold loop entirely; run the existing single-
    /// deterministic search against the precomputed mean wind map.
    /// `E[f(x, wind)] ≠ f(x, E[wind])` in general but for linear-
    /// polar regimes the gap is typically <1%. Massive speedup.
    FastMean,
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
) -> Result<SearchResult, SearchError> {
    let (route_evolution, boat, benchmark, best_fit) = waypoint_match!(waypoint_count, N, wrap, {
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
        // PSO uses ensemble K-fold; baselines / boundary repulsion
        // query the mean wind (single representative). The init code
        // only needs `WindSource::sample_wind`, not per-member
        // fitness — using the mean here keeps the baselines stable.
        let (gbest, evolution) = search::<N, _, _, _, _>(
            &ship,
            &mean,
            land,
            route_bounds,
            &ensemble_fit_calc,
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
        // Benchmark uses single-wind fit calc against the mean. This
        // makes the bench a "mean-wind reference" the user can read
        // alongside the ensemble PSO result.
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
        (wrap(evolution), ship, benchmark, gbest.best_fit)
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
            run_time_reopt_inner(baked, route_bounds, settings, ship, fixed_path, weights, land)
        }
        Some(fine) => {
            let land = landmass_grid_two_tier(sdf_resolution_deg, fine);
            run_time_reopt_inner(baked, route_bounds, settings, ship, fixed_path, weights, land)
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
