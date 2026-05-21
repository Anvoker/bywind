//! End-to-end ensemble-wind smoke. Builds a synthetic K-member
//! ensemble (different seeds per member so they aren't identical),
//! bakes it, runs the search via the same `run_search_blocking_with_baked`
//! the CLI uses, and asserts:
//!
//! 1. `WindInput::Single(baked)` and a synthetic single-member ensemble
//!    in `FastMean` mode that contains *only that same baked map*
//!    produce identical results — the fast-mean path is a strict
//!    no-op equivalent when the ensemble has one member.
//! 2. `WindInput::ensemble_full` produces a finite fitness on a
//!    multi-member synthetic ensemble.
//! 3. The K-fold path runs without panicking through every code path
//!    (search, bench, segment metrics).
//! 4. Mean of K identical members reduces to a single-deterministic
//!    search at the same wind values (within tolerance of the K-fold
//!    averaging numerical noise).

use bywind::{
    BAKE_STEP, BakedEnsembleWindMap, BoatConfig, MapBounds, SearchConfig, SearchResult,
    SearchWeights, TimedEnsembleWindMap, TimedWindMap, WindInput, run_search_blocking_with_baked,
};
use rand::SeedableRng as _;
use rand::rngs::SmallRng;

const PSO_SEED: u64 = 0xCAFE_BABE;
const WIND_SEED_BASE: u64 = 0xDEAD_BEEF;

fn synthetic_wind(rng_seed: u64) -> TimedWindMap {
    let mut rng = SmallRng::seed_from_u64(rng_seed);
    TimedWindMap::generate_random_with_rng(20.0, 20.0, 1.0, 1, 3600.0, 5.0..15.0, &mut rng)
}

fn small_search_cfg() -> SearchConfig {
    let mut cfg = SearchConfig {
        seed: Some(PSO_SEED),
        ..SearchConfig::default()
    };
    cfg.particles_space = 12;
    cfg.particles_time = 12;
    cfg.iter_space = 8;
    cfg.iter_time = 6;
    cfg
}

fn run_with_wind(wind: WindInput, bounds: MapBounds) -> SearchResult {
    let cfg = small_search_cfg();
    let route_bounds = bounds.to_route_bounds((2.0, 2.0), (18.0, 18.0));
    let weights = SearchWeights {
        time_weight: cfg.time_weight,
        fuel_weight: cfg.fuel_weight,
        land_weight: 0.0,
    };
    run_search_blocking_with_baked(
        wind,
        route_bounds,
        cfg.waypoint_count,
        cfg.to_search_settings(),
        BoatConfig::default().to_boat(),
        weights,
        bywind::SDF_RESOLUTION_DEG,
        None,
    )
    .expect("smoke inputs produce a feasible route")
}

fn final_fitness(result: &SearchResult) -> f64 {
    let last = result.route_evolution.iter_count().saturating_sub(1);
    result
        .route_evolution
        .gbest_at(last)
        .expect("at least one iteration ran")
        .best_fit
}

#[test]
fn ensemble_single_member_fast_mean_matches_single_deterministic() {
    // Build a single TimedWindMap. Use it twice:
    // (a) Search it as `WindInput::Single(baked)` directly.
    // (b) Wrap it as a 1-member ensemble and search with FastMean mode.
    // The fast-mean of a 1-element ensemble is just the same wind, so
    // (a) and (b) must produce the same converged fitness given the
    // same seed.
    let wind = synthetic_wind(WIND_SEED_BASE);
    let bounds = MapBounds::from_wind_map(&wind).expect("non-empty");
    let bake_bounds = bounds.to_bake_bounds(BAKE_STEP);
    let baked_a = wind.clone().bake(bake_bounds);
    let baked_b = wind.clone().bake(bake_bounds);

    let single_result = run_with_wind(WindInput::single(baked_a), bounds);

    // Verify the loader-shaped constructor accepts the same data,
    // even though this test bakes directly to keep the wind input
    // bit-identical to (a). Discard via drop() to satisfy the
    // must-use lint cleanly.
    drop(TimedEnsembleWindMap::from_members(
        vec![wind],
        vec!["gec00".to_owned()],
    ));
    let baked_ens =
        BakedEnsembleWindMap::from_members(vec![baked_b], vec!["gec00".to_owned()]);
    let fast_mean_result =
        run_with_wind(WindInput::ensemble_fast_mean(baked_ens), bounds);

    let single_fit = final_fitness(&single_result);
    let fast_fit = final_fitness(&fast_mean_result);
    let rel_diff = ((single_fit - fast_fit) / single_fit.abs()).abs();
    // Allow tiny floating-point drift from the 1-element averaging
    // (sum/1 should be bit-identical but the dispatch goes through
    // different code paths; allow 1e-6 relative).
    assert!(
        rel_diff < 1e-6,
        "single-tier vs 1-member fast-mean fitness diverged: single={single_fit}, fast={fast_fit}, rel_diff={rel_diff}",
    );
}

#[test]
fn ensemble_full_k_three_produces_finite_fitness() {
    // Three distinct synthetic wind maps. Verifies the K-fold loop
    // runs through the search end-to-end without panicking and
    // produces a finite (negative-but-real) fitness.
    let members: Vec<TimedWindMap> = (0..3)
        .map(|i| synthetic_wind(WIND_SEED_BASE + i))
        .collect();
    let bounds = MapBounds::from_wind_map(&members[0]).expect("non-empty");
    let bake_bounds = bounds.to_bake_bounds(BAKE_STEP);
    let baked: Vec<_> = members.iter().map(|m| m.clone().bake(bake_bounds)).collect();
    let names: Vec<String> = (0..3).map(|i| format!("gep{i:02}")).collect();
    let ensemble = BakedEnsembleWindMap::from_members(baked, names);

    let result = run_with_wind(WindInput::ensemble_full(ensemble), bounds);
    let fit = final_fitness(&result);
    assert!(fit.is_finite(), "ensemble Full fitness must be finite, got {fit}");
    assert!(
        fit < 0.0,
        "fitness is negated cost, expected < 0, got {fit}",
    );
}

#[test]
fn ensemble_assessment_time_varies_across_distinct_members() {
    // Regression guard: per-member `MemberMetrics.time_s` must reflect
    // a per-member time-reopt against that member's wind, not a sum of
    // the converged gbest path's segment durations (which would be
    // identical across members because `walk_segments_against_wind`
    // integrates segment times from `path.t`, not the wind).
    let members: Vec<TimedWindMap> = (0..3)
        .map(|i| synthetic_wind(WIND_SEED_BASE + i + 100))
        .collect();
    let bounds = MapBounds::from_wind_map(&members[0]).expect("non-empty");
    let bake_bounds = bounds.to_bake_bounds(BAKE_STEP);
    let baked: Vec<_> = members.iter().map(|m| m.clone().bake(bake_bounds)).collect();
    let names: Vec<String> = (0..3).map(|i| format!("gep{i:02}")).collect();
    let ensemble = BakedEnsembleWindMap::from_members(baked, names);

    let result = run_with_wind(WindInput::ensemble_full(ensemble), bounds);
    let assessment = result
        .ensemble
        .as_ref()
        .expect("Full mode must populate the ensemble assessment");
    assert_eq!(assessment.per_member.len(), 3);
    let times: Vec<f64> = assessment.per_member.iter().map(|m| m.time_s).collect();
    // At least one pair must differ: distinct synthetic winds under
    // per-member time-reopt cannot produce bit-identical schedules.
    let any_differ = times.iter().any(|t| (*t - times[0]).abs() > 1e-9);
    assert!(
        any_differ,
        "per-member time_s collapsed to a single value across distinct members — \
         per-member time-reopt likely regressed: times={times:?}",
    );
}

#[test]
fn ensemble_full_with_identical_members_matches_single_deterministic() {
    // K copies of the same wind map. The mean is just that wind map.
    // K-fold Mean fitness reduces to single-deterministic fitness on
    // the same map. They should agree within tiny floating-point
    // drift from the K-element sum-then-divide vs single eval.
    let wind = synthetic_wind(WIND_SEED_BASE + 7);
    let bounds = MapBounds::from_wind_map(&wind).expect("non-empty");
    let bake_bounds = bounds.to_bake_bounds(BAKE_STEP);

    let baked_single = wind.clone().bake(bake_bounds);
    let single_result = run_with_wind(WindInput::single(baked_single), bounds);

    let k = 4;
    let baked_members: Vec<_> = (0..k).map(|_| wind.clone().bake(bake_bounds)).collect();
    let names: Vec<String> = (0..k).map(|i| format!("gep{i:02}")).collect();
    let ensemble = BakedEnsembleWindMap::from_members(baked_members, names);
    let ens_result = run_with_wind(WindInput::ensemble_full(ensemble), bounds);

    let single_fit = final_fitness(&single_result);
    let ens_fit = final_fitness(&ens_result);
    let rel_diff = ((single_fit - ens_fit) / single_fit.abs()).abs();
    // Same seed, same map, same fitness in expectation. The K-fold
    // averaging may introduce ~1e-12 numerical drift relative to a
    // single eval; allow 1e-5 for safety.
    assert!(
        rel_diff < 1e-5,
        "K=4-of-identical ensemble vs single fitness drifted: single={single_fit}, ens={ens_fit}, rel_diff={rel_diff}",
    );
}
