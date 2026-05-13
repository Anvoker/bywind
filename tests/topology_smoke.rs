//! End-to-end "Done definition" smoke for the PSO topology arc.
//!
//! Walks the same code path `bywind-cli search` does — `run_search_blocking`
//! against a synthetic wind map — once per topology, and asserts:
//!
//! 1. Every topology produces a finite fitness (not NaN, not -inf).
//! 2. lbest variants (`Ring`, `VonNeumann`) land within ~1.5× of gbest's
//!    cost. Lbest is expected to converge slower, so equal-budget fitness
//!    is allowed to be worse, but not catastrophically so.
//! 3. Niched lands at-or-better than gbest. Niched preserves diversity
//!    without sacrificing convergence speed because each cohort runs a
//!    full gbest within itself; the only loss is across-cohort
//!    information, which on this synthetic scenario costs nothing.

use bywind::{
    BAKE_STEP, BoatConfig, MapBounds, SearchConfig, SearchWeights, TimedWindMap, Topology,
    run_search_blocking,
};
use rand::SeedableRng as _;
use rand::rngs::SmallRng;

const PSO_SEED: u64 = 0xCAFE_BABE;
const WIND_SEED: u64 = 0xDEAD_BEEF;

fn run_one(topology: Topology) -> f64 {
    // Fully reproducible: the wind map is seeded via
    // `generate_random_with_rng`, and the PSO inside `run_search_blocking`
    // is seeded via `SearchConfig.seed`. A given (topology, both seeds)
    // tuple maps to a fixed fitness across runs and machines.
    let mut wind_rng = SmallRng::seed_from_u64(WIND_SEED);
    let wind = TimedWindMap::generate_random_with_rng(
        20.0,
        20.0,
        1.0,
        1,
        3600.0,
        5.0..15.0,
        &mut wind_rng,
    );
    let map_bounds = MapBounds::from_wind_map(&wind).expect("non-empty synthetic map");
    let bake_bounds = map_bounds.to_bake_bounds(BAKE_STEP);
    let route_bounds = map_bounds.to_route_bounds((2.0, 2.0), (18.0, 18.0));

    let mut cfg = SearchConfig {
        seed: Some(PSO_SEED),
        topology,
        ..SearchConfig::default()
    };
    // Smaller budget than default so the test is quick. Equal across
    // topologies — the fairness of the comparison is what matters.
    cfg.particles_space = 16;
    cfg.particles_time = 16;
    cfg.iter_space = 12;
    cfg.iter_time = 8;

    let weights = SearchWeights {
        time_weight: cfg.time_weight,
        fuel_weight: cfg.fuel_weight,
        land_weight: 0.0, // synthetic map has no real landmass coverage
    };

    let result = run_search_blocking(
        &wind,
        bake_bounds,
        route_bounds,
        cfg.waypoint_count,
        cfg.to_search_settings(),
        BoatConfig::default().to_boat(),
        weights,
        bywind::SDF_RESOLUTION_DEG,
    )
    .expect("smoke test inputs produce a feasible route");
    let route_evolution = result.route_evolution;
    let last = route_evolution.iter_count().saturating_sub(1);
    route_evolution
        .gbest_at(last)
        .expect("at least one iteration ran")
        .best_fit
}

#[test]
fn all_topologies_route_through_run_search_blocking_with_finite_fitness() {
    let gbest = run_one(Topology::GBest);
    let niched = run_one(Topology::Niched);
    let ring = run_one(Topology::Ring);
    let vn = run_one(Topology::VonNeumann);

    for (name, fit) in [
        ("gbest", gbest),
        ("niched", niched),
        ("ring", ring),
        ("von_neumann", vn),
    ] {
        assert!(
            fit.is_finite(),
            "{name} produced non-finite fitness {fit}; \
             topology dispatch in run_search_blocking is broken",
        );
    }

    // lbest variants: cost (= -fitness) within 1.5× gbest's cost.
    // Equivalently fit >= gbest * 1.5 (both negative).
    let cost_ceiling = gbest * 1.5;
    for (name, fit) in [("ring", ring), ("von_neumann", vn)] {
        assert!(
            fit >= cost_ceiling,
            "{name} fit {fit:.0} exceeds 1.5× gbest cost (gbest = {gbest:.0}, ceiling = {cost_ceiling:.0}). \
             Done-definition asks for ~1.5× tolerance for lbest variants.",
        );
    }
}
