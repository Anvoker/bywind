//! End-to-end smoke coverage of the sailing search pipeline.
//!
//! Replaces the pre-spherical-switch perf bench. Runs the full outer/inner
//! PSO at small sizes against a synthetic wind field, and asserts that the
//! pipeline produces a finite-fitness result and emits one `Evolution`
//! frame per iteration. Catches regressions in the chained mover / nested
//! searcher / segment-range cache wiring on every `cargo test`.

#![allow(
    clippy::float_cmp,
    reason = "tests rely on bit-exact comparisons of constant or stored f32/f64 values."
)]

mod common;

use common::{GapWind, route_bounds_for_smoke};
use swarmkit_sailing::{
    Boat, LandmassSourceDummy, SailboatFitCalc, SearchSettings, Topology, search,
};

const N: usize = 6;

#[test]
fn search_produces_finite_fitness_and_full_evolution() {
    let wind = GapWind::smoke_default();
    let land = LandmassSourceDummy;
    let bounds = route_bounds_for_smoke();
    let boat = Boat::default();

    let fit_calc = SailboatFitCalc::<N, _, _, _> {
        time_weight: 1.0,
        fuel_weight: 10.0,
        land_weight: 0.0,
        departure_time: 0.0,
        step_distance_max: bounds.step_distance_max,
        ship: &boat,
        wind_source: &wind,
        landmass: &land,
    };

    // Small sizes: enough iterations to exercise the full chain (kicks,
    // boundary clamps, nested time PSO) but short enough to finish in
    // well under a second even in debug builds.
    let settings = SearchSettings {
        particle_count_space: 8,
        particle_count_time: 8,
        max_iteration_space: 5,
        max_iteration_time: 4,
        ..SearchSettings::default()
    };

    let (best, evolution) =
        search::<N, _, _, _, _>(&boat, &wind, &land, bounds, &fit_calc, settings);

    assert!(
        best.best_fit.is_finite(),
        "expected finite best_fit, got {}",
        best.best_fit
    );
    assert_eq!(
        evolution.frames().len(),
        settings.max_iteration_space,
        "expected one snapshot per outer iteration"
    );
}

/// Two `search` runs with the same `seed` must produce identical
/// `best_fit` and `best_pos`, for *every* topology. Locks down the seed
/// plumbing so the PSO-tuning study can rely on per-seed reproducibility,
/// and catches RNG-bookkeeping bugs in the niched / lbest paths (per-niche
/// slicing, pbest snapshot, neighbour lookup) that gbest-only coverage
/// would miss.
#[test]
fn seeded_search_is_deterministic_for_every_topology() {
    let wind = GapWind::smoke_default();
    let land = LandmassSourceDummy;
    let bounds = route_bounds_for_smoke();
    let boat = Boat::default();

    let fit_calc = SailboatFitCalc::<N, _, _, _> {
        time_weight: 1.0,
        fuel_weight: 10.0,
        land_weight: 0.0,
        departure_time: 0.0,
        step_distance_max: bounds.step_distance_max,
        ship: &boat,
        wind_source: &wind,
        landmass: &land,
    };

    for &topology in Topology::ALL {
        let settings = SearchSettings {
            particle_count_space: 8,
            particle_count_time: 8,
            max_iteration_space: 5,
            max_iteration_time: 4,
            seed: Some(0xC0FFEE),
            topology,
            ..SearchSettings::default()
        };

        let (a, _) = search::<N, _, _, _, _>(&boat, &wind, &land, bounds, &fit_calc, settings);
        let (b, _) = search::<N, _, _, _, _>(&boat, &wind, &land, bounds, &fit_calc, settings);

        assert_eq!(
            a.best_fit, b.best_fit,
            "topology {topology:?}: same seed must yield identical best_fit",
        );
        assert_eq!(
            a.best_pos.xy.0, b.best_pos.xy.0,
            "topology {topology:?}: same seed must yield identical xy.0",
        );
        assert_eq!(
            a.best_pos.xy.1, b.best_pos.xy.1,
            "topology {topology:?}: same seed must yield identical xy.1",
        );
        assert_eq!(
            a.best_pos.t, b.best_pos.t,
            "topology {topology:?}: same seed must yield identical t",
        );
    }
}
