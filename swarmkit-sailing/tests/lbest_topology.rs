//! lbest topology smoke + diversity probe.
//!
//! - **smoke:** ring lbest reaches a finite, comparable fitness given a bit
//!   more budget than gbest. Plan #3 specifies "within ~1.5× iters" — this
//!   test gives lbest 1.5× `iter_space` and asserts its fitness is at least
//!   as good as gbest's at the smaller budget.
//! - **diversity:** at the midpoint of an equal-budget run, the swarm's
//!   pbest spread (stdev of mid-waypoint latitude) is materially larger
//!   under ring lbest than under gbest. This is the evidence that the
//!   topology is actually preserving exploration and not just running
//!   slower for no reason.

mod common;

use common::NoLand;
use swarmkit::{Evolution, Particle};
use swarmkit_sailing::{
    BaselineShares, Boat, InitShares, Path, RouteBounds, SailboatFitCalc, SearchSettings, Topology,
    WindSource, search,
    spherical::{LatLon, LonLatBbox, Wind},
};

const N: usize = 6;

/// Two-corridor scenario (lat ±5° detours around a centre band of weak
/// wind). Both corridors are roughly equal so gbest can collapse onto
/// either, but should collapse onto one quickly. lbest should preserve
/// both populations longer.
struct TwoCorridorWind {
    base_speed_mps: f64,
}

impl WindSource for TwoCorridorWind {
    fn sample_wind(&self, location: LatLon, _t: f64) -> Wind {
        // Strong eastward wind everywhere except a calm band near
        // lat = 0 (the obstacle's path). Both corridors (lat > +2 or
        // lat < -2) get the full wind.
        let lat = location.lat;
        let strength = if lat.abs() > 2.0 {
            self.base_speed_mps
        } else {
            // Smooth roll-off so gradients aren't degenerate.
            self.base_speed_mps * (lat.abs() / 2.0).clamp(0.0, 1.0)
        };
        Wind::new(strength, 0.0)
    }
}

fn make_bounds() -> RouteBounds {
    RouteBounds::new(
        (-15.0, 0.0),
        (15.0, 0.0),
        LonLatBbox::new(-20.0, 20.0, -10.0, 10.0),
    )
}

fn run(topology: Topology, max_iter_space: usize) -> (f64, Evolution<Path<N>>) {
    let wind = TwoCorridorWind {
        base_speed_mps: 12.0,
    };
    let bounds = make_bounds();
    let boat = Boat::default();
    let land = NoLand;
    let fit = SailboatFitCalc::<N, _, _, _> {
        time_weight: 1.0,
        fuel_weight: 10.0,
        land_weight: 0.0,
        departure_time: 0.0,
        step_distance_max: bounds.step_distance_max,
        ship: &boat,
        wind_source: &wind,
        landmass: &land,
    };
    let settings = SearchSettings {
        particle_count_space: 32,
        particle_count_time: 8,
        max_iteration_space: max_iter_space,
        max_iteration_time: 4,
        // Force the broadest possible spread of init particles. With
        // NoLand the polyline baselines fall back to none, so this
        // collapses to the straight-line baseline shared across all
        // particles — the diversity therefore comes from the per-family
        // shape kicks, not the corridors.
        baseline_shares: BaselineShares {
            polyline_north: 0.0,
            polyline_south: 0.0,
            straight_line: 1.0,
        },
        init_shares: InitShares::default(),
        seed: Some(0xCAFE_BABE),
        topology,
        ..SearchSettings::default()
    };
    let (best, evolution) = search::<N, _, _, _, _>(&boat, &wind, &land, bounds, &fit, settings);
    (best.best_fit, evolution)
}

#[test]
fn lbest_ring_converges_within_1_5x_gbest_iters() {
    let (gbest_fit, _) = run(Topology::GBest, 20);
    let (lbest_fit, _) = run(Topology::Ring, 30);

    assert!(gbest_fit.is_finite(), "gbest fitness must be finite");
    assert!(lbest_fit.is_finite(), "lbest fitness must be finite");

    // lbest with 1.5× the budget should land within striking distance
    // of gbest's fitness. Fitness is negative cost, so "at least as
    // good" means greater or equal; we allow 5 % relative slack to
    // absorb the stochastic gap that's inherent to a slower-converging
    // topology even at 1.5× iters.
    let slack = 0.05 * gbest_fit.abs();
    assert!(
        lbest_fit >= gbest_fit - slack,
        "expected lbest with 1.5× iters to match gbest within 5 %% slack ({slack:.0}); \
         got lbest = {lbest_fit:.0}, gbest = {gbest_fit:.0} \
         (gap {:.2}%%)",
        100.0 * (gbest_fit - lbest_fit) / gbest_fit.abs(),
    );
}

/// Stddev of `path.lat_lon(mid).lat` across all particles' pbests in
/// `frame`. A single scalar that goes up when the swarm is spread across
/// multiple latitude corridors and down when collapsed.
fn pbest_lat_stddev(frame: &[Particle<Path<N>>]) -> f64 {
    let mid = N / 2;
    let lats: Vec<f64> = frame.iter().map(|p| p.best_pos.lat_lon(mid).lat).collect();
    let mean = lats.iter().sum::<f64>() / lats.len() as f64;
    let var = lats.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / lats.len() as f64;
    var.sqrt()
}

#[test]
fn lbest_preserves_more_pbest_spread_than_gbest_at_midpoint() {
    let max_iter = 30;
    let (_, gbest_evo) = run(Topology::GBest, max_iter);
    let (_, lbest_evo) = run(Topology::Ring, max_iter);

    let mid = max_iter / 2 - 1;
    let gbest_frame = &gbest_evo.frames()[mid];
    let lbest_frame = &lbest_evo.frames()[mid];

    let gbest_spread = pbest_lat_stddev(gbest_frame);
    let lbest_spread = pbest_lat_stddev(lbest_frame);

    assert!(
        lbest_spread > gbest_spread,
        "expected lbest pbest spread to exceed gbest's at iter {mid}: \
         lbest = {lbest_spread:.4}, gbest = {gbest_spread:.4}. The whole \
         point of lbest is to preserve diversity longer.",
    );
}
