//! Tests Stage 5: multi-baseline init with biased polyline seeds. Sets
//! up a wide landmass that the straight-line init would mostly cross,
//! and verifies that with multi-baseline shares the majority of init
//! particles are already land-free.

mod common;

use common::GapWind;
use rand::SeedableRng as _;
use rand::rngs::SmallRng;
use swarmkit::ParticleInit as _;
use swarmkit_sailing::{
    BaselineShares, Boat, InitShares, LandmassSource, Path, PathInit, RouteBounds, SailboatFitCalc,
    SeaPathBias, SearchSettings, get_segment_land_metres, search,
    spherical::{LatLon, LonLatBbox},
};

const N: usize = 8;

/// Vertical bar of land plus a hand-crafted "A* result" for north and
/// south detours. Lets the test exercise the multi-baseline allocation
/// without depending on the real `bywind::landmass::LandmassGrid`.
struct SimulatedLandmass {
    /// Land bbox: lon ∈ [`lon_min`, `lon_max`], lat ∈ [`lat_min`, `lat_max`].
    lon_min: f64,
    lon_max: f64,
    lat_min: f64,
    lat_max: f64,
    /// Pre-baked polylines that detour around the bar. The struct
    /// returns these from `find_sea_path` for the corresponding bias.
    north_path: Vec<LatLon>,
    south_path: Vec<LatLon>,
}

impl LandmassSource for SimulatedLandmass {
    fn signed_distance_m(&self, location: LatLon) -> f64 {
        let dx = (self.lon_min - location.lon).max(location.lon - self.lon_max);
        let dy = (self.lat_min - location.lat).max(location.lat - self.lat_max);
        let deg_to_m = 111_000.0;
        let outside = dx.max(0.0).hypot(dy.max(0.0)) * deg_to_m;
        let inside = dx.max(dy).min(0.0) * deg_to_m;
        if outside > 0.0 { outside } else { inside }
    }

    fn find_sea_path(
        &self,
        _origin: LatLon,
        _destination: LatLon,
        _bounds: &RouteBounds,
        bias: SeaPathBias,
    ) -> Option<Vec<LatLon>> {
        match bias {
            SeaPathBias::None | SeaPathBias::North => Some(self.north_path.clone()),
            SeaPathBias::South => Some(self.south_path.clone()),
        }
    }
}

fn route_land_metres(land: &SimulatedLandmass, path: &Path<N>, step: f64) -> f64 {
    let mut total = 0.0;
    for seg in path.iter_with_running_clock(0.0) {
        total += get_segment_land_metres(land, seg.origin, seg.destination, step);
    }
    total
}

fn make_landmass() -> SimulatedLandmass {
    // 4°-wide bar from -2° to 2°, spanning lat ±9°. The straight-line
    // route from (-15, 0) → (15, 0) crosses it dead-centre.
    SimulatedLandmass {
        lon_min: -2.0,
        lon_max: 2.0,
        lat_min: -9.0,
        lat_max: 9.0,
        // Hand-crafted detour polylines with corner waypoints just
        // outside the bar. Linear interpolation along these polylines
        // produces sea-only segments.
        north_path: vec![
            LatLon::new(-15.0, 0.0),
            LatLon::new(-3.0, 9.5),
            LatLon::new(0.0, 10.0),
            LatLon::new(3.0, 9.5),
            LatLon::new(15.0, 0.0),
        ],
        south_path: vec![
            LatLon::new(-15.0, 0.0),
            LatLon::new(-3.0, -9.5),
            LatLon::new(0.0, -10.0),
            LatLon::new(3.0, -9.5),
            LatLon::new(15.0, 0.0),
        ],
    }
}

fn make_bounds() -> RouteBounds {
    RouteBounds::new(
        (-15.0, 0.0),
        (15.0, 0.0),
        LonLatBbox::new(-20.0, 20.0, -12.0, 12.0),
    )
}

#[test]
fn multi_baseline_init_produces_mostly_land_free_particles() {
    let land = make_landmass();
    let wind = GapWind {
        center: LatLon::new(20.0, 20.0),
        sigma_deg: 3.0,
        base_speed_mps: 8.0,
    };
    let bounds = make_bounds();
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

    let particle_count = 32;
    let init = PathInit::<N, _, _, _>::new(
        &bounds,
        &boat,
        &wind,
        &fit_calc,
        particle_count,
        InitShares::default(),
        BaselineShares::default(),
    );
    let mut rng = SmallRng::seed_from_u64(42);
    let particles: Vec<Path<N>> = init.init_pos(&mut rng);
    assert_eq!(particles.len(), particle_count);

    let land_free = particles
        .iter()
        .filter(|p| route_land_metres(&land, p, bounds.step_distance_max) < 50_000.0)
        .count();
    // 80 % polyline / 20 % straight-line default, so we expect ~80 %
    // of particles seeded on a land-free baseline. Mutation noise can
    // push a few off the polyline into land, so 60 % is a comfortable
    // floor that still proves the multi-baseline allocation worked.
    let pct = land_free as f64 / particle_count as f64;
    assert!(
        pct > 0.6,
        "expected >60 % land-free init particles, got {pct} ({land_free}/{particle_count})",
    );
}

/// Runs the multi-baseline-init invariant check at a caller-chosen seed.
/// Same rationale as the niched and `landmass_avoidance` tests: the
/// search should land on a substep-clean route on essentially every
/// RNG draw, but `ShapeKickMover` Cauchy-tail outliers can leave one
/// waypoint inside the obstacle band on a specific seed. A small slate
/// of pinned seeds catches that without going full Monte Carlo.
fn search_with_multi_baseline_init_finds_land_free_route_with_seed(seed: u64) {
    let land = make_landmass();
    let wind = GapWind {
        center: LatLon::new(20.0, 20.0),
        sigma_deg: 3.0,
        base_speed_mps: 8.0,
    };
    let bounds = make_bounds();
    let boat = Boat::default();

    let settings = SearchSettings {
        particle_count_space: 32,
        particle_count_time: 12,
        max_iteration_space: 25,
        max_iteration_time: 6,
        seed: Some(seed),
        ..SearchSettings::default()
    };

    let fit_calc = SailboatFitCalc::<N, _, _, _> {
        time_weight: 1.0,
        fuel_weight: 10.0,
        land_weight: 50.0,
        departure_time: 0.0,
        step_distance_max: bounds.step_distance_max,
        ship: &boat,
        wind_source: &wind,
        landmass: &land,
    };

    let (best, _evolution) =
        search::<N, _, _, _, _>(&boat, &wind, &land, bounds, &fit_calc, settings);
    let land_metres = route_land_metres(&land, &best.best_pos, bounds.step_distance_max);
    let tol = 2.0 * bounds.step_distance_max;
    assert!(
        land_metres < tol,
        "best route still crosses land: {land_metres} m (tol {tol} m, seed={seed})",
    );
    assert!(
        best.best_fit.is_finite(),
        "best fit must be finite (seed={seed})"
    );
}

#[test]
fn search_with_multi_baseline_init_finds_land_free_route_seed_0() {
    search_with_multi_baseline_init_finds_land_free_route_with_seed(0);
}
#[test]
fn search_with_multi_baseline_init_finds_land_free_route_seed_1() {
    search_with_multi_baseline_init_finds_land_free_route_with_seed(1);
}
#[test]
fn search_with_multi_baseline_init_finds_land_free_route_seed_2() {
    search_with_multi_baseline_init_finds_land_free_route_with_seed(2);
}
#[test]
fn search_with_multi_baseline_init_finds_land_free_route_seed_42() {
    search_with_multi_baseline_init_finds_land_free_route_with_seed(42);
}
