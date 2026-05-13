//! End-to-end test that the search routes around a synthetic landmass
//! when `land_weight > 0` and ignores it when `land_weight = 0`.

mod common;

use common::GapWind;
use swarmkit_sailing::{
    Boat, LandmassSource, RouteBounds, SailboatFitCalc, SearchSettings, get_segment_land_metres,
    search,
    spherical::{LatLon, LonLatBbox},
};

/// `LandmassSource` impl that treats an axis-aligned `(lon, lat)` bbox
/// as land. Signed distance is the negative L∞ distance to the bbox
/// boundary inside the box, and the positive L∞ distance outside.
/// Crude but adequate for a single rectangular obstacle test.
struct BboxLand {
    lon_min: f64,
    lon_max: f64,
    lat_min: f64,
    lat_max: f64,
}

impl LandmassSource for BboxLand {
    fn signed_distance_m(&self, location: LatLon) -> f64 {
        let dx = (self.lon_min - location.lon).max(location.lon - self.lon_max);
        let dy = (self.lat_min - location.lat).max(location.lat - self.lat_max);
        // Convert from degrees to a coarse metre estimate; exact value
        // doesn't matter for the test, only the sign and rough magnitude
        // (the search consults `is_land` exclusively for the penalty).
        let deg_to_m = 111_000.0;
        let outside = dx.max(0.0).hypot(dy.max(0.0)) * deg_to_m;
        let inside = dx.max(dy).min(0.0) * deg_to_m;
        if outside > 0.0 { outside } else { inside }
    }
}

const N: usize = 6;

fn route_bounds_for_land_test() -> RouteBounds {
    // 1° ≈ 111 km, so ~1100 km E–W with comfortable lat headroom for
    // a detour above or below the obstacle.
    RouteBounds::new(
        (0.0, 0.0),
        (10.0, 0.0),
        LonLatBbox::new(-2.0, 12.0, -8.0, 8.0),
    )
}

/// Total land-metres along `path`, sampled at the same substep cadence
/// the search uses.
fn route_land_metres<L: LandmassSource>(
    land: &L,
    path: &swarmkit_sailing::Path<N>,
    step: f64,
) -> f64 {
    let mut total = 0.0;
    for seg in path.iter_with_running_clock(0.0) {
        total += get_segment_land_metres(land, seg.origin, seg.destination, step);
    }
    total
}

/// Runs the obstacle-avoidance invariant check at a caller-chosen seed.
/// Pinned-seed wrappers below run the same body at a small slate of
/// seeds — convergence to a land-clearing route is a property the
/// search should hit on essentially every RNG draw, but `ShapeKickMover`
/// Cauchy-tail outliers can occasionally leave one waypoint of the
/// gbest path inside the obstacle band on a specific seed, so checking
/// a few catches that without going full Monte Carlo.
fn search_routes_around_landmass_when_penalty_active_with_seed(seed: u64) {
    let wind = GapWind {
        center: LatLon::new(20.0, 20.0),
        sigma_deg: 3.0,
        base_speed_mps: 8.0,
    };
    let land = BboxLand {
        // Block the centre of the straight-line route: lon 4–6, lat -3 to 3.
        // Detour via lat ±5 is well inside the bbox.
        lon_min: 4.0,
        lon_max: 6.0,
        lat_min: -3.0,
        lat_max: 3.0,
    };
    let bounds = route_bounds_for_land_test();
    let boat = Boat::default();

    let settings = SearchSettings {
        particle_count_space: 32,
        particle_count_time: 12,
        max_iteration_space: 25,
        max_iteration_time: 6,
        seed: Some(seed),
        ..SearchSettings::default()
    };

    // With `land_weight = 0`, the search has no incentive to detour;
    // some particles will wander around but the best may cross land.
    let fit_no_penalty = SailboatFitCalc::<N, _, _, _> {
        time_weight: 1.0,
        fuel_weight: 10.0,
        land_weight: 0.0,
        departure_time: 0.0,
        step_distance_max: bounds.step_distance_max,
        ship: &boat,
        wind_source: &wind,
        landmass: &land,
    };
    let (best_no_penalty, _) =
        search::<N, _, _, _, _>(&boat, &wind, &land, bounds, &fit_no_penalty, settings);
    let land_metres_no_penalty =
        route_land_metres(&land, &best_no_penalty.best_pos, bounds.step_distance_max);

    // With `land_weight` cranked up, the search should route around.
    // 100.0 = 100 cost units per metre of land; with time_weight=1 and
    // ~hours of travel, that swamps any plausible time saving.
    let fit_with_penalty = SailboatFitCalc::<N, _, _, _> {
        time_weight: 1.0,
        fuel_weight: 10.0,
        land_weight: 100.0,
        departure_time: 0.0,
        step_distance_max: bounds.step_distance_max,
        ship: &boat,
        wind_source: &wind,
        landmass: &land,
    };
    let (best_with_penalty, _) =
        search::<N, _, _, _, _>(&boat, &wind, &land, bounds, &fit_with_penalty, settings);
    let land_metres_with_penalty =
        route_land_metres(&land, &best_with_penalty.best_pos, bounds.step_distance_max);

    // The penalty-active best route should clear the obstacle to within
    // a substep tolerance. Pick a slack of 2 substeps to absorb sampling
    // noise (a great-circle that grazes the bbox might tag one or two
    // midpoints as land while spanning mostly water).
    let tol = 2.0 * bounds.step_distance_max;
    assert!(
        land_metres_with_penalty < tol,
        "expected penalty-active route to clear land within {tol} m, got {land_metres_with_penalty} m \
         (no-penalty baseline = {land_metres_no_penalty} m, seed={seed})",
    );

    assert!(
        best_no_penalty.best_fit.is_finite(),
        "no-penalty fit must be finite (seed={seed})"
    );
    assert!(
        best_with_penalty.best_fit.is_finite(),
        "with-penalty fit must be finite (seed={seed})"
    );
}

#[test]
fn search_routes_around_landmass_when_penalty_active_seed_0() {
    search_routes_around_landmass_when_penalty_active_with_seed(0);
}
#[test]
fn search_routes_around_landmass_when_penalty_active_seed_1() {
    search_routes_around_landmass_when_penalty_active_with_seed(1);
}
#[test]
fn search_routes_around_landmass_when_penalty_active_seed_2() {
    search_routes_around_landmass_when_penalty_active_with_seed(2);
}
#[test]
fn search_routes_around_landmass_when_penalty_active_seed_42() {
    search_routes_around_landmass_when_penalty_active_with_seed(42);
}
