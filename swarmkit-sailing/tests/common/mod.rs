//! Shared test fixtures.
//!
//! Both integration tests need a `WindSource` over real `(lon°, lat°)`
//! coordinates so the spherical-PSO and `Boat::get_travel_time`
//! integrators see well-formed inputs. A synthetic field is enough for
//! end-to-end smoke coverage and segment-cache probe accounting.

// Each integration test compiles `common` separately; items unused by a
// given test would otherwise warn. Allow rather than per-item-prune so
// adding fixtures stays cheap.
#![allow(
    dead_code,
    reason = "common test fixtures are shared across integration tests; \
              each test crate only uses a subset."
)]

use swarmkit_sailing::{
    LandmassSource, RouteBounds, SeaPathBias, WindSource,
    spherical::{LatLon, Wind},
};

/// Time-invariant wind: an eastward background flow whose magnitude is
/// attenuated by a Gaussian "gap" centered at `center`. With reasonable
/// parameters the boat's motor always carries it through (so all
/// integrators terminate finitely) but the gap creates a nontrivial
/// optimization landscape — straight-line-through-gap is suboptimal,
/// detour-around-gap costs distance, and the fastest route depends on
/// boat parameters.
pub struct GapWind {
    /// Gap center as `(lon°, lat°)`.
    pub center: LatLon,
    /// Gaussian sigma in degrees. `sigma_deg = 3.0` gives a gap roughly
    /// the size of a small sea.
    pub sigma_deg: f64,
    /// Background eastward wind speed in m/s outside the gap.
    pub base_speed_mps: f64,
}

impl WindSource for GapWind {
    fn sample_wind(&self, location: LatLon, _t: f64) -> Wind {
        let dx = location.lon - self.center.lon;
        let dy = location.lat - self.center.lat;
        let r2 = dx * dx + dy * dy;
        let sigma2 = self.sigma_deg * self.sigma_deg;
        let attenuation = (-0.5 * r2 / sigma2).exp();
        let speed = self.base_speed_mps * (1.0 - attenuation);
        Wind::new(speed, 0.0)
    }
}

impl GapWind {
    /// Defaults paired with [`route_bounds_for_smoke`] so the gap sits roughly
    /// in the middle of the search region.
    pub fn smoke_default() -> Self {
        Self {
            center: LatLon::new(-5.0, 50.0),
            sigma_deg: 3.0,
            base_speed_mps: 8.0,
        }
    }
}

/// Small lon/lat region in the North Atlantic, paired with
/// [`GapWind::smoke_default`].
pub fn route_bounds_for_smoke() -> swarmkit_sailing::RouteBounds {
    swarmkit_sailing::RouteBounds::new(
        (-10.0, 45.0),
        (0.0, 55.0),
        swarmkit_sailing::spherical::LonLatBbox::new(-12.0, 2.0, 43.0, 57.0),
    )
}

/// `LandmassSource` impl with no land anywhere — `signed_distance_m` is
/// `+∞` and `find_sea_path` returns `None` for every bias. Used by tests
/// that want to disable all landmass-related logic without setting up a
/// real grid; `compute_baselines` falls back to a single straight-line
/// baseline when both polyline searches fail.
pub struct NoLand;

impl LandmassSource for NoLand {
    fn signed_distance_m(&self, _location: LatLon) -> f64 {
        f64::INFINITY
    }

    fn find_sea_path(
        &self,
        _origin: LatLon,
        _destination: LatLon,
        _bounds: &RouteBounds,
        _bias: SeaPathBias,
    ) -> Option<Vec<LatLon>> {
        None
    }
}
