use crate::route_bounds::RouteBounds;
use crate::spherical::{LatLon, Segment, TangentMetres, Wind};

/// Direction bias for [`LandmassSource::find_sea_path`].
///
/// Used to pull the pathfinder toward one side of an obstruction so
/// distinct macro topologies (around the north of a landmass vs around
/// the south) can each be obtained as separate baselines for init.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SeaPathBias {
    /// No bias — return the shortest land-free path.
    None,
    /// Prefer routes that stay north of the straight-line interpolation
    /// between origin and destination.
    North,
    /// Prefer routes that stay south of the straight-line interpolation.
    South,
}

pub trait WindSource: Sync {
    /// Sample the wind at `(location, t)`. Returns the wind velocity in
    /// the local east-north tangent frame at `location`, in m/s.
    fn sample_wind(&self, location: LatLon, t: f64) -> Wind;
}

/// Zero-wind `WindSource` for tests/demos where the wind field is irrelevant.
pub struct WindSourceDummy;

impl WindSource for WindSourceDummy {
    fn sample_wind(&self, _location: LatLon, _t: f64) -> Wind {
        Wind::zero()
    }
}

pub trait Sailboat: Sync {
    /// Fuel consumed (kg) by burning at the load `mcr_01 ∈ [0, 1]` for `delta_time` seconds.
    fn get_fuel_consumed(&self, mcr_01: f64, delta_time: f64) -> f64;
    fn get_travel_time<WS: WindSource>(
        &self,
        wind_source: &WS,
        segment: Segment,
        mcr_01: f64,
    ) -> f64;

    /// Batched form of [`Self::get_travel_time`] when many `mcr_01`
    /// values share the same [`Segment`].
    ///
    /// Typical caller is `SegmentRangeTables::build`, which fills a
    /// per-segment travel-time × mcr table. Concrete impls can share
    /// the great-circle substep walk across mcr values
    /// (`initial_bearing` + `destination_point` per substep are
    /// mcr-independent), avoiding redundant trig that the per-mcr loop
    /// would otherwise repeat.
    ///
    /// `out.len()` must equal `mcr_01_values.len()`. The default impl
    /// loops over `get_travel_time` per mcr and gives correct results
    /// for any `Sailboat`; the [`crate::Boat`] override is where the
    /// geometric sharing actually saves work.
    fn get_travel_times_for_mcrs<WS: WindSource>(
        &self,
        wind_source: &WS,
        segment: Segment,
        mcr_01_values: &[f64],
        out: &mut [f64],
    ) {
        debug_assert_eq!(
            mcr_01_values.len(),
            out.len(),
            "out buffer must match input mcr_01 length",
        );
        for (i, &mcr) in mcr_01_values.iter().enumerate() {
            out[i] = self.get_travel_time(wind_source, segment, mcr);
        }
    }
}

/// Static landmass / coastline data for the search to consult.
///
/// Implementers expose a signed-distance field (negative inside land,
/// positive over water, zero on the coast) plus an outward-pointing
/// gradient.
///
/// The default no-op impl is [`crate::LandmassSourceDummy`], which reports
/// every location as open water with no coast nearby; pass it when no
/// landmass data is available so the search behaves as it did before
/// landmass support existed.
pub trait LandmassSource: Sync {
    /// Signed distance from `location` to the nearest coast, in real ground
    /// metres. Negative inside land, positive over water, zero on the coast.
    fn signed_distance_m(&self, location: LatLon) -> f64;

    /// Outward gradient at `location`, in tangent-frame metres per metre.
    /// Points away from land; magnitude is approximately 1 over open water
    /// and may be smaller at multi-coast saddle points. Default returns
    /// zero — implementers without a usable gradient can skip it; callers
    /// that need a gradient (e.g. waypoint projection) fall back to a
    /// finite-difference probe on `signed_distance_m` in that case.
    fn gradient(&self, _location: LatLon) -> TangentMetres {
        TangentMetres::zero()
    }

    /// Convenience: `true` iff `location` is inside a landmass.
    fn is_land(&self, location: LatLon) -> bool {
        self.signed_distance_m(location) < 0.0
    }

    /// Find a polyline of `(lon°, lat°)` points from `origin` to
    /// `destination` that stays over water, restricted to the search
    /// rectangle in `bounds`. The polyline starts at `origin` and ends
    /// at `destination` exactly; interior points are landmass-aware
    /// waypoints (for example, cell centres of an A* search over the
    /// SDF grid). `bias` lets callers request distinct macro topologies.
    ///
    /// Default returns `None`. Implementations without pathfinding
    /// (e.g. [`crate::LandmassSourceDummy`]) inherit the default; the
    /// init layer falls back to the straight-line baseline when no
    /// polyline is available.
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

/// Land-free `LandmassSource` for tests/demos and for callers that
/// don't have landmass data.
///
/// Reports every location as deep open water (`+∞` signed distance)
/// with zero gradient, so penalties and projections driven by the
/// trait are exactly disabled.
pub struct LandmassSourceDummy;

impl LandmassSource for LandmassSourceDummy {
    fn signed_distance_m(&self, _location: LatLon) -> f64 {
        f64::INFINITY
    }
}

/// Lets downstream fit calcs (notably `TimeFitCalc`) reach
/// `SailboatFitCalc`'s state without binding to the concrete type.
///
/// State accessible through this trait: the ship, wind source,
/// landmass source, integration step, departure time, and weights.
pub trait SailboatFitData: Sync {
    type Ship: Sailboat;
    type Wind: WindSource;
    type Land: LandmassSource;

    fn ship(&self) -> &Self::Ship;
    fn wind_source(&self) -> &Self::Wind;
    fn landmass(&self) -> &Self::Land;
    fn step_distance_max(&self) -> f64;
    fn departure_time(&self) -> f64;
    fn time_weight(&self) -> f64;
    fn fuel_weight(&self) -> f64;
    /// Penalty per metre of segment that lies inside a landmass. Set
    /// to zero to disable the soft constraint and recover the
    /// pre-landmass-support behaviour exactly.
    fn land_weight(&self) -> f64;
}
