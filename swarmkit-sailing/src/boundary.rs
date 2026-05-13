use crate::dynamics::get_travel_time_range;
use crate::route_bounds::RouteBounds;
use crate::spherical::{LatLon, Segment, TangentMetres};
use crate::units::{Floats, Path, Time};
use crate::{LandmassSource, Sailboat, WindSource};
use swarmkit::{Boundary, Contextful};

/// Outer-search boundary: clamps interior waypoints to the route bounds,
/// pushes any waypoint that landed inside a coastline back over water
/// using the [`LandmassSource`]'s SDF, and clamps per-segment times to
/// each segment's `(best, worst)` range under the current wind field.
/// Holds direct refs to the components it needs — no shared context
/// object.
///
/// Land projection is a no-op for [`crate::LandmassSourceDummy`] (which
/// reports every cell as deep open water), so callers without landmass
/// data pay nothing for the new code path.
pub(crate) struct SailingPathBoundary<
    'a,
    const N: usize,
    SB: Sailboat,
    WS: WindSource,
    LS: LandmassSource,
> {
    bounds: &'a RouteBounds,
    boat: &'a SB,
    wind_source: &'a WS,
    landmass: &'a LS,
}

impl<'a, const N: usize, SB: Sailboat, WS: WindSource, LS: LandmassSource>
    SailingPathBoundary<'a, N, SB, WS, LS>
{
    pub fn new(
        bounds: &'a RouteBounds,
        boat: &'a SB,
        wind_source: &'a WS,
        landmass: &'a LS,
    ) -> Self {
        SailingPathBoundary {
            bounds,
            boat,
            wind_source,
            landmass,
        }
    }
}

impl<const N: usize, SB: Sailboat, WS: WindSource, LS: LandmassSource> Contextful
    for SailingPathBoundary<'_, N, SB, WS, LS>
{
    type TContext = Path<N>;
}

impl<const N: usize, SB: Sailboat, WS: WindSource, LS: LandmassSource> Boundary
    for SailingPathBoundary<'_, N, SB, WS, LS>
{
    type T = Path<N>;

    fn handle(&self, pos: Self::T) -> Self::T {
        #[cfg(feature = "profile-timers")]
        let __profile_start = std::time::Instant::now();

        let mut clamped = self.bounds.constrain_xy(&pos);
        // Project each interior waypoint that landed inside land back
        // toward water. Endpoints are pinned by `constrain_xy` and stay
        // put — projecting them would fight the user's intent.
        for i in 1..N - 1 {
            let projected = project_off_land(self.landmass, clamped.xy.lat_lon(i));
            // Re-clamp to bbox: a single linearised projection step can
            // overshoot the bounds, especially near concave coasts. If
            // the clamp pushes the waypoint back inside land, the Stage
            // 3 fitness penalty handles the residual.
            clamped.xy.set_lat_lon(i, self.bounds.clamp(projected));
        }
        let mut times = [0.0f64; N];
        let mut acc_time = 0.0;
        for (i, seg) in clamped.iter_segments().enumerate() {
            let (a, b) = get_travel_time_range(
                self.boat,
                self.wind_source,
                Segment {
                    origin: seg.origin,
                    destination: seg.destination,
                    origin_time: acc_time,
                    step_distance_max: self.bounds.step_distance_max,
                },
            );
            times[i + 1] = f64::clamp(pos.t[i + 1], a, b);
            acc_time += times[i + 1];
        }
        clamped.t = Time(Floats(times));

        #[cfg(feature = "profile-timers")]
        crate::profile_timers::SAILING_BOUNDARY.record(__profile_start.elapsed().as_nanos() as u64);

        clamped
    }
}

/// Iterations cap for the projection loop. Concave coastlines can leave
/// the gradient pointing in a "wrong" direction — usually one or two
/// steps is enough to escape; the cap is defensive against pathological
/// SDF saddles.
const PROJECTION_MAX_ITER: usize = 8;

/// How far off the coast (in metres) a projected waypoint must end up
/// before the iteration stops. A small multiple of the typical SDF cell
/// size; 50 km comfortably exceeds bilinear-interpolation noise on the
/// 0.5° grid the default `LandmassSource` produces.
const SAFETY_BUFFER_M: f64 = 50_000.0;

/// Cap on the per-iteration projection step size, in metres. Bounds the
/// linearised step so that a waypoint deep inside a continent doesn't
/// teleport across the world based on a single gradient lookup.
const MAX_PROJECTION_STEP_M: f64 = 100_000.0;

/// Probe distance for the finite-difference fallback (m). Chosen to
/// match `MAX_PROJECTION_STEP_M`'s order of magnitude so the probe
/// crosses at least one SDF cell at the default 0.5° resolution.
const FINITE_DIFFERENCE_PROBE_M: f64 = 50_000.0;

/// Push `waypoint` along the SDF gradient until it sits at least
/// [`SAFETY_BUFFER_M`] outside any landmass, capped at
/// [`PROJECTION_MAX_ITER`] iterations. No-op if the waypoint is already
/// over open water.
fn project_off_land<LS: LandmassSource>(landmass: &LS, waypoint: LatLon) -> LatLon {
    let mut p = waypoint;
    let mut sd = landmass.signed_distance_m(p);
    if sd >= SAFETY_BUFFER_M {
        return p;
    }
    for _ in 0..PROJECTION_MAX_ITER {
        let g = landmass.gradient(p);
        let dir = if g.norm_squared() > 1e-12 {
            g.normalize()
        } else if let Some(d) = finite_difference_gradient(landmass, p, sd) {
            d
        } else {
            // No gradient anywhere within the probe — give up; the
            // fitness penalty in Stage 3 still pushes the particle out.
            break;
        };
        let step_m = (SAFETY_BUFFER_M - sd).clamp(0.0, MAX_PROJECTION_STEP_M);
        p = p.offset_by(dir * step_m);
        sd = landmass.signed_distance_m(p);
        if sd >= SAFETY_BUFFER_M {
            break;
        }
    }
    p
}

/// Cardinal-probe gradient estimate: sample the SDF at four offsets in
/// the local east-north tangent frame and return the unit direction in
/// which the SDF rises fastest. Returns `None` if all four probes are
/// no better than `centre_sd` (saddle / flat region with no clear
/// outward direction).
fn finite_difference_gradient<LS: LandmassSource>(
    landmass: &LS,
    p: LatLon,
    centre_sd: f64,
) -> Option<TangentMetres> {
    const PROBES: [TangentMetres; 4] = [
        TangentMetres::new(FINITE_DIFFERENCE_PROBE_M, 0.0),
        TangentMetres::new(-FINITE_DIFFERENCE_PROBE_M, 0.0),
        TangentMetres::new(0.0, FINITE_DIFFERENCE_PROBE_M),
        TangentMetres::new(0.0, -FINITE_DIFFERENCE_PROBE_M),
    ];
    let mut best_dir: Option<TangentMetres> = None;
    let mut best_diff = 0.0;
    for probe_dir in PROBES {
        let probe_pos = p.offset_by(probe_dir);
        let probe_sd = landmass.signed_distance_m(probe_pos);
        let diff = probe_sd - centre_sd;
        if diff > best_diff {
            best_diff = diff;
            best_dir = Some(probe_dir.normalize());
        }
    }
    best_dir
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LandmassSourceDummy;

    /// `LandmassSource` impl that treats an axis-aligned `(lon, lat)`
    /// box as land. Crude — degrees treated as metres for SDF magnitude
    /// — but adequate for verifying projection direction.
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
            let deg_to_m = 111_000.0;
            let outside = dx.max(0.0).hypot(dy.max(0.0)) * deg_to_m;
            let inside = dx.max(dy).min(0.0) * deg_to_m;
            if outside > 0.0 { outside } else { inside }
        }
        // Gradient deliberately not implemented — exercises the
        // finite-difference fallback path.
    }

    #[test]
    fn project_off_land_pushes_waypoint_out_of_synthetic_island() {
        let land = BboxLand {
            lon_min: -2.0,
            lon_max: 2.0,
            lat_min: -2.0,
            lat_max: 2.0,
        };
        // Waypoint right in the centre of the island.
        let inside = LatLon::new(0.0, 0.0);
        assert!(land.is_land(inside));
        let projected = project_off_land(&land, inside);
        let sd = land.signed_distance_m(projected);
        assert!(
            sd >= 0.0,
            "expected projected waypoint to be over water, got SDF = {sd} m at {projected:?}",
        );
    }

    #[test]
    fn project_off_land_is_noop_for_dummy_source() {
        let land = LandmassSourceDummy;
        let p = LatLon::new(3.0, 4.0);
        let projected = project_off_land(&land, p);
        assert_eq!(projected, p, "dummy source must short-circuit projection");
    }

    #[test]
    fn project_off_land_is_noop_when_already_safe() {
        let land = BboxLand {
            lon_min: -2.0,
            lon_max: 2.0,
            lat_min: -2.0,
            lat_max: 2.0,
        };
        // Waypoint well outside the safety buffer.
        let p = LatLon::new(20.0, 20.0);
        let projected = project_off_land(&land, p);
        assert_eq!(projected, p, "safe waypoint must not move");
    }
}
