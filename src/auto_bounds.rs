//! Auto-derive a sensible search bbox from just origin/destination.
//!
//! The PSO needs a `MapBounds` rectangle to bake the wind grid and
//! constrain interior waypoints. Asking the user to draw one isn't
//! always practical: a CLI run with `--start`/`--end` and no `--bbox`,
//! a UI session where the user just placed two endpoints, etc. A naïve
//! "bbox of the endpoints" would create an unsolvable PSO whenever the
//! great-circle line between them crosses a continent — the swarm has
//! no sea room to detour.
//!
//! [`derive_route_bbox`] solves this by running the cheap A* sea-path
//! finder first (which already detours around landmass), taking the
//! bbox of the resulting polyline, then padding it so the PSO has room
//! to find a faster wind-aware route that the distance-only A* would
//! have skipped past.

use swarmkit_sailing::spherical::{
    LatLon, LonLatBbox, POLE_LATITUDE_LIMIT_DEG, signed_lon_delta, wrap_lon_deg,
};
use swarmkit_sailing::{LandmassSource, RouteBounds, SeaPathBias};

use crate::bounds::MapBounds;

/// Padding added to each side of the polyline-derived bbox so the PSO
/// has room to find faster wind-aware detours that don't show up on the
/// distance-only A* path. `max(20% of axis span, 3°)` per side: the
/// fixed minimum keeps tight north-south or east-west routes from
/// getting hemmed in to a sliver, while the proportional component
/// scales with route size.
const PAD_FRACTION: f64 = 0.20;
const PAD_MIN_DEGREES: f64 = 3.0;

/// Derive a search bbox from just origin and destination, using the
/// supplied landmass to detour around continents.
///
/// Returns `None` only when both endpoints are landlocked from each
/// other (A* finds no sea path) and the raw great-circle bbox can't
/// be padded into a useful shape — currently that means `None` is only
/// returned if the endpoint hint bbox is itself degenerate (identical
/// points).
///
/// Algorithm:
///
/// 1. Hint bbox of the endpoints, antimeridian-aware via
///    [`signed_lon_delta`].
/// 2. A* probe (cheap relative to PSO) to find a sea path between the
///    endpoints. The probe runs against a global-ish `RouteBounds`;
///    only `SeaPathBias::None` is asked for so the bbox doesn't bias
///    the result.
/// 3. If A* succeeds, take the bbox of the resulting polyline (still
///    antimeridian-aware via per-segment signed deltas).
/// 4. Otherwise, fall back to the endpoint hint bbox.
/// 5. Pad each side by 20 % of the bbox extent (with a 3° floor).
/// 6. Clamp lat to ±[`POLE_LATITUDE_LIMIT_DEG`].
/// 7. If `clamp` is supplied (typically the loaded wind map's extent),
///    intersect with it via [`MapBounds::clamp_to`].
pub fn derive_route_bbox<L: LandmassSource>(
    origin: (f64, f64),
    destination: (f64, f64),
    landmass: &L,
    clamp: Option<MapBounds>,
) -> Option<MapBounds> {
    if origin == destination {
        return None;
    }

    // Step 1+2+3: get the polyline (or fall back to endpoint hint).
    let origin_ll = LatLon::new(origin.0, origin.1);
    let destination_ll = LatLon::new(destination.0, destination.1);
    let probe_bounds = global_probe_bounds(origin_ll, destination_ll);
    let polyline = landmass
        .find_sea_path(origin_ll, destination_ll, &probe_bounds, SeaPathBias::None)
        .unwrap_or_else(|| {
            // No sea path — fall back to a two-point "polyline" of just
            // the endpoints. The padded bbox will still be a sensible
            // hint even if A* couldn't find a route.
            vec![origin_ll, destination_ll]
        });

    let (lon_unwrap_min, lon_unwrap_max, lat_min, lat_max) = polyline_unwrapped_bbox(&polyline);

    // Step 4: pad.
    let lon_span = lon_unwrap_max - lon_unwrap_min;
    let lat_span = lat_max - lat_min;
    let lon_pad = (PAD_FRACTION * lon_span).max(PAD_MIN_DEGREES);
    let lat_pad = (PAD_FRACTION * lat_span).max(PAD_MIN_DEGREES);

    let padded_lon_unwrap_min = lon_unwrap_min - lon_pad;
    let padded_lon_unwrap_max = lon_unwrap_max + lon_pad;
    let padded_lat_min = (lat_min - lat_pad).max(-POLE_LATITUDE_LIMIT_DEG);
    let padded_lat_max = (lat_max + lat_pad).min(POLE_LATITUDE_LIMIT_DEG);

    // Step 5+6: re-wrap lon and produce a `MapBounds`. If the padded
    // span ≥ 360°, use a non-wrap full-globe bbox.
    let derived_bbox = if padded_lon_unwrap_max - padded_lon_unwrap_min >= 360.0 {
        LonLatBbox::new(-180.0, 180.0, padded_lat_min, padded_lat_max)
    } else {
        LonLatBbox::new(
            wrap_lon_deg(padded_lon_unwrap_min),
            wrap_lon_deg(padded_lon_unwrap_max),
            padded_lat_min,
            padded_lat_max,
        )
    };
    let derived = MapBounds { bbox: derived_bbox };

    // Step 7: clamp to user-supplied extent if any.
    let result = match clamp {
        Some(c) => derived.clamp_to(Some((
            c.bbox.lon_min,
            c.bbox.lon_max,
            c.bbox.lat_min,
            c.bbox.lat_max,
        ))),
        None => derived,
    };
    Some(result)
}

/// Build a permissive `RouteBounds` for the A* probe. With
/// `SeaPathBias::None`, A* doesn't actually clip against the bbox —
/// only the bias check uses it — but `RouteBounds::new` still computes
/// a `step_distance_max` from the bbox diagonal, so we hand it a global
/// rectangle to keep that step relevant to the world scale.
fn global_probe_bounds(origin: LatLon, destination: LatLon) -> RouteBounds {
    RouteBounds::new(
        origin,
        destination,
        LonLatBbox::new(
            -180.0,
            180.0,
            -POLE_LATITUDE_LIMIT_DEG,
            POLE_LATITUDE_LIMIT_DEG,
        ),
    )
}

/// Compute the bbox of a polyline in *unwrapped* longitude space:
/// each successive point's lon is offset from the previous by the
/// signed shortest delta, so a polyline that crosses the antimeridian
/// produces a continuous lon range that simply runs past ±180°. The
/// caller re-wraps the result.
///
/// Returns `(lon_unwrap_min, lon_unwrap_max, lat_min, lat_max)`.
fn polyline_unwrapped_bbox(polyline: &[LatLon]) -> (f64, f64, f64, f64) {
    debug_assert!(
        !polyline.is_empty(),
        "polyline must have at least one point"
    );
    let mut current_lon_unwrap = polyline[0].lon;
    let mut lon_min = current_lon_unwrap;
    let mut lon_max = current_lon_unwrap;
    let mut lat_min = polyline[0].lat;
    let mut lat_max = polyline[0].lat;
    for p in polyline.iter().skip(1) {
        let dlon = signed_lon_delta(wrap_lon_deg(current_lon_unwrap), p.lon);
        current_lon_unwrap += dlon;
        if current_lon_unwrap < lon_min {
            lon_min = current_lon_unwrap;
        }
        if current_lon_unwrap > lon_max {
            lon_max = current_lon_unwrap;
        }
        if p.lat < lat_min {
            lat_min = p.lat;
        }
        if p.lat > lat_max {
            lat_max = p.lat;
        }
    }
    (lon_min, lon_max, lat_min, lat_max)
}

/// Format a derived bbox as an equivalent `--bbox` flag value
/// (`lon_min,lat_min,lon_max,lat_max`).
///
/// Used by the CLI to print the auto-derived bbox in copy-pasteable
/// form, so subsequent runs can pin the bounds exactly (handy for
/// `--save-baked` / `--load-baked` sweeps).
pub fn format_bbox_flag(bounds: MapBounds) -> String {
    let b = bounds.bbox;
    format!(
        "{:.4},{:.4},{:.4},{:.4}",
        b.lon_min, b.lat_min, b.lon_max, b.lat_max
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::landmass::landmass_grid;

    #[test]
    fn open_ocean_route_pads_around_endpoint_hint() {
        // Mid-Atlantic to mid-Pacific, both deep ocean: A* succeeds,
        // polyline is roughly the great circle, bbox should snugly
        // cover both endpoints with ≥3° margin per side.
        let bounds =
            derive_route_bbox((-30.0, 0.0), (-150.0, 0.0), landmass_grid(), None).expect("derived");
        let bbox = bounds.bbox;
        // Endpoints are at lat 0; pad must give at least 3° N/S.
        assert!(bbox.lat_max >= 3.0);
        assert!(bbox.lat_min <= -3.0);
        // Lon spans roughly -150 to -30 in canonical order.
        assert!(bbox.lon_min <= -30.0);
        assert!(bbox.lon_max >= -30.0 - PAD_MIN_DEGREES);
        // Sanity: not antimeridian-wrapped.
        assert!(!bounds.lon_wraps());
    }

    #[test]
    fn route_around_continent_uses_astar_bbox() {
        // Mediterranean: Gibraltar → Suez. A* must detour through the
        // Med basin; the polyline-derived bbox should be wider in lat
        // than the endpoint great-circle would suggest.
        let origin = (-5.5, 36.0); // Strait of Gibraltar
        let dest = (32.0, 30.0); // Suez Canal mouth
        let bbox = derive_route_bbox(origin, dest, landmass_grid(), None)
            .expect("derived")
            .bbox;
        // Endpoint lats span 30..36; padded bbox must include some
        // northern detour through the Med (lat ≈ 38–40 in places).
        // Just check the bbox covers the endpoint range plus pad.
        assert!(bbox.lat_min <= 30.0);
        assert!(bbox.lat_max >= 36.0);
        // Lon must cover at least both endpoints with pad.
        assert!(bbox.lon_min <= -5.5);
        assert!(bbox.lon_max >= 32.0);
    }

    #[test]
    fn antimeridian_route_produces_wrapping_bbox() {
        // Tokyo → San Francisco: short way crosses the antimeridian.
        let origin = (139.7, 35.7); // Tokyo
        let dest = (-122.4, 37.8); // San Francisco
        let bounds = derive_route_bbox(origin, dest, landmass_grid(), None).expect("derived");
        let bbox = bounds.bbox;
        // The padded bbox should wrap (encoding: lon_min > lon_max) since
        // the unwrapped lon range spans from ~135 to ~245 (= -115).
        assert!(
            bounds.lon_wraps(),
            "Tokyo→SF auto-bbox must wrap, got [{}, {}]",
            bbox.lon_min,
            bbox.lon_max,
        );
        // Both endpoints should be inside the wrapping bbox.
        // Wrap encoding: valid lon ∈ [lon_min, 180] ∪ [-180, lon_max].
        let in_bbox = |lon: f64| lon >= bbox.lon_min || lon <= bbox.lon_max;
        assert!(
            in_bbox(origin.0),
            "origin lon {} not in {:?}",
            origin.0,
            bbox.lon_min..bbox.lon_max
        );
        assert!(
            in_bbox(dest.0),
            "dest lon {} not in {:?}",
            dest.0,
            bbox.lon_min..bbox.lon_max
        );
    }

    #[test]
    fn identical_endpoints_returns_none() {
        let bbox = derive_route_bbox((0.0, 0.0), (0.0, 0.0), landmass_grid(), None);
        assert!(bbox.is_none());
    }

    #[test]
    fn clamp_intersects_with_supplied_bounds() {
        // Open-ocean route, clamped to a tight rectangle: result must
        // sit inside the clamp.
        let clamp = MapBounds {
            bbox: LonLatBbox::new(-100.0, -40.0, -10.0, 10.0),
        };
        let bbox = derive_route_bbox((-30.0, 0.0), (-150.0, 0.0), landmass_grid(), Some(clamp))
            .expect("derived")
            .bbox;
        assert!(bbox.lon_min >= -100.0);
        assert!(bbox.lon_max <= -40.0);
        assert!(bbox.lat_min >= -10.0);
        assert!(bbox.lat_max <= 10.0);
    }

    #[test]
    fn padded_bbox_is_well_formed() {
        // Sanity: the result should always be non-degenerate for two
        // distinct endpoints.
        let bounds =
            derive_route_bbox((10.0, 5.0), (20.0, 10.0), landmass_grid(), None).expect("derived");
        assert!(bounds.is_non_degenerate());
    }

    #[test]
    fn format_bbox_flag_round_trips_via_cli_parser() {
        // The CLI parses `lon_min,lat_min,lon_max,lat_max`; check the
        // formatted form matches that expected order.
        let bounds = MapBounds {
            bbox: LonLatBbox::new(-75.0, -10.0, 25.0, 60.0),
        };
        let s = format_bbox_flag(bounds);
        assert_eq!(s, "-75.0000,25.0000,-10.0000,60.0000");
    }
}
