use swarmkit_sailing::RouteBounds;
use swarmkit_sailing::spherical::LonLatBbox;

use crate::TimedWindMap;
use crate::wind_map::BakeBounds;

/// Hard cap on the bake grid resolution per axis. The bake step is grown
/// past the caller-requested value when needed to honour this. Sized so
/// the worst case stays under ~500 MB at typical frame counts:
/// 1024 × 1024 × 50 frames × 16 B = 838 MB.
const MAX_BAKE_CELLS_PER_SIDE: f64 = 1024.0;

/// Tight axis-aligned bounding box of a wind map's sample positions.
///
/// Domain wrapper around [`LonLatBbox`] — same lon-lat-degrees-with-wrap
/// shape, plus the wind-map-specific factories (`from_wind_map`,
/// `to_route_bounds`, `to_bake_bounds`, `resolve_endpoints`). Generic
/// bbox queries (`is_non_degenerate`, `wraps_antimeridian`,
/// `lon_extent`, etc.) go through the embedded `bbox` field.
#[derive(Clone, Copy)]
pub struct MapBounds {
    pub bbox: LonLatBbox,
}

impl MapBounds {
    /// `true` if this bounding box wraps east through the antimeridian
    /// — encoded as `lon_min > lon_max` (canonical longitudes). Latitude
    /// is straightforward: it never wraps.
    pub fn lon_wraps(self) -> bool {
        self.bbox.wraps_antimeridian()
    }

    pub fn from_wind_map(wind_map: &TimedWindMap) -> Option<Self> {
        let frame = wind_map.frame(0)?;
        let rows = frame.rows();
        if rows.is_empty() {
            return None;
        }
        Some(Self {
            bbox: LonLatBbox::new(
                rows.iter()
                    .map(|r| f64::from(r.lon))
                    .fold(f64::INFINITY, f64::min),
                rows.iter()
                    .map(|r| f64::from(r.lon))
                    .fold(f64::NEG_INFINITY, f64::max),
                rows.iter()
                    .map(|r| f64::from(r.lat))
                    .fold(f64::INFINITY, f64::min),
                rows.iter()
                    .map(|r| f64::from(r.lat))
                    .fold(f64::NEG_INFINITY, f64::max),
            ),
        })
    }

    /// Build a [`RouteBounds`] for the sailing search. `origin` and
    /// `destination` are the start / end waypoints; callers pass the
    /// user's editor endpoints when set, or the bbox corners as a
    /// fallback. The bbox extents come from `self.bbox` regardless —
    /// they only constrain the PSO's interior waypoints.
    pub fn to_route_bounds(self, origin: (f64, f64), destination: (f64, f64)) -> RouteBounds {
        RouteBounds::new(origin, destination, self.bbox)
    }

    /// Same as [`Self::to_route_bounds`] but with a caller-chosen
    /// `step_distance_max = fraction * bbox_diagonal`. Forwards to
    /// [`RouteBounds::new_with_step_fraction`].
    pub fn to_route_bounds_with_step_fraction(
        self,
        origin: (f64, f64),
        destination: (f64, f64),
        fraction: f64,
    ) -> RouteBounds {
        RouteBounds::new_with_step_fraction(origin, destination, self.bbox, fraction)
    }

    /// Resolve the search's origin/destination given an optional user-set
    /// pair. Falls back to the bbox diagonal (`(lon_min, lat_min)` →
    /// `(lon_max, lat_max)`), which preserves the historical default
    /// when no waypoints have been placed.
    pub fn resolve_endpoints(
        self,
        start: Option<(f64, f64)>,
        end: Option<(f64, f64)>,
    ) -> ((f64, f64), (f64, f64)) {
        (
            start.unwrap_or((self.bbox.lon_min, self.bbox.lat_min)),
            end.unwrap_or((self.bbox.lon_max, self.bbox.lat_max)),
        )
    }

    /// Intersect this bbox with `(lon_min, lon_max, lat_min, lat_max)`.
    /// Used to shrink the search/bake domain to the user-defined Route
    /// Bounds rectangle before calling [`Self::to_route_bounds`] /
    /// [`Self::to_bake_bounds`]. Returns the original bbox if `sub` is
    /// `None`. The clamp can produce an empty rectangle if the user's
    /// bounds don't overlap the wind map at all — callers should guard.
    ///
    /// Antimeridian-wrapping `sub` (encoded `lon_min > lon_max`) is
    /// passed through verbatim for the longitude axis: the wind map is
    /// presumed wide enough to cover both halves of a wrap. The latitude
    /// axis is intersected normally. The full "wrap-with-non-wrap
    /// intersection" case is left for a follow-up — uncommon enough in
    /// practice that the simplification is fine for now.
    pub fn clamp_to(self, sub: Option<(f64, f64, f64, f64)>) -> Self {
        let Some((sub_lon_min, sub_lon_max, sub_lat_min, sub_lat_max)) = sub else {
            return self;
        };
        let lat_min = self.bbox.lat_min.max(sub_lat_min);
        let lat_max = self.bbox.lat_max.min(sub_lat_max);
        let (lon_min, lon_max) = if sub_lon_min > sub_lon_max || self.bbox.wraps_antimeridian() {
            // Either side wraps: pass through `sub`'s wrap convention.
            (sub_lon_min, sub_lon_max)
        } else {
            // Both non-wrap: standard 1D intersection.
            (
                self.bbox.lon_min.max(sub_lon_min),
                self.bbox.lon_max.min(sub_lon_max),
            )
        };
        Self {
            bbox: LonLatBbox::new(lon_min, lon_max, lat_min, lat_max),
        }
    }

    /// True if the bbox has positive area. Use after [`Self::clamp_to`]
    /// to guard against the user drawing a Route Bounds rectangle that
    /// lies entirely outside the wind map. Wrapping bboxes
    /// (`lon_min > lon_max`) are non-degenerate as long as the lons aren't
    /// equal — they cover `[lon_min, 180] ∪ [−180, lon_max]`.
    pub fn is_non_degenerate(self) -> bool {
        self.bbox.is_non_degenerate()
    }

    /// Build a [`BakeBounds`] for the search's spatial precompute.
    /// `step` is the *requested* cell size in degrees; if honouring it
    /// would exceed the per-axis cell cap (1024 cells per side, sized
    /// to keep a worst-case 1024×1024×50-frame bake under ~500 MB), the
    /// step is grown to fit the cap and a warning is logged.
    ///
    /// The wrap encoding is carried through verbatim — the bake-time
    /// `+360` extension to keep the lon axis monotonic is now done at
    /// use time via [`LonLatBbox::lon_max_unwrapped`] inside
    /// `BakedWindMap::from_timed_map`.
    pub fn to_bake_bounds(self, step: f64) -> BakeBounds {
        let extent_x = self.bbox.lon_extent();
        let extent_y = self.bbox.lat_extent();
        let max_extent = extent_x.max(extent_y);
        let min_step_for_cap = max_extent / MAX_BAKE_CELLS_PER_SIDE;
        let effective_step = step.max(min_step_for_cap);
        if effective_step > step {
            log::warn!(
                "bake step grown from {step} to {effective_step} so the bake grid stays \
                 under {MAX_BAKE_CELLS_PER_SIDE} cells per side (map extent {max_extent})",
            );
        }
        BakeBounds {
            bbox: self.bbox,
            step: effective_step,
            coord_scale: 1.0,
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "tests rely on bit-exact comparisons of constant or stored f32/f64 values."
    )]
    use super::*;

    #[test]
    fn from_wind_map_uniform_grid_matches_generate_inputs() {
        // generate(100, 100, 50, ...) builds x in {0, 50, 100} (cols = 3) and
        // the same for y.
        let wm = TimedWindMap::generate(100.0, 100.0, 50.0, 1, 3600.0);
        let bounds = MapBounds::from_wind_map(&wm).expect("non-empty");
        assert_eq!(bounds.bbox.lon_min, 0.0);
        assert_eq!(bounds.bbox.lon_max, 100.0);
        assert_eq!(bounds.bbox.lat_min, 0.0);
        assert_eq!(bounds.bbox.lat_max, 100.0);
    }

    #[test]
    fn to_bake_bounds_carries_extents_and_step() {
        // Extent 120 ≪ 1024 * 7.5, so the step is honoured as-is.
        let b = MapBounds {
            bbox: LonLatBbox::new(-10.0, 110.0, 5.0, 95.0),
        };
        let bb = b.to_bake_bounds(7.5);
        assert_eq!(bb.bbox.lon_min, -10.0);
        assert_eq!(bb.bbox.lon_max, 110.0);
        assert_eq!(bb.bbox.lat_min, 5.0);
        assert_eq!(bb.bbox.lat_max, 95.0);
        assert_eq!(bb.step, 7.5);
        assert_eq!(bb.coord_scale, 1.0);
    }

    #[test]
    fn to_bake_bounds_grows_step_to_cap_grid_resolution() {
        // GFS-scale extent: 4e7 m wide. With BAKE_STEP=5 the unclamped grid
        // would be 8M cells per side; clamp must grow the step so neither
        // axis exceeds MAX_BAKE_CELLS_PER_SIDE = 1024.
        let b = MapBounds {
            bbox: LonLatBbox::new(-2.0e7, 2.0e7, -1.0e7, 1.0e7),
        };
        let bb = b.to_bake_bounds(5.0);
        let lon_span = bb.bbox.lon_max - bb.bbox.lon_min;
        let lat_span = bb.bbox.lat_max - bb.bbox.lat_min;
        let nx = (lon_span / bb.step).ceil() as usize + 1;
        let ny = (lat_span / bb.step).ceil() as usize + 1;
        assert!(nx <= MAX_BAKE_CELLS_PER_SIDE as usize + 1, "nx = {nx}");
        assert!(ny <= MAX_BAKE_CELLS_PER_SIDE as usize + 1, "ny = {ny}");
        assert!(
            bb.step > 5.0,
            "expected step grown above requested 5.0, got {}",
            bb.step
        );
    }

    #[test]
    fn to_route_bounds_does_not_panic() {
        // RouteBounds fields aren't all public; this test just exercises the
        // construction path so a future RouteBounds::new signature change
        // doesn't go unnoticed here.
        let b = MapBounds {
            bbox: LonLatBbox::new(0.0, 100.0, 0.0, 100.0),
        };
        let _rb = b.to_route_bounds((0.0, 0.0), (100.0, 100.0));
    }

    #[test]
    fn resolve_endpoints_falls_back_to_bbox_corners() {
        let b = MapBounds {
            bbox: LonLatBbox::new(-5.0, 15.0, 1.0, 9.0),
        };
        let (start, end) = b.resolve_endpoints(None, None);
        assert_eq!(start, (-5.0, 1.0));
        assert_eq!(end, (15.0, 9.0));
    }

    #[test]
    fn resolve_endpoints_honours_user_overrides() {
        let b = MapBounds {
            bbox: LonLatBbox::new(-5.0, 15.0, 1.0, 9.0),
        };
        let (start, end) = b.resolve_endpoints(Some((0.0, 0.0)), Some((10.0, 5.0)));
        assert_eq!(start, (0.0, 0.0));
        assert_eq!(end, (10.0, 5.0));
    }
}
