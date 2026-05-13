#![cfg(feature = "probe-stats")]

mod common;

use common::{GapWind, route_bounds_for_smoke};
use swarmkit_sailing::units::{Floats, PathXY};
use swarmkit_sailing::{Boat, RouteBounds, SegmentRangeTables};

const WAYPOINTS: usize = 10;

/// Straight-line waypoints from origin to destination of `bounds`, sampled
/// uniformly across N points.
fn straight_path<const N: usize>(bounds: &RouteBounds) -> PathXY<N> {
    let origin = bounds.origin;
    let destination = bounds.destination;
    let mut xs = [0.0; N];
    let mut ys = [0.0; N];
    for i in 0..N {
        let t = if N > 1 {
            i as f64 / (N - 1) as f64
        } else {
            0.0
        };
        xs[i] = origin.lon + t * (destination.lon - origin.lon);
        ys[i] = origin.lat + t * (destination.lat - origin.lat);
    }
    PathXY(Floats(xs), Floats(ys))
}

#[test]
fn probe_counters_track_in_and_out_of_range_queries() {
    let wind = GapWind::smoke_default();
    let bounds = route_bounds_for_smoke();
    let boat = Boat::default();
    let xy = straight_path::<WAYPOINTS>(&bounds);

    let tables =
        SegmentRangeTables::<WAYPOINTS>::build(&xy, &boat, &wind, bounds.step_distance_max, 0.0);

    // Fresh instance starts at zero.
    let initial = tables.probe_stats();
    assert_eq!(initial.total, 0);
    assert_eq!(initial.out_of_range, 0);

    // Phase 1: a deliberately in-range query at dep = 0 on segment 0
    // (segment 0 is single-point so any dep clamps cleanly; pick a later
    // segment for a non-degenerate range).
    let seg = WAYPOINTS / 2;
    tables.query_range(seg, 0.0);
    let after_in_range = tables.probe_stats();
    assert_eq!(after_in_range.total, 1, "locate should have run once");

    // Phase 2: out-of-range queries — wildly negative and wildly positive
    // dep values that are guaranteed past the tabulated [dep_min, dep_max].
    tables.query_range(seg, -1e9);
    tables.query_range(seg, 1e9);
    let after_out = tables.probe_stats();
    assert_eq!(after_out.total, 3);
    assert!(
        after_out.out_of_range >= 2,
        "expected at least the two extreme queries to flag out-of-range, got {}",
        after_out.out_of_range
    );

    // Reset returns the counter to zero without affecting the cache contents.
    tables.reset_probe_stats();
    let after_reset = tables.probe_stats();
    assert_eq!(after_reset.total, 0);
    assert_eq!(after_reset.out_of_range, 0);

    // Cache still works after reset.
    tables.query_range(seg, 0.0);
    assert_eq!(tables.probe_stats().total, 1);
}

#[test]
fn cloned_tables_get_independent_counters() {
    let wind = GapWind::smoke_default();
    let bounds = route_bounds_for_smoke();
    let boat = Boat::default();
    let xy = straight_path::<WAYPOINTS>(&bounds);

    let original =
        SegmentRangeTables::<WAYPOINTS>::build(&xy, &boat, &wind, bounds.step_distance_max, 0.0);
    let seg = WAYPOINTS / 2;
    original.query_range(seg, 0.0);
    original.query_range(seg, 0.0);
    assert_eq!(original.probe_stats().total, 2);

    let clone = original.clone();
    // Snapshot at clone-time carries forward.
    assert_eq!(clone.probe_stats().total, 2);

    // Subsequent queries are accounted to each instance independently.
    clone.query_range(seg, 0.0);
    assert_eq!(clone.probe_stats().total, 3);
    assert_eq!(original.probe_stats().total, 2);
}
