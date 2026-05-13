use std::collections::HashSet;

use crate::LandmassSource;
use crate::dynamics::get_segment_land_metres;
use crate::route_bounds::RouteBounds;
use crate::spherical::{LatLon, TangentMetres, delta_to_tangent_metres, haversine};

/// Reference path that the init families perturb.
///
/// Each particle is built by walking the baseline and adding
/// `offset[i] * scale[i]` metres along `perpendiculars[i]` at every
/// interior waypoint. Scale is `perp_scale_pos` for `offset ≥ 0`,
/// `perp_scale_neg` otherwise — asymmetric so a polyline running
/// closer to one coast can still wiggle freely on the other side.
/// Decouples "where the path goes" from "how particles vary."
#[derive(Clone, Debug)]
pub struct PathBaseline<const N: usize> {
    pub positions: [LatLon; N],
    /// Outward perpendicular, unit east-north tangent (90° CCW of the
    /// local tangent direction).
    pub perpendiculars: [TangentMetres; N],
    /// Perturbation scale (metres) for the +perpendicular side.
    pub perp_scale_pos: [f64; N],
    /// Perturbation scale (metres) for the −perpendicular side.
    pub perp_scale_neg: [f64; N],
}

impl<const N: usize> PathBaseline<N> {
    /// Straight-line lerp baseline. Symmetric scale = half the
    /// haversine route length on both sides.
    pub fn straight_line(bounds: &RouteBounds) -> Self {
        let mut positions = [LatLon::default(); N];
        for (i, slot) in positions.iter_mut().enumerate() {
            let t = i as f64 / (N - 1) as f64;
            *slot = bounds.lerp_between_endpoints(t);
        }
        // Pin endpoints exactly (lerp at t=0 / t=1 should already match,
        // but guard against floating-point drift).
        positions[0] = bounds.origin;
        positions[N - 1] = bounds.destination;

        let perpendiculars = compute_perpendiculars(&positions);
        let line_metres = haversine(positions[0], positions[N - 1]);
        let symmetric = [line_metres * 0.5; N];
        Self {
            positions,
            perpendiculars,
            perp_scale_pos: symmetric,
            perp_scale_neg: symmetric,
        }
    }

    /// Topology-aware baseline from a polyline (e.g. an A* sea path).
    /// Samples N waypoints uniformly by arc length and probes the
    /// signed-distance field for the perpendicular clearance on each
    /// side, so a path running close to one coast still wiggles freely
    /// on the open-sea side.
    pub fn from_polyline<LS: LandmassSource>(
        polyline: &[LatLon],
        bounds: &RouteBounds,
        landmass: &LS,
    ) -> Self {
        let positions = sample_uniform_arclength::<N>(polyline, bounds);
        let perpendiculars = compute_perpendiculars(&positions);
        let (perp_scale_pos, perp_scale_neg) =
            compute_perpendicular_clearance(&positions, &perpendiculars, landmass);
        Self {
            positions,
            perpendiculars,
            perp_scale_pos,
            perp_scale_neg,
        }
    }

    /// Sampler that prioritises land-free chords over uniform spacing.
    /// Use for benchmark routes (unperturbed `xy`) so "Bench land"
    /// reads 0 on a correctly-routed input. *Not* for PSO init seeds —
    /// tighter sampling hugs the coastline, and perpendicular kicks
    /// then push particles into land too often.
    pub fn from_polyline_land_respecting<LS: LandmassSource>(
        polyline: &[LatLon],
        bounds: &RouteBounds,
        landmass: &LS,
    ) -> Self {
        let positions = sample_land_respecting::<N, LS>(polyline, bounds, landmass);
        let perpendiculars = compute_perpendiculars(&positions);
        let (perp_scale_pos, perp_scale_neg) =
            compute_perpendicular_clearance(&positions, &perpendiculars, landmass);
        Self {
            positions,
            perpendiculars,
            perp_scale_pos,
            perp_scale_neg,
        }
    }
}

/// Greedy sampler: prefer breaking the worst crossing first, fall
/// back to longest-gap arc-midpoint splits. Tries every polyline
/// vertex in the gap (not just the midpoint) so a single well-placed
/// detour vertex can break a crossing that midpoint refinement
/// wouldn't reach within the N budget. Skip-set tracks segments where
/// no split improves things, so one un-breakable chord doesn't halt
/// the loop. Falls back to plain arc-length sampling if `N` can't be
/// reached. Endpoints are pinned to `bounds.origin` /
/// `bounds.destination` regardless of where A* snapped its endpoints.
fn sample_land_respecting<const N: usize, LS: LandmassSource>(
    polyline: &[LatLon],
    bounds: &RouteBounds,
    landmass: &LS,
) -> [LatLon; N] {
    let mut out = [LatLon::default(); N];
    out[0] = bounds.origin;
    out[N - 1] = bounds.destination;
    if polyline.len() < 2 || N < 3 {
        for (i, slot) in out.iter_mut().enumerate().skip(1).take(N.saturating_sub(2)) {
            let t = i as f64 / (N - 1) as f64;
            *slot = bounds.lerp_between_endpoints(t);
        }
        return out;
    }

    let cumulative = cumulative_arc_lengths(polyline);
    let total = *cumulative.last().expect("polyline has >= 2 vertices");
    if total <= 0.0 {
        return sample_uniform_arclength::<N>(polyline, bounds);
    }

    // Selected: (position, polyline_arc_length_from_polyline[0]).
    let mut selected: Vec<(LatLon, f64)> =
        vec![(polyline[0], 0.0), (polyline[polyline.len() - 1], total)];

    // Skip set keyed on the segment's arc-length endpoints (as f64
    // bits). `selected` is append-only so the key stays valid; stale
    // entries pointing at later-split segments are harmless.
    let mut skip: HashSet<(u64, u64)> = HashSet::new();
    let key_of = |a: f64, b: f64| (a.to_bits(), b.to_bits());

    let step = bounds.step_distance_max;

    while selected.len() < N {
        // Phase 1 work: break the worst remaining crossing.
        let mut worst_land = 0.0_f64;
        let mut worst_idx: Option<usize> = None;
        for i in 0..selected.len() - 1 {
            if skip.contains(&key_of(selected[i].1, selected[i + 1].1)) {
                continue;
            }
            let land = get_segment_land_metres(landmass, selected[i].0, selected[i + 1].0, step);
            if land > worst_land {
                worst_land = land;
                worst_idx = Some(i);
            }
        }
        if let Some(i) = worst_idx {
            let arc_a = selected[i].1;
            let arc_b = selected[i + 1].1;
            let pa = selected[i].0;
            let pb = selected[i + 1].0;
            if let Some((pos, arc, score)) =
                best_break_candidate(polyline, &cumulative, arc_a, arc_b, pa, pb, landmass, step)
                && score < worst_land
            {
                selected.insert(i + 1, (pos, arc));
                continue;
            }
            // Smart-break couldn't strictly improve on this segment.
            // Happens when the parent's great-circle traverses
            // fundamentally different geography from the polyline
            // detour: e.g. an around-continent route where the chord
            // cuts across the continent and any single detour vertex
            // still leaves one half on the continent's other side.
            // `max(l_left, l_right) < parent_land` is unattainable,
            // so smart-break alone would skip the segment and leave
            // the crossing in place forever.
            //
            // Fall back to inserting the polyline arc-midpoint. The
            // polyline is land-free by construction (it's an A* path
            // through sea cells), so repeated halving subdivides the
            // parent's polyline range until consecutive selections
            // span a single polyline edge — at which point the chord
            // tracks the sea route and lands no metres on land.
            let target = (arc_a + arc_b) * 0.5;
            if let Some((pos, arc)) = polyline_at_arc_length(polyline, &cumulative, target)
                && (arc - arc_a).abs() > 1e-9
                && (arc_b - arc).abs() > 1e-9
            {
                selected.insert(i + 1, (pos, arc));
                continue;
            }
            // Arc-midpoint coincides with one of the segment's
            // endpoints (degenerate very-short range): nothing
            // meaningful to insert.
            skip.insert(key_of(arc_a, arc_b));
            continue;
        }

        // Phase 2 work: fill the longest open-sea gap.
        let mut longest_gap = 0.0_f64;
        let mut longest_idx: Option<usize> = None;
        for i in 0..selected.len() - 1 {
            if skip.contains(&key_of(selected[i].1, selected[i + 1].1)) {
                continue;
            }
            let gap = selected[i + 1].1 - selected[i].1;
            if gap > longest_gap {
                longest_gap = gap;
                longest_idx = Some(i);
            }
        }
        let Some(i) = longest_idx else {
            break;
        };
        let arc_a = selected[i].1;
        let arc_b = selected[i + 1].1;
        let pa = selected[i].0;
        let pb = selected[i + 1].0;
        let target = (arc_a + arc_b) * 0.5;
        let Some((pos, arc)) = polyline_at_arc_length(polyline, &cumulative, target) else {
            skip.insert(key_of(arc_a, arc_b));
            continue;
        };
        if (arc - arc_a).abs() < 1e-9 || (arc_b - arc).abs() < 1e-9 {
            skip.insert(key_of(arc_a, arc_b));
            continue;
        }
        let l_left = get_segment_land_metres(landmass, pa, pos, step);
        let l_right = get_segment_land_metres(landmass, pos, pb, step);
        if l_left > 0.0 || l_right > 0.0 {
            // Splitting at this gap's polyline midpoint would create
            // a fresh crossing — happens when the polyline curves
            // through coastline that the parent chord didn't clip
            // (a previously-open-sea gap whose arc midpoint lands on
            // a polyline detour). Skip and look at the next gap.
            skip.insert(key_of(arc_a, arc_b));
            continue;
        }
        selected.insert(i + 1, (pos, arc));
    }

    if selected.len() < N {
        return sample_uniform_arclength::<N>(polyline, bounds);
    }

    for (i, slot) in out.iter_mut().enumerate().take(N - 1).skip(1) {
        *slot = selected[i].0;
    }
    out
}

/// Search for the split candidate that minimises `max(land_left,
/// land_right)` after insertion, considering every polyline vertex
/// strictly inside `(arc_a, arc_b)` plus the arc-length midpoint
/// interpolation. Returns `None` if no candidate falls inside the gap
/// (e.g. when the gap spans no polyline vertices and the arc midpoint
/// is at a duplicate position).
#[expect(
    clippy::too_many_arguments,
    reason = "candidate search needs full segment context (endpoints, arcs, polyline, landmass)"
)]
fn best_break_candidate<LS: LandmassSource>(
    polyline: &[LatLon],
    cumulative: &[f64],
    arc_a: f64,
    arc_b: f64,
    pa: LatLon,
    pb: LatLon,
    landmass: &LS,
    step: f64,
) -> Option<(LatLon, f64, f64)> {
    let mut best: Option<(LatLon, f64, f64)> = None;
    let consider = |pos: LatLon, arc: f64, best: &mut Option<(LatLon, f64, f64)>| {
        if (arc - arc_a).abs() < 1e-9 || (arc_b - arc).abs() < 1e-9 {
            return;
        }
        let l_left = get_segment_land_metres(landmass, pa, pos, step);
        let l_right = get_segment_land_metres(landmass, pos, pb, step);
        let score = l_left.max(l_right);
        match *best {
            Some((_, _, s)) if s <= score => {}
            _ => *best = Some((pos, arc, score)),
        }
    };
    for k in 0..polyline.len() {
        let v_arc = cumulative[k];
        if v_arc > arc_a && v_arc < arc_b {
            consider(polyline[k], v_arc, &mut best);
        }
    }
    let target = (arc_a + arc_b) * 0.5;
    if let Some((pos, arc)) = polyline_at_arc_length(polyline, cumulative, target) {
        consider(pos, arc, &mut best);
    }
    best
}

/// Cumulative haversine arc length from `polyline[0]` to each subsequent
/// vertex. `cumulative[k]` is the arc length from `polyline[0]` to
/// `polyline[k]`.
fn cumulative_arc_lengths(polyline: &[LatLon]) -> Vec<f64> {
    let mut cum = vec![0.0_f64; polyline.len()];
    for k in 1..polyline.len() {
        cum[k] = cum[k - 1] + haversine(polyline[k - 1], polyline[k]);
    }
    cum
}

/// Position (and exact arc length) on the polyline at the given
/// cumulative arc length. Linearly interpolates within the bracketing
/// segment for sub-vertex precision. Returns `None` only for empty
/// polylines.
fn polyline_at_arc_length(
    polyline: &[LatLon],
    cumulative: &[f64],
    target: f64,
) -> Option<(LatLon, f64)> {
    if polyline.is_empty() {
        return None;
    }
    let total = *cumulative.last()?;
    let target = target.clamp(0.0, total);
    // Find the segment k..k+1 containing `target`.
    let mut k = 0;
    while k + 1 < cumulative.len() && cumulative[k + 1] < target {
        k += 1;
    }
    let seg_start = cumulative[k];
    let seg_end = cumulative.get(k + 1).copied().unwrap_or(total);
    let span = (seg_end - seg_start).max(1e-12);
    let alpha = ((target - seg_start) / span).clamp(0.0, 1.0);
    let a = polyline[k];
    let b = polyline.get(k + 1).copied().unwrap_or(a);
    // Linear interpolation in (lon, lat) — acceptable here because the
    // polyline carries enough vertices that sub-vertex distances are
    // small enough for the arc-length-vs-great-circle error to be
    // negligible for init seeding.
    let pos = LatLon::new(
        a.lon * (1.0 - alpha) + b.lon * alpha,
        a.lat * (1.0 - alpha) + b.lat * alpha,
    );
    Some((pos, target))
}

/// Sample N points uniformly along the cumulative arc-length of a
/// polyline. The first and last samples are pinned to `bounds.origin`
/// and `bounds.destination` so init particles always start and end at
/// the user-specified endpoints regardless of polyline drift.
fn sample_uniform_arclength<const N: usize>(
    polyline: &[LatLon],
    bounds: &RouteBounds,
) -> [LatLon; N] {
    let mut out = [LatLon::default(); N];
    out[0] = bounds.origin;
    out[N - 1] = bounds.destination;
    if polyline.len() < 2 || N < 3 {
        // Fall back to lerp between the pinned endpoints.
        for (i, slot) in out.iter_mut().enumerate().skip(1).take(N.saturating_sub(2)) {
            let t = i as f64 / (N - 1) as f64;
            *slot = bounds.lerp_between_endpoints(t);
        }
        return out;
    }

    // Cumulative haversine distances: cumulative[k] = arc length from
    // polyline[0] to polyline[k].
    let mut cumulative = vec![0.0f64; polyline.len()];
    for k in 1..polyline.len() {
        cumulative[k] = cumulative[k - 1] + haversine(polyline[k - 1], polyline[k]);
    }
    let total = *cumulative.last().expect("polyline has >= 2 points");
    if total <= 0.0 {
        for slot in out.iter_mut().skip(1).take(N.saturating_sub(2)) {
            *slot = polyline[0];
        }
        return out;
    }

    // For interior i, walk the polyline to find the segment containing
    // arc length `target`, then linearly interpolate between its endpoints.
    let mut k = 0;
    for (i, slot) in out.iter_mut().enumerate().skip(1).take(N.saturating_sub(2)) {
        let target = (i as f64 / (N - 1) as f64) * total;
        while k + 1 < cumulative.len() && cumulative[k + 1] < target {
            k += 1;
        }
        let seg_start = cumulative[k];
        let seg_end = cumulative.get(k + 1).copied().unwrap_or(total);
        let span = (seg_end - seg_start).max(1e-12);
        let alpha = ((target - seg_start) / span).clamp(0.0, 1.0);
        let a = polyline[k];
        let b = polyline.get(k + 1).copied().unwrap_or(a);
        // Interpolate in (lon, lat) — for sub-cell distances this is
        // close enough to a great-circle interpolation that the small
        // error doesn't matter for init seeding.
        *slot = LatLon::new(
            a.lon * (1.0 - alpha) + b.lon * alpha,
            a.lat * (1.0 - alpha) + b.lat * alpha,
        );
    }
    out
}

/// Per-waypoint perturbation scales for the +`perpendiculars[i]` and
/// −`perpendiculars[i]` directions, derived by walking each side of
/// the polyline until the SDF crosses zero (or a route-length cap is
/// reached). Asymmetric so a polyline running 20 km off a coast can
/// still let particles wiggle hundreds of km on the open-sea side.
///
/// Probing strategy: an exponential coarse probe to find the first
/// land hit, then a short bisection to refine the crossover within ~1 %
/// of the cap. ~11 SDF lookups per direction per waypoint — cheap
/// enough that even N=60 init at startup costs <1 ms total.
fn compute_perpendicular_clearance<const N: usize, LS: LandmassSource>(
    positions: &[LatLon; N],
    perpendiculars: &[TangentMetres; N],
    landmass: &LS,
) -> ([f64; N], [f64; N]) {
    /// Min perturbation amplitude (m). Always allow at least this much
    /// so coast-hugging waypoints get *some* family-induced variation.
    const MIN_SCALE_M: f64 = 5_000.0;
    /// Cap as a fraction of the great-circle route length. Smaller than
    /// the straight-line baseline's 0.5 because the polyline already
    /// does the macro detour work — families just refine locally.
    const MAX_SCALE_FRACTION: f64 = 0.2;

    let line_metres = haversine(positions[0], positions[N - 1]).max(MIN_SCALE_M * 4.0);
    let max_scale = (line_metres * MAX_SCALE_FRACTION).max(MIN_SCALE_M * 2.0);
    let mut pos_scale = [0.0; N];
    let mut neg_scale = [0.0; N];
    for i in 0..N {
        let here = positions[i];
        let perp = perpendiculars[i];
        pos_scale[i] = probe_clearance(landmass, here, perp, max_scale).max(MIN_SCALE_M);
        neg_scale[i] = probe_clearance(landmass, here, -perp, max_scale).max(MIN_SCALE_M);
    }
    (pos_scale, neg_scale)
}

/// Walk from `pos` along `direction` (unit east-north tangent metres)
/// until the SDF crosses zero, returning the largest sea-side distance
/// found. Capped at `max_distance_m`. Implementation: exponential
/// probe to bracket the crossover, then 6-step bisection to refine.
fn probe_clearance<LS: LandmassSource>(
    landmass: &LS,
    pos: LatLon,
    direction: TangentMetres,
    max_distance_m: f64,
) -> f64 {
    // Exponential probe distances; capped to `max_distance_m` so we
    // never probe further than we'd ever use. The last entry is always
    // the cap so we don't miss the case "all sea up to the cap."
    let probes: [f64; 6] = {
        let mut p = [
            100_000.0,
            300_000.0,
            1_000_000.0,
            3_000_000.0,
            max_distance_m,
            max_distance_m,
        ];
        // Replace the next-to-last with `max_distance_m` if we'd
        // otherwise probe further than the cap.
        for slot in p.iter_mut().take(5) {
            if *slot > max_distance_m {
                *slot = max_distance_m;
            }
        }
        p
    };
    let mut last_good = 0.0;
    let mut first_bad: Option<f64> = None;
    for &d in &probes {
        if landmass.signed_distance_m(pos.offset_by(direction * d)) >= 0.0 {
            last_good = d;
        } else {
            first_bad = Some(d);
            break;
        }
    }
    let Some(bad) = first_bad else {
        return max_distance_m;
    };
    // Bisect [last_good, bad] for sub-cell accuracy on the crossover.
    let mut lo = last_good;
    let mut hi = bad;
    for _ in 0..6 {
        let mid = 0.5 * (lo + hi);
        if landmass.signed_distance_m(pos.offset_by(direction * mid)) >= 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

/// Local outward perpendiculars in the east-north tangent frame, one
/// per waypoint. Uses the central-difference tangent (prev → next) at
/// interior waypoints and one-sided differences at endpoints. Returns
/// the 90° CCW rotation of the unit tangent.
fn compute_perpendiculars<const N: usize>(positions: &[LatLon; N]) -> [TangentMetres; N] {
    let mut perps = [TangentMetres::zero(); N];
    for i in 0..N {
        let prev_i = if i == 0 { 0 } else { i - 1 };
        let next_i = if i + 1 >= N { N - 1 } else { i + 1 };
        let prev = positions[prev_i];
        let next = positions[next_i];
        let here = positions[i];
        // Tangent in tangent metres at `here`, computed from the
        // wrap-aware delta between the neighbours converted via the
        // east-north metric at `here`'s latitude. `LatLon::delta_to`
        // takes the short way across the antimeridian, so a polyline
        // segment from 179° to −179° gives a +2° east tangent rather
        // than a −358° west one.
        let tangent = delta_to_tangent_metres(here, prev.delta_to(next));
        let mag = tangent.norm();
        perps[i] = if mag > 1e-12 {
            TangentMetres::new(-tangent.north / mag, tangent.east / mag)
        } else {
            // Degenerate (consecutive duplicate positions) — pick a
            // sensible default so the family kicks aren't zeroed out.
            TangentMetres::new(0.0, 1.0)
        };
    }
    perps
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "tests rely on bit-exact comparisons of constant or stored f32/f64 values."
    )]
    use super::*;
    use crate::LandmassSourceDummy;
    use crate::spherical::LonLatBbox;

    /// `LandmassSource` with land in the slab `lon ∈ [lon_min, lon_max]`,
    /// sea everywhere else. Lets us reason about perpendicular clearance
    /// against a known geometry.
    struct CoastSlab {
        lon_min: f64,
        lon_max: f64,
    }

    impl LandmassSource for CoastSlab {
        fn signed_distance_m(&self, location: LatLon) -> f64 {
            let dx = (self.lon_min - location.lon).max(location.lon - self.lon_max);
            // Treat 1° ≈ 111 km for the SDF magnitude. Sign flips inside
            // the slab.
            dx * 111_000.0
        }
    }

    #[test]
    fn perpendiculars_handle_antimeridian_crossing() {
        // Polyline crossing the antimeridian eastward at the equator:
        // [178°, 180°, −178°]. The short-way tangent at the middle
        // waypoint is +2°/step due east, so the 90°-CCW perpendicular
        // is due north `(east=0, north=1)`. With a raw `next.lon −
        // prev.lon` delta the tangent comes out −356° (west) and the
        // perpendicular points south — this test pins the wrap fix.
        let positions: [LatLon; 3] = [
            LatLon::new(178.0, 0.0),
            LatLon::new(180.0, 0.0),
            LatLon::new(-178.0, 0.0),
        ];
        let perps = compute_perpendiculars(&positions);
        let mid = perps[1];
        assert!(
            mid.east.abs() < 1e-9 && (mid.north - 1.0).abs() < 1e-9,
            "expected ~(east=0, north=1) at antimeridian crossing, got {mid:?}",
        );
    }

    #[test]
    fn perpendicular_clearance_is_asymmetric_near_a_coast() {
        // Polyline running north–south at lon = 3 (1° east of a coast
        // that runs from lon -2 to 2). Tangent at the middle waypoint
        // is north, perpendicular is west (rotate 90° CCW). Long route
        // so the route-length cap is well above the coast-side clearance
        // and the asymmetry is visible.
        let coast = CoastSlab {
            lon_min: -2.0,
            lon_max: 2.0,
        };
        let positions: [LatLon; 3] = [
            LatLon::new(3.0, -45.0),
            LatLon::new(3.0, 0.0),
            LatLon::new(3.0, 45.0),
        ];
        let perpendiculars = compute_perpendiculars(&positions);
        let (pos_scale, neg_scale) =
            compute_perpendicular_clearance(&positions, &perpendiculars, &coast);
        // +perpendicular at index 1 should be west (toward coast). The
        // coast edge is 1° = ~111 km away; clearance should be in that
        // ballpark (within bisection tolerance, ~8 km on this scale).
        assert!(
            (pos_scale[1] - 111_000.0).abs() < 30_000.0,
            "expected ~111 km west clearance, got {} m",
            pos_scale[1],
        );
        // −perpendicular is east — open ocean for far longer than the
        // coast distance. Clearance should saturate at the route-length
        // cap (~0.2 × 10 000 km = 2000 km), much larger than coast-side.
        assert!(
            neg_scale[1] > pos_scale[1] * 5.0,
            "expected east clearance >> west, got pos={}, neg={}",
            pos_scale[1],
            neg_scale[1],
        );
    }

    /// Bounded land patch — `lon ∈ [lon_min, lon_max]`, `lat ∈ [lat_min,
    /// lat_max]`. Mirrors `multi_baseline_init.rs`'s `SimulatedLandmass`
    /// SDF formula so the test exercises the sampler against a real
    /// rectangular obstacle the polyline can detour around.
    struct LandPatch {
        lon_min: f64,
        lon_max: f64,
        lat_min: f64,
        lat_max: f64,
    }

    impl LandmassSource for LandPatch {
        fn signed_distance_m(&self, location: LatLon) -> f64 {
            let dx = (self.lon_min - location.lon).max(location.lon - self.lon_max);
            let dy = (self.lat_min - location.lat).max(location.lat - self.lat_max);
            let deg_to_m = 111_000.0;
            let outside = dx.max(0.0).hypot(dy.max(0.0)) * deg_to_m;
            let inside = dx.max(dy).min(0.0) * deg_to_m;
            if outside > 0.0 { outside } else { inside }
        }
    }

    #[test]
    fn land_respecting_sampler_produces_land_free_chords() {
        // 4°-wide × 18°-tall land patch centred on the equator. A
        // polyline detours over the top: cell-centred chain of 5
        // vertices, all on the sea side of the patch.
        let land = LandPatch {
            lon_min: -2.0,
            lon_max: 2.0,
            lat_min: -9.0,
            lat_max: 9.0,
        };
        let polyline = vec![
            LatLon::new(-15.0, 0.0),
            LatLon::new(-3.0, 9.5),
            LatLon::new(0.0, 10.0),
            LatLon::new(3.0, 9.5),
            LatLon::new(15.0, 0.0),
        ];
        let bounds = RouteBounds::new(
            (-15.0, 0.0),
            (15.0, 0.0),
            LonLatBbox::new(-20.0, 20.0, -12.0, 12.0),
        );
        let positions = sample_land_respecting::<8, _>(&polyline, &bounds, &land);

        // Every consecutive-pair chord must be land-free. With N=8 and a
        // 5-vertex polyline, the uniform sampler's central chord cuts
        // straight across the patch; the new sampler breaks that crossing
        // by inserting polyline midpoints until each chord stays in sea.
        let step = bounds.step_distance_max;
        for w in positions.windows(2) {
            let land_m = get_segment_land_metres(&land, w[0], w[1], step);
            assert_eq!(land_m, 0.0, "chord {:?} -> {:?} crossed land", w[0], w[1]);
        }

        // Sanity: endpoints pinned to bounds origin / destination.
        assert_eq!(positions[0], LatLon::new(-15.0, 0.0));
        assert_eq!(positions[7], LatLon::new(15.0, 0.0));
    }

    #[test]
    fn land_respecting_sampler_falls_back_for_open_ocean() {
        // No land anywhere — sampler should still produce N waypoints,
        // and the result should be a valid path (consecutive haversines
        // monotonically increase along the route arc).
        let dummy = LandmassSourceDummy;
        let polyline = vec![
            LatLon::new(0.0, 0.0),
            LatLon::new(50.0, 0.0),
            LatLon::new(100.0, 0.0),
        ];
        let bounds = RouteBounds::new(
            (0.0, 0.0),
            (100.0, 0.0),
            LonLatBbox::new(-10.0, 110.0, -5.0, 5.0),
        );
        let positions = sample_land_respecting::<6, _>(&polyline, &bounds, &dummy);
        assert_eq!(positions[0], LatLon::new(0.0, 0.0));
        assert_eq!(positions[5], LatLon::new(100.0, 0.0));
        // Interior waypoints should march monotonically east.
        for i in 1..6 {
            assert!(
                positions[i].lon >= positions[i - 1].lon,
                "waypoint {i} ({:?}) is west of {} ({:?})",
                positions[i],
                i - 1,
                positions[i - 1],
            );
        }
    }

    #[test]
    fn perpendicular_clearance_caps_at_max_distance_in_open_ocean() {
        // No land anywhere — both directions saturate at the cap.
        let dummy = LandmassSourceDummy;
        let positions: [LatLon; 3] = [
            LatLon::new(0.0, 0.0),
            LatLon::new(50.0, 0.0),
            LatLon::new(100.0, 0.0),
        ];
        let perpendiculars = compute_perpendiculars(&positions);
        let (pos_scale, neg_scale) =
            compute_perpendicular_clearance(&positions, &perpendiculars, &dummy);
        assert_eq!(pos_scale[1], neg_scale[1], "open ocean should be symmetric");
        // Cap = 0.2 × line_metres; line_metres ≈ haversine(100°, 0°) at
        // the equator ≈ 11 130 km, so cap ≈ 2226 km. Just check it's
        // big and matches both sides.
        assert!(pos_scale[1] > 1_000_000.0);
    }
}
