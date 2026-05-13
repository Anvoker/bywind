use crate::range_cache::SegmentRangeTables;
use crate::spherical::{LatLon, Segment, destination_point, haversine, initial_bearing};
use crate::units::Path;
use crate::{LandmassSource, Sailboat, WindSource};

/// Returns `(t_min, t_max)` for a segment, ordered. In time-variant wind the
/// full-motor and sail-only candidates can sample wind at different timestamps
/// along the segment and produce inverted times, so we order defensively here
/// — callers should treat the result as a range, not as `(best, worst)`.
pub(crate) fn get_travel_time_range<SB: Sailboat, WS: WindSource>(
    sailboat: &SB,
    wind_source: &WS,
    segment: Segment,
) -> (f64, f64) {
    let a = sailboat.get_travel_time(wind_source, segment, 1.0);
    let (_, b) = get_worst_travel_time(sailboat, wind_source, segment);
    (a.min(b), a.max(b))
}

/// Returns `(worst_mcr, worst_time)` for a segment, using `mcr_01 = 0.0` and
/// falling back to `0.1` if that produces infinite travel time (e.g. zero
/// wind with no motor). When even `0.1 MCR` yields infinite time —
/// typically a segment hugging the pole-latitude clamp where bearing math
/// degenerates rather than wind / motor power being the issue — returns
/// `(0.1, f64::INFINITY)` so the caller's fitness path naturally penalises
/// the candidate (`fitness = -inf`) and the PSO moves away from it instead
/// of panicking the worker thread.
pub(crate) fn get_worst_travel_time<SB: Sailboat, WS: WindSource>(
    sailboat: &SB,
    wind_source: &WS,
    segment: Segment,
) -> (f64, f64) {
    let t = sailboat.get_travel_time(wind_source, segment, 0.0);
    if t.is_infinite() {
        let t = sailboat.get_travel_time(wind_source, segment, 0.1);
        // `t` may still be infinite for pole-locked or otherwise
        // numerically degenerate segments. Surface it; the upstream
        // fitness calc treats infinite time as the worst possible result
        // and the PSO swarm moves away from this region of search space.
        (0.1, t)
    } else {
        (0.0, t)
    }
}

/// Inverts `get_travel_time` to find the constant `mcr_01 ∈ [worst_mcr, 1.0]` whose
/// wind-integrated travel time across the segment matches `delta_time`. Returns 0.0
/// for a degenerate (zero-distance) segment.
pub(crate) fn solve_mcr_01<SB: Sailboat, WS: WindSource>(
    sailboat: &SB,
    wind_source: &WS,
    segment: Segment,
    arrival_time: f64,
) -> f64 {
    if haversine(segment.origin, segment.destination) <= 0.0 {
        return 0.0;
    }
    let delta_time = arrival_time - segment.origin_time;

    let best_time = sailboat.get_travel_time(wind_source, segment, 1.0);
    let (worst_mcr, worst_time) = get_worst_travel_time(sailboat, wind_source, segment);

    if delta_time <= best_time {
        return 1.0;
    }
    if delta_time >= worst_time {
        return worst_mcr;
    }

    // Bisect for the mcr_01 whose wind-integrated travel time matches delta_time.
    // `get_travel_time` is monotonically decreasing in mcr_01 (more motor → faster
    // → shorter time), so bisection is safe.
    // 12 iterations → ~2.4e-4 precision in mcr_01 (negligible fuel error).
    const MAX_ITER: usize = 12;
    let mut lo = worst_mcr;
    let mut hi = 1.0;
    for _ in 0..MAX_ITER {
        let mid = 0.5 * (lo + hi);
        let t = sailboat.get_travel_time(wind_source, segment, mid);
        if t > delta_time {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// Returns `(mcr_01, fuel)` for a single segment, integrating wind along the segment.
pub(crate) fn get_segment_metrics<SB: Sailboat, WS: WindSource>(
    sailboat: &SB,
    wind_source: &WS,
    segment: Segment,
    arrival_time: f64,
) -> (f64, f64) {
    let mcr_01 = solve_mcr_01(sailboat, wind_source, segment, arrival_time);
    let fuel = sailboat.get_fuel_consumed(mcr_01, arrival_time - segment.origin_time);
    (mcr_01, fuel)
}

/// Cached variant of `solve_mcr_01` backed by a 2D (`dep_time` × mcr) travel-time
/// grid. The 12-iter bisection becomes a piecewise-linear lookup, so this path
/// performs **zero** wind integrations per call.
///
/// Error comes entirely from linear interpolation on the tabulated curves —
/// see `SegmentRangeTables::query_mcr_for_delta_time` and `K_MCR` for the
/// tolerance.
pub(crate) fn solve_mcr_01_cached<const N: usize>(
    tables: &SegmentRangeTables<N>,
    seg_i: usize,
    origin: LatLon,
    destination: LatLon,
    departure_time: f64,
    arrival_time: f64,
) -> f64 {
    if haversine(origin, destination) <= 0.0 {
        return 0.0;
    }
    tables.query_mcr_for_delta_time(seg_i, departure_time, arrival_time - departure_time)
}

/// Total ground distance (m) over land along the great-circle from
/// `origin` to `destination`.
///
/// Walks the route in substeps of `step_distance_max`, sampling the
/// landmass source at each substep midpoint and accumulating only the
/// portions that lie inside land. Mirrors the substep cadence of
/// [`crate::Sailboat::get_travel_time`] so the two passes cover the
/// same midpoints.
///
/// Returns `f64::INFINITY` if the segment crosses a pole (bearing
/// undefined), matching the boat integrator's failure mode for the same
/// case so the fitness penalty stays consistent with the travel-time
/// term.
pub fn get_segment_land_metres<LS: LandmassSource>(
    landmass: &LS,
    origin: LatLon,
    destination: LatLon,
    step_distance_max: f64,
) -> f64 {
    let total_distance = haversine(origin, destination);
    if total_distance <= 0.0 {
        return 0.0;
    }
    let step_count = (total_distance / step_distance_max).ceil().max(1.0) as usize;
    let step_distance = total_distance / step_count as f64;
    let mut position = origin;
    let mut land_metres = 0.0;
    for _ in 0..step_count {
        let Some(bearing) = initial_bearing(position, destination) else {
            return f64::INFINITY;
        };
        let Some(mid) = destination_point(position, step_distance * 0.5, bearing) else {
            return f64::INFINITY;
        };
        if landmass.is_land(mid) {
            land_metres += step_distance;
        }
        let Some(next) = destination_point(position, step_distance, bearing) else {
            return f64::INFINITY;
        };
        position = next;
    }
    land_metres
}

pub(crate) fn get_segment_metrics_cached<SB: Sailboat, const N: usize>(
    sailboat: &SB,
    tables: &SegmentRangeTables<N>,
    seg_i: usize,
    origin: LatLon,
    destination: LatLon,
    departure_time: f64,
    arrival_time: f64,
) -> (f64, f64) {
    let mcr_01 = solve_mcr_01_cached(
        tables,
        seg_i,
        origin,
        destination,
        departure_time,
        arrival_time,
    );
    let fuel = sailboat.get_fuel_consumed(mcr_01, arrival_time - departure_time);
    (mcr_01, fuel)
}

/// Returns `(mcr_01, fuel, time)` for each segment of `path`, in order.
pub fn get_segment_fuel_and_time<const N: usize, SB: Sailboat, WS: WindSource>(
    sailboat: &SB,
    wind_source: &WS,
    path: Path<N>,
    departure_time: f64,
    step_distance_max: f64,
) -> Vec<(f64, f64, f64)> {
    path.iter_with_running_clock(departure_time)
        .map(|seg| {
            let segment = Segment {
                origin: seg.origin,
                destination: seg.destination,
                origin_time: seg.t_depart,
                step_distance_max,
            };
            let (mcr_01, fuel) = get_segment_metrics(sailboat, wind_source, segment, seg.t_arrive);
            (mcr_01, fuel, seg.segment_time)
        })
        .collect()
}
