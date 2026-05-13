//! Per-segment metrics derived from a search-result `Path<N>` against a
//! baked wind field. These feed the GUI's per-segment labels and the
//! summary-panel totals; nothing here is rendering-specific.

use swarmkit_sailing::spherical::haversine;
use swarmkit_sailing::{Boat, Path, get_segment_fuel_and_time, get_segment_land_metres};

use crate::landmass::landmass_grid;
use crate::route::RouteEvolution;
use crate::route_evolution_match;
use crate::wind_map::BakedWindMap;

/// Conversion from internal speed (m/s) to km/h for display.
const SPEED_INTERNAL_TO_KMH: f64 = 3.6;

/// Per-segment fitness breakdown for one path. Mirrors the tuple returned by
/// `get_segment_fuel_and_time`, plus a derived `speed_kmh` and the
/// per-segment over-land distance from `get_segment_land_metres`.
#[derive(Clone, Copy, Debug)]
pub struct SegmentMetrics {
    /// Engine load fraction in `[0, 1]`. Drives the per-segment colour ramp.
    pub mcr_01: f64,
    /// Fuel burned over the segment, kilograms.
    pub fuel: f64,
    /// Travel time over the segment, seconds.
    pub time: f64,
    /// Average ground speed over the segment, km/h. `0` when `time` is zero.
    pub speed_kmh: f64,
    /// Distance the segment spends over land, in metres. A correctly-routed
    /// path has this at zero; non-zero values flag a route that's cutting
    /// across landmass and indicate the search escaped the land penalty.
    pub land_metres: f64,
}

/// Per-segment metrics for the gbest particle at iteration `iter`.
///
/// Erases the const-generic `N` of the underlying `Path<N>` via
/// `route_evolution_match!` so callers (CLI, GUI) don't have to repeat
/// the dispatch. `boat` and `baked` are the ones the search ran with;
/// passing different values here is silently allowed but yields
/// physically inconsistent metrics.
///
/// Returns `None` when iteration `iter` has no particles (e.g. the search
/// produced zero iterations).
pub fn gbest_segment_metrics(
    re: &RouteEvolution,
    iter: usize,
    boat: &Boat,
    baked: &BakedWindMap,
    step_distance_max: f64,
) -> Option<Vec<SegmentMetrics>> {
    route_evolution_match!(re, |evo| {
        let frames = evo.frames();
        let iter_idx = iter.min(frames.len().saturating_sub(1));
        let particles = frames.get(iter_idx)?;
        let best = particles.iter().max_by(|a, b| {
            a.best_fit
                .partial_cmp(&b.best_fit)
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        Some(compute_segment_metrics(
            boat,
            baked,
            best.best_pos,
            step_distance_max,
        ))
    })
}

/// Compute per-segment metrics for `path`.
///
/// Wraps `get_segment_fuel_and_time` and `get_segment_land_metres` and
/// folds in the speed-from-distance/time derivation that callers would
/// otherwise duplicate. Uses the cached `landmass_grid()` for the
/// land-distance probe so callers don't have to thread the landmass
/// through.
pub fn compute_segment_metrics<const N: usize>(
    boat: &Boat,
    bwm: &BakedWindMap,
    path: Path<N>,
    step_distance_max: f64,
) -> Vec<SegmentMetrics> {
    let raw = get_segment_fuel_and_time(boat, bwm, path, 0.0, step_distance_max);
    let landmass = landmass_grid();
    raw.into_iter()
        .enumerate()
        .map(|(i, (mcr_01, fuel, time))| {
            let a = path.lat_lon(i);
            let b = path.lat_lon(i + 1);
            // `a` and `b` are `LatLon`. We can't use raw Cartesian distance
            // on degrees of mixed lon/lat, so use the great-circle ground
            // distance (metres) the rest of the codebase already uses.
            let distance_m = haversine(a, b);
            let speed_kmh = if time > 0.0 {
                (distance_m / time) * SPEED_INTERNAL_TO_KMH
            } else {
                0.0
            };
            let land_metres = get_segment_land_metres(landmass, a, b, step_distance_max);
            SegmentMetrics {
                mcr_01,
                fuel,
                time,
                speed_kmh,
                land_metres,
            }
        })
        .collect()
}
