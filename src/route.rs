// Re-exported (not `use`d) so consumers can name them as
// `bywind::route::{Evolution, Path}`. Not at crate root — `Path` would
// collide with `std::path::Path`.
pub use swarmkit::Evolution;
pub use swarmkit_sailing::Path;

/// Waypoint counts the sailing search can dispatch over. Fixed at
/// compile time — each variant becomes a const-generic `Path<N>`.
///
/// The count is the total number of waypoints (endpoints included).
/// Higher counts give the PSO more degrees of freedom to bend the route
/// around weather and landmass at quadratic cost in fitness evaluation.
#[derive(serde::Deserialize, serde::Serialize, PartialEq, Eq, Clone, Copy, Default, Debug)]
pub enum WaypointCount {
    /// 5-waypoint path.
    N5,
    /// 8-waypoint path.
    N8,
    /// 10-waypoint path.
    N10,
    /// 15-waypoint path.
    N15,
    /// 20-waypoint path.
    N20,
    /// 30-waypoint path. Default — works well for transatlantic / transpacific routes.
    #[default]
    N30,
    /// 40-waypoint path.
    N40,
    /// 50-waypoint path.
    N50,
    /// 60-waypoint path.
    N60,
}

impl WaypointCount {
    /// Every supported variant in ascending order. Iteration order
    /// matches the GUI's dropdown.
    pub const ALL: [Self; 9] = [
        Self::N5,
        Self::N8,
        Self::N10,
        Self::N15,
        Self::N20,
        Self::N30,
        Self::N40,
        Self::N50,
        Self::N60,
    ];

    /// The literal waypoint count this variant represents.
    pub fn as_usize(self) -> usize {
        match self {
            Self::N5 => 5,
            Self::N8 => 8,
            Self::N10 => 10,
            Self::N15 => 15,
            Self::N20 => 20,
            Self::N30 => 30,
            Self::N40 => 40,
            Self::N50 => 50,
            Self::N60 => 60,
        }
    }

    /// Inverse of [`Self::as_usize`]. `None` if `n` is not one of the
    /// supported compile-time variants.
    pub fn from_usize(n: usize) -> Option<Self> {
        match n {
            5 => Some(Self::N5),
            8 => Some(Self::N8),
            10 => Some(Self::N10),
            15 => Some(Self::N15),
            20 => Some(Self::N20),
            30 => Some(Self::N30),
            40 => Some(Self::N40),
            50 => Some(Self::N50),
            60 => Some(Self::N60),
            _ => None,
        }
    }
}

/// Type-erased `Evolution<Path<N>>`; lets the UI app struct hold one
/// without going generic itself.
pub enum RouteEvolution {
    N5(Evolution<Path<5>>),
    N8(Evolution<Path<8>>),
    N10(Evolution<Path<10>>),
    N15(Evolution<Path<15>>),
    N20(Evolution<Path<20>>),
    N30(Evolution<Path<30>>),
    N40(Evolution<Path<40>>),
    N50(Evolution<Path<50>>),
    N60(Evolution<Path<60>>),
}

/// Dispatch on `RouteEvolution`, binding the inner
/// `Evolution<Path<N>>` per variant. Body monomorphises via type
/// inference.
#[macro_export]
macro_rules! route_evolution_match {
    ($expr:expr, |$evo:ident| $body:expr) => {
        match $expr {
            $crate::route::RouteEvolution::N5($evo) => $body,
            $crate::route::RouteEvolution::N8($evo) => $body,
            $crate::route::RouteEvolution::N10($evo) => $body,
            $crate::route::RouteEvolution::N15($evo) => $body,
            $crate::route::RouteEvolution::N20($evo) => $body,
            $crate::route::RouteEvolution::N30($evo) => $body,
            $crate::route::RouteEvolution::N40($evo) => $body,
            $crate::route::RouteEvolution::N50($evo) => $body,
            $crate::route::RouteEvolution::N60($evo) => $body,
        }
    };
}
pub use route_evolution_match;

/// Dispatch on `WaypointCount`, binding `$n` to the const N and
/// `$wrap` to the matching `RouteEvolution` constructor.
#[macro_export]
macro_rules! waypoint_match {
    ($wc:expr, $n:ident, $wrap:ident, $body:expr) => {
        match $wc {
            $crate::route::WaypointCount::N5 => {
                const $n: usize = 5;
                let $wrap = $crate::route::RouteEvolution::N5;
                $body
            }
            $crate::route::WaypointCount::N8 => {
                const $n: usize = 8;
                let $wrap = $crate::route::RouteEvolution::N8;
                $body
            }
            $crate::route::WaypointCount::N10 => {
                const $n: usize = 10;
                let $wrap = $crate::route::RouteEvolution::N10;
                $body
            }
            $crate::route::WaypointCount::N15 => {
                const $n: usize = 15;
                let $wrap = $crate::route::RouteEvolution::N15;
                $body
            }
            $crate::route::WaypointCount::N20 => {
                const $n: usize = 20;
                let $wrap = $crate::route::RouteEvolution::N20;
                $body
            }
            $crate::route::WaypointCount::N30 => {
                const $n: usize = 30;
                let $wrap = $crate::route::RouteEvolution::N30;
                $body
            }
            $crate::route::WaypointCount::N40 => {
                const $n: usize = 40;
                let $wrap = $crate::route::RouteEvolution::N40;
                $body
            }
            $crate::route::WaypointCount::N50 => {
                const $n: usize = 50;
                let $wrap = $crate::route::RouteEvolution::N50;
                $body
            }
            $crate::route::WaypointCount::N60 => {
                const $n: usize = 60;
                let $wrap = $crate::route::RouteEvolution::N60;
                $body
            }
        }
    };
}
pub use waypoint_match;

/// A*-shortest sea path + time-PSO baseline.
///
/// Computed alongside the main search so the user can see whether
/// full PSO beats "shortest path, optimal throttle." `None` when A*
/// fails (landlocked endpoints, no sea route, etc.).
#[derive(Clone, Debug)]
pub struct BenchmarkRoute {
    /// `(lon, lat)` per waypoint; length matches the search's N.
    pub waypoints: Vec<(f64, f64)>,
    /// Total travel time across all segments (seconds).
    pub total_time: f64,
    /// Total fuel consumed (kg).
    pub total_fuel: f64,
    /// Total over-land distance (m). A correctly-routed benchmark
    /// reads zero; non-zero flags a routing failure.
    pub total_land_metres: f64,
    /// Same `SailboatFitCalc` as the main search — same weights, same
    /// penalty, so the comparison is apples-to-apples.
    pub fitness: f64,
}

/// Non-generic borrow of a gbest particle's path. Slice lengths
/// all equal the path's `N`.
pub struct GbestView<'a> {
    /// Longitude (degrees) per waypoint.
    pub xs: &'a [f64],
    /// Latitude (degrees) per waypoint.
    pub ys: &'a [f64],
    /// Cumulative arrival time at each waypoint (seconds since the
    /// search's `departure_time`).
    pub ts: &'a [f64],
    /// Fitness at the iteration this view was taken from. Higher is
    /// better (`fitness = -(weighted cost)`).
    pub best_fit: f64,
}

/// Mutable [`GbestView`]; `best_fit` omitted — editing the path leaves
/// the search-time fitness stale.
pub struct GbestViewMut<'a> {
    /// Longitude (degrees) per waypoint.
    pub xs: &'a mut [f64],
    /// Latitude (degrees) per waypoint.
    pub ys: &'a mut [f64],
    /// Cumulative arrival time at each waypoint (seconds).
    pub ts: &'a mut [f64],
}

impl RouteEvolution {
    pub fn iter_count(&self) -> usize {
        route_evolution_match!(self, |e| e.frames().len())
    }

    /// Read-only view of the global-best particle at iteration `iter`. Clamps
    /// `iter` to the last valid index. Returns `None` when the iteration slot
    /// is empty.
    pub fn gbest_at(&self, iter: usize) -> Option<GbestView<'_>> {
        route_evolution_match!(self, |evo| {
            let frames = evo.frames();
            let iter_idx = iter.min(frames.len().saturating_sub(1));
            let particles = frames.get(iter_idx)?;
            let best = particles.iter().max_by(|a, b| {
                a.best_fit
                    .partial_cmp(&b.best_fit)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })?;
            Some(GbestView {
                xs: &best.best_pos.xy.0.0[..],
                ys: &best.best_pos.xy.1.0[..],
                ts: &best.best_pos.t.0.0[..],
                best_fit: best.best_fit,
            })
        })
    }

    /// Mutable view of the global-best particle at iteration `iter`.
    pub fn gbest_at_mut(&mut self, iter: usize) -> Option<GbestViewMut<'_>> {
        route_evolution_match!(self, |evo| {
            let frames = evo.frames_mut();
            let iter_idx = iter.min(frames.len().saturating_sub(1));
            let particles = frames.get_mut(iter_idx)?;
            let best_idx = particles
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| {
                    a.best_fit
                        .partial_cmp(&b.best_fit)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(|(i, _)| i)?;
            let p = particles.get_mut(best_idx)?;
            Some(GbestViewMut {
                xs: &mut p.best_pos.xy.0.0[..],
                ys: &mut p.best_pos.xy.1.0[..],
                ts: &mut p.best_pos.t.0.0[..],
            })
        })
    }

    /// Translate the gbest path's xy at `idx` by `(dx, dy)` (in route-search
    /// coords). `best_fit` is intentionally left stale — callers that need a
    /// fresh fitness should rerun the search or the time-reopt.
    pub fn mutate_waypoint(&mut self, iter: usize, idx: usize, dx: f64, dy: f64) {
        let Some(g) = self.gbest_at_mut(iter) else {
            return;
        };
        if let Some(x) = g.xs.get_mut(idx) {
            *x += dx;
        }
        if let Some(y) = g.ys.get_mut(idx) {
            *y += dy;
        }
    }

    /// Splice `new_times` into the gbest path's `t` at iteration `iter`.
    /// No-op if the lengths don't match (e.g. waypoint count changed since
    /// the reopt was dispatched).
    pub fn apply_reopt_times(&mut self, iter: usize, new_times: &[f64]) {
        let Some(g) = self.gbest_at_mut(iter) else {
            return;
        };
        if g.ts.len() != new_times.len() {
            return;
        }
        g.ts.copy_from_slice(new_times);
    }
}

/// Compiles to a no-op in release. Catching NaN escaping the search worker is
/// useful while debugging the upstream PSO, but it's per-particle-per-iteration
/// overhead that nothing in release does anything useful with — a NaN here
/// just panics the worker thread.
pub(crate) fn debug_assert_path_no_nans<const N: usize>(path: &Path<N>, context: &str) {
    if !cfg!(debug_assertions) {
        return;
    }
    for (i, &x) in path.xy.0.0.iter().enumerate() {
        assert!(!x.is_nan(), "NaN in {context}: x[{i}]");
    }
    for (i, &y) in path.xy.1.0.iter().enumerate() {
        assert!(!y.is_nan(), "NaN in {context}: y[{i}]");
    }
    for (i, &t) in path.t.0.0.iter().enumerate() {
        assert!(!t.is_nan(), "NaN in {context}: t[{i}]");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn waypoint_count_round_trips_through_usize() {
        for &wc in &WaypointCount::ALL {
            let n = wc.as_usize();
            assert_eq!(WaypointCount::from_usize(n), Some(wc));
        }
    }

    #[test]
    fn waypoint_count_rejects_unsupported_n() {
        for unsupported in [0, 1, 7, 11, 25, 45, 100] {
            assert_eq!(
                WaypointCount::from_usize(unsupported),
                None,
                "n={unsupported}",
            );
        }
    }

    #[test]
    fn waypoint_count_default_is_n30() {
        assert_eq!(WaypointCount::default().as_usize(), 30);
    }

    #[test]
    fn waypoint_count_all_is_sorted_and_complete() {
        let sizes: Vec<usize> = WaypointCount::ALL.iter().map(|wc| wc.as_usize()).collect();
        assert_eq!(sizes, vec![5, 8, 10, 15, 20, 30, 40, 50, 60]);
    }
}
