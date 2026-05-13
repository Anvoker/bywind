//! Search-result state types persisted on `BywindApp`. The thread-spawning
//! adapters that drive `bywind::run_search_blocking` /
//! `run_time_reopt_blocking` live in `app.rs` next to the `BywindApp` they
//! mutate; this module owns only the value-side types those adapters
//! produce.

use bywind::{BakedWindMap, BenchmarkRoute, RouteEvolution, SegmentMetrics};
use swarmkit_sailing::{Boat, RouteBounds};

/// Message sent from the time-reoptimization worker thread back to the UI on
/// drag-release of a Waypoint Edit move. `iteration` is the iteration index the
/// reopt was launched against; `new_times` is the optimized `t` array (length
/// equals the path's N — checked at apply time so a stale message from a prior
/// `waypoint_count` is dropped instead of corrupting the path).
pub(crate) struct ReoptMsg {
    pub(crate) iteration: usize,
    pub(crate) new_times: Vec<f64>,
}

/// Outputs of a completed sailing search, plus the iteration the user is
/// currently scrubbed to. `iteration` persists across sessions (so the user
/// returns to the same view); the rest are transient (cleared on any
/// non-trivial state change like a new search or a fresh load).
#[derive(serde::Deserialize, serde::Serialize, Default)]
#[serde(default)]
pub(crate) struct SearchOutputs {
    pub(crate) iteration: usize,

    #[serde(skip)]
    pub(crate) route_evolution: Option<RouteEvolution>,

    /// Spatial bounds (origin, destination, search area) of the route problem.
    #[serde(skip)]
    pub(crate) route_bounds: Option<RouteBounds>,

    /// Baked wind field used by the last completed search, kept around so per-frame
    /// stats display doesn't have to re-bake on every repaint.
    #[serde(skip)]
    pub(crate) baked_wind_map: Option<BakedWindMap>,

    /// Boat used by the last completed search; reused for per-frame segment stats so
    /// display matches the boat the search actually optimized against.
    #[serde(skip)]
    pub(crate) boat: Option<Boat>,

    /// Per-segment metrics for the best path at the currently displayed iteration.
    #[serde(skip)]
    pub(crate) segment_stats: Option<Vec<SegmentMetrics>>,

    /// Best particle fitness at the currently displayed iteration.
    #[serde(skip)]
    pub(crate) best_fitness: Option<f64>,

    /// A*-shortest-path benchmark route from the last completed search:
    /// the unbiased sea path with PSO over the time dimension only,
    /// scored against the same fit calc as the main result. `None` when
    /// the search hasn't run yet or when A* couldn't find a sea route
    /// for the requested origin / destination.
    #[serde(skip)]
    pub(crate) benchmark: Option<BenchmarkRoute>,

    /// Wall-clock duration of the last completed wind-map bake (the
    /// `BakedWindMap::from_timed_map` work that runs on the search worker
    /// before the PSO loop starts). Time-only reopts triggered by Waypoint
    /// Edit drag-release deliberately don't update this — they reuse the
    /// existing baked map, and the displayed value reflects the most recent
    /// full search.
    #[serde(skip)]
    pub(crate) bake_duration: Option<std::time::Duration>,

    /// Wall-clock duration of the last completed full search. Time-only
    /// reopts deliberately don't update this; their cost isn't currently
    /// surfaced in the Summary panel.
    #[serde(skip)]
    pub(crate) search_duration: Option<std::time::Duration>,

    /// Seed actually fed into the PSO on the most recent Run Search.
    /// Populated whether or not the user typed a seed: when
    /// `SearchConfig::seed` is `None` we resolve it to a fresh OS-entropy
    /// `u64` here before spawning the worker, so the Advanced Params UI
    /// can show what was used after the fact. `None` until the first
    /// search starts; not persisted (session-local state).
    #[serde(skip)]
    pub(crate) last_search_seed: Option<u64>,
}
