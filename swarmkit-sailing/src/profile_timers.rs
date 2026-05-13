//! Sub-stage `Instant::now` counters around the sailing-search hot paths.
//!
//! Compiled behind `feature = "profile-timers"`; non-feature builds get
//! exactly nothing (no atomic traffic, no extra fields, no extra calls).
//!
//! Usage from inside the sailing crate:
//!
//! ```ignore
//! #[cfg(feature = "profile-timers")]
//! let __t = std::time::Instant::now();
//!
//! // ... hot-path work ...
//!
//! #[cfg(feature = "profile-timers")]
//! crate::profile_timers::OUTER_FIT.record(__t.elapsed().as_nanos() as u64);
//! ```
//!
//! At the start of [`crate::search`] the module's [`reset_all`] is called,
//! and at the end [`dump_to_stderr`] prints a one-line-per-stage
//! breakdown. Combined with the CLI's reported `search_dur` this gives a
//! coarse "where's the time?" picture without a sampling profiler.

#[cfg(feature = "profile-timers")]
use std::sync::atomic::{AtomicU64, Ordering};

#[cfg(feature = "profile-timers")]
pub struct StageTimer {
    nanos: AtomicU64,
    calls: AtomicU64,
    name: &'static str,
}

#[cfg(feature = "profile-timers")]
impl StageTimer {
    pub const fn new(name: &'static str) -> Self {
        Self {
            nanos: AtomicU64::new(0),
            calls: AtomicU64::new(0),
            name,
        }
    }

    /// Add `elapsed_nanos` to this stage's accumulator. Atomic relaxed
    /// adds — fine for accounting, no cross-thread ordering concerns.
    pub fn record(&self, elapsed_nanos: u64) {
        self.nanos.fetch_add(elapsed_nanos, Ordering::Relaxed);
        self.calls.fetch_add(1, Ordering::Relaxed);
    }

    pub fn snapshot(&self) -> (u64, u64) {
        (
            self.nanos.load(Ordering::Relaxed),
            self.calls.load(Ordering::Relaxed),
        )
    }

    pub fn reset(&self) {
        self.nanos.store(0, Ordering::Relaxed);
        self.calls.store(0, Ordering::Relaxed);
    }
}

// One static per stage. Each instrumentation site fetches its own static
// directly; the dump/reset helpers below iterate over an array of refs to
// keep them in sync without macro dispatch.

/// Time spent inside `SailingPathBoundary::handle` — bbox clamp +
/// land-projection + per-segment time clamp. One call per outer particle
/// per outer iteration.
#[cfg(feature = "profile-timers")]
pub static SAILING_BOUNDARY: StageTimer = StageTimer::new("sailing_boundary");

/// Time spent inside `TimeNestedMover::update` — runs the entire inner
/// time-PSO for one outer particle. Wraps the cache-build below.
#[cfg(feature = "profile-timers")]
pub static TIME_NESTED_MOVER: StageTimer = StageTimer::new("time_nested_mover");

/// Time spent inside `SegmentRangeTables::build` — the inner-PSO's
/// per-segment travel-time cache. Called once per outer-particle xy
/// refresh, inside `TIME_NESTED_MOVER`'s scope.
#[cfg(feature = "profile-timers")]
pub static SEGMENT_CACHE_BUILD: StageTimer = StageTimer::new("segment_cache_build");

/// Time spent inside `SailboatFitCalc::calculate_fit` — the outer
/// fitness eval. Walks the path and integrates wind per segment.
#[cfg(feature = "profile-timers")]
pub static OUTER_FIT: StageTimer = StageTimer::new("outer_fit");

/// Time spent inside `TimeFitCalc::calculate_fit` — the inner fitness
/// eval. Cached, no wind integration.
#[cfg(feature = "profile-timers")]
pub static INNER_FIT: StageTimer = StageTimer::new("inner_fit");

/// Time spent inside the inner `GBestMover<Time<N>>::update` — the
/// per-iteration PSO velocity + position update on the segment-time
/// vector. One call per inner particle per inner iteration; with default
/// sizing that's 1.92 M calls per search.
#[cfg(feature = "profile-timers")]
pub static INNER_MOVER: StageTimer = StageTimer::new("inner_mover");

/// Time spent inside `TimeBoundary::handle` — the per-segment clamp of
/// each candidate `Time<N>` against the cached `(best, worst)` range.
/// Same call count as `INNER_MOVER` (it runs immediately after the
/// mover step, inside the same `BoundedMover` wrapper).
#[cfg(feature = "profile-timers")]
pub static INNER_BOUNDARY: StageTimer = StageTimer::new("inner_boundary");

/// Time spent inside `PathTimeInit::init_pos` — seeds the inner-PSO's
/// initial per-segment times by computing each segment's travel-time
/// range. Per outer particle, runs `particles_time × (N-1) × 2`
/// wind-integrated travel-time evaluations. One call per outer particle.
#[cfg(feature = "profile-timers")]
pub static INIT_POS_INNER: StageTimer = StageTimer::new("init_pos_inner");

#[cfg(feature = "profile-timers")]
const ALL_TIMERS: &[&StageTimer] = &[
    &SAILING_BOUNDARY,
    &TIME_NESTED_MOVER,
    &SEGMENT_CACHE_BUILD,
    &OUTER_FIT,
    &INNER_FIT,
    &INNER_MOVER,
    &INNER_BOUNDARY,
    &INIT_POS_INNER,
];

/// Tiny pass-through `ParticleMover` wrapper that records the wall time
/// of every inner `update` call into a [`StageTimer`]. Used to instrument
/// the inner `GBestMover<Time<N>>` without touching swarmkit core. Pure
/// thin pipe — `set_context` / `set_iteration` / `PAR_LEAF_SIZE` are all
/// forwarded to the wrapped mover unchanged.
#[cfg(feature = "profile-timers")]
pub(crate) struct TimedMover<M> {
    inner: M,
    timer: &'static StageTimer,
}

#[cfg(feature = "profile-timers")]
impl<M> TimedMover<M> {
    pub fn new(inner: M, timer: &'static StageTimer) -> Self {
        Self { inner, timer }
    }
}

#[cfg(feature = "profile-timers")]
impl<M> swarmkit::Contextful for TimedMover<M>
where
    M: swarmkit::Contextful,
{
    type TContext = M::TContext;

    fn set_context(&mut self, context: Self::TContext) {
        self.inner.set_context(context);
    }

    fn set_iteration(&mut self, iteration: usize, max_iteration: usize) {
        self.inner.set_iteration(iteration, max_iteration);
    }
}

#[cfg(feature = "profile-timers")]
impl<M> swarmkit::ParticleMover for TimedMover<M>
where
    M: swarmkit::ParticleMover,
{
    type TUnit = M::TUnit;
    type TCommon = M::TCommon;

    const PAR_LEAF_SIZE: usize = M::PAR_LEAF_SIZE;

    fn update<R: rand::Rng>(
        &self,
        common: &Self::TCommon,
        rng: &mut R,
        idx: usize,
        p: &mut swarmkit::ParticleRefMut<Self::TUnit>,
    ) {
        let start = std::time::Instant::now();
        self.inner.update(common, rng, idx, p);
        self.timer.record(start.elapsed().as_nanos() as u64);
    }
}

/// Zero every stage's accumulator. Called at the top of [`crate::search`]
/// so per-search dumps reflect that one search only.
#[cfg(feature = "profile-timers")]
pub fn reset_all() {
    for t in ALL_TIMERS {
        t.reset();
    }
}

/// Print a one-line-per-stage breakdown to stderr. Format is fixed-width
/// columns (stage name, total ms, call count, avg µs/call) so easy to
/// `grep` out or eyeball alongside the CLI's `search_dur`.
#[cfg(feature = "profile-timers")]
pub fn dump_to_stderr() {
    eprintln!("=== Profile timers (sub-stage breakdown) ===");
    eprintln!(
        "  {:<22} {:>12}  {:>12}  {:>14}",
        "stage", "total_ms", "calls", "avg_us_per_call",
    );
    for t in ALL_TIMERS {
        let (ns, calls) = t.snapshot();
        let total_ms = (ns as f64) / 1_000_000.0;
        let avg_us = if calls > 0 {
            (ns as f64 / calls as f64) / 1_000.0
        } else {
            0.0
        };
        eprintln!(
            "  {:<22} {:>12.3}  {:>12}  {:>14.3}",
            t.name, total_ms, calls, avg_us,
        );
    }
}
