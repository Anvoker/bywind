//! Time-search components. `SegmentRangeTables` is built once per
//! outer particle and shared by `TimeFitCalc` + `TimeBoundary` for one
//! inner search. The inner PSO is a stock `GBestSearcher`; sailing
//! specifics stay in the three types below.

use crate::range_cache::SegmentRangeTables;
use crate::units::{Floats, Path, PathXY, Time};
use crate::{SailboatFitData, dynamics};
use rand::{Rng, RngExt as _};
use swarmkit::{
    Best, Boundary, Contextful, FitCalc, GBestMover, GBestSearcher, Group, PSOCoeffs, Particle,
    ParticleMover, ParticleRefMut, Searcher as _, SetTo as _,
};

// Serial: profile shows per-call work ~1-2 µs, well under rayon's
// task overhead. Outer parallelism (`TimeNestedMover` at
// `PAR_LEAF_SIZE=1`) already runs each outer particle's inner search
// on its own core. (Was `4` before profiling.)
const FIT_PAR_LEAF_SIZE: usize = usize::MAX;

/// Inner time-PSO fitness. Borrows the segment-range cache from
/// whoever built it; that cache must outlive this fit-calc.
pub(crate) struct TimeFitCalc<'cache, 'a, const N: usize, TFit>
where
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    fit_calc: &'a TFit,
    tables: &'cache SegmentRangeTables<N>,
    xy: PathXY<N>,
}

impl<'cache, 'a, const N: usize, TFit> TimeFitCalc<'cache, 'a, N, TFit>
where
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    pub fn new(fit_calc: &'a TFit, tables: &'cache SegmentRangeTables<N>, xy: PathXY<N>) -> Self {
        Self {
            fit_calc,
            tables,
            xy,
        }
    }
}

impl<const N: usize, TFit> Contextful for TimeFitCalc<'_, '_, N, TFit>
where
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    type TContext = Path<N>;

    fn set_context(&mut self, context: Path<N>) {
        self.xy = context.xy;
    }
}

impl<const N: usize, TFit> FitCalc for TimeFitCalc<'_, '_, N, TFit>
where
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    type T = Time<N>;
    const PAR_LEAF_SIZE: usize = FIT_PAR_LEAF_SIZE;

    fn calculate_fit(&self, time: Time<N>) -> f64 {
        #[cfg(feature = "profile-timers")]
        let __profile_start = std::time::Instant::now();

        let path = Path {
            xy: self.xy,
            t: time,
        };
        let mut fuel_acc = 0.0;
        let mut time_acc = 0.0;
        for (seg_i, seg) in path
            .iter_with_running_clock(self.fit_calc.departure_time())
            .enumerate()
        {
            let (_mcr_01, fuel) = dynamics::get_segment_metrics_cached(
                self.fit_calc.ship(),
                self.tables,
                seg_i,
                seg.origin,
                seg.destination,
                seg.t_depart,
                seg.t_arrive,
            );
            fuel_acc += fuel;
            time_acc += seg.segment_time;
        }
        let fit =
            -(time_acc * self.fit_calc.time_weight() + fuel_acc * self.fit_calc.fuel_weight());

        #[cfg(feature = "profile-timers")]
        crate::profile_timers::INNER_FIT.record(__profile_start.elapsed().as_nanos() as u64);

        fit
    }
}

/// `Boundary<T = Time<N>>` for the inner time-PSO. Clamps each per-segment
/// time against the tabulated `(best, worst)` range from the shared cache.
pub(crate) struct TimeBoundary<'cache, const N: usize> {
    tables: &'cache SegmentRangeTables<N>,
    departure_time: f64,
}

impl<'cache, const N: usize> TimeBoundary<'cache, N> {
    pub fn new(tables: &'cache SegmentRangeTables<N>, departure_time: f64) -> Self {
        Self {
            tables,
            departure_time,
        }
    }
}

impl<const N: usize> Contextful for TimeBoundary<'_, N> {
    // xy is baked into `tables` at build time; the default no-op
    // `set_context` is exactly what we want.
    type TContext = Path<N>;
}

impl<const N: usize> Boundary for TimeBoundary<'_, N> {
    type T = Time<N>;

    fn handle(&self, pos: Time<N>) -> Time<N> {
        #[cfg(feature = "profile-timers")]
        let __profile_start = std::time::Instant::now();

        let mut clamped = [0.0f64; N];
        let mut acc_time = self.departure_time;
        for (i, slot) in clamped.iter_mut().enumerate().skip(1) {
            let (best_t, worst_t) = self.tables.query_range(i - 1, acc_time);
            *slot = f64::clamp(pos.0[i], best_t, worst_t);
            acc_time += *slot;
        }

        #[cfg(feature = "profile-timers")]
        crate::profile_timers::INNER_BOUNDARY.record(__profile_start.elapsed().as_nanos() as u64);

        Time(Floats(clamped))
    }
}

/// Build cache → seed group from cache (no fresh wind integrations,
/// the cache already tabulated the per-segment `(t_min, t_max)`
/// ranges) → run a stock `GBestSearcher`. Shared by
/// [`TimeNestedMover::update`] and `reoptimize_times`.
#[expect(
    clippy::too_many_arguments,
    reason = "Inner-PSO knobs the caller picks independently."
)]
fn run_inner_time_pso<R, const N: usize, TFit>(
    fit_calc: &TFit,
    particle_count: usize,
    pso_coeffs: PSOCoeffs,
    max_iteration: usize,
    range_k: usize,
    k_mcr: usize,
    fixed_path: Path<N>,
    rng: &mut R,
) -> Time<N>
where
    R: Rng,
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    let xy = fixed_path.xy;

    let cache = SegmentRangeTables::build(
        &xy,
        fit_calc.ship(),
        fit_calc.wind_source(),
        fit_calc.step_distance_max(),
        fit_calc.departure_time(),
        range_k,
        k_mcr,
    );

    let mut group = build_inner_group_from_cache(
        &cache,
        fit_calc.departure_time(),
        fixed_path,
        particle_count,
        rng,
    );

    let inner_fit = TimeFitCalc::new(fit_calc, &cache, xy);
    let inner_bnd = TimeBoundary::new(&cache, fit_calc.departure_time());
    // Timer wraps the mover (not the boundary, which has its own
    // counter). Compiles out without `profile-timers`.
    #[cfg(feature = "profile-timers")]
    let inner_mover = crate::profile_timers::TimedMover::new(
        GBestMover::<Time<N>, Path<N>>::new(pso_coeffs),
        &crate::profile_timers::INNER_MOVER,
    )
    .bounded_by(inner_bnd);
    #[cfg(not(feature = "profile-timers"))]
    let inner_mover = GBestMover::<Time<N>, Path<N>>::new(pso_coeffs).bounded_by(inner_bnd);

    let mut searcher = GBestSearcher::new(inner_fit, inner_mover);
    searcher.reseed(rng);
    searcher.set_context(fixed_path);

    let best = searcher
        .iter(max_iteration, &mut group, None)
        .last()
        .expect("max_iteration must be > 0");
    best.best_pos
}

/// Seeds the inner-PSO group from the cache rather than re-integrating
/// wind. Position: uniform sample inside each segment's tabulated
/// `(t_min, t_max)` (sub-1% interpolation error is irrelevant — the
/// PSO refines it away). Velocity: per-segment random in
/// `[-0.1·t, +0.1·t]` where `t` is the outer particle's existing
/// segment time. Saved ~40% of inner-PSO CPU vs the prior fresh-
/// integration init.
fn build_inner_group_from_cache<R, const N: usize>(
    cache: &SegmentRangeTables<N>,
    departure_time: f64,
    fixed_path: Path<N>,
    particle_count: usize,
    rng: &mut R,
) -> Group<Time<N>>
where
    R: Rng,
{
    #[cfg(feature = "profile-timers")]
    let __profile_start = std::time::Instant::now();

    let seg_count = N.saturating_sub(1);
    let mut group = Group::<Time<N>>::with_capacity(particle_count);
    for _ in 0..particle_count {
        let mut times = [0.0f64; N];
        let mut vel = [0.0f64; N];
        let mut acc_time = departure_time;
        for seg_i in 0..seg_count {
            let (best_t, worst_t) = cache.query_range(seg_i, acc_time);
            let segment_time = if best_t < worst_t {
                rng.random_range(best_t..worst_t)
            } else {
                best_t
            };
            times[seg_i + 1] = segment_time;
            acc_time += segment_time;

            let outer_t = fixed_path.t[seg_i + 1];
            let vel_scale = if outer_t > 0.0 { outer_t * 0.1 } else { 1.0 };
            vel[seg_i + 1] = (rng.random::<f64>() - 0.5) * 2.0 * vel_scale;
        }
        let pos = Time(Floats(times));
        let vel_unit = Time(Floats(vel));
        group.push(Particle {
            pos,
            vel: vel_unit,
            fit: f64::MIN,
            best_pos: pos,
            best_fit: f64::MIN,
        });
    }

    #[cfg(feature = "profile-timers")]
    crate::profile_timers::INIT_POS_INNER.record(__profile_start.elapsed().as_nanos() as u64);

    group
}

/// Outer-mover wrapper that runs an inner GBest time-PSO per outer
/// (xy) particle. The segment-range cache lives on [`update`]'s
/// stack and is shared by reference with the inner fit-calc, boundary,
/// and init.
///
/// [`update`]: ParticleMover::update
pub(crate) struct TimeNestedMover<'a, const N: usize, TFit>
where
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    fit_calc: &'a TFit,
    pso_coeffs: PSOCoeffs,
    max_iteration: usize,
    particle_count: usize,
    range_k: usize,
    k_mcr: usize,
}

impl<'a, const N: usize, TFit> TimeNestedMover<'a, N, TFit>
where
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    pub fn new(
        fit_calc: &'a TFit,
        pso_coeffs: PSOCoeffs,
        max_iteration: usize,
        particle_count: usize,
        range_k: usize,
        k_mcr: usize,
    ) -> Self {
        Self {
            fit_calc,
            pso_coeffs,
            max_iteration,
            particle_count,
            range_k,
            k_mcr,
        }
    }
}

// Manual Clone: `derive(Clone)` would demand `TFit: Clone`; the fit
// calc is held by reference, so a clone is just a borrow copy.
impl<const N: usize, TFit> Clone for TimeNestedMover<'_, N, TFit>
where
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    fn clone(&self) -> Self {
        Self {
            fit_calc: self.fit_calc,
            pso_coeffs: self.pso_coeffs,
            max_iteration: self.max_iteration,
            particle_count: self.particle_count,
            range_k: self.range_k,
            k_mcr: self.k_mcr,
        }
    }
}

impl<const N: usize, TFit> Contextful for TimeNestedMover<'_, N, TFit>
where
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    // Context comes from each outer particle in `update`; the default
    // no-op `set_context` is exactly what we want.
    type TContext = Path<N>;
}

impl<const N: usize, TFit> ParticleMover for TimeNestedMover<'_, N, TFit>
where
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    type TUnit = Path<N>;
    type TCommon = Best<Path<N>>;
    // One rayon task per outer particle; each runs a full inner search.
    const PAR_LEAF_SIZE: usize = 1;

    fn update<R: Rng>(
        &self,
        _common: &Self::TCommon,
        rng: &mut R,
        _idx: usize,
        p: &mut ParticleRefMut<'_, Self::TUnit>,
    ) {
        #[cfg(feature = "profile-timers")]
        let __profile_start = std::time::Instant::now();

        let outer = *p.pos;
        let best_time = run_inner_time_pso(
            self.fit_calc,
            self.particle_count,
            self.pso_coeffs,
            self.max_iteration,
            self.range_k,
            self.k_mcr,
            outer,
            rng,
        );
        best_time.set_to_ref_mut(p);

        #[cfg(feature = "profile-timers")]
        crate::profile_timers::TIME_NESTED_MOVER
            .record(__profile_start.elapsed().as_nanos() as u64);
    }
}

/// One inner time-PSO over a fixed `xy`. Used by `reoptimize_times`.
#[expect(clippy::too_many_arguments, reason = "mirrors run_inner_time_pso.")]
pub(crate) fn reoptimize_time<R, const N: usize, TFit>(
    fit_calc: &TFit,
    particle_count: usize,
    pso_coeffs: PSOCoeffs,
    max_iteration: usize,
    range_k: usize,
    k_mcr: usize,
    fixed_path: Path<N>,
    rng: &mut R,
) -> Time<N>
where
    R: Rng,
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    run_inner_time_pso(
        fit_calc,
        particle_count,
        pso_coeffs,
        max_iteration,
        range_k,
        k_mcr,
        fixed_path,
        rng,
    )
}
