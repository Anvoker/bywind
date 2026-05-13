//! Sailing-route PSO physics on top of the generic [`swarmkit`]
//! particle-swarm library.
//!
//! Provides the spherical-Earth coordinate primitives, wind / boat /
//! landmass traits, and PSO movers (init, mutation, time, range-cache)
//! that the [`bywind`](https://crates.io/crates/bywind) sailing-route
//! optimiser composes into a full search. Headless: no GUI, no GRIB2
//! I/O — those live in `bywind`.
//!
//! - [`spherical`] — coordinate / segment / wind types and great-circle
//!   primitives shared by every other module.
//! - [`Boat`] / [`Sailboat`] — boat physics model and the trait the PSO
//!   evaluates fitness against.
//! - [`SearchSettings`] — outer / inner PSO sizing and topology choice.
//! - [`RouteBounds`] / [`LonLatBbox`] — search-domain types.
//! - [`Topology`] — `GBest`, `Ring`, `VonNeumann`, `Niched`.

mod boat;
mod boundary;
mod dynamics;
pub(crate) mod fit;
mod init;
mod mutation;
mod path_baseline;
mod range_cache;
mod route_bounds;
pub mod spherical;
mod spherical_pso;
mod time;
mod traits;
pub mod units;

// Sub-stage timing counters for the search hot paths. Whole module is
// gated on the `profile-timers` feature so non-feature builds get
// nothing — no atomics, no extra calls, no extra fields.
#[cfg(feature = "profile-timers")]
pub mod profile_timers;

pub use boat::Boat;
pub use dynamics::{get_segment_fuel_and_time, get_segment_land_metres};
pub use fit::{SailboatFitCalc, weighted_fitness};
pub use init::{BaselineShares, InitShares, PathInit};
pub use path_baseline::PathBaseline;
pub use route_bounds::{DEFAULT_STEP_DISTANCE_FRACTION, RouteBounds};
pub use spherical::{LatLon, LatLonDelta, LonLatBbox, Segment, TangentMetres, Wind};
pub use traits::*;
pub use units::{Floats, Path, PathXY, Time};

#[cfg(feature = "probe-stats")]
pub use range_cache::{ProbeCounters, ProbeStats, SegmentRangeTables};

use boundary::SailingPathBoundary;
use fit::PathFitCalc;
use mutation::{CauchyKickMover, ShapeKickMover};
use rand::SeedableRng as _;
use rand::rngs::SmallRng;
use spherical::{haversine, initial_bearing};
use spherical_pso::SphericalPSOMover;
use swarmkit::{
    Best, Evolution, FitCalc, IntoGBestSearcher as _, IntoLBestSearcher as _,
    IntoNichedSearcher as _, LBestKind, PSOCoeffs, ParticleMover as _, Searcher,
};
use time::TimeNestedMover;

/// Outer-loop search topology. The match seam in [`search`] is the
/// only topology-aware code; init, kicks, boundary, time PSO, and
/// fitness are topology-agnostic.
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Topology {
    /// Stock gbest. Fastest convergence; collapses corridor diversity
    /// onto whichever corridor wins the first few iterations.
    #[default]
    GBest,
    /// Each baseline cohort (north / south polyline, straight-line)
    /// gets its own social attractor. Preserves corridor diversity at
    /// the cost of slower per-niche convergence. v0 is static (no
    /// migration / merging).
    Niched,
    /// Ring lbest with k=1: each particle pulls toward the better of
    /// `(i-1, i+1) mod n`. Slower convergence than gbest, longer
    /// diversity.
    Ring,
    /// Lbest on an `r×c` torus, 4 neighbours each with wraparound.
    /// Slightly faster diffusion than ring at the same swarm size;
    /// falls back to ring when no balanced factorization exists.
    VonNeumann,
}

impl Topology {
    /// Lowercase string for CLI flags / TOML keys.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::GBest => "gbest",
            Self::Niched => "niched",
            Self::Ring => "ring",
            Self::VonNeumann => "von_neumann",
        }
    }

    /// All variants in declaration order, for help text / completion.
    pub const ALL: &'static [Self] = &[Self::GBest, Self::Niched, Self::Ring, Self::VonNeumann];
}

impl std::fmt::Display for Topology {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Unknown topology name from [`Topology`]'s `FromStr`. The message
/// lists `Topology::ALL` for CLI / TOML diagnostics.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseTopologyError(pub String);

impl std::fmt::Display for ParseTopologyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let valid = Topology::ALL
            .iter()
            .map(|t| t.as_str())
            .collect::<Vec<_>>()
            .join(", ");
        write!(
            f,
            "unknown topology '{}' (expected one of: {valid})",
            self.0
        )
    }
}

impl std::error::Error for ParseTopologyError {}

impl std::str::FromStr for Topology {
    type Err = ParseTopologyError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        for &t in Self::ALL {
            if t.as_str().eq_ignore_ascii_case(s) {
                return Ok(t);
            }
        }
        Err(ParseTopologyError(s.to_owned()))
    }
}

/// Outer (space) + inner (time) PSO sizing and coefficients. Both PSOs
/// share the velocity-update coefficients; the inner PSO runs once per
/// outer particle per iteration.
///
/// `mutation_gamma_*_fraction`: per-waypoint independent Cauchy mutation
/// scales, as a fraction of the straight-line route length,
/// cosine-decayed across iterations. `0.0` disables.
///
/// `path_kick_*`: probability + initial/floor magnitudes of a
/// coordinated `sin(k·π·t)` perpendicular shape kick (k ∈ {2, 3, 4}),
/// the only mechanism that escapes smooth-arc basins to invent tacking
/// topologies. `probability = 0` disables.
#[derive(Clone, Copy, Debug)]
pub struct SearchSettings {
    pub particle_count_space: usize,
    pub particle_count_time: usize,
    pub max_iteration_space: usize,
    pub max_iteration_time: usize,
    pub inertia: f64,
    pub cognitive_coeff: f64,
    pub social_coeff: f64,
    pub init_shares: InitShares,
    /// Outer-init baseline shares. Default 80% polyline (40/40
    /// north/south) + 20% straight-line.
    pub baseline_shares: BaselineShares,
    pub mutation_gamma_0_fraction: f64,
    pub mutation_gamma_min_fraction: f64,
    pub path_kick_probability: f64,
    pub path_kick_gamma_0_fraction: f64,
    pub path_kick_gamma_min_fraction: f64,
    /// `None` draws fresh OS entropy; `Some(s)` reproduces the run.
    pub seed: Option<u64>,
    /// Inner time-PSO lookup-table samples along departure-time. ≥ 2.
    pub range_k: usize,
    /// Inner time-PSO lookup-table samples along `mcr_01` throttle. ≥ 2.
    pub k_mcr: usize,
    pub topology: Topology,
}

impl Default for SearchSettings {
    fn default() -> Self {
        Self {
            particle_count_space: 40,
            particle_count_time: 40,
            max_iteration_space: 40,
            max_iteration_time: 30,
            inertia: 0.2,
            cognitive_coeff: 1.6,
            social_coeff: 0.85,
            init_shares: InitShares::default(),
            baseline_shares: BaselineShares::default(),
            mutation_gamma_0_fraction: 0.0,
            mutation_gamma_min_fraction: 0.0,
            path_kick_probability: 0.1,
            path_kick_gamma_0_fraction: 0.05,
            path_kick_gamma_min_fraction: 0.005,
            seed: None,
            range_k: crate::range_cache::DEFAULT_RANGE_K,
            k_mcr: crate::range_cache::DEFAULT_K_MCR,
            topology: Topology::default(),
        }
    }
}

pub fn search<
    const N: usize,
    SB: Sailboat,
    WS: WindSource,
    LS: LandmassSource,
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
>(
    boat: &SB,
    wind_source: &WS,
    landmass: &LS,
    route_bounds: RouteBounds,
    fit_calc: &TFit,
    settings: SearchSettings,
) -> (Best<Path<N>>, Evolution<Path<N>>) {
    // Per-search reset of the sub-stage counters so the dump at the end
    // reflects only this search. No-op when the feature is off.
    #[cfg(feature = "profile-timers")]
    profile_timers::reset_all();

    let SearchSettings {
        particle_count_space,
        particle_count_time,
        max_iteration_space,
        max_iteration_time,
        inertia,
        cognitive_coeff,
        social_coeff,
        init_shares,
        baseline_shares,
        mutation_gamma_0_fraction,
        mutation_gamma_min_fraction,
        path_kick_probability,
        path_kick_gamma_0_fraction,
        path_kick_gamma_min_fraction,
        seed,
        range_k,
        k_mcr,
        topology,
    } = settings;

    // Master RNG for both PathInit and the GBestSearcher's per-particle
    // streams. With `seed = Some(s)` the run is fully reproducible; with
    // `seed = None` we draw fresh OS entropy (legacy behaviour).
    let mut master_rng: SmallRng = match seed {
        Some(s) => SmallRng::seed_from_u64(s),
        None => rand::make_rng(),
    };

    let pso_coeffs = PSOCoeffs::new(inertia, cognitive_coeff, social_coeff);

    let path_fit = PathFitCalc::new(fit_calc);

    // `init_with_partition` returns the per-baseline cohort ranges
    // alongside the group. Topologies that don't care (gbest) just
    // ignore the partition; niched feeds it to `NichedSearcher`.
    let (mut group, partition) = PathInit::new(
        &route_bounds,
        boat,
        wind_source,
        &path_fit,
        particle_count_space,
        init_shares,
        baseline_shares,
    )
    .init_with_partition(&mut master_rng);

    // Mutation kick scales are expressed as fractions of the straight-line
    // ground distance, so the noise budget rescales naturally with the
    // route size. Mutation movers apply samples in tangent-frame metres,
    // which means the line length must be metres too — degree-hypot would
    // mis-scale routes that aren't equator-aligned.
    let line_length_m = haversine(route_bounds.origin, route_bounds.destination);
    let gamma_0 = (mutation_gamma_0_fraction * line_length_m).max(0.0);
    let gamma_min = (mutation_gamma_min_fraction * line_length_m).max(0.0);
    let path_gamma_0 = (path_kick_gamma_0_fraction * line_length_m).max(0.0);
    let path_gamma_min = (path_kick_gamma_min_fraction * line_length_m).max(0.0);

    // Perpendicular to the route's initial bearing, in compass radians.
    // CCW rotation in geographic terms is `bearing − π/2` (compass is
    // clockwise). Pole fallback is arbitrary east — Cauchy magnitudes are
    // symmetric, so direction sign doesn't change the kick distribution.
    let route_bearing =
        initial_bearing(route_bounds.origin, route_bounds.destination).unwrap_or(0.0);
    let perp_bearing = route_bearing - std::f64::consts::FRAC_PI_2;

    // Spherical-aware velocity update: each waypoint's pull is converted
    // to local east-north tangent metres before the inertia/cognitive/
    // social arithmetic, then back to `(Δlon, Δlat)` for the position
    // update. See `spherical_pso` for the full rationale.
    let space_mover = SphericalPSOMover::<N, Path<N>>::new(pso_coeffs)
        .chain(CauchyKickMover::<N, Path<N>>::new(gamma_0, gamma_min))
        .chain(ShapeKickMover::<N, Path<N>>::new(
            path_kick_probability,
            path_gamma_0,
            path_gamma_min,
            perp_bearing,
        ))
        .adapt::<Best<Path<N>>>()
        .bounded_by(SailingPathBoundary::new(
            &route_bounds,
            boat,
            wind_source,
            landmass,
        ));

    let time_mover = TimeNestedMover::new(
        &path_fit,
        pso_coeffs,
        max_iteration_time,
        particle_count_time,
        range_k,
        k_mcr,
    );

    // Topology dispatch: this match is the single seam where the choice of
    // outer-loop searcher lands. Each arm constructs its own searcher and
    // runs it to completion via `run_to_completion`, because the searchers
    // are different concrete types. Everything above (init, mover chain,
    // kick movers, boundary, time PSO) is topology-agnostic.
    let mut evolution: Evolution<Path<N>> = Evolution::default();
    let chained = space_mover.chain(time_mover);
    let path_fit_for_searcher = PathFitCalc::new(fit_calc);
    let last = match topology {
        Topology::GBest => run_to_completion(
            chained.into_gbest_searcher(path_fit_for_searcher),
            &mut master_rng,
            max_iteration_space,
            &mut group,
            &mut evolution,
        ),
        Topology::Niched => run_to_completion(
            chained.into_niched_searcher(path_fit_for_searcher, partition),
            &mut master_rng,
            max_iteration_space,
            &mut group,
            &mut evolution,
        ),
        Topology::Ring => run_to_completion(
            chained.into_lbest_searcher(path_fit_for_searcher, LBestKind::Ring { k: 1 }),
            &mut master_rng,
            max_iteration_space,
            &mut group,
            &mut evolution,
        ),
        Topology::VonNeumann => run_to_completion(
            chained.into_lbest_searcher(path_fit_for_searcher, LBestKind::VonNeumann),
            &mut master_rng,
            max_iteration_space,
            &mut group,
            &mut evolution,
        ),
    };

    #[cfg(feature = "profile-timers")]
    profile_timers::dump_to_stderr();

    (last, evolution)
}

/// Reseed `searcher` with the caller's master RNG and run it to the end
/// of `max_iter` iterations, returning the final `Best`. Each topology
/// arm in [`search`] funnels its constructed searcher through this
/// helper so the dispatch match doesn't repeat `reseed + iter + fold`
/// four times.
///
/// The `reseed` call overrides whatever RNG state
/// `<Topology>Searcher::new` initialised (each constructor seeds from
/// OS entropy by default) with the master RNG built from
/// `settings.seed`, so a `Some(seed)` run is fully reproducible.
///
/// When `max_iter == 0` the iterator yields nothing and the returned
/// `Best` is `Best::default()` (a sentinel with `best_fit = f64::MIN`).
/// `SearchSettings::validate` rejects zero iteration counts, so this
/// branch is unreachable for sanctioned callers.
fn run_to_completion<const N: usize, S>(
    mut searcher: S,
    master_rng: &mut SmallRng,
    max_iter: usize,
    group: &mut swarmkit::Group<Path<N>>,
    evolution: &mut Evolution<Path<N>>,
) -> Best<Path<N>>
where
    S: Searcher<TUnit = Path<N>>,
{
    searcher.reseed(master_rng);
    // Fold the per-iteration snapshots; the closure discards the
    // accumulator and keeps the latest snapshot, so the final return
    // value is whatever the searcher yielded last.
    searcher
        .iter(max_iter, group, Some(evolution))
        .fold(Best::default(), |_acc, snapshot| snapshot)
}

/// Re-runs only the inner time PSO over `fixed_path`, holding
/// `fixed_path.xy` constant and producing an optimized `t`.
///
/// The returned `Path<N>` carries the caller's xy unchanged. Mirrors
/// what the inner PSO inside [`search`] would have produced for that
/// xy if it had been a candidate of the outer space PSO.
///
/// `settings.particle_count_time` and `settings.max_iteration_time` control the
/// PSO sizing; the `*_space` fields are unused. Initialization is random — the
/// caller's existing `t` is *not* used as a seed.
pub fn reoptimize_times<const N: usize, TFit: FitCalc<T = Path<N>> + SailboatFitData>(
    fit_calc: &TFit,
    settings: SearchSettings,
    fixed_path: Path<N>,
) -> Path<N> {
    let SearchSettings {
        particle_count_time,
        max_iteration_time,
        inertia,
        cognitive_coeff,
        social_coeff,
        seed,
        range_k,
        k_mcr,
        ..
    } = settings;

    let mut master_rng: SmallRng = match seed {
        Some(s) => SmallRng::seed_from_u64(s),
        None => rand::make_rng(),
    };

    let pso_coeffs = PSOCoeffs::new(inertia, cognitive_coeff, social_coeff);

    let path_fit = PathFitCalc::new(fit_calc);
    let best_t = time::reoptimize_time(
        &path_fit,
        particle_count_time,
        pso_coeffs,
        max_iteration_time,
        range_k,
        k_mcr,
        fixed_path,
        &mut master_rng,
    );

    Path {
        xy: fixed_path.xy,
        t: best_t,
    }
}
