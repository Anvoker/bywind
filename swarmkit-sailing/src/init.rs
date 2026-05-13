use crate::dynamics::get_travel_time_range;
use crate::path_baseline::PathBaseline;
use crate::route_bounds::RouteBounds;
use crate::spherical::Segment;
use crate::units::Path;
use crate::{LandmassSource, Sailboat, SailboatFitData, SeaPathBias, WindSource};
use rand::{Rng, RngExt as _};
use std::f64::consts::PI;
use std::ops::Range;
use swarmkit::{FitCalc, Group, Particle, ParticleInit};

/// Allocation shares across init shape families. Weights are
/// normalised at allocation; negative is clamped to 0; all-zero falls
/// back to `sin_k1`-only. The largest-share family absorbs rounding
/// drift.
#[derive(Clone, Copy, Debug)]
pub struct InitShares {
    /// Smooth arc `sin(π·t)`; curvature spread across `[-1, 1]`.
    pub sin_k1: f64,
    /// S-curve `sin(2π·t)`. One inflection.
    pub sin_k2: f64,
    /// 1-tack zigzag `sin(3π·t)`. Two inflections.
    pub sin_k3: f64,
    /// 1.5-tack zigzag `sin(4π·t)`. Three inflections.
    pub sin_k4: f64,
    /// Random anchor with triangular falloff to both endpoints —
    /// asymmetric one-feature paths.
    pub anchor: f64,
    /// Independent per-waypoint Gaussian perpendicular noise.
    pub gaussian: f64,
}

impl Default for InitShares {
    fn default() -> Self {
        Self {
            sin_k1: 0.30,
            sin_k2: 0.20,
            sin_k3: 0.15,
            sin_k4: 0.10,
            anchor: 0.15,
            gaussian: 0.10,
        }
    }
}

/// Allocation shares across baseline reference paths.
///
/// The shape families ([`InitShares`]) give *shape* diversity around
/// one baseline; multiple baselines give *topology* diversity (e.g.
/// around-north vs around-south) that no shape family can interpolate
/// between. Same normalisation rules as [`InitShares`].
#[derive(Clone, Copy, Debug)]
pub struct BaselineShares {
    /// Straight-line lerp. Insurance for short routes where
    /// "straight line + big kick" is competitive with detouring.
    pub straight_line: f64,
    /// A* polyline biased north (south of the straight line is blocked).
    pub polyline_north: f64,
    /// Mirror of `polyline_north`.
    pub polyline_south: f64,
}

impl Default for BaselineShares {
    fn default() -> Self {
        // 80% polyline (40/40 north/south) + 20% straight-line.
        Self {
            straight_line: 0.2,
            polyline_north: 0.4,
            polyline_south: 0.4,
        }
    }
}

#[derive(Copy, Clone, Debug)]
enum Family {
    SinK1,
    SinK2,
    SinK3,
    SinK4,
    Anchor,
    Gaussian,
}

/// Per-particle `(family, within_family_index, family_count)`. Falls
/// back to all-`SinK1` for degenerate input.
fn allocate_families(particle_count: usize, shares: &InitShares) -> Vec<(Family, usize, usize)> {
    let raw = [
        (Family::SinK1, shares.sin_k1.max(0.0)),
        (Family::SinK2, shares.sin_k2.max(0.0)),
        (Family::SinK3, shares.sin_k3.max(0.0)),
        (Family::SinK4, shares.sin_k4.max(0.0)),
        (Family::Anchor, shares.anchor.max(0.0)),
        (Family::Gaussian, shares.gaussian.max(0.0)),
    ];
    let total: f64 = raw.iter().map(|(_, s)| s).sum();
    if total <= 0.0 || particle_count == 0 {
        return (0..particle_count)
            .map(|i| (Family::SinK1, i, particle_count))
            .collect();
    }
    let mut counts: Vec<usize> = raw
        .iter()
        .map(|(_, s)| (particle_count as f64 * s / total).round() as usize)
        .collect();
    let sum: usize = counts.iter().sum();
    if sum != particle_count {
        // `total_cmp` is total on f64 (NaN sorts somewhere), eliminating
        // the `partial_cmp().unwrap()` pair. The outer `unwrap_or(0)` is
        // defensive: `raw` is non-empty in every observed call path
        // (the function has 6 families, and the `total <= 0` guard
        // above returns before we get here), but the fallback to 0
        // keeps `counts[largest] += …` well-defined if a future caller
        // ever passes an empty `raw`.
        let largest = raw
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.1.total_cmp(&b.1.1))
            .map_or(0, |(i, _)| i);
        if sum < particle_count {
            counts[largest] += particle_count - sum;
        } else {
            counts[largest] = counts[largest].saturating_sub(sum - particle_count);
        }
    }
    let mut out = Vec::with_capacity(particle_count);
    for (i, (family, _)) in raw.iter().enumerate() {
        let n = counts[i];
        for j in 0..n {
            out.push((*family, j, n));
        }
    }
    out
}

pub struct PathInit<'a, const N: usize, SB, WS, TFit>
where
    SB: Sailboat,
    WS: WindSource,
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    bounds: &'a RouteBounds,
    boat: &'a SB,
    wind_source: &'a WS,
    fit_calc: &'a TFit,
    particle_count: usize,
    init_shares: InitShares,
    baseline_shares: BaselineShares,
}

impl<'a, const N: usize, SB, WS, TFit> PathInit<'a, N, SB, WS, TFit>
where
    SB: Sailboat,
    WS: WindSource,
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    pub fn new(
        bounds: &'a RouteBounds,
        boat: &'a SB,
        wind_source: &'a WS,
        fit_calc: &'a TFit,
        particle_count: usize,
        init_shares: InitShares,
        baseline_shares: BaselineShares,
    ) -> Self {
        PathInit {
            bounds,
            boat,
            wind_source,
            fit_calc,
            particle_count,
            init_shares,
            baseline_shares,
        }
    }

    /// Same as [`ParticleInit::init`] plus a per-baseline partition
    /// for the niched topology — one `Range` per cohort in the order
    /// the internal baseline-computation routine emits, empty cohorts
    /// absent. RNG draws match `init` so the same seed produces the
    /// identical layout.
    pub fn init_with_partition<R: Rng>(&self, rng: &mut R) -> (Group<Path<N>>, Vec<Range<usize>>) {
        let baselines =
            compute_baselines::<N, _>(self.bounds, self.fit_calc.landmass(), &self.baseline_shares);
        let counts = allocate_particles_across_baselines(self.particle_count, &baselines);

        let mut positions: Vec<Path<N>> = Vec::with_capacity(self.particle_count);
        let mut partition: Vec<Range<usize>> = Vec::with_capacity(counts.len());
        let mut cursor = 0usize;
        for ((baseline, _share), count) in baselines.iter().zip(counts.iter()) {
            let ctx = InitContext {
                baseline,
                bounds: self.bounds,
                boat: self.boat,
                wind_source: self.wind_source,
            };
            let mut sub = init_diverse_per_baseline(&ctx, *count, &self.init_shares, rng);
            positions.append(&mut sub);
            partition.push(cursor..cursor + count);
            cursor += count;
        }

        let velocities = self.init_vel(rng);
        debug_assert_eq!(
            positions.len(),
            velocities.len(),
            "init_vel must produce one velocity per position",
        );

        let mut group = Group::<Path<N>>::with_capacity(self.particle_count);
        for (pos, vel) in positions.into_iter().zip(velocities) {
            let fit = self.fit_calc.calculate_fit(pos);
            group.push(Particle {
                pos,
                vel,
                fit,
                best_pos: pos,
                best_fit: fit,
            });
        }
        (group, partition)
    }
}

impl<const N: usize, SB, WS, TFit> ParticleInit for PathInit<'_, N, SB, WS, TFit>
where
    SB: Sailboat,
    WS: WindSource,
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    type T = Path<N>;

    fn init_pos<R: Rng>(&self, rng: &mut R) -> Vec<Self::T> {
        let baselines =
            compute_baselines::<N, _>(self.bounds, self.fit_calc.landmass(), &self.baseline_shares);
        let counts = allocate_particles_across_baselines(self.particle_count, &baselines);
        let mut out: Vec<Self::T> = Vec::with_capacity(self.particle_count);
        for ((baseline, _share), count) in baselines.iter().zip(counts.iter()) {
            let ctx = InitContext {
                baseline,
                bounds: self.bounds,
                boat: self.boat,
                wind_source: self.wind_source,
            };
            let mut sub = init_diverse_per_baseline(&ctx, *count, &self.init_shares, rng);
            out.append(&mut sub);
        }
        out
    }

    fn init_vel<R: Rng>(&self, rng: &mut R) -> Vec<Self::T> {
        (0..self.particle_count)
            .map(|_| init_random_vel_particle(self.bounds, rng, 0.3))
            .collect()
    }
}

/// Active baselines for the next init pass. Falls back to a single
/// straight-line baseline if every share is zero or every A* fails.
fn compute_baselines<const N: usize, LS: LandmassSource>(
    bounds: &RouteBounds,
    landmass: &LS,
    shares: &BaselineShares,
) -> Vec<(PathBaseline<N>, f64)> {
    let mut out: Vec<(PathBaseline<N>, f64)> = Vec::new();
    if shares.straight_line > 0.0 {
        out.push((
            PathBaseline::straight_line(bounds),
            shares.straight_line.max(0.0),
        ));
    }
    if shares.polyline_north > 0.0
        && let Some(poly) = landmass.find_sea_path(
            bounds.origin,
            bounds.destination,
            bounds,
            SeaPathBias::North,
        )
    {
        out.push((
            PathBaseline::from_polyline(&poly, bounds, landmass),
            shares.polyline_north.max(0.0),
        ));
    }
    if shares.polyline_south > 0.0
        && let Some(poly) = landmass.find_sea_path(
            bounds.origin,
            bounds.destination,
            bounds,
            SeaPathBias::South,
        )
    {
        out.push((
            PathBaseline::from_polyline(&poly, bounds, landmass),
            shares.polyline_south.max(0.0),
        ));
    }
    if out.is_empty() {
        out.push((PathBaseline::straight_line(bounds), 1.0));
    }
    out
}

/// Split `particle_count` across baselines proportional to their share,
/// rounding to integers and assigning the rounding drift to the
/// largest-share baseline. Returns one count per input baseline.
fn allocate_particles_across_baselines<const N: usize>(
    particle_count: usize,
    baselines: &[(PathBaseline<N>, f64)],
) -> Vec<usize> {
    if baselines.is_empty() || particle_count == 0 {
        return vec![0; baselines.len()];
    }
    let total: f64 = baselines.iter().map(|(_, s)| s).sum();
    if total <= 0.0 {
        // Degenerate — assign everything to the first baseline.
        let mut counts = vec![0; baselines.len()];
        counts[0] = particle_count;
        return counts;
    }
    let mut counts: Vec<usize> = baselines
        .iter()
        .map(|(_, s)| (particle_count as f64 * s / total).round() as usize)
        .collect();
    let sum: usize = counts.iter().sum();
    if sum != particle_count {
        let largest = baselines
            .iter()
            .enumerate()
            .max_by(|a, b| {
                a.1.1
                    .partial_cmp(&b.1.1)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(0);
        if sum < particle_count {
            counts[largest] += particle_count - sum;
        } else {
            counts[largest] = counts[largest].saturating_sub(sum - particle_count);
        }
    }
    counts
}

// Inner-PSO `Time<N>` init lives in
// `time.rs::build_inner_group_from_cache` now — see
// `bywind/profiling/stage2-counters.md` for why.

// Free functions: the path-building machinery the inits dispatch into.

/// Borrows shared by the init helpers.
pub(crate) struct InitContext<'a, const N: usize, SB, WS> {
    pub baseline: &'a PathBaseline<N>,
    pub bounds: &'a RouteBounds,
    pub boat: &'a SB,
    pub wind_source: &'a WS,
}

/// One slot from [`allocate_families`]: family + `(index, count)` for
/// positioning within the family's curvature-parameter row.
#[derive(Copy, Clone)]
struct FamilyAllocation {
    family: Family,
    within_index: usize,
    family_count: usize,
}

/// Multi-baseline allocation entry point used by `PathInit`.
pub(crate) fn init_diverse_per_baseline<const N: usize, SB, WS, R>(
    ctx: &InitContext<'_, N, SB, WS>,
    particle_count: usize,
    shares: &InitShares,
    rng: &mut R,
) -> Vec<Path<N>>
where
    SB: Sailboat,
    WS: WindSource,
    R: Rng,
{
    let allocation = allocate_families(particle_count, shares);
    allocation
        .into_iter()
        .map(|(family, within_index, family_count)| {
            init_diverse_particle(
                ctx,
                rng,
                FamilyAllocation {
                    family,
                    within_index,
                    family_count,
                },
            )
        })
        .collect()
}

fn init_diverse_particle<const N: usize, SB, WS, R>(
    ctx: &InitContext<'_, N, SB, WS>,
    rng: &mut R,
    alloc: FamilyAllocation,
) -> Path<N>
where
    SB: Sailboat,
    WS: WindSource,
    R: Rng,
{
    let FamilyAllocation {
        family,
        within_index,
        family_count,
    } = alloc;
    let offsets: [f64; N] = match family {
        Family::SinK1 => offsets_sinusoidal::<N>(1, within_index, family_count),
        Family::SinK2 => offsets_sinusoidal::<N>(2, within_index, family_count),
        Family::SinK3 => offsets_sinusoidal::<N>(3, within_index, family_count),
        Family::SinK4 => offsets_sinusoidal::<N>(4, within_index, family_count),
        Family::Anchor => offsets_anchor::<N, _>(rng),
        Family::Gaussian => offsets_gaussian::<N, _>(rng),
    };

    build_path_with_offsets(ctx, rng, &offsets)
}

/// Build a `Path<N>` by perturbing `baseline` with unitless `offsets`.
/// Each interior waypoint is placed at `baseline.positions[i] +
/// (perpendicular metres in tangent frame, converted back to
/// (lon°, lat°))`. Endpoints are pinned to the bounds.
///
/// `bounds` is consulted only for endpoint pinning and per-segment
/// random time seeding; the *position* of interior waypoints comes
/// entirely from the baseline + offsets.
fn build_path_with_offsets<const N: usize, SB, WS, R>(
    ctx: &InitContext<'_, N, SB, WS>,
    rng: &mut R,
    offsets: &[f64; N],
) -> Path<N>
where
    SB: Sailboat,
    WS: WindSource,
    R: Rng,
{
    let InitContext {
        baseline,
        bounds,
        boat,
        wind_source,
    } = *ctx;
    let mut p: Path<N> = Path::default();
    bounds.constrain_endpoints_xyt(&mut p);
    let mut accumulated_time = 0.0;

    for (i, &offset) in offsets.iter().enumerate().skip(1) {
        if i != N - 1 {
            let here = baseline.positions[i];
            let scale = if offset >= 0.0 {
                baseline.perp_scale_pos[i]
            } else {
                baseline.perp_scale_neg[i]
            };
            let amplitude_m = offset * scale;
            let perp_metres = baseline.perpendiculars[i] * amplitude_m;
            // Bbox-clamp: without it, a large `perp_scale` on a long
            // baseline can push past ±90° lat and crash spherical math
            // downstream with NaN / infinite travel times.
            p.xy.set_lat_lon(i, bounds.clamp(here.offset_by(perp_metres)));
        }
        init_random_time(
            bounds,
            boat,
            wind_source,
            rng,
            &mut p,
            &mut accumulated_time,
            i,
        );
    }

    p
}

fn init_random_vel_particle<const N: usize, R: Rng>(
    bounds: &RouteBounds,
    rng: &mut R,
    magnitude_coefficient_01: f64,
) -> Path<N> {
    // Endpoint velocities stay zero (positions constrained); the rest
    // are tangent-frame metres scaled to the bbox diagonal. First-iter
    // cognitive/social pulls dominate this anyway.
    let mut p: Path<N> = Path::default();
    let scale_m = bounds.bbox.diagonal_m() * magnitude_coefficient_01;

    for i in 1..N - 1 {
        let dx = rng.random::<f64>() - 0.5;
        let dy = rng.random::<f64>() - 0.5;
        let mag = dx.hypot(dy).max(1e-12);
        p.xy.0[i] = (dx / mag) * scale_m;
        p.xy.1[i] = (dy / mag) * scale_m;
    }

    p
}

fn init_random_time<const N: usize, SB, WS, R>(
    bounds: &RouteBounds,
    boat: &SB,
    wind_source: &WS,
    rng: &mut R,
    p: &mut Path<N>,
    accumulated_time: &mut f64,
    i: usize,
) where
    SB: Sailboat,
    WS: WindSource,
    R: Rng,
{
    let (t_min, t_max) = get_travel_time_range(
        boat,
        wind_source,
        Segment {
            origin: p.xy.lat_lon(i - 1),
            destination: p.xy.lat_lon(i),
            origin_time: *accumulated_time,
            step_distance_max: bounds.step_distance_max,
        },
    );

    p.t[i] = if t_min < t_max {
        rng.random_range(t_min..t_max)
    } else {
        t_min
    };
    *accumulated_time += p.t[i];
}

/// Unitless `sin(k·π·t)` offsets in `[-1/k, 1/k]`. Curvature factor
/// `c` spreads across `[-1, 1]` over the family; amplitude divided by
/// `k` keeps total path length comparable across frequencies.
fn offsets_sinusoidal<const N: usize>(
    k: u32,
    within_index: usize,
    family_count: usize,
) -> [f64; N] {
    let mut offsets = [0.0; N];
    let c = if family_count > 1 {
        let middle = (family_count as f64 - 1.0) / 2.0;
        (within_index as f64 - middle) / middle
    } else {
        0.0
    };
    let amplitude = c / k as f64;
    for (i, slot) in offsets
        .iter_mut()
        .enumerate()
        .skip(1)
        .take(N.saturating_sub(2))
    {
        let t = (i as f64) / (N as f64);
        *slot = (k as f64 * PI * t).sin() * amplitude;
    }
    offsets
}

/// Unitless offsets: one anchor displacement in `[-1, 1]` with linear
/// falloff to both endpoints. Anchor `t ∈ [0.2, 0.8]` avoids degenerate
/// near-endpoint falloffs.
fn offsets_anchor<const N: usize, R: Rng>(rng: &mut R) -> [f64; N] {
    let mut offsets = [0.0; N];
    if N < 3 {
        return offsets;
    }
    let anchor_t = rng.random_range(0.2_f64..0.8);
    let displacement = rng.random_range(-1.0_f64..1.0);
    let falloff_width = anchor_t.max(1.0 - anchor_t);
    for (i, slot) in offsets
        .iter_mut()
        .enumerate()
        .skip(1)
        .take(N.saturating_sub(2))
    {
        let t = (i as f64) / (N as f64);
        let dist = (t - anchor_t).abs();
        let factor = if dist >= falloff_width {
            0.0
        } else {
            1.0 - dist / falloff_width
        };
        *slot = displacement * factor;
    }
    offsets
}

/// Independent per-waypoint Box–Muller Gaussian offsets, σ = 0.2.
fn offsets_gaussian<const N: usize, R: Rng>(rng: &mut R) -> [f64; N] {
    const SIGMA: f64 = 0.2;
    let mut offsets = [0.0; N];
    for slot in offsets.iter_mut().skip(1).take(N.saturating_sub(2)) {
        let u1 = rng.random::<f64>().max(f64::MIN_POSITIVE);
        let u2 = rng.random::<f64>();
        let z = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
        *slot = z * SIGMA;
    }
    offsets
}
