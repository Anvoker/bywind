use crate::spherical::TangentMetres;
use crate::units::PathXY;
use rand::{Rng, RngExt as _};
use std::f64::consts::PI;
use std::marker::PhantomData;
use swarmkit::{Best, Contextful, ParticleMover, ParticleRefMut};

/// Independent 2D `Cauchy(γ_t)` per interior waypoint in tangent
/// (east, north) metres — isotropic in ground metres, not degrees.
/// γ cosine-decays: `γ_t = γ_min + (γ_0 - γ_min) · 0.5 · (1 + cos(π·t))`.
/// Cauchy fat tails give occasional basin-escape jumps; the boundary
/// downstream clamps anything out of bounds. Chained after
/// `SphericalPSOMover` so the PSO velocity update stays pure.
#[derive(Copy, Clone, Debug)]
pub(crate) struct CauchyKickMover<const N: usize, TContext>
where
    TContext: Copy,
{
    gamma_0: f64,
    gamma_min: f64,
    iteration: usize,
    max_iteration: usize,
    phantom: PhantomData<fn() -> TContext>,
}

impl<const N: usize, TContext> CauchyKickMover<N, TContext>
where
    TContext: Copy,
{
    pub fn new(gamma_0: f64, gamma_min: f64) -> Self {
        Self {
            gamma_0,
            gamma_min,
            iteration: 0,
            max_iteration: 1,
            phantom: PhantomData,
        }
    }

    fn current_gamma(&self) -> f64 {
        cosine_decay(
            self.iteration,
            self.max_iteration,
            self.gamma_0,
            self.gamma_min,
        )
    }
}

impl<const N: usize, TContext> Contextful for CauchyKickMover<N, TContext>
where
    TContext: Copy,
{
    type TContext = TContext;

    fn set_iteration(&mut self, iteration: usize, max_iteration: usize) {
        self.iteration = iteration;
        self.max_iteration = max_iteration;
    }
}

impl<const N: usize, TContext> ParticleMover for CauchyKickMover<N, TContext>
where
    TContext: Copy,
{
    type TUnit = PathXY<N>;
    // Match SphericalPSOMover's `TCommon` so they share a `ChainedMover`.
    type TCommon = Best<PathXY<N>>;

    fn update<R: Rng>(
        &self,
        _common: &Self::TCommon,
        rng: &mut R,
        _idx: usize,
        p: &mut ParticleRefMut<'_, Self::TUnit>,
    ) {
        if N < 3 {
            return;
        }
        let gamma = self.current_gamma();
        if gamma <= 0.0 {
            return;
        }
        for i in 1..N - 1 {
            let tangent_en =
                TangentMetres::new(gamma * sample_cauchy(rng), gamma * sample_cauchy(rng));
            p.pos.set_lat_lon(i, p.pos.lat_lon(i).offset_by(tangent_en));
        }
    }
}

/// Coherent path-level kick.
///
/// With probability `p`, picks `k ∈ {2, 3, 4}` and adds
/// `magnitude · sin(k·π·t)` perpendicular to all interior waypoints —
/// an S-curve (k=2), 1-tack zigzag (k=3), or 1.5-tack (k=4).
/// `magnitude` is a single `Cauchy(γ_t)` sample (same cosine schedule
/// as `CauchyKickMover`). Unlike that mover's white-noise per-waypoint
/// kicks, this is the move that escapes smooth-arc basins to discover
/// tacking. k=1 is excluded — PSO refinement and `init_diverse`'s k=1
/// share already cover that shape.
#[derive(Copy, Clone, Debug)]
pub(crate) struct ShapeKickMover<const N: usize, TContext>
where
    TContext: Copy,
{
    probability: f64,
    gamma_0: f64,
    gamma_min: f64,
    perp_east: f64,
    perp_north: f64,
    iteration: usize,
    max_iteration: usize,
    phantom: PhantomData<fn() -> TContext>,
}

impl<const N: usize, TContext> ShapeKickMover<N, TContext>
where
    TContext: Copy,
{
    pub fn new(probability: f64, gamma_0: f64, gamma_min: f64, perp_bearing_rad: f64) -> Self {
        // Compass tangent frame: east = sin(bearing), north = cos(bearing).
        let (perp_east, perp_north) = perp_bearing_rad.sin_cos();
        Self {
            probability,
            gamma_0,
            gamma_min,
            perp_east,
            perp_north,
            iteration: 0,
            max_iteration: 1,
            phantom: PhantomData,
        }
    }

    fn current_gamma(&self) -> f64 {
        cosine_decay(
            self.iteration,
            self.max_iteration,
            self.gamma_0,
            self.gamma_min,
        )
    }
}

impl<const N: usize, TContext> Contextful for ShapeKickMover<N, TContext>
where
    TContext: Copy,
{
    type TContext = TContext;

    fn set_iteration(&mut self, iteration: usize, max_iteration: usize) {
        self.iteration = iteration;
        self.max_iteration = max_iteration;
    }
}

impl<const N: usize, TContext> ParticleMover for ShapeKickMover<N, TContext>
where
    TContext: Copy,
{
    type TUnit = PathXY<N>;
    type TCommon = Best<PathXY<N>>;

    fn update<R: Rng>(
        &self,
        _common: &Self::TCommon,
        rng: &mut R,
        _idx: usize,
        p: &mut ParticleRefMut<'_, Self::TUnit>,
    ) {
        if N < 3 || self.probability <= 0.0 {
            return;
        }
        if rng.random::<f64>() >= self.probability {
            return;
        }
        let gamma = self.current_gamma();
        if gamma <= 0.0 {
            return;
        }
        const K_CHOICES: [u32; 3] = [2, 3, 4];
        let k = K_CHOICES[rng.random_range(0..K_CHOICES.len())] as f64;
        let magnitude = gamma * sample_cauchy(rng);
        for i in 1..N - 1 {
            let t = (i as f64) / (N as f64);
            let amp = magnitude * (k * PI * t).sin();
            let tangent_en = TangentMetres::new(amp * self.perp_east, amp * self.perp_north);
            p.pos.set_lat_lon(i, p.pos.lat_lon(i).offset_by(tangent_en));
        }
    }
}

/// Cauchy(0, 1) via inverse-CDF, with `u` clamped off 0/1 so `tan`
/// stays finite.
fn sample_cauchy<R: Rng>(rng: &mut R) -> f64 {
    let u = rng.random::<f64>().clamp(1e-10, 1.0 - 1e-10);
    (PI * (u - 0.5)).tan()
}

/// `γ_t = γ_min + (γ_0 - γ_min) · 0.5 · (1 + cos(π·t))` with `t` mapping
/// iteration `[1, max_iteration]` onto `[0, 1]`. Iteration 0 is init
/// (no mover runs), so the first call lands at `γ_0`.
fn cosine_decay(iteration: usize, max_iteration: usize, gamma_0: f64, gamma_min: f64) -> f64 {
    let denom = max_iteration.saturating_sub(1).max(1) as f64;
    let t = (iteration.saturating_sub(1) as f64 / denom).clamp(0.0, 1.0);
    let cos_factor = 0.5 * (1.0 + (PI * t).cos());
    gamma_min + (gamma_0 - gamma_min) * cos_factor
}
