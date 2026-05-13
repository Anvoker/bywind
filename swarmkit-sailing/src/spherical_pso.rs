//! Tangent-frame PSO velocity update for `PathXY<N>` particles.
//!
//! Replaces `swarmkit::topology::gbest::GBestMover<PathXY<N>>` for the *space*
//! search in [`crate::search`] so swarm exploration is unbiased on the
//! sphere. The generic `GBestMover` treats `(Δlon, Δlat)` deltas as a
//! Cartesian vector and adds them directly to position; that compresses
//! east-west motion at high latitudes (1° lon at lat=60° is half the
//! ground metres of 1° lon at the equator). This mover converts each
//! per-waypoint delta into local east-north tangent-frame metres at
//! that waypoint, runs PSO arithmetic in metres, then converts back to
//! `(Δlon, Δlat)` for the position update — so 1 m of east-tangent
//! velocity always means 1 m of ground motion east, anywhere on the
//! sphere.
//!
//! Persistent `vel` semantics shift accordingly: each waypoint's velocity
//! is stored in tangent-frame metres-per-iteration, not degrees. For PSO
//! step sizes (damped by `inertia` ≈ 0.2), the parallel-transport
//! approximation — treating last step's tangent vector as still valid in
//! this step's tangent frame — is well within the noise floor.

use crate::spherical::{
    LatLon, POLE_LATITUDE_LIMIT_DEG, TangentMetres, delta_to_tangent_metres,
    tangent_metres_to_delta, wrap_lon_deg,
};
use crate::units::PathXY;
use rand::distr::StandardUniform;
use rand::{Rng, RngExt as _};
use swarmkit::{Best, Contextful, PSOCoeffs, ParticleMover, ParticleRefMut};

/// PSO mover that does tangent-frame velocity updates per waypoint. See
/// the module docstring.
///
/// The `TContext` parameter mirrors the generic [`GBestMover`]'s context
/// pass-through; this mover doesn't actually use it, but the searcher
/// chain needs the type to line up with the rest of the pipeline.
///
/// [`GBestMover`]: swarmkit::GBestMover
pub struct SphericalPSOMover<const N: usize, TContext = ()> {
    config: PSOCoeffs,
    // `fn() -> TContext` keeps the struct `Send + Sync` regardless of
    // `TContext`'s thread bounds — `ParticleMover` requires `Sync`, and
    // the context isn't actually held on the struct.
    _ctx: std::marker::PhantomData<fn() -> TContext>,
}

impl<const N: usize, TContext> SphericalPSOMover<N, TContext> {
    pub fn new(config: PSOCoeffs) -> Self {
        Self {
            config,
            _ctx: std::marker::PhantomData,
        }
    }
}

impl<const N: usize, TContext> Contextful for SphericalPSOMover<N, TContext>
where
    TContext: Copy,
{
    type TContext = TContext;
}

impl<const N: usize, TContext> ParticleMover for SphericalPSOMover<N, TContext>
where
    TContext: Copy,
{
    type TUnit = PathXY<N>;
    type TCommon = Best<PathXY<N>>;

    fn update<R: Rng>(
        &self,
        common: &Self::TCommon,
        rng: &mut R,
        _idx: usize,
        p: &mut ParticleRefMut<'_, Self::TUnit>,
    ) {
        // Same `r1, r2 ∈ [0, 1)` shared across all waypoints — matches
        // GBestMover's per-particle sampling so the swarm dynamics stay
        // comparable.
        let r1: f64 = rng.sample::<f64, StandardUniform>(StandardUniform);
        let r2: f64 = rng.sample::<f64, StandardUniform>(StandardUniform);
        for i in 0..N {
            let pos = p.pos.lat_lon(i);
            let pbest = p.best_pos.lat_lon(i);
            let social = common.best_pos.lat_lon(i);
            let vel = TangentMetres::new(p.vel.0[i], p.vel.1[i]);

            let (new_vel, new_pos) =
                spherical_waypoint_update(pos, pbest, social, vel, &self.config, r1, r2);

            p.vel.0[i] = new_vel.east;
            p.vel.1[i] = new_vel.north;
            p.pos.set_lat_lon(i, new_pos);
        }
    }
}

/// Per-waypoint tangent-frame PSO velocity + position update. The kernel
/// shared by all spherical-PSO movers: gbest passes the swarm-wide best as
/// `social`; lbest passes the best neighbour's pbest; niched passes the
/// niche's best. The mathematics is identical — only the social attractor
/// differs.
///
/// `r1` / `r2` are sampled once per particle (not per waypoint) by the
/// caller so the swarm dynamics stay comparable to `GBestMover`'s
/// per-particle sampling.
pub fn spherical_waypoint_update(
    pos: LatLon,
    pbest: LatLon,
    social: LatLon,
    vel: TangentMetres,
    config: &PSOCoeffs,
    r1: f64,
    r2: f64,
) -> (TangentMetres, LatLon) {
    // Convert pbest/social deltas to tangent-frame metres at the current
    // position. `delta_to` picks the antimeridian short-way every time;
    // raw `(b - a)` would steer particles the long way round near 180°.
    let cog_metres = delta_to_tangent_metres(pos, pos.delta_to(pbest));
    let soc_metres = delta_to_tangent_metres(pos, pos.delta_to(social));

    let new_vel = vel * config.inertia
        + cog_metres * (config.cognitive_coeff * r1)
        + soc_metres * (config.social_coeff * r2);

    // Position update in `(lon, lat)` via the inverse tangent conversion.
    // Longitude wraps and latitude is clamped strictly inside
    // `POLE_LATITUDE_LIMIT_DEG` so the next iteration's
    // `tangent_metres_to_delta` doesn't see a pole singularity. Bbox
    // clamping happens in `SailingPathBoundary` after this mover runs.
    let dlonlat = tangent_metres_to_delta(pos, new_vel);
    let new_pos = LatLon::new(
        wrap_lon_deg(pos.lon + dlonlat.lon),
        (pos.lat + dlonlat.lat).clamp(-POLE_LATITUDE_LIMIT_DEG, POLE_LATITUDE_LIMIT_DEG),
    );

    (new_vel, new_pos)
}
