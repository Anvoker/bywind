//! Spherical-Earth math primitives used by the sailing search.
//!
//! Three named-field types carry the unit discipline through every API
//! boundary:
//!
//! - [`LatLon`] — an absolute `(lon°, lat°)` position. Cannot be
//!   subtracted from another `LatLon` directly; use [`signed_lon_delta`]
//!   for the lon component (wrap-aware) and a plain `b.lat - a.lat` for
//!   the lat component when an explicit delta is needed.
//! - [`LatLonDelta`] — a small `(Δlon°, Δlat°)` delta at an implicit
//!   reference position. Adding to a `LatLon` produces a new `LatLon`.
//!   Crossing the antimeridian or a pole is the caller's responsibility:
//!   use [`crate::route_bounds::RouteBounds::constrain_xy`] downstream.
//! - [`TangentMetres`] — a 2D vector in tangent-frame metres (east, north)
//!   *at* an implicit reference position. Standard 2D Cartesian: add,
//!   scale, take norm, etc. Re-using a `TangentMetres` at a different
//!   reference position silently mis-scales — re-derive at each point.
//! - [`Wind`] — a 2D wind velocity in metres per second, in the same
//!   east-north tangent frame as `TangentMetres` but with units of m/s.
//!   Kept distinct from `TangentMetres` so the unit (m vs m/s) is
//!   visible at every API boundary.
//!
//! Bearings are stored as `f64` in **radians**, compass convention:
//! `0 = north`, `π/2 = east`, clockwise. Cheaper for trig (no per-call
//! degree-conversion), at the cost of bearings not being human-readable
//! when printed.
//!
//! Distances are real ground metres on the sphere.
//!
//! Pole behaviour: bearing-producing functions return `Option` and yield
//! `None` for input positions whose latitude exceeds
//! [`POLE_LATITUDE_LIMIT_DEG`]. Caller decides how to handle. PSO position
//! clamping in this crate keeps particles strictly inside the limit, so the
//! `None` branch is genuinely a programmer error inside the search loop.

use std::ops::{Add, AddAssign, Mul, Neg, Sub, SubAssign};

/// Mean Earth radius (m), the spherical-Earth approximation used throughout
/// the search. Single source of truth so haversine / bearing / step
/// formulas all agree.
pub const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Ground metres per degree of latitude (constant on a sphere) and per
/// degree of longitude *at the equator*.
///
/// Off-equator longitudes scale by `cos(lat)`. Derived from
/// [`EARTH_RADIUS_M`] so both constants stay consistent.
pub const METRES_PER_DEGREE: f64 = std::f64::consts::PI * EARTH_RADIUS_M / 180.0;

/// Latitude limit (degrees) beyond which bearings and tangent-frame
/// conversions are considered undefined. Slightly under 90° so trig stays
/// numerically stable and PSO clamping has somewhere safe to land.
pub const POLE_LATITUDE_LIMIT_DEG: f64 = 89.99;

// =============================================================================
// Newtypes
// =============================================================================

/// Absolute position on the sphere as `(lon°, lat°)`.
///
/// `lon ∈ (−180, 180]`,
/// `lat ∈ [−POLE_LATITUDE_LIMIT_DEG, POLE_LATITUDE_LIMIT_DEG]` when
/// constrained by [`crate::route_bounds::RouteBounds::constrain_xy`].
///
/// Not Cartesian: subtracting two `LatLon`s isn't meaningful (longitude
/// wraps, latitude doesn't), so `Sub` is deliberately not implemented.
/// To get a delta between positions, use [`signed_lon_delta`] for the
/// lon component.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct LatLon {
    pub lon: f64,
    pub lat: f64,
}

impl LatLon {
    pub const fn new(lon: f64, lat: f64) -> Self {
        Self { lon, lat }
    }

    /// Wrap-aware delta from `self` to `other`. Longitude takes the
    /// short way across the antimeridian via [`signed_lon_delta`]; lat
    /// is a plain subtraction (latitude doesn't wrap). The inverse of
    /// `Add<LatLonDelta> for LatLon`: `a + a.delta_to(b)` recovers `b`
    /// up to wrap canonicalisation.
    ///
    /// `Sub<LatLon> for LatLon` is deliberately *not* implemented so
    /// that callers can't accidentally compute a wrap-naive delta; use
    /// this method when you want a `LatLonDelta` between two positions.
    pub fn delta_to(self, other: Self) -> LatLonDelta {
        LatLonDelta::new(signed_lon_delta(self.lon, other.lon), other.lat - self.lat)
    }

    /// Project `self` forward by a tangent-frame metres vector
    /// expressed at `self`'s reference frame. Sugar for
    /// `self + tangent_metres_to_delta(self, tangent)` — saves the
    /// caller from writing the position twice. Out-of-bounds /
    /// pole-singularity handling is the caller's responsibility (apply
    /// `RouteBounds::constrain_xy` afterwards if needed).
    pub fn offset_by(self, tangent: TangentMetres) -> Self {
        self + tangent_metres_to_delta(self, tangent)
    }
}

impl From<(f64, f64)> for LatLon {
    /// Tuple constructor: `(lon, lat)`.
    fn from((lon, lat): (f64, f64)) -> Self {
        Self::new(lon, lat)
    }
}

impl Add<LatLonDelta> for LatLon {
    type Output = Self;
    fn add(self, rhs: LatLonDelta) -> Self::Output {
        Self::new(self.lon + rhs.lon, self.lat + rhs.lat)
    }
}

impl AddAssign<LatLonDelta> for LatLon {
    fn add_assign(&mut self, rhs: LatLonDelta) {
        self.lon += rhs.lon;
        self.lat += rhs.lat;
    }
}

/// Small `(Δlon°, Δlat°)` delta at an implicit reference position.
/// Result of [`tangent_metres_to_delta`]; expected to be added to a
/// nearby `LatLon`.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct LatLonDelta {
    pub lon: f64,
    pub lat: f64,
}

impl LatLonDelta {
    pub const fn new(lon: f64, lat: f64) -> Self {
        Self { lon, lat }
    }
    pub const fn zero() -> Self {
        Self::new(0.0, 0.0)
    }
}

impl From<(f64, f64)> for LatLonDelta {
    fn from((lon, lat): (f64, f64)) -> Self {
        Self::new(lon, lat)
    }
}

impl Add for LatLonDelta {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.lon + rhs.lon, self.lat + rhs.lat)
    }
}

impl Sub for LatLonDelta {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.lon - rhs.lon, self.lat - rhs.lat)
    }
}

impl Mul<f64> for LatLonDelta {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(self.lon * rhs, self.lat * rhs)
    }
}

impl Neg for LatLonDelta {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self::new(-self.lon, -self.lat)
    }
}

/// 2D vector in tangent-frame metres at an implicit reference position.
///
/// `east` is +x, `north` is +y. Standard Cartesian: add, scale, take
/// norm. Re-using a `TangentMetres` at a different reference position
/// silently mis-scales because tangent frames vary with latitude — the
/// type does NOT track its own reference, so the discipline must come
/// from the surrounding code.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct TangentMetres {
    pub east: f64,
    pub north: f64,
}

impl TangentMetres {
    pub const fn new(east: f64, north: f64) -> Self {
        Self { east, north }
    }
    pub const fn zero() -> Self {
        Self::new(0.0, 0.0)
    }

    /// Magnitude in real ground metres.
    pub fn norm(self) -> f64 {
        self.east.hypot(self.north)
    }

    pub fn norm_squared(self) -> f64 {
        self.east * self.east + self.north * self.north
    }

    /// Unit vector in the same direction. Returns `Self::zero()` for a
    /// zero input rather than panicking with a NaN, matching how the
    /// callers (gradient probes, perpendicular sampling) want to handle
    /// degenerate cases.
    pub fn normalize(self) -> Self {
        let n = self.norm();
        if n > 0.0 {
            Self::new(self.east / n, self.north / n)
        } else {
            Self::zero()
        }
    }
}

impl From<(f64, f64)> for TangentMetres {
    /// Tuple constructor: `(east_m, north_m)`.
    fn from((east, north): (f64, f64)) -> Self {
        Self::new(east, north)
    }
}

impl Add for TangentMetres {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.east + rhs.east, self.north + rhs.north)
    }
}

impl AddAssign for TangentMetres {
    fn add_assign(&mut self, rhs: Self) {
        self.east += rhs.east;
        self.north += rhs.north;
    }
}

impl Sub for TangentMetres {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.east - rhs.east, self.north - rhs.north)
    }
}

impl SubAssign for TangentMetres {
    fn sub_assign(&mut self, rhs: Self) {
        self.east -= rhs.east;
        self.north -= rhs.north;
    }
}

impl Mul<f64> for TangentMetres {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(self.east * rhs, self.north * rhs)
    }
}

impl Mul<TangentMetres> for f64 {
    type Output = TangentMetres;
    fn mul(self, rhs: TangentMetres) -> TangentMetres {
        rhs * self
    }
}

impl Neg for TangentMetres {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self::new(-self.east, -self.north)
    }
}

// =============================================================================
// Spherical-Earth primitives
// =============================================================================

/// Great-circle distance in metres between two `(lon°, lat°)` points.
pub fn haversine(a: LatLon, b: LatLon) -> f64 {
    let lat_a = a.lat.to_radians();
    let lat_b = b.lat.to_radians();
    let dlat = (b.lat - a.lat).to_radians();
    let dlon = signed_lon_delta(a.lon, b.lon).to_radians();

    let sin_dlat_half = (dlat * 0.5).sin();
    let sin_dlon_half = (dlon * 0.5).sin();
    let h =
        sin_dlat_half * sin_dlat_half + lat_a.cos() * lat_b.cos() * sin_dlon_half * sin_dlon_half;
    // `h` clamps to [0, 1] in exact arithmetic; rounding can push it slightly
    // out, so saturate before `asin` to avoid NaN on antipodal-ish cases.
    2.0 * EARTH_RADIUS_M * h.clamp(0.0, 1.0).sqrt().asin()
}

/// Initial bearing in radians (compass: `0 = N`, `π/2 = E`, clockwise) from
/// `a` toward `b`. Returns `None` when `a` is at a pole.
pub fn initial_bearing(a: LatLon, b: LatLon) -> Option<f64> {
    if a.lat.abs() >= POLE_LATITUDE_LIMIT_DEG {
        return None;
    }
    let lat_a = a.lat.to_radians();
    let lat_b = b.lat.to_radians();
    let dlon = signed_lon_delta(a.lon, b.lon).to_radians();
    let y = dlon.sin() * lat_b.cos();
    let x = lat_a.cos() * lat_b.sin() - lat_a.sin() * lat_b.cos() * dlon.cos();
    Some(y.atan2(x))
}

/// Advance `start` by `distance_m` metres along `bearing_rad` and
/// return the destination `(lon°, lat°)`.
///
/// `bearing_rad` is in compass convention (radians). Longitude is
/// wrapped into `(−180, 180]`. Returns `None` when `start` is at a pole.
pub fn destination_point(start: LatLon, distance_m: f64, bearing_rad: f64) -> Option<LatLon> {
    if start.lat.abs() >= POLE_LATITUDE_LIMIT_DEG {
        return None;
    }
    let angular = distance_m / EARTH_RADIUS_M;
    let lat_start = start.lat.to_radians();
    let lon_start = start.lon.to_radians();

    let sin_lat_dest =
        lat_start.sin() * angular.cos() + lat_start.cos() * angular.sin() * bearing_rad.cos();
    let lat_dest = sin_lat_dest.clamp(-1.0, 1.0).asin();
    let lon_dest = lon_start
        + (bearing_rad.sin() * angular.sin() * lat_start.cos())
            .atan2(angular.cos() - lat_start.sin() * sin_lat_dest);

    Some(LatLon::new(
        wrap_lon_deg(lon_dest.to_degrees()),
        lat_dest.to_degrees(),
    ))
}

/// Signed shortest delta between two longitudes (degrees), in
/// `(−180°, 180°]`. Use this everywhere instead of raw `b - a` so
/// antimeridian crossings pick the short way round.
///
/// Examples:
/// `signed_lon_delta(179, -179) == 2.0`,
/// `signed_lon_delta(-179, 179) == -2.0`,
/// `signed_lon_delta(0, 90) == 90.0`.
#[expect(
    clippy::float_cmp,
    reason = "`rem_euclid(360.0) - 180.0` produces an exactly-representable \
              `-180.0` at the antipodal boundary; bit equality is the intended \
              canonicalisation check."
)]
pub fn signed_lon_delta(a_lon: f64, b_lon: f64) -> f64 {
    let raw = ((b_lon - a_lon + 540.0).rem_euclid(360.0)) - 180.0;
    // Canonicalise the antipodal boundary to +180 to match `wrap_lon_deg`.
    // Either sign is correct mathematically — both describe a half-rotation
    // — but having one canonical answer keeps callers simpler.
    if raw == -180.0 { 180.0 } else { raw }
}

/// Wrap a longitude in degrees into `(−180, 180]`.
#[expect(
    clippy::float_cmp,
    reason = "`rem_euclid(360.0) - 180.0` produces an exactly-representable \
              `-180.0` at the antimeridian; bit equality is the intended \
              canonicalisation check."
)]
pub fn wrap_lon_deg(lon: f64) -> f64 {
    let wrapped = ((lon + 540.0).rem_euclid(360.0)) - 180.0;
    // `rem_euclid` returns `[0, 360)`, so `wrapped` lands in `[-180, 180)`.
    // Re-canonicalise the boundary so −180 maps to +180 (keeps the range
    // half-open on the same side as `signed_lon_delta`'s output).
    if wrapped == -180.0 { 180.0 } else { wrapped }
}

/// `f32` convenience wrapper around [`wrap_lon_deg`] for callers like
/// the view layer that work in `f32` throughout.
///
/// The conversion adds a tiny rounding error but keeps the canonical
/// `(−180, 180]` half-open range.
pub fn wrap_lon_deg_f32(lon: f32) -> f32 {
    wrap_lon_deg(f64::from(lon)) as f32
}

/// Convert a `(Δlon°, Δlat°)` delta at position `pos` into local
/// east-north tangent-frame metres.
///
/// Used at PSO velocity-update sites so the swarm explores in real
/// ground metres rather than in degrees (which compresses east-west
/// motion at high latitudes).
///
/// `delta.lon` is *signed shortest* longitude delta — caller is
/// responsible for using [`signed_lon_delta`] when computing it.
pub fn delta_to_tangent_metres(pos: LatLon, delta: LatLonDelta) -> TangentMetres {
    let cos_lat = pos.lat.to_radians().cos();
    TangentMetres::new(
        delta.lon * cos_lat * METRES_PER_DEGREE,
        delta.lat * METRES_PER_DEGREE,
    )
}

/// Inverse of [`delta_to_tangent_metres`]: tangent-frame metres at `pos`
/// → `(Δlon°, Δlat°)`.
///
/// Saturates `cos(lat)` away from zero at the pole rather than producing
/// NaN; PSO clamps positions to [`POLE_LATITUDE_LIMIT_DEG`] so this guard
/// only matters if a caller misuses the function.
pub fn tangent_metres_to_delta(pos: LatLon, tangent: TangentMetres) -> LatLonDelta {
    let cos_lat = pos.lat.to_radians().cos().max(1e-9);
    LatLonDelta::new(
        tangent.east / (cos_lat * METRES_PER_DEGREE),
        tangent.north / METRES_PER_DEGREE,
    )
}

// =============================================================================
// Bbox
// =============================================================================

/// Axis-aligned bounding box on the sphere in `(lon°, lat°)` degrees.
///
/// Longitude convention matches [`LatLon::lon`]: canonical (−180, 180].
/// When `lon_min > lon_max` the bbox **wraps the antimeridian** — the
/// valid lon range is `[lon_min, 180] ∪ [−180, lon_max]`. This encoding
/// is load-bearing: callers that want a monotonic eastern edge for a
/// bake-grid axis (or similar) use [`Self::lon_max_unwrapped`] rather
/// than reading `lon_max` directly.
///
/// `lon_min == lon_max` is degenerate (zero lon span); see
/// [`Self::is_non_degenerate`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LonLatBbox {
    /// Western lon edge (canonical, in (−180, 180]).
    pub lon_min: f64,
    /// Eastern lon edge (canonical, in (−180, 180]). When less than
    /// `lon_min` the bbox wraps the antimeridian.
    pub lon_max: f64,
    pub lat_min: f64,
    pub lat_max: f64,
}

impl LonLatBbox {
    pub const fn new(lon_min: f64, lon_max: f64, lat_min: f64, lat_max: f64) -> Self {
        Self {
            lon_min,
            lon_max,
            lat_min,
            lat_max,
        }
    }

    /// True iff this bbox crosses the antimeridian.
    pub fn wraps_antimeridian(self) -> bool {
        self.lon_min > self.lon_max
    }

    /// Eastern lon edge as a *monotonic* longitude — canonical when the
    /// bbox doesn't wrap, `lon_max + 360` when it does. Used by
    /// downstream grid construction (e.g. `BakeBounds`) so the lon axis
    /// stays monotonic without asking [`LatLon`] to carry an
    /// out-of-canonical-range value.
    pub fn lon_max_unwrapped(self) -> f64 {
        if self.wraps_antimeridian() {
            self.lon_max + 360.0
        } else {
            self.lon_max
        }
    }

    /// Lon span in degrees (always non-negative).
    pub fn lon_extent(self) -> f64 {
        (self.lon_max_unwrapped() - self.lon_min).max(0.0)
    }

    /// Lat span in degrees (always non-negative).
    pub fn lat_extent(self) -> f64 {
        (self.lat_max - self.lat_min).max(0.0)
    }

    /// True iff the bbox has positive area. Wrap-encoded bboxes are
    /// non-degenerate as long as `lon_min != lon_max` — they cover
    /// `[lon_min, 180] ∪ [−180, lon_max]`.
    #[expect(
        clippy::float_cmp,
        reason = "bbox endpoints are stored values, not computed; exact \
                  inequality is the precise zero-extent check (including \
                  the wrap-encoded case where `lon_min > lon_max`)."
    )]
    pub fn is_non_degenerate(self) -> bool {
        self.lat_max > self.lat_min && self.lon_min != self.lon_max
    }

    /// Great-circle diagonal of the bbox, in real ground metres.
    /// Wrap-aware via [`signed_lon_delta`] inside [`haversine`], so a
    /// wrap-encoded bbox `[170°, −170°]` returns the diagonal of the
    /// 20°-wide region rather than the 340°-wide complement.
    pub fn diagonal_m(self) -> f64 {
        haversine(
            LatLon::new(self.lon_min, self.lat_min),
            LatLon::new(self.lon_max, self.lat_max),
        )
    }

    /// Wrap-aware lon clamp + pole-safe lat clamp into the bbox.
    ///
    /// Lon: when the bbox wraps (`lon_min > lon_max`), points already
    /// inside `[lon_min, 180] ∪ [−180, lon_max]` are passed through;
    /// otherwise the closer of the two boundaries (along the short way
    /// round) wins. Non-wrap bboxes use a plain interval clamp.
    ///
    /// Lat: clamped to `[lat_min, lat_max]`, then further restricted to
    /// strictly inside `±POLE_LATITUDE_LIMIT_DEG` so downstream
    /// [`tangent_metres_to_delta`] calls don't see a pole singularity.
    pub fn clamp(self, p: LatLon) -> LatLon {
        LatLon::new(
            clamp_lon_wrap_inline(p.lon, self.lon_min, self.lon_max),
            f64::clamp(p.lat, self.lat_min, self.lat_max)
                .clamp(-POLE_LATITUDE_LIMIT_DEG, POLE_LATITUDE_LIMIT_DEG),
        )
    }

    /// True iff `p` lies inside the bbox. Wrap-aware on lon: when
    /// `lon_max < lon_min` the bbox crosses the antimeridian and the
    /// admissible lon range is `[lon_min, 180] ∪ [−180, lon_max]`.
    /// Lat range is the plain `[lat_min, lat_max]`. Boundary points
    /// count as inside.
    pub fn contains(self, p: LatLon) -> bool {
        if p.lat < self.lat_min || p.lat > self.lat_max {
            return false;
        }
        if self.wraps_antimeridian() {
            p.lon >= self.lon_min || p.lon <= self.lon_max
        } else {
            p.lon >= self.lon_min && p.lon <= self.lon_max
        }
    }
}

/// Wind velocity in the local east-north tangent frame at an implicit
/// reference position, in metres per second.
///
/// Same axis convention as [`TangentMetres`] (east is +x, north is +y)
/// but the unit (m/s vs m) is what distinguishes the two; PSO velocities
/// and ground-metre kicks live in `TangentMetres`, wind samples live here.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Wind {
    pub east_mps: f64,
    pub north_mps: f64,
}

impl Wind {
    pub const fn new(east_mps: f64, north_mps: f64) -> Self {
        Self {
            east_mps,
            north_mps,
        }
    }

    pub const fn zero() -> Self {
        Self::new(0.0, 0.0)
    }

    /// True wind speed in m/s — the magnitude of the wind vector.
    pub fn speed(self) -> f64 {
        self.east_mps.hypot(self.north_mps)
    }
}

/// Great-circle segment that the boat physics integrates along.
///
/// Bundles the four arguments that consistently travel together through
/// the [`crate::Sailboat`] trait methods and the dynamics helpers around
/// them: `(origin, destination)` defines the great-circle endpoints;
/// `origin_time` is the absolute departure wall-clock used when sampling
/// time-varying wind; `step_distance_max` caps the substep size (real
/// ground metres) of the integrator. The struct derives `Copy` so it
/// passes by value cheaply at every call site.
#[derive(Copy, Clone, Debug)]
pub struct Segment {
    pub origin: LatLon,
    pub destination: LatLon,
    pub origin_time: f64,
    pub step_distance_max: f64,
}

/// Implementation moved here from `route_bounds::clamp_lon_wrap` so
/// `LonLatBbox::clamp` doesn't depend on a sibling module's pub(crate)
/// helper. Same algorithm: standard interval clamp for non-wrap bboxes,
/// short-way snap-to-nearest-edge for wrap bboxes.
fn clamp_lon_wrap_inline(lon: f64, lon_min: f64, lon_max: f64) -> f64 {
    if lon_min <= lon_max {
        return lon.clamp(lon_min, lon_max);
    }
    if lon >= lon_min || lon <= lon_max {
        return lon;
    }
    let d_to_min = signed_lon_delta(lon, lon_min).abs();
    let d_to_max = signed_lon_delta(lon, lon_max).abs();
    if d_to_min <= d_to_max {
        lon_min
    } else {
        lon_max
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "tests rely on bit-exact comparisons of constant or stored f32/f64 values."
    )]
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    fn approx_ll(a: LatLon, b: LatLon, tol: f64) -> bool {
        approx(a.lon, b.lon, tol) && approx(a.lat, b.lat, tol)
    }

    fn approx_ld(a: LatLonDelta, b: LatLonDelta, tol: f64) -> bool {
        approx(a.lon, b.lon, tol) && approx(a.lat, b.lat, tol)
    }

    #[test]
    fn haversine_zero_for_identical_points() {
        let p = LatLon::new(12.5, 45.0);
        assert!(approx(haversine(p, p), 0.0, 1e-6));
    }

    #[test]
    fn haversine_symmetric() {
        let a = LatLon::new(0.0, 0.0);
        let b = LatLon::new(10.0, 20.0);
        assert!(approx(haversine(a, b), haversine(b, a), 1e-6));
    }

    #[test]
    fn haversine_one_degree_of_latitude_is_metres_per_degree() {
        // One degree of latitude is ~111 km on a sphere, regardless of
        // longitude or starting latitude.
        let a = LatLon::new(0.0, 0.0);
        let b = LatLon::new(0.0, 1.0);
        assert!(approx(haversine(a, b), METRES_PER_DEGREE, 1.0));
    }

    #[test]
    fn haversine_one_degree_of_longitude_scales_with_cos_lat() {
        let a = LatLon::new(0.0, 60.0);
        let b = LatLon::new(1.0, 60.0);
        // At lat=60°, 1° of longitude is half what it is at the equator.
        let expected = 0.5 * METRES_PER_DEGREE;
        assert!(approx(haversine(a, b), expected, 5.0));
    }

    #[test]
    fn haversine_handles_antimeridian() {
        // Two points 2° apart across 180°.
        let a = LatLon::new(179.0, 0.0);
        let b = LatLon::new(-179.0, 0.0);
        let expected = 2.0 * METRES_PER_DEGREE;
        assert!(approx(haversine(a, b), expected, 1.0));
    }

    #[test]
    fn initial_bearing_due_north_is_zero() {
        let a = LatLon::new(0.0, 0.0);
        let b = LatLon::new(0.0, 10.0);
        let bearing = initial_bearing(a, b).unwrap();
        assert!(approx(bearing, 0.0, 1e-9));
    }

    #[test]
    fn initial_bearing_due_east_is_pi_over_two() {
        let a = LatLon::new(0.0, 0.0);
        let b = LatLon::new(10.0, 0.0);
        let bearing = initial_bearing(a, b).unwrap();
        assert!(approx(bearing, std::f64::consts::FRAC_PI_2, 1e-9));
    }

    #[test]
    fn initial_bearing_due_south_is_pi() {
        let a = LatLon::new(0.0, 10.0);
        let b = LatLon::new(0.0, 0.0);
        // atan2 returns ±π for due south; either is correct.
        let bearing = initial_bearing(a, b).unwrap();
        assert!(approx(bearing.abs(), std::f64::consts::PI, 1e-9));
    }

    #[test]
    fn initial_bearing_at_pole_is_none() {
        let pole = LatLon::new(0.0, 90.0);
        let other = LatLon::new(50.0, 80.0);
        assert!(initial_bearing(pole, other).is_none());
    }

    #[test]
    fn destination_point_round_trips_with_haversine_and_bearing() {
        let cases = [
            (LatLon::new(0.0, 0.0), LatLon::new(10.0, 5.0)),
            (LatLon::new(45.0, 30.0), LatLon::new(50.0, 35.0)),
            (LatLon::new(-100.0, -20.0), LatLon::new(-80.0, 10.0)),
            // Antimeridian crossing.
            (LatLon::new(170.0, 0.0), LatLon::new(-170.0, 5.0)),
        ];
        for (a, b) in cases {
            let dist = haversine(a, b);
            let bearing = initial_bearing(a, b).unwrap();
            let arrived = destination_point(a, dist, bearing).unwrap();
            assert!(
                approx_ll(arrived, b, 1e-6),
                "from {a:?} → {b:?}: arrived {arrived:?} (dist={dist}, bearing={bearing})",
            );
        }
    }

    #[test]
    fn destination_point_at_pole_is_none() {
        let pole = LatLon::new(0.0, 90.0);
        assert!(destination_point(pole, 1000.0, 0.0).is_none());
    }

    #[test]
    fn signed_lon_delta_handles_antimeridian() {
        assert!(approx(signed_lon_delta(179.0, -179.0), 2.0, 1e-12));
        assert!(approx(signed_lon_delta(-179.0, 179.0), -2.0, 1e-12));
        assert!(approx(signed_lon_delta(0.0, 90.0), 90.0, 1e-12));
        assert!(approx(signed_lon_delta(90.0, 0.0), -90.0, 1e-12));
        assert!(approx(signed_lon_delta(0.0, 180.0), 180.0, 1e-12));
        assert!(approx(signed_lon_delta(0.0, -180.0), 180.0, 1e-12));
    }

    #[test]
    fn wrap_lon_deg_canonicalises_to_half_open_range() {
        assert!(approx(wrap_lon_deg(0.0), 0.0, 1e-12));
        assert!(approx(wrap_lon_deg(180.0), 180.0, 1e-12));
        assert!(approx(wrap_lon_deg(-180.0), 180.0, 1e-12));
        assert!(approx(wrap_lon_deg(181.0), -179.0, 1e-12));
        assert!(approx(wrap_lon_deg(-181.0), 179.0, 1e-12));
        assert!(approx(wrap_lon_deg(540.0), 180.0, 1e-12));
    }

    #[test]
    fn tangent_frame_round_trips_at_various_latitudes() {
        for &lat in &[0.0_f64, 30.0, 45.0, 60.0, 85.0] {
            let pos = LatLon::new(10.0, lat);
            let original = LatLonDelta::new(0.5, -0.25);
            let metres = delta_to_tangent_metres(pos, original);
            let recovered = tangent_metres_to_delta(pos, metres);
            assert!(
                approx_ld(original, recovered, 1e-12),
                "lat={lat}: {original:?} → {metres:?} → {recovered:?}",
            );
        }
    }

    #[test]
    fn lat_lon_delta_to_handles_antimeridian() {
        // `signed_lon_delta`'s short-way pick must come through on the
        // method form too. Going east from 179° to −179° is +2° lon;
        // lat is a plain subtraction.
        let a = LatLon::new(179.0, 10.0);
        let b = LatLon::new(-179.0, 12.0);
        let d = a.delta_to(b);
        assert!(approx(d.lon, 2.0, 1e-12));
        assert!(approx(d.lat, 2.0, 1e-12));
        // Round-trip: a + a.delta_to(b) recovers b's lat exactly and
        // its lon up to wrap canonicalisation.
        let recovered = a + d;
        assert!(approx(recovered.lat, b.lat, 1e-12));
        assert!(approx(wrap_lon_deg(recovered.lon), b.lon, 1e-12));
    }

    #[test]
    fn lat_lon_offset_by_round_trips_with_haversine_at_short_range() {
        // Step ~10 km east at lat=45°: offset_by → new LatLon whose
        // haversine distance from the start matches the requested
        // tangent magnitude (within numerical noise on a sphere).
        let pos = LatLon::new(0.0, 45.0);
        let step = TangentMetres::new(10_000.0, 0.0);
        let arrived = pos.offset_by(step);
        let dist = haversine(pos, arrived);
        assert!(approx(dist, step.norm(), 1.0));
    }

    #[test]
    fn tangent_frame_compresses_lon_at_high_latitude() {
        // A 1° eastward motion at lat=60° should become half as many
        // ground metres east as the same 1° motion at the equator.
        let at_equator = delta_to_tangent_metres(LatLon::new(0.0, 0.0), LatLonDelta::new(1.0, 0.0));
        let at_60 = delta_to_tangent_metres(LatLon::new(0.0, 60.0), LatLonDelta::new(1.0, 0.0));
        let ratio = at_60.east / at_equator.east;
        // cos(60°) = 0.5
        assert!((ratio - 0.5).abs() < 1e-9);
        // North component is unaffected.
        assert!(at_equator.north.abs() < 1e-9);
        assert!(at_60.north.abs() < 1e-9);
    }

    // -------- LonLatBbox --------

    #[test]
    fn bbox_non_wrap_basic_predicates() {
        let b = LonLatBbox::new(-10.0, 30.0, -5.0, 5.0);
        assert!(!b.wraps_antimeridian());
        assert!(b.is_non_degenerate());
        assert_eq!(b.lon_max_unwrapped(), 30.0);
        assert!((b.lon_extent() - 40.0).abs() < 1e-12);
        assert!((b.lat_extent() - 10.0).abs() < 1e-12);
    }

    #[test]
    fn bbox_wrapping_extent_reaches_across_antimeridian() {
        // Tokyo→SF-style wrap: lon_min = 139, lon_max = -122 covers
        // [139, 180] ∪ [-180, -122]. Total lon span = 99°.
        let b = LonLatBbox::new(139.0, -122.0, -10.0, 10.0);
        assert!(b.wraps_antimeridian());
        assert!(b.is_non_degenerate());
        assert!((b.lon_max_unwrapped() - 238.0).abs() < 1e-12);
        assert!((b.lon_extent() - 99.0).abs() < 1e-12);
    }

    #[test]
    fn bbox_degenerate_collapses_lon_or_lat() {
        // Equal lon edges: zero lon extent, degenerate.
        let b = LonLatBbox::new(0.0, 0.0, -5.0, 5.0);
        assert!(!b.is_non_degenerate());
        // Equal lat edges: zero lat extent, degenerate.
        let b = LonLatBbox::new(-10.0, 10.0, 5.0, 5.0);
        assert!(!b.is_non_degenerate());
    }

    #[test]
    fn bbox_clamp_non_wrap_uses_interval_clamp() {
        let b = LonLatBbox::new(-10.0, 30.0, -5.0, 5.0);
        // Inside: pass-through.
        let c = b.clamp(LatLon::new(20.0, 0.0));
        assert!(approx_ll(c, LatLon::new(20.0, 0.0), 1e-12));
        // Lon below min: snap to min.
        let c = b.clamp(LatLon::new(-50.0, 0.0));
        assert!(approx(c.lon, -10.0, 1e-12));
        // Lat above max: snap to max.
        let c = b.clamp(LatLon::new(0.0, 80.0));
        assert!(approx(c.lat, 5.0, 1e-12));
    }

    #[test]
    fn bbox_clamp_wrap_passes_through_inside_ranges_and_snaps_outside() {
        let b = LonLatBbox::new(170.0, -170.0, -10.0, 10.0);
        // Inside the eastern half of the wrap.
        let c = b.clamp(LatLon::new(178.0, 0.0));
        assert!(approx(c.lon, 178.0, 1e-12));
        // Inside the western half of the wrap.
        let c = b.clamp(LatLon::new(-175.0, 0.0));
        assert!(approx(c.lon, -175.0, 1e-12));
        // Outside both halves: snap to whichever edge is closer along
        // the short way. lon=160 is 10° from 170 (closer) vs ~30° from
        // -170 going east through 180.
        let c = b.clamp(LatLon::new(160.0, 0.0));
        assert!(approx(c.lon, 170.0, 1e-12));
    }

    #[test]
    fn bbox_clamp_lat_stays_inside_pole_limit() {
        // Even when lat_min is at the pole, clamp must shave back inside
        // POLE_LATITUDE_LIMIT_DEG so tangent-frame conversions stay
        // defined.
        let b = LonLatBbox::new(-10.0, 10.0, -90.0, 90.0);
        let c = b.clamp(LatLon::new(0.0, 89.999));
        assert!(c.lat <= POLE_LATITUDE_LIMIT_DEG);
        let c = b.clamp(LatLon::new(0.0, -89.999));
        assert!(c.lat >= -POLE_LATITUDE_LIMIT_DEG);
    }
}
