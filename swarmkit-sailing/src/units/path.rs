use crate::spherical::LatLon;
use crate::units::Floats;
use derive_more::{Add, Deref, DerefMut, Sub};
use std::f64;
use std::ops::Mul;
use swarmkit::{FieldwiseClamp, ParticleRefFrom, ParticleRefMut, SetTo};

// ============================================================================
// Type Definitions
// ============================================================================

/// Geometry of a single segment of a `Path<N>`. Pure geometry — no time
/// component. Use [`ClockedSegment`] (via [`Path::iter_with_running_clock`])
/// when you need per-segment departure/arrival times.
#[derive(Copy, Clone, Debug)]
pub struct RouteSegment {
    pub origin: LatLon,
    pub destination: LatLon,
}

/// A segment paired with its running-clock readings for a particular
/// traversal.
///
/// `t_depart` and `t_arrive` are absolute times along a forward pass
/// that started at some `start_time` (see
/// [`Path::iter_with_running_clock`]); `segment_time = t_arrive -
/// t_depart` and equals `Path::t[i + 1]`.
///
/// Returned by [`ClockedSegmentIterator`]. Distinct from [`RouteSegment`]
/// because the timing values only make sense in the context of a
/// running-clock traversal — yielding them by default would tempt callers
/// to re-derive the cumulative time themselves and drift out of sync.
#[derive(Copy, Clone, Debug)]
pub struct ClockedSegment {
    pub origin: LatLon,
    pub destination: LatLon,
    pub segment_time: f64,
    pub t_depart: f64,
    pub t_arrive: f64,
}

#[derive(Copy, Clone, Debug, PartialEq, Add, Sub)]
pub struct Path<const N: usize> {
    pub xy: PathXY<N>,
    pub t: Time<N>,
}

#[derive(Copy, Clone, Debug, PartialEq, Add, Sub)]
pub struct PathXY<const N: usize>(pub Floats<N>, pub Floats<N>);

#[derive(Copy, Clone, Debug, PartialEq, Deref, DerefMut, Add, Sub)]
pub struct Time<const N: usize>(pub Floats<N>);

pub struct SegmentIterator<'a, const N: usize> {
    i: usize,
    path: &'a Path<N>,
}

/// Walks the segments of a `Path<N>` while accumulating an absolute
/// clock, yielding [`ClockedSegment`] per pair.
///
/// The clock starts at the `start_time` passed to
/// [`Path::iter_with_running_clock`] and advances by `segment_time`
/// (= `Path::t[i + 1]`) at each step.
pub struct ClockedSegmentIterator<'a, const N: usize> {
    i: usize,
    t_running: f64,
    path: &'a Path<N>,
}

// ============================================================================
// Path<N> Implementations
// ============================================================================

impl<const N: usize> Path<N> {
    pub fn lat_lon(&self, i: usize) -> LatLon {
        self.xy.lat_lon(i)
    }

    pub fn get_segment(&self, i: usize) -> RouteSegment {
        RouteSegment {
            origin: self.lat_lon(i),
            destination: self.lat_lon(i + 1),
        }
    }

    pub fn len(&self) -> usize {
        N
    }

    /// Always `N == 0` — included for clippy's `len_without_is_empty` lint;
    /// every `Path<N>` with `N >= 1` is non-empty by construction.
    pub fn is_empty(&self) -> bool {
        N == 0
    }

    /// Iterates segment geometry only — `(origin, destination)` per pair.
    /// For traversals that need per-segment departure/arrival times, use
    /// [`Path::iter_with_running_clock`] instead.
    pub fn iter_segments(&'_ self) -> SegmentIterator<'_, N> {
        SegmentIterator::new(self)
    }

    /// Iterates segments while advancing an absolute clock starting at
    /// `start_time`. Each yielded [`ClockedSegment`] carries `t_depart`
    /// and `t_arrive` (absolute times at the segment endpoints) plus
    /// `segment_time` (the segment's stored duration, `t[i + 1]`).
    /// Use this whenever a traversal needs cumulative timing — wind-field
    /// integration, fuel accumulation, etc. — instead of hand-rolling the
    /// `dep += segment_time` loop on top of `iter_segments`.
    pub fn iter_with_running_clock(&'_ self, start_time: f64) -> ClockedSegmentIterator<'_, N> {
        ClockedSegmentIterator {
            i: 0,
            t_running: start_time,
            path: self,
        }
    }
}

impl<const N: usize> Default for Path<N> {
    fn default() -> Self {
        Self {
            xy: PathXY(Floats([0.0; N]), Floats([0.0; N])),
            t: Time(Floats([0.0; N])),
        }
    }
}

impl<const N: usize> FieldwiseClamp for Path<N> {
    fn clamp(&self, min: Self, max: Self) -> Self {
        Self {
            xy: self.xy.clamp(min.xy, max.xy),
            t: self.t.clamp(min.t, max.t),
        }
    }
}

impl<const N: usize> Mul<f64> for Path<N> {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        let mut result = Self::default();
        for i in 0..N {
            result.xy.0[i] = self.xy.0[i] * rhs;
            result.t.0[i] = self.t.0[i] * rhs;
        }
        result
    }
}

// ============================================================================
// PathXY<N> Implementations
// ============================================================================

impl<const N: usize> PathXY<N> {
    /// Read waypoint `i` as a [`LatLon`]. Mirror of [`Path::lat_lon`]
    /// for callers that only have the spatial component (movers, fit
    /// calcs, the segment-range cache builder).
    pub fn lat_lon(&self, i: usize) -> LatLon {
        LatLon::new(self.0[i], self.1[i])
    }

    /// Write waypoint `i` from a [`LatLon`]. Counterpart of
    /// [`PathXY::lat_lon`] for the write side of mover loops, where
    /// `p.xy.0[i] = pos.lon; p.xy.1[i] = pos.lat;` would otherwise
    /// repeat per call site.
    pub fn set_lat_lon(&mut self, i: usize, pos: LatLon) {
        self.0[i] = pos.lon;
        self.1[i] = pos.lat;
    }
}

impl<const N: usize> Default for PathXY<N> {
    fn default() -> Self {
        Self(Floats([0.0; N]), Floats([0.0; N]))
    }
}

impl<const N: usize> ParticleRefFrom for PathXY<N> {
    type TSource = Path<N>;

    fn divide_from<'a>(
        source: &'a mut ParticleRefMut<'_, Self::TSource>,
    ) -> ParticleRefMut<'a, Self>
    where
        Self: Copy,
    {
        ParticleRefMut {
            pos: &mut source.pos.xy,
            vel: &mut source.vel.xy,
            fit: source.fit,
            best_pos: &mut source.best_pos.xy,
            best_fit: source.best_fit,
        }
    }
}

impl<const N: usize> From<Path<N>> for PathXY<N> {
    fn from(value: Path<N>) -> Self {
        value.xy
    }
}

impl<const N: usize> FieldwiseClamp for PathXY<N> {
    fn clamp(&self, min: Self, max: Self) -> Self {
        Self(self.0.clamp(min.0, max.0), self.1.clamp(min.1, max.1))
    }
}

impl<const N: usize> Mul<f64> for PathXY<N> {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self(self.0 * rhs, self.1 * rhs)
    }
}

// ============================================================================
// Time<N> Implementations
// ============================================================================

impl<const N: usize> Default for Time<N> {
    fn default() -> Self {
        Self(Floats([0.0; N]))
    }
}

impl<const N: usize> ParticleRefFrom for Time<N> {
    type TSource = Path<N>;

    fn divide_from<'a>(
        source: &'a mut ParticleRefMut<'_, Self::TSource>,
    ) -> ParticleRefMut<'a, Self>
    where
        Self: Copy,
    {
        ParticleRefMut {
            pos: &mut source.pos.t,
            vel: &mut source.vel.t,
            fit: source.fit,
            best_pos: &mut source.best_pos.t,
            best_fit: source.best_fit,
        }
    }
}

impl<const N: usize> SetTo for Time<N> {
    type TTarget = Path<N>;

    fn set_to_ref_mut(&self, target: &mut ParticleRefMut<'_, Self::TTarget>) {
        target.pos.t = *self;
    }
}

impl<const N: usize> From<Path<N>> for Time<N> {
    fn from(value: Path<N>) -> Self {
        value.t
    }
}

impl<const N: usize> FieldwiseClamp for Time<N> {
    fn clamp(&self, min: Self, max: Self) -> Self {
        Self(self.0.clamp(min.0, max.0))
    }
}

impl<const N: usize> Mul<f64> for Time<N> {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self(self.0 * rhs)
    }
}

// ============================================================================
// SegmentIterator Implementation
// ============================================================================

impl<'a, const N: usize> SegmentIterator<'a, N> {
    pub fn new(path: &'a Path<N>) -> Self {
        SegmentIterator { i: 0, path }
    }
}

impl<const N: usize> Iterator for SegmentIterator<'_, N> {
    type Item = RouteSegment;

    fn next(&mut self) -> Option<Self::Item> {
        let i = self.i;
        self.i += 1;
        if i + 1 < self.path.len() {
            Some(self.path.get_segment(i))
        } else {
            None
        }
    }
}

impl<const N: usize> Iterator for ClockedSegmentIterator<'_, N> {
    type Item = ClockedSegment;

    fn next(&mut self) -> Option<Self::Item> {
        let i = self.i;
        if i + 1 >= self.path.len() {
            return None;
        }
        self.i += 1;
        let segment_time = self.path.t[i + 1];
        let t_depart = self.t_running;
        let t_arrive = t_depart + segment_time;
        self.t_running = t_arrive;
        Some(ClockedSegment {
            origin: self.path.lat_lon(i),
            destination: self.path.lat_lon(i + 1),
            segment_time,
            t_depart,
            t_arrive,
        })
    }
}
