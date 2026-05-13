use crate::spherical::{LatLon, LonLatBbox, haversine, signed_lon_delta, wrap_lon_deg};
use crate::units::{Path, PathXY};

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Search domain and endpoint constraints.
///
/// `bbox` carries the antimeridian-wrap convention from [`LonLatBbox`]
/// (`lon_min > lon_max` ⇒ wraps). `step_distance_max` is real ground
/// metres used by the boat integrator to discretise long segments.
#[derive(Copy, Clone, Debug)]
pub struct RouteBounds {
    pub origin: LatLon,
    pub destination: LatLon,
    pub bbox: LonLatBbox,
    pub step_distance_max: f64,
}

/// 1% of the bbox diagonal — ~100 substeps along the longest path.
pub const DEFAULT_STEP_DISTANCE_FRACTION: f64 = 0.01;

impl RouteBounds {
    /// `step_distance_max` defaults to
    /// [`DEFAULT_STEP_DISTANCE_FRACTION`] × diagonal.
    pub fn new(
        origin: impl Into<LatLon>,
        destination: impl Into<LatLon>,
        bbox: LonLatBbox,
    ) -> Self {
        Self::new_with_step_fraction(origin, destination, bbox, DEFAULT_STEP_DISTANCE_FRACTION)
    }

    /// `step_distance_max = fraction × bbox diagonal`. Smaller →
    /// finer wind integration, higher per-segment cost.
    pub fn new_with_step_fraction(
        origin: impl Into<LatLon>,
        destination: impl Into<LatLon>,
        bbox: LonLatBbox,
        fraction: f64,
    ) -> Self {
        let diagonal_m = haversine(
            LatLon::new(bbox.lon_min, bbox.lat_min),
            LatLon::new(bbox.lon_max, bbox.lat_max),
        );
        Self {
            origin: origin.into(),
            destination: destination.into(),
            bbox,
            step_distance_max: diagonal_m * fraction,
        }
    }

    pub fn constrain_endpoints_xy<const N: usize>(&self, path: &mut PathXY<N>) {
        path.set_lat_lon(0, self.origin);
        path.set_lat_lon(N - 1, self.destination);
    }

    pub fn constrain_endpoints_xyt<const N: usize>(&self, path: &mut Path<N>) {
        self.constrain_endpoints_xy(&mut path.xy);
        path.t[0] = 0.0;
    }

    pub fn clamp(&self, p: LatLon) -> LatLon {
        self.bbox.clamp(p)
    }

    /// Endpoints pinned, interior xy bbox-clamped, all times zeroed.
    /// Pure geometry — time-dependent clamping is the boundary's job.
    pub fn constrain_xy<const N: usize>(&self, pos: &Path<N>) -> Path<N> {
        let mut clamped: Path<N> = Path::default();
        self.constrain_endpoints_xyt(&mut clamped);
        for i in 1..N - 1 {
            clamped.xy.set_lat_lon(i, self.clamp(pos.xy.lat_lon(i)));
        }
        clamped
    }

    /// Lerp from `origin` (t=0) to `destination` (t=1) along the
    /// short way across the antimeridian.
    pub fn lerp_between_endpoints(&self, t: f64) -> LatLon {
        // `signed_lon_delta` picks the short way: Tokyo→LA goes east
        // through 180°, not west through 0°.
        let dlon = signed_lon_delta(self.origin.lon, self.destination.lon);
        let lon = wrap_lon_deg(self.origin.lon + t * dlon);
        let lat = lerp(self.origin.lat, self.destination.lat, t);
        LatLon::new(lon, lat)
    }
}
