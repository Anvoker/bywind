//! Static landmass / coastline data for the sailing search.
//!
//! Loads Natural Earth 1:50m polygons once, rasterises them into a binary
//! land mask at [`SDF_RESOLUTION_DEG`], and computes a signed distance
//! field plus an outward east-north tangent gradient via the 8SSEDT
//! (Saito–Toriwaki) two-pass algorithm. The resulting [`LandmassGrid`]
//! implements [`swarmkit_sailing::LandmassSource`] so the search can
//! consult the SDF without knowing this crate's grid layout.
//!
//! Construction is gated behind [`OnceLock`]s and only happens when the
//! corresponding accessor (`raw_polygons` or `landmass_grid`) is first
//! called, so unrelated tests don't pay the rasterisation cost.
//!
//! # Coordinate convention and approximations
//!
//! - Cells are indexed as `j * width + i` with `i ∈ 0..width` along
//!   longitude (full 360°, antimeridian-wrap aware) and `j ∈ 0..height`
//!   along latitude going south → north.
//! - Cell distances are converted to metres using a *uniform* factor of
//!   `cell_deg * METRES_PER_DEGREE`. This is exact at the equator and
//!   underestimates by `cos(lat)` for east-west distances at higher
//!   latitudes — fine for PSO-scale routes (tens to thousands of km)
//!   where sub-cell precision is irrelevant, but worth knowing if a
//!   downstream consumer needs metric exactness.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, VecDeque};
use std::sync::{Mutex, OnceLock};

use serde::Deserialize;
use swarmkit_sailing::spherical::{
    LatLon, METRES_PER_DEGREE, TangentMetres, haversine, signed_lon_delta, wrap_lon_deg,
};
use swarmkit_sailing::{LandmassSource, RouteBounds, SeaPathBias};

/// Public-domain Natural Earth 1:50m landmass data, embedded so callers
/// stay self-contained. Single source of truth for both the search-side
/// SDF (this module) and the rendering layer in `bywind-viz`.
#[expect(
    clippy::large_include_file,
    reason = "Natural Earth coastline data is embedded deliberately so the \
              landmass-aware search works without an asset-loading step on \
              every consumer."
)]
const LAND_GEOJSON: &[u8] = include_bytes!("../assets/ne_50m_land.geojson");

/// Default cell size of the rasterised land mask / SDF (degrees).
///
/// 0.5° → 720 × 360 ≈ 260k cells, plenty for PSO-scale segments.
/// [`landmass_grid`] uses this; callers driving
/// `SearchConfig::sdf_resolution_deg` go through
/// [`landmass_grid_at_resolution`].
pub const SDF_RESOLUTION_DEG: f64 = 0.5;

// ============================================================================
// GeoJSON parsing
// ============================================================================

/// One Natural Earth landmass: an outer ring plus zero or more interior
/// holes (e.g. inland seas), each ring as `(lon°, lat°)` pairs stored
/// open (no duplicate first/last vertex).
#[derive(Debug, Clone)]
pub struct Polygon {
    pub rings: Vec<Vec<(f64, f64)>>,
}

#[derive(Deserialize)]
struct FeatureCollection {
    features: Vec<Feature>,
}

#[derive(Deserialize)]
struct Feature {
    geometry: Geometry,
}

#[derive(Deserialize)]
#[serde(tag = "type")]
enum Geometry {
    Polygon {
        coordinates: Vec<Vec<[f64; 2]>>,
    },
    MultiPolygon {
        coordinates: Vec<Vec<Vec<[f64; 2]>>>,
    },
    #[serde(other)]
    Other,
}

impl Geometry {
    fn into_polygons(self) -> Vec<Polygon> {
        let convert_ring = |ring: Vec<[f64; 2]>| -> Vec<(f64, f64)> {
            let mut out: Vec<(f64, f64)> = ring.into_iter().map(|p| (p[0], p[1])).collect();
            // GeoJSON rings are closed; strip the duplicate so downstream
            // edge iteration with `(k, (k + 1) % n)` doesn't produce a
            // zero-length edge.
            if out.len() >= 2 && out.first() == out.last() {
                out.pop();
            }
            out
        };
        match self {
            Self::Polygon { coordinates } => {
                vec![Polygon {
                    rings: coordinates.into_iter().map(convert_ring).collect(),
                }]
            }
            Self::MultiPolygon { coordinates } => coordinates
                .into_iter()
                .map(|p| Polygon {
                    rings: p.into_iter().map(convert_ring).collect(),
                })
                .collect(),
            Self::Other => Vec::new(),
        }
    }
}

/// Parsed Natural Earth polygons, cached for the process lifetime. Public
/// so the rendering layer can consume the same source data without
/// re-parsing or maintaining its own copy.
pub fn raw_polygons() -> &'static [Polygon] {
    static POLYGONS: OnceLock<Vec<Polygon>> = OnceLock::new();
    POLYGONS.get_or_init(
        || match serde_json::from_slice::<FeatureCollection>(LAND_GEOJSON) {
            Ok(fc) => fc
                .features
                .into_iter()
                .flat_map(|f| f.geometry.into_polygons())
                .collect(),
            Err(e) => {
                log::error!("failed to parse bundled landmasses: {e}");
                Vec::new()
            }
        },
    )
}

// ============================================================================
// Rasterisation
// ============================================================================

/// Inclusive lon/lat bounding box of a polygon (across all its rings).
fn polygon_bbox(poly: &Polygon) -> (f64, f64, f64, f64) {
    let mut lon_min = f64::INFINITY;
    let mut lon_max = f64::NEG_INFINITY;
    let mut lat_min = f64::INFINITY;
    let mut lat_max = f64::NEG_INFINITY;
    for ring in &poly.rings {
        for &(lon, lat) in ring {
            lon_min = lon_min.min(lon);
            lon_max = lon_max.max(lon);
            lat_min = lat_min.min(lat);
            lat_max = lat_max.max(lat);
        }
    }
    (lon_min, lon_max, lat_min, lat_max)
}

/// Even-odd point-in-polygon over all rings. Each ring's standard
/// ray-casting toggles the result, so an outer ring marks the cell as
/// inside and a hole flips it back to outside.
fn point_in_polygon(lon: f64, lat: f64, poly: &Polygon) -> bool {
    let mut inside = false;
    for ring in &poly.rings {
        if point_in_ring(lon, lat, ring) {
            inside = !inside;
        }
    }
    inside
}

/// Standard horizontal-ray crossing test. The ray points east; the
/// polygon edge `(x1, y1) → (x2, y2)` counts iff `lat` lies between
/// `y1` and `y2` (half-open) AND the `lon` query lies west of the
/// edge's intersection with `lat`.
fn point_in_ring(lon: f64, lat: f64, ring: &[(f64, f64)]) -> bool {
    let n = ring.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    for k in 0..n {
        let (x1, y1) = ring[k];
        let (x2, y2) = ring[(k + 1) % n];
        if (y1 > lat) != (y2 > lat) {
            // Edge straddles the horizontal ray. Compute the lon at the
            // intersection and toggle if the query point is to the west.
            let xint = x1 + (lat - y1) * (x2 - x1) / (y2 - y1);
            if lon < xint {
                inside = !inside;
            }
        }
    }
    inside
}

/// Rasterise the polygon set into a binary land mask. Cells whose
/// centres fall inside any polygon are marked land. Bbox prefilter so
/// each polygon only tests cells in its lon/lat span.
fn rasterise_mask(polygons: &[Polygon], width: usize, height: usize, cell_deg: f64) -> Vec<bool> {
    let mut mask = vec![false; width * height];
    for poly in polygons {
        let (lon_min, lon_max, lat_min, lat_max) = polygon_bbox(poly);
        // Cell coordinates: cell i has centre lon = -180 + (i + 0.5) * cell_deg.
        // Inverting: the lowest cell whose centre is >= lon_min is
        // ceil((lon_min + 180) / cell_deg - 0.5).
        let i_lo = (((lon_min + 180.0) / cell_deg - 0.5).ceil() as isize).max(0) as usize;
        let i_hi = (((lon_max + 180.0) / cell_deg - 0.5).floor() as isize + 1).max(0) as usize;
        let j_lo = (((lat_min + 90.0) / cell_deg - 0.5).ceil() as isize).max(0) as usize;
        let j_hi = (((lat_max + 90.0) / cell_deg - 0.5).floor() as isize + 1).max(0) as usize;
        let i_hi = i_hi.min(width);
        let j_hi = j_hi.min(height);

        for j in j_lo..j_hi {
            let lat = -90.0 + (j as f64 + 0.5) * cell_deg;
            for i in i_lo..i_hi {
                let lon = -180.0 + (i as f64 + 0.5) * cell_deg;
                if !mask[j * width + i] && point_in_polygon(lon, lat, poly) {
                    mask[j * width + i] = true;
                }
            }
        }
    }
    mask
}

// ============================================================================
// 8SSEDT distance transform
// ============================================================================

/// Sentinel for "no seed seen yet". Smaller than `i32::MAX / 2` so the
/// `+ 1` propagation in either pass cannot overflow.
const INF: i32 = 1 << 24;

/// Square magnitude of an integer offset, in `i64` to avoid overflow
/// when cells are at world-scale distances (~720 cells max).
fn dist_sq(g: (i32, i32)) -> i64 {
    i64::from(g.0) * i64::from(g.0) + i64::from(g.1) * i64::from(g.1)
}

fn better(p: (i32, i32), q: (i32, i32)) -> (i32, i32) {
    if dist_sq(q) < dist_sq(p) { q } else { p }
}

/// Two-pass exact-Euclidean signed distance transform. Each cell stores
/// `(p − seed)`, the displacement from the cell's nearest seed point to
/// the cell itself. Output unit is *cells*; multiply by cell-metres to
/// convert.
///
/// `lon_wrap = true` makes the leftmost and rightmost cell columns
/// neighbours (antimeridian wrap). Latitude does not wrap.
fn distance_transform(
    seed_mask: &[bool],
    width: usize,
    height: usize,
    lon_wrap: bool,
) -> Vec<(i32, i32)> {
    let mut g = vec![(INF, INF); width * height];
    for (idx, &m) in seed_mask.iter().enumerate() {
        if m {
            g[idx] = (0, 0);
        }
    }

    let lon_minus = |i: usize| -> Option<usize> {
        if i > 0 {
            Some(i - 1)
        } else if lon_wrap {
            Some(width - 1)
        } else {
            None
        }
    };
    let lon_plus = |i: usize| -> Option<usize> {
        if i + 1 < width {
            Some(i + 1)
        } else if lon_wrap {
            Some(0)
        } else {
            None
        }
    };

    // Forward sweep: visit cells in raster order and propagate offsets
    // from the four already-visited neighbours (W, NW, N, NE).
    for j in 0..height {
        for i in 0..width {
            let mut best = g[j * width + i];
            if let Some(il) = lon_minus(i) {
                let n = g[j * width + il];
                best = better(best, (n.0 + 1, n.1));
            }
            if j > 0 {
                if let Some(il) = lon_minus(i) {
                    let n = g[(j - 1) * width + il];
                    best = better(best, (n.0 + 1, n.1 + 1));
                }
                let n = g[(j - 1) * width + i];
                best = better(best, (n.0, n.1 + 1));
                if let Some(ir) = lon_plus(i) {
                    let n = g[(j - 1) * width + ir];
                    best = better(best, (n.0 - 1, n.1 + 1));
                }
            }
            g[j * width + i] = best;
        }
    }

    // Backward sweep: reverse raster order, propagate from (E, SE, S, SW).
    for j in (0..height).rev() {
        for i in (0..width).rev() {
            let mut best = g[j * width + i];
            if let Some(ir) = lon_plus(i) {
                let n = g[j * width + ir];
                best = better(best, (n.0 - 1, n.1));
            }
            if j + 1 < height {
                if let Some(ir) = lon_plus(i) {
                    let n = g[(j + 1) * width + ir];
                    best = better(best, (n.0 - 1, n.1 - 1));
                }
                let n = g[(j + 1) * width + i];
                best = better(best, (n.0, n.1 - 1));
                if let Some(il) = lon_minus(i) {
                    let n = g[(j + 1) * width + il];
                    best = better(best, (n.0 + 1, n.1 - 1));
                }
            }
            g[j * width + i] = best;
        }
    }

    g
}

// ============================================================================
// LandmassGrid
// ============================================================================

/// Rasterised land mask + signed distance field + outward-pointing
/// gradient. Implements [`LandmassSource`] via bilinear lookup.
pub struct LandmassGrid {
    width: usize,
    height: usize,
    cell_deg: f64,
    /// Signed distance to nearest coast (m). Negative inside land,
    /// positive over water, zero on the cell-precision coastline.
    sdf_m: Vec<f32>,
    /// Outward unit gradient in the local east-north tangent frame.
    /// Stored as `(east, north)`. Magnitude ≈ 1 over open water; may
    /// degenerate to zero at multi-coast saddle points where central
    /// differences cancel.
    grad_en: Vec<(f32, f32)>,
}

impl LandmassGrid {
    /// Build a grid from a polygon set at the requested cell size.
    /// Used by the default global builder and by tests with synthetic
    /// shapes.
    pub fn build(polygons: &[Polygon], cell_deg: f64) -> Self {
        assert!(cell_deg > 0.0, "cell_deg must be positive, got {cell_deg}");
        let width = (360.0 / cell_deg).round() as usize;
        let height = (180.0 / cell_deg).round() as usize;
        let mask = rasterise_mask(polygons, width, height, cell_deg);
        Self::from_mask(&mask, width, height, cell_deg)
    }

    /// Build a grid directly from a precomputed binary mask. Primarily
    /// for tests; production code goes through [`build`](Self::build).
    pub fn from_mask(mask: &[bool], width: usize, height: usize, cell_deg: f64) -> Self {
        assert_eq!(mask.len(), width * height, "mask length mismatch");
        // Distance to nearest land for water cells, and to nearest water
        // for land cells. Two passes; we'll combine into a signed value.
        let to_land = distance_transform(mask, width, height, true);
        let inverted: Vec<bool> = mask.iter().map(|&b| !b).collect();
        let to_water = distance_transform(&inverted, width, height, true);

        // Cell-to-metres factor at the equator. Treats one cell of lon
        // as the same metres as one cell of lat (uniform conversion);
        // see module docstring for the cos(lat) caveat.
        let cell_m = cell_deg * METRES_PER_DEGREE;

        let mut sdf_m = vec![0.0f32; width * height];
        for (idx, &m) in mask.iter().enumerate() {
            let unsigned = if m {
                (dist_sq(to_water[idx]) as f64).sqrt()
            } else {
                (dist_sq(to_land[idx]) as f64).sqrt()
            };
            let metres = unsigned * cell_m;
            sdf_m[idx] = if m { -metres as f32 } else { metres as f32 };
        }

        let grad_en = compute_gradient(&sdf_m, width, height, cell_deg);

        Self {
            width,
            height,
            cell_deg,
            sdf_m,
            grad_en,
        }
    }

    /// Convert `lon` to a fractional cell index in `0..width`. Wraps
    /// around the antimeridian by `rem_euclid` so callers can pass any
    /// representation.
    fn lon_to_cell(&self, lon: f64) -> f64 {
        ((lon + 180.0).rem_euclid(360.0)) / self.cell_deg - 0.5
    }

    /// Convert `lat` to a fractional cell index in `-0.5..height-0.5`.
    /// Caller is responsible for clamping out-of-range lats; we do it
    /// inside the bilinear lookup.
    fn lat_to_cell(&self, lat: f64) -> f64 {
        (lat + 90.0) / self.cell_deg - 0.5
    }

    /// Wrap lon-cell `i` modulo `width`. Negative inputs wrap correctly
    /// because we go through `usize` only after the modulo.
    fn wrap_i(&self, i: isize) -> usize {
        i.rem_euclid(self.width as isize) as usize
    }

    /// Centre `(lon°, lat°)` of cell `(i, j)`.
    fn cell_centre(&self, i: usize, j: usize) -> LatLon {
        let lon = -180.0 + (i as f64 + 0.5) * self.cell_deg;
        let lat = -90.0 + (j as f64 + 0.5) * self.cell_deg;
        LatLon::new(lon, lat)
    }

    /// Snap a `(lon°, lat°)` location to the index of its containing
    /// cell. Lon wraps; lat clamps to `[0, height-1]`.
    fn cell_index_of(&self, location: LatLon) -> (usize, usize) {
        let fi = self.lon_to_cell(location.lon) + 0.5;
        let fj = (self.lat_to_cell(location.lat) + 0.5).clamp(0.0, (self.height - 1) as f64);
        let i = self.wrap_i(fi.floor() as isize);
        let j = (fj.floor() as usize).min(self.height - 1);
        (i, j)
    }

    fn cell_idx(&self, i: usize, j: usize) -> usize {
        j * self.width + i
    }

    fn is_sea(&self, i: usize, j: usize) -> bool {
        self.sdf_m[self.cell_idx(i, j)] >= 0.0
    }

    /// Bilinear interpolation of a per-cell scalar field. Lat is
    /// clamped to the cell-centre range `[0, height-1]`; lon wraps.
    fn bilinear<F: Fn(usize, usize) -> f32>(&self, lon: f64, lat: f64, sample: F) -> f32 {
        let fi = self.lon_to_cell(lon);
        let fj = self.lat_to_cell(lat).clamp(0.0, (self.height - 1) as f64);
        let i0 = fi.floor() as isize;
        let j0 = (fj.floor() as isize).clamp(0, self.height as isize - 1);
        let alpha = (fi - fi.floor()) as f32;
        let beta = (fj - fj.floor()) as f32;
        let i1 = i0 + 1;
        let j1 = (j0 + 1).min(self.height as isize - 1);
        let i0w = self.wrap_i(i0);
        let i1w = self.wrap_i(i1);
        let j0u = j0 as usize;
        let j1u = j1 as usize;
        let s00 = sample(i0w, j0u);
        let s10 = sample(i1w, j0u);
        let s01 = sample(i0w, j1u);
        let s11 = sample(i1w, j1u);
        let s0 = s00 * (1.0 - alpha) + s10 * alpha;
        let s1 = s01 * (1.0 - alpha) + s11 * alpha;
        s0 * (1.0 - beta) + s1 * beta
    }
}

impl LandmassSource for LandmassGrid {
    fn signed_distance_m(&self, location: LatLon) -> f64 {
        let lon = wrap_lon_deg(location.lon);
        f64::from(self.bilinear(lon, location.lat, |i, j| self.sdf_m[j * self.width + i]))
    }

    fn gradient(&self, location: LatLon) -> TangentMetres {
        let lon = wrap_lon_deg(location.lon);
        let east = self.bilinear(lon, location.lat, |i, j| self.grad_en[j * self.width + i].0);
        let north = self.bilinear(lon, location.lat, |i, j| self.grad_en[j * self.width + i].1);
        TangentMetres::new(f64::from(east), f64::from(north))
    }

    fn find_sea_path(
        &self,
        origin: LatLon,
        destination: LatLon,
        bounds: &RouteBounds,
        bias: SeaPathBias,
    ) -> Option<Vec<LatLon>> {
        astar_sea_path(self, origin, destination, bounds, bias)
    }
}

/// Outward unit gradient at every cell, derived by central differences
/// on the signed distance field. Sign convention: positive = away from
/// land. Magnitude is normalised to ~1 wherever the SDF is locally
/// non-flat; degenerates to zero at saddle points.
fn compute_gradient(sdf_m: &[f32], width: usize, height: usize, cell_deg: f64) -> Vec<(f32, f32)> {
    let mut grad = vec![(0.0f32, 0.0f32); width * height];
    let cell_m = cell_deg * METRES_PER_DEGREE;
    for j in 0..height {
        let lat = -90.0 + (j as f64 + 0.5) * cell_deg;
        let cos_lat = lat.to_radians().cos().max(1e-9);
        // Central differences in *cells*. Convert to metres using the
        // east-cell metre length at this latitude (cos(lat) shrinkage)
        // and the lat-cell metre length (constant on a sphere).
        let inv_dx_m = 1.0 / (2.0 * cell_m * cos_lat);
        let inv_dy_m = 1.0 / (2.0 * cell_m);
        for i in 0..width {
            // Lon wraps; lat clamps at the poles using one-sided
            // differences via min/max on the index.
            let il = ((i as isize - 1).rem_euclid(width as isize)) as usize;
            let ir = ((i as isize + 1).rem_euclid(width as isize)) as usize;
            let jb = j.saturating_sub(1);
            let jt = (j + 1).min(height - 1);
            let dsdx = (sdf_m[j * width + ir] - sdf_m[j * width + il]) as f64 * inv_dx_m;
            let dsdy = (sdf_m[jt * width + i] - sdf_m[jb * width + i]) as f64 * inv_dy_m;
            // SDF is negative inside land and increases outward, so
            // (∂SDF/∂east, ∂SDF/∂north) already points outward. Normalise.
            let mag = dsdx.hypot(dsdy);
            let g = if mag > 1e-12 {
                (dsdx / mag, dsdy / mag)
            } else {
                (0.0, 0.0)
            };
            grad[j * width + i] = (g.0 as f32, g.1 as f32);
        }
    }
    grad
}

// ============================================================================
// A* sea pathfinder
// ============================================================================

/// Slack on the bias hard-barrier, in degrees of latitude. Cells within
/// this distance of the straight-line interpolation are passable under
/// either bias. Lets the path leave the origin / arrive at the
/// destination cleanly even when those cells sit on the line itself.
const BIAS_BARRIER_SLACK_DEG: f64 = 0.5;

/// Hard cap on the BFS-snap radius (cells). Bounds the cost of snapping
/// a deeply on-land start/goal — for legitimate routes the snap finds
/// sea within a couple of cells.
const SNAP_TO_SEA_MAX_RING: usize = 32;

/// Hard cap on cells expanded by A*. Defensive: a 720×360 grid has
/// ~260 k cells, so this bound only triggers for searches that have
/// already explored the entire reachable region without finding the
/// goal. Returns None on overflow.
const ASTAR_MAX_EXPANSIONS: usize = 1_000_000;

/// Priority-queue entry for A*. Ordered by `f_score` ascending — Rust's
/// `BinaryHeap` is a max-heap, so we invert the comparison.
#[derive(Copy, Clone, Debug)]
struct AStarNode {
    f_score: f64,
    cell: u32,
}

impl PartialEq for AStarNode {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}
impl Eq for AStarNode {}
impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reversed so BinaryHeap acts as a min-heap on f_score.
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

/// Snap an arbitrary `(lon, lat)` to the index of the nearest sea cell
/// via 8-connected BFS rings. Returns `None` if no sea cell is found
/// within [`SNAP_TO_SEA_MAX_RING`] cells (either the input is hopelessly
/// landlocked, or the grid has no sea at all).
fn snap_to_sea_cell(grid: &LandmassGrid, location: LatLon) -> Option<(usize, usize)> {
    let (i0, j0) = grid.cell_index_of(location);
    if grid.is_sea(i0, j0) {
        return Some((i0, j0));
    }
    let mut visited = vec![false; grid.width * grid.height];
    let mut queue = VecDeque::new();
    visited[grid.cell_idx(i0, j0)] = true;
    queue.push_back((i0, j0, 0usize));
    while let Some((i, j, ring)) = queue.pop_front() {
        if grid.is_sea(i, j) {
            return Some((i, j));
        }
        if ring >= SNAP_TO_SEA_MAX_RING {
            continue;
        }
        for (ni, nj) in neighbours_8(grid, i, j) {
            let idx = grid.cell_idx(ni, nj);
            if !visited[idx] {
                visited[idx] = true;
                queue.push_back((ni, nj, ring + 1));
            }
        }
    }
    None
}

/// Yield the eight neighbours of cell `(i, j)`. Lon wraps; lat clamps
/// (cells at the top / bottom row simply skip the missing neighbours).
fn neighbours_8(
    grid: &LandmassGrid,
    i: usize,
    j: usize,
) -> impl Iterator<Item = (usize, usize)> + '_ {
    [-1isize, 0, 1].into_iter().flat_map(move |dj| {
        [-1isize, 0, 1].into_iter().filter_map(move |di| {
            if di == 0 && dj == 0 {
                return None;
            }
            let nj_signed = j as isize + dj;
            if nj_signed < 0 || nj_signed >= grid.height as isize {
                return None;
            }
            let ni = grid.wrap_i(i as isize + di);
            Some((ni, nj_signed as usize))
        })
    })
}

/// Lat at `lon` along the straight-line interpolation between origin
/// and destination of `bounds`. Antimeridian-aware via `signed_lon_delta`.
fn line_lat_at_lon(bounds: &RouteBounds, lon: f64) -> f64 {
    let origin = bounds.origin;
    let dest = bounds.destination;
    let total_dlon = signed_lon_delta(origin.lon, dest.lon);
    if total_dlon.abs() < 1e-12 {
        // Pure meridional route — no lon variation; just average the lats.
        return (origin.lat + dest.lat) * 0.5;
    }
    let dlon_to_query = signed_lon_delta(origin.lon, lon);
    let t = (dlon_to_query / total_dlon).clamp(0.0, 1.0);
    origin.lat + t * (dest.lat - origin.lat)
}

/// Hard-barrier filter for biased A*: returns `true` iff cell `(i, j)`
/// is allowed under `bias`. Cells equal to `start` or `goal` are always
/// allowed so the path can leave / arrive even when those cells sit
/// outside the slack band.
fn bias_allows(
    grid: &LandmassGrid,
    bounds: &RouteBounds,
    start: (usize, usize),
    goal: (usize, usize),
    cell: (usize, usize),
    bias: SeaPathBias,
) -> bool {
    if cell == start || cell == goal {
        return true;
    }
    let centre = grid.cell_centre(cell.0, cell.1);
    let line_lat = line_lat_at_lon(bounds, centre.lon);
    let offset = centre.lat - line_lat;
    match bias {
        SeaPathBias::None => true,
        SeaPathBias::North => offset >= -BIAS_BARRIER_SLACK_DEG,
        SeaPathBias::South => offset <= BIAS_BARRIER_SLACK_DEG,
    }
}

/// Run A* on the binary land mask. 8-connected, edge cost = great-circle
/// distance between cell centres, heuristic = great-circle to goal.
/// Returns the cell-centre polyline including endpoints, or `None` if
/// the goal is unreachable / outside the bbox / blocked by the bias.
fn astar_sea_path(
    grid: &LandmassGrid,
    origin: LatLon,
    destination: LatLon,
    bounds: &RouteBounds,
    bias: SeaPathBias,
) -> Option<Vec<LatLon>> {
    let start = snap_to_sea_cell(grid, origin)?;
    let goal = snap_to_sea_cell(grid, destination)?;
    if start == goal {
        return Some(vec![origin, destination]);
    }

    let n_cells = grid.width * grid.height;
    let mut g_score = vec![f64::INFINITY; n_cells];
    let mut came_from: Vec<u32> = vec![u32::MAX; n_cells];
    let mut closed = vec![false; n_cells];
    let mut open: BinaryHeap<AStarNode> = BinaryHeap::new();

    let goal_centre = grid.cell_centre(goal.0, goal.1);
    let start_idx = grid.cell_idx(start.0, start.1);
    g_score[start_idx] = 0.0;
    open.push(AStarNode {
        f_score: haversine(grid.cell_centre(start.0, start.1), goal_centre),
        cell: start_idx as u32,
    });

    let mut expansions = 0usize;
    while let Some(AStarNode { cell, .. }) = open.pop() {
        let cell = cell as usize;
        if closed[cell] {
            continue;
        }
        closed[cell] = true;
        if cell == grid.cell_idx(goal.0, goal.1) {
            return Some(reconstruct_polyline(
                grid,
                &came_from,
                cell,
                origin,
                destination,
            ));
        }
        expansions += 1;
        if expansions > ASTAR_MAX_EXPANSIONS {
            log::warn!(
                "find_sea_path: A* aborted after {ASTAR_MAX_EXPANSIONS} expansions \
                 (origin={origin:?}, destination={destination:?})",
            );
            return None;
        }
        let i = cell % grid.width;
        let j = cell / grid.width;
        let cur_centre = grid.cell_centre(i, j);
        for (ni, nj) in neighbours_8(grid, i, j) {
            if !grid.is_sea(ni, nj) {
                continue;
            }
            if !bias_allows(grid, bounds, start, goal, (ni, nj), bias) {
                continue;
            }
            let neighbour_centre = grid.cell_centre(ni, nj);
            // Restrict the search to the route's bbox so the polyline
            // stays inside the user's chosen domain. Start and goal
            // are exempt: the snap to a sea cell may move the cell
            // centre slightly outside the bbox even when the
            // user-supplied endpoint was inside, and we don't want
            // that snap-induced drift to make a legitimate route
            // unreachable.
            if (ni, nj) != start && (ni, nj) != goal && !bounds.bbox.contains(neighbour_centre) {
                continue;
            }
            let step = haversine(cur_centre, neighbour_centre);
            let tentative_g = g_score[cell] + step;
            let n_idx = grid.cell_idx(ni, nj);
            if tentative_g < g_score[n_idx] {
                g_score[n_idx] = tentative_g;
                came_from[n_idx] = cell as u32;
                let h = haversine(neighbour_centre, goal_centre);
                open.push(AStarNode {
                    f_score: tentative_g + h,
                    cell: n_idx as u32,
                });
            }
        }
    }
    None
}

fn reconstruct_polyline(
    grid: &LandmassGrid,
    came_from: &[u32],
    goal_cell: usize,
    origin: LatLon,
    destination: LatLon,
) -> Vec<LatLon> {
    let mut cells: Vec<usize> = vec![goal_cell];
    let mut cur = goal_cell;
    while came_from[cur] != u32::MAX {
        cur = came_from[cur] as usize;
        cells.push(cur);
    }
    cells.reverse();
    let mut polyline: Vec<LatLon> = Vec::with_capacity(cells.len() + 2);
    polyline.push(origin);
    // Skip the snapped-cell endpoints if they coincide with the actual
    // origin / destination (avoids a tiny zero-distance leg). For the
    // typical case (start/goal cells away from the literal lat/lon),
    // include the cell centres so the polyline reflects the search.
    for cell in cells {
        let i = cell % grid.width;
        let j = cell / grid.width;
        polyline.push(grid.cell_centre(i, j));
    }
    polyline.push(destination);
    polyline
}

// ============================================================================
// Default global instance
// ============================================================================

/// Default landmass grid at [`SDF_RESOLUTION_DEG`].
pub fn landmass_grid() -> &'static LandmassGrid {
    landmass_grid_at_resolution(SDF_RESOLUTION_DEG)
}

/// Landmass grid at a caller-chosen resolution, cached per distinct value.
///
/// Cache keys are `f64::to_bits` (no fixed-point rounding collisions).
/// Builds run under the registry mutex so concurrent same-key requests
/// can't both leak; each unique resolution leaks one `LandmassGrid`
/// for the program's lifetime. Mutex poison is recovered (`into_inner`)
/// — the cache is insert-only, so a post-panic reader is safe.
pub fn landmass_grid_at_resolution(resolution_deg: f64) -> &'static LandmassGrid {
    static REGISTRY: OnceLock<Mutex<HashMap<u64, &'static LandmassGrid>>> = OnceLock::new();
    let registry = REGISTRY.get_or_init(|| Mutex::new(HashMap::new()));
    let key = resolution_deg.to_bits();
    let mut guard = match registry.lock() {
        Ok(g) => g,
        Err(poison) => poison.into_inner(),
    };
    if let Some(grid) = guard.get(&key) {
        return grid;
    }
    let built: &'static LandmassGrid = Box::leak(Box::new(LandmassGrid::build(
        raw_polygons(),
        resolution_deg,
    )));
    guard.insert(key, built);
    built
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "tests rely on bit-exact comparisons of constant or stored f32/f64 values."
    )]
    use super::*;
    use swarmkit_sailing::spherical::LonLatBbox;
    use swarmkit_sailing::{PathBaseline, get_segment_land_metres};

    /// Regression for the around-continent benchmark land-clipping bug:
    /// `from_polyline_land_respecting` previously rejected every break
    /// candidate (smart-break required `max(l_left, l_right) < parent`,
    /// which is unattainable when the polyline detour is far from the
    /// parent chord) and produced one giant land-crossing chord with
    /// the rest of the budget crammed near one endpoint. Two scenarios
    /// from `grib-data/a-star-{land-bug,goes-over-land-a-lot}.toml`:
    ///
    /// - Bay of Biscay → off Angola (N=40, around West Africa)
    /// - Bay of Biscay → Arabian Sea (N=60, around all of Africa)
    ///
    /// Both must produce land-free chords throughout.
    #[test]
    fn benchmark_sampler_handles_around_continent_routes() {
        let grid = landmass_grid();

        let bob_to_angola_bbox = LonLatBbox::new(
            -23.450000000000045,
            16.450000000000045,
            -20.317062759399413,
            57.033261680603026,
        );
        let bob_to_arabian_bbox = LonLatBbox::new(
            -32.94999999999999,
            73.45000000000005,
            -51.650000000000006,
            63.150000000000006,
        );

        let scenarios: &[(&str, LatLon, LatLon, LonLatBbox)] = &[
            (
                "BoB -> off Angola",
                LatLon::new(-6.041525363922119, 45.98321533203125),
                LatLon::new(10.615649223327637, -9.267016410827637),
                bob_to_angola_bbox,
            ),
            (
                "BoB -> Arabian Sea",
                LatLon::new(-12.371894836425781, 46.70365524291992),
                LatLon::new(58.23857498168945, 10.672679901123047),
                bob_to_arabian_bbox,
            ),
        ];

        for &(name, origin, destination, bbox) in scenarios {
            let bounds = swarmkit_sailing::RouteBounds::new(origin, destination, bbox);
            let polyline = grid
                .find_sea_path(
                    origin,
                    destination,
                    &bounds,
                    swarmkit_sailing::SeaPathBias::None,
                )
                .unwrap_or_else(|| panic!("{name}: A* found no path"));

            // Sanity: the raw A* polyline should be land-free
            // (it walks sea cell centres). If this ever fails the bug
            // is upstream of the sampler.
            let raw_land: f64 = polyline
                .windows(2)
                .map(|w| get_segment_land_metres(grid, w[0], w[1], bounds.step_distance_max))
                .sum();
            assert_eq!(
                raw_land, 0.0,
                "{name}: raw A* polyline crosses {raw_land:.0} m of land",
            );

            // The land-respecting sampler must produce land-free chords
            // at the scenario's N. Both scenarios at N=40 and N=60 to
            // cover the budgets the user's scenario files specify.
            macro_rules! check_at {
                ($n:literal) => {{
                    let baseline =
                        PathBaseline::<$n>::from_polyline_land_respecting(&polyline, &bounds, grid);
                    let mut bad: Vec<(usize, f64)> = Vec::new();
                    for (i, w) in baseline.positions.windows(2).enumerate() {
                        let land =
                            get_segment_land_metres(grid, w[0], w[1], bounds.step_distance_max);
                        if land > 0.0 {
                            bad.push((i, land));
                        }
                    }
                    assert!(
                        bad.is_empty(),
                        "{name} N={}: {} chord(s) cross land, worst {:.0} m at chord {}",
                        $n,
                        bad.len(),
                        bad.iter().map(|&(_, l)| l).fold(0.0_f64, f64::max),
                        bad.iter().max_by(|a, b| a.1.total_cmp(&b.1)).unwrap().0,
                    );
                }};
            }
            check_at!(40);
            check_at!(60);
        }
    }

    /// Build a binary mask from a closure that decides land/water per
    /// cell. Cell index `(i, j)` corresponds to cell-centre lon
    /// `-180 + (i + 0.5) * cell_deg`, lat `-90 + (j + 0.5) * cell_deg`.
    fn mask_from_predicate(
        width: usize,
        height: usize,
        cell_deg: f64,
        is_land: impl Fn(f64, f64) -> bool,
    ) -> Vec<bool> {
        let mut m = vec![false; width * height];
        for j in 0..height {
            for i in 0..width {
                let lon = -180.0 + (i as f64 + 0.5) * cell_deg;
                let lat = -90.0 + (j as f64 + 0.5) * cell_deg;
                m[j * width + i] = is_land(lon, lat);
            }
        }
        m
    }

    #[test]
    fn synthetic_square_island_signed_distance_signs_correctly() {
        // 2° square island centered at (0, 0). Coarse cells so the test
        // is fast and the synthetic shape is comfortably larger than a
        // few cells.
        let cell_deg = 0.5;
        let width = (360.0_f64 / cell_deg).round() as usize;
        let height = (180.0_f64 / cell_deg).round() as usize;
        let mask = mask_from_predicate(width, height, cell_deg, |lon, lat| {
            lon.abs() < 1.0 && lat.abs() < 1.0
        });
        let grid = LandmassGrid::from_mask(&mask, width, height, cell_deg);

        // Centre of the island: deeply negative SDF.
        let sd_centre = grid.signed_distance_m(LatLon::new(0.0, 0.0));
        assert!(
            sd_centre < -50_000.0,
            "expected deep land, got {sd_centre} m"
        );

        // Far in the open ocean: deeply positive SDF.
        let sd_ocean = grid.signed_distance_m(LatLon::new(60.0, 0.0));
        assert!(
            sd_ocean > 1_000_000.0,
            "expected deep ocean, got {sd_ocean} m"
        );
    }

    #[test]
    fn synthetic_square_island_gradient_points_outward() {
        let cell_deg = 0.5;
        let width = (360.0_f64 / cell_deg).round() as usize;
        let height = (180.0_f64 / cell_deg).round() as usize;
        let mask = mask_from_predicate(width, height, cell_deg, |lon, lat| {
            lon.abs() < 2.0 && lat.abs() < 2.0
        });
        let grid = LandmassGrid::from_mask(&mask, width, height, cell_deg);

        // At a point a few cells east of the island, gradient should
        // point east (positive `east`).
        let g_east = grid.gradient(LatLon::new(5.0, 0.0));
        assert!(
            g_east.east > 0.5,
            "expected eastward gradient, got {g_east:?}"
        );
        // At a point north of the island, gradient should point north.
        let g_north = grid.gradient(LatLon::new(0.0, 5.0));
        assert!(
            g_north.north > 0.5,
            "expected northward gradient, got {g_north:?}"
        );
    }

    #[test]
    fn raw_polygons_yields_at_least_one_polygon() {
        let polys = raw_polygons();
        assert!(!polys.is_empty(), "expected polygons from bundled GeoJSON");
        assert!(
            polys
                .iter()
                .any(|p| p.rings.first().is_some_and(|r| r.len() >= 3)),
            "expected at least one non-degenerate ring",
        );
    }

    #[test]
    fn default_grid_classifies_known_continents_and_oceans() {
        let grid = landmass_grid();
        // Centre of Sahara — definitively land.
        let sahara = grid.signed_distance_m(LatLon::new(20.0, 25.0));
        assert!(sahara < 0.0, "Sahara should be land, got SDF = {sahara} m");
        // Mid-Pacific — definitively ocean.
        let pacific = grid.signed_distance_m(LatLon::new(-150.0, 0.0));
        assert!(
            pacific > 0.0,
            "Mid-Pacific should be ocean, got SDF = {pacific} m"
        );
    }

    #[test]
    fn antimeridian_lookup_is_continuous() {
        // SDF queries at lon = +179.99 and lon = -179.99 (almost-same
        // physical point) should agree to within one cell of distance.
        let grid = landmass_grid();
        let lat = 0.0;
        let east = grid.signed_distance_m(LatLon::new(179.99, lat));
        let west = grid.signed_distance_m(LatLon::new(-179.99, lat));
        let cell_m = SDF_RESOLUTION_DEG * METRES_PER_DEGREE;
        assert!(
            (east - west).abs() < cell_m,
            "antimeridian discontinuity: east={east} m, west={west} m",
        );
    }

    /// Build a small synthetic grid with a vertical landmass blocking
    /// the straight line between two opposite-side ocean points.
    /// Coarse cells (1°) so tests stay snappy.
    fn synthetic_grid_with_vertical_obstacle() -> LandmassGrid {
        let cell_deg = 1.0;
        let width = 360;
        let height = 180;
        // Land bar at lon ∈ [-2, 2], lat ∈ [-10, 10].
        let mask = mask_from_predicate(width, height, cell_deg, |lon, lat| {
            lon.abs() < 2.0 && lat.abs() < 10.0
        });
        LandmassGrid::from_mask(&mask, width, height, cell_deg)
    }

    fn route_bounds(origin: LatLon, destination: LatLon) -> RouteBounds {
        RouteBounds::new(
            origin,
            destination,
            LonLatBbox::new(-30.0, 30.0, -25.0, 25.0),
        )
    }

    #[test]
    fn find_sea_path_returns_polyline_for_open_ocean_route() {
        // Mid-Atlantic to mid-Pacific — both endpoints in deep ocean,
        // straight line is land-free.
        let grid = landmass_grid();
        let origin = LatLon::new(-30.0, 0.0);
        let destination = LatLon::new(-150.0, 0.0);
        let bounds = RouteBounds::new(
            origin,
            destination,
            LonLatBbox::new(-180.0, 180.0, -45.0, 45.0),
        );
        let polyline = grid
            .find_sea_path(origin, destination, &bounds, SeaPathBias::None)
            .expect("path");
        assert!(polyline.len() >= 2, "polyline must have at least endpoints");
        // The polyline holds the literal `origin` / `destination` as its
        // outer endpoints (not snapped cell centres), so equality is
        // exact.
        assert_eq!(polyline.first(), Some(&origin), "starts at origin");
        assert_eq!(polyline.last(), Some(&destination), "ends at destination");
        for &point in &polyline {
            assert!(
                grid.signed_distance_m(point) >= -SDF_RESOLUTION_DEG * METRES_PER_DEGREE,
                "intermediate point {point:?} on land",
            );
        }
    }

    #[test]
    fn find_sea_path_routes_around_synthetic_obstacle() {
        let grid = synthetic_grid_with_vertical_obstacle();
        let origin = LatLon::new(-10.0, 0.0);
        let destination = LatLon::new(10.0, 0.0);
        let bounds = route_bounds(origin, destination);
        let polyline = grid
            .find_sea_path(origin, destination, &bounds, SeaPathBias::None)
            .expect("path");
        // Polyline must detour around the land bar — the straight line
        // would put many interior cells inside the bar.
        assert!(polyline.len() > 4, "expected detour, got {polyline:?}");
        for &point in &polyline {
            assert!(!grid.is_land(point), "polyline point {point:?} on land",);
        }
    }

    #[test]
    fn find_sea_path_respects_route_bbox() {
        // Synthetic vertical-bar obstacle at lon ∈ [-2, 2],
        // lat ∈ [-10, 10]. A bbox of `lat ∈ [-20, 9]` excludes the
        // north detour (cells with centres at lat ≥ 9.5, so the
        // first sea row above the obstacle at lat 10.5 is blocked),
        // leaving only the south detour as a valid path.
        //
        // Without bbox enforcement A* would freely pick the north
        // detour; the test pins that the polyline stays inside the
        // bbox and detours south.
        let grid = synthetic_grid_with_vertical_obstacle();
        let origin = LatLon::new(-10.0, 0.0);
        let destination = LatLon::new(10.0, 0.0);
        let bounds = RouteBounds::new(
            origin,
            destination,
            LonLatBbox::new(-30.0, 30.0, -20.0, 9.0),
        );
        let polyline = grid
            .find_sea_path(origin, destination, &bounds, SeaPathBias::None)
            .expect("south detour should be reachable inside the bbox");
        // Every polyline vertex (except possibly the literal endpoint
        // pair) must fall inside the bbox.
        for &p in &polyline {
            if p == origin || p == destination {
                continue;
            }
            assert!(
                bounds.bbox.contains(p),
                "polyline vertex {p:?} outside bbox {:?}",
                bounds.bbox,
            );
        }
        // Detour direction: must go south (min lat < -9 — below the
        // obstacle's lower edge), not north.
        let min_lat = polyline.iter().map(|p| p.lat).fold(f64::INFINITY, f64::min);
        let max_lat = polyline
            .iter()
            .map(|p| p.lat)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            min_lat < -9.0,
            "expected south detour (min lat < -9), got min_lat = {min_lat}",
        );
        assert!(
            max_lat <= 9.0,
            "no vertex should exceed bbox lat_max = 9, got max_lat = {max_lat}",
        );
    }

    #[test]
    fn biased_sea_paths_take_opposite_sides_of_obstacle() {
        let grid = synthetic_grid_with_vertical_obstacle();
        let origin = LatLon::new(-10.0, 0.0);
        let destination = LatLon::new(10.0, 0.0);
        let bounds = route_bounds(origin, destination);
        let north = grid
            .find_sea_path(origin, destination, &bounds, SeaPathBias::North)
            .expect("north path");
        let south = grid
            .find_sea_path(origin, destination, &bounds, SeaPathBias::South)
            .expect("south path");
        let north_max_lat = north
            .iter()
            .map(|p| p.lat)
            .fold(f64::NEG_INFINITY, f64::max);
        let south_min_lat = south.iter().map(|p| p.lat).fold(f64::INFINITY, f64::min);
        assert!(
            north_max_lat > 9.0,
            "north-biased path should detour above the bar, max lat = {north_max_lat}",
        );
        assert!(
            south_min_lat < -9.0,
            "south-biased path should detour below the bar, min lat = {south_min_lat}",
        );
    }

    #[test]
    fn find_sea_path_returns_none_when_endpoints_landlocked() {
        // A grid that's entirely land except a small isolated island.
        let cell_deg = 1.0;
        let width = 360;
        let height = 180;
        let mask = mask_from_predicate(width, height, cell_deg, |lon, lat| {
            // Land everywhere except a tiny pond near (0, 0).
            !(lon.abs() < 2.0 && lat.abs() < 2.0)
        });
        let grid = LandmassGrid::from_mask(&mask, width, height, cell_deg);
        // Try to route from one landlocked point to another with no
        // sea connecting them. Snap should fail because the point is
        // deeply on land beyond the snap radius.
        let origin = LatLon::new(60.0, 30.0);
        let destination = LatLon::new(-60.0, -30.0);
        let bounds = RouteBounds::new(
            origin,
            destination,
            LonLatBbox::new(-90.0, 90.0, -60.0, 60.0),
        );
        assert!(
            grid.find_sea_path(origin, destination, &bounds, SeaPathBias::None)
                .is_none()
        );
    }
}
