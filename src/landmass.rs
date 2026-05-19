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
    LatLon, LonLatBbox, METRES_PER_DEGREE, TangentMetres, haversine, signed_lon_delta, wrap_lon_deg,
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
///
/// Cell `i, j` has centre `(origin_lon + (i + 0.5) * cell_deg,
/// origin_lat + (j + 0.5) * cell_deg)`. The global mask uses
/// `MaskFrame::global` (origin `(-180°, -90°)`); fine patches use their
/// bbox's lower-left corner as the origin.
fn rasterise_mask(polygons: &[Polygon], frame: MaskFrame) -> Vec<bool> {
    let MaskFrame {
        width,
        height,
        cell_deg,
        origin_lon,
        origin_lat,
        ..
    } = frame;
    let mut mask = vec![false; width * height];
    for poly in polygons {
        let (lon_min, lon_max, lat_min, lat_max) = polygon_bbox(poly);
        // Inverting `centre_lon = origin_lon + (i + 0.5) * cell_deg`,
        // the lowest cell whose centre is `>= lon_min` is
        // `ceil((lon_min - origin_lon) / cell_deg - 0.5)`. Symmetric for
        // lat. Out-of-frame polygons get zero-length loop bounds via the
        // `max(0) / min(width)` clamps.
        let i_lo = (((lon_min - origin_lon) / cell_deg - 0.5).ceil() as isize).max(0) as usize;
        let i_hi = (((lon_max - origin_lon) / cell_deg - 0.5).floor() as isize + 1).max(0) as usize;
        let j_lo = (((lat_min - origin_lat) / cell_deg - 0.5).ceil() as isize).max(0) as usize;
        let j_hi = (((lat_max - origin_lat) / cell_deg - 0.5).floor() as isize + 1).max(0) as usize;
        let i_hi = i_hi.min(width);
        let j_hi = j_hi.min(height);

        for j in j_lo..j_hi {
            let lat = origin_lat + (j as f64 + 0.5) * cell_deg;
            for i in i_lo..i_hi {
                let lon = origin_lon + (i as f64 + 0.5) * cell_deg;
                if !mask[j * width + i] && point_in_polygon(lon, lat, poly) {
                    mask[j * width + i] = true;
                }
            }
        }
    }
    mask
}

// ============================================================================
// Strait carve-outs
// ============================================================================

/// One narrow waterway that point-in-polygon rasterisation closes at any
/// realistic [`SDF_RESOLUTION_DEG`]. Applied as a post-step on the binary
/// land mask in [`LandmassGrid::build`] before the distance transform,
/// so the SDF, gradient, A* sea grid, and PSO land-penalty all see the
/// strait as open consistently — without this the mask, the SDF derived
/// from it, and the search built on top would disagree.
#[derive(Debug, Clone, Copy)]
struct StraitCarveOut {
    name: &'static str,
    /// Channel centreline as `(lon°, lat°)` waypoints. Two points → one
    /// segment; more → a chain (kinked straits like the Dardanelles).
    waypoints: &'static [(f64, f64)],
    /// Radius around the centreline, in cells. ≥1.0 guarantees an
    /// 8-connected A* can walk through at the target resolution.
    /// Expressed in cells (not metres or degrees) so the same value
    /// scales naturally across resolutions.
    clearance_cells: f64,
}

impl StraitCarveOut {
    /// Bounding box enclosing every waypoint. Useful for sizing fine
    /// SDF patches whose bbox should cover the strait's region.
    /// Returns a degenerate (zero-extent) bbox at the single point for
    /// one-waypoint entries.
    fn waypoint_bbox(&self) -> LonLatBbox {
        let mut lon_min = f64::INFINITY;
        let mut lon_max = f64::NEG_INFINITY;
        let mut lat_min = f64::INFINITY;
        let mut lat_max = f64::NEG_INFINITY;
        for &(lon, lat) in self.waypoints {
            lon_min = lon_min.min(lon);
            lon_max = lon_max.max(lon);
            lat_min = lat_min.min(lat);
            lat_max = lat_max.max(lat);
        }
        LonLatBbox::new(lon_min, lon_max, lat_min, lat_max)
    }

    /// [`Self::waypoint_bbox`] expanded uniformly by `padding_deg` on
    /// each side. Latitude is not clamped; downstream patch builders
    /// clamp before allocating cells.
    fn padded_bbox(&self, padding_deg: f64) -> LonLatBbox {
        let b = self.waypoint_bbox();
        LonLatBbox::new(
            b.lon_min - padding_deg,
            b.lon_max + padding_deg,
            b.lat_min - padding_deg,
            b.lat_max + padding_deg,
        )
    }
}

/// Known narrow waterways below the rasterisation resolution.
///
/// Artificial canals (Suez, Panama) are intentionally omitted: adding
/// them would let the search shortcut continent-rounding routes.
const STRAIT_CARVE_OUTS: &[StraitCarveOut] = &[
    StraitCarveOut {
        name: "Strait of Gibraltar",
        waypoints: &[(-5.60, 35.95), (-5.30, 36.00)],
        clearance_cells: 1.2,
    },
    // Dardanelles + Sea of Marmara chained into Bosporus. The Marmara
    // is naturally sea but its coastal cells at 0.5° resolution can
    // straddle the Asian / European shores, so a diagonal A* chord
    // across the strait region clips ~14 km of land via bilinear-SDF
    // interpolation at the cell corners. Chaining the carve-outs
    // through the middle of the Marmara guarantees a continuously
    // positive SDF corridor between Aegean and Black Sea.
    StraitCarveOut {
        name: "Dardanelles + Sea of Marmara",
        waypoints: &[
            (26.15, 40.00),
            (26.40, 40.20),
            (26.70, 40.42),
            (27.50, 40.75),
            (28.50, 40.92),
            (28.97, 41.00),
        ],
        clearance_cells: 1.2,
    },
    StraitCarveOut {
        name: "Bosporus",
        waypoints: &[(28.97, 41.00), (29.07, 41.13), (29.15, 41.25)],
        clearance_cells: 1.2,
    },
    StraitCarveOut {
        name: "Bab-el-Mandeb",
        // Northern southern-Red-Sea approach narrows to ~50 km between
        // Eritrea and Yemen well before the strait proper, so the
        // carve-out has to begin upstream or A* clips coastal cells on
        // the diagonal approach.
        waypoints: &[
            (42.50, 14.50),
            (42.80, 13.80),
            (43.00, 13.20),
            (43.30, 12.55),
            (43.60, 12.50),
        ],
        clearance_cells: 1.2,
    },
    StraitCarveOut {
        name: "Strait of Malacca",
        waypoints: &[
            (95.50, 5.70),
            (98.50, 4.50),
            (100.50, 3.00),
            (102.50, 1.80),
            (103.40, 1.30),
        ],
        clearance_cells: 1.2,
    },
    StraitCarveOut {
        name: "Singapore Strait",
        waypoints: &[(103.40, 1.20), (103.85, 1.20), (104.40, 1.20)],
        clearance_cells: 1.2,
    },
];

/// Geometry of the mask buffer a carve-out is applied to. Separated from
/// the carve-out itself so the same `apply_strait_carve_outs` /
/// `carve_capsule` code drives both the global landmass grid (origin
/// `(-180°, -90°)`, `lon_wrap = true`) and bounded fine patches
/// (origin = patch's lower-left, `lon_wrap = false`).
#[derive(Copy, Clone, Debug)]
struct MaskFrame {
    width: usize,
    height: usize,
    cell_deg: f64,
    origin_lon: f64,
    origin_lat: f64,
    lon_wrap: bool,
}

impl MaskFrame {
    /// Frame for the global rasterisation. Origin sits at the
    /// `(-180°, -90°)` corner; lon wraps modulo width.
    fn global(width: usize, height: usize, cell_deg: f64) -> Self {
        Self {
            width,
            height,
            cell_deg,
            origin_lon: -180.0,
            origin_lat: -90.0,
            lon_wrap: true,
        }
    }
}

/// Apply every entry in [`STRAIT_CARVE_OUTS`] to a freshly rasterised
/// land mask. Each strait clears the cells within `clearance_cells` of
/// its centreline (capsule shape: segment + rounded endpoints).
///
/// Works on both the global grid and bounded fine patches via the
/// [`MaskFrame`] parameter. Carve-outs whose centrelines fall entirely
/// outside the frame still iterate but the per-cell distance test
/// silently drops them — cheap enough at the current `STRAIT_CARVE_OUTS`
/// size that a tube-bbox prefilter isn't worth the code.
fn apply_strait_carve_outs(mask: &mut [bool], frame: MaskFrame) {
    for strait in STRAIT_CARVE_OUTS {
        let pts = strait.waypoints;
        log::debug!(
            "carving strait {:?} ({} waypoints, clearance {} cells)",
            strait.name,
            pts.len(),
            strait.clearance_cells,
        );
        if pts.len() == 1 {
            carve_capsule(mask, frame, pts[0], pts[0], strait.clearance_cells);
        } else {
            for w in pts.windows(2) {
                carve_capsule(mask, frame, w[0], w[1], strait.clearance_cells);
            }
        }
    }
}

/// Force every cell within `radius` cells of the segment `a` → `b` to
/// sea, where `a` and `b` are `(lon°, lat°)`. Lon wraps modulo the
/// frame's width when `frame.lon_wrap` is true (global grid); otherwise
/// out-of-range columns are silently dropped (bounded patch). Lat clamps
/// in both cases. A segment whose endpoints straddle the antimeridian is
/// not handled — only Bering would need that, and it can be expressed as
/// two adjacent carve-outs on either side of ±180.
fn carve_capsule(
    mask: &mut [bool],
    frame: MaskFrame,
    a: (f64, f64),
    b: (f64, f64),
    radius: f64,
) {
    let MaskFrame {
        width,
        height,
        cell_deg,
        origin_lon,
        origin_lat,
        lon_wrap,
    } = frame;
    let to_cell = |lon: f64, lat: f64| -> (f64, f64) {
        let cx = if lon_wrap {
            (lon - origin_lon).rem_euclid(360.0) / cell_deg - 0.5
        } else {
            (lon - origin_lon) / cell_deg - 0.5
        };
        let cy = (lat - origin_lat) / cell_deg - 0.5;
        (cx, cy)
    };
    let (ax, ay) = to_cell(a.0, a.1);
    let (bx, by) = to_cell(b.0, b.1);

    let lo_i = (ax.min(bx) - radius).floor() as isize;
    let hi_i = (ax.max(bx) + radius).ceil() as isize;
    let lo_j = (ay.min(by) - radius).floor().max(0.0) as isize;
    let hi_j = (ay.max(by) + radius)
        .ceil()
        .min((height - 1) as f64) as isize;
    let r2 = radius * radius;

    for j in lo_j..=hi_j {
        for ci in lo_i..=hi_i {
            let i = if lon_wrap {
                ci.rem_euclid(width as isize) as usize
            } else if ci < 0 || (ci as usize) >= width {
                continue;
            } else {
                ci as usize
            };
            let d2 = point_to_segment_dist_sq(i as f64, j as f64, ax, ay, bx, by);
            if d2 <= r2 {
                mask[j as usize * width + i] = false;
            }
        }
    }
}

/// Squared distance from `(px, py)` to the segment `(ax, ay) → (bx, by)`.
/// All coordinates are in fractional cell space. Falls back to point
/// distance when the segment has zero length.
fn point_to_segment_dist_sq(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let dx = bx - ax;
    let dy = by - ay;
    let len_sq = dx.mul_add(dx, dy * dy);
    let t = if len_sq < 1e-12 {
        0.0
    } else {
        (((px - ax) * dx + (py - ay) * dy) / len_sq).clamp(0.0, 1.0)
    };
    let qx = ax + t * dx;
    let qy = ay + t * dy;
    let ex = px - qx;
    let ey = py - qy;
    ex.mul_add(ex, ey * ey)
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
    /// shapes. Applies [`STRAIT_CARVE_OUTS`] to the rasterised mask
    /// before the distance transform so the SDF, gradient, A* sea grid,
    /// and PSO land-penalty all see the listed straits as passable.
    pub fn build(polygons: &[Polygon], cell_deg: f64) -> Self {
        assert!(cell_deg > 0.0, "cell_deg must be positive, got {cell_deg}");
        let width = (360.0 / cell_deg).round() as usize;
        let height = (180.0 / cell_deg).round() as usize;
        let frame = MaskFrame::global(width, height, cell_deg);
        let mut mask = rasterise_mask(polygons, frame);
        apply_strait_carve_outs(&mut mask, frame);
        Self::from_mask(&mask, width, height, cell_deg)
    }

    /// Build a grid directly from a precomputed binary mask. Primarily
    /// for tests; production code goes through [`build`](Self::build).
    pub fn from_mask(mask: &[bool], width: usize, height: usize, cell_deg: f64) -> Self {
        assert_eq!(mask.len(), width * height, "mask length mismatch");
        let frame = MaskFrame::global(width, height, cell_deg);
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

        let grad_en = compute_gradient(&sdf_m, frame);

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

    /// Cell size in degrees of lon / lat.
    pub fn cell_deg(&self) -> f64 {
        self.cell_deg
    }

    /// Grid dimensions in cells: `(width, height)` along lon and lat.
    pub fn dims(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    /// Signed distance at cell `(i, j)`, in metres. Negative inside
    /// land, positive over water, zero on the cell-precision coastline.
    /// Out-of-range indices panic.
    pub fn sdf_at_cell(&self, i: usize, j: usize) -> f32 {
        self.sdf_m[self.cell_idx(i, j)]
    }

    /// Centre `(lon°, lat°)` of cell `(i, j)`.
    pub fn cell_centre(&self, i: usize, j: usize) -> LatLon {
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

    /// Half a cell's worth of metres at the equator. Picks the substep
    /// at which midpoint sampling in
    /// [`swarmkit_sailing::dynamics::get_segment_land_metres`] can detect
    /// the smallest land feature this grid actually resolves: any land
    /// at least one cell wide guarantees a substep whose midpoint sits
    /// inside a negative-SDF cell.
    fn sampling_step_metres(&self) -> f64 {
        self.cell_deg * METRES_PER_DEGREE * 0.5
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

// ============================================================================
// FinePatch: bounded high-resolution SDF tile
// ============================================================================

/// Rectangular SDF + gradient tile at a fine resolution, covering a
/// caller-chosen bbox.
///
/// Used as the fine tier in [`TwoTierLandmass`] so strait carve-out
/// tubes inside the patch are narrower (proportional to the smaller
/// cell size) than they are on the coarse global grid.
///
/// Indexed relative to `bbox.lon_min / bbox.lat_min` (no antimeridian
/// wrap — patches are local rectangles). The bilinear lookup assumes
/// the query point sits inside `bbox`; callers must filter via
/// `bbox.contains(p)` before consulting a patch.
///
/// SDF values near the patch edge are biased low (the in-patch distance
/// transform can't see land outside the patch, so a sea cell near the
/// boundary may underestimate its true distance to land). Patches built
/// from the [`StraitCarveOut::padded_bbox`] helper have a configurable
/// padding so the search-relevant cells sit well inside the patch
/// interior and avoid edge bias.
pub struct FinePatch {
    bbox: LonLatBbox,
    cell_deg: f64,
    width: usize,
    height: usize,
    sdf_m: Vec<f32>,
    grad_en: Vec<(f32, f32)>,
}

impl FinePatch {
    /// Build a fine-resolution patch over `bbox` at `cell_deg`.
    /// Rasterises any polygons that intersect the bbox, applies the
    /// global [`STRAIT_CARVE_OUTS`] (those whose tubes touch the patch
    /// will write into the mask via the per-cell distance test inside
    /// `carve_capsule`; the rest no-op), runs the 8SSEDT distance
    /// transform within the patch, and computes the outward gradient.
    pub fn build(polygons: &[Polygon], bbox: LonLatBbox, cell_deg: f64) -> Self {
        assert!(cell_deg > 0.0, "cell_deg must be positive, got {cell_deg}");
        assert!(
            bbox.is_non_degenerate(),
            "patch bbox must be non-degenerate, got {bbox:?}",
        );
        let lon_extent = bbox.lon_extent();
        let lat_extent = bbox.lat_extent();
        let width = (lon_extent / cell_deg).ceil() as usize;
        let height = (lat_extent / cell_deg).ceil() as usize;
        let frame = MaskFrame {
            width,
            height,
            cell_deg,
            origin_lon: bbox.lon_min,
            origin_lat: bbox.lat_min,
            lon_wrap: false,
        };

        let mut mask = rasterise_mask(polygons, frame);
        apply_strait_carve_outs(&mut mask, frame);

        let to_land = distance_transform(&mask, width, height, false);
        let inverted: Vec<bool> = mask.iter().map(|&b| !b).collect();
        let to_water = distance_transform(&inverted, width, height, false);
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
        let grad_en = compute_gradient(&sdf_m, frame);

        Self {
            bbox,
            cell_deg,
            width,
            height,
            sdf_m,
            grad_en,
        }
    }

    /// Geographic bbox covered by this patch.
    pub fn bbox(&self) -> LonLatBbox {
        self.bbox
    }

    /// Cell size in degrees.
    pub fn cell_deg(&self) -> f64 {
        self.cell_deg
    }

    /// Grid dimensions in cells: `(width, height)`.
    pub fn dims(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    /// Signed distance at cell `(i, j)`. Out-of-range panics.
    pub fn sdf_at_cell(&self, i: usize, j: usize) -> f32 {
        self.sdf_m[j * self.width + i]
    }

    /// Centre `(lon°, lat°)` of cell `(i, j)`.
    pub fn cell_centre(&self, i: usize, j: usize) -> LatLon {
        LatLon::new(
            self.bbox.lon_min + (i as f64 + 0.5) * self.cell_deg,
            self.bbox.lat_min + (j as f64 + 0.5) * self.cell_deg,
        )
    }

    /// Snap a geographic `(lon°, lat°)` to its containing cell index
    /// `(i, j)`. Clamps to the patch's edge cells; callers must already
    /// have verified `bbox.contains(location)` before calling.
    fn cell_index_of(&self, location: LatLon) -> (usize, usize) {
        let fi = ((location.lon - self.bbox.lon_min) / self.cell_deg)
            .floor()
            .clamp(0.0, (self.width.saturating_sub(1)) as f64);
        let fj = ((location.lat - self.bbox.lat_min) / self.cell_deg)
            .floor()
            .clamp(0.0, (self.height.saturating_sub(1)) as f64);
        (fi as usize, fj as usize)
    }

    /// `true` iff cell `(i, j)` is sea (positive SDF).
    fn is_sea_cell(&self, i: usize, j: usize) -> bool {
        self.sdf_m[j * self.width + i] >= 0.0
    }

    /// Bilinear-interpolated SDF at `(lon°, lat°)`. Caller must ensure
    /// the point lies in `bbox` — out-of-bbox queries clamp to the
    /// patch's nearest edge cell and the returned value is meaningless.
    pub fn signed_distance_m(&self, location: LatLon) -> f64 {
        f64::from(self.bilinear(location.lon, location.lat, |i, j| {
            self.sdf_m[j * self.width + i]
        }))
    }

    /// Bilinear-interpolated outward gradient at `(lon°, lat°)`. Same
    /// in-bbox precondition as [`Self::signed_distance_m`].
    pub fn gradient(&self, location: LatLon) -> TangentMetres {
        let east = self.bilinear(location.lon, location.lat, |i, j| {
            self.grad_en[j * self.width + i].0
        });
        let north = self.bilinear(location.lon, location.lat, |i, j| {
            self.grad_en[j * self.width + i].1
        });
        TangentMetres::new(f64::from(east), f64::from(north))
    }

    /// Patch-local bilinear lookup. Lat / lon clamp to the cell-centre
    /// range; the caller is expected to bbox-check first via [`Self::bbox`].
    fn bilinear<F: Fn(usize, usize) -> f32>(&self, lon: f64, lat: f64, sample: F) -> f32 {
        let fi = ((lon - self.bbox.lon_min) / self.cell_deg - 0.5)
            .clamp(0.0, (self.width - 1) as f64);
        let fj = ((lat - self.bbox.lat_min) / self.cell_deg - 0.5)
            .clamp(0.0, (self.height - 1) as f64);
        let i0 = fi.floor() as usize;
        let j0 = fj.floor() as usize;
        let i1 = (i0 + 1).min(self.width - 1);
        let j1 = (j0 + 1).min(self.height - 1);
        let alpha = (fi - fi.floor()) as f32;
        let beta = (fj - fj.floor()) as f32;
        let s00 = sample(i0, j0);
        let s10 = sample(i1, j0);
        let s01 = sample(i0, j1);
        let s11 = sample(i1, j1);
        let s0 = s00 * (1.0 - alpha) + s10 * alpha;
        let s1 = s01 * (1.0 - alpha) + s11 * alpha;
        s0 * (1.0 - beta) + s1 * beta
    }
}

// ============================================================================
// TwoTierLandmass: coarse global grid + fine regional patches
// ============================================================================

/// Coarse global [`LandmassGrid`] plus a vector of fine [`FinePatch`]es.
///
/// Lookups (`signed_distance_m`, `gradient`) scan the fine patches in
/// declaration order and return the first hit; anything outside every
/// patch falls through to the coarse grid. `find_sea_path` delegates to
/// the coarse grid entirely — A\* expansion stays on a uniform graph
/// (Phase 7 of the two-tier plan upgrades this so the benchmark route
/// also threads the narrow fine tubes).
///
/// Patches are stored in `Vec`; a linear scan is fine for the expected
/// 4–8 patches derived from [`STRAIT_CARVE_OUTS`]. If patch count grows
/// past ~32 a bucket index keyed on lat band would amortise.
pub struct TwoTierLandmass {
    coarse: LandmassGrid,
    fine: Vec<FinePatch>,
    /// Cumulative flat-index offsets for the unified-graph A\*. Element
    /// `k` is the first cell index of `fine[k]`; `fine_offsets.last()`
    /// is the total cell count. Length is `fine.len() + 1`.
    fine_offsets: Vec<usize>,
}

impl TwoTierLandmass {
    /// Construct from a pre-built coarse grid and the set of fine
    /// patches. Test-shaped entry point; production code goes through
    /// `landmass_grid_two_tier` once that lands in Phase 4.
    pub fn new(coarse: LandmassGrid, fine: Vec<FinePatch>) -> Self {
        let mut fine_offsets = Vec::with_capacity(fine.len() + 1);
        let (cw, ch) = coarse.dims();
        let mut acc = cw * ch;
        fine_offsets.push(acc);
        for p in &fine {
            let (w, h) = p.dims();
            acc += w * h;
            fine_offsets.push(acc);
        }
        Self {
            coarse,
            fine,
            fine_offsets,
        }
    }

    /// Borrow the coarse global grid. Used by the viz overlay so cells
    /// outside every fine patch still paint.
    pub fn coarse(&self) -> &LandmassGrid {
        &self.coarse
    }

    /// Borrow the fine patches. Used by the viz overlay to draw the
    /// finer grid inside each fine bbox on top of the coarse cells.
    pub fn fine_patches(&self) -> &[FinePatch] {
        &self.fine
    }
}

/// Decoded form of a flat unified-graph cell index.
///
/// The flat index space lays cells out as `[coarse, fine[0], fine[1],
/// ...]` — the offsets are cached on the [`TwoTierLandmass`] in
/// `fine_offsets`. A\* operates on flat `usize` indices so its
/// `closed` / `g_score` `Vec`s can be allocated once; this enum is
/// just for decoding inside the per-cell hot path.
#[derive(Copy, Clone, Debug)]
enum TwoTierCell {
    Coarse { i: usize, j: usize },
    Fine { patch: usize, i: usize, j: usize },
}

impl TwoTierLandmass {
    /// Total flat cell count: coarse + every fine patch.
    fn total_cells(&self) -> usize {
        *self.fine_offsets.last().expect("fine_offsets non-empty")
    }

    fn encode_cell(&self, cell: TwoTierCell) -> usize {
        match cell {
            TwoTierCell::Coarse { i, j } => {
                let (w, _) = self.coarse.dims();
                j * w + i
            }
            TwoTierCell::Fine { patch, i, j } => {
                let (w, _) = self.fine[patch].dims();
                self.fine_offsets[patch] + j * w + i
            }
        }
    }

    #[expect(
        clippy::unreachable,
        reason = "the loop exhausts every fine_offsets range and the leading \
                  check covers coarse, so the trailing call is only reached \
                  via an out-of-bounds idx — a caller bug we want to panic on \
                  loudly rather than swallow."
    )]
    fn decode_cell(&self, idx: usize) -> TwoTierCell {
        let coarse_size = self.fine_offsets[0];
        if idx < coarse_size {
            let (w, _) = self.coarse.dims();
            return TwoTierCell::Coarse {
                i: idx % w,
                j: idx / w,
            };
        }
        for (k, p) in self.fine.iter().enumerate() {
            if idx < self.fine_offsets[k + 1] {
                let (w, _) = p.dims();
                let local = idx - self.fine_offsets[k];
                return TwoTierCell::Fine {
                    patch: k,
                    i: local % w,
                    j: local / w,
                };
            }
        }
        unreachable!("idx out of range: {idx} (total {})", self.total_cells())
    }

    /// Flat index of the active cell containing `location`. Fine patch
    /// hits override the coarse fallback; patches are checked in order.
    fn active_cell_at(&self, location: LatLon) -> usize {
        let p = LatLon::new(wrap_lon_deg(location.lon), location.lat);
        for (k, patch) in self.fine.iter().enumerate() {
            if patch.bbox().contains(p) {
                let (i, j) = patch.cell_index_of(p);
                return self.encode_cell(TwoTierCell::Fine { patch: k, i, j });
            }
        }
        let coarse = &self.coarse;
        let (w, h) = coarse.dims();
        let fi = ((p.lon + 180.0).rem_euclid(360.0)) / coarse.cell_deg() - 0.5;
        let fj = (p.lat + 90.0) / coarse.cell_deg() - 0.5;
        let i = (fi + 0.5).floor() as isize;
        let j = (fj + 0.5).floor().clamp(0.0, (h - 1) as f64) as isize;
        let i = i.rem_euclid(w as isize) as usize;
        let j = (j as usize).min(h - 1);
        self.encode_cell(TwoTierCell::Coarse { i, j })
    }

    fn cell_centre_at_idx(&self, idx: usize) -> LatLon {
        match self.decode_cell(idx) {
            TwoTierCell::Coarse { i, j } => self.coarse.cell_centre(i, j),
            TwoTierCell::Fine { patch, i, j } => self.fine[patch].cell_centre(i, j),
        }
    }

    fn is_sea_at_idx(&self, idx: usize) -> bool {
        match self.decode_cell(idx) {
            TwoTierCell::Coarse { i, j } => self.coarse.sdf_at_cell(i, j) >= 0.0,
            TwoTierCell::Fine { patch, i, j } => self.fine[patch].is_sea_cell(i, j),
        }
    }

    /// 8-connected neighbours of `idx` at the cell's own tier
    /// resolution, each remapped to the active cell containing the
    /// neighbour's geographic position. Crossing a fine-bbox boundary
    /// hops between tiers naturally — a fine cell's outside neighbours
    /// land on the coarse cell containing them and vice versa.
    fn neighbour_cells(&self, idx: usize) -> impl Iterator<Item = usize> + '_ {
        let cell = self.decode_cell(idx);
        let (centre, cell_deg) = match cell {
            TwoTierCell::Coarse { i, j } => (self.coarse.cell_centre(i, j), self.coarse.cell_deg()),
            TwoTierCell::Fine { patch, i, j } => {
                (self.fine[patch].cell_centre(i, j), self.fine[patch].cell_deg())
            }
        };
        [-1isize, 0, 1].into_iter().flat_map(move |dj| {
            [-1isize, 0, 1].into_iter().filter_map(move |di| {
                if di == 0 && dj == 0 {
                    return None;
                }
                let n_lat = centre.lat + f64::from(di32(dj)) * cell_deg;
                if !(-90.0..=90.0).contains(&n_lat) {
                    return None;
                }
                let n_lon = wrap_lon_deg(centre.lon + f64::from(di32(di)) * cell_deg);
                Some(self.active_cell_at(LatLon::new(n_lon, n_lat)))
            })
        })
    }
}

#[inline]
fn di32(x: isize) -> i32 {
    x as i32
}

impl LandmassSource for TwoTierLandmass {
    fn signed_distance_m(&self, location: LatLon) -> f64 {
        let lon = wrap_lon_deg(location.lon);
        let p = LatLon::new(lon, location.lat);
        for patch in &self.fine {
            if patch.bbox().contains(p) {
                return patch.signed_distance_m(p);
            }
        }
        self.coarse.signed_distance_m(location)
    }

    fn gradient(&self, location: LatLon) -> TangentMetres {
        let lon = wrap_lon_deg(location.lon);
        let p = LatLon::new(lon, location.lat);
        for patch in &self.fine {
            if patch.bbox().contains(p) {
                return patch.gradient(p);
            }
        }
        self.coarse.gradient(location)
    }

    /// Use the finest cell-size in the two-tier configuration to drive
    /// substep cadence. This forces open-ocean chords to be sampled at
    /// fine resolution too, which is more expensive but keeps the
    /// substep budget consistent regardless of whether a given chord
    /// happens to enter a fine patch. Phase 7 / smart-substep would
    /// recover the open-ocean cost.
    fn sampling_step_metres(&self) -> f64 {
        let fine_min = self
            .fine
            .iter()
            .map(FinePatch::cell_deg)
            .fold(f64::INFINITY, f64::min);
        let min_cell = fine_min.min(self.coarse.cell_deg());
        min_cell * METRES_PER_DEGREE * 0.5
    }

    fn find_sea_path(
        &self,
        origin: LatLon,
        destination: LatLon,
        bounds: &RouteBounds,
        bias: SeaPathBias,
    ) -> Option<Vec<LatLon>> {
        astar_sea_path_two_tier(self, origin, destination, bounds, bias)
    }
}

/// Outward unit gradient at every cell, derived by central differences
/// on the signed distance field. Sign convention: positive = away from
/// land. Magnitude is normalised to ~1 wherever the SDF is locally
/// non-flat; degenerates to zero at saddle points.
///
/// The `cos(lat)` east-axis shrinkage uses each cell's actual latitude
/// derived from `frame.origin_lat`, so fine patches at high latitudes
/// get the correct east-cell metric. `frame.lon_wrap` controls whether
/// the left/right neighbours of edge columns wrap around (global grid)
/// or clamp via one-sided differences (bounded patch).
fn compute_gradient(sdf_m: &[f32], frame: MaskFrame) -> Vec<(f32, f32)> {
    let MaskFrame {
        width,
        height,
        cell_deg,
        origin_lat,
        lon_wrap,
        ..
    } = frame;
    let mut grad = vec![(0.0f32, 0.0f32); width * height];
    let cell_m = cell_deg * METRES_PER_DEGREE;
    for j in 0..height {
        let lat = origin_lat + (j as f64 + 0.5) * cell_deg;
        let cos_lat = lat.to_radians().cos().max(1e-9);
        // Central differences in *cells*. Convert to metres using the
        // east-cell metre length at this latitude (cos(lat) shrinkage)
        // and the lat-cell metre length (constant on a sphere).
        let inv_dx_m = 1.0 / (2.0 * cell_m * cos_lat);
        let inv_dy_m = 1.0 / (2.0 * cell_m);
        for i in 0..width {
            // Lon wraps on the global grid; on bounded patches we clamp
            // at the edge columns via one-sided differences. Lat always
            // clamps at the poles (and at the patch lat edges).
            let (il, ir) = if lon_wrap {
                (
                    ((i as isize - 1).rem_euclid(width as isize)) as usize,
                    ((i as isize + 1).rem_euclid(width as isize)) as usize,
                )
            } else {
                (i.saturating_sub(1), (i + 1).min(width - 1))
            };
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
// Two-tier A* (unified coarse + fine cell graph)
// ============================================================================

/// BFS-snap an arbitrary `(lon, lat)` to the nearest sea cell in the
/// unified two-tier graph. Returns the flat cell index, or `None` if
/// no sea cell is reachable within [`SNAP_TO_SEA_MAX_RING`] expansion
/// hops.
fn snap_to_sea_cell_two_tier(two_tier: &TwoTierLandmass, location: LatLon) -> Option<usize> {
    let start = two_tier.active_cell_at(location);
    if two_tier.is_sea_at_idx(start) {
        return Some(start);
    }
    let mut visited = vec![false; two_tier.total_cells()];
    let mut queue = VecDeque::new();
    visited[start] = true;
    queue.push_back((start, 0usize));
    while let Some((idx, ring)) = queue.pop_front() {
        if two_tier.is_sea_at_idx(idx) {
            return Some(idx);
        }
        if ring >= SNAP_TO_SEA_MAX_RING {
            continue;
        }
        for neighbour in two_tier.neighbour_cells(idx) {
            if !visited[neighbour] {
                visited[neighbour] = true;
                queue.push_back((neighbour, ring + 1));
            }
        }
    }
    None
}

/// Bias barrier check for the unified-graph A*. Identical semantics to
/// [`bias_allows`], just operating on flat cell indices.
fn bias_allows_two_tier(
    two_tier: &TwoTierLandmass,
    bounds: &RouteBounds,
    start: usize,
    goal: usize,
    cell: usize,
    bias: SeaPathBias,
) -> bool {
    if cell == start || cell == goal {
        return true;
    }
    let centre = two_tier.cell_centre_at_idx(cell);
    let line_lat = line_lat_at_lon(bounds, centre.lon);
    let offset = centre.lat - line_lat;
    match bias {
        SeaPathBias::None => true,
        SeaPathBias::North => offset >= -BIAS_BARRIER_SLACK_DEG,
        SeaPathBias::South => offset <= BIAS_BARRIER_SLACK_DEG,
    }
}

/// A\* over the unified two-tier cell graph: coarse outside fine
/// bboxes, fine inside. Each cell's neighbours are 8-connected at the
/// cell's own tier resolution, remapped to whichever active cell
/// contains each neighbour's geographic position — so at a fine-bbox
/// boundary the path naturally hops between tiers. Edge cost is
/// great-circle distance between cell centres; heuristic is
/// great-circle to the goal cell.
fn astar_sea_path_two_tier(
    two_tier: &TwoTierLandmass,
    origin: LatLon,
    destination: LatLon,
    bounds: &RouteBounds,
    bias: SeaPathBias,
) -> Option<Vec<LatLon>> {
    let start = snap_to_sea_cell_two_tier(two_tier, origin)?;
    let goal = snap_to_sea_cell_two_tier(two_tier, destination)?;
    if start == goal {
        return Some(vec![origin, destination]);
    }

    let n_cells = two_tier.total_cells();
    let mut g_score = vec![f64::INFINITY; n_cells];
    let mut came_from = vec![u32::MAX; n_cells];
    let mut closed = vec![false; n_cells];
    let mut open: BinaryHeap<AStarNode> = BinaryHeap::new();

    let start_centre = two_tier.cell_centre_at_idx(start);
    let goal_centre = two_tier.cell_centre_at_idx(goal);
    g_score[start] = 0.0;
    open.push(AStarNode {
        f_score: haversine(start_centre, goal_centre),
        cell: u32::try_from(start).expect("cell index fits in u32"),
    });

    let mut expansions = 0usize;
    while let Some(AStarNode { cell, .. }) = open.pop() {
        let cell = cell as usize;
        if closed[cell] {
            continue;
        }
        closed[cell] = true;
        if cell == goal {
            return Some(reconstruct_polyline_two_tier(
                two_tier,
                &came_from,
                cell,
                origin,
                destination,
            ));
        }
        expansions += 1;
        if expansions > ASTAR_MAX_EXPANSIONS {
            log::warn!(
                "find_sea_path (two-tier): A* aborted after {ASTAR_MAX_EXPANSIONS} expansions \
                 (origin={origin:?}, destination={destination:?})",
            );
            return None;
        }
        let cur_centre = two_tier.cell_centre_at_idx(cell);
        for neighbour in two_tier.neighbour_cells(cell) {
            if !two_tier.is_sea_at_idx(neighbour) {
                continue;
            }
            if !bias_allows_two_tier(two_tier, bounds, start, goal, neighbour, bias) {
                continue;
            }
            let neighbour_centre = two_tier.cell_centre_at_idx(neighbour);
            if neighbour != start
                && neighbour != goal
                && !bounds.bbox.contains(neighbour_centre)
            {
                continue;
            }
            let step = haversine(cur_centre, neighbour_centre);
            let tentative_g = g_score[cell] + step;
            if tentative_g < g_score[neighbour] {
                g_score[neighbour] = tentative_g;
                came_from[neighbour] = u32::try_from(cell).expect("cell index fits in u32");
                let h = haversine(neighbour_centre, goal_centre);
                open.push(AStarNode {
                    f_score: tentative_g + h,
                    cell: u32::try_from(neighbour).expect("cell index fits in u32"),
                });
            }
        }
    }
    None
}

fn reconstruct_polyline_two_tier(
    two_tier: &TwoTierLandmass,
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
    for cell in cells {
        polyline.push(two_tier.cell_centre_at_idx(cell));
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

/// Two-tier landmass at the requested coarse and fine cell sizes.
///
/// Cached per distinct `(coarse, fine)` pair for the program's lifetime
/// (same `OnceLock + Mutex` pattern as [`landmass_grid_at_resolution`]).
///
/// The coarse grid is global at `coarse_deg`; fine patches cover each
/// [`STRAIT_CARVE_OUTS`] entry's waypoint chain padded by
/// `max(1.0°, 2 × coarse_deg)`. Overlapping padded bboxes are merged so
/// the Turkish-straits chain (Dardanelles + Sea of Marmara + Bosporus)
/// ends up as one patch rather than three.
pub fn landmass_grid_two_tier(
    coarse_deg: f64,
    fine_deg: f64,
) -> &'static TwoTierLandmass {
    static REGISTRY: OnceLock<Mutex<HashMap<(u64, u64), &'static TwoTierLandmass>>> =
        OnceLock::new();
    let registry = REGISTRY.get_or_init(|| Mutex::new(HashMap::new()));
    let key = (coarse_deg.to_bits(), fine_deg.to_bits());
    let mut guard = match registry.lock() {
        Ok(g) => g,
        Err(poison) => poison.into_inner(),
    };
    if let Some(grid) = guard.get(&key) {
        return grid;
    }
    let built: &'static TwoTierLandmass =
        Box::leak(Box::new(build_two_tier(coarse_deg, fine_deg)));
    guard.insert(key, built);
    built
}

fn build_two_tier(coarse_deg: f64, fine_deg: f64) -> TwoTierLandmass {
    let polygons = raw_polygons();
    let coarse = LandmassGrid::build(polygons, coarse_deg);
    let padding_deg = (2.0 * coarse_deg).max(1.0);
    let padded_bboxes: Vec<LonLatBbox> = STRAIT_CARVE_OUTS
        .iter()
        .map(|c| c.padded_bbox(padding_deg))
        .collect();
    let merged = merge_overlapping_bboxes(padded_bboxes);
    log::debug!(
        "two-tier landmass: coarse={coarse_deg}°, fine={fine_deg}°, \
         padding={padding_deg}°, {} fine patch(es) from {} carve-out(s)",
        merged.len(),
        STRAIT_CARVE_OUTS.len(),
    );
    let fine: Vec<FinePatch> = merged
        .into_iter()
        .map(|bbox| FinePatch::build(polygons, bbox, fine_deg))
        .collect();
    TwoTierLandmass::new(coarse, fine)
}

/// Greedy bbox union: repeatedly fold any two overlapping bboxes into
/// one and stop when no more overlaps remain. O(n²) per pass, O(n³)
/// worst case overall — fine for the current `STRAIT_CARVE_OUTS` size
/// (≤ ~10 entries). Assumes no input bbox crosses the antimeridian
/// (none of the existing carve-outs do).
fn merge_overlapping_bboxes(mut boxes: Vec<LonLatBbox>) -> Vec<LonLatBbox> {
    loop {
        let mut merged_one = false;
        'outer: for i in 0..boxes.len() {
            for j in (i + 1)..boxes.len() {
                if bboxes_overlap(boxes[i], boxes[j]) {
                    let union = bbox_union(boxes[i], boxes[j]);
                    boxes.swap_remove(j);
                    boxes[i] = union;
                    merged_one = true;
                    break 'outer;
                }
            }
        }
        if !merged_one {
            break;
        }
    }
    boxes
}

/// Axis-aligned rectangle overlap on `(lon, lat)`. Boundary touches
/// count as overlapping — adjacent carve-out bboxes should merge.
/// Antimeridian-crossing inputs are not handled.
fn bboxes_overlap(a: LonLatBbox, b: LonLatBbox) -> bool {
    !(a.lat_max < b.lat_min
        || b.lat_max < a.lat_min
        || a.lon_max < b.lon_min
        || b.lon_max < a.lon_min)
}

fn bbox_union(a: LonLatBbox, b: LonLatBbox) -> LonLatBbox {
    LonLatBbox::new(
        a.lon_min.min(b.lon_min),
        a.lon_max.max(b.lon_max),
        a.lat_min.min(b.lat_min),
        a.lat_max.max(b.lat_max),
    )
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
    fn carved_strait_waypoints_are_sea_in_default_grid() {
        let grid = landmass_grid();
        for strait in STRAIT_CARVE_OUTS {
            for &(lon, lat) in strait.waypoints {
                let sd = grid.signed_distance_m(LatLon::new(lon, lat));
                assert!(
                    sd >= 0.0,
                    "{}: waypoint ({lon}, {lat}) classified as land (SDF = {sd} m)",
                    strait.name,
                );
            }
        }
    }

    #[test]
    fn aegean_to_black_sea_path_traverses_the_turkish_straits() {
        // Without the Dardanelles + Bosporus carve-outs, A* would either
        // fail (Black Sea fully enclosed) or fall back to the snap-to-sea
        // ring radius and produce a degenerate path. With both straits
        // open, a real polyline must exist.
        let grid = landmass_grid();
        let aegean = LatLon::new(25.0, 39.0);
        let black_sea = LatLon::new(31.0, 42.5);
        let bounds = RouteBounds::new(
            aegean,
            black_sea,
            LonLatBbox::new(22.0, 35.0, 36.0, 46.0),
        );
        let path = grid
            .find_sea_path(aegean, black_sea, &bounds, SeaPathBias::None)
            .expect("Turkish straits should be open after carve-out");
        let max_lon = path.iter().map(|p| p.lon).fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (26.0..=32.0).contains(&max_lon),
            "expected path to thread the Turkish straits, max lon = {max_lon}",
        );
        let total_land: f64 = path
            .windows(2)
            .map(|w| get_segment_land_metres(grid, w[0], w[1], bounds.step_distance_max))
            .sum();
        assert_eq!(
            total_land, 0.0,
            "polyline crossed {total_land:.0} m of land threading Dardanelles + Bosporus",
        );
    }

    #[test]
    fn sinai_scenario_unbiased_path_rounds_africa() {
        // Coordinates from `grib-data/sinai-land-crossing.toml`. Both
        // biased searches return `None` here — the path out of the Med
        // must briefly travel north of `line_lat + slack` to clear the
        // Sicily-Tunisia narrows, which the bias barrier blocks. The
        // PSO init layer falls back to the unbiased polyline (see
        // `swarmkit_sailing::init::compute_baselines`), so this test
        // pins that the unbiased call succeeds and the polyline really
        // does round Africa (min lat well south of the equator).
        let grid = landmass_grid();
        let origin = LatLon::new(12.894705772399902, 36.113094329833984);
        let destination = LatLon::new(55.21006393432617, 10.023117065429688);
        let bounds = RouteBounds::new(
            origin,
            destination,
            LonLatBbox::new(-32.35, 69.85, -49.85, 52.35),
        );
        let polyline = grid
            .find_sea_path(origin, destination, &bounds, SeaPathBias::None)
            .expect("unbiased A* must succeed for the init fallback to work");
        let min_lat = polyline.iter().map(|p| p.lat).fold(f64::INFINITY, f64::min);
        assert!(
            min_lat < -30.0,
            "expected an around-Africa detour (min lat < -30°), got min lat = {min_lat}",
        );
        let total_land: f64 = polyline
            .windows(2)
            .map(|w| get_segment_land_metres(grid, w[0], w[1], bounds.step_distance_max))
            .sum();
        assert_eq!(
            total_land, 0.0,
            "around-Africa polyline crossed {total_land:.0} m of land",
        );
    }

    #[test]
    fn land_metres_detect_sinai_on_long_med_to_arabian_chord() {
        // Regression: a Mediterranean → Arabian-Sea great-circle chord
        // cuts across Egypt and Saudi Arabia. With a wind-sized substep
        // (1% of a continental bbox ≈ 160 km), the old midpoint sampler
        // only got 1 substep over a ~150 km PSO segment — its single
        // midpoint frequently landed in a sea cell while the chord
        // actually traversed hundreds of km of desert. The fix clamps
        // the substep to the grid's native cell-scale so a chord like
        // this is sampled densely enough to register the crossing.
        let grid = landmass_grid();
        let mediterranean = LatLon::new(12.89, 36.11);
        let arabian_sea = LatLon::new(55.21, 10.02);
        // The Sinai scenario's bbox-derived step (~160 km). Old code
        // path would underreport with this value; the new clamp inside
        // `get_segment_land_metres` kicks in regardless.
        let coarse_step_m = 160_000.0;
        let land_m =
            swarmkit_sailing::get_segment_land_metres(grid, mediterranean, arabian_sea, coarse_step_m);
        // The chord physically traverses roughly 2 500–3 500 km of land
        // (Libya, Egypt, Saudi Arabia, Yemen) out of ~6 000 km total.
        // Even a conservative lower bound of 1 000 km would have been
        // missed by the old single-midpoint sampler on PSO-scale
        // segments.
        assert!(
            land_m > 1_000_000.0,
            "expected >1 000 km of land on the Med → Arabian-Sea chord, got {land_m} m",
        );
    }

    #[test]
    fn atlantic_to_mediterranean_path_traverses_gibraltar() {
        let grid = landmass_grid();
        let atlantic = LatLon::new(-7.0, 36.0);
        let mediterranean = LatLon::new(-3.0, 36.0);
        let bounds = RouteBounds::new(
            atlantic,
            mediterranean,
            LonLatBbox::new(-10.0, 5.0, 33.0, 40.0),
        );
        let path = grid
            .find_sea_path(atlantic, mediterranean, &bounds, SeaPathBias::None)
            .expect("Gibraltar should be open after carve-out");
        // Path should stay roughly along lat 36 — no detour around all
        // of Africa.
        let max_lat = path.iter().map(|p| p.lat).fold(f64::NEG_INFINITY, f64::max);
        let min_lat = path.iter().map(|p| p.lat).fold(f64::INFINITY, f64::min);
        assert!(
            min_lat > 33.0 && max_lat < 40.0,
            "Gibraltar path detoured (lat range {min_lat}..{max_lat})",
        );
        let total_land: f64 = path
            .windows(2)
            .map(|w| get_segment_land_metres(grid, w[0], w[1], bounds.step_distance_max))
            .sum();
        assert_eq!(
            total_land, 0.0,
            "Gibraltar polyline crossed {total_land:.0} m of land",
        );
    }

    #[test]
    fn two_tier_outside_patches_falls_back_to_coarse() {
        // Mid-Pacific is far from every strait carve-out, so the
        // two-tier lookup must fall through to the coarse grid and
        // return the exact same value the coarse-only grid would.
        let coarse = landmass_grid_at_resolution(0.5);
        let two_tier = landmass_grid_two_tier(0.5, 0.1);
        let p = LatLon::new(-150.0, 0.0);
        let coarse_sd = coarse.signed_distance_m(p);
        let two_tier_sd = two_tier.signed_distance_m(p);
        #[expect(
            clippy::float_cmp,
            reason = "two-tier must literally delegate to coarse outside \
                      every fine patch — not 'approximately equal'."
        )]
        let equal = coarse_sd == two_tier_sd;
        assert!(
            equal,
            "two-tier mid-Pacific SDF must match coarse exactly: coarse = {coarse_sd}, two-tier = {two_tier_sd}",
        );
    }

    #[test]
    fn two_tier_inside_strait_patch_classifies_centerline_as_sea() {
        // A point on the Bosporus carve-out centerline must be sea
        // under both single-tier and two-tier: the carve-out applies
        // in both representations. The two-tier value comes from the
        // fine patch (the point lies inside the Bosporus-region
        // patch's padded bbox), so this implicitly exercises the
        // fine-tier lookup path.
        let two_tier = landmass_grid_two_tier(0.5, 0.1);
        let p = LatLon::new(29.07, 41.13);
        let sd = two_tier.signed_distance_m(p);
        assert!(
            sd >= 0.0,
            "Bosporus centerline must read as sea in two-tier, got SDF = {sd} m",
        );
    }

    #[test]
    fn carve_out_tube_is_narrower_in_fine_tier() {
        // Pick a point ~0.33° east of the Bosporus centerline at the
        // strait's mid-latitude — well inside the coarse 1.2-cell tube
        // (~0.6° radius) but well outside the fine 1.2-cell tube
        // (~0.12° radius). At full polygon resolution the point sits
        // in Asian Istanbul / surrounding Turkish land. The fine tier
        // therefore reports it as land while the coarse tier still
        // calls it carved-out sea.
        let coarse = landmass_grid_at_resolution(0.5);
        let two_tier = landmass_grid_two_tier(0.5, 0.1);
        let p = LatLon::new(29.40, 41.13);
        let coarse_sd = coarse.signed_distance_m(p);
        let two_tier_sd = two_tier.signed_distance_m(p);
        assert!(
            coarse_sd >= 0.0,
            "coarse tier should still classify the point as carved-out sea, got SDF = {coarse_sd} m",
        );
        assert!(
            two_tier_sd < 0.0,
            "two-tier should expose the underlying Turkish land at this offset, got SDF = {two_tier_sd} m",
        );
    }

    #[test]
    fn chord_outside_fine_tube_reports_land_only_under_two_tier() {
        // Construct a chord that walks east from the Bosporus
        // centerline far enough to leave the fine carve-out tube but
        // not the coarse one. The single-tier coarse search reports
        // ~0 km of land along this chord (everything is inside the
        // coarse 66 km tube); the two-tier search reports >0 km
        // because once a substep midpoint clears the fine 13 km tube
        // it sees the real Turkish land beneath.
        let coarse = landmass_grid_at_resolution(0.5);
        let two_tier = landmass_grid_two_tier(0.5, 0.1);
        let origin = LatLon::new(29.07, 41.13);
        let destination = LatLon::new(29.50, 41.13);
        // Use the coarse-grid substep so both calls walk the same
        // midpoints, isolating the lookup-source difference from
        // sampling-cadence differences.
        let step_m = coarse.sampling_step_metres();
        let single_tier_land =
            swarmkit_sailing::get_segment_land_metres(coarse, origin, destination, step_m);
        let two_tier_land =
            swarmkit_sailing::get_segment_land_metres(two_tier, origin, destination, step_m);
        assert_eq!(
            single_tier_land, 0.0,
            "coarse tube still covers the offset chord; expected 0 m land, got {single_tier_land}",
        );
        assert!(
            two_tier_land > 0.0,
            "two-tier must see Turkish land east of the fine tube; expected >0 m land, got {two_tier_land}",
        );
    }

    #[test]
    fn two_tier_a_star_polyline_is_land_free_through_multiple_straits() {
        // scenario-2 regression: Black Sea → off Morocco, threads
        // Bosporus + Dardanelles + Marmara + Aegean + Mediterranean +
        // Gibraltar. Before Phase 7 the A* polyline used coarse cells
        // (66 km tube) so several vertices fell on fine-land (cells
        // outside the 13 km fine tube), and the land-respecting
        // sampler couldn't repair the resulting chord crossings —
        // benchmark reported 218 km of land. Phase 7 puts A* on a
        // unified coarse + fine cell graph so the polyline threads
        // narrow fine tubes inside fine patches.
        let two_tier = landmass_grid_two_tier(0.5, 0.1);
        let origin = LatLon::new(33.226_245_880_126_953, 43.453_300_476_074_22);
        let destination = LatLon::new(-10.553_238_868_713_379, 35.199_138_641_357_42);
        let bounds = RouteBounds::new(
            origin,
            destination,
            LonLatBbox::new(-19.55, 42.05, 32.20, 46.45),
        );
        let polyline = two_tier
            .find_sea_path(origin, destination, &bounds, SeaPathBias::None)
            .expect("unbiased two-tier A* must succeed for scenario-2");
        // Every interior vertex of the polyline (i.e., every cell
        // centre A* picked, excluding the literal origin/destination
        // endpoints which the caller supplies) must be sea under the
        // two-tier model — otherwise the sampler downstream will be
        // unable to produce a land-free baseline.
        let interior = &polyline[1..polyline.len().saturating_sub(1)];
        let bad_count = interior
            .iter()
            .filter(|p| two_tier.signed_distance_m(**p) < 0.0)
            .count();
        assert_eq!(
            bad_count, 0,
            "{bad_count} of {} A* polyline vertices report fine-land",
            interior.len(),
        );
    }

    #[test]
    fn red_sea_to_singapore_path_clears_bab_el_mandeb_malacca_and_singapore() {
        // End-to-end traversal of three carve-outs: Bab-el-Mandeb (out of
        // the Red Sea), Strait of Malacca (south past Sumatra/Malaysia),
        // and Singapore Strait (east into the South China Sea). Every
        // polyline chord must be sea — failure here indicates one of the
        // strait carve-outs is too narrow or its centreline misplaced.
        let grid = landmass_grid();
        // Mid Red Sea between Egypt and Saudi Arabia (clearly offshore so
        // the literal origin / destination vertices don't sit on land).
        let red_sea = LatLon::new(37.5, 22.0);
        let south_china_sea = LatLon::new(104.80, 1.20);
        let bounds = RouteBounds::new(
            red_sea,
            south_china_sea,
            LonLatBbox::new(30.0, 110.0, -15.0, 30.0),
        );
        let polyline = grid
            .find_sea_path(red_sea, south_china_sea, &bounds, SeaPathBias::None)
            .expect("Bab-el-Mandeb + Malacca + Singapore carve-outs should yield a sea path");
        let total_land: f64 = polyline
            .windows(2)
            .map(|w| get_segment_land_metres(grid, w[0], w[1], bounds.step_distance_max))
            .sum();
        assert_eq!(
            total_land, 0.0,
            "Red Sea → Singapore polyline crossed {total_land:.0} m of land",
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
