//! Bundled Natural Earth 50m landmass overlay.
//!
//! The raw `(lon, lat)` polygon rings parsed from the embedded `GeoJSON`
//! are kept once for the lifetime of the app, along with pre-triangulated
//! index lists (egui's path fill is convex-only, so we preprocess
//! landmasses with `earcutr` to get correct fills for arbitrary shapes
//! including holes like the Caspian Sea).
//!
//! Vertices are kept natively in `(lon, lat)` degrees. The view layer
//! ([`crate::view::ViewTransform::map_to_screen`]) projects each vertex
//! to screen pixels per frame, so this module no longer carries a
//! projection origin or rebuilds geometry on map load. Triangulation
//! happens once at startup and is cached in a `OnceLock`.

use std::sync::OnceLock;

use serde::Deserialize;

/// Public-domain Natural Earth 1:50m landmass data, embedded at compile
/// time so the app stays self-contained. ~1.6 MB; about 2× the linear
/// detail of the 110m tier without ballooning the binary. Larger than
/// the clippy default tolerance, but deliberately chosen for the
/// visualisation quality.
///
/// The file lives in the parent `bywind` crate so the search-side
/// landmass module ([`bywind::landmass`]) and this rendering layer
/// stay in sync from a single source of truth.
#[expect(
    clippy::large_include_file,
    reason = "deliberate map detail vs. binary-size trade"
)]
const LAND_GEOJSON: &[u8] = include_bytes!("../../assets/ne_50m_land.geojson");

/// Threshold (degrees) above which an edge between two canonical
/// longitudes is unambiguously an antimeridian wrap. **Outline polylines**
/// crossing this threshold are split into separate sub-strips so the
/// stroke doesn't draw a horizontal line all the way across the map at
/// the seam.
///
/// Fill **triangles** crossing the threshold are *kept*: in the bundled
/// Natural Earth 50m data, Antarctica is represented as a single ring
/// that walks around the south pole using "fake" vertices at
/// `(180, −90) → (−180, −90)`, and earcut produces interior triangles
/// using those vertex pairs as edges. Those triangles have raw
/// `|Δlon| ≈ 360°` but a tiny lat span (they sit right at the pole), so
/// the projected triangle is a thin horizontal strip near the bottom of
/// the world — exactly the right shape for "fill at the south pole".
/// Dropping them used to leave large interior holes in Antarctica.
const ANTIMERIDIAN_WRAP_THRESHOLD_DEG: f32 = 180.0;

/// One landmass: an outer ring plus zero or more interior holes (e.g.
/// inland seas), and a pre-baked triangle-index list that earcutr
/// computed once. Vertices stay in `(lon, lat)` degrees so the view
/// layer can project them per frame without re-triangulating.
struct RawLandmass {
    /// Outer ring then holes, each ring as `(lon, lat)` pairs. Rings
    /// are stored open (no duplicate first/last vertex), as `earcutr`
    /// expects, so the caller has to close them when stroking outlines.
    rings: Vec<Vec<(f32, f32)>>,
    /// Triangle indices (groups of 3) into the flattened ring list:
    /// outer ring vertices first, then each hole in order. Same indexing
    /// scheme `earcutr` returns, ready to drive an `egui::Mesh` once
    /// each vertex has been projected.
    triangles: Vec<usize>,
    /// Total vertex count = sum of ring lengths.
    vertex_count: usize,
}

/// Pre-triangulated landmass geometry in `(lon°, lat°)`. Each vertex is
/// stored as `egui::Pos2` with `.x = lon`, `.y = lat` so the view layer
/// can pass it straight through `latlon_to_screen` (well, `map_to_screen`
/// after the rename). Edges/triangles spanning the antimeridian have
/// already been pruned during triangulation.
pub(crate) struct Landmasses {
    /// Flat list of triangles, three `Pos2`s each. Drawn as a single
    /// `egui::Mesh` per frame for efficient batching.
    pub(crate) triangles: Vec<[egui::Pos2; 3]>,
    /// One polyline per ring (outer + holes), used to stroke coastlines
    /// on top of the fill. Each strip is closed (last == first) where it
    /// hasn't been split by an antimeridian crossing; split halves are
    /// open. Drawing as `Shape::line` produces the right shape either way.
    pub(crate) outlines: Vec<Vec<egui::Pos2>>,
}

/// Lazy parse + triangulate: shared across all viz instances within
/// the process, computed once on first use.
static RAW: OnceLock<Vec<RawLandmass>> = OnceLock::new();

/// Lazy build of [`Landmasses`] from the raw triangulated data —
/// independent of any view, projection, or wind-map load. Built once on
/// first call and cached.
static LANDMASSES: OnceLock<Landmasses> = OnceLock::new();

/// Returns every landmass parsed from the embedded `GeoJSON`, with
/// each polygon already triangulated into ear-clipped indices.
fn raw_landmasses() -> &'static [RawLandmass] {
    RAW.get_or_init(
        || match serde_json::from_slice::<FeatureCollection>(LAND_GEOJSON) {
            Ok(fc) => fc
                .features
                .into_iter()
                .flat_map(|f| f.geometry.into_polygons())
                .map(triangulate_polygon)
                .collect(),
            Err(e) => {
                log::error!("failed to parse bundled landmasses: {e}");
                Vec::new()
            }
        },
    )
}

/// Convert a parsed polygon (rings as `(lon, lat)` pairs) into a
/// [`RawLandmass`] by ear-clipping its vertices once. Triangulation is
/// done in `(lon, lat)` space — the geometry is stable enough at the 50m
/// scale that the resulting triangles look correct after projection.
fn triangulate_polygon(rings: Vec<Vec<(f64, f64)>>) -> RawLandmass {
    if rings.is_empty() {
        return RawLandmass {
            rings: Vec::new(),
            triangles: Vec::new(),
            vertex_count: 0,
        };
    }

    // Strip closing duplicate vertex (GeoJSON rings are closed; earcutr
    // expects them open) and convert to f32 for storage.
    let rings_f32: Vec<Vec<(f32, f32)>> = rings
        .into_iter()
        .map(|r| {
            let mut r = r;
            if r.len() >= 2 && r.first() == r.last() {
                r.pop();
            }
            r.into_iter()
                .map(|(lon, lat)| (lon as f32, lat as f32))
                .collect()
        })
        .collect();

    // Flat coords (f64 — earcutr is more numerically stable in f64) +
    // hole-start indices for earcutr.
    let mut coords: Vec<f64> = Vec::new();
    let mut hole_indices: Vec<usize> = Vec::new();
    for (i, ring) in rings_f32.iter().enumerate() {
        if i > 0 {
            hole_indices.push(coords.len() / 2);
        }
        for &(lon, lat) in ring {
            coords.push(f64::from(lon));
            coords.push(f64::from(lat));
        }
    }

    let triangles = earcutr::earcut(&coords, &hole_indices, 2).unwrap_or_else(|e| {
        log::warn!("earcutr failed on a landmass polygon: {e:?}");
        Vec::new()
    });
    let vertex_count = coords.len() / 2;

    RawLandmass {
        rings: rings_f32,
        triangles,
        vertex_count,
    }
}

/// Return the cached, triangulated landmasses in `(lon°, lat°)`. The view
/// layer projects each vertex per frame.
///
/// Antimeridian-crossing triangles are pruned at build time so the
/// caller doesn't need a per-frame edge-length check; polylines are
/// split at the same threshold.
pub(crate) fn landmasses() -> &'static Landmasses {
    LANDMASSES.get_or_init(|| {
        let mut triangles: Vec<[egui::Pos2; 3]> = Vec::new();
        let mut outlines: Vec<Vec<egui::Pos2>> = Vec::new();

        for landmass in raw_landmasses() {
            // Linearise rings into a single flat vertex list in the same
            // order earcutr's indices reference.
            let mut verts: Vec<egui::Pos2> = Vec::with_capacity(landmass.vertex_count);
            for ring in &landmass.rings {
                for &(lon, lat) in ring {
                    verts.push(egui::Pos2::new(lon, lat));
                }
            }

            for tri_idx in landmass.triangles.chunks_exact(3) {
                let (Some(&ai), Some(&bi), Some(&ci)) =
                    (tri_idx.first(), tri_idx.get(1), tri_idx.get(2))
                else {
                    continue;
                };
                let (Some(&a), Some(&b), Some(&c)) = (verts.get(ai), verts.get(bi), verts.get(ci))
                else {
                    continue;
                };
                // Triangles with antimeridian-wrap edges are kept; see
                // [`ANTIMERIDIAN_WRAP_THRESHOLD_DEG`] for why. Earcut may
                // produce them for polygons that walk around the pole
                // (Antarctica, primarily), and dropping them leaves
                // visible holes in the fill.
                triangles.push([a, b, c]);
            }

            // Outlines: one closed polyline per ring, split at antimeridian
            // crossings so a ring spanning ±180° doesn't draw a horizontal
            // line across the whole map.
            let mut cursor = verts.iter();
            for ring in &landmass.rings {
                let mut strip: Vec<egui::Pos2> =
                    cursor.by_ref().take(ring.len()).copied().collect();
                if let Some(&first) = strip.first() {
                    strip.push(first);
                }
                for sub in split_long_edges(&strip) {
                    if sub.len() >= 2 {
                        outlines.push(sub);
                    }
                }
            }
        }

        Landmasses {
            triangles,
            outlines,
        }
    })
}

/// Split a polyline anywhere two consecutive vertices have a longitude
/// jump larger than [`ANTIMERIDIAN_WRAP_THRESHOLD_DEG`] — that's an antimeridian wrap.
/// Without this, drawing a ring that crosses ±180° produces a horizontal
/// line across the whole map between its two halves.
fn split_long_edges(strip: &[egui::Pos2]) -> Vec<Vec<egui::Pos2>> {
    let mut out = Vec::new();
    let mut current: Vec<egui::Pos2> = Vec::new();
    for &p in strip {
        if let Some(&prev) = current.last()
            && (p.x - prev.x).abs() > ANTIMERIDIAN_WRAP_THRESHOLD_DEG
        {
            if current.len() >= 2 {
                out.push(std::mem::take(&mut current));
            } else {
                current.clear();
            }
        }
        current.push(p);
    }
    if current.len() >= 2 {
        out.push(current);
    }
    out
}

// ---- GeoJSON shape (only the bits we need) ----

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
    /// Flatten a `Geometry` into a list of polygons (each polygon is a
    /// list of rings). `MultiPolygon` returns multiple; `Polygon`
    /// returns one; anything else returns none.
    fn into_polygons(self) -> Vec<Vec<Vec<(f64, f64)>>> {
        let convert = |ring: Vec<[f64; 2]>| -> Vec<(f64, f64)> {
            ring.into_iter().map(|p| (p[0], p[1])).collect()
        };
        match self {
            Self::Polygon { coordinates } => {
                vec![coordinates.into_iter().map(convert).collect()]
            }
            Self::MultiPolygon { coordinates } => coordinates
                .into_iter()
                .map(|p| p.into_iter().map(convert).collect())
                .collect(),
            Self::Other => Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_landmasses_parses_at_least_one_polygon() {
        let landmasses = raw_landmasses();
        assert!(
            !landmasses.is_empty(),
            "expected polygons from bundled GeoJSON"
        );
        // Sanity: at least one landmass should have a non-trivial
        // outer ring and a non-empty triangulation.
        assert!(
            landmasses
                .iter()
                .any(|l| l.rings.first().is_some_and(|r| r.len() >= 3) && !l.triangles.is_empty())
        );
    }

    #[test]
    fn triangulate_polygon_handles_simple_quad() {
        // A unit square produces 2 triangles. Indices come out in some
        // order; just check the count.
        let rings = vec![vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]];
        let lm = triangulate_polygon(rings);
        assert_eq!(lm.rings.first().expect("one ring").len(), 4);
        assert_eq!(lm.triangles.len(), 2 * 3);
        assert_eq!(lm.vertex_count, 4);
    }

    #[test]
    fn triangulate_polygon_strips_closing_vertex() {
        // GeoJSON-style closed ring: first == last. Triangulator must
        // see only the unique points or it produces a degenerate triangle.
        let rings = vec![vec![
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0),
        ]];
        let lm = triangulate_polygon(rings);
        assert_eq!(lm.rings.first().expect("one ring").len(), 4);
        assert_eq!(lm.vertex_count, 4);
    }

    #[test]
    fn split_long_edges_breaks_antimeridian_jump() {
        // Two points more than ANTIMERIDIAN_WRAP_THRESHOLD_DEG apart in lon → wrap.
        let strip = vec![egui::Pos2::new(-179.0, 0.0), egui::Pos2::new(179.0, 0.0)];
        let parts = split_long_edges(&strip);
        assert!(parts.is_empty() || parts.iter().all(|p| p.len() < 2));
    }

    #[test]
    fn landmasses_yields_some_triangles() {
        let lm = landmasses();
        assert!(
            !lm.triangles.is_empty(),
            "expected non-empty fill triangles"
        );
        assert!(
            !lm.outlines.is_empty(),
            "expected non-empty outline polylines"
        );
    }
}
