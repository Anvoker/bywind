use bywind::{
    BakedWindMap, BenchmarkRoute, MapBounds, SearchWeights, SegmentMetrics, WeatherRow, WindMap,
    compute_segment_metrics,
};
use swarmkit::Evolution;
use swarmkit_sailing::{Boat, Path, RouteBounds, weighted_fitness};

use crate::view::ViewTransform;

/// Fraction of one grid cell a barb's shaft spans. 0.6 leaves a
/// comfortable margin between adjacent barbs at the natural cell density.
const BARB_CELL_FILL_FRACTION: f32 = 0.6;
/// Pixel shaft length for the kd-tree / no-grid fallback path. Matches
/// the historical default so non-grid wind maps render the way they used
/// to.
const BARB_FALLBACK_PIXELS: f32 = 30.0;

pub(crate) fn draw_windmap(
    painter: &egui::Painter,
    wind_map: &WindMap,
    view: &ViewTransform,
    stroke: impl Into<egui::Stroke> + Copy,
) {
    // Grid-backed maps (everything from GRIB2 + the synthetic generators)
    // get the fast path: walk only the cells whose `(x, y)` overlap the
    // visible panel rect, with an LOD stride so zoomed-out views skip the
    // dense grid that would otherwise issue millions of `Shape` calls per
    // frame. kd-tree maps fall back to the project-and-cull walk.
    if let Some(layout) = wind_map.grid_layout() {
        draw_windmap_gridded(painter, wind_map, &layout, view, stroke);
    } else {
        draw_windmap_kdtree(painter, wind_map, view, stroke);
    }
}

/// Visible-cell + LOD-stride render path for grid-backed wind maps.
fn draw_windmap_gridded(
    painter: &egui::Painter,
    wind_map: &WindMap,
    layout: &bywind::GridLayout,
    view: &ViewTransform,
    stroke: impl Into<egui::Stroke> + Copy,
) {
    // Pixels per cell on screen. Used both for the LOD stride decision and
    // for the barb-shaft length.
    let cell_px_x =
        layout.step_x * view.cos_lat0 * crate::view::METRES_PER_DEGREE * view.render_scale;
    if !cell_px_x.is_finite() || cell_px_x <= 0.0 {
        return;
    }
    // Stride: thin the grid so on-screen cell spacing stays at least
    // `MIN_CELL_PX` apart. Below that threshold barbs are visually
    // unreadable anyway and the per-shape tessellation cost dominates.
    const MIN_CELL_PX: f32 = 8.0;
    let stride = if cell_px_x < MIN_CELL_PX {
        ((MIN_CELL_PX / cell_px_x).ceil() as usize).max(1)
    } else {
        1
    };
    // Shaft length is sized to the *strided* cell so thinned-out barbs
    // grow to fill the gap they leave behind, instead of looking like
    // tiny dots in a sparse field.
    let shaft_length = cell_px_x * stride as f32 * BARB_CELL_FILL_FRACTION;

    let panel = painter.clip_rect();
    let visible = panel.expand(shaft_length);
    let world_width_px = world_width_pixels(view);
    let rows = wind_map.rows();

    // For each shadow shift (left tile, centre, right tile), compute the
    // (i, j) range whose projected position would land inside the panel,
    // then iterate stride'd cells in that range. Off-tile shadows give
    // empty ranges and skip cleanly.
    for shift in shadow_offsets(world_width_px) {
        if !shift.is_finite() {
            continue;
        }
        // Invert `screen.x = (lon - lon0) * k + offset.x + shift` for the
        // panel's x bounds to find the lon range visible at this shift.
        // y has no shift.
        let inv_kx = 1.0 / (view.cos_lat0 * crate::view::METRES_PER_DEGREE * view.render_scale);
        let inv_ky = 1.0 / (crate::view::METRES_PER_DEGREE * view.render_scale);
        let lon_min = view.lon0 + (panel.min.x - view.offset.x - shift) * inv_kx;
        let lon_max = view.lon0 + (panel.max.x - view.offset.x - shift) * inv_kx;
        // Projection negates y so the panel's *top* (smaller y) is the
        // *highest* latitude — invert the order when converting back.
        let lat_max = view.lat0 - (panel.min.y - view.offset.y) * inv_ky;
        let lat_min = view.lat0 - (panel.max.y - view.offset.y) * inv_ky;

        // Convert (lon, lat) range to (i, j) cell range. Expand by 1 cell
        // to catch barbs whose shafts extend past the panel edge.
        let i_min_f = ((lon_min - layout.origin_x) / layout.step_x).floor() - 1.0;
        let i_max_f = ((lon_max - layout.origin_x) / layout.step_x).ceil() + 1.0;
        let j_min_f = ((lat_min - layout.origin_y) / layout.step_y).floor() - 1.0;
        let j_max_f = ((lat_max - layout.origin_y) / layout.step_y).ceil() + 1.0;
        let i_min = (i_min_f.max(0.0) as usize).min(layout.nx.saturating_sub(1));
        let i_max = (i_max_f.max(0.0) as usize).min(layout.nx.saturating_sub(1));
        let j_min = (j_min_f.max(0.0) as usize).min(layout.ny.saturating_sub(1));
        let j_max = (j_max_f.max(0.0) as usize).min(layout.ny.saturating_sub(1));
        if i_min > i_max || j_min > j_max {
            continue;
        }

        // Align the strided iteration to *global* multiples of `stride`
        // (i.e. `i % stride == 0`) instead of starting at `i_min`. Both
        // the centre and shadow tiles iterate the same canonical cell
        // indices, so the rendered cells line up across the antimeridian
        // — without this, the gap between the centre tile's last cell
        // and the shadow tile's first cell is whatever
        // `(i_max - i_min) % stride` happens to be (usually a tiny
        // mismatch that reads as a 2-px cluster at the seam).
        let stride_u = stride; // already > 0
        let start_i = i_min.div_ceil(stride_u) * stride_u;
        let start_j = j_min.div_ceil(stride_u) * stride_u;
        let mut i = start_i;
        while i <= i_max {
            let mut j = start_j;
            while j <= j_max {
                let idx = i * layout.ny + j;
                if let Some(row) = rows.get(idx) {
                    let base = view.map_to_screen(egui::Pos2::new(row.lon, row.lat));
                    let origin = egui::Pos2::new(base.x + shift, base.y);
                    if visible.contains(origin) {
                        draw_wind_barb(painter, row, origin, shaft_length, stroke);
                    }
                }
                j += stride;
            }
            i += stride;
        }
    }
}

/// Project-and-cull fallback for kd-tree-backed wind maps (non-uniform
/// grids — preserved so synthetic CSV-loaded maps with arbitrary point
/// clouds still render).
fn draw_windmap_kdtree(
    painter: &egui::Painter,
    wind_map: &WindMap,
    view: &ViewTransform,
    stroke: impl Into<egui::Stroke> + Copy,
) {
    let shaft_length = BARB_FALLBACK_PIXELS;
    let visible = painter.clip_rect().expand(shaft_length);
    let world_width_px = world_width_pixels(view);
    for row in wind_map.rows() {
        let base = view.map_to_screen(egui::Pos2::new(row.lon, row.lat));
        for shift in shadow_offsets(world_width_px) {
            if !shift.is_finite() {
                continue;
            }
            let origin = egui::Pos2::new(base.x + shift, base.y);
            if visible.contains(origin) {
                draw_wind_barb(painter, row, origin, shaft_length, stroke);
            }
        }
    }
}

/// Width of one full `360°` rotation of longitude in screen pixels at the
/// view's current `lat0` / `render_scale`. Used by shadow-copy renderers
/// to offset duplicated geometry by exactly one world.
pub(crate) fn world_width_pixels(view: &ViewTransform) -> f32 {
    360.0 * view.cos_lat0 * crate::view::METRES_PER_DEGREE * view.render_scale
}

/// `(−world, 0, +world)` shifts for shadow-copy rendering. NaN entries
/// — emitted when the world is sub-pixel (extreme zoom-out or near a
/// pole) — should be filtered out by the caller via `is_finite()` so a
/// single offset of `0.0` covers every visible point.
pub(crate) fn shadow_offsets(world_width_px: f32) -> [f32; 3] {
    if world_width_px.is_finite() && world_width_px >= 1.0 {
        [-world_width_px, 0.0, world_width_px]
    } else {
        [f32::NAN, 0.0, f32::NAN]
    }
}

/// Renders a single wind barb at `origin` with shaft length already in
/// screen pixels. Caller (`draw_windmap`) is responsible for computing
/// `shaft_length` from the wind map's grid spacing.
fn draw_wind_barb(
    painter: &egui::Painter,
    row: &WeatherRow,
    origin: egui::Pos2,
    shaft_length: f32,
    stroke: impl Into<egui::Stroke>,
) {
    let long_barb = shaft_length / 3.0;
    let short_barb = shaft_length / 6.0;
    let spacing = shaft_length / 6.0;
    let barb_spacing = spacing / 1.5;

    let stroke: egui::Stroke = stroke.into();

    // Calm wind (< 2.5 kts): draw a circle with no shaft.
    if row.sample.speed < 2.5 {
        painter.circle_stroke(origin, shaft_length / 4.0, stroke);
        return;
    }

    let dir_rad = row.sample.direction.to_radians();
    let shaft_dir = egui::Vec2::new(dir_rad.sin(), -dir_rad.cos()).normalized();
    // Left of the shaft in screen space (y-down), Northern Hemisphere convention.
    let barb_dir = egui::Vec2::new(shaft_dir.y, -shaft_dir.x);

    let base = origin - shaft_dir * (shaft_length / 2.0);
    let tip = base + shaft_dir * shaft_length;
    painter.line_segment([base, tip], stroke);

    // Decompose speed (rounded to nearest 5 kts) into pennants, long barbs, short barbs.
    let speed_rounded = ((row.sample.speed / 5.0).round() as u32) * 5;
    let pennants = speed_rounded / 50;
    let remainder = speed_rounded % 50;
    let long_barbs = remainder / 10;
    let short_barbs = (remainder % 10) / 5;

    // Traverse from tip toward origin, placing elements largest-first.
    let mut pos = tip;

    // Pennants: filled triangle. Base spans one `spacing` along the shaft;
    // peak extends `long_barb` perpendicular (making it taller than wide).
    for _ in 0..pennants {
        let base_near = pos - shaft_dir * spacing;
        let peak = pos + barb_dir * long_barb;
        painter.add(egui::Shape::convex_polygon(
            vec![pos, base_near, peak],
            stroke.color,
            egui::Stroke::NONE,
        ));
        pos -= shaft_dir * spacing;
    }

    // Extra gap between pennants and barbs.
    if pennants > 0 {
        pos -= shaft_dir * barb_spacing;
    }

    for _ in 0..long_barbs {
        painter.line_segment([pos, pos + barb_dir * long_barb], stroke);
        pos -= shaft_dir * barb_spacing;
    }

    for _ in 0..short_barbs {
        painter.line_segment([pos, pos + barb_dir * short_barb], stroke);
        pos -= shaft_dir * barb_spacing;
    }
}

/// Render the bundled Natural Earth landmasses on top of the wind
/// barbs: a faint translucent fill so the user can tell water from
/// land at a glance, then a white halo + dark stroke per ring to keep
/// the coast crisp at any zoom. `landmasses` carries `(lon, lat)`
/// vertices — triangles already ear-clipped, outlines already split
/// at the antimeridian — and `view.map_to_screen` does the per-frame
/// equirectangular projection here on the draw path.
pub(crate) fn draw_coastlines(
    painter: &egui::Painter,
    view: &ViewTransform,
    landmasses: &crate::coastlines::Landmasses,
) {
    // Subtle "land" tint — an alpha-30 warm beige reads as land
    // without competing with the wind barbs visually.
    let fill_color = egui::Color32::from_rgba_premultiplied(160, 140, 110, 30);
    let world_width_px = world_width_pixels(view);
    let clip = painter.clip_rect();
    if !landmasses.triangles.is_empty() {
        // Build one mesh per visible shadow offset. A single combined mesh
        // would also work but per-tile lets us cheaply skip whole tiles
        // that don't intersect the panel via a quick aabb test.
        for shift in shadow_offsets(world_width_px) {
            if !shift.is_finite() {
                continue;
            }
            let mut mesh = egui::Mesh::default();
            let mut any_visible = false;
            for tri in &landmasses.triangles {
                let a = shift_x(view.map_to_screen(tri[0]), shift);
                let b = shift_x(view.map_to_screen(tri[1]), shift);
                let c = shift_x(view.map_to_screen(tri[2]), shift);
                // Per-triangle aabb test: skip if all three points share
                // the same off-screen side. Lets sub-pixel-thin maps
                // bypass mesh allocation entirely on far tiles.
                if !triangle_intersects_clip(a, b, c, clip) {
                    continue;
                }
                any_visible = true;
                let i = mesh.vertices.len() as u32;
                mesh.colored_vertex(a, fill_color);
                mesh.colored_vertex(b, fill_color);
                mesh.colored_vertex(c, fill_color);
                mesh.add_triangle(i, i + 1, i + 2);
            }
            if any_visible {
                painter.add(egui::Shape::mesh(mesh));
            }
        }
    }

    let halo = egui::Stroke::new(
        4.0,
        egui::Color32::from_rgba_premultiplied(255, 255, 255, 200),
    );
    let line = egui::Stroke::new(1.5, egui::Color32::from_rgb(20, 20, 20));
    for strip in &landmasses.outlines {
        if strip.len() < 2 {
            continue;
        }
        let projected: Vec<egui::Pos2> = strip.iter().map(|p| view.map_to_screen(*p)).collect();
        for shift in shadow_offsets(world_width_px) {
            if !shift.is_finite() {
                continue;
            }
            // Cheapest possible per-strip cull: skip tiles whose x-extent
            // doesn't overlap the panel. The polyline tessellator is
            // expensive enough that this is worth a separate pass.
            let (mut min_x, mut max_x) = (f32::INFINITY, f32::NEG_INFINITY);
            for p in &projected {
                let sx = p.x + shift;
                if sx < min_x {
                    min_x = sx;
                }
                if sx > max_x {
                    max_x = sx;
                }
            }
            if max_x < clip.min.x || min_x > clip.max.x {
                continue;
            }
            let shifted: Vec<egui::Pos2> = projected.iter().map(|p| shift_x(*p, shift)).collect();
            painter.add(egui::Shape::line(shifted.clone(), halo));
            painter.add(egui::Shape::line(shifted, line));
        }
    }
}

/// Apply a horizontal pixel offset to a screen position. Used by
/// shadow-copy renderers to shift pre-projected geometry by exactly one
/// world width without re-running the projection.
fn shift_x(p: egui::Pos2, dx: f32) -> egui::Pos2 {
    egui::Pos2::new(p.x + dx, p.y)
}

/// Conservative aabb-vs-rect intersection test for a triangle. False
/// positives are fine (we'll just emit a culled mesh entry); false
/// negatives would drop visible geometry.
fn triangle_intersects_clip(a: egui::Pos2, b: egui::Pos2, c: egui::Pos2, clip: egui::Rect) -> bool {
    let min_x = a.x.min(b.x).min(c.x);
    let max_x = a.x.max(b.x).max(c.x);
    let min_y = a.y.min(b.y).min(c.y);
    let max_y = a.y.max(b.y).max(c.y);
    !(max_x < clip.min.x || min_x > clip.max.x || max_y < clip.min.y || min_y > clip.max.y)
}

/// Translucent rectangle for the user-defined search domain (`route_bbox`)
/// plus a live preview while the Route Bounds tool is mid-drag.
/// `committed` is in map coordinates; `live_drag` is `(anchor_screen,
/// current_screen)` so the preview tracks the cursor across pan/zoom.
pub(crate) fn draw_route_bounds(
    painter: &egui::Painter,
    view: &ViewTransform,
    committed: Option<(f64, f64, f64, f64)>,
    live_drag: Option<(egui::Pos2, egui::Pos2)>,
) {
    const FILL: egui::Color32 = egui::Color32::from_rgba_premultiplied(40, 120, 255, 30);
    const STROKE: egui::Color32 = egui::Color32::from_rgb(40, 120, 255);
    let world_width_px = world_width_pixels(view);
    let clip = painter.clip_rect();
    if let Some((lon_min, lon_max, lat_min, lat_max)) = committed {
        // Antimeridian-wrapping bbox is encoded as `lon_min > lon_max`
        // (canonical). Project the eastern edge `+360°` past the western
        // one so the rect spans a contiguous range in projected metres;
        // shadow copies below fold the off-screen half back into view.
        let raw_east = if lon_max >= lon_min {
            lon_max
        } else {
            lon_max + 360.0
        };
        // Snap to a full-world span when the user's bbox is "the whole
        // wind map" (the auto-default for a global GFS load is something
        // like `−180 .. 179.75`, leaving a half-cell strip unfilled at the
        // antimeridian once shadow copies abut). At ≥ 359° wide we treat
        // the user's intent as "everything" and extend the rect to a
        // perfect 360° so the centre and shadow tiles meet without a gap.
        let width = raw_east - lon_min;
        let lon_east_unwrapped = if width > 359.0 {
            lon_min + 360.0
        } else {
            raw_east
        };
        let a = view.map_to_screen(egui::Pos2::new(lon_min as f32, lat_min as f32));
        let b = view.map_to_screen(egui::Pos2::new(lon_east_unwrapped as f32, lat_max as f32));
        for shift in shadow_offsets(world_width_px) {
            if !shift.is_finite() {
                continue;
            }
            let rect = egui::Rect::from_two_pos(shift_x(a, shift), shift_x(b, shift));
            if !rect.intersects(clip) {
                continue;
            }
            painter.rect(
                rect,
                0.0,
                FILL,
                egui::Stroke::new(2.0, STROKE),
                egui::StrokeKind::Inside,
            );
        }
    }
    if let Some((anchor, current)) = live_drag {
        // Dashed preview: redraw shorter alternating segments via two
        // thin lines stacked on top of one another.
        let rect = egui::Rect::from_two_pos(anchor, current);
        painter.rect(
            rect,
            0.0,
            egui::Color32::TRANSPARENT,
            egui::Stroke::new(1.0, STROKE.gamma_multiply(0.7)),
            egui::StrokeKind::Inside,
        );
    }
}

/// Filled markers at the user-placed start (green) and end (red) waypoints.
/// Drawn in screen space via the `ViewTransform` so they track pan / zoom.
/// Either or both may be `None` (nothing drawn for the missing one).
pub(crate) fn draw_endpoint_markers(
    painter: &egui::Painter,
    view: &ViewTransform,
    start: Option<(f64, f64)>,
    end: Option<(f64, f64)>,
) {
    const RADIUS: f32 = 7.0;
    const OUTLINE: f32 = 2.0;
    const LABEL_SIZE: f32 = 11.0;

    let world_width_px = world_width_pixels(view);
    let clip = painter.clip_rect().expand(RADIUS + LABEL_SIZE);
    let draw = |xy: (f64, f64), fill: egui::Color32, label: &str| {
        let base = view.map_to_screen(egui::Pos2::new(xy.0 as f32, xy.1 as f32));
        for shift in shadow_offsets(world_width_px) {
            if !shift.is_finite() {
                continue;
            }
            let screen = shift_x(base, shift);
            if !clip.contains(screen) {
                continue;
            }
            painter.circle_filled(screen, RADIUS, fill);
            painter.circle_stroke(
                screen,
                RADIUS,
                egui::Stroke::new(OUTLINE, egui::Color32::BLACK),
            );
            painter.text(
                screen,
                egui::Align2::CENTER_CENTER,
                label,
                egui::FontId::proportional(LABEL_SIZE),
                egui::Color32::WHITE,
            );
        }
    };
    if let Some(xy) = start {
        draw(xy, egui::Color32::from_rgb(40, 180, 40), "S");
    }
    if let Some(xy) = end {
        draw(xy, egui::Color32::from_rgb(220, 60, 60), "E");
    }
}

/// Inset minimap in the bottom-right corner: a fixed-scale Plate-Carrée
/// rectangle of the whole world, with the wind data's bbox tinted green
/// and the visible viewport drawn as a translucent blue rectangle. Using
/// a fixed world frame (rather than fitting the minimap to the wind
/// bbox) means a small viewport stays visible as a small rect inside the
/// minimap even when the panel can see far more lon/lat than the data
/// itself spans. The viewport rectangle splits in two when it straddles
/// the antimeridian so wrap-around panning reads as "looking at the
/// seam" instead of pinning to an edge.
pub(crate) fn draw_minimap(
    painter: &egui::Painter,
    panel_rect: egui::Rect,
    bounds: &MapBounds,
    view: &ViewTransform,
    route: Option<(&[f64], &[f64])>,
) {
    // World is 360° × 180°; a 2:1 minimap preserves Plate-Carrée aspect.
    const MINI_W: f32 = 160.0;
    const MINI_H: f32 = 80.0;
    const MARGIN: f32 = 6.0;
    const MIN_THUMB: f32 = 3.0;

    let mini_rect = egui::Rect::from_min_size(
        egui::Pos2::new(
            panel_rect.right() - MINI_W - MARGIN,
            panel_rect.bottom() - MINI_H - MARGIN,
        ),
        egui::Vec2::new(MINI_W, MINI_H),
    );

    let bg_color = egui::Color32::from_black_alpha(160);
    let data_color = egui::Color32::from_rgba_unmultiplied(90, 150, 90, 170);
    let viewport_color = egui::Color32::from_rgba_unmultiplied(80, 160, 255, 200);
    let viewport_stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(180, 220, 255));
    let route_stroke = egui::Stroke::new(1.5, egui::Color32::from_rgb(255, 140, 0));
    painter.rect_filled(mini_rect, 3.0, bg_color);

    // Lon band `[lo, hi]` × lat band `[lat_min, lat_max]` → minimap rect.
    let band_rect = |lo: f32, hi: f32, lat_min: f32, lat_max: f32| -> egui::Rect {
        let x0 = mini_rect.left() + (lo + 180.0) / 360.0 * MINI_W;
        let x1 = mini_rect.left() + (hi + 180.0) / 360.0 * MINI_W;
        let y0 = mini_rect.top() + (90.0 - lat_max) / 180.0 * MINI_H;
        let y1 = mini_rect.top() + (90.0 - lat_min) / 180.0 * MINI_H;
        egui::Rect::from_min_max(egui::Pos2::new(x0, y0), egui::Pos2::new(x1, y1))
    };

    // (lon, lat) → point inside the minimap. Caller is responsible for
    // canonicalising lon to `[-180, 180]` first.
    let to_mini = |lon: f32, lat: f32| -> egui::Pos2 {
        egui::Pos2::new(
            mini_rect.left() + (lon + 180.0) / 360.0 * MINI_W,
            mini_rect.top() + (90.0 - lat) / 180.0 * MINI_H,
        )
    };

    // Wind data bbox tint. Wrap-bbox splits at the antimeridian.
    let bbox = bounds.bbox;
    let dlat_min = bbox.lat_min as f32;
    let dlat_max = bbox.lat_max as f32;
    if bbox.wraps_antimeridian() {
        painter.rect_filled(
            band_rect(bbox.lon_min as f32, 180.0, dlat_min, dlat_max),
            0.0,
            data_color,
        );
        painter.rect_filled(
            band_rect(-180.0, bbox.lon_max as f32, dlat_min, dlat_max),
            0.0,
            data_color,
        );
    } else {
        painter.rect_filled(
            band_rect(bbox.lon_min as f32, bbox.lon_max as f32, dlat_min, dlat_max),
            0.0,
            data_color,
        );
    }

    // Viewport in (lon, lat). Latitude clamps to the world; longitude
    // folds into [-180, 180) and may split.
    let v_tl = view.screen_to_map(panel_rect.min);
    let v_br = view.screen_to_map(panel_rect.max);
    let vlat_hi = v_tl.y.clamp(-90.0, 90.0);
    let vlat_lo = v_br.y.clamp(-90.0, 90.0);
    if vlat_hi > vlat_lo {
        let vlon_w = (v_br.x - v_tl.x).max(0.0);
        let draw_viewport_band = |lo: f32, hi: f32| {
            let rect = band_rect(lo, hi, vlat_lo, vlat_hi);
            // Enforce a minimum visible thumb so deep-zoom views still
            // show an indicator. Grow the rect's bottom-right edge
            // outward (the top-left edge stays anchored so the position
            // still tracks the viewport), then clip back inside the
            // minimap.
            let right = rect
                .right()
                .max(rect.left() + MIN_THUMB)
                .min(mini_rect.right());
            let bottom = rect
                .bottom()
                .max(rect.top() + MIN_THUMB)
                .min(mini_rect.bottom());
            let rect = egui::Rect::from_min_max(rect.min, egui::Pos2::new(right, bottom));
            painter.rect_filled(rect, 1.0, viewport_color);
            painter.rect_stroke(rect, 1.0, viewport_stroke, egui::StrokeKind::Inside);
        };
        if vlon_w >= 360.0 {
            draw_viewport_band(-180.0, 180.0);
        } else {
            let lo_folded = -180.0 + (v_tl.x + 180.0).rem_euclid(360.0);
            let hi_folded = lo_folded + vlon_w;
            if hi_folded <= 180.0 {
                draw_viewport_band(lo_folded, hi_folded);
            } else {
                draw_viewport_band(lo_folded, 180.0);
                draw_viewport_band(-180.0, hi_folded - 360.0);
            }
        }
    }

    // Route polyline. Drawn last so it sits on top of both the data
    // tint and the viewport rectangle. Each segment picks the shortest
    // east-west direction between consecutive waypoints and is split at
    // the antimeridian when that shortest path crosses ±180° — matches
    // the central panel's antimeridian unwrap so the same route reads
    // the same way at both scales.
    if let Some((lons, lats)) = route
        && lons.len() == lats.len()
        && lons.len() >= 2
    {
        for i in 0..lons.len() - 1 {
            let a_lon = wrap_lon_to_180(lons[i] as f32);
            let b_lon = wrap_lon_to_180(lons[i + 1] as f32);
            let a_lat = (lats[i] as f32).clamp(-90.0, 90.0);
            let b_lat = (lats[i + 1] as f32).clamp(-90.0, 90.0);
            let east_delta = (b_lon - a_lon).rem_euclid(360.0); // [0, 360)
            let west_delta = 360.0 - east_delta;
            if east_delta <= west_delta {
                if a_lon + east_delta <= 180.0 {
                    painter
                        .line_segment([to_mini(a_lon, a_lat), to_mini(b_lon, b_lat)], route_stroke);
                } else {
                    // Crosses the +180 edge — split into two segments at
                    // the seam, interpolating lat at the cross point.
                    let t = (180.0 - a_lon) / east_delta;
                    let mid_lat = a_lat + (b_lat - a_lat) * t;
                    painter.line_segment(
                        [to_mini(a_lon, a_lat), to_mini(180.0, mid_lat)],
                        route_stroke,
                    );
                    painter.line_segment(
                        [to_mini(-180.0, mid_lat), to_mini(b_lon, b_lat)],
                        route_stroke,
                    );
                }
            } else if a_lon - west_delta >= -180.0 {
                painter.line_segment([to_mini(a_lon, a_lat), to_mini(b_lon, b_lat)], route_stroke);
            } else {
                // Crosses the -180 edge.
                let t = (a_lon + 180.0) / west_delta;
                let mid_lat = a_lat + (b_lat - a_lat) * t;
                painter.line_segment(
                    [to_mini(a_lon, a_lat), to_mini(-180.0, mid_lat)],
                    route_stroke,
                );
                painter.line_segment(
                    [to_mini(180.0, mid_lat), to_mini(b_lon, b_lat)],
                    route_stroke,
                );
            }
        }
    }
}

/// Fold a longitude (any real value, possibly outside `[-180, 180]`) to
/// the canonical range `[-180, 180]`. `180.0` aliases to `-180.0`
/// (same meridian), keeping the result in a single half-open interval.
fn wrap_lon_to_180(lon: f32) -> f32 {
    (lon + 180.0).rem_euclid(360.0) - 180.0
}

/// Which on-path text labels to draw at each waypoint.
///
/// `None` skips labels entirely (used for "Show all particles" mode and
/// for any context where stacking text would just be noise). `Index`
/// stamps each waypoint with its ordinal (`0` = start, `N-1` = end);
/// useful as the default route hint. `TimeFrame` stamps each waypoint
/// with `"Nh Nm Ns [frame]"` cumulative arrival time + frame index,
/// reserved for the `Set Time From Waypoint` tool where the user is
/// picking which waypoint to snap the time slider to.
#[derive(Clone, Copy)]
pub(crate) enum WaypointLabel {
    Index,
    TimeFrame {
        step_seconds: f32,
        frame_count: usize,
    },
}

fn draw_path<const N: usize>(
    painter: &egui::Painter,
    path: &Path<N>,
    view: &ViewTransform,
    alpha: u8,
    segment_stats: Option<&[SegmentMetrics]>,
    waypoint_label: Option<WaypointLabel>,
) {
    let node_color = egui::Color32::from_rgba_unmultiplied(255, 100, 0, alpha);
    let origin_color = egui::Color32::from_rgba_unmultiplied(0, 200, 80, alpha);
    let destination_color = egui::Color32::from_rgba_unmultiplied(220, 40, 40, alpha);

    let mut points: Vec<egui::Pos2> = (0..path.len())
        .map(|i| {
            let v = path.lat_lon(i);
            view.route_to_screen(v.lon, v.lat)
        })
        .collect();

    // Unwrap successive segments across the antimeridian: if two
    // consecutive waypoints' projected screen.x differ by more than half
    // a world width, shift the later point by ±world_width_px so the
    // segment draws the short way around. Shadow-copy rendering below
    // then maps the unwrapped chain back into the visible panel.
    let world_width_px = world_width_pixels(view);
    if world_width_px.is_finite() && world_width_px > 0.0 {
        let half = world_width_px * 0.5;
        // Index-free traversal: keep a running `prev_x` to avoid clippy's
        // `indexing_slicing` lint and the bounds check it implies.
        let mut prev_x = points.first().map_or(0.0, |p| p.x);
        for p in points.iter_mut().skip(1) {
            let dx = p.x - prev_x;
            if dx > half {
                p.x -= world_width_px;
            } else if dx < -half {
                p.x += world_width_px;
            }
            prev_x = p.x;
        }
    }

    // Compute the unwrapped path's screen-x bounds so each shadow tile can
    // skip the whole render with a cheap `intersects` test when it can't
    // overlap the panel.
    let (path_x_min, path_x_max) = points
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), p| {
            (lo.min(p.x), hi.max(p.x))
        });
    let panel = painter.clip_rect();

    // Render the path at each shadow tile so a route that crosses the
    // antimeridian — and therefore has its unwrapped centre off-screen
    // for any view that doesn't include both endpoints in the central
    // tile — still shows up. Without this, panning the seam into the
    // middle of the panel makes the whole route silently disappear.
    for shift in shadow_offsets(world_width_px) {
        if !shift.is_finite() {
            continue;
        }
        if path_x_max + shift < panel.min.x || path_x_min + shift > panel.max.x {
            continue;
        }

        for (seg_idx, w) in points.windows(2).enumerate() {
            if let [a, b] = w {
                let a = shift_x(*a, shift);
                let b = shift_x(*b, shift);
                let seg_stat = segment_stats.and_then(|s| s.get(seg_idx));
                let seg_color = seg_stat
                    .map(|m| {
                        let t = m.mcr_01.clamp(0.0, 1.0) as f32;
                        egui::Color32::from_rgba_unmultiplied(
                            (255.0 * t) as u8,
                            0,
                            (255.0 * (1.0 - t)) as u8,
                            alpha,
                        )
                    })
                    .unwrap_or_else(|| egui::Color32::from_rgba_unmultiplied(255, 165, 0, alpha));
                painter.line_segment([a, b], egui::Stroke::new(2.0, seg_color));
            }
        }

        let last = points.len().saturating_sub(1);
        for (i, &p) in points.iter().enumerate() {
            let p = shift_x(p, shift);
            let color = if i == 0 {
                origin_color
            } else if i == last {
                destination_color
            } else {
                node_color
            };
            painter.circle_filled(p, 4.0, color);
        }

        match waypoint_label {
            Some(WaypointLabel::Index) => {
                draw_waypoint_index_labels(painter, &points, shift, alpha);
            }
            Some(WaypointLabel::TimeFrame {
                step_seconds,
                frame_count,
            }) => {
                if let Some(stats) = segment_stats {
                    draw_waypoint_time_labels(
                        painter,
                        &points,
                        shift,
                        alpha,
                        stats,
                        step_seconds,
                        frame_count,
                    );
                }
            }
            None => {}
        }
    }
}

/// Paint text on top of a translucent rounded-rect "pill" so it stays
/// readable on any map background (wind barbs, coastline fill, sea).
/// The pill's visual weight scales with the text itself, which avoids
/// the halo-style problem where a fixed-px outline starts to dominate
/// short glyphs at small font sizes. Text + pill colours flip with the
/// egui theme: black-on-white in light mode, white-on-black in dark.
/// The pill is opaque-ish so a label over busy wind barbs reads
/// cleanly; the trailing `alpha` parameter further multiplies it so
/// non-best particles (drawn at 128 alpha) get equally faded labels
/// when they're shown at all.
fn paint_label_with_pill(
    painter: &egui::Painter,
    pos: egui::Pos2,
    anchor: egui::Align2,
    text: &str,
    font: egui::FontId,
    alpha: u8,
) {
    let dark = painter.ctx().global_style().visuals.dark_mode;
    let (text_color, pill_color) = if dark {
        (
            egui::Color32::from_rgba_unmultiplied(240, 240, 240, alpha),
            egui::Color32::from_rgba_unmultiplied(0, 0, 0, mul_alpha(alpha, 200)),
        )
    } else {
        (
            egui::Color32::from_rgba_unmultiplied(10, 10, 10, alpha),
            egui::Color32::from_rgba_unmultiplied(255, 255, 255, mul_alpha(alpha, 215)),
        )
    };
    // Layout once so we can size the pill before drawing the text.
    let galley = painter.layout_no_wrap(text.to_owned(), font, text_color);
    let text_rect = anchor.anchor_size(pos, galley.size());
    // Small horizontal + vertical padding gives the glyphs breathing
    // room against the rounded corners without looking sticker-thick.
    let pill_rect = text_rect.expand2(egui::Vec2::new(3.0, 1.0));
    painter.rect_filled(pill_rect, 3.0, pill_color);
    painter.galley(text_rect.min, galley, text_color);
}

/// Multiply two 0-255 alphas in fixed-point (rounded) so combining a
/// per-particle alpha with a per-style opacity gives a value that
/// stays in `[0, 255]` without an `f32` round-trip per label.
fn mul_alpha(a: u8, b: u8) -> u8 {
    ((u16::from(a) * u16::from(b) + 127) / 255) as u8
}

/// Per-waypoint ordinal label (`0` = start, `N-1` = end). Cheap default
/// hint for navigating the route without crowding the central panel
/// with full per-segment timings.
fn draw_waypoint_index_labels(
    painter: &egui::Painter,
    points: &[egui::Pos2],
    shift: f32,
    alpha: u8,
) {
    for (i, &p) in points.iter().enumerate() {
        let p = shift_x(p, shift);
        paint_label_with_pill(
            painter,
            p + egui::Vec2::new(8.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            &format!("{i}"),
            egui::FontId::proportional(12.0),
            alpha,
        );
    }
}

/// Per-waypoint cumulative-time labels of the form `"1h32m20s [N]"`, where
/// `N` is the integer wind-map frame the waypoint lands on. Extracted from
/// [`draw_path`] purely to keep the parent under clippy's line budget.
fn draw_waypoint_time_labels(
    painter: &egui::Painter,
    points: &[egui::Pos2],
    shift: f32,
    alpha: u8,
    stats: &[SegmentMetrics],
    step_seconds: f32,
    frame_count: usize,
) {
    let step = f64::from(step_seconds);
    let nt = frame_count as i64;
    let mut cumulative = 0.0_f64;
    for (i, &p) in points.iter().enumerate() {
        let p = shift_x(p, shift);
        if i > 0
            && let Some(m) = stats.get(i - 1)
        {
            cumulative += m.time;
        }
        // Wrap modularly to mirror `BakedWindMap::sample_wind`: a waypoint that
        // arrives past the loaded duration sees the field's periodic repeat.
        let frame_idx = if step > 0.0 && nt > 0 {
            ((cumulative / step).round() as i64).rem_euclid(nt)
        } else {
            0
        };
        paint_label_with_pill(
            painter,
            p + egui::Vec2::new(8.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            &format!("{} [{frame_idx}]", format_hms(cumulative)),
            egui::FontId::proportional(12.0),
            alpha,
        );
    }
}

fn format_hms(seconds: f64) -> String {
    let total = seconds.max(0.0).round() as u64;
    let h = total / 3600;
    let m = (total % 3600) / 60;
    let s = total % 60;
    format!("{h}h{m:02}m{s:02}s")
}

/// Render the benchmark route (A* + time-only PSO) as a dashed reference
/// overlay so the user can see at a glance whether — and where — the
/// main PSO improved on the obvious shortest-path solution. Drawn
/// underneath the gbest path (so the gbest stays the visual focus) in
/// a desaturated cyan that contrasts with the gbest's orange without
/// competing for attention.
pub(crate) fn draw_benchmark_route(
    painter: &egui::Painter,
    benchmark: &BenchmarkRoute,
    view: &ViewTransform,
) {
    let stroke_color = egui::Color32::from_rgba_unmultiplied(80, 180, 200, 200);
    let dash_len = 8.0;
    let gap_len = 6.0;

    let mut points: Vec<egui::Pos2> = benchmark
        .waypoints
        .iter()
        .map(|&(lon, lat)| view.route_to_screen(lon, lat))
        .collect();
    if points.len() < 2 {
        return;
    }

    // Same antimeridian-unwrap logic as draw_path: shift consecutive
    // points by ±world_width when the screen-x jump exceeds half a world.
    let world_width_px = world_width_pixels(view);
    if world_width_px.is_finite() && world_width_px > 0.0 {
        let half = world_width_px * 0.5;
        let mut prev_x = points.first().map_or(0.0, |p| p.x);
        for p in points.iter_mut().skip(1) {
            let dx = p.x - prev_x;
            if dx > half {
                p.x -= world_width_px;
            } else if dx < -half {
                p.x += world_width_px;
            }
            prev_x = p.x;
        }
    }

    let (path_x_min, path_x_max) = points
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), p| {
            (lo.min(p.x), hi.max(p.x))
        });
    let panel = painter.clip_rect();

    for shift in shadow_offsets(world_width_px) {
        if !shift.is_finite() {
            continue;
        }
        if path_x_max + shift < panel.min.x || path_x_min + shift > panel.max.x {
            continue;
        }
        for w in points.windows(2) {
            if let [a, b] = w {
                let a = shift_x(*a, shift);
                let b = shift_x(*b, shift);
                draw_dashed_segment(painter, a, b, stroke_color, dash_len, gap_len);
            }
        }
    }
}

/// Render a dashed line from `a` to `b` by stepping a unit vector along
/// the segment. Pure egui — no helper for dashed strokes.
fn draw_dashed_segment(
    painter: &egui::Painter,
    a: egui::Pos2,
    b: egui::Pos2,
    color: egui::Color32,
    dash_len: f32,
    gap_len: f32,
) {
    let total = (b - a).length();
    if total < 1e-3 {
        return;
    }
    let dir = (b - a) / total;
    let stride = dash_len + gap_len;
    let mut t = 0.0;
    while t < total {
        let dash_end = (t + dash_len).min(total);
        let p0 = a + dir * t;
        let p1 = a + dir * dash_end;
        painter.line_segment([p0, p1], egui::Stroke::new(1.5, color));
        t += stride;
    }
}

#[expect(
    clippy::too_many_arguments,
    reason = "render orchestrator pulls together view, evolution, baked wind, boat, bounds, weights, and stats — splitting any of these into a struct would just shuffle the parameter list"
)]
pub(crate) fn render_route_evolution<const N: usize>(
    painter: &egui::Painter,
    view: &ViewTransform,
    evolution: &Evolution<Path<N>>,
    iteration: usize,
    show_all_particles: bool,
    baked_wind_map: Option<&BakedWindMap>,
    boat: Option<&Boat>,
    route_bounds: Option<RouteBounds>,
    weights: SearchWeights,
    segment_stats: &mut Option<Vec<SegmentMetrics>>,
    best_fitness: &mut Option<f64>,
    waypoint_label: Option<WaypointLabel>,
) {
    let frames = evolution.frames();
    let iter_idx = iteration.min(frames.len().saturating_sub(1));
    let Some(particles) = frames.get(iter_idx) else {
        return;
    };
    // Stats need all three of: baked wind, boat, and bounds. They're set
    // together by the search worker (and by the solution-load path), so in
    // practice they're aligned, but the type doesn't enforce that — zip them
    // explicitly rather than reaching for `expect`.
    let stats_ctx = baked_wind_map.zip(boat).zip(route_bounds);
    if show_all_particles {
        // Per-waypoint labels would stack illegibly across particles, so they're
        // suppressed here even when `waypoint_label_data` is set. The best
        // particle is drawn as part of this loop (at the same 128-alpha as the
        // rest); its stats are recomputed below for the side panel.
        for particle in particles {
            let stats = stats_ctx.map(|((bwm, b), rb)| {
                compute_segment_metrics(b, bwm, particle.best_pos, rb.step_distance_max)
            });
            draw_path(
                painter,
                &particle.best_pos,
                view,
                128,
                stats.as_deref(),
                None,
            );
        }
    }
    let best = particles.iter().max_by(|a, b| {
        a.best_fit
            .partial_cmp(&b.best_fit)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    if let Some(best_particle) = best {
        let stats = stats_ctx.map(|((bwm, b), rb)| {
            compute_segment_metrics(b, bwm, best_particle.best_pos, rb.step_distance_max)
        });
        if !show_all_particles {
            draw_path(
                painter,
                &best_particle.best_pos,
                view,
                255,
                stats.as_deref(),
                waypoint_label,
            );
        }
        // Recompute fitness from the (possibly user-mutated) path geometry
        // so the Summary panel tracks waypoint drags / time-reopt result
        // rather than holding the search-time value. Routed through the
        // shared `weighted_fitness` helper so the formula stays in lockstep
        // with `SailboatFitCalc::calculate_fit`. Falls back to the cached
        // `best_fit` when we lack the context to recompute (no baked wind /
        // boat / bounds).
        let recomputed_fit = stats.as_deref().map(|s| {
            let total_time: f64 = s.iter().map(|m| m.time).sum();
            let total_fuel: f64 = s.iter().map(|m| m.fuel).sum();
            let total_land: f64 = s.iter().map(|m| m.land_metres).sum();
            weighted_fitness(
                total_time,
                total_fuel,
                total_land,
                weights.time_weight,
                weights.fuel_weight,
                weights.land_weight,
            )
        });
        *segment_stats = stats;
        *best_fitness = Some(recomputed_fit.unwrap_or(best_particle.best_fit));
    }
}
