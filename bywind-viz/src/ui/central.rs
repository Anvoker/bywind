use super::{
    LonLatBbox, MAP_PADDING, MapBounds, SCALE_MAX, Tool, ViewTransform, draw_benchmark_route,
    draw_coastlines, draw_endpoint_markers, draw_minimap, draw_route_bounds, draw_windmap,
    min_render_scale, powered_by_egui_and_eframe, render_route_evolution, route_evolution_match,
};
use crate::app::BywindApp;

impl BywindApp {
    /// Central panel: builds the per-frame `ViewTransform`, draws the wind
    /// barbs and route, dispatches tool input via `tool_interaction`, and
    /// overlays the bottom-right minimap.
    pub(crate) fn render_central_panel(&mut self, ui: &mut egui::Ui) {
        egui::CentralPanel::default().show_inside(ui, |ui| {
            ui.set_min_height(256.0);
            ui.set_min_width(256.0);

            let panel_rect = ui.max_rect();
            // The bundled `wind_av1` sample takes ~12 s to decode on a
            // pure-Rust rav1d build, during which the random fallback
            // is still on screen. Surface a status banner so users
            // don't think the world map is missing. Floated in a
            // foreground `egui::Area` so it doesn't carve layout space
            // out of the central panel and so it paints *on top* of
            // any coastline / barb / route the panel's painter writes
            // later in the frame.
            if self.bundled_sample_job.is_running() {
                egui::Area::new(egui::Id::new("central_panel_decode_toast"))
                    .order(egui::Order::Foreground)
                    .fixed_pos(panel_rect.left_top() + egui::Vec2::splat(MAP_PADDING * 0.5))
                    .interactable(false)
                    .show(ui.ctx(), |ui| {
                        ui.horizontal(|ui| {
                            ui.spinner();
                            ui.label("Decoding bundled wind sample…");
                        });
                    });
            }
            self.view.last_panel_height = panel_rect.height();
            self.view.last_panel_rect = Some(panel_rect);
            let min_scale = min_render_scale(panel_rect.height());
            self.view.render_scale = self.view.render_scale.max(min_scale);
            let base_offset = panel_rect.min.to_vec2() + egui::Vec2::splat(MAP_PADDING);

            // Consume a "Fit to view" request before building the ViewTransform
            // so the new scale and pan_offset apply this same frame.
            if std::mem::take(&mut self.view.autoscale_pending) {
                let bounds = self.wind_map.as_ref().and_then(MapBounds::from_wind_map);
                if let Some(bounds) = bounds {
                    self.fit_view_to_bounds(panel_rect, bounds);
                }
            }

            // Same idea, but framed on the freshly loaded route rather than
            // the full wind-map extent. Falls back to the wind-map fit so
            // there's still some sensible framing if the scenario lacked
            // both endpoints and a bbox.
            if std::mem::take(&mut self.view.fit_route_pending) {
                let bounds = self
                    .route_fit_bounds()
                    .or_else(|| self.wind_map.as_ref().and_then(MapBounds::from_wind_map));
                if let Some(bounds) = bounds {
                    self.fit_view_to_bounds(panel_rect, bounds);
                }
            }

            // Acquire the panel's input response before building the view
            // so wheel zoom can mutate `render_scale` / `pan_offset` and
            // have the change applied this same frame to the rendering.
            let response = ui.interact(panel_rect, ui.id(), egui::Sense::click_and_drag());
            self.handle_wheel_zoom(ui, &response, base_offset, min_scale);

            // Equirectangular projection origin: persistent across frames so
            // horizontal pan-wrap can rotate `lon0` past the antimeridian
            // without snapping back. Initialise from the wind-map bbox
            // centre on first frame after a map load (`view_lon0` is reset
            // to `None` whenever the map changes); the user's panning then
            // takes over.
            if self.view.view_lon0.is_none()
                && let Some(MapBounds { bbox }) =
                    self.wind_map.as_ref().and_then(MapBounds::from_wind_map)
            {
                self.view.view_lon0 = Some(((bbox.lon_min + bbox.lon_max) * 0.5) as f32);
                self.view.view_lat0 = Some(((bbox.lat_min + bbox.lat_max) * 0.5) as f32);
            }
            let lat0 = self.view.view_lat0.unwrap_or(0.0);
            // Pan-wrap: if accumulated horizontal pan has carried the user
            // more than half a world-width past the seam, fold the
            // displacement into `view_lon0` so panning effectively wraps
            // around the globe. Same content lands at the same screen
            // pixel before and after the fold.
            self.apply_pan_wrap(lat0);
            let lon0 = self.view.view_lon0.unwrap_or(0.0);
            let view = ViewTransform {
                lat0,
                lon0,
                cos_lat0: lat0.to_radians().cos(),
                offset: base_offset + self.view.pan_offset,
                render_scale: self.view.render_scale,
            };

            if let Some(wind_map) = &self.wind_map {
                let current_frame = self.editor.current_frame;
                // Direct frame access when the slider is inside the
                // data span; otherwise build (or reuse) a synthesised
                // frame at the requested time. The cache keys on
                // frame index so dragging the slider re-uses the
                // synth result across many render frames per tick.
                let direct_frame = wind_map.frame(current_frame);
                let frame_to_draw = if let Some(frame) = direct_frame {
                    frame
                } else {
                    let needs_rebuild = self
                        .view
                        .synthesized_frame
                        .as_ref()
                        .is_none_or(|(k, _)| *k != current_frame);
                    if needs_rebuild {
                        let t = current_frame as f32 * wind_map.step_seconds();
                        let synth = wind_map.synthesize_frame_at(t);
                        self.view.synthesized_frame = Some((current_frame, synth));
                    }
                    // SAFETY-equivalent: just unwrapped because the
                    // branch above guarantees `Some`.
                    &self
                        .view
                        .synthesized_frame
                        .as_ref()
                        .expect("synthesised frame set above")
                        .1
                };
                draw_windmap(
                    ui.painter(),
                    frame_to_draw,
                    &view,
                    egui::Stroke::new(1.0, egui::Color32::BLUE),
                );
            }

            // Coastlines are drawn after the barbs so the halo masks
            // any barb that would otherwise sit right on the coastline,
            // keeping continents legible at any barb density. The
            // landmass geometry is in `(lon, lat)`; the view projects
            // each vertex on draw via `map_to_screen`.
            draw_coastlines(ui.painter(), &view, crate::coastlines::landmasses());

            // On-map waypoint labels: nothing in "Show all particles" (text
            // would just stack illegibly across the cloud). With the Set
            // Time From Waypoint tool, show cumulative arrival time + frame
            // index — the user is choosing which waypoint to snap to. Any
            // other tool gets a plain ordinal hint so the route's waypoint
            // ordering is readable without crowding the panel with timings.
            let waypoint_label = if self.view.show_all_particles {
                None
            } else if self.editor.selected_tool == Tool::WaypointTime {
                self.wind_map
                    .as_ref()
                    .map(|wm| crate::draw::WaypointLabel::TimeFrame {
                        step_seconds: wm.step_seconds(),
                        frame_count: wm.frame_count(),
                    })
            } else {
                Some(crate::draw::WaypointLabel::Index)
            };

            // Draw the benchmark route first so the gbest renders on
            // top (gbest is the visual focus; benchmark is a faint
            // reference). Suppressed when "show all particles" is on
            // since it would blend into the particle cloud.
            if !self.view.show_all_particles
                && let Some(bench) = self.outputs.benchmark.as_ref()
            {
                draw_benchmark_route(ui.painter(), bench, &view);
            }

            if let Some(re) = &self.outputs.route_evolution {
                let weights = bywind::SearchWeights {
                    time_weight: self.search.time_weight,
                    fuel_weight: self.search.fuel_weight,
                    land_weight: self.search.land_weight,
                };
                route_evolution_match!(re, |evolution| render_route_evolution(
                    ui.painter(),
                    &view,
                    evolution,
                    self.outputs.iteration,
                    self.view.show_all_particles,
                    self.outputs.baked_wind_map.as_ref(),
                    self.outputs.boat.as_ref(),
                    self.outputs.route_bounds,
                    weights,
                    &mut self.outputs.segment_stats,
                    &mut self.outputs.best_fitness,
                    waypoint_label,
                ));
            }

            self.tool_interaction(ui, ui.ctx(), &view, &response);

            // The route-bounds rectangle is only relevant while the user
            // is editing it; hiding it on other tools keeps the map
            // uncluttered. The bbox itself stays in `EditorState`, so
            // switching back to the Route Bounds tool brings it back.
            if self.editor.selected_tool == Tool::RouteBounds {
                let live_drag = if response.dragged()
                    && let (Some(anchor), Some(current)) = (
                        self.editor.route_bbox_drag_anchor,
                        response.interact_pointer_pos(),
                    ) {
                    Some((anchor, current))
                } else {
                    None
                };
                draw_route_bounds(ui.painter(), &view, self.editor.route_bbox, live_drag);
            }

            draw_endpoint_markers(
                ui.painter(),
                &view,
                self.editor.start_waypoint,
                self.editor.end_waypoint,
            );

            if let Some(wind_map) = &self.wind_map
                && let Some(bounds) = MapBounds::from_wind_map(wind_map)
            {
                let gbest = self
                    .outputs
                    .route_evolution
                    .as_ref()
                    .and_then(|re| re.gbest_at(self.outputs.iteration));
                let route = gbest.as_ref().map(|g| (g.xs, g.ys));
                draw_minimap(ui.painter(), panel_rect, &bounds, &view, route);
            }

            ui.with_layout(egui::Layout::bottom_up(egui::Align::Center), |ui| {
                ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                    powered_by_egui_and_eframe(ui);
                    egui::warn_if_debug_build(ui);
                });
            });
        });
    }

    /// Pick `render_scale` so the wind-map bounds fit inside the panel's
    /// drawable area (panel rect minus `MAP_PADDING` on every side), then
    /// pick `pan_offset` so the bounds' centre lands at the panel centre.
    /// Scale is clamped to the slider range so the slider remains usable as
    /// an adjustment knob after fitting.
    /// Mouse-wheel zoom on the central panel. While the cursor is over
    /// the panel, vertical scroll multiplies `render_scale` by an
    /// exponential factor (so each wheel "click" feels uniform across
    /// zoom levels), clamped to the slider range. `pan_offset` is
    /// adjusted simultaneously so the map point under the cursor stays
    /// under the cursor — i.e. zoom-to-cursor, the standard interaction
    /// for any 2D map view.
    fn handle_wheel_zoom(
        &mut self,
        ui: &egui::Ui,
        response: &egui::Response,
        base_offset: egui::Vec2,
        min_scale: f32,
    ) {
        if !response.hovered() {
            return;
        }
        let (scroll_y, cursor) = ui.input(|i| (i.smooth_scroll_delta.y, i.pointer.hover_pos()));
        let Some(cursor) = cursor else { return };
        if scroll_y == 0.0 {
            return;
        }

        // Empirically-tuned rate: ~1 wheel click ≈ 50–100 px of
        // smooth_scroll_delta on Windows; this gives a 1.13–1.28×
        // factor per click, which feels neither sluggish nor jumpy.
        const RATE: f32 = 0.0025;
        let factor = (scroll_y * RATE).exp();
        let old_scale = self.view.render_scale;
        let new_scale = (old_scale * factor).clamp(min_scale, SCALE_MAX);
        if (new_scale - old_scale).abs() < f32::EPSILON {
            return;
        }

        // Map point currently under the cursor in the *pre-zoom* view.
        let total_offset = base_offset + self.view.pan_offset;
        let map_x = (cursor.x - total_offset.x) / old_scale;
        let map_y = (cursor.y - total_offset.y) / old_scale;

        // Solve for the new pan_offset that keeps (map_x, map_y) under
        // the cursor at the new scale:
        //   cursor = (map_x, map_y) * new_scale + base_offset + new_pan
        self.view.render_scale = new_scale;
        self.view.pan_offset = egui::Vec2::new(
            cursor.x - map_x * new_scale - base_offset.x,
            cursor.y - map_y * new_scale - base_offset.y,
        );
    }

    /// Adjust `pan_offset` so the central panel's centre lands on the
    /// same map point at the new scale that it did at `old_scale`.
    /// Called after the View slider / steppers / typed scale value
    /// move `render_scale`, since none of those carry a cursor anchor
    /// the way mouse-wheel zoom does. No-op until the central panel
    /// has rendered at least once (its rect populates
    /// `view.last_panel_rect`).
    pub(super) fn zoom_around_panel_centre(&mut self, old_scale: f32) {
        let Some(panel_rect) = self.view.last_panel_rect else {
            return;
        };
        let new_scale = self.view.render_scale;
        if (new_scale - old_scale).abs() < f32::EPSILON || old_scale <= 0.0 {
            return;
        }
        let base_offset = panel_rect.min.to_vec2() + egui::Vec2::splat(MAP_PADDING);
        let anchor = panel_rect.center();
        let total_offset = base_offset + self.view.pan_offset;
        let map_x = (anchor.x - total_offset.x) / old_scale;
        let map_y = (anchor.y - total_offset.y) / old_scale;
        self.view.pan_offset = egui::Vec2::new(
            anchor.x - map_x * new_scale - base_offset.x,
            anchor.y - map_y * new_scale - base_offset.y,
        );
    }

    /// Fill any unset endpoint slot with the wind-map's bbox corner so
    /// the user sees the implicit defaults (`(x_min, y_min)` /
    /// `(x_max, y_max)` — the same fallback `MapBounds::resolve_endpoints`
    /// uses) the moment they activate the Endpoints tool. Already-set
    /// values are left alone so this never clobbers a user's placement.
    /// No-op when no wind map is loaded yet.
    pub(super) fn populate_endpoint_defaults_if_unset(&mut self) {
        let Some(wind_map) = &self.wind_map else {
            return;
        };
        let Some(bounds) = MapBounds::from_wind_map(wind_map) else {
            return;
        };
        let bbox = bounds.bbox;
        if self.editor.start_waypoint.is_none() {
            self.editor.start_waypoint = Some((bbox.lon_min, bbox.lat_min));
        }
        if self.editor.end_waypoint.is_none() {
            self.editor.end_waypoint = Some((bbox.lon_max, bbox.lat_max));
        }
    }

    /// Fill the route-bounds rectangle with the full wind-map bbox if
    /// the user hasn't drawn one yet. Mirrors the implicit default that
    /// `MapBounds::clamp_to(None)` falls back to, just made visible.
    /// When endpoints are placed, the auto-derive path
    /// ([`Self::auto_set_route_bbox`]) wins and produces a tighter,
    /// A*-informed rectangle instead.
    pub(super) fn populate_route_bbox_default_if_unset(&mut self) {
        if self.editor.route_bbox.is_some() {
            return;
        }
        if self.auto_set_route_bbox() {
            return;
        }
        let Some(wind_map) = &self.wind_map else {
            return;
        };
        let Some(bounds) = MapBounds::from_wind_map(wind_map) else {
            return;
        };
        let bbox = bounds.bbox;
        self.editor.route_bbox = Some((bbox.lon_min, bbox.lon_max, bbox.lat_min, bbox.lat_max));
    }

    /// Derive a route-bounds rectangle from the current endpoints + the
    /// loaded wind map's extent via [`bywind::derive_route_bbox`], and
    /// commit it as the auto-managed bbox. Returns `true` on success
    /// so callers can distinguish the "no endpoints / no wind map"
    /// fallback path. Sets `route_bbox_auto = true` so subsequent
    /// endpoint moves re-derive (until the user manually edits).
    pub(crate) fn auto_set_route_bbox(&mut self) -> bool {
        let Some(wind_map) = &self.wind_map else {
            return false;
        };
        let Some(map_bounds) = MapBounds::from_wind_map(wind_map) else {
            return false;
        };
        let Some(start) = self.editor.start_waypoint else {
            return false;
        };
        let Some(end) = self.editor.end_waypoint else {
            return false;
        };
        let Some(derived) =
            bywind::derive_route_bbox(start, end, bywind::landmass_grid(), Some(map_bounds))
        else {
            return false;
        };
        let dbox = derived.bbox;
        self.editor.route_bbox = Some((dbox.lon_min, dbox.lon_max, dbox.lat_min, dbox.lat_max));
        self.editor.route_bbox_auto = true;
        self.editor.last_auto_endpoints = Some((start, end));
        true
    }

    /// Per-frame hook: when auto-mode is on and the endpoints have
    /// moved (or were just placed), recompute the bbox. Cheap but not
    /// free, so the cached `last_auto_endpoints` short-circuits the
    /// common case where nothing changed since the last frame.
    pub(crate) fn update_auto_route_bbox(&mut self) {
        if !self.editor.route_bbox_auto {
            return;
        }
        let (Some(start), Some(end)) = (self.editor.start_waypoint, self.editor.end_waypoint)
        else {
            // No endpoints yet — nothing to auto-derive. Forget any
            // stale cache so a re-placement triggers a fresh derive.
            self.editor.last_auto_endpoints = None;
            return;
        };
        if self.editor.last_auto_endpoints == Some((start, end)) {
            return;
        }
        self.auto_set_route_bbox();
    }

    /// Wrap accumulated horizontal pan around the globe.
    ///
    /// One full world (`360°` of longitude) projected at the current scale
    /// is `world_width_px = 360 · cos(lat0) · METRES_PER_DEGREE ·
    /// render_scale`. Whenever `|pan_offset.x|` has grown past half a world,
    /// fold a 360° shift into `view_lon0` and offset `pan_offset.x` by
    /// the matching pixel amount — the projection of any geographic point
    /// is unchanged across the fold (same screen pixel) so there's no
    /// visual jump. Looped because a fast pan can carry past multiple
    /// worlds in one frame.
    ///
    /// Skipped when the world is sub-pixel (extreme zoom-out or near a
    /// pole where `cos(lat0)` collapses) — the fold would oscillate
    /// indefinitely without bringing visible content closer to centre.
    fn apply_pan_wrap(&mut self, lat0: f32) {
        let world_width_px = 360.0
            * lat0.to_radians().cos()
            * crate::view::METRES_PER_DEGREE
            * self.view.render_scale;
        if !world_width_px.is_finite() || world_width_px < 1.0 {
            return;
        }
        let half = world_width_px * 0.5;
        let Some(lon0) = self.view.view_lon0 else {
            return;
        };
        let mut new_lon0 = lon0;
        let mut new_pan_x = self.view.pan_offset.x;
        // Each wrap fold: lon0 → lon0 ∓ 360°, pan.x → pan.x ∓ world_width_px.
        // The two cancel in `screen.x = (lon - lon0) · k + pan.x` (with
        // `k = cos_lat0 · MPD · render_scale = world_width_px / 360`).
        while new_pan_x > half {
            new_pan_x -= world_width_px;
            new_lon0 -= 360.0;
        }
        while new_pan_x < -half {
            new_pan_x += world_width_px;
            new_lon0 += 360.0;
        }
        // Keep `view_lon0` in (−180, 180]; the projection is purely linear
        // in `lon - lon0` so an arbitrarily large `lon0` would still draw
        // the right pixels, but the canonical range keeps printf debug-
        // ging readable.
        if (new_lon0 - lon0).abs() > 0.0 {
            self.view.view_lon0 = Some(swarmkit_sailing::spherical::wrap_lon_deg_f32(new_lon0));
            self.view.pan_offset.x = new_pan_x;
        }
    }

    /// Bounds for the "fit to route" path: the user-set route bbox if
    /// present, else the lon/lat span of the placed endpoints, padded by
    /// 15 % (and at least 1°) on each side so the route isn't pressed
    /// against the panel edges. `None` if neither is available — the
    /// caller falls back to the wind-map fit. The simple linear span
    /// here overshoots for antimeridian-crossing routes; that case is
    /// noted on `ViewState::fit_route_pending` and would need a separate
    /// shortest-arc fit to address.
    fn route_fit_bounds(&self) -> Option<MapBounds> {
        let (lon_min, lon_max, lat_min, lat_max) = if let Some(bbox) = self.editor.route_bbox {
            bbox
        } else {
            let s = self.editor.start_waypoint?;
            let e = self.editor.end_waypoint?;
            (s.0.min(e.0), s.0.max(e.0), s.1.min(e.1), s.1.max(e.1))
        };
        let lon_pad = ((lon_max - lon_min) * 0.15).max(1.0);
        let lat_pad = ((lat_max - lat_min) * 0.15).max(1.0);
        Some(MapBounds {
            bbox: LonLatBbox::new(
                lon_min - lon_pad,
                lon_max + lon_pad,
                (lat_min - lat_pad).max(-89.99),
                (lat_max + lat_pad).min(89.99),
            ),
        })
    }

    fn fit_view_to_bounds(&mut self, panel_rect: egui::Rect, bounds: MapBounds) {
        // Bounds are now in degrees. Convert to *projected metres* using
        // the same equirectangular projection the ViewTransform applies,
        // anchored at the bbox centre, then size the scale so the
        // projected map fits the panel.
        use crate::view::METRES_PER_DEGREE;
        let bbox = bounds.bbox;
        let lat0 = ((bbox.lat_min + bbox.lat_max) * 0.5) as f32;
        let lon0 = ((bbox.lon_min + bbox.lon_max) * 0.5) as f32;
        let cos_lat0 = lat0.to_radians().cos();
        // Re-anchor the view origin on fit. The user may have pan-wrapped
        // away from the bbox centre; "Fit to view" reverts that so the map
        // is actually centred on its data again.
        self.view.view_lon0 = Some(lon0);
        self.view.view_lat0 = Some(lat0);
        let map_w_m = (bbox.lon_max - bbox.lon_min) as f32 * cos_lat0 * METRES_PER_DEGREE;
        let map_h_m = (bbox.lat_max - bbox.lat_min) as f32 * METRES_PER_DEGREE;
        if map_w_m <= 0.0 || map_h_m <= 0.0 {
            return;
        }
        let avail_w = (panel_rect.width() - 2.0 * MAP_PADDING).max(1.0);
        let avail_h = (panel_rect.height() - 2.0 * MAP_PADDING).max(1.0);
        let min_scale = min_render_scale(panel_rect.height());
        let scale = (avail_w / map_w_m)
            .min(avail_h / map_h_m)
            .clamp(min_scale, SCALE_MAX);
        self.view.render_scale = scale;

        // The bbox centre maps to projected `(0, 0)` (it's the projection
        // origin used by the ViewTransform built each frame), so centring
        // it on the panel just means setting `pan_offset` so the
        // projected origin lands on the panel midpoint.
        let half_size = panel_rect.size() * 0.5;
        self.view.pan_offset = half_size - egui::Vec2::splat(MAP_PADDING);
    }

    /// Floating window in the bottom-right corner showing the most recent
    /// reported error. Cloning the message keeps the borrow checker happy when
    /// the Dismiss button needs `&mut self`.
    pub(crate) fn render_error_toast(&mut self, ui: &egui::Ui) {
        let Some(message) = self.last_error.clone() else {
            return;
        };
        let mut dismiss = false;
        egui::Window::new("Error")
            .anchor(egui::Align2::RIGHT_BOTTOM, [-12.0, -12.0])
            .resizable(false)
            .collapsible(false)
            .show(ui.ctx(), |ui| {
                ui.label(&message);
                if ui.button("Dismiss").clicked() {
                    dismiss = true;
                }
            });
        if dismiss {
            self.last_error = None;
        }
    }
}
