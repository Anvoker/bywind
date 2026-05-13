use crate::app::BywindApp;
use crate::config::Tool;
use crate::view::ViewTransform;

impl BywindApp {
    /// Returns the index of the gbest waypoint nearest to `screen_pos`, or `None`
    /// if no waypoint lies within `max_dist` pixels or no search result is loaded.
    /// Used by the Waypoint Edit tool both to pick a waypoint to drag and to
    /// switch the cursor to "grab" while hovering over one.
    pub(crate) fn find_closest_waypoint(
        &self,
        screen_pos: egui::Pos2,
        view: &ViewTransform,
        max_dist: f32,
    ) -> Option<usize> {
        let gbest = self
            .outputs
            .route_evolution
            .as_ref()?
            .gbest_at(self.outputs.iteration)?;
        let max_dist_sq = max_dist * max_dist;
        let mut closest_idx = 0usize;
        let mut closest_dist_sq = f32::INFINITY;
        for (i, (&x, &y)) in gbest.xs.iter().zip(gbest.ys.iter()).enumerate() {
            let p = view.route_to_screen(x, y);
            let d_sq = (p - screen_pos).length_sq();
            if d_sq < closest_dist_sq {
                closest_dist_sq = d_sq;
                closest_idx = i;
            }
        }
        if closest_dist_sq <= max_dist_sq {
            Some(closest_idx)
        } else {
            None
        }
    }

    /// Translate the gbest path's xy at `idx` by `screen_delta`. The next render
    /// recomputes segment stats from the mutated path, so per-segment, total,
    /// and frame-index labels all update on the next frame.
    pub(crate) fn mutate_gbest_waypoint(
        &mut self,
        idx: usize,
        screen_delta: egui::Vec2,
        view: &ViewTransform,
    ) {
        let (dx, dy) = view.screen_delta_to_route(screen_delta);
        if let Some(re) = self.outputs.route_evolution.as_mut() {
            re.mutate_waypoint(self.outputs.iteration, idx, dx, dy);
        }
    }

    /// Snap `current_frame` to the time slice closest to when the ship arrives at
    /// the waypoint nearest to `click_pos`. Cumulative arrival time at waypoint
    /// `i` is the sum of segment times `[0..i]` from the cached per-segment
    /// metrics. Does nothing if no search has run, no segment stats are cached
    /// (e.g. while showing all particles), or there is no wind map.
    pub(crate) fn snap_to_waypoint_time(&mut self, click_pos: egui::Pos2, view: &ViewTransform) {
        if self.outputs.route_evolution.is_none() {
            return;
        }
        let Some(stats) = &self.outputs.segment_stats else {
            return;
        };
        let Some(wind_map) = &self.wind_map else {
            return;
        };

        let waypoint_idx = self
            .find_closest_waypoint(click_pos, view, f32::INFINITY)
            .unwrap_or(0);

        let cumulative_time: f64 = stats.iter().take(waypoint_idx).map(|m| m.time).sum();
        let step_seconds = f64::from(wind_map.step_seconds());
        if step_seconds <= 0.0 {
            return;
        }
        let frame_count = wind_map.frame_count() as i64;
        if frame_count <= 0 {
            return;
        }
        // Wind data is treated as periodic: a route that runs past the loaded
        // duration wraps back to frame 0 (matches `BakedWindMap::sample_wind`).
        let target_frame = (cumulative_time / step_seconds).round() as i64;
        self.editor.current_frame = target_frame.rem_euclid(frame_count) as usize;
    }

    /// Central-panel input dispatch: brush-size keyboard adjustment plus the
    /// per-tool click/drag handler. Called once per frame from
    /// `render_central_panel`.
    pub(crate) fn tool_interaction(
        &mut self,
        ui: &egui::Ui,
        ctx: &egui::Context,
        view: &ViewTransform,
        response: &egui::Response,
    ) {
        self.handle_brush_keys(ui);
        match self.editor.selected_tool {
            Tool::Pointer => self.handle_pointer_tool(ctx, response),
            Tool::WaypointEdit => self.handle_waypoint_edit_tool(ui, ctx, view, response),
            Tool::Speed => self.handle_speed_tool(ui, view, response),
            Tool::WaypointTime => self.handle_waypoint_time_tool(view, response),
            Tool::Direction => self.handle_direction_tool(ui, view, response),
            Tool::Endpoint => self.handle_endpoint_tool(ctx, view, response),
            Tool::RouteBounds => self.handle_route_bounds_tool(ctx, view, response),
        }
    }

    /// `+` / `=` enlarge the paint brush, `-` shrinks. Active regardless of
    /// tool so the user can pre-adjust before switching. Brush radius is
    /// in degrees of the wind-map coordinate system; the floor of 0.05°
    /// ≈ 5 km keeps a press of `−` from clamping the brush to zero.
    fn handle_brush_keys(&mut self, ui: &egui::Ui) {
        const BRUSH_MIN_DEG: f32 = 0.05;
        ui.input(|i| {
            if i.key_pressed(egui::Key::Plus) || i.key_pressed(egui::Key::Equals) {
                let change = (self.editor.brush_radius * 0.1).max(BRUSH_MIN_DEG);
                self.editor.brush_radius += change;
            }
            if i.key_pressed(egui::Key::Minus) {
                let change = (self.editor.brush_radius * 0.1).max(BRUSH_MIN_DEG);
                self.editor.brush_radius = (self.editor.brush_radius - change).max(BRUSH_MIN_DEG);
            }
        });
    }

    /// Brush-circle visual at the pointer for the paint tools (Speed/Direction).
    /// Drawn black-then-white so it's visible against any wind-barb background.
    fn draw_brush_overlay(&self, ui: &egui::Ui, view: &ViewTransform) {
        if let Some(hover_pos) = ui.input(|i| i.pointer.hover_pos())
            && ui.max_rect().contains(hover_pos)
        {
            // `brush_radius` is in degrees (wind-map coords). Convert to
            // screen pixels via the projection: `degrees * cos_lat0 *
            // metres-per-degree * pixels-per-metre`. Using `cos_lat0` (the
            // bbox-centre cos) keeps the brush a stable size visually
            // regardless of where the cursor is on the map.
            let screen_radius = self.editor.brush_radius
                * view.cos_lat0
                * crate::view::METRES_PER_DEGREE
                * view.render_scale;
            ui.painter().circle_stroke(
                hover_pos,
                screen_radius,
                egui::Stroke::new(3.0, egui::Color32::BLACK),
            );
            ui.painter().circle_stroke(
                hover_pos,
                screen_radius,
                egui::Stroke::new(1.0, egui::Color32::WHITE),
            );
        }
    }

    /// Pointer: drag to pan, hover for grab cursor.
    fn handle_pointer_tool(&mut self, ctx: &egui::Context, response: &egui::Response) {
        if response.dragged() {
            self.view.pan_offset += response.drag_delta();
            ctx.set_cursor_icon(egui::CursorIcon::Grabbing);
        } else if response.hovered() {
            ctx.set_cursor_icon(egui::CursorIcon::Grab);
        }
    }

    /// Waypoint Edit: press near a waypoint to grab it, drag to move it,
    /// release to commit. Per-frame mutation means the next render naturally
    /// recomputes segment stats and totals from the new path. The 20-pixel
    /// pick threshold lets the user click on the waypoint dot itself rather
    /// than scrubbing the whole path. `dragging_waypoint` is set on
    /// `drag_started` so a stray click without a nearby waypoint is a no-op.
    fn handle_waypoint_edit_tool(
        &mut self,
        ui: &egui::Ui,
        ctx: &egui::Context,
        view: &ViewTransform,
        response: &egui::Response,
    ) {
        const PICK_RADIUS_PX: f32 = 20.0;
        if response.drag_started()
            && let Some(click_pos) = response.interact_pointer_pos()
        {
            self.editor.dragging_waypoint =
                self.find_closest_waypoint(click_pos, view, PICK_RADIUS_PX);
        }
        if response.dragged() {
            if let Some(idx) = self.editor.dragging_waypoint {
                self.mutate_gbest_waypoint(idx, response.drag_delta(), view);
                ctx.set_cursor_icon(egui::CursorIcon::Grabbing);
            }
        } else if response.hovered()
            && let Some(hover_pos) = ui.input(|i| i.pointer.hover_pos())
            && self
                .find_closest_waypoint(hover_pos, view, PICK_RADIUS_PX)
                .is_some()
        {
            ctx.set_cursor_icon(egui::CursorIcon::Grab);
        }
        if response.drag_stopped() {
            let was_dragging = self.editor.dragging_waypoint.is_some();
            self.editor.dragging_waypoint = None;
            if was_dragging {
                // Reoptimize per-waypoint arrival times for the new xy.
                // Without this the segment-time array stays at whatever
                // the search picked for the old geometry, so total time
                // wouldn't move when waypoints are dragged.
                self.start_time_reopt(ctx);
            }
        }
    }

    /// Speed brush: click to add to wind speed, right-click to subtract;
    /// shift multiplies the step (5 -> 50).
    fn handle_speed_tool(
        &mut self,
        ui: &egui::Ui,
        view: &ViewTransform,
        response: &egui::Response,
    ) {
        self.draw_brush_overlay(ui, view);
        if !(response.clicked() || response.secondary_clicked()) {
            return;
        }
        let current_frame = self.editor.current_frame;
        let brush_radius = self.editor.brush_radius;
        if let (Some(click_pos), Some(wind_map)) =
            (response.interact_pointer_pos(), self.wind_map.as_mut())
            && let Some(frame) = wind_map.frame_mut(current_frame)
        {
            let map_pos = view.screen_to_map(click_pos);
            let increment = if ui.input(|i| i.modifiers.shift) {
                50.0
            } else {
                5.0
            };
            let speed_delta = if response.clicked() {
                increment
            } else {
                -increment
            };
            let indices = frame.query_circle_indices(map_pos.x, map_pos.y, brush_radius);
            for idx in indices {
                if let Some(sample) = frame
                    .rows()
                    .get(idx)
                    .map(|r| (r.sample.speed, r.sample.direction))
                {
                    let new_speed = (sample.0 + speed_delta).max(0.0);
                    frame.set_sample(idx, new_speed, sample.1);
                }
            }
        }
    }

    /// Waypoint Time: click to snap `current_frame` to the arrival time of
    /// the nearest waypoint.
    fn handle_waypoint_time_tool(&mut self, view: &ViewTransform, response: &egui::Response) {
        if response.clicked()
            && let Some(click_pos) = response.interact_pointer_pos()
        {
            self.snap_to_waypoint_time(click_pos, view);
        }
    }

    /// Endpoint: left-click sets the route's start waypoint, right-click
    /// sets the end. Position is captured in wind-map (x, y) coordinates so
    /// it stays fixed relative to the data while the user pans / zooms.
    /// The next search picks these up via [`bywind::MapBounds::resolve_endpoints`].
    fn handle_endpoint_tool(
        &mut self,
        ctx: &egui::Context,
        view: &ViewTransform,
        response: &egui::Response,
    ) {
        if response.hovered() {
            ctx.set_cursor_icon(egui::CursorIcon::Crosshair);
        }
        if let Some(click_pos) = response.interact_pointer_pos() {
            let map_pos = view.screen_to_map(click_pos);
            let xy = (f64::from(map_pos.x), f64::from(map_pos.y));
            if response.clicked() {
                self.editor.start_waypoint = Some(xy);
            } else if response.secondary_clicked() {
                self.editor.end_waypoint = Some(xy);
            }
        }
    }

    /// Route Bounds: left-click-drag to define a rectangular search domain
    /// in map coordinates; right-click clears any committed rectangle.
    /// Anchor is captured in screen space so the live preview rectangle
    /// drawn by `render_central_panel` tracks the cursor across pan/zoom.
    #[expect(
        clippy::float_cmp,
        reason = "single-click drags produce bit-identical lon endpoints; \
                  exact inequality correctly rejects zero-extent bboxes."
    )]
    fn handle_route_bounds_tool(
        &mut self,
        ctx: &egui::Context,
        view: &ViewTransform,
        response: &egui::Response,
    ) {
        if response.hovered() {
            ctx.set_cursor_icon(egui::CursorIcon::Crosshair);
        }
        // `press_origin` is the actual mousedown position. Using
        // `interact_pointer_pos` here would anchor at wherever the cursor
        // had drifted to by the time egui's drag threshold (a few pixels)
        // was crossed, producing a visibly offset top-left corner.
        if response.drag_started()
            && let Some(anchor) = ctx.input(|i| i.pointer.press_origin())
        {
            self.editor.route_bbox_drag_anchor = Some(anchor);
        }
        if response.drag_stopped() {
            if let (Some(anchor), Some(end_screen)) = (
                self.editor.route_bbox_drag_anchor.take(),
                response.interact_pointer_pos(),
            ) {
                let a = view.screen_to_map(anchor);
                let b = view.screen_to_map(end_screen);
                // Pick west/east by which way the user dragged in screen
                // space. `screen_to_map` returns raw (unwrapped) lon, so a
                // drag that visually crossed the antimeridian comes back
                // with one end's lon outside (−180, 180]; canonicalising
                // each end produces `lon_west > lon_east`, which is our
                // convention for "bbox wraps east through 180°".
                let (raw_lon_west, raw_lon_east) = if anchor.x <= end_screen.x {
                    (a.x, b.x)
                } else {
                    (b.x, a.x)
                };
                let lon_west =
                    f64::from(swarmkit_sailing::spherical::wrap_lon_deg_f32(raw_lon_west));
                let lon_east =
                    f64::from(swarmkit_sailing::spherical::wrap_lon_deg_f32(raw_lon_east));
                let lat_min = f64::from(a.y.min(b.y));
                let lat_max = f64::from(a.y.max(b.y));
                // Reject zero-area drags. Wrap-bbox case (lon_west > lon_east)
                // is non-degenerate as long as the lons aren't equal.
                if lat_max > lat_min && lon_west != lon_east {
                    self.editor.route_bbox = Some((lon_west, lon_east, lat_min, lat_max));
                    // Manual drag = the user has taken control of the
                    // bbox. Auto-recompute stays off until they click
                    // "Auto-set bounds" again.
                    self.editor.route_bbox_auto = false;
                }
            } else {
                self.editor.route_bbox_drag_anchor = None;
            }
        }
        // Right-click clears (matches the Endpoint tool's right-click =
        // "remove" sense). Drag preview state is also cleared so a stray
        // half-finished drag doesn't linger. Manual clear also disables
        // auto-recompute so we don't immediately repopulate the bbox.
        if response.secondary_clicked() {
            self.editor.route_bbox = None;
            self.editor.route_bbox_drag_anchor = None;
            self.editor.route_bbox_auto = false;
        }
    }

    /// Direction brush: drag to set wind direction; skip the first frame of
    /// the drag (no previous position to diff against).
    fn handle_direction_tool(
        &mut self,
        ui: &egui::Ui,
        view: &ViewTransform,
        response: &egui::Response,
    ) {
        self.draw_brush_overlay(ui, view);
        if !response.dragged() || response.drag_started() {
            return;
        }
        let delta = response.drag_delta();
        if delta.length_sq() <= 2.0_f32.powi(2) {
            return;
        }
        let current_frame = self.editor.current_frame;
        let brush_radius = self.editor.brush_radius;
        if let (Some(pointer_pos), Some(wind_map)) =
            (ui.input(|i| i.pointer.hover_pos()), self.wind_map.as_mut())
            && let Some(frame) = wind_map.frame_mut(current_frame)
        {
            let map_pos = view.screen_to_map(pointer_pos);
            // Convert screen-space delta to wind direction:
            // 0° = north (up), clockwise. delta.x.atan2(-delta.y) matches
            // the same convention used when drawing the barb shaft.
            // Direction is rotation-invariant under uniform render scale,
            // so the screen-space delta can be used directly.
            let dir_deg = delta.x.atan2(-delta.y).to_degrees().rem_euclid(360.0);
            let indices = frame.query_circle_indices(map_pos.x, map_pos.y, brush_radius);
            for idx in indices {
                if let Some(speed) = frame.rows().get(idx).map(|r| r.sample.speed) {
                    frame.set_sample(idx, speed, dir_deg);
                }
            }
        }
    }
}
