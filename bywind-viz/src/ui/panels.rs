use super::{
    BoatConfig, SCALE_MAX, Tool, Topology, WaypointCount, draw_fuel_curve,
    format_duration_breakdown, format_fitness_magnitude, format_fuel, format_land_km,
    format_pso_delta, format_scale_value, int_slider_with_steppers, log_slider_with_steppers,
    min_render_scale, paint_endpoint_highlight, parse_scale_value,
};
use crate::app::BywindApp;

impl BywindApp {
    /// Right-side stats panel. Always rendered (so it acts as the
    /// central panel's right boundary even before any search). With no
    /// stats yet, the Summary / Segments headings show with a single
    /// "(no search yet)" line each; with a completed search, the full
    /// Summary totals + benchmark deltas and scrollable per-segment
    /// breakdown render in the bottom-up layout.
    pub(crate) fn render_stats_panel(&mut self, ui: &mut egui::Ui) {
        let mut toggle_summary = false;
        let stats = self.outputs.segment_stats.clone();
        // Nested layout: the right panel hosts a bottom sub-panel pinned to
        // the Summary totals, and a central sub-panel above it for the
        // scrolling Segments list. Earlier we tried a single `bottom_up`
        // panel with a `with_layout(top_down)` block wrapping a `ScrollArea`
        // for the segments; with long routes (~30 waypoints) the ScrollArea
        // sized to its content instead of the available height, the right
        // panel grew past the window, and the bottom evolution panel /
        // central minimap got pushed off-screen. Splitting into nested
        // panels gives the segments scroll a hard height ceiling from the
        // central sub-panel.
        egui::Panel::right("stats_panel").show_inside(ui, |ui| {
            egui::Panel::bottom("stats_panel_summary").show_inside(ui, |ui| {
                toggle_summary = self.render_stats_summary(ui, stats.as_deref());
            });
            egui::CentralPanel::default().show_inside(ui, |ui| {
                ui.heading("Segments");
                ui.separator();
                egui::ScrollArea::vertical()
                    .id_salt("segment_stats_scroll")
                    .auto_shrink([true, false])
                    .show(ui, |ui| match stats.as_deref() {
                        Some(stats) => self.render_segment_rows(ui, stats),
                        None => {
                            ui.label("(no search yet)");
                        }
                    });
            });
        });
        if toggle_summary {
            self.view.total_time_breakdown = !self.view.total_time_breakdown;
            self.view.total_fuel_tonnes = !self.view.total_fuel_tonnes;
        }
    }

    /// Top-down summary block for the right-panel's bottom sub-panel:
    /// Summary heading, gbest totals, then the benchmark grid (when
    /// available). `stats == None` renders a placeholder. Returns true if
    /// the Summary heading was right-clicked this frame so the caller can
    /// toggle the unit display.
    fn render_stats_summary(
        &self,
        ui: &mut egui::Ui,
        stats: Option<&[bywind::SegmentMetrics]>,
    ) -> bool {
        let toggle_summary = ui.heading("Summary").secondary_clicked();
        ui.separator();
        let Some(stats) = stats else {
            ui.label("(no search yet)");
            return toggle_summary;
        };
        let total_time: f64 = stats.iter().map(|m| m.time).sum();
        let total_fuel: f64 = stats.iter().map(|m| m.fuel).sum();
        let total_land_metres: f64 = stats.iter().map(|m| m.land_metres).sum();
        let best_fitness = self.outputs.best_fitness;
        let bake_duration = self.outputs.bake_duration;
        let search_duration = self.outputs.search_duration;
        let total_time_str = if self.view.total_time_breakdown {
            format_duration_breakdown(total_time)
        } else {
            format!("{total_time:.1}s")
        };
        let total_fuel_str = format_fuel(total_fuel, self.view.total_fuel_tonnes);
        let segment_in_tonnes = self.view.total_fuel_tonnes;
        ui.label(format!("Total time: {total_time_str}"));
        ui.label(format!("Total fuel: {total_fuel_str}"));
        ui.label(format!("Total land: {}", format_land_km(total_land_metres)));
        if let Some(d) = bake_duration {
            ui.label(format!("Bake:       {:.2}s", d.as_secs_f64()));
        }
        if let Some(d) = search_duration {
            ui.label(format!("Search:     {:.2}s", d.as_secs_f64()));
        }
        if let Some(fit) = best_fitness {
            let fit_str = if self.view.total_time_breakdown {
                format_fitness_magnitude(fit)
            } else {
                format!("{fit:.4}")
            };
            ui.label(format!("Fitness:    {fit_str}"));
        }
        // Benchmark group sits below the gbest summary, separated by a
        // divider. The two cells per row are split into a Grid so the
        // "(PSO X% better)" deltas land in their own column and stay
        // vertically aligned even when the bench labels have wildly
        // different lengths (`"Bench time: 1h32m"` vs `"Bench fuel: 12.345 t"`).
        if let Some(b) = self.outputs.benchmark.as_ref() {
            ui.separator();
            egui::Grid::new("bench_pso_grid")
                .num_columns(2)
                .spacing([12.0, 4.0])
                .show(ui, |ui| {
                    let bench_time_str = if self.view.total_time_breakdown {
                        format_duration_breakdown(b.total_time)
                    } else {
                        format!("{:.1}s", b.total_time)
                    };
                    ui.label(format!("Bench time: {bench_time_str}"));
                    ui.label(format!(
                        "({})",
                        format_pso_delta(total_time, b.total_time, false),
                    ));
                    ui.end_row();

                    let bench_fuel_str = format_fuel(b.total_fuel, segment_in_tonnes);
                    ui.label(format!("Bench fuel: {bench_fuel_str}"));
                    ui.label(format!(
                        "({})",
                        format_pso_delta(total_fuel, b.total_fuel, false),
                    ));
                    ui.end_row();

                    ui.label(format!(
                        "Bench land: {}",
                        format_land_km(b.total_land_metres)
                    ));
                    ui.label(format!(
                        "({})",
                        format_pso_delta(total_land_metres, b.total_land_metres, false),
                    ));
                    ui.end_row();

                    let bench_fit_str = if self.view.total_time_breakdown {
                        format_fitness_magnitude(b.fitness)
                    } else {
                        format!("{:.4}", b.fitness)
                    };
                    ui.label(format!("Bench fit:  {bench_fit_str}"));
                    if let Some(fit) = best_fitness {
                        ui.label(format!("({})", format_pso_delta(fit, b.fitness, true)));
                    } else {
                        ui.label("");
                    }
                    ui.end_row();
                });
        }
        toggle_summary
    }

    fn render_segment_rows(&self, ui: &mut egui::Ui, stats: &[bywind::SegmentMetrics]) {
        let segment_in_tonnes = self.view.total_fuel_tonnes;
        for (i, m) in stats.iter().enumerate() {
            ui.label(format!(
                "[{i}] {:.1}s\nmotor={:.2}\nfuel={}\nspeed={:.1} km/h",
                m.time,
                m.mcr_01,
                format_fuel(m.fuel, segment_in_tonnes),
                m.speed_kmh,
            ));
            ui.separator();
        }
    }

    /// Frame index covering the end of the current route's ETA, scaled
    /// by `step_seconds`. Used to extend the time slider's range past
    /// the loaded wind-map data when a search has produced a route
    /// that runs longer than the data covers. Returns 0 if no route
    /// is loaded or the ETA is non-finite.
    fn route_max_frame(&self, step_seconds: f32) -> usize {
        let Some(re) = self.outputs.route_evolution.as_ref() else {
            return 0;
        };
        let Some(gbest) = re.gbest_at(self.outputs.iteration) else {
            return 0;
        };
        // Final waypoint's `t` is the route's total elapsed time. ETA
        // is usually monotonic but we max over the whole slice to
        // tolerate noisy mid-iteration paths.
        let route_eta = gbest.ts.iter().copied().fold(0.0_f64, f64::max) as f32;
        if !route_eta.is_finite() || step_seconds <= 0.0 {
            return 0;
        }
        (route_eta / step_seconds).ceil() as usize
    }

    /// Left-side tools panel: tool picker, generate sub-panel, time slider,
    /// search-parameter grid and Run Search button, view-scale sliders.
    pub(crate) fn render_tools_panel(&mut self, ui: &mut egui::Ui) {
        egui::Panel::left("tools_panel").show_inside(ui, |ui| {
            // Wrap the whole panel body in a ScrollArea so expanding the
            // Boat / Generate / Advanced collapsing headers can't push
            // the panel's content past the window height — without this
            // the egui panel would grow the outer ui to fit, which in
            // turn shoves the central panel's rect downward and the
            // minimap (anchored to `panel_rect.bottom()`) scrolls off
            // screen even though it's on the opposite side.
            egui::ScrollArea::vertical()
                .id_salt("tools_panel_scroll")
                .show(ui, |ui| self.tools_panel_body(ui));
        });
    }

    fn tools_panel_body(&mut self, ui: &mut egui::Ui) {
        self.render_tool_picker(ui);

        ui.separator();
        ui.heading("View");
        let has_map = self.wind_map.is_some();
        if ui
            .add_enabled(has_map, egui::Button::new("Fit to view"))
            .clicked()
        {
            self.view.autoscale_pending = true;
        }
        let min_scale = min_render_scale(self.view.last_panel_height);
        self.view.render_scale = self.view.render_scale.max(min_scale);
        // Snapshot the scale around the Scale-mutating widgets so we can
        // recentre the view on the panel centre after the user changes
        // it. Mouse-wheel zoom already pivots around the cursor; the
        // slider / steppers / typed value should pivot around the view
        // centre so the visible region stays roughly where the user
        // expects rather than drifting toward the projection origin.
        let scale_before = self.view.render_scale;
        // "Scale:" label outside the edit box so the box itself shows only
        // the value (`"1 px : 10 km"`). `DragValue` with `speed(0.0)`
        // keeps click-to-type and disables drag-scrubbing — a linear drag
        // on a log scale feels awful — while the slider+steppers below
        // do the live adjusting.
        ui.horizontal(|ui| {
            ui.label("Scale:");
            ui.add(
                egui::DragValue::new(&mut self.view.render_scale)
                    .range(min_scale..=SCALE_MAX)
                    .speed(0.0)
                    .custom_formatter(|v, _| format_scale_value(v as f32))
                    .custom_parser(parse_scale_value),
            );
        });
        log_slider_with_steppers(ui, &mut self.view.render_scale, min_scale, SCALE_MAX);
        if (self.view.render_scale - scale_before).abs() > f32::EPSILON {
            self.zoom_around_panel_centre(scale_before);
        }

        // Iteration scrubber: was a bottom panel; moved here so it sits
        // next to the other view-state controls (which gbest snapshot the
        // central panel renders is conceptually a view setting). Always
        // shown so the central panel's bottom edge stays stable; disabled
        // with a `0 / 0` caption when no search has run yet. The current
        // index lives in its own `DragValue` so the user can type a
        // specific iteration directly instead of dragging the slider to
        // the right value.
        let iter_count = self
            .outputs
            .route_evolution
            .as_ref()
            .map_or(0, |e| e.iter_count());
        let max_iter = iter_count.saturating_sub(1);
        ui.add_enabled_ui(iter_count > 0, |ui| {
            ui.horizontal(|ui| {
                ui.label("Iteration:");
                ui.add(
                    egui::DragValue::new(&mut self.outputs.iteration)
                        .range(0..=max_iter)
                        .speed(0.0),
                );
                ui.label(format!("/ {max_iter}"));
            });
            int_slider_with_steppers(ui, &mut self.outputs.iteration, max_iter);
        });

        // Time slider: snaps to integer frame indices so edit brushes
        // always land on a real frame rather than an interpolated
        // mid-point. The slider's max extends past the wind-map data
        // when a search has produced a route that runs longer than
        // the loaded data — frames past `data_max_frame` are
        // synthesised on the fly via the crossfade-aware
        // `TimedWindMap::query`, so the user can scrub through what
        // the search "sees" past the data end.
        if let Some(wind_map) = &self.wind_map {
            let data_max_frame = wind_map.frame_count().saturating_sub(1);
            let step_seconds = wind_map.step_seconds();
            let route_max_frame = self.route_max_frame(step_seconds);
            let slider_max = data_max_frame.max(route_max_frame);
            let t_seconds = self.editor.current_frame as f32 * step_seconds;
            let in_wrap = self.editor.current_frame > data_max_frame;
            ui.separator();
            ui.heading("Time");
            let wrap_marker = if in_wrap { "  (wrapped)" } else { "" };
            ui.label(format!(
                "Frame {} / {} ({:.2}h){}",
                self.editor.current_frame,
                slider_max,
                t_seconds / 3600.0,
                wrap_marker,
            ));
            match wind_map.time_range() {
                Some((start, end)) => {
                    // Monospace so the `From:` / `To: ` labels (which
                    // differ by one character) still line up the
                    // timestamps in a single column.
                    ui.monospace(format!("From: {}", start.format("%Y-%m-%d %H:%M UTC")));
                    ui.monospace(format!("To:   {}", end.format("%Y-%m-%d %H:%M UTC")));
                }
                None => {
                    ui.label("(no UTC range)");
                }
            }
            ui.add_enabled_ui(slider_max > 0, |ui| {
                int_slider_with_steppers(ui, &mut self.editor.current_frame, slider_max);
            });
            // Below-slider visual cue showing data vs wrap regions.
            // The slider itself is one continuous bar regardless of
            // value, so this strip is the cheapest way to make the
            // wrap region visible without recreating egui's slider
            // widget. Aligned to the slider's full horizontal extent.
            if slider_max > data_max_frame && data_max_frame > 0 {
                let (rect, _) = ui.allocate_exact_size(
                    egui::Vec2::new(ui.available_width(), 4.0),
                    egui::Sense::hover(),
                );
                let data_color = egui::Color32::from_rgb(80, 140, 200);
                let wrap_color = egui::Color32::from_rgb(220, 150, 60);
                let split_x =
                    rect.left() + rect.width() * (data_max_frame as f32 / slider_max as f32);
                ui.painter().rect_filled(
                    egui::Rect::from_min_max(rect.min, egui::Pos2::new(split_x, rect.bottom())),
                    1.0,
                    data_color,
                );
                ui.painter().rect_filled(
                    egui::Rect::from_min_max(egui::Pos2::new(split_x, rect.top()), rect.max),
                    1.0,
                    wrap_color,
                );
            }
        }

        ui.separator();
        self.render_search_section(ui);
    }

    /// Tool selector at the top of the tools panel, plus any per-tool
    /// secondary controls (currently only the Endpoint tool has them).
    fn render_tool_picker(&mut self, ui: &mut egui::Ui) {
        ui.heading("Tools");
        ui.separator();
        ui.selectable_value(&mut self.editor.selected_tool, Tool::Pointer, "Pointer");
        // Picking Set Endpoints or Set Route Bounds with no values set
        // leaves the user staring at an empty map even though the search
        // would run with implicit defaults (the wind-map bbox corners /
        // bbox). Pre-populate from those defaults the moment the tool is
        // selected so the UI shows what the search would do, with
        // markers/rectangle ready to drag.
        let endpoint_resp = ui.selectable_value(
            &mut self.editor.selected_tool,
            Tool::Endpoint,
            "Set Endpoints",
        );
        if endpoint_resp.changed() {
            self.populate_endpoint_defaults_if_unset();
            // The user is acting on the "missing endpoints" prompt;
            // the animated highlight has served its purpose.
            self.editor.highlight_endpoint_tool = false;
        }
        if self.editor.highlight_endpoint_tool {
            paint_endpoint_highlight(ui, endpoint_resp.rect);
            ui.ctx().request_repaint();
        }
        if ui
            .selectable_value(
                &mut self.editor.selected_tool,
                Tool::RouteBounds,
                "Set Route Bounds",
            )
            .changed()
        {
            self.populate_route_bbox_default_if_unset();
        }
        ui.selectable_value(
            &mut self.editor.selected_tool,
            Tool::Speed,
            "Edit Wind Speed",
        );
        ui.selectable_value(
            &mut self.editor.selected_tool,
            Tool::Direction,
            "Edit Wind Direction",
        );
        ui.selectable_value(
            &mut self.editor.selected_tool,
            Tool::WaypointEdit,
            "Edit Waypoint Position",
        );
        ui.selectable_value(
            &mut self.editor.selected_tool,
            Tool::WaypointTime,
            "Set Time From Waypoint",
        );
        if self.editor.selected_tool == Tool::Endpoint {
            ui.label("Left-click: start, right-click: end");
            let has_endpoints =
                self.editor.start_waypoint.is_some() || self.editor.end_waypoint.is_some();
            // Swap is enabled when at least one endpoint exists — swapping
            // a `(Some, None)` pair into `(None, Some)` is still useful as
            // it relabels which corner the user already placed.
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(has_endpoints, egui::Button::new("Clear"))
                    .clicked()
                {
                    self.editor.start_waypoint = None;
                    self.editor.end_waypoint = None;
                }
                if ui
                    .add_enabled(has_endpoints, egui::Button::new("Swap"))
                    .clicked()
                {
                    std::mem::swap(
                        &mut self.editor.start_waypoint,
                        &mut self.editor.end_waypoint,
                    );
                }
            });
        }
        if self.editor.selected_tool == Tool::RouteBounds {
            ui.label("Drag to define, right-click to clear");
            let has_bbox = self.editor.route_bbox.is_some();
            if ui
                .add_enabled(has_bbox, egui::Button::new("Clear bounds"))
                .clicked()
            {
                self.editor.route_bbox = None;
                self.editor.route_bbox_auto = false;
            }
            // Auto-set is only meaningful with both endpoints placed and a
            // wind map loaded (otherwise we have nothing to derive bounds
            // around or to clamp against).
            let can_auto = self.wind_map.is_some()
                && self.editor.start_waypoint.is_some()
                && self.editor.end_waypoint.is_some();
            let auto_label = if self.editor.route_bbox_auto {
                "Auto-set bounds (active)"
            } else {
                "Auto-set bounds"
            };
            if ui
                .add_enabled(can_auto, egui::Button::new(auto_label))
                .on_hover_text(
                    "Derive a sensible search rectangle from the placed endpoints \
                     using the A* sea-path finder, plus a 20% / 3° margin so the \
                     PSO can find faster wind-aware detours. Recomputes when \
                     endpoints move, until you manually drag or clear the bbox.",
                )
                .clicked()
            {
                self.auto_set_route_bbox();
            }
        }
    }

    /// Search-section block of the tools panel: waypoint count, the
    /// fitness-weights / PSO-parameter grid, and the Run Search button.
    /// Extracted from `render_tools_panel` purely to keep that function
    /// under clippy's `too_many_lines` cap.
    fn render_search_section(&mut self, ui: &mut egui::Ui) {
        ui.heading("Search");

        let is_searching = self.search_job.is_running();
        let can_run = self.wind_map.is_some();
        // Run Search is the section's primary action — full-width and
        // saturated green so it pops out of the panel's grey-and-text
        // wash. While a search is running it morphs into a red Cancel
        // Search button; clicking detaches the worker's receiver via
        // `AsyncJob::cancel` so the UI is responsive immediately (the
        // background thread runs to completion, its send becomes a
        // no-op). Keep this (and "Show all particles") above the
        // collapsing headers so the routine workflow stays one click
        // away even when every collapsible is closed.
        let (label, fill, enabled) = if is_searching {
            // Whole-second elapsed suffix so the button reads
            // "Cancel Search (27s)" while the worker is busy. Falls
            // back to plain "Cancel Search" if the start instant is
            // somehow missing (shouldn't happen — set together with
            // `search_job.set_running`).
            let elapsed_label = self
                .search_started_at
                .map(|t| format!("Cancel Search ({}s)", t.elapsed().as_secs()))
                .unwrap_or_else(|| "Cancel Search".to_owned());
            (elapsed_label, egui::Color32::from_rgb(180, 60, 60), true)
        } else {
            (
                "Run Search".to_owned(),
                egui::Color32::from_rgb(70, 150, 70),
                can_run,
            )
        };
        let button_resp = ui
            .add_enabled_ui(enabled, |ui| {
                ui.add_sized(
                    [ui.available_width(), 28.0],
                    egui::Button::new(egui::RichText::new(label).size(15.0).strong()).fill(fill),
                )
            })
            .inner;
        if button_resp.clicked() {
            if is_searching {
                self.search_job.cancel();
                self.search_started_at = None;
            } else {
                self.run_search(ui.ctx());
            }
        }
        if is_searching {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("Searching…");
            });
        }
        ui.checkbox(&mut self.view.show_all_particles, "Show all particles");

        egui::CollapsingHeader::new("Parameters")
            .default_open(false)
            .show(ui, |ui| self.search_parameters(ui));
        egui::CollapsingHeader::new("Boat")
            .default_open(false)
            .show(ui, |ui| self.boat_panel(ui));
    }

    /// PSO parameter grid: waypoint count, fitness weights, swarm sizes,
    /// iteration counts, inertia / cognitive / social coefficients,
    /// path-kick knobs, and the outer-PSO topology. Pulled out so the
    /// Run Search call site stays compact and the parameters can be
    /// folded behind a collapsing header.
    fn search_parameters(&mut self, ui: &mut egui::Ui) {
        egui::ComboBox::from_label("Waypoints")
            .selected_text(format!("{}", self.search.waypoint_count.as_usize()))
            .show_ui(ui, |ui| {
                for &wc in &WaypointCount::ALL {
                    ui.selectable_value(
                        &mut self.search.waypoint_count,
                        wc,
                        format!("{}", wc.as_usize()),
                    );
                }
            });
        egui::Grid::new("fitness_weights_grid")
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("Time weight");
                ui.add(egui::DragValue::new(&mut self.search.time_weight).range(0.0..=f64::MAX).speed(0.1));
                ui.end_row();
                ui.label("Fuel weight");
                ui.add(egui::DragValue::new(&mut self.search.fuel_weight).range(0.0..=f64::MAX).speed(0.1));
                ui.end_row();
                ui.label("Land weight").on_hover_text(
                    "Penalty per metre of route segment over land. 0 disables landmass avoidance. \
                     Units match time/fuel weight: pick a value where land-metres dominate the cost \
                     ratio you want to enforce.",
                );
                ui.add(egui::DragValue::new(&mut self.search.land_weight).range(0.0..=f64::MAX).speed(0.01));
                ui.end_row();
                ui.label("Particles (space)");
                ui.add(egui::DragValue::new(&mut self.search.particles_space).range(1..=usize::MAX));
                ui.end_row();
                ui.label("Particles (time)");
                ui.add(egui::DragValue::new(&mut self.search.particles_time).range(1..=usize::MAX));
                ui.end_row();
                ui.label("Iterations (space)");
                ui.add(egui::DragValue::new(&mut self.search.iter_space).range(1..=usize::MAX));
                ui.end_row();
                ui.label("Iterations (time)");
                ui.add(egui::DragValue::new(&mut self.search.iter_time).range(1..=usize::MAX));
                ui.end_row();
                ui.label("Inertia");
                ui.add(egui::DragValue::new(&mut self.search.inertia).range(0.0..=f64::MAX).speed(0.01));
                ui.end_row();
                ui.label("Cognitive coeff");
                ui.add(egui::DragValue::new(&mut self.search.cognitive_coeff).range(0.0..=f64::MAX).speed(0.01));
                ui.end_row();
                ui.label("Social coeff");
                ui.add(egui::DragValue::new(&mut self.search.social_coeff).range(0.0..=f64::MAX).speed(0.01));
                ui.end_row();
                ui.label("Path-kick prob").on_hover_text(
                    "Per-particle, per-iteration chance of applying a coherent path-shape \
                     kick (a sine-profile perpendicular offset that lets the search jump into \
                     tacking topologies). 0 disables.",
                );
                ui.add(egui::DragValue::new(&mut self.search.path_kick_probability).range(0.0..=1.0).speed(0.01));
                ui.end_row();
                ui.label("Path-kick γ₀").on_hover_text(
                    "Initial kick magnitude as a fraction of the route's straight-line \
                     length. Cosine-decays toward γ_min across iterations.",
                );
                ui.add(
                    egui::DragValue::new(&mut self.search.path_kick_gamma_0_fraction)
                        .range(0.0..=1.0)
                        .speed(0.005),
                );
                ui.end_row();
                ui.label("Path-kick γ_min").on_hover_text(
                    "Floor kick magnitude as a fraction of route length, reached at the \
                     last iteration. Must be ≤ γ₀.",
                );
                ui.add(
                    egui::DragValue::new(&mut self.search.path_kick_gamma_min_fraction)
                        .range(0.0..=1.0)
                        .speed(0.001),
                );
                ui.end_row();
            });
        ui.horizontal(|ui| {
            ui.label("Topology").on_hover_text(
                "Outer-PSO swarm topology. `gbest` is the stock single-attractor PSO; \
                 the others trade convergence speed for corridor diversity.",
            );
            egui::ComboBox::from_id_salt("search_topology_combo")
                .selected_text(self.search.topology.as_str())
                .show_ui(ui, |ui| {
                    for &t in Topology::ALL {
                        ui.selectable_value(&mut self.search.topology, t, t.as_str());
                    }
                });
        });
        // Restores only the fields the Parameters section exposes; the
        // Advanced Params window has its own Reset for the precision /
        // determinism knobs so toggling one doesn't surprise the user by
        // wiping the other.
        if ui.button("Reset to defaults").clicked() {
            let defaults = bywind::SearchConfig::default();
            self.search.waypoint_count = defaults.waypoint_count;
            self.search.time_weight = defaults.time_weight;
            self.search.fuel_weight = defaults.fuel_weight;
            self.search.land_weight = defaults.land_weight;
            self.search.particles_space = defaults.particles_space;
            self.search.particles_time = defaults.particles_time;
            self.search.iter_space = defaults.iter_space;
            self.search.iter_time = defaults.iter_time;
            self.search.inertia = defaults.inertia;
            self.search.cognitive_coeff = defaults.cognitive_coeff;
            self.search.social_coeff = defaults.social_coeff;
            self.search.path_kick_probability = defaults.path_kick_probability;
            self.search.path_kick_gamma_0_fraction = defaults.path_kick_gamma_0_fraction;
            self.search.path_kick_gamma_min_fraction = defaults.path_kick_gamma_min_fraction;
            self.search.topology = defaults.topology;
        }
    }

    /// Precision / determinism knobs for `SearchConfig`. Opened from
    /// `Advanced → Advanced Params`; changes take effect on the next
    /// Run Search. Tooltips on each control explain the tradeoff.
    pub(crate) fn render_advanced_settings_window(&mut self, ctx: &egui::Context) {
        if !self.editor.advanced_settings_open {
            return;
        }
        let mut open = self.editor.advanced_settings_open;
        egui::Window::new("Advanced Params")
            .open(&mut open)
            .resizable(false)
            .collapsible(true)
            .show(ctx, |ui| {
                ui.label(
                    "Precision and determinism knobs that aren't part of the routine \
                     Search panel. Lower step / cell sizes give better fitness fidelity \
                     at higher per-search cost. Changes take effect on the next Run Search.",
                );
                ui.separator();

                egui::Grid::new("advanced_settings_grid")
                    .num_columns(2)
                    .spacing([8.0, 4.0])
                    .show(ui, |ui| {
                        ui.label("Step distance fraction").on_hover_text(
                            "Substep size for wind / landmass integration along each \
                             segment, as a fraction of the route bbox diagonal. The single \
                             biggest fitness-accuracy knob: 0.005 doubles per-segment \
                             precision at ~2× per-fitness cost. Default 0.01.",
                        );
                        ui.add(
                            egui::DragValue::new(&mut self.search.step_distance_fraction)
                                .range(1e-6..=1.0)
                                .speed(0.001),
                        );
                        ui.end_row();

                        ui.label("Bake step (°)").on_hover_text(
                            "Cell size of the search-side baked wind grid, in degrees \
                             of lon / lat. Quadratic memory in this value; the bake-bounds \
                             builder grows it past the requested value if needed to stay \
                             under the per-axis cell cap. Default 0.25° (typical GFS \
                             resolution).",
                        );
                        ui.add(
                            egui::DragValue::new(&mut self.search.bake_step_deg)
                                .range(1e-3..=10.0)
                                .speed(0.01),
                        );
                        ui.end_row();

                        ui.label("SDF resolution (°)").on_hover_text(
                            "Landmass SDF cell size in degrees. Finer resolutions \
                             improve coastline-following baselines and the land-metres \
                             penalty integral; each distinct resolution pays a one-shot \
                             rasterise + SDF build (~sub-second) and is cached for the \
                             rest of the session. Default 0.5°.",
                        );
                        ui.add(
                            egui::DragValue::new(&mut self.search.sdf_resolution_deg)
                                .range(0.05..=5.0)
                                .speed(0.05),
                        );
                        ui.end_row();

                        ui.label("Range K").on_hover_text(
                            "Departure-time samples per segment in the inner time-PSO \
                             lookup table. Higher values reduce time-axis interpolation \
                             error at the cost of a bigger table build per outer \
                             particle. Must be ≥ 2. Default 8.",
                        );
                        ui.add(egui::DragValue::new(&mut self.search.range_k).range(2..=64));
                        ui.end_row();

                        ui.label("K_MCR").on_hover_text(
                            "Throttle (`mcr_01`) samples per departure-time bucket in \
                             the inner time-PSO lookup table. Lower values trade fitness \
                             fidelity for wallclock (k_mcr=4 → ~11% wallclock saving, \
                             ≤2.1% fitness regression on profiling). Must be ≥ 2. \
                             Default 8.",
                        );
                        ui.add(egui::DragValue::new(&mut self.search.k_mcr).range(2..=64));
                        ui.end_row();

                        ui.label("Deterministic seed").on_hover_text(
                            "When enabled, fixes the RNG seed so identical inputs \
                             reproduce the exact same swarm trajectory. When disabled, \
                             each run draws a fresh seed and shows it below after the \
                             search starts. Toggling the box off parks the current \
                             value so re-checking restores it; check the box after a \
                             fresh-entropy run to recover that run's seed.",
                        );
                        ui.horizontal(|ui| {
                            let mut use_seed = self.search.seed.is_some();
                            if ui.checkbox(&mut use_seed, "").changed() {
                                if use_seed {
                                    // Re-check prefers the explicitly parked
                                    // value, falling back to the seed used by
                                    // the most recent run so the user can
                                    // reproduce a fresh-entropy result with
                                    // one click. Final fallback: 0.
                                    let restored = self
                                        .editor
                                        .parked_seed
                                        .or(self.outputs.last_search_seed)
                                        .unwrap_or(0);
                                    self.search.seed = Some(restored);
                                } else {
                                    self.editor.parked_seed = self.search.seed;
                                    self.search.seed = None;
                                }
                            }
                            if let Some(seed) = self.search.seed.as_mut() {
                                ui.add(egui::DragValue::new(seed).speed(1.0));
                            } else if let Some(last) = self.outputs.last_search_seed {
                                // Read-only display of the seed used by the
                                // most recent fresh-entropy run.
                                let mut shown = last;
                                ui.add_enabled(false, egui::DragValue::new(&mut shown).speed(0.0));
                            } else {
                                ui.weak("(fresh entropy)");
                            }
                        });
                        ui.end_row();
                    });

                ui.separator();
                if ui.button("Reset to defaults").clicked() {
                    let defaults = bywind::SearchConfig::default();
                    self.search.step_distance_fraction = defaults.step_distance_fraction;
                    self.search.bake_step_deg = defaults.bake_step_deg;
                    self.search.sdf_resolution_deg = defaults.sdf_resolution_deg;
                    self.search.range_k = defaults.range_k;
                    self.search.k_mcr = defaults.k_mcr;
                    self.search.seed = defaults.seed;
                }
            });
        // Mirror `open` back so the title-bar X toggles the flag.
        self.editor.advanced_settings_open = open;
    }

    /// Renders the Boat section in the tools panel: parameter inputs for the
    /// engine, polar curve, and fuel-rate model. The next search and any
    /// reload of a saved solution will pick these up via
    /// [`bywind::BoatConfig::to_boat`].
    fn boat_panel(&mut self, ui: &mut egui::Ui) {
        egui::Grid::new("boat_grid")
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("MCR (kW)");
                ui.add(
                    egui::DragValue::new(&mut self.boat.mcr_kw)
                        .range(0.0..=f64::MAX)
                        .speed(1.0),
                );
                ui.end_row();
                ui.label("Drag k");
                ui.add(
                    egui::DragValue::new(&mut self.boat.k)
                        .range(0.0..=f64::MAX)
                        .speed(10.0),
                );
                ui.end_row();
                ui.label("Polar c");
                ui.add(
                    egui::DragValue::new(&mut self.boat.polar_c)
                        .range(0.0..=f64::MAX)
                        .speed(0.01),
                );
                ui.end_row();
                ui.label("Polar sin power");
                ui.add(
                    egui::DragValue::new(&mut self.boat.polar_sin_power)
                        .range(0.0..=f64::MAX)
                        .speed(0.05),
                );
                ui.end_row();
                ui.label("Fuel a");
                ui.add(egui::DragValue::new(&mut self.boat.fuel_a).speed(0.001));
                ui.end_row();
                ui.label("Fuel b");
                ui.add(egui::DragValue::new(&mut self.boat.fuel_b).speed(0.001));
                ui.end_row();
                ui.label("Fuel c");
                ui.add(egui::DragValue::new(&mut self.boat.fuel_c).speed(0.001));
                ui.end_row();
            });
        draw_fuel_curve(ui, self.boat.fuel_a, self.boat.fuel_b, self.boat.fuel_c);
        if ui.button("Reset to defaults").clicked() {
            self.boat = BoatConfig::default();
        }
    }
}
