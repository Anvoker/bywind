use crate::app::BywindApp;

impl BywindApp {
    /// Top menu bar: File menu (load / save wind map and solution, quit) plus
    /// the egui theme toggle. The native-only file-dialog branches are gated
    /// with `cfg(not(target_arch = "wasm32"))`.
    pub(crate) fn render_menu_bar(&mut self, ui: &mut egui::Ui) {
        egui::Panel::top("top_panel").show_inside(ui, |ui| {
            egui::MenuBar::new().ui(ui, |ui| {
                let is_web = cfg!(target_arch = "wasm32");
                if !is_web {
                    ui.menu_button("File", |ui| {
                        // Same reason as the Advanced menu below: egui's
                        // automatic menu sizing clips long entries on
                        // some font/zoom combos. Pin a width that holds
                        // the longest label ("Load Scenario (TOML)...").
                        ui.set_min_width(200.0);
                        #[cfg(not(target_arch = "wasm32"))]
                        {
                            if ui.button("Load GRIB2...").clicked() {
                                // Defer to the in-app dialog so stride
                                // and bbox can be reviewed before the
                                // native file picker pops up.
                                ui.close();
                                self.editor.grib2_load_dialog_open = true;
                            }
                            let has_map = self.wind_map.is_some();
                            // AV1 near-lossless `.wcav` is the only
                            // binary wind-map format now (wc1 was
                            // retired in Phase 1.4). Encode happens on
                            // the UI thread; rav1e at our settings is
                            // ~30 s for a 720-frame map.
                            if ui
                                .add_enabled(has_map, egui::Button::new("Save Wind Map..."))
                                .on_disabled_hover_text("Load or generate a wind map first.")
                                .clicked()
                            {
                                ui.close();
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("Bywind AV1", &["wcav"])
                                    .set_file_name("wind_map.wcav")
                                    .save_file()
                                {
                                    self.save_windmap_av1_to_file(&path);
                                }
                            }
                            if ui.button("Load Wind Map...").clicked() {
                                ui.close();
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("Bywind AV1", &["wcav"])
                                    .pick_file()
                                {
                                    self.load_windmap_av1_from_file(&path);
                                }
                            }
                            ui.separator();
                            // Streams GFS frames out of `s3://noaa-gfs-bdp-pds`
                            // into either a `.wcav` (encoded on the fly) or
                            // a `.grib2` (raw concatenation) in a worker
                            // thread, with a cancel button wired to an
                            // `AtomicBool` that the fetch loop checks every
                            // frame.
                            if ui.button("Fetch NOAA GFS...").clicked() {
                                ui.close();
                                self.editor.fetch_dialog.open = true;
                                self.fetch_job.reset_log();
                            }
                            ui.separator();
                            if ui.button("Load Scenario (TOML)...").clicked() {
                                ui.close();
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("TOML", &["toml"])
                                    .pick_file()
                                {
                                    self.load_scenario_from_file(&path);
                                }
                            }
                            if ui.button("Save Scenario (TOML)...").clicked() {
                                ui.close();
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("TOML", &["toml"])
                                    .set_file_name("scenario.toml")
                                    .save_file()
                                {
                                    self.save_scenario_to_file(&path);
                                }
                            }
                            ui.separator();
                            if ui.button("Load Solution...").clicked() {
                                ui.close();
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("JSON", &["json"])
                                    .pick_file()
                                {
                                    self.load_solution_from_file(&path);
                                }
                            }
                            let has_solution = self.outputs.route_evolution.is_some();
                            if ui
                                .add_enabled(has_solution, egui::Button::new("Save Solution..."))
                                .on_disabled_hover_text(
                                    "Run a search first to produce a route to save.",
                                )
                                .clicked()
                            {
                                ui.close();
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("JSON", &["json"])
                                    .set_file_name("solution.json")
                                    .save_file()
                                {
                                    self.save_solution_to_file(&path);
                                }
                            }
                            ui.separator();
                            // Reload the embedded `wind_av1` sample
                            // dataset (the same one decoded at startup
                            // when the `bundled-sample` cargo feature
                            // is on). The button greys out for builds
                            // without the sample so it doesn't lie.
                            let has_bundled = crate::bundled_sample::has_bundled_sample();
                            let label = if self.bundled_sample_job.is_running() {
                                "Reload Bundled Sample (decoding…)"
                            } else {
                                "Reload Bundled Sample"
                            };
                            let disabled_hint = if !has_bundled {
                                "This build was compiled without a bundled sample dataset."
                            } else {
                                "A bundled-sample decode is already running."
                            };
                            if ui
                                .add_enabled(
                                    has_bundled && !self.bundled_sample_job.is_running(),
                                    egui::Button::new(label),
                                )
                                .on_disabled_hover_text(disabled_hint)
                                .clicked()
                            {
                                ui.close();
                                let ctx = ui.ctx().clone();
                                self.start_bundled_sample_decode(&ctx);
                            }
                            ui.separator();
                        }
                        if ui.button("Quit").clicked() {
                            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
                        }
                    });
                    ui.add_space(16.0);
                }

                ui.menu_button("Advanced", |ui| {
                    // Force the dropdown wide enough for the longest entry;
                    // egui sizes menus to fit content but rounds down on
                    // some font/zoom combos and the labels end up clipped.
                    ui.set_min_width(180.0);
                    if ui.button("Advanced Params").clicked() {
                        ui.close();
                        self.editor.advanced_settings_open = true;
                    }
                    if ui.button("Generate Wind Map").clicked() {
                        ui.close();
                        self.editor.generate_window_open = true;
                    }
                });
                ui.add_space(16.0);

                egui::widgets::global_theme_preference_buttons(ui);
            });
        });
    }
}
