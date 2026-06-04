use super::TimedWindMap;
#[cfg(not(target_arch = "wasm32"))]
use super::sync_out_path_extension;
use crate::app::BywindApp;

impl BywindApp {
    /// In-app modal-ish dialog with the GRIB2 load-time options
    /// (stride, optional lat/lon bbox), shown when the user picks
    /// "Load GRIB2..." in the File menu. Continue opens the native
    /// file picker; Cancel (or the X) just closes. Settings live on
    /// `EditorState` so they persist across opens of the dialog and
    /// across sessions.
    pub(crate) fn render_grib2_load_dialog(&mut self, ui: &egui::Ui) {
        if !self.editor.grib2_load_dialog_open {
            return;
        }
        let mut open = true;
        let mut continue_clicked = false;
        let mut cancel_clicked = false;
        egui::Window::new("Load GRIB2")
            .open(&mut open)
            .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
            .resizable(false)
            .collapsible(false)
            .show(ui.ctx(), |ui| {
                self.render_grib2_load_options(ui);
                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("Continue").clicked() {
                        continue_clicked = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancel_clicked = true;
                    }
                });
            });

        // The Window's X button toggles `open` to false; treat that the
        // same as Cancel. Continue also dismisses the dialog before
        // opening the native picker so the two never overlap.
        if cancel_clicked || !open {
            self.editor.grib2_load_dialog_open = false;
        }
        if continue_clicked {
            self.editor.grib2_load_dialog_open = false;
            #[cfg(not(target_arch = "wasm32"))]
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("GRIB2", &["grib2", "grb2", "grib"])
                .pick_file()
            {
                self.load_windmap_from_grib2(&path);
            }
        }
    }

    /// GRIB2 load-time options: stride and the optional lat/lon bbox
    /// filter. Rendered inside the load dialog (`render_grib2_load_dialog`)
    /// so the user can adjust them before triggering the file picker.
    fn render_grib2_load_options(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("GRIB2 stride:");
            ui.add(egui::DragValue::new(&mut self.editor.grib2_load_stride).range(1usize..=64));
        });
        ui.checkbox(
            &mut self.editor.grib2_load_bbox_active,
            "Filter to lat/lon bbox",
        );
        ui.add_enabled_ui(self.editor.grib2_load_bbox_active, |ui| {
            egui::Grid::new("grib2_bbox_grid")
                .num_columns(2)
                .spacing([6.0, 2.0])
                .show(ui, |ui| {
                    let lat_speed = 0.5_f32;
                    let lon_speed = 0.5_f32;
                    ui.label("Lat min");
                    ui.add(
                        egui::DragValue::new(&mut self.editor.grib2_load_bbox_lat_min)
                            .range(-90.0_f32..=90.0)
                            .speed(lat_speed),
                    );
                    ui.end_row();
                    ui.label("Lat max");
                    ui.add(
                        egui::DragValue::new(&mut self.editor.grib2_load_bbox_lat_max)
                            .range(-90.0_f32..=90.0)
                            .speed(lat_speed),
                    );
                    ui.end_row();
                    ui.label("Lon min");
                    ui.add(
                        egui::DragValue::new(&mut self.editor.grib2_load_bbox_lon_min)
                            .range(-180.0_f32..=180.0)
                            .speed(lon_speed),
                    );
                    ui.end_row();
                    ui.label("Lon max");
                    ui.add(
                        egui::DragValue::new(&mut self.editor.grib2_load_bbox_lon_max)
                            .range(-180.0_f32..=180.0)
                            .speed(lon_speed),
                    );
                    ui.end_row();
                });
        });
    }

    /// `File → Fetch NOAA GFS…` modal. Three text inputs (start, end,
    /// output path), an interval dropdown, a Start / Cancel / Close
    /// row, and a scrolling log area underneath. The worker thread
    /// emits one `FetchEvent::Progress` per frame the spec expands to,
    /// which `app::ui()` drains into `fetch_job.log` so we can render
    /// it here verbatim.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn render_fetch_dialog(&mut self, ui: &egui::Ui) {
        if !self.editor.fetch_dialog.open {
            return;
        }
        self.populate_fetch_dialog_defaults_once();

        let mut open = true;
        let mut start_clicked = false;
        let mut cancel_clicked = false;
        let mut browse_clicked = false;
        egui::Window::new("Fetch NOAA GFS")
            .open(&mut open)
            .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
            .resizable(true)
            .default_width(520.0)
            .collapsible(false)
            .show(ui.ctx(), |ui| {
                self.render_fetch_dialog_inputs(ui, &mut browse_clicked);
                ui.separator();
                ui.horizontal(|ui| {
                    let running = self.fetch_job.is_running();
                    if ui
                        .add_enabled(!running, egui::Button::new("Start"))
                        .on_disabled_hover_text("A fetch is already in progress.")
                        .clicked()
                    {
                        start_clicked = true;
                    }
                    if ui
                        .add_enabled(running, egui::Button::new("Cancel"))
                        .on_disabled_hover_text("No fetch is currently running.")
                        .clicked()
                    {
                        cancel_clicked = true;
                    }
                    match self.fetch_job.phase {
                        crate::fetch::FetchPhase::Idle => {}
                        crate::fetch::FetchPhase::Fetching => {
                            ui.label("fetching…");
                        }
                        crate::fetch::FetchPhase::Encoding => {
                            ui.label("encoding…");
                        }
                        crate::fetch::FetchPhase::Cancelling => {
                            ui.label("cancelling…");
                        }
                    }
                });
                ui.separator();
                self.render_fetch_dialog_log(ui);
            });

        if !open {
            // The Window's X button closes the dialog regardless of the
            // worker state; the worker keeps running in the background.
            // The cancel button is the explicit way to stop it.
            self.editor.fetch_dialog.open = false;
        }
        if browse_clicked {
            // Use the format combo's selection as the primary filter
            // so the native picker defaults to writing the extension
            // the user expects. The other format stays available as a
            // secondary filter for users who change their mind in the
            // native dialog.
            let (primary_label, primary_exts, secondary_label, secondary_exts): (
                &str,
                &[&str],
                &str,
                &[&str],
            ) = match self.editor.fetch_dialog.out_format {
                crate::config::FetchOutputFormat::Wcav => {
                    ("Bywind AV1", &["wcav"], "GRIB2", &["grib2", "grb2", "grib"])
                }
                crate::config::FetchOutputFormat::Grib2 => {
                    ("GRIB2", &["grib2", "grb2", "grib"], "Bywind AV1", &["wcav"])
                }
            };
            let default_name = format!(
                "fetched.{}",
                self.editor.fetch_dialog.out_format.extension()
            );
            if let Some(path) = rfd::FileDialog::new()
                .add_filter(primary_label, primary_exts)
                .add_filter(secondary_label, secondary_exts)
                .set_file_name(default_name)
                .save_file()
            {
                self.editor.fetch_dialog.out_path = path.to_string_lossy().into_owned();
                // Mirror the picked file's extension back onto the
                // format combo so the next Browse defaults match.
                if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
                    self.editor.fetch_dialog.out_format = match ext.to_ascii_lowercase().as_str() {
                        "wcav" => crate::config::FetchOutputFormat::Wcav,
                        "grib2" | "grb2" | "grib" => crate::config::FetchOutputFormat::Grib2,
                        _ => self.editor.fetch_dialog.out_format,
                    };
                }
            }
        }
        if cancel_clicked {
            self.fetch_job.request_cancel();
        }
        if start_clicked {
            self.start_fetch_worker(ui.ctx());
        }
    }

    /// Form fields for the fetch dialog: start, end, interval, output
    /// path. Pulled out of `render_fetch_dialog` so the dialog body
    /// stays a thin orchestrator over input / buttons / log.
    #[cfg(not(target_arch = "wasm32"))]
    fn render_fetch_dialog_inputs(&mut self, ui: &mut egui::Ui, browse_clicked: &mut bool) {
        let running = self.fetch_job.is_running();
        ui.add_enabled_ui(!running, |ui| {
            egui::Grid::new("fetch_dialog_inputs")
                .num_columns(2)
                .spacing([8.0, 4.0])
                .show(ui, |ui| {
                    ui.label("Start (UTC):")
                        .on_hover_text("YYYYMMDD or YYYYMMDDHH; must be 00, 06, 12, or 18z");
                    ui.text_edit_singleline(&mut self.editor.fetch_dialog.start_text);
                    ui.end_row();

                    ui.label("End (UTC):")
                        .on_hover_text("YYYYMMDD or YYYYMMDDHH (exclusive)");
                    ui.text_edit_singleline(&mut self.editor.fetch_dialog.end_text);
                    ui.end_row();

                    ui.label("Interval (h):");
                    egui::ComboBox::from_id_salt("fetch_interval")
                        .selected_text(format!("{} h", self.editor.fetch_dialog.interval_h))
                        .show_ui(ui, |ui| {
                            for n in [1u32, 2, 3, 6] {
                                ui.selectable_value(
                                    &mut self.editor.fetch_dialog.interval_h,
                                    n,
                                    format!("{n} h"),
                                );
                            }
                        });
                    ui.end_row();

                    ui.label("Format:");
                    let prev_fmt = self.editor.fetch_dialog.out_format;
                    egui::ComboBox::from_id_salt("fetch_output_format")
                        .selected_text(self.editor.fetch_dialog.out_format.label())
                        .show_ui(ui, |ui| {
                            for fmt in [
                                crate::config::FetchOutputFormat::Wcav,
                                crate::config::FetchOutputFormat::Grib2,
                            ] {
                                ui.selectable_value(
                                    &mut self.editor.fetch_dialog.out_format,
                                    fmt,
                                    fmt.label(),
                                );
                            }
                        });
                    if self.editor.fetch_dialog.out_format != prev_fmt {
                        // Rewrite the output path's extension so the
                        // path field tracks the combo. The user can
                        // still hand-edit it afterwards.
                        sync_out_path_extension(
                            &mut self.editor.fetch_dialog.out_path,
                            self.editor.fetch_dialog.out_format.extension(),
                        );
                    }
                    ui.end_row();

                    ui.label("Output:");
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::TextEdit::singleline(&mut self.editor.fetch_dialog.out_path)
                                .desired_width(320.0),
                        );
                        if ui.button("Browse…").clicked() {
                            *browse_clicked = true;
                        }
                    });
                    ui.end_row();
                });
        });
    }

    /// Scrolling log pane at the bottom of the fetch dialog. Auto-sticks
    /// to the bottom while events are streaming in so the most recent
    /// progress line is always visible.
    #[cfg(not(target_arch = "wasm32"))]
    fn render_fetch_dialog_log(&self, ui: &mut egui::Ui) {
        let running = self.fetch_job.is_running();
        egui::ScrollArea::vertical()
            .max_height(220.0)
            .auto_shrink([false, false])
            .stick_to_bottom(running)
            .show(ui, |ui| {
                if self.fetch_job.log.is_empty() {
                    ui.weak("(no activity yet)");
                } else {
                    for line in &self.fetch_job.log {
                        ui.label(line);
                    }
                }
            });
    }

    /// Populate the fetch dialog's text fields with the
    /// "last 240 h, ending at the most recent 6 h cycle" preset on the
    /// first open per session. Subsequent opens leave the user's edits
    /// in place.
    #[cfg(not(target_arch = "wasm32"))]
    fn populate_fetch_dialog_defaults_once(&mut self) {
        if self.editor.fetch_dialog.populated {
            return;
        }
        let end = crate::fetch::snap_to_cycle(chrono::Utc::now());
        let start = end - chrono::TimeDelta::hours(240);
        if self.editor.fetch_dialog.start_text.is_empty() {
            self.editor.fetch_dialog.start_text = crate::fetch::format_yyyymmddhh(start);
        }
        if self.editor.fetch_dialog.end_text.is_empty() {
            self.editor.fetch_dialog.end_text = crate::fetch::format_yyyymmddhh(end);
        }
        if self.editor.fetch_dialog.interval_h == 0 {
            self.editor.fetch_dialog.interval_h = 1;
        }
        if self.editor.fetch_dialog.out_path.is_empty() {
            self.editor.fetch_dialog.out_path = format!(
                "fetched.{}",
                self.editor.fetch_dialog.out_format.extension()
            );
        }
        self.editor.fetch_dialog.populated = true;
    }

    /// Validate the dialog's inputs and spawn the fetch worker, or log
    /// the validation error into the dialog's log area if anything
    /// rejects.
    #[cfg(not(target_arch = "wasm32"))]
    fn start_fetch_worker(&mut self, ctx: &egui::Context) {
        let dlg = &self.editor.fetch_dialog;
        let start = match crate::fetch::parse_yyyymmddhh(&dlg.start_text) {
            Ok(t) => t,
            Err(e) => {
                self.fetch_job.log.push(format!("start: {e}"));
                return;
            }
        };
        let end = match crate::fetch::parse_yyyymmddhh(&dlg.end_text) {
            Ok(t) => t,
            Err(e) => {
                self.fetch_job.log.push(format!("end: {e}"));
                return;
            }
        };
        if dlg.out_path.trim().is_empty() {
            self.fetch_job.log.push("output path is empty".to_owned());
            return;
        }
        let spec = bywind::fetch::FetchSpec {
            start,
            end,
            interval_h: dlg.interval_h,
        };
        let out_path = std::path::PathBuf::from(dlg.out_path.clone());
        self.fetch_job.reset_log();
        let (rx, cancel) = crate::fetch::spawn_worker(spec, out_path, ctx.clone());
        self.fetch_job.attach(rx, cancel);
    }

    /// Standalone wind-map generator window, opened from
    /// `Advanced → Generate Wind Map`. Hosts the same parameter inputs
    /// (plus Random and Empty buttons) that used to live in the tools
    /// panel's Generate collapsing header; broken out so the tools
    /// panel can focus on the routine edit / search workflow.
    pub(crate) fn render_generate_window(&mut self, ctx: &egui::Context) {
        if !self.editor.generate_window_open {
            return;
        }
        let mut open = self.editor.generate_window_open;
        egui::Window::new("Generate Wind Map")
            .open(&mut open)
            .resizable(false)
            .collapsible(true)
            .show(ctx, |ui| self.generate_panel(ui));
        self.editor.generate_window_open = open;
    }

    /// Parameter inputs plus "Random" and "Empty" buttons that replace
    /// `self.wind_map`. The Random button is disabled when `speed_max
    /// <= speed_min` (rand requires a strictly non-empty range).
    fn generate_panel(&mut self, ui: &mut egui::Ui) {
        egui::Grid::new("generate_grid")
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("Size X");
                ui.add(
                    egui::DragValue::new(&mut self.generate.size_x)
                        .range(1.0..=f32::MAX)
                        .speed(10.0),
                );
                ui.end_row();
                ui.label("Size Y");
                ui.add(
                    egui::DragValue::new(&mut self.generate.size_y)
                        .range(1.0..=f32::MAX)
                        .speed(10.0),
                );
                ui.end_row();
                ui.label("Density");
                ui.add(
                    egui::DragValue::new(&mut self.generate.density)
                        .range(1.0..=f32::MAX)
                        .speed(1.0),
                );
                ui.end_row();
                ui.label("Frames");
                ui.add(egui::DragValue::new(&mut self.generate.frame_count).range(1..=usize::MAX));
                ui.end_row();
                ui.label("Step (s)");
                ui.add(
                    egui::DragValue::new(&mut self.generate.step_seconds)
                        .range(1.0..=f32::MAX)
                        .speed(60.0)
                        .suffix(" s"),
                );
                ui.end_row();
                ui.label("Speed min");
                ui.add(
                    egui::DragValue::new(&mut self.generate.speed_min)
                        .range(0.0..=f32::MAX)
                        .speed(1.0),
                );
                ui.end_row();
                ui.label("Speed max");
                ui.add(
                    egui::DragValue::new(&mut self.generate.speed_max)
                        .range(0.0..=f32::MAX)
                        .speed(1.0),
                );
                ui.end_row();
            });

        ui.horizontal(|ui| {
            let speed_ok = self.generate.speed_max > self.generate.speed_min;
            if ui
                .add_enabled(speed_ok, egui::Button::new("Random"))
                .clicked()
            {
                self.wind_map = Some(TimedWindMap::generate_random(
                    self.generate.size_x,
                    self.generate.size_y,
                    self.generate.density,
                    self.generate.frame_count,
                    self.generate.step_seconds,
                    self.generate.speed_min..self.generate.speed_max,
                ));
                self.editor.current_frame = self
                    .editor
                    .current_frame
                    .min(self.generate.frame_count.saturating_sub(1));
            }
            if ui.button("Empty").clicked() {
                self.wind_map = Some(TimedWindMap::generate(
                    self.generate.size_x,
                    self.generate.size_y,
                    self.generate.density,
                    self.generate.frame_count,
                    self.generate.step_seconds,
                ));
                self.editor.current_frame = self
                    .editor
                    .current_frame
                    .min(self.generate.frame_count.saturating_sub(1));
            }
        });
    }

    /// `File → Fetch GEFS Ensemble…` modal. Mirrors the single-fetch
    /// dialog in layout (text inputs, Start / Cancel row, scrolling
    /// log) but defaults its start / end / interval from the
    /// currently-loaded `wind_map` so the perturbations cover the
    /// same window as the main file.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn render_fetch_ensemble_dialog(&mut self, ui: &egui::Ui) {
        if !self.editor.fetch_ensemble_dialog.open {
            return;
        }
        self.populate_fetch_ensemble_dialog_defaults_once();

        let mut open = true;
        let mut start_clicked = false;
        let mut cancel_clicked = false;
        let mut browse_clicked = false;
        egui::Window::new("Fetch GEFS Ensemble")
            .open(&mut open)
            .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
            .resizable(true)
            .default_width(560.0)
            .collapsible(false)
            .show(ui.ctx(), |ui| {
                self.render_fetch_ensemble_dialog_inputs(ui, &mut browse_clicked);
                ui.separator();
                ui.horizontal(|ui| {
                    let running = self.fetch_ensemble_job.is_running();
                    if ui
                        .add_enabled(!running, egui::Button::new("Start"))
                        .on_disabled_hover_text("An ensemble fetch is already in progress.")
                        .clicked()
                    {
                        start_clicked = true;
                    }
                    if ui
                        .add_enabled(running, egui::Button::new("Cancel"))
                        .on_disabled_hover_text("No ensemble fetch is currently running.")
                        .clicked()
                    {
                        cancel_clicked = true;
                    }
                    match self.fetch_ensemble_job.phase {
                        crate::fetch::FetchPhase::Idle => {}
                        crate::fetch::FetchPhase::Fetching => {
                            ui.label("fetching…");
                        }
                        crate::fetch::FetchPhase::Encoding => {
                            ui.label("encoding…");
                        }
                        crate::fetch::FetchPhase::Cancelling => {
                            ui.label("cancelling…");
                        }
                    }
                });
                ui.separator();
                self.render_fetch_ensemble_dialog_log(ui);
            });

        if !open {
            self.editor.fetch_ensemble_dialog.open = false;
        }
        if browse_clicked {
            if let Some(path) = rfd::FileDialog::new()
                .set_directory(
                    self.editor
                        .fetch_ensemble_dialog
                        .out_dir
                        .as_str()
                        .trim_start_matches(['"', '\'']),
                )
                .pick_folder()
            {
                self.editor.fetch_ensemble_dialog.out_dir = path.to_string_lossy().into_owned();
            }
        }
        if cancel_clicked {
            self.fetch_ensemble_job.request_cancel();
        }
        if start_clicked {
            self.start_fetch_ensemble_worker(ui.ctx());
        }
    }

    /// Form fields for the ensemble fetch dialog: start, end,
    /// interval, members, destination dir, basename (subfolder).
    #[cfg(not(target_arch = "wasm32"))]
    fn render_fetch_ensemble_dialog_inputs(
        &mut self,
        ui: &mut egui::Ui,
        browse_clicked: &mut bool,
    ) {
        let running = self.fetch_ensemble_job.is_running();
        ui.add_enabled_ui(!running, |ui| {
            egui::Grid::new("fetch_ensemble_dialog_inputs")
                .num_columns(2)
                .spacing([8.0, 4.0])
                .show(ui, |ui| {
                    ui.label("Start (UTC):").on_hover_text(
                        "YYYYMMDD or YYYYMMDDHH. Snapped to the most recent 00 / 06 / 12 / 18z \
                         GEFS cycle. Default = the loaded wind map's start.",
                    );
                    ui.text_edit_singleline(&mut self.editor.fetch_ensemble_dialog.start_text);
                    ui.end_row();

                    ui.label("End (UTC):").on_hover_text(
                        "YYYYMMDD or YYYYMMDDHH (exclusive). Default = the loaded wind \
                         map's end.",
                    );
                    ui.text_edit_singleline(&mut self.editor.fetch_ensemble_dialog.end_text);
                    ui.end_row();

                    ui.label("Interval (h):").on_hover_text(
                        "Hours between successive frames. Default = the loaded wind map's \
                         step (snapped to GEFS-valid values). The GEFS `pgrb2sp25` subset \
                         actually publishes at 3-hourly cadence — values below 3 will 404 \
                         on most frames.",
                    );
                    egui::ComboBox::from_id_salt("fetch_ensemble_interval")
                        .selected_text(format!(
                            "{} h",
                            self.editor.fetch_ensemble_dialog.interval_h,
                        ))
                        .show_ui(ui, |ui| {
                            for n in [1u32, 2, 3, 6] {
                                ui.selectable_value(
                                    &mut self.editor.fetch_ensemble_dialog.interval_h,
                                    n,
                                    format!("{n} h"),
                                );
                            }
                        });
                    ui.end_row();

                    ui.label("Members:").on_hover_text(
                        "Number of GEFS members to fetch, stride-sampled from the 31-member \
                         set (1 control + 30 perturbed). Each member is a full wind dataset \
                         over the window — K members = K× wallclock and disk.",
                    );
                    ui.add(
                        egui::DragValue::new(&mut self.editor.fetch_ensemble_dialog.members)
                            .range(1usize..=31)
                            .speed(1.0),
                    );
                    ui.end_row();

                    ui.label("Destination dir:")
                        .on_hover_text("Parent directory for the run's output folder.");
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::TextEdit::singleline(
                                &mut self.editor.fetch_ensemble_dialog.out_dir,
                            )
                            .desired_width(320.0),
                        );
                        if ui.button("Browse…").clicked() {
                            *browse_clicked = true;
                        }
                    });
                    ui.end_row();

                    ui.label("Basename:").on_hover_text(
                        "Subfolder created under the destination directory. Each member's \
                         `.wcav` lands inside as `gec00.wcav` / `gepNN.wcav` so the bywind \
                         loader can read the folder directly. Empty = write into the \
                         destination dir itself.",
                    );
                    ui.text_edit_singleline(&mut self.editor.fetch_ensemble_dialog.basename);
                    ui.end_row();
                });
        });
    }

    /// Scrolling log pane for the ensemble dialog. Same shape as the
    /// single-fetch log; sticks to the bottom while events stream in.
    #[cfg(not(target_arch = "wasm32"))]
    fn render_fetch_ensemble_dialog_log(&self, ui: &mut egui::Ui) {
        let running = self.fetch_ensemble_job.is_running();
        egui::ScrollArea::vertical()
            .max_height(240.0)
            .auto_shrink([false, false])
            .stick_to_bottom(running)
            .show(ui, |ui| {
                if self.fetch_ensemble_job.log.is_empty() {
                    ui.weak("(no activity yet)");
                } else {
                    for line in &self.fetch_ensemble_job.log {
                        ui.label(line);
                    }
                }
            });
    }

    /// On first open this session — and only when a wind map is
    /// loaded — populate the dialog's text fields from
    /// `wind_map.time_range()` and `wind_map.step_seconds()`. The
    /// time range is snapped down to the nearest prior 6 h GEFS
    /// cycle; the interval is snapped to the nearest valid GEFS
    /// cadence in `{1, 2, 3, 6}`. Subsequent opens leave the user's
    /// edits in place.
    #[cfg(not(target_arch = "wasm32"))]
    fn populate_fetch_ensemble_dialog_defaults_once(&mut self) {
        if self.editor.fetch_ensemble_dialog.populated {
            return;
        }
        let Some(wind_map) = self.wind_map.as_ref() else {
            return;
        };
        if let Some((start, end)) = wind_map.time_range() {
            let start_snapped = crate::fetch::snap_to_cycle(start);
            if self.editor.fetch_ensemble_dialog.start_text.is_empty() {
                self.editor.fetch_ensemble_dialog.start_text =
                    crate::fetch::format_yyyymmddhh(start_snapped);
            }
            if self.editor.fetch_ensemble_dialog.end_text.is_empty() {
                self.editor.fetch_ensemble_dialog.end_text = crate::fetch::format_yyyymmddhh(end);
            }
        }
        if self.editor.fetch_ensemble_dialog.interval_h == 0 {
            self.editor.fetch_ensemble_dialog.interval_h =
                snap_to_gefs_interval(wind_map.step_seconds());
        }
        if self.editor.fetch_ensemble_dialog.members == 0 {
            self.editor.fetch_ensemble_dialog.members = 10;
        }
        if self.editor.fetch_ensemble_dialog.out_dir.is_empty() {
            self.editor.fetch_ensemble_dialog.out_dir =
                std::env::temp_dir().to_string_lossy().into_owned();
        }
        if self.editor.fetch_ensemble_dialog.basename.is_empty() {
            // Date-stamp default so consecutive opens don't collide.
            self.editor.fetch_ensemble_dialog.basename =
                chrono::Utc::now().format("gefs-%Y%m%d-%H%M").to_string();
        }
        self.editor.fetch_ensemble_dialog.populated = true;
    }

    /// Validate dialog inputs and spawn the ensemble fetch worker.
    /// Failures land as a single log line in the dialog's log area;
    /// nothing toast-y, since the dialog itself is already open.
    #[cfg(not(target_arch = "wasm32"))]
    fn start_fetch_ensemble_worker(&mut self, ctx: &egui::Context) {
        let dlg = &self.editor.fetch_ensemble_dialog;
        let start = match crate::fetch::parse_yyyymmddhh(&dlg.start_text) {
            Ok(t) => t,
            Err(e) => {
                self.fetch_ensemble_job.log.push(format!("start: {e}"));
                return;
            }
        };
        let end = match crate::fetch::parse_yyyymmddhh(&dlg.end_text) {
            Ok(t) => t,
            Err(e) => {
                self.fetch_ensemble_job.log.push(format!("end: {e}"));
                return;
            }
        };
        if dlg.out_dir.trim().is_empty() {
            self.fetch_ensemble_job
                .log
                .push("destination directory is empty".to_owned());
            return;
        }
        if dlg.members == 0 {
            self.fetch_ensemble_job
                .log
                .push("members must be > 0".to_owned());
            return;
        }
        let spec = bywind::fetch::FetchSpec {
            start,
            end,
            interval_h: dlg.interval_h,
        };
        let members = bywind::fetch_ensemble::stride_sample_members(dlg.members);
        let out_dir = std::path::PathBuf::from(dlg.out_dir.trim());
        let out_dir = if dlg.basename.trim().is_empty() {
            out_dir
        } else {
            out_dir.join(dlg.basename.trim())
        };
        self.fetch_ensemble_job.reset_log();
        let (rx, cancel) = crate::fetch::spawn_ensemble_worker(spec, members, out_dir, ctx.clone());
        self.fetch_ensemble_job.attach(rx, cancel);
    }
}

/// Snap a source `step_seconds` (typically the loaded wind map's
/// frame cadence) onto the nearest sensible *default* GEFS interval
/// in hours.
///
/// Combo-box choices are `{1, 2, 3, 6}` (every value the bucket
/// publishes at), but the *default* floors at 3 because the GEFS
/// `pgrb2sp25` subset only ships frames at `f000, f003, f006, …`.
/// Defaulting to 1 or 2 for a fine-grained main file (the bundled
/// sample is 1 h) would silently produce a fetch where two out of
/// every three timestamps 404. Users who want sparse coverage can
/// still pick 1 or 2 explicitly.
#[cfg(not(target_arch = "wasm32"))]
fn snap_to_gefs_interval(step_seconds: f32) -> u32 {
    const DEFAULT_CHOICES: [u32; 2] = [3, 6];
    let hours = (step_seconds / 3600.0).round().max(1.0) as u32;
    *DEFAULT_CHOICES
        .iter()
        .min_by_key(|v| v.abs_diff(hours))
        .expect("DEFAULT_CHOICES is non-empty")
}
