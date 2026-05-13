use bywind::{Grib2Bbox, MapBounds, SavedSolution, WaypointCount};

use crate::app::BywindApp;

impl BywindApp {
    /// Returns a serializable snapshot of the path currently displayed (the gbest
    /// at the displayed iteration), plus the metadata needed to reproduce or
    /// reason about the search that produced it. Returns `None` when no search
    /// result is loaded.
    fn extract_displayed_solution(&self) -> Option<SavedSolution> {
        let gbest = self
            .outputs
            .route_evolution
            .as_ref()?
            .gbest_at(self.outputs.iteration)?;
        Some(SavedSolution {
            n: gbest.xs.len(),
            xs: gbest.xs.to_vec(),
            ys: gbest.ys.to_vec(),
            ts: gbest.ts.to_vec(),
            best_fit: gbest.best_fit,
            time_weight: self.search.time_weight,
            fuel_weight: self.search.fuel_weight,
            particles_space: self.search.particles_space,
            particles_time: self.search.particles_time,
            iter_space: self.search.iter_space,
            iter_time: self.search.iter_time,
            seed: self.search.seed,
            topology: self.search.topology,
            path_kick_probability: self.search.path_kick_probability,
            path_kick_gamma_0_fraction: self.search.path_kick_gamma_0_fraction,
            path_kick_gamma_min_fraction: self.search.path_kick_gamma_min_fraction,
        })
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn save_solution_to_file(&mut self, path: &std::path::Path) {
        let Some(saved) = self.extract_displayed_solution() else {
            self.report_error("No solution to save".to_owned());
            return;
        };
        let save = || -> Result<(), Box<dyn std::error::Error>> {
            let json = serde_json::to_string_pretty(&saved)?;
            std::fs::write(path, json)?;
            Ok(())
        };
        if let Err(e) = save() {
            self.report_error(format!(
                "Failed to save solution to {}: {e}",
                path.display()
            ));
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn load_solution_from_file(&mut self, path: &std::path::Path) {
        let saved: SavedSolution = match std::fs::read_to_string(path)
            .map_err(|e| -> Box<dyn std::error::Error> { Box::new(e) })
            .and_then(|json| {
                serde_json::from_str(&json)
                    .map_err(|e| -> Box<dyn std::error::Error> { Box::new(e) })
            }) {
            Ok(s) => s,
            Err(e) => {
                self.report_error(format!(
                    "Failed to load solution from {}: {e}",
                    path.display()
                ));
                return;
            }
        };

        let (wc, route_evolution) = match saved.to_route_evolution() {
            Ok(v) => v,
            Err(e) => {
                self.report_error(format!(
                    "Failed to load solution from {}: {e}",
                    path.display()
                ));
                return;
            }
        };

        self.search.waypoint_count = wc;
        self.outputs.route_evolution = Some(route_evolution);
        self.outputs.iteration = 0;
        self.search.time_weight = saved.time_weight;
        self.search.fuel_weight = saved.fuel_weight;
        self.search.particles_space = saved.particles_space;
        self.search.particles_time = saved.particles_time;
        self.search.iter_space = saved.iter_space;
        self.search.iter_time = saved.iter_time;
        self.search.seed = saved.seed;
        self.search.topology = saved.topology;
        self.search.path_kick_probability = saved.path_kick_probability;
        self.search.path_kick_gamma_0_fraction = saved.path_kick_gamma_0_fraction;
        self.search.path_kick_gamma_min_fraction = saved.path_kick_gamma_min_fraction;
        self.outputs.segment_stats = None;
        self.outputs.best_fitness = None;
        self.outputs.bake_duration = None;
        self.outputs.search_duration = None;

        // Re-bake the currently-loaded wind map so segment stats can be displayed
        // for the loaded path. Skipped if no wind map is present — geometry still
        // renders, stats just don't.
        if let Some(wind_map) = &self.wind_map
            && let Some(map_bounds) = MapBounds::from_wind_map(wind_map)
        {
            let bounds = map_bounds.clamp_to(self.editor.route_bbox);
            if bounds.is_non_degenerate() {
                let (origin, destination) =
                    bounds.resolve_endpoints(self.editor.start_waypoint, self.editor.end_waypoint);
                self.outputs.route_bounds = Some(bounds.to_route_bounds_with_step_fraction(
                    origin,
                    destination,
                    self.search.step_distance_fraction,
                ));
                self.outputs.baked_wind_map =
                    Some(wind_map.bake(bounds.to_bake_bounds(self.search.bake_step_deg)));
                self.outputs.boat = Some(self.boat.to_boat());
            }
        }
    }

    /// Write the current wind map to `path` as a `bywind::wind_av1`
    /// (AV1 near-lossless) file. Uses `EncodeParams::default()` — the
    /// rav1e quantizer / speed preset chosen to match the previous
    /// libaom CRF 10 baseline on size + drift.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn save_windmap_av1_to_file(&mut self, path: &std::path::Path) {
        let Some(wind_map) = self.wind_map.as_ref() else {
            self.report_error("No wind map loaded".to_owned());
            return;
        };
        let save = || -> Result<(), Box<dyn std::error::Error>> {
            let file = std::fs::File::create(path)?;
            let writer = std::io::BufWriter::new(file);
            bywind::wind_av1::encode(wind_map, writer, bywind::wind_av1::EncodeParams::default())?;
            Ok(())
        };
        if let Err(e) = save() {
            self.report_error(format!(
                "Failed to save AV1 wind map to {}: {e}",
                path.display()
            ));
        }
    }

    /// Read a `bywind::wind_av1` file and replace the current wind map.
    /// Decoder is the same rav1d-backed path the bundled-sample loader
    /// uses at startup.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn load_windmap_av1_from_file(&mut self, path: &std::path::Path) {
        match bywind::io::load_wcav(path) {
            Ok(map) => self.replace_wind_map(map),
            Err(e) => self.report_error(format!(
                "Failed to load AV1 wind map from {}: {e}",
                path.display(),
            )),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn load_windmap_from_grib2(&mut self, path: &std::path::Path) {
        let stride = self.editor.grib2_load_stride;
        let bbox = self.editor.grib2_load_bbox_active.then_some(Grib2Bbox {
            lat_min: self.editor.grib2_load_bbox_lat_min,
            lat_max: self.editor.grib2_load_bbox_lat_max,
            lon_min: self.editor.grib2_load_bbox_lon_min,
            lon_max: self.editor.grib2_load_bbox_lon_max,
        });
        match bywind::io::load_grib2(path, stride, bbox) {
            Ok(map) => self.replace_wind_map(map),
            Err(e) => {
                self.report_error(format!("Failed to load GRIB2 from {}: {e}", path.display()));
            }
        }
    }

    /// Snapshot the current editor / search / boat state into the same
    /// schema [`Self::load_scenario_from_file`] consumes. `[run]` carries
    /// only the fields the user has set (start / end / bounds may be
    /// missing); waypoints, weights, and the full `[boat]` / `[search]`
    /// sections are always emitted so the saved file is sufficient on
    /// its own to drive a `bywind-cli search` invocation.
    fn current_scenario(&self) -> bywind::scenario::CliConfigFile {
        use bywind::scenario::{
            BoatOverrides, CliConfigFile, RunOverrides, SearchOverrides, TuneOverrides,
        };
        CliConfigFile {
            run: RunOverrides {
                map: None,
                start: self.editor.start_waypoint.map(<[f64; 2]>::from),
                end: self.editor.end_waypoint.map(<[f64; 2]>::from),
                // EditorState stores `(lon_min, lon_max, lat_min, lat_max)`;
                // the TOML schema lays them out SW/NE.
                bounds: self
                    .editor
                    .route_bbox
                    .map(|(lon_min, lon_max, lat_min, lat_max)| {
                        [lon_min, lat_min, lon_max, lat_max]
                    }),
                waypoints: Some(self.search.waypoint_count.as_usize()),
                time_weight: Some(self.search.time_weight),
                fuel_weight: Some(self.search.fuel_weight),
                land_weight: Some(self.search.land_weight),
            },
            boat: BoatOverrides {
                mcr_kw: Some(self.boat.mcr_kw),
                k: Some(self.boat.k),
                polar_c: Some(self.boat.polar_c),
                polar_sin_power: Some(self.boat.polar_sin_power),
                fuel_a: Some(self.boat.fuel_a),
                fuel_b: Some(self.boat.fuel_b),
                fuel_c: Some(self.boat.fuel_c),
            },
            search: SearchOverrides {
                particles_space: Some(self.search.particles_space),
                particles_time: Some(self.search.particles_time),
                iter_space: Some(self.search.iter_space),
                iter_time: Some(self.search.iter_time),
                inertia: Some(self.search.inertia),
                cognitive_coeff: Some(self.search.cognitive_coeff),
                social_coeff: Some(self.search.social_coeff),
                path_kick_probability: Some(self.search.path_kick_probability),
                path_kick_gamma_0_fraction: Some(self.search.path_kick_gamma_0_fraction),
                path_kick_gamma_min_fraction: Some(self.search.path_kick_gamma_min_fraction),
                seed: self.search.seed,
                topology: Some(self.search.topology),
            },
            // The viz has no [tune] state of its own — that's a study-level
            // concept owned by `bywind-cli tune`. Saved scenarios from the
            // GUI emit an empty section header; loading a file with a
            // populated `[tune]` is parsed but currently dropped on save.
            tune: TuneOverrides::default(),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn save_scenario_to_file(&mut self, path: &std::path::Path) {
        let cfg = self.current_scenario();
        let save = || -> Result<(), Box<dyn std::error::Error>> {
            let text = cfg.to_toml_string()?;
            std::fs::write(path, text)?;
            Ok(())
        };
        if let Err(e) = save() {
            self.report_error(format!(
                "Failed to save scenario to {}: {e}",
                path.display(),
            ));
        }
    }

    /// Apply a TOML scenario file (`[run]` / `[boat]` / `[search]` sections,
    /// same schema `bywind-cli` consumes) to the current editor / search /
    /// boat state. Any `Some` field overwrites; `None` fields leave the
    /// existing value alone. Schedules a "Fit to view" so the next frame
    /// frames the loaded endpoints / bbox.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn load_scenario_from_file(&mut self, path: &std::path::Path) {
        let cfg = match bywind::scenario::CliConfigFile::from_path(path) {
            Ok(c) => c,
            Err(e) => {
                self.report_error(format!(
                    "Failed to load scenario from {}: {e}",
                    path.display(),
                ));
                return;
            }
        };

        if let Some(start) = cfg.run.start {
            self.editor.start_waypoint = Some(start.into());
        }
        if let Some(end) = cfg.run.end {
            self.editor.end_waypoint = Some(end.into());
        }
        if let Some([lon_min, lat_min, lon_max, lat_max]) = cfg.run.bounds {
            // `EditorState::route_bbox` is `(lon_min, lon_max, lat_min, lat_max)`;
            // the TOML lays them out SW/NE for human readability.
            self.editor.route_bbox = Some((lon_min, lon_max, lat_min, lat_max));
            // Take manual control: a user-supplied bbox shouldn't be
            // silently overwritten when endpoints move.
            self.editor.route_bbox_auto = false;
            self.editor.last_auto_endpoints = None;
        }
        if let Some(n) = cfg.run.waypoints {
            match WaypointCount::from_usize(n) {
                Some(wc) => self.search.waypoint_count = wc,
                None => log::warn!(
                    "scenario {} requested waypoints={n}, which isn't in WaypointCount::ALL; ignored",
                    path.display(),
                ),
            }
        }
        if let Some(w) = cfg.run.time_weight {
            self.search.time_weight = w;
        }
        if let Some(w) = cfg.run.fuel_weight {
            self.search.fuel_weight = w;
        }
        if let Some(w) = cfg.run.land_weight {
            self.search.land_weight = w;
        }

        cfg.boat.apply_to(&mut self.boat);
        cfg.search.apply_to(&mut self.search);

        self.view.fit_route_pending = true;
    }

    /// Install a freshly loaded wind map and reset all the per-map editor
    /// and view state so leftovers from the previous map don't bleed through.
    /// Called from the GRIB2 and `wind_av1` load paths.
    #[cfg(not(target_arch = "wasm32"))]
    fn replace_wind_map(&mut self, map: bywind::TimedWindMap) {
        self.editor.current_frame = self
            .editor
            .current_frame
            .min(map.frame_count().saturating_sub(1));
        self.editor.start_waypoint = None;
        self.editor.end_waypoint = None;
        self.editor.route_bbox = None;
        self.editor.route_bbox_drag_anchor = None;
        // Reset the view's projection origin so the next render re-derives
        // it from the new map's bbox centre.
        self.view.view_lon0 = None;
        self.view.view_lat0 = None;
        // Drop any cached wrap-region synthesised frame — it was built
        // against the previous wind map's grid and would render stale
        // content otherwise.
        self.view.synthesized_frame = None;
        self.wind_map = Some(map);
    }
}
