use std::sync::mpsc::{Receiver, TryRecvError};

use bywind::{
    BoatConfig, GenerateConfig, LonLatBbox, MapBounds, RobustMode, SearchConfig, SearchError,
    SearchResult, SearchWeights, TimedWindMap, WindInput, route_evolution_match,
    run_search_blocking, run_search_blocking_with_baked, run_time_reopt_blocking,
};

use crate::config::{EditorState, Tool, ViewState};
use crate::search::{ReoptMsg, SearchOutputs, SoloMemberRun};

/// We derive Deserialize/Serialize so we can persist app state on shutdown.
#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)] // if we add new fields, give them default values when deserializing old state
pub struct BywindApp {
    pub(crate) editor: EditorState,
    pub(crate) view: ViewState,
    pub(crate) generate: GenerateConfig,
    pub(crate) search: SearchConfig,
    pub(crate) boat: BoatConfig,
    pub(crate) outputs: SearchOutputs,

    #[serde(skip)]
    pub(crate) wind_map: Option<TimedWindMap>,

    /// Background sailing-search worker.
    #[serde(skip)]
    pub(crate) search_job: AsyncJob<Result<SearchResult, SearchError>>,

    /// Wall-clock instant the running search started, so the Cancel
    /// button can render an elapsed-time suffix. `Some` only while
    /// `search_job` is in `Running`; cleared when the result arrives or
    /// the user cancels.
    #[serde(skip)]
    pub(crate) search_started_at: Option<std::time::Instant>,

    /// Background time-only PSO fired on Waypoint-Edit drag-release.
    /// Each new drag overwrites the slot, cancelling the prior worker.
    #[serde(skip)]
    pub(crate) reopt_job: AsyncJob<ReoptMsg>,

    /// Background per-member solo-PSO cohort. Fired from the
    /// "Run solo per member" button; on success its payload replaces
    /// `outputs.solo_runs`. Independent of `search_job` so a main
    /// search can run alongside (though the UI gates them sequentially
    /// today to keep CPU contention predictable).
    #[serde(skip)]
    pub(crate) solo_job: AsyncJob<Result<Vec<SoloMemberRun>, SearchError>>,

    /// Wall-clock instant the running solo cohort started, mirroring
    /// `search_started_at` so the button can show elapsed time while
    /// the K-search loop is running.
    #[serde(skip)]
    pub(crate) solo_started_at: Option<std::time::Instant>,

    /// Background decoder for the embedded `wind_av1` sample dataset.
    /// Spawned at startup when `assets/sample_wind.wcav` was present at
    /// build time; the decoded `TimedWindMap` slots into `wind_map` as
    /// soon as it's ready.
    #[serde(skip)]
    pub(crate) bundled_sample_job: AsyncJob<Result<TimedWindMap, String>>,

    /// Background worker + log buffer for `File → Fetch from AWS…`.
    /// Streams `bywind::fetch::FetchProgress` events back via mpsc and
    /// holds the shared cancel flag.
    #[cfg(not(target_arch = "wasm32"))]
    #[serde(skip)]
    pub(crate) fetch_job: crate::fetch::FetchJob,

    /// User-facing error rendered as a toast. Cleared on Dismiss.
    #[serde(skip)]
    pub(crate) last_error: Option<String>,
}

// Every field uses its type's `Default`, including `wind_map: None`.
// `clippy::derivable_impls` would have us `#[derive(Default)]` instead,
// but the explicit impl is the natural place to call out *why* there's
// no up-front wind data: the embedded `wind_av1` sample (when present)
// decodes asynchronously and slots in within a few seconds, and
// without a sample the user reaches for `Advanced → Generate Wind Map`
// or `File → Load GRIB2…`. The prior behaviour of synthesising a
// random grid up-front was just a distractor that always got replaced.
#[expect(
    clippy::derivable_impls,
    reason = "documents the no-wind-map startup choice"
)]
impl Default for BywindApp {
    fn default() -> Self {
        Self {
            editor: EditorState::default(),
            view: ViewState::default(),
            generate: GenerateConfig::default(),
            search: SearchConfig::default(),
            boat: BoatConfig::default(),
            outputs: SearchOutputs::default(),
            wind_map: None,
            search_job: AsyncJob::default(),
            search_started_at: None,
            reopt_job: AsyncJob::default(),
            solo_job: AsyncJob::default(),
            solo_started_at: None,
            bundled_sample_job: AsyncJob::default(),
            #[cfg(not(target_arch = "wasm32"))]
            fetch_job: crate::fetch::FetchJob::default(),
            last_error: None,
        }
    }
}

impl BywindApp {
    /// Log `message` and surface it as a UI toast.
    pub(crate) fn report_error(&mut self, message: String) {
        log::error!("{message}");
        self.last_error = Some(message);
    }

    /// While CTRL is held, force `Tool::Pointer` so the user can pan /
    /// inspect without changing their selected tool. Release restores.
    pub(crate) fn apply_ctrl_pointer_override(&mut self, ctx: &egui::Context) {
        let ctrl = ctx.input(|i| i.modifiers.ctrl);
        if ctrl {
            if self.editor.pre_ctrl_tool.is_none() && self.editor.selected_tool != Tool::Pointer {
                self.editor.pre_ctrl_tool = Some(self.editor.selected_tool);
                self.editor.selected_tool = Tool::Pointer;
            }
        } else if let Some(prev) = self.editor.pre_ctrl_tool.take() {
            self.editor.selected_tool = prev;
        }
    }

    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut app: Self = if let Some(storage) = cc.storage {
            eframe::get_value(storage, eframe::APP_KEY).unwrap_or_default()
        } else {
            Default::default()
        };
        // Kick off the sample loader on every cold start. The random
        // wind map from `Default::default()` stays visible while the
        // worker runs — embed-decode (~12 s) or cache-decode is fast;
        // first-launch download adds the network round-trip on top.
        #[cfg(not(target_arch = "wasm32"))]
        app.start_bundled_sample_decode(&cc.egui_ctx);
        app
    }

    /// Spawn a worker that loads the `wind_av1` sample dataset —
    /// going through embed → cache → download per `bundled_sample`
    /// — then decodes the bytes into a `TimedWindMap`. Replaces an
    /// already-running job so the menu entry can re-trigger after the
    /// user has loaded other data. No-op on wasm.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn start_bundled_sample_decode(&mut self, ctx: &egui::Context) {
        let ctx = ctx.clone();
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let result = crate::bundled_sample::load_sample_bytes()
                .and_then(|bytes| bywind::wind_av1::decode(&bytes[..]).map_err(|e| e.to_string()));
            drop(tx.send(result));
            ctx.request_repaint();
        });
        self.bundled_sample_job.set_running(rx);
    }

    /// Spawn a background sailing search. Validates up-front so a bad
    /// slider doesn't waste a worker thread silently.
    pub(crate) fn run_search(&mut self, ctx: &egui::Context) {
        if self.search_job.is_running() {
            return;
        }
        // Missing endpoints would silently fall back to the wind-map
        // bbox corners (see `MapBounds::resolve_endpoints`). The result
        // is rarely what the user wants, and they have no visual cue
        // that the fallback was used. Refuse the click, animate the
        // Set Endpoints tool, and let the user place them.
        if self.editor.start_waypoint.is_none() || self.editor.end_waypoint.is_none() {
            self.editor.highlight_endpoint_tool = true;
            ctx.request_repaint();
            return;
        }
        if let Err(e) = self.boat.validate() {
            self.report_error(format!("Invalid boat config — {e}"));
            return;
        }
        if let Err(e) = self.search.validate() {
            self.report_error(format!("Invalid search config — {e}"));
            return;
        }
        // Ensemble dispatch: when the user has set an ensemble dir in
        // advanced settings, the GUI doesn't need an in-memory wind
        // map — the worker thread will load the ensemble itself. We
        // still need *some* map bounds to derive the bake / route
        // window; the user-drawn route bbox is the only source we
        // have without paying the load cost on the UI thread, so
        // require it.
        let ensemble_dispatch = self.search.ensemble_path.clone();
        let map_bounds = if let Some(_ensemble) = &ensemble_dispatch {
            let Some(bbox) = self.editor.route_bbox else {
                self.report_error(
                    "Ensemble search needs a route bbox to size the bake. \
                     Draw one with the Route Bounds tool, then click Run Search."
                        .to_owned(),
                );
                return;
            };
            // `EditorState::route_bbox` is `(lon_min, lon_max, lat_min,
            // lat_max)` per io.rs:286 (the TOML layout convention),
            // which matches `LonLatBbox::new`'s argument order
            // exactly. Passing it through verbatim — earlier code
            // here had the components permuted, which made the
            // ensemble bake bounds malformed and silently broke the
            // A* benchmark.
            MapBounds {
                bbox: LonLatBbox::new(bbox.0, bbox.1, bbox.2, bbox.3),
            }
        } else {
            let Some(wind_map) = &self.wind_map else {
                return;
            };
            let Some(mb) = MapBounds::from_wind_map(wind_map) else {
                return;
            };
            mb
        };
        let bounds = map_bounds.clamp_to(self.editor.route_bbox);
        if !bounds.is_non_degenerate() {
            self.report_error(
                "Route Bounds rectangle does not overlap the wind map; clear it or redraw."
                    .to_owned(),
            );
            return;
        }

        let (origin, destination) =
            bounds.resolve_endpoints(self.editor.start_waypoint, self.editor.end_waypoint);
        // Both endpoints are present by the early-return above; clear
        // the "set endpoints" prompt now that the worker is launching.
        self.editor.highlight_endpoint_tool = false;
        let route_bounds = bounds.to_route_bounds_with_step_fraction(
            origin,
            destination,
            self.search.step_distance_fraction,
        );
        let bake_bounds = bounds.to_bake_bounds(self.search.bake_step_deg);
        let sdf_resolution = self.search.sdf_resolution_deg;
        let fine_sdf_resolution = self.search.fine_sdf_resolution_deg;

        let ctx = ctx.clone();
        let waypoint_count = self.search.waypoint_count;
        let weights = SearchWeights {
            time_weight: self.search.time_weight,
            fuel_weight: self.search.fuel_weight,
            land_weight: self.search.land_weight,
        };
        // Resolve the seed up front so the Advanced Params UI can show
        // what the search actually ran with — including when the user
        // left "Deterministic seed" unchecked (we still pick a u64 here
        // and pass it through as `Some(_)` so the run is reproducible
        // after the fact via the displayed value).
        let mut search_settings = self.search.to_search_settings();
        let effective_seed = search_settings.seed.unwrap_or_else(rand::random);
        search_settings.seed = Some(effective_seed);
        self.outputs.last_search_seed = Some(effective_seed);
        let ship = self.boat.to_boat();
        let robust_mode = self.search.robust_mode;
        let (tx, rx) = std::sync::mpsc::channel();

        if let Some(ensemble_path) = ensemble_dispatch {
            // Ensemble path: load + bake + search all in the worker
            // thread so the UI stays responsive during the ~3 s load.
            std::thread::spawn(move || {
                let result = run_ensemble_search(
                    &ensemble_path,
                    bake_bounds,
                    route_bounds,
                    waypoint_count,
                    search_settings,
                    ship,
                    weights,
                    sdf_resolution,
                    fine_sdf_resolution,
                    robust_mode,
                );
                drop(tx.send(result));
                ctx.request_repaint();
            });
        } else {
            // Single-deterministic path: existing flow. `wind_map` is
            // guaranteed `Some` here because the ensemble dispatch
            // branch above is the only way to skip the `wind_map`
            // requirement, and we wouldn't be on this branch if it
            // had fired.
            let wind_map_snapshot: TimedWindMap = self
                .wind_map
                .as_ref()
                .expect("wind_map presence is checked at the top of run_search")
                .clone();
            std::thread::spawn(move || {
                let result = run_search_blocking(
                    &wind_map_snapshot,
                    bake_bounds,
                    route_bounds,
                    waypoint_count,
                    search_settings,
                    ship,
                    weights,
                    sdf_resolution,
                    fine_sdf_resolution,
                );
                drop(tx.send(result));
                ctx.request_repaint();
            });
        }

        self.search_job.set_running(rx);
        self.search_started_at = Some(std::time::Instant::now());
    }

    /// Per-member solo PSO cohort. Runs K independent
    /// single-deterministic searches, one per ensemble member, so the
    /// user can see what alternate routes each member's wind would
    /// produce — complement to the ensemble assessment, which only
    /// scores the chosen gbest against each member. Requires
    /// `ensemble_path` and a route bbox to be set; otherwise reports a
    /// toast and bails. The K searches run sequentially in the worker
    /// to keep CPU usage predictable (each search is already
    /// internally rayon-parallel).
    pub(crate) fn run_solo_per_member(&mut self, ctx: &egui::Context) {
        if self.solo_job.is_running() {
            return;
        }
        let Some(ensemble_path) = self.search.ensemble_path.clone() else {
            self.report_error(
                "Solo per-member needs an ensemble dir set in Advanced Params.".to_owned(),
            );
            return;
        };
        let Some(route_bbox) = self.editor.route_bbox else {
            self.report_error(
                "Solo per-member needs a route bbox to size the bake. Draw one with \
                 the Route Bounds tool, then click Run solo per member."
                    .to_owned(),
            );
            return;
        };
        if let Err(e) = self.boat.validate() {
            self.report_error(format!("Invalid boat config — {e}"));
            return;
        }
        if let Err(e) = self.search.validate() {
            self.report_error(format!("Invalid search config — {e}"));
            return;
        }
        let map_bounds = MapBounds {
            bbox: LonLatBbox::new(route_bbox.0, route_bbox.1, route_bbox.2, route_bbox.3),
        };
        let bounds = map_bounds.clamp_to(Some(route_bbox));
        if !bounds.is_non_degenerate() {
            self.report_error(
                "Route Bounds rectangle is degenerate; clear it or redraw.".to_owned(),
            );
            return;
        }
        let (origin, destination) =
            bounds.resolve_endpoints(self.editor.start_waypoint, self.editor.end_waypoint);
        self.editor.highlight_endpoint_tool = false;
        let route_bounds = bounds.to_route_bounds_with_step_fraction(
            origin,
            destination,
            self.search.step_distance_fraction,
        );
        let bake_bounds = bounds.to_bake_bounds(self.search.bake_step_deg);
        let sdf_resolution = self.search.sdf_resolution_deg;
        let fine_sdf_resolution = self.search.fine_sdf_resolution_deg;
        let waypoint_count = self.search.waypoint_count;
        let weights = SearchWeights {
            time_weight: self.search.time_weight,
            fuel_weight: self.search.fuel_weight,
            land_weight: self.search.land_weight,
        };
        // Resolve the seed up front like `run_search`. We then derive
        // per-member seeds as `base + k` inside the worker so distinct
        // members aren't initialised with the exact same swarm cloud —
        // identical seeds would let two near-identical winds collapse
        // to bit-identical routes, defeating the point of an overlay.
        let mut search_settings = self.search.to_search_settings();
        let base_seed = search_settings.seed.unwrap_or_else(rand::random);
        search_settings.seed = Some(base_seed);
        let ship = self.boat.to_boat();
        let ctx = ctx.clone();
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let result = run_solo_per_member_search(
                &ensemble_path,
                bake_bounds,
                route_bounds,
                waypoint_count,
                search_settings,
                base_seed,
                ship,
                weights,
                sdf_resolution,
                fine_sdf_resolution,
            );
            drop(tx.send(result));
            ctx.request_repaint();
        });
        self.solo_job.set_running(rx);
        self.solo_started_at = Some(std::time::Instant::now());
    }

    /// Re-run only the time PSO with the user-edited xy fixed. Each
    /// drag-release overwrites the slot so the prior reopt is cancelled.
    pub(crate) fn start_time_reopt(&mut self, ctx: &egui::Context) {
        let Some(baked_ref) = self.outputs.baked_wind_map.as_ref() else {
            return;
        };
        let Some(rb) = self.outputs.route_bounds else {
            return;
        };
        let Some(re) = self.outputs.route_evolution.as_ref() else {
            return;
        };

        let iteration = self.outputs.iteration;
        let weights = SearchWeights {
            time_weight: self.search.time_weight,
            fuel_weight: self.search.fuel_weight,
            land_weight: self.search.land_weight,
        };
        let settings = self.search.to_search_settings();
        let ship = self.boat.to_boat();
        let sdf_resolution = self.search.sdf_resolution_deg;
        let fine_sdf_resolution = self.search.fine_sdf_resolution_deg;

        let baked = baked_ref.clone();
        let ctx = ctx.clone();
        let (tx, rx) = std::sync::mpsc::channel::<ReoptMsg>();

        // The macro dispatches over the gbest path's const-generic N
        // so a typed `Path<N>` can move into the closure.
        route_evolution_match!(re, |evo| {
            let frames = evo.frames();
            let iter_idx = iteration.min(frames.len().saturating_sub(1));
            let Some(particles) = frames.get(iter_idx) else {
                return;
            };
            let Some(best) = particles.iter().max_by(|a, b| {
                a.best_fit
                    .partial_cmp(&b.best_fit)
                    .unwrap_or(std::cmp::Ordering::Equal)
            }) else {
                return;
            };
            let fixed_path = best.best_pos;
            std::thread::spawn(move || {
                let path = run_time_reopt_blocking(
                    &baked,
                    rb,
                    settings,
                    &ship,
                    fixed_path,
                    weights,
                    sdf_resolution,
                    fine_sdf_resolution,
                );
                // Drop silently if a later drag-release overwrote the slot.
                drop(tx.send(ReoptMsg {
                    iteration: iter_idx,
                    new_times: path.t.0.0.to_vec(),
                }));
                ctx.request_repaint();
            });
        });

        self.reopt_job.set_running(rx);
    }

    /// Splice a reopt result's `new_times` into the gbest particle's `best_pos.t`
    /// at the recorded iteration. The length check inside `apply_reopt_times`
    /// drops the message if the path's N has changed since dispatch (e.g. the
    /// user re-ran the search with a different waypoint count between drag-
    /// release and result arrival).
    pub(crate) fn apply_reopt_result(&mut self, msg: &ReoptMsg) {
        if let Some(re) = self.outputs.route_evolution.as_mut() {
            re.apply_reopt_times(msg.iteration, &msg.new_times);
        }
    }
}

/// Single-shot background-job slot. Wraps the `Option<Receiver<T>>` +
/// `try_recv` polling pattern used by the search worker and the time-reopt
/// worker.
///
/// Idle by default. `set_running` parks a fresh receiver; `poll` drains the
/// channel, transitioning back to `Idle` on either `Ok` or `Disconnected`
/// so the caller doesn't have to distinguish "result arrived" from "worker
/// died". Lives in `app.rs` because both call sites — and the abstraction
/// itself — are tied to `BywindApp`'s update loop.
#[derive(Default)]
pub(crate) enum AsyncJob<T> {
    #[default]
    Idle,
    Running(Receiver<T>),
}

impl<T> AsyncJob<T> {
    pub(crate) fn is_running(&self) -> bool {
        matches!(self, Self::Running(_))
    }

    /// Replaces any in-flight receiver. The prior one's eventual
    /// `tx.send` becomes a no-op — used by time-reopt to cancel the
    /// prior worker on each drag-release.
    pub(crate) fn set_running(&mut self, rx: Receiver<T>) {
        *self = Self::Running(rx);
    }

    /// Drop the receiver and go back to `Idle` without waiting on the
    /// worker. Used by the Cancel button: the spawned thread keeps
    /// running until it finishes naturally, but its eventual `tx.send`
    /// becomes a no-op so the UI is responsive immediately.
    pub(crate) fn cancel(&mut self) {
        *self = Self::Idle;
    }

    /// `Some(v)` once, then `Idle` on either `Ok` or `Disconnected`.
    pub(crate) fn poll(&mut self) -> Option<T> {
        let rx = match self {
            Self::Idle => return None,
            Self::Running(rx) => rx,
        };
        match rx.try_recv() {
            Ok(value) => {
                *self = Self::Idle;
                Some(value)
            }
            Err(TryRecvError::Disconnected) => {
                *self = Self::Idle;
                None
            }
            Err(TryRecvError::Empty) => None,
        }
    }
}

impl eframe::App for BywindApp {
    /// Per-frame orchestrator. Panel render order matches egui's
    /// requirement: top/bottom → side → central.
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.apply_ctrl_pointer_override(ui.ctx());

        if let Some(msg) = self.search_job.poll() {
            self.search_started_at = None;
            match msg {
                Ok(SearchResult {
                    route_evolution,
                    route_bounds,
                    baked,
                    boat,
                    benchmark,
                    bake_duration,
                    search_duration,
                    ensemble,
                }) => {
                    self.outputs.iteration = route_evolution.iter_count().saturating_sub(1);
                    self.outputs.route_evolution = Some(route_evolution);
                    self.outputs.route_bounds = Some(route_bounds);
                    self.outputs.baked_wind_map = Some(baked);
                    self.outputs.boat = Some(boat);
                    self.outputs.benchmark = benchmark;
                    self.outputs.bake_duration = Some(bake_duration);
                    self.outputs.search_duration = Some(search_duration);
                    self.outputs.ensemble = ensemble;
                    // Drop any prior solo cohort — it was scored
                    // against the previous route bounds / endpoints
                    // and would draw stale routes over the new gbest.
                    self.outputs.solo_runs.clear();
                }
                Err(e) => {
                    // Prior outputs stay displayed — discarding them
                    // would hide a search the user may still be looking at.
                    self.report_error(format!("Search failed — {e}"));
                }
            }
        }

        // Reopt result is spliced into the gbest path's `t` so segment stats
        // and totals refresh on the next render.
        if let Some(msg) = self.reopt_job.poll() {
            self.apply_reopt_result(&msg);
        }

        // Solo per-member cohort lands as a `Vec<SoloMemberRun>` —
        // assign wholesale, overwriting any prior cohort. Failures
        // surface a toast and leave the previous cohort visible (same
        // policy as the main search arm).
        if let Some(result) = self.solo_job.poll() {
            self.solo_started_at = None;
            match result {
                Ok(runs) => {
                    self.outputs.solo_runs = runs;
                }
                Err(e) => {
                    self.report_error(format!("Solo per-member failed — {e}"));
                }
            }
        }

        // The embedded-sample decoder runs once at startup. When it
        // lands we slot the bundled wind map into `wind_map` and clear
        // any persisted waypoint state pinned to a prior session's
        // coordinates.
        if let Some(result) = self.bundled_sample_job.poll() {
            match result {
                Ok(map) => {
                    self.wind_map = Some(map);
                    self.editor.start_waypoint = None;
                    self.editor.end_waypoint = None;
                    self.editor.route_bbox = None;
                    self.view.view_lon0 = None;
                    self.view.view_lat0 = None;
                    // Invalidate any wrap-region cache pinned to the
                    // pre-swap wind map's grid layout.
                    self.view.synthesized_frame = None;
                }
                Err(e) => {
                    self.report_error(format!("failed to load bundled wind sample: {e}"));
                }
            }
        }

        // Drain progress events from the AWS-fetch worker (if any) and
        // swap the resulting map in when the worker reports `Done`.
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(map) = self.fetch_job.poll() {
            self.wind_map = Some(map);
            self.editor.start_waypoint = None;
            self.editor.end_waypoint = None;
            self.editor.route_bbox = None;
            self.view.view_lon0 = None;
            self.view.view_lat0 = None;
            self.view.synthesized_frame = None;
        }

        // Clamp persisted UI state against the (possibly reloaded) wind map.
        if let Some(wind_map) = &self.wind_map {
            let max_frame = wind_map.frame_count().saturating_sub(1);
            self.editor.current_frame = self.editor.current_frame.min(max_frame);
        }

        // Re-derive the route bbox before panel render so the new
        // rectangle draws this frame, not next.
        self.update_auto_route_bbox();

        self.render_menu_bar(ui);
        self.render_stats_panel(ui);
        self.render_tools_panel(ui);
        self.render_central_panel(ui);
        self.render_grib2_load_dialog(ui);
        #[cfg(not(target_arch = "wasm32"))]
        self.render_fetch_dialog(ui);
        self.render_advanced_settings_window(ui.ctx());
        self.render_generate_window(ui.ctx());
        self.render_error_toast(ui);
    }

    /// Called by the framework to save state before shutdown.
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }
}

/// Worker-side ensemble search. Loads the K members from disk, bakes
/// them in parallel, wraps them in the right [`WindInput`] variant
/// based on `robust_mode`, and dispatches to
/// `run_search_blocking_with_baked`.
///
/// Returns the same `Result<SearchResult, SearchError>` shape as
/// [`run_search_blocking`] so the UI-side polling code doesn't need to
/// know which path produced the result.
#[expect(
    clippy::too_many_arguments,
    reason = "the search worker is a glue function with no natural shared bundle"
)]
fn run_ensemble_search(
    ensemble_path: &std::path::Path,
    bake_bounds: bywind::wind_map::BakeBounds,
    route_bounds: bywind::RouteBounds,
    waypoint_count: bywind::WaypointCount,
    search_settings: bywind::SearchSettings,
    ship: bywind::Boat,
    weights: SearchWeights,
    sdf_resolution_deg: f64,
    fine_sdf_resolution_deg: Option<f64>,
    robust_mode: RobustMode,
) -> Result<SearchResult, SearchError> {
    let ensemble = match bywind::TimedEnsembleWindMap::load_dir(ensemble_path) {
        Ok(e) => e,
        Err(e) => {
            log::error!("ensemble load failed: {e}");
            return Err(SearchError::NoFeasibleRoute {
                best_fit: f64::NAN,
            });
        }
    };
    let baked = ensemble.bake(bake_bounds);
    let wind_input = match robust_mode {
        RobustMode::Full => WindInput::ensemble_full(baked),
        RobustMode::FastMean => WindInput::ensemble_fast_mean(baked),
    };
    run_search_blocking_with_baked(
        wind_input,
        route_bounds,
        waypoint_count,
        search_settings,
        ship,
        weights,
        sdf_resolution_deg,
        fine_sdf_resolution_deg,
    )
}

/// Worker-side per-member solo cohort. Loads + bakes the ensemble
/// once, then runs K single-deterministic searches — one per member,
/// each with seed `base_seed + k` so identically-initialised swarms
/// don't collapse near-identical winds to bit-identical routes.
///
/// Sequential rather than rayon-parallel: each inner search is
/// already internally parallel via rayon's global pool, so nesting
/// would oversubscribe cores. On a K=3 GEFS smoke this is ~3× the
/// wallclock of a single search; on K=31 (full GEFS) it's ~31×.
///
/// First failure short-circuits the whole cohort (returns the error)
/// so a wind input that's infeasible for one member doesn't quietly
/// publish a partial overlay.
#[expect(
    clippy::too_many_arguments,
    reason = "glue function bridging Run-Solo button to the inner search; \
              bundling args would just shuffle them around"
)]
fn run_solo_per_member_search(
    ensemble_path: &std::path::Path,
    bake_bounds: bywind::wind_map::BakeBounds,
    route_bounds: bywind::RouteBounds,
    waypoint_count: bywind::WaypointCount,
    mut search_settings: bywind::SearchSettings,
    base_seed: u64,
    ship: bywind::Boat,
    weights: SearchWeights,
    sdf_resolution_deg: f64,
    fine_sdf_resolution_deg: Option<f64>,
) -> Result<Vec<SoloMemberRun>, SearchError> {
    let ensemble = match bywind::TimedEnsembleWindMap::load_dir(ensemble_path) {
        Ok(e) => e,
        Err(e) => {
            log::error!("ensemble load failed: {e}");
            return Err(SearchError::NoFeasibleRoute {
                best_fit: f64::NAN,
            });
        }
    };
    let baked = ensemble.bake(bake_bounds);
    let names = baked.member_names().to_vec();
    let mut solo_runs = Vec::with_capacity(names.len());
    for (k, name) in names.iter().enumerate() {
        search_settings.seed = Some(base_seed.wrapping_add(k as u64));
        let member_baked = baked.member(k).clone();
        let result = run_search_blocking_with_baked(
            WindInput::single(member_baked),
            route_bounds,
            waypoint_count,
            search_settings,
            ship,
            weights,
            sdf_resolution_deg,
            fine_sdf_resolution_deg,
        )?;
        solo_runs.push(SoloMemberRun {
            name: name.clone(),
            route_evolution: result.route_evolution,
        });
    }
    Ok(solo_runs)
}

#[cfg(test)]
mod async_job_tests {
    use super::AsyncJob;
    use std::sync::mpsc;

    #[test]
    fn idle_poll_stays_idle() {
        let mut job: AsyncJob<i32> = AsyncJob::default();
        assert!(!job.is_running());
        assert_eq!(job.poll(), None);
        assert!(!job.is_running());
    }

    #[test]
    fn running_with_no_message_polls_none_and_stays_running() {
        let (_tx, rx) = mpsc::channel::<i32>();
        let mut job = AsyncJob::default();
        job.set_running(rx);
        assert!(job.is_running());
        assert_eq!(job.poll(), None);
        assert!(
            job.is_running(),
            "an empty channel must not transition to Idle"
        );
    }

    #[test]
    fn running_with_message_returns_value_then_becomes_idle() {
        let (tx, rx) = mpsc::channel();
        let mut job = AsyncJob::default();
        job.set_running(rx);
        tx.send(42).unwrap();
        assert_eq!(job.poll(), Some(42));
        assert!(!job.is_running());
        assert_eq!(job.poll(), None, "second poll on idle returns None");
    }

    #[test]
    fn disconnected_sender_transitions_to_idle() {
        let (tx, rx) = mpsc::channel::<i32>();
        let mut job = AsyncJob::default();
        job.set_running(rx);
        drop(tx);
        assert_eq!(job.poll(), None);
        assert!(!job.is_running());
    }

    #[test]
    fn set_running_overwrites_previous_receiver() {
        // Time-reopt cancellation: replacing the slot drops `rx1` and
        // its queued value.
        let (tx1, rx1) = mpsc::channel();
        tx1.send(1).unwrap();
        let (tx2, rx2) = mpsc::channel();
        let mut job = AsyncJob::default();
        job.set_running(rx1);
        job.set_running(rx2);
        tx2.send(2).unwrap();
        assert_eq!(job.poll(), Some(2));
    }
}
