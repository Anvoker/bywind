#[derive(serde::Deserialize, serde::Serialize, PartialEq, Default, Clone, Copy)]
pub(crate) enum Tool {
    #[default]
    Pointer,
    Speed,
    Direction,
    WaypointEdit,
    WaypointTime,
    /// Left-click sets the route start, right-click sets the end.
    /// Supersedes the bbox-corner default for the next search.
    Endpoint,
    /// Click-drag to define the rectangular search domain in map
    /// coordinates; right-click clears it.
    RouteBounds,
}

/// Persistent + transient editor state.
#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub(crate) struct EditorState {
    pub(crate) selected_tool: Tool,
    pub(crate) brush_radius: f32,
    pub(crate) current_frame: usize,

    /// Keep every Nth unique lat / lon on "Load GRIB2...". `1` keeps
    /// everything. Persisted because users typically want the same
    /// coarsening across sessions.
    pub(crate) grib2_load_stride: usize,

    /// When true, "Load GRIB2..." filters input points to the
    /// `grib2_load_bbox_*` rectangle.
    pub(crate) grib2_load_bbox_active: bool,
    pub(crate) grib2_load_bbox_lat_min: f32,
    pub(crate) grib2_load_bbox_lat_max: f32,
    pub(crate) grib2_load_bbox_lon_min: f32,
    pub(crate) grib2_load_bbox_lon_max: f32,

    /// User-placed start point in wind-map `(x, y)`. Cleared on every
    /// wind-map load — coordinates are map-specific.
    #[serde(skip)]
    pub(crate) start_waypoint: Option<(f64, f64)>,

    /// User-placed end point. Same lifecycle as `start_waypoint`.
    #[serde(skip)]
    pub(crate) end_waypoint: Option<(f64, f64)>,

    #[serde(skip)]
    pub(crate) grib2_load_dialog_open: bool,

    /// User-defined search domain `(x_min, x_max, y_min, y_max)`.
    /// Cleared on every wind-map load (coordinate system changes).
    #[serde(skip)]
    pub(crate) route_bbox: Option<(f64, f64, f64, f64)>,

    /// True while `route_bbox` is auto-managed from the endpoints via
    /// `derive_route_bbox`. Goes off when the user takes manual
    /// control; "Auto-set bounds" brings it back. Default true so a
    /// fresh session with placed endpoints picks up an automatic bbox.
    #[serde(skip)]
    pub(crate) route_bbox_auto: bool,

    /// Endpoints used by the last auto-derive. Lets the per-frame
    /// update skip the A* probe when endpoints haven't moved.
    #[serde(skip)]
    pub(crate) last_auto_endpoints: Option<((f64, f64), (f64, f64))>,

    /// Screen-space anchor of an in-flight Route Bounds drag — held
    /// in screen space so the live preview tracks the cursor through
    /// mid-drag pan / zoom.
    #[serde(skip)]
    pub(crate) route_bbox_drag_anchor: Option<egui::Pos2>,

    /// Index of the gbest waypoint being dragged. `None` between drags.
    #[serde(skip)]
    pub(crate) dragging_waypoint: Option<usize>,

    /// Tool active before the user started holding CTRL. While `Some`,
    /// `selected_tool` is forced to Pointer; release restores it.
    #[serde(skip)]
    pub(crate) pre_ctrl_tool: Option<Tool>,

    #[serde(skip)]
    pub(crate) advanced_settings_open: bool,

    /// Inputs for the `File → Fetch from AWS…` dialog. Reset every
    /// session — relative defaults (`now`-relative) would be confusing
    /// to load from a stale settings file.
    #[serde(skip)]
    pub(crate) fetch_dialog: FetchDialogState,

    /// True while the standalone Wind-data generator window (opened from
    /// Settings → Advanced…) is showing. Reset on every session.
    #[serde(skip)]
    pub(crate) generate_window_open: bool,

    /// Last typed seed before the user unchecked "Deterministic seed";
    /// re-checking restores it instead of resetting to 0.
    #[serde(skip)]
    pub(crate) parked_seed: Option<u64>,

    /// True after the user clicked Run Search with no endpoints set:
    /// the Set Endpoints selectable in the tool picker paints an
    /// animated highlight so the user knows what's missing. Cleared
    /// once Set Endpoints is selected or a search is successfully
    /// initiated with both endpoints in place.
    #[serde(skip)]
    pub(crate) highlight_endpoint_tool: bool,
}

impl Default for EditorState {
    fn default() -> Self {
        Self {
            selected_tool: Tool::Pointer,
            // 0.5° ≈ 55 km at the equator — fine grain on a 0.25° GFS
            // map without wiping out the whole region.
            brush_radius: 0.5,
            current_frame: 0,
            grib2_load_stride: 1,
            grib2_load_dialog_open: false,
            grib2_load_bbox_active: false,
            // North-Atlantic-ish placeholder so first-time toggle of
            // bbox-active doesn't land on a degenerate (0, 0, 0, 0).
            grib2_load_bbox_lat_min: 25.0,
            grib2_load_bbox_lat_max: 60.0,
            grib2_load_bbox_lon_min: -75.0,
            grib2_load_bbox_lon_max: -10.0,
            start_waypoint: None,
            end_waypoint: None,
            route_bbox: None,
            route_bbox_auto: true,
            last_auto_endpoints: None,
            route_bbox_drag_anchor: None,
            dragging_waypoint: None,
            pre_ctrl_tool: None,
            advanced_settings_open: false,
            fetch_dialog: FetchDialogState::default(),
            generate_window_open: false,
            parked_seed: None,
            highlight_endpoint_tool: false,
        }
    }
}

/// `File → Fetch from AWS…` dialog state. Lives outside `EditorState`
/// in the same file so the dialog renderer in `ui.rs` and the worker
/// spawn helper in `app.rs` share one home for the inputs.
#[derive(Default)]
pub(crate) struct FetchDialogState {
    /// True while the modal is being rendered. Cleared by Close /
    /// window-X / Esc.
    pub(crate) open: bool,
    /// Window start as `YYYYMMDDHH`. Populated from `Utc::now() - 240h`
    /// the first time the dialog opens this session.
    pub(crate) start_text: String,
    /// Window end as `YYYYMMDDHH`. Populated from the most recent 6 h
    /// GFS cycle ≤ `Utc::now()` on first open.
    pub(crate) end_text: String,
    /// Frame cadence in hours. Allowed: 1, 2, 3, 6.
    pub(crate) interval_h: u32,
    /// Destination path. The format combo keeps its extension in sync;
    /// users can also edit the field directly.
    pub(crate) out_path: String,
    /// Output format the user picked. Drives the Save dialog's filter
    /// on Browse… and keeps `out_path`'s extension in step.
    pub(crate) out_format: FetchOutputFormat,
    /// `true` after the first open in this session populated the
    /// defaults. Stops `Utc::now()` from clobbering user edits the next
    /// time they reopen the dialog.
    pub(crate) populated: bool,
}

/// Which encoder/path the fetch dialog targets. Maps onto the same
/// `bywind::io::Format` the rest of the binary uses, but kept separate
/// so the UI doesn't drag a `bywind::io` dependency into a state struct.
#[derive(Default, PartialEq, Eq, Clone, Copy)]
pub(crate) enum FetchOutputFormat {
    /// AV1 near-lossless `.wcav`. Default — keeps the artifact small
    /// enough to ship.
    #[default]
    Wcav,
    /// Raw GRIB2 concatenation. Slightly faster (no re-encode pass)
    /// and the canonical exchange format for downstream tools.
    Grib2,
}

impl FetchOutputFormat {
    /// File-extension this format writes to, without a leading dot.
    pub(crate) fn extension(self) -> &'static str {
        match self {
            Self::Wcav => "wcav",
            Self::Grib2 => "grib2",
        }
    }

    /// Human-readable label for the combo box.
    pub(crate) fn label(self) -> &'static str {
        match self {
            Self::Wcav => "Bywind AV1 (.wcav)",
            Self::Grib2 => "GRIB2 (.grib2)",
        }
    }
}

/// View / camera state for the central panel.
#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub(crate) struct ViewState {
    /// Pixels per ground metre at the view's projection origin
    /// (`cos(lat0)` longitude factor applied in
    /// [`crate::view::ViewTransform`]). Default `1e-4` ≈ 11 px/° at
    /// the equator. Render-only; doesn't affect the search.
    pub(crate) render_scale: f32,

    /// True → draw every particle's pbest; false → only the swarm best.
    pub(crate) show_all_particles: bool,

    /// True formats route time as `Nd Nh Nm Ns`; false as raw seconds.
    /// Right-click on the Summary heading toggles.
    pub(crate) total_time_breakdown: bool,

    /// True → fuel labels in tonnes; false → kg. Toggled together
    /// with `total_time_breakdown` on Summary right-click.
    pub(crate) total_fuel_tonnes: bool,

    /// Pointer-tool pan offset on top of the panel's base offset.
    /// Reset each session — stale pans land on empty space when the
    /// next wind map has different bounds.
    #[serde(skip)]
    pub(crate) pan_offset: egui::Vec2,

    /// Equirectangular projection origin `(lon0, lat0)`. Set to the
    /// wind-map bbox centre on load, driven by pan-wrap after.
    /// `None` until the first map loads.
    #[serde(skip)]
    pub(crate) view_lon0: Option<f32>,
    #[serde(skip)]
    pub(crate) view_lat0: Option<f32>,

    /// Set by "Fit to view"; consumed once the central panel knows
    /// its rect.
    #[serde(skip)]
    pub(crate) autoscale_pending: bool,

    /// Set by `load_scenario_from_file` — fit the view to the loaded
    /// route's span instead of the full wind-map extent. Antimeridian
    /// wrap-arounds still need manual recentring (linear-bbox fit
    /// overshoots).
    #[serde(skip)]
    pub(crate) fit_route_pending: bool,

    /// Height of the central panel from the most recent render. Drives
    /// the dynamic zoom-out floor in `ui::min_render_scale` so the world
    /// map can never shrink below the panel height. Zero until the first
    /// central-panel render has happened.
    #[serde(skip)]
    pub(crate) last_panel_height: f32,

    /// Central panel's rect from the most recent render. Drives
    /// "zoom around centre" when the user adjusts the scale via the
    /// View slider / steppers / typed value (as opposed to the mouse
    /// wheel, which already zooms around the cursor). `None` until
    /// the first central-panel render has happened.
    #[serde(skip)]
    pub(crate) last_panel_rect: Option<egui::Rect>,

    /// Cached synthesised wind frame for time-axis indexes past the
    /// data end (in the crossfade / wrap region). Key is the frame
    /// index the cache was built for; mismatches trigger a rebuild.
    /// `None` means "no cache yet" or "current frame index is inside
    /// the data and uses a real frame directly".
    #[serde(skip)]
    pub(crate) synthesized_frame: Option<(usize, bywind::WindMap)>,
}

impl Default for ViewState {
    fn default() -> Self {
        Self {
            render_scale: 1e-4,
            show_all_particles: false,
            total_time_breakdown: true,
            total_fuel_tonnes: true,
            pan_offset: egui::Vec2::ZERO,
            view_lon0: None,
            view_lat0: None,
            autoscale_pending: false,
            fit_route_pending: false,
            last_panel_height: 0.0,
            last_panel_rect: None,
            synthesized_frame: None,
        }
    }
}
