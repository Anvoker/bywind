//! Wind-data types and sailing-search logic shared by `bywind-viz` (the
//! egui GUI), `bywind-cli` (the command-line frontend), and any other
//! headless consumer. No GUI dependencies.
//!
//! Module layout:
//!
//! - [`wind_map`] — `WindMap` / `TimedWindMap` / `BakedWindMap`, the time-varying
//!   wind grid and its precomputed sampling structure.
//! - [`grib2`] — load a `TimedWindMap` from GRIB2 files.
//! - [`wind_av1`] — AV1 near-lossless wind-map codec for fast save/load.
//! - [`io`] — extension-dispatched loaders that pick between GRIB2 and
//!   `wind_av1` automatically. The single-stop entry point used by both the
//!   CLI and the GUI's file dialogs.
//! - [`fetch`] — pull GFS UGRD/VGRD-at-10m messages from NOAA's S3 bucket
//!   into a concatenated GRIB2 stream the rest of the pipeline can ingest.
//! - [`fmt`] — human-readable formatters for durations / fuel / distances /
//!   PSO-vs-benchmark deltas, shared by the GUI's stats panel and the CLI's
//!   summary output.
//! - [`bounds`] — derive search and bake bounds from a wind map.
//! - [`route`] — `RouteEvolution` (type-erases the const-generic waypoint count)
//!   plus the `waypoint_match!` / `route_evolution_match!` dispatch macros.
//! - [`search`] — pure, blocking entry points for the PSO and time-reopt loops.
//! - [`solution`] — persistence schema for a single search result.
//! - [`metrics`] — per-segment fuel / time / speed breakdown.
//! - [`landmass`] — Natural Earth landmass grid and A* sea-path finder.
//!
//! Most consumers can reach for the re-exports at this crate's root; specialised
//! types (e.g. error enums, low-level landmass primitives) are accessible via
//! `bywind::<module>::*`.

use serde::{Deserialize, Serialize};

// Modules.
pub mod auto_bounds;
#[cfg(not(target_arch = "wasm32"))]
pub mod baked_codec;
pub mod bounds;
pub mod config;
#[cfg(not(target_arch = "wasm32"))]
pub mod fetch;
pub mod fmt;
pub mod grib2;
#[cfg(not(target_arch = "wasm32"))]
pub mod io;
pub mod landmass;
pub mod metrics;
pub mod route;
pub mod scenario;
pub mod search;
pub mod solution;
#[cfg(not(target_arch = "wasm32"))]
pub mod wind_av1;
pub mod wind_map;

// Wind data: the core types most consumers reach for first.
// `BakeBounds` stays under `wind_map::` — it's a specialised parameter
// type only used to call `BakedWindMap::from_timed_map`.
pub use wind_map::{BakedWindMap, GridLayout, TimedWindMap, WindMap};

// I/O: `Grib2Bbox` is a value type used to call the GRIB2 loader, so it
// gets a crate-root re-export. Named distinctly from the canonical
// `LonLatBbox` because its field order is lat-first (matching the GRIB2
// spec) rather than lon-first like the rest of the crate. Error enums
// stay scoped under their owning module (`grib2::LoadError`,
// `wind_av1::DecodeError`, `wind_av1::EncodeError`,
// `solution::LoadError`, `baked_codec::DecodeError`,
// `baked_codec::EncodeError`, `io::LoadError`,
// `config::ValidationError`) — the bare verb-level names read cleanly
// when fully qualified and avoid crate-root collisions between modules
// that all need to define `LoadError` / `DecodeError` etc.
pub use grib2::Grib2Bbox;

// Bounds and search: build a routing problem and solve it.
pub use auto_bounds::{derive_route_bbox, format_bbox_flag};
pub use bounds::MapBounds;
// Re-export the `swarmkit-sailing` types that appear in `bywind`'s public
// API surface (parameters / return types / public fields), so the surface
// reads as a single unified crate even though the sailing-PSO is its own
// publishable sibling. The two ship in lockstep — bywind pins a specific
// minor version of swarmkit-sailing and bumps along with it. Consumers
// who want to use the underlying sailing API directly can still depend
// on `swarmkit-sailing` themselves; the types are identical (`pub use`
// preserves type identity).
//
// - `Boat`, `SearchSettings` — passed to `run_search_blocking` / built
//   from `BoatConfig::to_boat` / `SearchConfig::to_search_settings`.
// - `RouteBounds`, `LonLatBbox` — search domain types reached through
//   `MapBounds`.
// - `Topology` — outer-PSO topology choice (field on `SearchConfig`).
//   Marked `#[non_exhaustive]` upstream so adding variants is a patch
//   change for both swarmkit-sailing and bywind.
pub use swarmkit_sailing::{
    Boat, DEFAULT_STEP_DISTANCE_FRACTION, RouteBounds, SearchSettings, Topology,
    spherical::LonLatBbox,
};
// `DEFAULT_FRAME_STEP_SECONDS` is a niche default; consumers that need it
// reach for `config::DEFAULT_FRAME_STEP_SECONDS`.
pub use config::{BoatConfig, GenerateConfig, SearchConfig};
pub use search::{
    BAKE_STEP, SearchError, SearchResult, SearchWeights, run_search_blocking,
    run_search_blocking_with_baked, run_time_reopt_blocking,
};

// Search results: the type-erased evolution wrapper plus its fixed waypoint
// counts; metrics for displaying per-segment breakdowns; benchmark route from
// the A* + time-PSO baseline.
pub use metrics::{SegmentMetrics, compute_segment_metrics, gbest_segment_metrics};
// `GbestView` / `GbestViewMut` stay under `route::` — they're borrowed
// views into a `RouteEvolution`, not types most consumers name directly.
pub use route::{BenchmarkRoute, RouteEvolution, WaypointCount};

// Persistence: serialisable schema for a single solution. The
// `LoadError` enum stays scoped at `solution::LoadError` — see the
// note above the `Grib2Bbox` re-export.
pub use solution::SavedSolution;

// Landmass: A* sea-path support. `landmass_grid()` returns the
// `SDF_RESOLUTION_DEG = 0.5°` default grid; `landmass_grid_at_resolution`
// returns a grid at any caller-chosen cell size (cached per distinct
// resolution). Supporting types (`LandmassGrid`, `Polygon`,
// `raw_polygons`) live under `landmass::` for callers that need them.
pub use landmass::{SDF_RESOLUTION_DEG, landmass_grid, landmass_grid_at_resolution};

/// A single wind measurement: speed plus the direction it's coming from.
///
/// `speed` is in knots (meteorological convention — same units the GRIB2
/// surface-wind path delivers; converted to m/s at bake time when the
/// sailing physics needs it). `direction` is in degrees compass
/// `0..=360`, *from*-bearing: `0` is wind blowing from the north, `90`
/// from the east, and so on. Wraparound at the `0`/`360` boundary is
/// handled by interpolating sin/cos components separately, so
/// [`WindMap::query`] returns continuous direction values across a
/// sample boundary that crosses true-north.
#[derive(Clone, Serialize, Deserialize, PartialEq, Debug)]
pub struct WindSample {
    pub speed: f32,
    pub direction: f32,
}

/// A single wind sample tagged with its `(lon, lat)` position.
///
/// Derives `Serialize` / `Deserialize` into the natural named-field
/// shape (`{"lon": …, "lat": …, "sample": {"speed": …, "direction": …}}`)
/// for any JSON / TOML / bincode consumer.
#[derive(Clone, Serialize, Deserialize, PartialEq, Debug)]
pub struct WeatherRow {
    /// Longitude in degrees (`-180..=180`, antimeridian-aware via
    /// `swarmkit_sailing::spherical::signed_lon_delta`).
    pub lon: f32,
    /// Latitude in degrees, clamped strictly inside
    /// `POLE_LATITUDE_LIMIT_DEG` (89.99°) so bearing math stays defined.
    pub lat: f32,
    pub sample: WindSample,
}

/// A [`WeatherRow`] tagged with a wall-clock time in seconds, used as the
/// row-level representation when serialising / deserialising a
/// [`TimedWindMap`] as a stream of samples.
#[derive(Clone, Serialize, Deserialize, PartialEq, Debug)]
pub struct TimedWeatherRow {
    pub lon: f32,
    pub lat: f32,
    pub t_seconds: f32,
    pub sample: WindSample,
}

#[cfg(test)]
mod tests;
