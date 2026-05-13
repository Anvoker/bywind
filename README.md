# bywind

Sailing-route optimisation over real wind data.

`bywind` decodes GRIB2 weather files (UGRD/VGRD at 10 m above ground),
bakes the time-varying wind into a regular spatial grid, then runs a
particle-swarm search for a fuel-vs-time-optimal route between two
points. Landmass avoidance comes from an embedded Natural Earth
coastline dataset rasterised into a signed-distance field and pre-A*-d
for fast sea-path baseline construction.

The crate is the headless engine. There's a binary CLI
([`bywind-cli`](https://crates.io/crates/bywind-cli)) and an egui-based
GUI editor ([`bywind-viz`](https://crates.io/crates/bywind-viz)) that
build on top of it.

## Install

```toml
[dependencies]
bywind = "0.1"
```

## What's in the crate

- **Wind data.** `WindMap` (one frame, AoS over `(lon, lat, sample)`),
  `TimedWindMap` (a stack of frames at a fixed step), `BakedWindMap`
  (a search-side regular grid with Cartesian `(u, v)` wind, built
  once per search).
- **I/O.** `bywind::io::load` auto-dispatches by extension between
  GRIB2 (read-only) and `wind_av1` (`.wcav`, the crate's AV1
  near-lossless binary format for fast restart-from-cache and the
  bundled GUI sample).
- **GFS fetch.** `bywind::fetch::fetch_to_grib2` pulls
  UGRD/VGRD-at-10m messages from NOAA's public GFS S3 bucket via
  `.idx` sidecars + HTTP Range requests, producing a concatenated
  GRIB2 stream the rest of the pipeline ingests unchanged. A couple
  of MB per frame instead of the ~500 MB full-file size. Driven by
  the CLI's `bywind-cli fetch` subcommand; usable directly for
  custom batch / scheduled-download workflows.
- **Search entry points.** `run_search_blocking` runs the full
  outer-position + inner-time PSO and returns the gbest route plus an
  A*+time-PSO benchmark for context. `run_time_reopt_blocking` runs
  only the time-PSO holding a path's xy fixed (the GUI uses it after
  drag-edits).
- **Config schemas.** `BoatConfig` (polar / fuel rates), `SearchConfig`
  (PSO sizing + coefficients), `SearchWeights` (time / fuel / land
  trade-off). All serde-derived; `bywind::scenario::CliConfigFile`
  layers TOML files + CLI overrides for the CLI / GUI.
- **Bounds derivation.** `MapBounds::from_wind_map` plus
  `derive_route_bbox` (an A*-probed bbox that detours around
  continents) for the "I just have origin + destination, give me a
  sensible search domain" case.
- **Landmass.** `landmass_grid()` returns the lazily-initialised
  default Natural Earth grid (`SDF_RESOLUTION_DEG = 0.5°`, embedded as
  `assets/ne_50m_land.geojson`). `landmass_grid_at_resolution(deg)`
  returns a grid at a caller-chosen cell size, cached per distinct
  resolution; `SearchConfig::sdf_resolution_deg` plumbs that through
  the search.
- **`swarmkit-sailing` re-exports.** `Boat`, `SearchSettings`,
  `RouteBounds`, `LonLatBbox`, `Topology` are re-exported at the
  crate root so the common case (load wind, run search) needs only a
  `bywind` dependency. The two crates ship in lockstep; `pub use`
  preserves type identity for callers that do also depend on
  `swarmkit-sailing` directly.

## Minimal usage

```rust
use std::path::Path;
use bywind::{
    BoatConfig, SearchConfig, SearchWeights, SearchResult,
    WaypointCount, BAKE_STEP, SDF_RESOLUTION_DEG,
    run_search_blocking, derive_route_bbox, landmass_grid,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // NYC → Lisbon, transatlantic.
    let origin = (-73.95, 40.75);
    let destination = (-9.13, 38.71);

    // GRIB2 or `.wcav`, dispatched by extension.
    let wind_map = bywind::io::load(Path::new("forecast.grib2"), 1, None)?;

    // A*-probed bbox that detours around continents, then derive the
    // bake grid and route bounds from it in one step.
    let map_bounds = derive_route_bbox(origin, destination, landmass_grid(), None)
        .ok_or("endpoints too close to derive a useful bbox")?;
    let route_bounds = map_bounds.to_route_bounds(origin, destination);
    let bake_bounds = map_bounds.to_bake_bounds(BAKE_STEP);

    let boat = BoatConfig::default();
    let search_cfg = SearchConfig::default();
    let weights = SearchWeights { time_weight: 1.0, fuel_weight: 10.0, land_weight: 1.0 };

    let SearchResult {
        route_evolution,
        benchmark,
        bake_duration,
        search_duration,
        ..
    } = run_search_blocking(
        &wind_map,
        bake_bounds,
        route_bounds,
        WaypointCount::N10,
        search_cfg.to_search_settings(),
        boat.to_boat(),
        weights,
        SDF_RESOLUTION_DEG,
    )?;

    Ok(())
}
```

For a complete walkthrough — wind-map loading, error handling, summary
output — see `bywind-cli`'s `search.rs`. For a graphical view of the
swarm's evolution, see `bywind-viz`.

## Features

- `profile-timers` — forwards through to `swarmkit-sailing` and turns
  on sub-stage `Instant::now` counters in the search hot paths.
  Default off (atomic-add traffic at every call site).

## License

Dual-licensed under either of

- Apache License, Version 2.0 ([`LICENSE-APACHE`](./LICENSE-APACHE))
- MIT license ([`LICENSE-MIT`](./LICENSE-MIT))

at your option.
