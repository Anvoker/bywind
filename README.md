# bywind

Naval pathfinding with a focus on sailboats exploiting winds.

`bywind` is research into applying Particle Swarm Optimization (PSO) to naval navigation and a practical tool for visualising how sailboats perform across global routes over real wind data. It decodes GRIB2 weather files, bakes them into a regular spatial grid for performance, then runs a PSO search for a fuel-vs-time-optimal route between two points. Landmass avoidance comes from an embedded Natural Earth coastline dataset rasterised into a signed-distance field and pre-A\*-d for a fast sea-path baseline.

The crate itself is a headless engine — it does nothing on its own. To actually use it you'll want one of the frontends:

- The GUI editor: [`bywind-viz`](https://crates.io/crates/bywind-viz)
- The CLI: [`bywind-cli`](https://crates.io/crates/bywind-cli)

![bywind-viz screenshot](https://raw.githubusercontent.com/Anvoker/bywind/main/bywind-viz/docs/screenshot.png)

## Features

- Pull wind data from NOAA's public GFS S3 bucket, or load your own GRIB2 files.
- Visualise and manually edit wind maps and routes.
- Save wind data as `.wcav`, bywind's AV1 near-lossless binary format — a subset of GRIB2 data at upwards of 30× compression.
- Save and load scenarios and search configuration.
- Derives a sensible search domain from origin and destination.

### PSO search

- Supports gbest, niched, ring, and von Neumann swarm topologies.
- Search parameters fully customisable from both the GUI and the CLI.
- Seeds the swarm from a perturbed A\* path plus several families of route shapes.
- Landmass representation and avoidance via fitness penalties.

### Ensembles

Real scenarios involve weather-forecast uncertainty. A route optimised against a single forecast can perform poorly when the actual weather deviates from it.

To address this you can run an ensemble search, where the fitness function is evaluated over multiple forecasts at once. Currently it uses one main forecast and several probable perturbed forecasts published by NOAA. The result is a route that is much more robust to real-world conditions than one tuned to a single prediction.

## Known issues / limitations

- The search can misbehave around some straits and fine coastal features because the underlying landmass grid is coarser than what the user sees. A post-processing step could fix this.

## Install

### Prebuilt `bywind-viz` binaries

Download from the [latest release page](https://github.com/Anvoker/bywind/releases/latest).

### CLI and GUI via `cargo install`

```sh
cargo install bywind-cli
cargo install bywind-viz
```

### Prebuilt `bywind-viz` binaries via install script

**Shell script:**

```sh
curl --proto '=https' --tlsv1.2 -LsSf https://github.com/Anvoker/bywind/releases/latest/download/bywind-viz-installer.sh | sh
```

**PowerShell script:**

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://github.com/Anvoker/bywind/releases/latest/download/bywind-viz-installer.ps1 | iex"
```

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

    // Size the bbox around a land-avoiding A* path rather than the
    // straight great-circle line, then derive the bake grid and route
    // bounds from it in one step.
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
        None, // fine_sdf_resolution_deg — `None` = single-tier landmass
        &mut |_| {}, // ignore in-flight progress events; the final result is enough
    )?;

    Ok(())
}
```

For a complete walkthrough — wind-map loading, error handling, summary output — see `bywind-cli`'s `search.rs`. For a graphical view of the swarm's evolution, see `bywind-viz`.

## License

Dual-licensed under either of

- Apache License, Version 2.0 ([`LICENSE-APACHE`](./LICENSE-APACHE))
- MIT license ([`LICENSE-MIT`](./LICENSE-MIT))

at your option.
