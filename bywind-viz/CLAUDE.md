# bywind-viz

## Overview

`bywind-viz` is the GUI front-end for the [`bywind`](../bywind) library. It uses [egui](https://github.com/emilk/egui) via [eframe](https://github.com/emilk/egui/tree/master/crates/eframe) to provide a visual editor for wind data. The application is titled "Bywind" in the UI.

The app is `BywindApp` (`src/app.rs`); it holds an `Option<TimedWindMap>` plus grouped substructs for editor / view / generate / search / boat configuration and search outputs. Each substruct has `Default`s wired up so `serde(default)` covers schema additions on persisted state. Background search and time-reopt run via the `AsyncJob<T>` channel wrapper in `async_job.rs`.

## Relation to `bywind`

This crate depends directly on `bywind` via a path dependency (`../bywind`). All wind data types and logic live in `bywind`; this crate only handles UI concerns. When working on wind data structures, serialization, or spatial queries, make changes in `../bywind`. When working on how that data is displayed or edited, make changes here.

Key types from `bywind` used here:
- `TimedWindMap` — the main data structure held by `BywindApp`. A sequence of `WindMap` frames separated by `step_seconds`; queried through `frame()` / `frame_mut()` / `frame_count()` / `step_seconds()`. Generated synthetically via `TimedWindMap::generate_random` / `generate`, or loaded from GRIB2 / `wind_av1`.
- `WindMap` — a single time slice: a spatial grid of wind samples queryable by position. Tools mutate it through `frame.query_circle_indices(...)` + `frame.set_sample(...)`.
- `WeatherRow` — a single grid point with `(x, y)` position and a `WindSample`. **Coordinate convention: `x = lon°`, `y = lat°` natively** — the previous equirectangular projection layer was removed in favour of operating directly in `(lon, lat)` and projecting only at the view layer (`view::ViewTransform::map_to_screen`). The field names are kept for diff size; rename in a follow-up if desired.
- `WindSample` — `speed: f32` and `direction: f32` (degrees, compass from-bearing)

## Coordinate convention (cross-cutting)

Across this crate, `bywind`, and `swarmkit-sailing`:

- Positions are `(lon°, lat°)` carried as `Vector2<f64>` with `.x = lon`, `.y = lat`.
- Distances are real ground metres on the sphere — haversine in `swarmkit-sailing/src/spherical.rs`.
- Bearings are stored as **radians**, compass convention (`0 = N`, `π/2 = E`, clockwise).
- Wind vectors are `Vector2<f64>` in the local east-north tangent frame: `.x = u_east_m_per_s`, `.y = v_north_m_per_s`.
- The PSO operates on `(lon, lat)` directly and uses `SphericalGBestMover` (in `swarmkit-sailing/src/spherical_pso.rs`) to do velocity updates in the local east-north tangent frame at each waypoint, so swarm exploration is unbiased on the sphere. `step_distance_max` and `RouteBounds.step_distance_max` are in real ground metres.
- Earth-radius / metres-per-degree constants live in `swarmkit-sailing::spherical` (`EARTH_RADIUS_M = 6_371_000`, `METRES_PER_DEGREE = π·R/180`). The view layer re-derives `METRES_PER_DEGREE` from the same value to stay consistent.
- Antimeridian: longitude differences must go through `signed_lon_delta`, not raw subtraction. Latitude is clamped strictly inside `POLE_LATITUDE_LIMIT_DEG` (89.99°) so bearing math stays defined.
- The view layer applies an equirectangular projection at the loaded wind map's bbox centre for display only. This projection is lossy and does not feed back into the search.

`WindMap` supports:
- `generate(size_x, size_y, density)` — create a uniform grid with zero-initialized samples
- `new(rows)` — construct from arbitrary `Vec<WeatherRow>`
- `query(x, y)` — IDW-interpolated `WindSample` at any point
- `query_circle(x, y, radius)` / `query_circle_indices(...)` — spatial range queries
- `set_sample(index, speed, direction)` — mutate a sample by index

`WeatherRow` / `TimedWeatherRow` derive `serde::{Serialize, Deserialize}`; consumers pick whatever serde format they like (the GUI doesn't directly serialise samples).

## Project Structure

```
src/
  main.rs        — native entry point
  app.rs         — BywindApp: eframe::App impl, holds Option<TimedWindMap> and grouped substructs
  config.rs      — Tool enum + persisted UI substructs (EditorState, ViewState, GenerateConfig, SearchConfig, BoatConfig)
  view.rs        — ViewTransform: equirectangular projection between (lon°, lat°) and screen pixels
  ui.rs          — top/bottom/side panel renderers and the GRIB2-load / error-toast modals
  tools.rs       — central-panel input dispatch (Pointer / Speed / Direction / Endpoint / Waypoint Edit / Waypoint Time / Route Bounds)
  draw.rs        — wind-barb / coastline / route-evolution rendering, segment-metric computation
  coastlines.rs  — Natural Earth landmass triangulation, cached at startup
  route.rs       — RouteEvolution: non-generic wrapper hiding the const-generic Path<N> dispatch
  search.rs      — SearchOutputs, search worker + time-reopt worker entry points
  io.rs          — GRIB2 / wind_av1 / saved-solution load and save
  bounds.rs      — MapBounds: turn a TimedWindMap (+ optional user-set bbox/endpoints) into RouteBounds + BakeBounds
  async_job.rs   — AsyncJob<T>: thin wrapper around an mpsc receiver for background work
assets/          — icons, Natural Earth landmass GeoJSON
docs/            — README screenshot
```

## Build & Run

```sh
cargo run
```

## Lints

Lints are defined in `Cargo.toml` under `[workspace.lints]`. `unsafe_code` is denied. The full clippy lint set is strict — address all warnings before committing.
