# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace layout

Cargo workspace with four members:

- **`.`** (`bywind`) — core library. Wind-data types, GRIB2 / `wind_av1` I/O, search entry points, landmass A* sea-path, route bookkeeping, persistence schema.
- **`bywind-cli/`** — `bywind-cli` binary: `search`, `convert`, `info`, `inspect`, `tune-trial`, `tune` subcommands.
- **`bywind-viz/`** — egui-based GUI editor / search visualiser.
- **`swarmkit-sailing/`** — sailing-route PSO physics layer (spherical math, boat traits, movers, init, segment-range cache). Built on the generic [`swarmkit`](https://crates.io/crates/swarmkit) PSO library (pulled from crates.io). Lives in this workspace rather than swarmkit's because `bywind` is its only real consumer and it churns with the application.
- **`bywind-dev/`** — internal diagnostic / benchmarking binaries. Excluded from `default-members` so plain `cargo build` / `cargo test` / `cargo clippy` skip it; invoke explicitly with `cargo run -p bywind-dev --release --bin <name>`. Current bins: `av1_round_trip` (encode → decode → drift check on a GRIB2), `wcav_drift` (compare an existing `.wcav` against its source GRIB2, bucketed by source wind speed).

Workspace-shared metadata (`version`, `edition`, `rust-version`, `authors`, `license`, `repository`, `homepage`) lives in `[workspace.package]`; common deps (`serde`, `rand`, `rayon`, `swarmkit`, …) in `[workspace.dependencies]`. Each subcrate inherits with `.workspace = true`.

## Commands

```bash
cargo build --workspace          # Compile everything
cargo test --workspace           # Run all tests
cargo test <name>                # Run a single test by name
cargo clippy --workspace --all-targets
cargo fmt --all
```

## Project

`bywind` (2024 edition, MSRV 1.85) is a sailing-route optimiser that runs a particle-swarm search for a fuel-vs-time-optimal route over real wind data. It decodes GRIB2 weather files, bakes the time-varying wind into a regular grid, and avoids landmass via an embedded Natural Earth coastline rasterised into a signed-distance field with a precomputed A*-able sea grid.

The core library is headless; the GUI (`bywind-viz`) and CLI (`bywind-cli`) are thin frontends.

## Architecture

### Core data types (`src/lib.rs`)
- `WindSample` — `speed: f32` (knots, meteorological from-bearing convention) and `direction: f32` (degrees `0..=360`).
- `WeatherRow` — a `WindSample` tagged with `(lon, lat)`. Derives `Serialize` / `Deserialize` (natural named-field shape).
- `TimedWeatherRow` — a `WeatherRow` plus `t_seconds`, the row-level form for streaming a `TimedWindMap` through serde.

### Wind data (`src/wind_map.rs`)
- `WindMap` — one frame; `Vec<WeatherRow>` plus a private `SpatialIndex` enum. `WindMap::new(rows)` detects whether the points form a complete uniform grid and stores `(origin, step, nx, ny)` for O(1) cell lookup; otherwise falls back to `kiddo::ImmutableKdTree<f32, 2>`. The grid path exists because kiddo collapses on the colinear-axis case past a few hundred collisions.
- `WindMap::query(lon, lat) -> WindSample` — IDW (power 2) over the 4 nearest neighbours. Direction is interpolated as a circular quantity via sin/cos decomposition so the `0°`/`360°` boundary doesn't cause a discontinuity. Exact hits short-circuit to avoid division by zero. `wrap_lon_query` canonicalises queries into `(−180, 180]` before indexing so antimeridian-crossing bake grids hit the right cell.
- `TimedWindMap` — a stack of `WindMap` frames at a fixed `step_seconds`.
- `BakedWindMap` — a search-side regular grid with Cartesian `(u, v)` wind, built once per search.

### Modules (re-exports flagged in `src/lib.rs`'s `//!`)
- `grib2` — GRIB2 loader (`grib` crate, png-unpack feature only). `Grib2Bbox` is lat-first per the WMO spec.
- `wind_av1` — AV1 near-lossless wind-map format (`.wcav`) for fast restart-from-cache and the bundled-sample artifact. Pure-Rust rav1e / rav1d on both ends. Native-only (`cfg(not(target_arch = "wasm32"))`).
- `baked_codec` — companion binary format for `BakedWindMap` (zstd-compressed). Native-only.
- `io` — extension-dispatched `load(...)` that picks between GRIB2 and `wind_av1`. Native-only.
- `bounds`, `auto_bounds` — `MapBounds` and `derive_route_bbox` (A*-probed bbox that detours around continents).
- `ensemble` — `TimedEnsembleWindMap`, `BakedEnsembleWindMap`. K-member wind data loaded from a `.wcav` directory (GEFS naming: `gec00` control + `gepNN` perturbations). See "Ensemble feature" below for vocabulary.
- `search` — `run_search_blocking`, `run_search_blocking_with_baked`, `run_time_reopt_blocking`, `run_realizations`. Types: `SearchResult`, `SearchWeights`, `WindInput`, `EnsembleSpread`, `MemberMetrics`, `RealizationRun`. `BAKE_STEP`.
- `route` — `RouteEvolution` (type-erases the const-generic waypoint count), `BenchmarkRoute`, `WaypointCount`. `GbestView` / `GbestViewMut` stay scoped under `route::`.
- `metrics` — per-segment fuel / time / speed / land breakdown.
- `landmass` — Natural Earth landmass grid (`assets/ne_50m_land.geojson`, embedded) and A* sea-path finder. Cell size is configurable per call via `landmass_grid_at_resolution(deg)` (cached in a per-resolution registry behind a Mutex); the no-arg `landmass_grid()` returns the cached `SDF_RESOLUTION_DEG = 0.5°` default. Consumer-side, `SearchConfig::sdf_resolution_deg` (with UI / TOML plumbing) flows down to `run_search_blocking`'s `sdf_resolution_deg` argument.
- `solution` — `SavedSolution` JSON schema for a single search result. Carries `Option<SavedEnsemble>` (paired `EnsembleMode` + `EnsembleSpread`) for ensemble-sourced solutions; absent for single-deterministic.
- `scenario`, `config` — `BoatConfig`, `SearchConfig`, `SearchWeights`, `CliConfigFile`, `EnsembleMode`. Layer TOML files + CLI overrides.
- `fmt` — shared human-readable formatters (durations / fuel / distances / PSO-vs-benchmark deltas).

### Ensemble feature

The optimiser can run against a multi-member ensemble (GEFS perturbations: `gec00` + `gep01..gep30`) rather than a single deterministic wind map. The canonical vocabulary — used in types, fields, UI labels, and commit messages:

- **Ensemble** — collection of K wind realizations. Loaded from a directory of `.wcav` files via `TimedEnsembleWindMap::load_dir`; baked into a `BakedEnsembleWindMap` whose members share grid geometry.
- **Member** — one wind dataset inside an ensemble. Indexed by `k`, stably ordered by filename so the member-index ↔ file mapping is reproducible across runs.
- **Realization** — one member viewed as a self-contained weather hypothesis ("if member k were the true future wind"). Identity mapping from member → realization; we use "member" when talking about the data and "realization" when talking about the role it plays in a search.
- **`EnsembleMode`** — aggregation strategy. `Full` runs K-fold robust fitness via `RobustObjective::Mean` (every particle scored against all K members); `FastMean` collapses the ensemble to a single mean wind map (`BakedEnsembleWindMap::mean`) and runs the existing single-deterministic search against it. Fast-mean is ~K× faster but `E[f(x, wind)] ≠ f(x, E[wind])` — for routes in the linear-polar regime the gap is typically <1%.
- **Main search** — the primary PSO search against the whole ensemble under an `EnsembleMode`. Produces one converged gbest route. "main" is the canonical adjective in types, comments, and UI.
- **`EnsembleSpread`** — per-member evaluation of the main gbest path: *same route, K winds*. Held in `SearchResult::ensemble: Option<EnsembleSpread>`. Narrow spread ⇒ robust route; wide spread ⇒ brittle. Per-member time-reopt is required (see the doc comment in `src/search.rs`) so the time axis doesn't collapse to `sum(gbest.t)` across members.
- **`RealizationRun`** — output of an *independent* PSO search against one member (`bywind::run_realizations` runs K of these). K decoupled routes, displayed as a translucent overlay in the viz for visual comparison against the main gbest. Distinct from spread: spread evaluates *the main* path against members; realization runs produce *K independent* paths.

`SavedSolution` persists ensemble runs via `Option<SavedEnsemble> { mode, spread }`; single-deterministic saves keep the field omitted via `#[serde(skip_serializing_if = "Option::is_none")]` so the schema stays backward-compatible.

Detail-level data-path documentation (load / bake / mean) lives in `src/ensemble.rs`'s module-level `//!` block.

### Public-surface re-exports from `swarmkit-sailing`
`bywind` re-exports `Boat`, `SearchSettings`, `RouteBounds`, `LonLatBbox`, `Topology` from `swarmkit-sailing` at its crate root, so consumers can drive the search without depending on `swarmkit-sailing` directly. The two ship in lockstep (pinned minor); `pub use` preserves type identity for callers that do want both.

### Error enums
Each module that can fail keeps its own scoped `LoadError` / `DecodeError` / `EncodeError` (`grib2::LoadError`, `wind_av1::DecodeError`, etc.). They stay module-scoped — the bare verb-level names read cleanly when fully qualified and avoid root-level collisions.

### Coordinate convention (cross-cutting, bywind + swarmkit-sailing + bywind-viz)
- Positions are `(lon°, lat°)` as `Vector2<f64>` with `.x = lon`, `.y = lat`.
- Distances are real ground metres on the sphere — haversine in `swarmkit-sailing/src/spherical.rs`.
- Bearings are radians, compass convention (`0 = N`, clockwise).
- Wind vectors are local east-north tangent: `.x = u_east`, `.y = v_north` m/s.
- Earth-radius / metres-per-degree constants live in `swarmkit_sailing::spherical` (`EARTH_RADIUS_M = 6_371_000`).
- Antimeridian: longitude deltas go through `signed_lon_delta`, not raw subtraction. Latitude is clamped inside `POLE_LATITUDE_LIMIT_DEG` (89.99°) so bearing math stays defined.

### Tests
Unit tests live alongside each module (`#[cfg(test)] mod tests` blocks) plus the crate-root `src/tests/mod.rs`. Two foundational ones at the root:
- `round_trip_serialization` — `Vec<WeatherRow>` round-trip through `serde_json`.
- `interpolation_midpoint` — `WindMap::query` at the midpoint of two samples matches the expected average.
