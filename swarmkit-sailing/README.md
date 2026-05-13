# swarmkit-sailing

Sailing-route PSO physics layer built on top of the
[`swarmkit`](https://crates.io/crates/swarmkit) particle-swarm library.

## What this crate provides

- **Spherical-Earth coordinates and bearing math.** `LatLon`,
  `LonLatBbox`, `Segment`, `TangentMetres`, `Wind`. Haversine
  distance, signed lon delta, tangent-frame ↔ `(Δlon, Δlat)`
  conversions.
- **Boat physics.** A `Sailboat` trait modelling fuel consumption and
  segment travel time as a function of (origin, destination,
  departure time, MCR throttle); a `Boat` impl with polar-curve and
  fuel-rate parameters. `WindSource` and `LandmassSource` traits for
  pluggable environment.
- **PSO movers.** `SphericalPSOMover` does tangent-frame velocity
  updates so the swarm explores in real ground metres rather than
  raw `(Δlon, Δlat)` (which compresses east-west motion at high
  latitudes). Plus `CauchyKickMover` / `ShapeKickMover` for the
  diversity-preserving path-level kicks.
- **Init strategies.** `PathInit` seeds the outer swarm across a
  configurable mixture of shape families (sin-k1, sin-k2, anchor,
  gaussian, …) over multiple baselines (straight line, north-biased
  polyline, south-biased polyline) so the search starts with both
  shape and topology diversity.
- **Per-segment cache.** `SegmentRangeTables` precomputes a 2D
  (departure-time × MCR) travel-time grid per segment so the inner
  time-PSO replaces wind integrations with O(1) interpolated lookups.
- **Top-level entry points.** `search` chains all of the above,
  dispatched by `Topology` (`GBest`, `Niched`, `Ring`, `VonNeumann`).
  `reoptimize_times` runs only the inner time-PSO over a fixed `xy`
  path, returning a `Path<N>` with updated departure times — used by
  the GUI's after-drag re-fit and any "freeze the route, retime the
  legs" workflow.

## Use this directly only if

…you're integrating sailing-route PSO into something other than
[`bywind`](https://crates.io/crates/bywind). For the common case —
load a wind map, run a search, get a route — reach for `bywind`
instead; it composes this crate's primitives behind a higher-level
API and provides the GRIB2 / `wind_codec` / landmass plumbing.

## Install

```toml
[dependencies]
swarmkit-sailing = "0.1"
```

## Features

- `probe-stats` — atomic counters inside the segment-range cache for
  diagnostics (how often does a query fall outside the tabulated
  range?). Off by default; adds atomic traffic on the query hot
  path.
- `profile-timers` — sub-stage `Instant::now` counters around the
  hot paths (mover, boundary, fitness, segment-cache build). Used by
  `bywind/profiling/`. Off by default.

## License

Dual-licensed under either of

- Apache License, Version 2.0 ([`LICENSE-APACHE`](./LICENSE-APACHE))
- MIT license ([`LICENSE-MIT`](./LICENSE-MIT))

at your option.
