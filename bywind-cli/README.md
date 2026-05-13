# bywind-cli

Command-line frontend for the
[`bywind`](https://crates.io/crates/bywind) sailing-route optimiser.

## Install

```sh
cargo install bywind-cli
```

This installs a single binary, `bywind-cli`. Verify with
`bywind-cli --help` to see the subcommand list.

## Subcommands

### `search`

Run a sailing search and emit a `SavedSolution` JSON. The hard-required
inputs (`<map>`, `--start`, `--end`) can come either as flags or from a
`[run]` section in a `--config` TOML file.

```sh
bywind-cli search forecast.grib2 \
    --start -73.95,40.75 \
    --end   -9.13,38.71 \
    --waypoints 20 \
    --time-weight 1 --fuel-weight 10 --land-weight 40 \
    -o solution.json
```

Multiple `--config` files merge left-to-right; CLI flags override the
merged result. See `--help` for every flag.

### `fetch`

Pull a wind-map window from NOAA's public GFS S3 bucket and write it
directly to disk. Output format is inferred from the `--out` extension:
`.grib2` streams the concatenated GFS messages straight through; `.wcav`
stages a temporary GRIB2 and re-encodes it as AV1 near-lossless so the
final artifact is small enough to ship.

```sh
bywind-cli fetch 20260301 2026030107 --out window.wcav
```

`<start>` and `<end>` are UTC, format `YYYYMMDD` (hour defaults to 00z)
or `YYYYMMDDHH`. `<start>` must be a GFS cycle hour (00, 06, 12, or 18).
`--interval-h N` controls the cadence (1, 2, 3, or 6 hours; default 1
uses each cycle's `f000`..`f005` short-range forecasts for seamless
1-hour cadence). Only the UGRD / VGRD-at-10m messages are fetched via
HTTP Range requests, so total transfer is a couple of MB per frame.

### `convert`

GRIB2 → `wind_av1` (`.wcav`) conversion. Used to bake slow-to-parse
GRIB2 files into the compact AV1 near-lossless binary format so
subsequent searches over the same map skip the (often multi-second)
GRIB2 decode.

```sh
bywind-cli convert forecast.grib2 -o forecast.wcav
```

`--grib-stride N` decimates the lat/lon grid (every Nth row + col);
`--grib-bbox lat_min,lon_min,lat_max,lon_max` clips to a region of
interest before decoding.

### `info`

Print summary metadata about a wind map — frame count, time step,
extent, sample count, projected grid dimensions.

```sh
bywind-cli info forecast.grib2
```

Works on any supported format.

### `inspect`

Print metadata for a saved solution. With `--map`, also re-scores the
solution's gbest path against that wind map and prints a per-segment
breakdown of fuel / time / speed / land.

```sh
bywind-cli inspect solution.json --map forecast.grib2
```

### `tune-trial`

Per-trial worker for the PSO-tuning study. Reads a JSON trial spec
(params + seeds + routes) from stdin, runs the search across
`routes × seeds`, and writes aggregate JSON to stdout. Designed for
spawning by `tune` rather than direct invocation; see
`src/tune_trial.rs` for the schema.

### `tune`

Drives a TPE Bayesian optimisation over the PSO coefficient space by
spawning `tune-trial` as a subprocess per trial. Persists trials as
JSONL so an interrupted study leaves an inspectable record. Run a
baseline pass first via `tune-trial` with default coefficients, then
pass the resulting JSON via `--baseline`.

## Exit codes

| Code | Class | Meaning |
| --- | --- | --- |
| 0 | success | search completed |
| 1 | `BadInput` | CLI / config / file-parse error |
| 2 | `NoResult` | search ran but produced no usable route (landlocked endpoints, empty wind map after filters, etc.) |
| 3 | `Internal` | unexpected error in bywind — bug |

Subcommands return `Result<(), AppError>` where the variants carry the
exit-code class; unannotated `?` from `anyhow` contexts auto-converts
to `BadInput`.

## Features

- `profile-timers` — forwards through to `bywind` / `swarmkit-sailing`.
  Enables sub-stage `Instant::now` counters in the search hot paths;
  emits a per-stage breakdown to stderr at the end of every search.

## License

Dual-licensed under either of

- Apache License, Version 2.0 ([`LICENSE-APACHE`](./LICENSE-APACHE))
- MIT license ([`LICENSE-MIT`](./LICENSE-MIT))

at your option.
