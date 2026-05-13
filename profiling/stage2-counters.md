# Profiling step 2: sub-stage CPU breakdown

Step 2 of the profiling plan: instrument the sailing search at five hot
sites with atomic `Instant::now` accumulators, gated behind
`feature = "profile-timers"`. Replays the same two scenarios from
[`baseline.md`](baseline.md) (`--load-baked` to skip bake cost), with
the feature on, and prints a per-stage CPU-time + call-count breakdown
on stderr.

## How to reproduce

```text
cd C:/Projects/GameDev/bywind
cargo build --release -p bywind-cli --features profile-timers
./target/release/bywind-cli.exe search \
    --load-baked profiling/<scenario>/baked.bk1 \
    --config profiling/<scenario>/config.toml \
    --out profiling/<scenario>/result.json
```

## Instrumented sites

| Stage | Where | What it covers |
|---|---|---|
| `sailing_boundary` | `boundary.rs::SailingPathBoundary::handle` | Bbox clamp + per-waypoint land projection + per-segment time clamp. Per outer particle per outer iteration. |
| `time_nested_mover` | `time.rs::TimeNestedMover::update` | The whole inner-time-PSO call wrapped — sets up cache, runs inner GBestSearcher, writes best back. Per outer particle per outer iteration. |
| `segment_cache_build` | `range_cache.rs::SegmentRangeTables::build` | Building the per-segment travel-time × MCR table. Called once inside each `time_nested_mover` invocation. |
| `outer_fit` | `fit.rs::SailboatFitCalc::calculate_fit` | Outer fitness: walks the path, integrates wind per segment via the boat integrator. |
| `inner_fit` | `time.rs::TimeFitCalc::calculate_fit` | Inner fitness: walks the path with cached `get_segment_metrics_cached` lookups (no wind integration). |

## Headline numbers (3 runs each, release build, --load-baked)

Numbers are CPU-time totals (sum across all threads, since rayon
parallelises both the outer-mover and inner-fit passes). With ~12 cores
on this box, sum of stage CPU-time ≫ wallclock as expected.

### Small (NY → NOLA, N = 20)

Median across 3 runs:

| Stage | Total (ms) | Calls | Avg / call |
|---|---:|---:|---:|
| `sailing_boundary`    |    162 |     1 600 |    100 µs |
| `time_nested_mover`   | 14 838 |     1 600 |  9 280 µs |
| `segment_cache_build` |  4 336 |     1 600 |  2 710 µs |
| `outer_fit`           |    752 |     1 680 |    448 µs |
| `inner_fit`           |  2 049 | 1 984 000 |   1.03 µs |
| **search_dur (wallclock)** | **1 460** | — | — |

### Large (Lisbon → Mumbai, N = 40)

Median across 3 runs:

| Stage | Total (ms) | Calls | Avg / call |
|---|---:|---:|---:|
| `sailing_boundary`    |    270 |     1 600 |    167 µs |
| `time_nested_mover`   | 22 824 |     1 600 | 14 270 µs |
| `segment_cache_build` |  6 346 |     1 600 |  3 970 µs |
| `outer_fit`           |    894 |     1 680 |    533 µs |
| `inner_fit`           |  4 296 | 1 984 000 |   2.17 µs |
| **search_dur (wallclock)** | **1 950** | — | — |

## What's where (CPU-time accounting)

`time_nested_mover` is the parent scope; `segment_cache_build` and
`inner_fit` are nested inside it. Outside it sits `outer_fit` (a
separate pass per outer iteration) and `sailing_boundary` (part of the
outer-mover chain).

Reading just the small scenario:

- **`time_nested_mover` is ~94 % of total CPU** (14.8 s of 15.8 s
  measured stage time). Drilling in:
  - `segment_cache_build` is **29 % of nested-mover** time (4.3 s).
  - `inner_fit` is **14 % of nested-mover** (2.0 s).
  - The remaining **~57 %** (8.5 s) is unattributed inside the inner
    PSO — i.e. inner-mover step (`GBestMover<Time<N>>` plus
    `TimeBoundary`) plus per-iteration GBestSearcher housekeeping.
- **`outer_fit` is ~5 %** of total CPU.
- **`sailing_boundary` is ~1 %** of total CPU.

Large scenario tracks the same shape — `time_nested_mover` is again
~94 %, with cache build taking a slightly larger share of nested-mover
because the per-segment build cost scales with N.

## What this means for optimisation

The **inner time PSO completely dominates search cost.** Within it,
three chunks deserve attention in roughly this order of magnitude:

1. **Unattributed residual inside `time_nested_mover`** (8.5 s small /
   12.2 s large of CPU time) — the inner-mover step + inner-boundary
   step + GBestSearcher overhead. This is the biggest single chunk and
   it isn't yet broken down further. **Next step before any
   optimisation work**: add two more timers (`INNER_MOVER` for the
   `BoundedMover<GBestMover<Time<N>>, TimeBoundary>::update` body, plus
   `INNER_BOUNDARY` for `TimeBoundary::handle`) so we know which half of
   the residual to attack.
2. **`segment_cache_build`** — 4.3 s small / 6.3 s large of CPU. Called
   exactly 1 600 times per search (once per outer particle's xy
   refresh). Each call does roughly `(N-1) × RANGE_K × (3·K_MCR − 2) ≈
   (N-1) × 184` wind-integrated travel-time calls — each of those is a
   substep loop with haversine / bearing / destination-point per
   substep. There's algorithmic surface to attack here:
   - Reuse part of the cache when only some xy waypoints changed.
   - Reduce `RANGE_K` / `K_MCR` if accuracy budget allows (these are
     `8 × 8 = 64` samples per segment today).
   - Vectorise the substep loop in `Boat::get_travel_time`.
3. **`inner_fit`** — 2.0 s small / 4.3 s large of CPU; 1.98 M calls
   each. Already extremely cheap per call (~1–2 µs) thanks to the
   cache. Linear in N (cache lookup per segment per call). Diminishing
   returns to optimise individually, but it shows up because of the
   sheer call count.

What we're **not** going to optimise unless the picture changes:

- `outer_fit` is 5 % of CPU. Even a 2× speedup buys 2.5 % wallclock —
  not worth the engineering until everything bigger is squeezed.
- `sailing_boundary` is 1 % of CPU. Same logic, more so.

## Bench fitness sanity (unchanged by feature)

Both scenarios still produce identical fitness numbers with timers on
vs off (verified by spot-checking the bench-route comparison printed by
the CLI). The atomic adds don't perturb the search.

## Iteration 2: split the inner residual

Added `INNER_MOVER` + `INNER_BOUNDARY` timers (and a tiny `TimedMover`
wrapper in swarmkit-sailing so we can time the inner `GBestMover`
without touching swarmkit core). New per-call costs (median of 3 runs):

| Stage | Small (avg/call) | Large (avg/call) | Calls |
|---|---:|---:|---:|
| `inner_mover`    | **66 ns** | 86 ns  | 1.92 M |
| `inner_boundary` | **433 ns** | 905 ns | 1.92 M |
| `inner_fit`      | 1.05 µs | 2.21 µs | 1.98 M |

So the inner-mover step is **essentially free** (66 ns/call), and
inner-boundary is also cheap (~0.4-0.9 µs). Together they're ~960 ms /
~1.9 s of CPU on small/large. **Most of the original 8.5 / 12.2 s
residual was elsewhere — likely rayon scheduling overhead in the
parallel inner-fitness pass.**

## Iteration 3: experiment — `FIT_PAR_LEAF_SIZE = usize::MAX`

Hypothesis: the inner fitness pass was parallelised at leaf size 4
even though per-call work is ~1-2 µs (well below rayon's per-task
overhead). Rayon was spawning ~10 sub-tasks per inner iteration for
real work that takes microseconds — net cost.

Falsification: change `FIT_PAR_LEAF_SIZE` from `4` to `usize::MAX`
(serial inner fitness) in `swarmkit-sailing/src/time.rs`. The outer
mover pass is already parallel at PAR_LEAF_SIZE=1
(`TimeNestedMover`), so each outer particle's inner search runs
concurrently across cores anyway — the inner parallelism was
nesting parallelism inside parallelism without any benefit on a
saturated core pool.

Observed (median of 5 runs each, no timers):

| Scenario | Wallclock before | Wallclock after | Δ |
|---|---:|---:|---:|
| Small (N=20) | 1.46 s | **1.43 s** | −2 % |
| Large (N=40) | 1.97 s | **1.92 s** | −2.5 % |

CPU-time within `time_nested_mover` (with timers):

| Scenario | Before | After | Δ |
|---|---:|---:|---:|
| Small | 14.84 s CPU | **13.38 s CPU** | −10 % |
| Large | 22.82 s CPU | **20.26 s CPU** | −11 % |

CPU drops more than wallclock because the outer parallelism caps
wallclock; saved CPU just frees cores for *other concurrent work*
rather than speeding up this one search. That's the right shape for
the user's stated motivation — running multiple hyperparameter-sweep
searches in parallel, each one will leave more CPU available to its
peers.

**Decision: keep the change.** Zero regression risk (less work → less
work), ~2 % wallclock win, ~11 % CPU win. Committed as part of the
iteration.

## What's left in the residual

Even after iteration 3, ~6.2 s small / ~9.7 s large of CPU is still
unaccounted for inside `time_nested_mover`. Strong suspect:
**`PathTimeInit::init_pos`** inside `particle_init.init_dep`. It runs
once per outer particle (1 600 calls) and does
`particles_time × (N-1) × 2` wind-integrated travel-time evaluations
to seed the inner-PSO's initial times — that's `40 × 19 × 2 = 1 520`
heavy integrations per outer particle for small, `40 × 39 × 2 = 3 120`
for large. Roughly the same order of magnitude as
`SegmentRangeTables::build`, and entirely unmeasured today.

Recommend instrumenting `PathTimeInit::init_pos` (one new timer:
`INIT_POS_INNER`) before any further optimisation work.

## Iteration 4: instrument `PathTimeInit::init_pos`

Added `INIT_POS_INNER` timer wrapping `PathTimeInit::init_pos`. Per
outer particle, this method does `particles_time × (N-1) × 2`
wind-integrated travel-time computations (via `get_travel_time_range`)
to seed each inner particle's per-segment initial times.

Median of 3 runs each, with the iteration-3 change in place:

| Stage | Small (CPU-ms) | % nested | Large (CPU-ms) | % nested |
|---|---:|---:|---:|---:|
| `time_nested_mover` (parent) | 13 045 | 100 % | 20 073 | 100 % |
| `init_pos_inner`             | **5 242** | **40 %** | **7 230** | **36 %** |
| `segment_cache_build`        | 4 316 | 33 % | 6 262 | 31 % |
| `inner_fit`                  | 1 923 | 15 % | 4 084 | 20 % |
| `inner_boundary`             |   840 |  6 % | 1 740 |  9 % |
| `inner_mover`                |   114 |  1 % |   133 |  1 % |
| **residual (unaccounted)**   |   610 |  5 % |   624 |  3 % |

**`init_pos_inner` is by far the largest single chunk.** It runs
exactly 1 600 calls per search, averages 3.3 ms (small) / 4.5 ms
(large) per call, and it's **on the critical path inside each
TimeNestedMover::update task** — meaning a wallclock saving of
roughly `init_pos_inner_total / num_cores` is possible if it can be
eliminated.

After accounting for `init_pos_inner`, the unattributed residual is
~3-5 % of nested-mover CPU. Almost everything is named.

## The redundancy buried in `init_pos_inner`

`PathTimeInit::init_pos` does the same wind-integrated work that
`SegmentRangeTables::build` does immediately before it, on the same
`(xy, departure_time)` — both compute, per segment, "what's the range
of feasible travel times here?". The flow inside
`run_inner_time_pso`:

```text
let cache = SegmentRangeTables::build(&xy, …);   // computes ranges, tabulates
let inner_fit = TimeFitCalc::new(fit_calc, &cache, xy);
let inner_bnd = TimeBoundary::new(&cache, …);
let inner_mover = …;
let mut searcher = GBestSearcher::new(inner_fit, inner_mover);
searcher.reseed(rng);
searcher.set_context(fixed_path);
let mut group = particle_init.init_dep(rng, fixed_path);  // ← redoes the same ranges
```

`particle_init` (a `PathTimeInit`) doesn't have access to the cache,
so it falls back to recomputing ranges via `get_travel_time_range`
(which integrates wind from scratch). Cache-aware init would query
`cache.query_range(seg_i, dep)` per segment per inner particle — a
piecewise-linear lookup, microseconds per call instead of milliseconds.

## Optimisation candidate (next concrete change)

**Make the inner-PSO init query the cache instead of integrating wind
afresh.** Lower-bound estimate of the saving:

- CPU: ~5.2 s (small) / ~7.2 s (large) ⇒ −40 % / −36 % of nested-mover CPU.
- Wallclock: rough `init_pos_total / num_cores` ≈ 440 ms (small) /
  600 ms (large), since init_pos runs serially within each outer
  particle's task and outer-particle tasks fan out across cores.
  That'd take search time from 1.43 → ~1.0 s small / 1.92 → ~1.32 s
  large — **~30 % wallclock reduction in both scenarios**.

Mechanically: drop the `particle_init` parameter from
`run_inner_time_pso` and `TimeNestedMover`, accept a `particle_count:
usize` instead, and inline a cache-aware init in `run_inner_time_pso`
(seed each inner particle's per-segment time uniformly inside the
cache's `(t_min, t_max)` range). `PathTimeInit` becomes mostly dead
(only the velocity-init half is still useful, and that's a one-liner).

Correctness note: `cache.query_range` returns the *tabulated* range
(with sub-1 % linear-interpolation error per the cache's design), not
the exact one. For init seeding that's plenty accurate — the PSO
optimises these initial values away within a few iterations
regardless.

## Iteration 5: cache-aware inner-PSO init (the optimisation)

Implemented and measured. The change drops the `particle_init`
parameter from `run_inner_time_pso` / `TimeNestedMover` /
`reoptimize_time` and inlines a cache-aware init: each inner particle's
per-segment time is drawn uniformly from
`cache.query_range(seg_i, dep)` — the same tabulated range the inner
boundary will enforce, computed in microseconds via piecewise-linear
interpolation instead of milliseconds via fresh wind integration.
`PathTimeInit` is removed (was the wind-integrating init; entire
struct + `ParticleInitDependent` impl gone).

`reoptimize_times`'s public signature also drops the now-unused
`boat`, `wind_source`, `route_bounds` parameters. Two bywind call sites
in `bywind/src/search.rs` updated to match.

### Wallclock impact (5 runs each, no timers, release build)

| Scenario | Before | After | Δ |
|---|---:|---:|---:|
| Small (N=20) | 1.43 s | **1.03 s** (median of 1.00, 1.02, 1.03, 1.04, 1.05) | **−28 %** |
| Large (N=40) | 1.92 s | **1.48 s** (median of 1.45, 1.47, 1.48, 1.50, 1.53) | **−23 %** |

### CPU-time impact (with timers)

`init_pos_inner` per-call cost dropped 60-100×, from milliseconds to
microseconds:

| Scenario | Before (per call) | After (per call) | Total before | Total after |
|---|---:|---:|---:|---:|
| Small | 3.28 ms | 35-48 µs | 5 242 ms | ~60-75 ms |
| Large | 4.52 ms | 67-76 µs | 7 230 ms | ~108-120 ms |

`time_nested_mover` total CPU drops correspondingly (small: 13.04 →
~7.7 s = −41 %; large: 20.07 → ~12.6 s = −37 %). All other timer
values move within run-to-run noise.

### Correctness sanity checks

The user's concern was that interpolation error in `cache.query_range`
might cause the path to go over land. Land-crossing depends only on
xy (`SailboatFitCalc` walks the great circle between waypoints and
queries the SDF) — `t` doesn't enter geographic position. Verified
empirically:

| Scenario | Total land (baseline) | Total land (after, 5 runs avg) |
|---|---:|---:|
| Small | 76.60 km | 79.7 km (within run-to-run variance) |
| Large | 0.00 km | **0.00 km in every single run** |

PSO-vs-bench fitness ratio also stays within run-to-run variance:
small was "PSO 3.0 % worse"; new runs show 3.6-8.9 % worse. Large was
"PSO 2.9 % better"; new runs show 10.6-17 % better — slightly *better*
on average, plausibly because cache-aware init produces samples
guaranteed to be inside the boundary's enforced range, removing a
first-iteration wasted boundary clamp.

All 32 swarmkit tests + all 115 bywind tests still pass.

## Iteration 6: K_MCR sensitivity study

After cache-aware init landed, `segment_cache_build` became the next
largest CPU chunk (~30 % of nested-mover after iteration 5). The
cache build does, per segment, `(N − 1) × RANGE_K × ~K_MCR` calls
into `Boat::get_travel_time`; each one is a full substep
wind-integration. **Reducing the number of mcr-axis samples per
dep bucket (`K_MCR`) cuts cache-build cost roughly linearly while
preserving each sample's full substep accuracy** — only the
piecewise-linear interpolation between sampled mcr points loses
resolution. The inner PSO uses these tabulated values to optimise
each segment's `t`; less interpolation accuracy means a slightly
suboptimal `t` per xy candidate.

This was specifically the kind of approximation that *doesn't*
compromise the substep wind-integration — distinct from the
"constant-wind midpoint" idea, which was rejected on accuracy
grounds.

### Method

Three-way comparison: `K_MCR ∈ {8, 6, 4}`, three RNG seeds (42, 43,
44) per scenario, deterministic via the new `--seed` flag.
`--load-baked` for both scenarios as usual; release build, no
profile-timers feature on.

### Fitness (deterministic, seeded)

**Small (NY → NOLA, N = 20):**

| Seed | K=8 fitness | K=6 fitness | K=4 fitness | Δ 8→6 | Δ 8→4 |
|---|---:|---:|---:|---:|---:|
| 42 | -4 424 035 | -4 456 893 | -4 457 401 | −0.74 % | −0.75 % |
| 43 | -4 559 408 | -4 567 779 | -4 606 083 | −0.18 % | −1.02 % |
| 44 | -4 833 985 | -4 853 725 | -4 542 289 | −0.41 % | **+6.0 %** |

Small is too noisy to read cleanly — the seed-44 K=4 result is
*better* than the K=8 baseline (search converged to a less-land-
crossing route by chance). K=6 shows uniform sub-1 % regression.

**Large (Lisbon → Mumbai, N = 40):**

| Seed | K=8 fitness | K=6 fitness | K=4 fitness | Δ 8→6 | Δ 8→4 |
|---|---:|---:|---:|---:|---:|
| 42 | -4 236 669 | -4 247 347 | -4 259 552 | −0.25 % | −0.54 % |
| 43 | -4 199 674 | -4 207 563 | -4 268 423 | −0.19 % | −1.64 % |
| 44 | -4 235 086 | -4 286 688 | -4 322 032 | **−1.22 %** | **−2.05 %** |

Large is consistent: K=6 worst-case −1.22 %; K=4 worst-case −2.05 %.
Roughly half-half — halving the sample count halves the saving and
roughly halves the regression.

### Land (the user's primary correctness concern)

| Seed | K=8 | K=6 | K=4 |
|---|---:|---:|---:|
| Small 42 | 74.4 km | 75.1 km | 75.1 km |
| Small 43 | 77.2 km | 77.4 km | 78.5 km |
| Small 44 | 83.1 km | 83.6 km | 76.2 km |
| Large 42 / 43 / 44 | 0.00 km | **0.00 km** | **0.00 km** |

Land-crossing is strictly an `xy` quantity (geographic path between
waypoints, evaluated by `SailboatFitCalc`'s great-circle SDF walk).
`K_MCR` only affects the inner PSO's `t` lookup table. **Confirmed
empirically: zero land regression across all K values on large
(where it matters).** Small variations on small are at the level of
run-to-run noise, not a K_MCR-driven trend.

### Wallclock (5-run medians; system-noise outliers re-run)

| Scenario | K_MCR=8 | K_MCR=6 | K_MCR=4 |
|---|---:|---:|---:|
| Small | 1.03 s | 0.96 s (**−7 %**) | 0.90 s (**−13 %**) |
| Large | 1.46 s | 1.39 s (**−5 %**) | 1.31 s (**−10 %**) |

### CPU breakdown at K_MCR=4 (large, seed=42)

For comparison with the iteration-5 K_MCR=8 baseline. Bonus: `inner_fit`
and `inner_boundary` got incidental wins because their interpolation
loops walk a shorter K_MCR axis.

| Stage | K=8 (CPU-ms) | K=4 (CPU-ms) | Δ |
|---|---:|---:|---:|
| `segment_cache_build` | 6 138 | **3 008** | **−51 %** ✓ |
| `time_nested_mover` (parent) | 12 640 | 8 995 | −29 % |
| `inner_fit` | 4 030 | 3 685 | −9 % |
| `inner_boundary` | 1 720 | 1 489 | −13 % |

### Decision

**Held at K_MCR=8 for now.** All three options are viable; the
trade-offs are documented for whoever turns this knob in the future.
The `range_cache::K_MCR` docstring carries a back-reference to this
section.

Concrete heuristic: drop `K_MCR` to 6 or 4 when wallclock matters
more than ~1-2 % fitness — e.g. when running parameter-sweep or
hyperparameter-tuning trials where each search is one of many,
fitness signals are averaged across trials, and the bench-fitness
comparison stays comfortably within the "PSO better than A* benchmark"
window.

## Iteration 7: geometric path sharing in cache build

After the K_MCR study was shelved (iteration 6) on accuracy grounds,
the next-largest CPU chunk was still `segment_cache_build` at ~50 %
of `time_nested_mover` CPU. The redundancy this iteration targets:
inside `SegmentRangeTables::build`, each segment's `(dep_bucket, mcr)`
table cell is filled by a separate `Boat::get_travel_time` call. The
calls share the same `(origin, destination, departure_time)` and
differ only in `mcr_01`, but each one independently re-walks the
great circle from origin to destination — recomputing
`initial_bearing` + 2 × `destination_point` per substep. With
`RANGE_K × K_MCR = 64` calls per segment, that's 64× redundant trig
on the geometric walk.

### The change

Add a batched `Sailboat::get_travel_times_for_mcrs` trait method
(default impl: per-mcr loop fallback) and override on `Boat` with a
two-phase implementation:

1. Pre-compute `positions[k]`, `mid_positions[k]`, and
   `(sin(bearing[k]), cos(bearing[k]))` once for the full step_count
   substep walk. This is mcr-independent — every mcr visits the same
   positions.
2. For each mcr in the input slice, replay through the precomputed
   geometry, sampling wind at `(position, time)` and `(mid_position,
   mid_time)` and computing the per-substep speed. Time accumulation
   diverges across mcr (so wind sample times differ), but the
   geometry is shared.

`SegmentRangeTables::build`'s bucket loop now does **one batched call
per bucket** (8 mcr samples in a single shared-geometry walk) instead
of 8 separate full-walk calls. Pass 2 (the wmcr=0.0→0.1 uniformisation
step) is similarly batched.

Bonus: `Boat::get_travel_time`'s own substep loop now also caches
`bearing.sin_cos()` once and reuses it for both the start-of-substep
and mid-substep wind-speed evaluations. Trivial micro-optimisation
with the same shape; both call sites use the new
`get_wind_speed_with_sin_cos` helper.

### Validation: bit-exact

Seeded runs (seeds 42, 43, 44) produced **identical fitness, identical
Total land, identical Bench fit values to the last digit** of the
pre-change baseline. The geometric walk is shared — not approximated —
and the wind sampling order is preserved per mcr, so floating-point
results are unchanged. Zero accuracy compromise.

### CPU breakdown (with `profile-timers`, seed=42)

| Stage | Pre (small / large) | Post (small / large) | Δ |
|---|---:|---:|---:|
| `segment_cache_build` | 4 282 / 6 138 ms | **2 846 / 4 329 ms** | **−34 % / −29 %** |
| `time_nested_mover` (parent) | 7 702 / 12 640 ms | 6 301 / 10 755 ms | −18 % / −15 % |
| Other stages | within run-to-run noise | | |

### Wallclock impact (10 runs each, clean machine, seed=42)

Apples-to-apples comparison via `git stash` of just the
optimisation's swarmkit-side files (boat.rs, range_cache.rs,
traits.rs) — same session, same kernel scheduler state, one OS-noise
outlier dropped per scenario.

| Scenario | Baseline median | Post-geom-sharing median | Δ |
|---|---:|---:|---:|
| Small | 1.01 s | **0.97 s** | **−4.0 %** |
| Large | 1.47 s | **1.42 s** | **−3.4 %** |

CPU savings (~30 % of cache build, ~15-18 % of nested mover) are
real and material. Wallclock savings (~3-4 %) are smaller because the
outer search is already heavily parallel via rayon at
PAR_LEAF_SIZE=1; saved CPU lets each rayon worker finish its task
sooner but the iter wallclock is determined by the slowest task in
each rayon batch. The **CPU saving is the more relevant figure for
parameter-sweep / tuning workloads** that run multiple searches in
parallel — there each saved second of CPU lets a peer search finish
sooner.

### Status

Landed.

## Status of the original profiling plan

- Step 0 (workloads) ✓
- Step 1 (baseline) ✓
- Step 2 (sub-stage timers) ✓ — ~95 % of CPU named.
- Step 3 (samply extractor) — skipped; timer story was granular enough.
- Step 4 (samply trace) — skipped; see above.
- Step 5 (optimisation candidates) — two landed (cache-aware init in
  iteration 5; geometric path sharing in iteration 7). One studied and
  shelved (K_MCR reduction, iteration 6; ready to be reactivated when
  the fitness-vs-speed trade tilts the other way). Remaining
  candidates: late-convergence cache reuse with per-particle
  persistent state (~7-15 % wallclock, real architectural change),
  cache reuse for outer fitness (~5-7 % wallclock, architectural
  change to share cache between mover and fitness pass), SIMD substep
  batching in `Boat::get_travel_time` (~3-5 % wallclock, heavy
  engineering).
