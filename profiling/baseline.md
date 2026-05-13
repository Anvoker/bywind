# Profiling baseline (search-only)

Step 1 of the profiling plan: capture median + variance of `search_dur` for the
two real-port scenarios so any future optimisation has a credible noise floor
to beat. Bake cost is explicitly out of scope; every run uses `--load-baked`
against the per-scenario cached grid.

## Current vs. baseline (as of iteration 7)

> The original baseline numbers below are preserved as the historical
> reference point — what the search wallclock looked like *before* any
> optimisation work started. Cumulative impact of the
> [`stage2-counters.md`](stage2-counters.md) iterations on the same
> scenarios:

| Scenario | Original baseline (iter 1) | Current (post iter 7) | Cumulative Δ |
|---|---:|---:|---:|
| Small (NY → NOLA, N=20) | 1.46 s | **0.97 s** | **−34 %** |
| Large (Lisbon → Mumbai, N=40) | 1.97 s | **1.42 s** | **−28 %** |

What landed (each documented in detail in `stage2-counters.md`):

- **Iteration 5 — cache-aware inner-PSO init.** Replaced
  `PathTimeInit::init_pos`'s redundant wind integrations with cache
  queries. ~25 % wallclock improvement.
- **Iteration 6 — K_MCR sensitivity study (shelved).** Empirically
  validated K_MCR=4/6 alternatives; documented for future
  reactivation when the trade tilts toward speed at sub-2 % fitness
  cost.
- **Iteration 7 — geometric path sharing in cache build.** Batched
  `Sailboat::get_travel_times_for_mcrs` shares the great-circle
  substep walk across all mcr samples per bucket. Bit-exact fitness.
  ~3-4 % wallclock + ~30 % `segment_cache_build` CPU.
- **(Earlier, before iter 5)** Inner-fitness `FIT_PAR_LEAF_SIZE`
  switched to `usize::MAX` (rayon overhead exceeded the
  parallelism benefit on per-fitness-call work measured at 1-2 µs).
  ~10 % CPU saving, modest wallclock change.

**Still on the table** (rough cost/payoff order):

1. K_MCR=6 or 4 reduction — studied, parked. ~6-13 % wallclock at
   1-2 % fitness loss. Best activator: parameter-sweep workloads.
2. Late-convergence cache reuse with per-particle persistent state —
   ~7-15 % wallclock, real architectural change.
3. Cache reuse for outer fitness — ~5-7 % wallclock, architectural.
4. SIMD substep batching — ~3-5 % wallclock, heavy engineering.

---

## Hardware / build

- Machine: Windows 10 Pro 10.0.19045 (user's primary dev box).
- Build: `cargo build --release -p bywind-cli`. Release profile, default
  opt-level=3 across the workspace.
- Wind source: `C:\Projects\GameDev\grib-data\global_gfs_720h_hourly.grib2`
  (1.2 GB, 720h hourly GFS).

## Scenario definitions

Each scenario was baked once from the grib2 with `--save-baked`, then the
profiling runs used `--load-baked` against the cached grid. Bake step is
never timed and never re-run.

### Small — New York → New Orleans, N = 20

- Endpoints: `(-74.00, 40.71) → (-90.07, 29.95)`.
- Auto-derived bounds: `(-90.36, 25.00) → (-67.48, 41.07)` lon × lat.
- Baked grid size: ~90 MB on disk (`profiling/small/baked.bk1`).
- Optimal sea route exits the Atlantic, runs south past Florida, and
  crosses the Gulf of Mexico — the LandmassSource path is actively
  engaged at the Florida detour.
- PSO settings: defaults from `bywind::config::SearchConfig` (40
  particles × 40 outer iters / 40 particles × 30 inner iters, inertia
  0.2, cognitive 1.6, social 0.85, default mutation kicks).

### Large — Lisbon → Mumbai, N = 40

- Endpoints: `(-9.14, 38.72) → (72.83, 19.08)`.
- Auto-derived bounds: `(-35.87, -50.04) → (90.95, 53.51)` lon × lat
  (covers all of Africa + Indian Ocean, since the optimal route is
  around Cape of Good Hope).
- Baked grid size: 2.2 GB on disk (`profiling/large/baked.bk1`).
- Real shipping route, classic transoceanic landmass detour around
  southern Africa.
- PSO settings: same defaults as small (only `waypoints` differs).

## Original baseline timing samples (pre-optimisation, release build, `--load-baked`)

Five runs per scenario, back-to-back, on a quiescent machine. These are
the *historical* numbers from before any iteration 2+ optimisation work
landed; current numbers are at the top of this document.

| Run    | Small (N=20) | Large (N=40) |
|--------|--------------|--------------|
| 1      | 1.46 s       | 1.97 s       |
| 2      | 1.53 s       | 1.94 s       |
| 3      | 1.43 s       | 1.97 s       |
| 4      | 1.39 s       | 1.90 s       |
| 5      | 1.46 s       | 1.97 s       |
| **median** | **1.46 s** | **1.97 s** |
| mean   | 1.45 s       | 1.95 s       |
| range  | 0.14 s (10 %) | 0.07 s (4 %) |

Variance is tight — the large scenario is actually *more* stable in
relative terms (1.5 % σ-around-median vs ~5 % for small) because
proportional jitter from CPU scheduling shrinks against a heavier
workload.

## Observations

- **Both scenarios complete in 1.4–2.0 s.** Search is already fast; this
  is the noise floor any future optimisation has to beat to be credible.
- **N=20 → N=40 only adds ~35 % to search time.** Total per-iteration
  work is sub-quadratic in N — consistent with the inner time-PSO cache
  (`SegmentRangeTables`) doing its job, since otherwise we'd see the
  per-segment fitness cost compound with N.
- **Both bench fitness vs PSO fitness numbers came out close to 1:1**
  (small: PSO 3.0 % worse; large: PSO 2.9 % better), suggesting the
  search is well-converged at default sizing on these scenarios — there
  isn't headroom to be claimed by *more* iterations, only by *faster*
  ones.

## Status of the original profiling plan (resolved)

- **Step 0 — workloads.** ✓ Two scenarios above.
- **Step 1 — baseline (this doc).** ✓
- **Step 2 — sub-stage `Instant::now` counters.** ✓ See
  [`stage2-counters.md`](stage2-counters.md) iteration 2 onwards.
- **Step 3 — samply extractor.** Skipped; the timer story turned out
  granular enough to drive optimisation directly. Easy to revive if
  needed.
- **Step 4 — samply trace.** Skipped; same reason.
- **Step 5 — optimisation candidates.** Two landed (cache-aware init,
  geometric path sharing). One studied and parked (K_MCR reduction).
  Three remain on the table (see top of this doc).

## Bail-out check (resolved)

The plan's bail-out point was *"if median search time is sub-second
on the large scenario, ask whether optimisation is worthwhile."* Was
1.97 s at baseline; we kept going through iterations 2-7 because the
motivating use case (parameter-sweep / hyperparameter-tuning runs
multiple searches in parallel — saved CPU per search compounds) made
the absolute number less relevant than the per-search cost.

After iteration 7, large is at 1.42 s and small at 0.97 s. The
remaining candidates' realistic wallclock potential
(~5-15 % each) crosses the 1-second threshold for large only with
several more iterations stacked. From here the question shifts from
"what's the next easy win" to "what's the next change worth doing
given its engineering cost." See `stage2-counters.md`'s closing
section for that ranking.
