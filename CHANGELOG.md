# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows pre-1.0 SemVer where the minor version bumps on breaking changes.

## [Unreleased]

### Fixed
- **Viz right-panel regression.** A one-time-tall Summary block (ensemble
  spread + benchmark grid + long-route segment list) was locking the bottom
  sub-panel's height via `egui::Panel`'s persisted `rect`, permanently
  squishing the Segments scroll above it. Replaced the nested
  `Panel::bottom` + `CentralPanel` with a plain top-down flow where the
  Summary sizes to its actual content each frame and the Segments
  `ScrollArea` takes whatever's left. The view selector is now anchored
  directly under the Summary heading so it doesn't shift when the bench /
  spread / main-comparison blocks below it reflow.

## [0.2.0] - 2026-06-04

The PSO can now optimise against a K-member GEFS ensemble instead of a single
deterministic wind map, and the viz shows per-realization spread alongside the
main gbest. The landmass model also gains strait carve-outs and a two-tier SDF
so routes through narrow channels (Gibraltar, Bosporus, Malacca, …) come back
land-free.

### Added — ensemble
- **Ensemble search.** `TimedEnsembleWindMap` / `BakedEnsembleWindMap` load a
  directory of `.wcav` files (`gec00` control + `gepNN` perturbations) and feed
  them into the optimiser. `EnsembleMode::Full` runs K-fold robust fitness;
  `EnsembleMode::FastMean` collapses the ensemble to its mean wind for a ~K×
  speed-up. See `src/ensemble.rs` and the "Ensemble feature" section of
  `CLAUDE.md` for vocabulary.
- **`SearchResult::ensemble: Option<EnsembleSpread>`.** Per-member evaluation of
  the main gbest path — same route, K winds — so callers can read robustness
  off a single search.
- **`run_realizations`.** Runs K independent PSO searches, one per member, so
  the viz can overlay K decoupled gbest paths against the main solution.
- **`SavedSolution` carries `Option<SavedEnsemble> { mode, spread }`.** Ensemble
  runs round-trip through the on-disk JSON schema; single-deterministic saves
  remain byte-identical via `serde(skip_serializing_if)`.
- **`.wcav` v3.** Per-frame Unix timestamps are appended to the v2 header so
  fetch gaps (transient NOAA 404s) survive round-tripping. The decoder is
  backward-compatible with v1/v2.
- **Viz: GEFS fetch dialog.** `File → Fetch GEFS Ensemble…` pulls a 31-member
  ensemble for the currently-loaded wcav's time window and writes a sibling
  directory.

### Added — landmass
- **Strait carve-outs.** `STRAIT_CARVE_OUTS` punches holes in the rasterised
  land mask for Gibraltar, the Dardanelles/Marmara, the Bosporus,
  Bab-el-Mandeb, Malacca, and Singapore — A*, SDF, the fitness penalty, and
  boundary repulsion all share the same open channels.
- **Two-tier SDF.** `TwoTierLandmass` aggregates a coarse global grid with
  fine `FinePatch` tiles around each strait carve-out, and `find_sea_path`
  runs a unified-graph A* that expands at the cell's own tier and hops
  between tiers at fine-bbox boundaries. Opt-in via
  `SearchConfig::fine_sdf_resolution_deg: Option<f64>` (default `Some(0.1)`).
  On the Black Sea → Biscay scenario the benchmark route now reports 0 km of
  land (was 218 km) and PSO converges 0.8% better than the benchmark instead
  of 45% worse.
- **`sampling_step_metres` clamp.** `get_segment_land_metres` substeps at
  half a cell, so 150-km PSO chords no longer underreport land when
  `step_distance_max` was sized for continental wind integration.
- **Unbiased A* fallback** in `compute_baselines` when the biased pathfinder
  returns `None`, so init seeds particles on real sea polylines in narrow
  basins like the Mediterranean.
- **`sdf_resolution_deg` in TOML.** The existing GUI knob for landmass cell
  size now round-trips through scenario TOML and the CLI.
- **Viz: SDF cell overlay.** "Show SDF cells" paints sea cells translucent
  blue and land cells translucent red, so the user can see what the search
  actually considers sea — particularly around the strait carve-outs, which
  read as capsule-shaped sea tubes the polygon coastline doesn't show.
- **Viz: two-tier overlay.** When two-tier is enabled the overlay paints
  coarse cells outside fine bboxes plus each patch's own cells, with the
  fine patches at higher fill alpha and darker stroke so the two grids read
  distinctly where they meet.

### Added — viz
- **Live search progress.** The A* benchmark renders as soon as it's
  computed (before the PSO begins); the gbest-so-far streams to the canvas
  on every PSO iteration. A phase indicator chip (loading / baking /
  benchmark / search) covers the dead time on cold starts.
- **Route-horizon warning.** Flagged when the route runtime exceeds the
  loaded wind's data horizon.

### Changed
- **Ensemble vocabulary harmonised** across core, CLI, and viz. The canonical
  terms — "ensemble", "member", "realization", "main", `EnsembleMode`,
  `EnsembleSpread` — are documented in `CLAUDE.md`.
- **`swarmkit-sailing::search` → `search_with_progress`** as the primary
  entry point. `search` remains a thin delegate passing a no-op callback, so
  existing callers compile unchanged. The signature change is the one breaking
  surface in `swarmkit-sailing`.
- **`format_pso_delta` → `format_delta(subject, other, larger_is_better, subject_label)`**.
  Realization-comparison labels now read from the main route's perspective
  ("Main 1.1% worse") instead of the realization's ("PSO 1.1% better").
- **`LonLatBbox`** is now constructed exclusively through struct-literal syntax
  to prevent tuple-order regressions like the route-bbox permutation fixed in
  `aa57635`.

### Fixed
- **Mismatched-ensemble-member frame counts no longer panic.** Transient NOAA
  gaps (typically `gepNN` with one missing 3-hour slot) get clipped to the
  shortest member at load time instead of asserting in `mean()`.
- **Scenario state survives the bundled-sample arrival.** Loading the bundled
  sample on first launch no longer clears any in-progress route or scenario
  the user had loaded.
- **A* benchmark contrast** in the viz is now theme-aware (10% darker against
  light backgrounds).

### Internal
- **SDF resolution benchmarks** added under `bywind-dev` (unpublished):
  `sdf_resolution_bench` for grid-build / A* / bulk SDF query timing across
  0.5° → 0.1° resolutions, and `sdf_quality_bench` for full-search
  fitness × seeds. The latter confirms finer SDF is a perf knob, not a
  route-quality lever (~1% worse mean fitness at 0.2° with 26–63% wider
  stddev than 0.5°).
- `cargo fmt` normalised across the workspace as a separate `chore(fmt)`
  precursor commit.

[Unreleased]: https://github.com/Anvoker/bywind/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/Anvoker/bywind/releases/tag/v0.2.0
