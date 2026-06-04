# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows pre-1.0 SemVer where the minor version bumps on breaking changes.

## [0.2.0] - 2026-06-04

First release with ensemble-forecast support. The PSO can now optimise against a
K-member GEFS ensemble instead of a single deterministic wind map, and the viz
shows per-realization spread alongside the main gbest.

### Added
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
- **Viz: live search progress.** The A* benchmark renders as soon as it's
  computed (before the PSO begins); the gbest-so-far streams to the canvas on
  every PSO iteration. Phase indicator chip (loading / baking / benchmark /
  search) covers the dead time on cold starts.
- **Viz: route-horizon warning.** Flagged when the route runtime exceeds the
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
- `cargo fmt` normalised across the workspace as a separate `chore(fmt)`
  precursor commit.

[0.2.0]: https://github.com/Anvoker/bywind/releases/tag/v0.2.0
