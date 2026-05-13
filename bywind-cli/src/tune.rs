//! `tune` subcommand. Drives the PSO-tuning study via the `optimizer`
//! crate's TPE sampler. Each Optuna-style trial spawns a `tune-trial`
//! subprocess (this same binary, different subcommand) for crash
//! isolation; the subprocess's per-route raw fitness is normalised
//! against a pre-recorded baseline (`profiling/tuning/baseline.json`)
//! and the mean ratio across routes is the objective the optimiser
//! minimises. Trials persist to `study.jsonl` so an interrupted study
//! leaves an inspectable record.
//!
//! Default search space (3-D):
//!
//!   inertia          ∈ [0.0, 1.2]
//!   `cognitive_coeff`  ∈ [0.0, 4.0]
//!   `social_coeff`     ∈ [0.0, 4.0]
//!
//! Each dimension can be pinned (every trial uses one value) or given a
//! custom `[low, high]` interval via the `[tune]` section of
//! `<routes_dir>/_search.toml` — see [`bywind::scenario::TuneOverrides`].
//!
//! Meta-objective: `mean over routes of (trial_fitness / baseline_fitness)`.
//! Both are negative (fitness is the negated cost), so a ratio of 0.95
//! means the trial's fitness is 5 % "less negative" than baseline —
//! i.e. 5 % better. Optimiser is set to MINIMIZE this ratio.

use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::Instant;

use anyhow::{Context as _, Result, anyhow};
use bywind::scenario::{CliConfigFile, SearchOverrides, TuneOverrides, TuneSlot};
use optimizer::prelude::*;
use serde::{Deserialize, Serialize};
use serde_json::json;

use crate::error::AppError;
use crate::tune_trial::{TrialParams, TrialResult, TrialSpec};

/// Built-in search-space defaults applied to any coefficient the user
/// hasn't overridden via `[tune]`.
const DEFAULT_INERTIA_RANGE: (f64, f64) = (0.0, 1.2);
const DEFAULT_COGNITIVE_RANGE: (f64, f64) = (0.0, 4.0);
const DEFAULT_SOCIAL_RANGE: (f64, f64) = (0.0, 4.0);

/// Path-kick mover defaults. The probability and `gamma_0` ranges are
/// wide enough to cover both "off" and "very aggressive"; the
/// `decay_ratio` range covers the full unit interval. `gamma_min` is
/// computed from `gamma_0 * decay_ratio` so it can never exceed
/// `gamma_0`.
const DEFAULT_PATH_KICK_PROBABILITY_RANGE: (f64, f64) = (0.0, 0.5);
const DEFAULT_PATH_KICK_GAMMA_0_FRACTION_RANGE: (f64, f64) = (0.0, 0.20);
const DEFAULT_PATH_KICK_DECAY_RATIO_RANGE: (f64, f64) = (0.0, 1.0);

/// One trial's worth of suggested values, combining the three `GBest`
/// coefficients and the three path-kick knobs. `decay_ratio` is the
/// tune-side virtual parameter; the trial-side raw `gamma_min` is
/// derived via [`Self::path_kick_gamma_min_fraction`].
#[derive(Clone, Copy, Debug)]
struct SuggestedParams {
    inertia: f64,
    cognitive_coeff: f64,
    social_coeff: f64,
    path_kick_probability: f64,
    path_kick_gamma_0_fraction: f64,
    path_kick_decay_ratio: f64,
}

impl SuggestedParams {
    /// Cosine-decayed schedule floor as a fraction of route length. By
    /// construction `≤ gamma_0_fraction` since `decay_ratio ∈ [0, 1]`.
    fn path_kick_gamma_min_fraction(&self) -> f64 {
        self.path_kick_gamma_0_fraction * self.path_kick_decay_ratio
    }
}

/// One coefficient slot in the search: either a tunable `FloatParam`
/// (TPE picks values within `[low, high]`) or a fixed scalar that
/// every trial uses verbatim. Built from a [`TuneSlot`] override (or
/// the built-in default range) so callers can peg dimensions and
/// concentrate the trial budget on the still-unknown ones — e.g.
/// `[tune] inertia = 0.328` after a prior study has settled it.
struct ParamSlot {
    /// `Some((param, low, high))` for tunable dimensions; the `low`/`high`
    /// pair is kept alongside the optimiser handle because `FloatParam`
    /// itself doesn't expose them and we want them for `describe`.
    tunable: Option<(FloatParam, f64, f64)>,
    fixed: Option<f64>,
}

impl ParamSlot {
    /// Build a slot for `name`. Resolution order, highest precedence first:
    ///
    /// 1. `[tune].field = N`        → fixed at `N` (pin out of search space).
    /// 2. `[tune].field = { min, max }` → tunable in `[min, max]`.
    /// 3. `[search].field = N`       → fixed at `N` (the user pinned it for
    ///    every bywind-cli consumer; tune respects it like `search` does).
    /// 4. fallback                  → tunable in the built-in default range.
    ///
    /// Without rule 3, `[search].inertia = 0.328` would silently get
    /// overridden by trial-suggested values whenever `[tune].inertia` was
    /// missing, which contradicts the `[search]` schema's "this is the
    /// pinned value for every consumer" semantics.
    fn from_override(
        name: &str,
        tune_override: Option<TuneSlot>,
        search_pin: Option<f64>,
        default_range: (f64, f64),
    ) -> Self {
        match tune_override {
            Some(TuneSlot::Pinned(v)) => Self {
                tunable: None,
                fixed: Some(v),
            },
            Some(TuneSlot::Range { min, max }) => Self {
                tunable: Some((FloatParam::new(min, max).name(name), min, max)),
                fixed: None,
            },
            None => {
                if let Some(v) = search_pin {
                    Self {
                        tunable: None,
                        fixed: Some(v),
                    }
                } else {
                    let (low, high) = default_range;
                    Self {
                        tunable: Some((FloatParam::new(low, high).name(name), low, high)),
                        fixed: None,
                    }
                }
            }
        }
    }

    fn suggest(&self, trial: &mut Trial) -> Result<f64> {
        if let Some((p, _, _)) = &self.tunable {
            Ok(p.suggest(trial)?)
        } else {
            Ok(self.fixed.expect("invariant: tunable xor fixed"))
        }
    }

    fn extract_from_best(&self, best: &CompletedTrial<f64>, name: &str) -> Result<f64> {
        if let Some((p, _, _)) = &self.tunable {
            best.get(p)
                .ok_or_else(|| anyhow!("best trial missing param `{name}`"))
        } else {
            Ok(self.fixed.expect("invariant: tunable xor fixed"))
        }
    }

    fn describe(&self, name: &str) -> String {
        match (&self.tunable, self.fixed) {
            (Some((_, low, high)), _) => format!("{name}: tunable in [{low}, {high}]"),
            (None, Some(v)) => format!("{name}: pegged at {v}"),
            (None, None) => format!("{name}: ???"),
        }
    }
}

/// Default seeds run inside each trial. Three seeds gives a
/// 0.4–1.2 % per-route std-as-fraction-of-mean noise floor —
/// comfortable for detecting tuner improvements ≥ 1–2 %.
const DEFAULT_TRIAL_SEEDS: &[u64] = &[42, 1337, 9001];

/// Default training-route slugs.
const DEFAULT_TRAINING_ROUTES: &[&str] =
    &["short-easy", "archipelago", "coastal-detour", "mid-pacific"];

#[derive(Debug)]
pub struct TuneArgs {
    pub trials: usize,
    pub baseline: PathBuf,
    pub routes_dir: PathBuf,
    pub journal: PathBuf,
    pub sampler_seed: u64,
    pub trial_seeds: Vec<u64>,
    pub routes: Vec<String>,
}

impl TuneArgs {
    /// Apply CLI-flag overrides on top of the defaults so callers can
    /// build a `TuneArgs` by `..TuneArgs::defaults()`.
    pub fn defaults() -> Self {
        Self {
            trials: 50,
            baseline: PathBuf::from("profiling/tuning/baseline.json"),
            routes_dir: PathBuf::from("profiling/tuning"),
            journal: PathBuf::from("profiling/tuning/study.jsonl"),
            sampler_seed: 0,
            trial_seeds: DEFAULT_TRIAL_SEEDS.to_vec(),
            routes: DEFAULT_TRAINING_ROUTES
                .iter()
                .map(|s| (*s).to_owned())
                .collect(),
        }
    }
}

/// Filename of the optional shared TOML inside `routes_dir`. When
/// present, its `[tune]` section configures the per-coefficient search
/// space, and `[search]` provides default pins for any field that
/// `[tune]` doesn't override (so `[search].inertia = 0.328` actually
/// pins inertia for the study, matching the behaviour `bywind-cli
/// search` already gives the same field).
const SHARED_CONFIG_FILENAME: &str = "_search.toml";

/// Load the shared TOML's `[tune]` and `[search]` sections if the file
/// exists, validate `[tune]`, and return both. Missing file is not an
/// error — the user simply hasn't customised the search space.
fn load_shared_overrides(
    routes_dir: &std::path::Path,
) -> Result<(TuneOverrides, SearchOverrides), AppError> {
    let path = routes_dir.join(SHARED_CONFIG_FILENAME);
    if !path.exists() {
        return Ok((TuneOverrides::default(), SearchOverrides::default()));
    }
    let cfg = CliConfigFile::from_path(&path).map_err(anyhow::Error::from)?;
    cfg.tune
        .validate()
        .map_err(|msg| AppError::from(anyhow!("invalid `[tune]` in {}: {msg}", path.display())))?;
    Ok((cfg.tune, cfg.search))
}

/// Mirrors the structure of `tune-trial`'s baseline output: a per-route
/// table of `(slug, fitness_mean)` pairs.
#[derive(Deserialize, Debug)]
struct BaselineFile {
    per_route: Vec<BaselineRoute>,
}

#[derive(Deserialize, Debug)]
struct BaselineRoute {
    slug: String,
    fitness_mean: f64,
}

/// Per-trial summary written to stderr alongside the per-trial log
/// line. Also serialised into a final `tune-summary.json` next to the
/// journal when the study completes.
#[derive(Serialize, Debug)]
struct StudySummary {
    trials_run: usize,
    sampler: &'static str,
    sampler_seed: u64,
    routes: Vec<String>,
    trial_seeds: Vec<u64>,
    best_score: f64,
    best_inertia: f64,
    best_cognitive_coeff: f64,
    best_social_coeff: f64,
    best_path_kick_probability: f64,
    best_path_kick_gamma_0_fraction: f64,
    best_path_kick_decay_ratio: f64,
    best_path_kick_gamma_min_fraction: f64,
}

#[expect(
    clippy::too_many_lines,
    reason = "study setup + optimise loop + summary print are tightly coupled; splitting reduces readability"
)]
pub fn run(args: &TuneArgs) -> Result<(), AppError> {
    let baseline = load_baseline(&args.baseline)?;
    cross_check_routes(&baseline, &args.routes)?;

    let (tune, search) = load_shared_overrides(&args.routes_dir)?;
    let inertia = ParamSlot::from_override(
        "inertia",
        tune.inertia,
        search.inertia,
        DEFAULT_INERTIA_RANGE,
    );
    let cognitive = ParamSlot::from_override(
        "cognitive_coeff",
        tune.cognitive_coeff,
        search.cognitive_coeff,
        DEFAULT_COGNITIVE_RANGE,
    );
    let social = ParamSlot::from_override(
        "social_coeff",
        tune.social_coeff,
        search.social_coeff,
        DEFAULT_SOCIAL_RANGE,
    );
    let path_kick_prob = ParamSlot::from_override(
        "path_kick_probability",
        tune.path_kick_probability,
        search.path_kick_probability,
        DEFAULT_PATH_KICK_PROBABILITY_RANGE,
    );
    let path_kick_gamma_0 = ParamSlot::from_override(
        "path_kick_gamma_0_fraction",
        tune.path_kick_gamma_0_fraction,
        search.path_kick_gamma_0_fraction,
        DEFAULT_PATH_KICK_GAMMA_0_FRACTION_RANGE,
    );
    // `decay_ratio` is a tune-side virtual knob — there's no field for
    // it in `[search]`. Derive a pin from the layered gamma_0 / gamma_min
    // when both are set so the schedule matches what a non-tune search
    // would have used; otherwise let `from_override` vary it.
    let decay_ratio_search_pin = match (
        search.path_kick_gamma_0_fraction,
        search.path_kick_gamma_min_fraction,
    ) {
        (Some(g0), Some(g_min)) if g0 > 0.0 => Some(g_min / g0),
        _ => None,
    };
    let path_kick_decay_ratio = ParamSlot::from_override(
        "path_kick_decay_ratio",
        tune.path_kick_decay_ratio,
        decay_ratio_search_pin,
        DEFAULT_PATH_KICK_DECAY_RATIO_RANGE,
    );

    let sampler = TpeSampler::builder()
        .seed(args.sampler_seed)
        .build()
        .map_err(|e| anyhow!("building TpeSampler: {e}"))?;
    let storage = JournalStorage::new(&args.journal);
    let study: Study<f64> = Study::with_sampler_and_storage(Direction::Minimize, sampler, storage);

    let exe = std::env::current_exe().context("locating current exe")?;
    let trial_counter = std::cell::Cell::new(0usize);
    let study_start = Instant::now();

    eprintln!(
        "starting tune: {} trials, {} routes, {} seeds/trial, sampler=TpeSampler(seed={})",
        args.trials,
        args.routes.len(),
        args.trial_seeds.len(),
        args.sampler_seed,
    );
    eprintln!("  {}", inertia.describe("inertia"));
    eprintln!("  {}", cognitive.describe("cognitive_coeff"));
    eprintln!("  {}", social.describe("social_coeff"));
    eprintln!("  {}", path_kick_prob.describe("path_kick_probability"));
    eprintln!(
        "  {}",
        path_kick_gamma_0.describe("path_kick_gamma_0_fraction")
    );
    eprintln!(
        "  {}",
        path_kick_decay_ratio.describe("path_kick_decay_ratio")
    );
    eprintln!("baseline:    {}", args.baseline.display());
    eprintln!("journal:     {}", args.journal.display());
    eprintln!();

    study
        .optimize(args.trials, |trial: &mut Trial| -> Result<f64> {
            // `?` on `suggest()` works because `optimizer::Error` impls
            // `std::error::Error`, which `anyhow::Error` accepts via its
            // blanket `From` impl. Subprocess errors propagate the same
            // way. The optimizer trait only requires the closure error
            // to be `ToString + 'static`, which `anyhow::Error` is.
            // `ParamSlot::suggest` either calls FloatParam::suggest (when
            // tunable) or returns the pegged value (when fixed).
            let suggested = SuggestedParams {
                inertia: inertia.suggest(trial)?,
                cognitive_coeff: cognitive.suggest(trial)?,
                social_coeff: social.suggest(trial)?,
                path_kick_probability: path_kick_prob.suggest(trial)?,
                path_kick_gamma_0_fraction: path_kick_gamma_0.suggest(trial)?,
                path_kick_decay_ratio: path_kick_decay_ratio.suggest(trial)?,
            };

            let trial_idx = trial_counter.get() + 1;
            trial_counter.set(trial_idx);

            let trial_start = Instant::now();
            let result = run_trial_subprocess(&exe, args, &suggested)
                .with_context(|| format!("trial {trial_idx} subprocess"))?;
            let score = score(&result, &baseline);
            let elapsed = trial_start.elapsed().as_secs_f64();

            eprintln!(
                "[{:>3}/{:>3}] i={:.3} c={:.3} s={:.3} prob={:.3} g0={:.3} dr={:.2} \
                 → score={:.4} ({:.1}s)",
                trial_idx,
                args.trials,
                suggested.inertia,
                suggested.cognitive_coeff,
                suggested.social_coeff,
                suggested.path_kick_probability,
                suggested.path_kick_gamma_0_fraction,
                suggested.path_kick_decay_ratio,
                score,
                elapsed,
            );

            Ok(score)
        })
        .map_err(|e| anyhow!("study.optimize failed: {e}"))?;

    let total = study_start.elapsed().as_secs_f64();
    eprintln!();
    eprintln!("study completed in {total:.1}s");

    let best = study
        .best_trial()
        .map_err(|e| anyhow!("retrieving best trial: {e}"))?;
    let best_inertia = inertia.extract_from_best(&best, "inertia")?;
    let best_cognitive = cognitive.extract_from_best(&best, "cognitive_coeff")?;
    let best_social = social.extract_from_best(&best, "social_coeff")?;
    let best_path_kick_prob = path_kick_prob.extract_from_best(&best, "path_kick_probability")?;
    let best_path_kick_gamma_0 =
        path_kick_gamma_0.extract_from_best(&best, "path_kick_gamma_0_fraction")?;
    let best_decay_ratio =
        path_kick_decay_ratio.extract_from_best(&best, "path_kick_decay_ratio")?;
    let best_path_kick_gamma_min = best_path_kick_gamma_0 * best_decay_ratio;

    let best_score = best.value;
    eprintln!();
    eprintln!("=== Best trial ===");
    eprintln!("score:                          {best_score:.6}");
    eprintln!("inertia:                        {best_inertia:.4}");
    eprintln!("cognitive_coeff:                {best_cognitive:.4}");
    eprintln!("social_coeff:                   {best_social:.4}");
    eprintln!("path_kick_probability:          {best_path_kick_prob:.4}");
    eprintln!("path_kick_gamma_0_fraction:     {best_path_kick_gamma_0:.4}");
    eprintln!(
        "path_kick_gamma_min_fraction:   {best_path_kick_gamma_min:.4}  \
         (= γ_0 × {best_decay_ratio:.3})",
    );
    eprintln!();
    eprintln!(
        "  (default: inertia=0.20 cog=1.60 soc=0.85 prob=0.10 g0=0.05 g_min=0.005, \
         score by definition = 1.0)"
    );

    let summary = StudySummary {
        trials_run: args.trials,
        sampler: "TpeSampler",
        sampler_seed: args.sampler_seed,
        routes: args.routes.clone(),
        trial_seeds: args.trial_seeds.clone(),
        best_score: best.value,
        best_inertia,
        best_cognitive_coeff: best_cognitive,
        best_social_coeff: best_social,
        best_path_kick_probability: best_path_kick_prob,
        best_path_kick_gamma_0_fraction: best_path_kick_gamma_0,
        best_path_kick_decay_ratio: best_decay_ratio,
        best_path_kick_gamma_min_fraction: best_path_kick_gamma_min,
    };
    let summary_path = args.journal.with_file_name("tune-summary.json");
    write_summary(&summary_path, &summary)?;
    eprintln!();
    eprintln!("summary written to {}", summary_path.display());

    Ok(())
}

/// Spawn `bywind-cli tune-trial` as a subprocess, write the JSON spec
/// to its stdin, parse the JSON result from its stdout. Subprocess
/// stderr is inherited so the child's logs reach the user.
fn run_trial_subprocess(
    exe: &std::path::Path,
    args: &TuneArgs,
    suggested: &SuggestedParams,
) -> Result<TrialResult> {
    let spec = TrialSpec {
        params: TrialParams {
            inertia: Some(suggested.inertia),
            cognitive_coeff: Some(suggested.cognitive_coeff),
            social_coeff: Some(suggested.social_coeff),
            path_kick_probability: Some(suggested.path_kick_probability),
            path_kick_gamma_0_fraction: Some(suggested.path_kick_gamma_0_fraction),
            path_kick_gamma_min_fraction: Some(suggested.path_kick_gamma_min_fraction()),
        },
        seeds: args.trial_seeds.clone(),
        routes: args.routes.clone(),
        routes_dir: Some(args.routes_dir.clone()),
    };

    let mut child = Command::new(exe)
        .arg("tune-trial")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .context("spawning tune-trial subprocess")?;

    let mut stdin = child
        .stdin
        .take()
        .ok_or_else(|| anyhow!("no stdin handle"))?;
    serde_json::to_writer(&mut stdin, &spec).context("writing trial spec to subprocess stdin")?;
    drop(stdin); // close pipe so subprocess `read_to_string` returns

    let output = child
        .wait_with_output()
        .context("waiting for tune-trial subprocess")?;
    if !output.status.success() {
        return Err(anyhow!(
            "tune-trial exited with {}",
            output
                .status
                .code()
                .map_or("signal".to_owned(), |c| c.to_string()),
        ));
    }
    let result: TrialResult =
        serde_json::from_slice(&output.stdout).context("parsing tune-trial output JSON")?;
    Ok(result)
}

/// Meta-objective: mean over routes of `trial_fitness / baseline_fitness`.
/// Both fitnesses are negative, so the ratio is positive and lower =
/// better. A trial that exactly matches baseline scores 1.0; one that
/// improves by 5 % across the board scores 0.95.
///
/// Routes appearing in the trial result but not the baseline are
/// silently skipped — `cross_check_routes` already validated up front
/// that the trial's route set ⊆ the baseline's, so this only fires if
/// the per-trial spec deviates mid-run.
fn score(result: &TrialResult, baseline: &BaselineFile) -> f64 {
    let mut ratios = Vec::with_capacity(result.per_route.len());
    for trial_route in &result.per_route {
        let Some(baseline_route) = baseline
            .per_route
            .iter()
            .find(|b| b.slug == trial_route.slug)
        else {
            continue;
        };
        // Both negative; ratio comes out positive.
        ratios.push(trial_route.fitness_mean / baseline_route.fitness_mean);
    }
    if ratios.is_empty() {
        return f64::INFINITY;
    }
    ratios.iter().sum::<f64>() / ratios.len() as f64
}

fn load_baseline(path: &std::path::Path) -> Result<BaselineFile, AppError> {
    let text = std::fs::read_to_string(path).with_context(|| {
        format!(
            "reading baseline file {} (run `bywind-cli tune-trial` with default coeffs and \
             redirect to this path before starting the study)",
            path.display(),
        )
    })?;
    let baseline: BaselineFile = serde_json::from_str(&text)
        .with_context(|| format!("parsing baseline JSON at {}", path.display()))?;
    Ok(baseline)
}

fn cross_check_routes(baseline: &BaselineFile, routes: &[String]) -> Result<(), AppError> {
    for slug in routes {
        if !baseline.per_route.iter().any(|r| &r.slug == slug) {
            return Err(AppError::from(anyhow!(
                "route `{slug}` is in the tuning route list but missing from the baseline file \
                 — re-run baseline measurement to include it",
            )));
        }
    }
    Ok(())
}

fn write_summary(path: &std::path::Path, summary: &StudySummary) -> Result<(), AppError> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("creating {} for summary", parent.display()))?;
    }
    let text = serde_json::to_string_pretty(&json!(summary))
        .context("serialising study summary to JSON")?;
    std::fs::write(path, text).with_context(|| format!("writing summary {}", path.display()))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tune_trial::{RouteResult, SeedResult};

    fn baseline(routes: &[(&str, f64)]) -> BaselineFile {
        BaselineFile {
            per_route: routes
                .iter()
                .map(|(slug, fit)| BaselineRoute {
                    slug: (*slug).to_owned(),
                    fitness_mean: *fit,
                })
                .collect(),
        }
    }

    fn trial(routes: &[(&str, f64)]) -> TrialResult {
        TrialResult {
            fitness_mean: 0.0,
            fitness_std: 0.0,
            wall_seconds: 0.0,
            per_route: routes
                .iter()
                .map(|(slug, fit)| RouteResult {
                    slug: (*slug).to_owned(),
                    fitness_mean: *fit,
                    fitness_std: 0.0,
                    seeds: vec![SeedResult {
                        seed: 0,
                        fitness: *fit,
                        wall_seconds: 0.0,
                    }],
                    wall_seconds: 0.0,
                })
                .collect(),
        }
    }

    #[test]
    fn score_equals_one_for_identical_to_baseline() {
        let b = baseline(&[("a", -1000.0), ("b", -2000.0)]);
        let t = trial(&[("a", -1000.0), ("b", -2000.0)]);
        let s = score(&t, &b);
        assert!((s - 1.0).abs() < 1e-12, "got {s}");
    }

    #[test]
    fn score_below_one_means_better_than_baseline() {
        // Trial 5% less negative than baseline on both routes → score 0.95.
        let b = baseline(&[("a", -1000.0), ("b", -2000.0)]);
        let t = trial(&[("a", -950.0), ("b", -1900.0)]);
        let s = score(&t, &b);
        assert!((s - 0.95).abs() < 1e-12, "got {s}");
    }

    #[test]
    fn score_above_one_means_worse_than_baseline() {
        let b = baseline(&[("a", -1000.0)]);
        let t = trial(&[("a", -1100.0)]);
        let s = score(&t, &b);
        assert!((s - 1.1).abs() < 1e-12, "got {s}");
    }

    #[test]
    fn score_normalises_routes_with_different_magnitudes_equally() {
        // One route has 100x the magnitude. If we used raw averages the
        // big-magnitude route would dominate. With normalisation, the two
        // routes contribute equally.
        let b = baseline(&[("small", -100.0), ("huge", -10_000.0)]);
        // small: 5% better, huge: 5% worse → mean 1.0.
        let t = trial(&[("small", -95.0), ("huge", -10_500.0)]);
        let s = score(&t, &b);
        assert!((s - 1.0).abs() < 1e-12, "got {s}");
    }

    #[test]
    fn score_skips_routes_missing_from_baseline() {
        let b = baseline(&[("a", -1000.0)]);
        let t = trial(&[("a", -900.0), ("missing", -500.0)]);
        let s = score(&t, &b);
        // Only "a" contributes: 0.9.
        assert!((s - 0.9).abs() < 1e-12, "got {s}");
    }

    #[test]
    fn cross_check_rejects_route_not_in_baseline() {
        let b = baseline(&[("a", -1.0), ("b", -1.0)]);
        let routes = vec!["a".to_owned(), "missing".to_owned()];
        let err = cross_check_routes(&b, &routes).expect_err("missing route");
        assert!(err.to_string().contains("missing"), "got: {err}");
    }

    #[test]
    fn cross_check_accepts_subset() {
        let b = baseline(&[("a", -1.0), ("b", -1.0), ("c", -1.0)]);
        let routes = vec!["a".to_owned(), "c".to_owned()];
        cross_check_routes(&b, &routes).expect("subset is fine");
    }
}
