//! `bywind-cli` — native command-line frontend to the `bywind` sailing-search
//! library. Subcommand stubs only at this stage (Phase 1 of the plan in
//! `docs/bywind-cli-plan.md`); each subcommand exits with a "not yet
//! implemented" message until its phase lands.

// User-facing CLI output: writing to stderr (and, in later phases, stdout)
// is the binary's purpose. The workspace lints warn on direct print macros
// because that's the right policy for libraries; here we opt out.
#![expect(
    clippy::print_stderr,
    reason = "bywind-cli writes user-facing messages to stderr"
)]

use std::path::PathBuf;
use std::process::ExitCode;

use clap::{Parser, Subcommand};

mod convert;
mod display;
mod error;
mod fetch;
mod info;
mod inspect;
mod parsing;
mod search;
mod tune;
mod tune_trial;

#[derive(Parser, Debug)]
#[command(
    name = "bywind-cli",
    version,
    about = "Native CLI for the bywind sailing-search library",
    long_about = "Load wind maps (GRIB2 / wind_av1), set bounds and \
                  endpoints, run the sailing PSO search, and inspect saved \
                  solutions. See docs/bywind-cli-plan.md for the development \
                  roadmap."
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
#[expect(
    clippy::large_enum_variant,
    reason = "clap Subcommand parse target — allocated once in main, never \
              duplicated or stored. The size imbalance is a one-shot stack \
              cost paid only at startup."
)]
enum Command {
    /// Convert wind maps between supported formats. Auto-detects by extension
    /// (`.grib2` / `.grb2` / `.grib` / `.wcav`). Today this is GRIB2 →
    /// `wind_av1` only; the dispatch is kept in case future formats land.
    Convert {
        /// Input wind map.
        input: PathBuf,
        /// Output path. Format inferred from extension.
        #[arg(long, short = 'o')]
        out: PathBuf,
        /// Decimation factor for GRIB2 input: keep every Nth lat / lon.
        #[arg(long, value_name = "N")]
        grib_stride: Option<usize>,
        /// Lat/lon bbox for GRIB2 input as `lat_min,lon_min,lat_max,lon_max`.
        #[arg(long, value_name = "LAT0,LON0,LAT1,LON1")]
        grib_bbox: Option<String>,
    },

    /// Print metadata (frame count, step seconds, bbox, sample count) about
    /// a wind map.
    Info {
        /// Wind map path (any supported format).
        map: PathBuf,
    },

    /// Run a sailing search and write the gbest path as a `SavedSolution`
    /// JSON document. `map`, `--start`, and `--end` are required, but each
    /// can be supplied via the `[run]` section of a `--config` file
    /// instead of the corresponding CLI flag.
    Search(search::SearchArgs),

    /// Pull a wind-map window from NOAA's GFS S3 bucket and write it
    /// directly to disk as `.grib2` or `.wcav`. Output format is inferred
    /// from the `--out` extension. `<start>` and `<end>` are
    /// `YYYYMMDD[HH]` UTC; `<start>` must be a GFS cycle hour (00, 06, 12,
    /// or 18). `--interval-h` defaults to 1 (seamless 1-hour cadence
    /// using each cycle's f000..f005).
    Fetch(fetch::FetchArgs),

    /// Inspect a saved solution: print metadata and, if a wind map is given,
    /// re-score the gbest path against it (per-segment table + totals).
    /// Doesn't run any search.
    Inspect {
        /// `SavedSolution` JSON path.
        solution: PathBuf,
        /// Wind map for recomputing per-segment fuel / time / speed / land.
        #[arg(long)]
        map: Option<PathBuf>,
    },

    /// Per-trial worker for the PSO-tuning study. Reads a JSON trial
    /// spec from stdin (params + seeds + routes), runs the sailing
    /// search across `routes × seeds` against the pre-baked grids in
    /// `profiling/tuning/<slug>/baked.bk1`, and writes aggregate JSON
    /// to stdout. See `bywind-cli/src/tune_trial.rs` for the input /
    /// output schema. No flags — the JSON spec carries everything.
    TuneTrial,

    /// Drive the PSO-tuning study via TPE Bayesian optimisation.
    /// Spawns `tune-trial` as a subprocess per Optuna-style trial,
    /// normalises raw fitness against the per-route baseline, and
    /// minimises the mean ratio. Persists trials as JSONL so an
    /// interrupted study leaves an inspectable record. Run baseline
    /// measurement first via `tune-trial` with default params, then
    /// pass that JSON via `--baseline`.
    Tune {
        /// Number of optimiser trials to run.
        #[arg(long, default_value_t = 50)]
        trials: usize,
        /// Baseline JSON produced by a `tune-trial` run with default
        /// PSO coefficients across the same route set.
        #[arg(
            long,
            value_name = "PATH",
            default_value = "profiling/tuning/baseline.json"
        )]
        baseline: PathBuf,
        /// Directory containing per-route `<slug>/{config.toml,baked.bk1}`.
        /// Forwarded into each spawned `tune-trial` via the JSON spec.
        #[arg(long, value_name = "PATH", default_value = "profiling/tuning")]
        routes_dir: PathBuf,
        /// Path for the JSONL trial journal.
        #[arg(
            long,
            value_name = "PATH",
            default_value = "profiling/tuning/study.jsonl"
        )]
        journal: PathBuf,
        /// Seed for the `TpeSampler` — controls which params it suggests.
        #[arg(long, default_value_t = 0)]
        sampler_seed: u64,
    },
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    // Each subcommand returns `Result<(), error::AppError>`; the variants of
    // AppError carry the exit-code class (1=BadInput, 2=NoResult,
    // 3=Internal). Unannotated `?` from anyhow contexts auto-converts to
    // BadInput, so most error paths produce exit 1 by default; specific
    // sites in the subcommands construct NoResult / Internal explicitly.
    let result: Result<(), error::AppError> = match cli.command {
        Command::Convert {
            input,
            out,
            grib_stride,
            grib_bbox,
        } => convert::run(&input, &out, grib_stride, grib_bbox.as_deref()),
        Command::Fetch(args) => fetch::run(&args),
        Command::Info { map } => info::run(&map),
        Command::Search(args) => search::run(&args),
        Command::Inspect { solution, map } => inspect::run(&solution, map.as_deref()),
        Command::TuneTrial => tune_trial::run(),
        Command::Tune {
            trials,
            baseline,
            routes_dir,
            journal,
            sampler_seed,
        } => tune::run(&tune::TuneArgs {
            trials,
            baseline,
            routes_dir,
            journal,
            sampler_seed,
            ..tune::TuneArgs::defaults()
        }),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            e.exit_code()
        }
    }
}
