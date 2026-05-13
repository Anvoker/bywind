//! `inspect` subcommand: load a `SavedSolution` JSON and print metadata
//! about it. With `--map`, also re-score the saved gbest path against the
//! given wind map and print the per-segment table + totals — same shape
//! as `search`'s summary output, minus the search-timing rows and the A*
//! benchmark (neither is stored in `SavedSolution`).
//!
//! Doesn't run any search. Doesn't re-optimise time. Pure read-and-format.

use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use std::time::Instant;

use anyhow::{Context as _, Result, anyhow, bail};
use bywind::{
    BAKE_STEP, MapBounds, SavedSolution, SegmentMetrics,
    fmt::{format_duration_breakdown, format_fuel_auto, format_land_km},
    gbest_segment_metrics,
};

use crate::display::print_segment_table;
use crate::error::AppError;

pub fn run(solution_path: &Path, map_path: Option<&Path>) -> Result<(), AppError> {
    let saved = read_solution(solution_path)?;
    print_solution_metadata(solution_path, &saved);

    if let Some(map_path) = map_path {
        let segment_stats = score_against_map(&saved, map_path)?;
        print_route_summary(&saved, &segment_stats);
    } else {
        eprintln!();
        eprintln!("(pass --map <wind_map> for per-segment fuel / time / speed / land totals)");
    }

    Ok(())
}

fn read_solution(path: &Path) -> Result<SavedSolution> {
    let file = File::open(path).with_context(|| format!("opening {}", path.display()))?;
    let reader = BufReader::new(file);
    serde_json::from_reader(reader)
        .with_context(|| format!("parsing SavedSolution from {}", path.display()))
}

/// Build the search inputs the saved solution implies — derive `MapBounds`
/// from the wind map, build a `RouteBounds` whose origin/destination match
/// the saved gbest's first/last waypoints (so the reconstructed
/// `RouteEvolution` survives the `to_route_evolution` length checks), bake
/// the wind map, then call `gbest_segment_metrics` on the rebuilt evolution.
fn score_against_map(
    saved: &SavedSolution,
    map_path: &Path,
) -> Result<Vec<SegmentMetrics>, AppError> {
    eprintln!();
    eprintln!("loading {}...", map_path.display());
    let load_start = Instant::now();
    let map = bywind::io::load(map_path, 1, None)
        .with_context(|| format!("loading wind map from {}", map_path.display()))?;
    eprintln!(
        "  loaded in {:.2}s: {} frame(s), step = {} s",
        load_start.elapsed().as_secs_f64(),
        map.frame_count(),
        map.step_seconds(),
    );

    let map_bounds = MapBounds::from_wind_map(&map).ok_or_else(|| {
        AppError::no_result(anyhow!("wind map {} has no rows", map_path.display()))
    })?;
    if !map_bounds.is_non_degenerate() {
        return Err(AppError::no_result(anyhow!(
            "wind map {} has a degenerate bbox; can't bake for re-scoring",
            map_path.display(),
        )));
    }

    // Rebuild the const-generic `Path<N>` from the saved arrays, wrapped in
    // a single-frame `RouteEvolution` so we can reuse `gbest_segment_metrics`.
    // `solution::LoadError` doesn't auto-convert to `AppError`, but it impls
    // `std::error::Error` so anyhow can wrap it; from there `?` lands on the
    // standard `From<anyhow::Error>` impl as `BadInput`.
    let (_wc, route_evolution) = saved.to_route_evolution().map_err(anyhow::Error::from)?;

    // Reconstruct origin/destination from the path's endpoints so the route
    // bounds are self-consistent. Interior waypoints aren't re-clamped — the
    // saved path is ground truth.
    let (start_xy, end_xy) = endpoints_from_saved(saved)?;
    let route_bounds = map_bounds.to_route_bounds(start_xy, end_xy);
    let bake_bounds = map_bounds.to_bake_bounds(BAKE_STEP);

    eprintln!("baking wind map for re-scoring...");
    let bake_start = Instant::now();
    let baked = map.bake(bake_bounds);
    eprintln!("  baked in {:.2}s", bake_start.elapsed().as_secs_f64());

    // Use the GUI's default boat for re-scoring. A future enhancement could
    // round-trip the boat config through SavedSolution; for now the tool
    // surfaces what's stored, which doesn't include boat polar parameters.
    let boat = bywind::BoatConfig::default().to_boat();

    let metrics = gbest_segment_metrics(
        &route_evolution,
        0,
        &boat,
        &baked,
        route_bounds.step_distance_max,
    )
    .ok_or_else(|| AppError::internal(anyhow!("rebuilt route evolution has no iterations")))?;
    Ok(metrics)
}

fn endpoints_from_saved(saved: &SavedSolution) -> Result<((f64, f64), (f64, f64))> {
    let n = saved.n;
    if saved.xs.len() != n || saved.ys.len() != n || n < 2 {
        bail!(
            "saved solution has unexpected shape: n={n}, xs={}, ys={}",
            saved.xs.len(),
            saved.ys.len(),
        );
    }
    let (Some(&start_x), Some(&start_y), Some(&end_x), Some(&end_y)) = (
        saved.xs.first(),
        saved.ys.first(),
        saved.xs.last(),
        saved.ys.last(),
    ) else {
        bail!("saved solution has empty coordinate arrays");
    };
    Ok(((start_x, start_y), (end_x, end_y)))
}

fn print_solution_metadata(path: &Path, saved: &SavedSolution) {
    eprintln!("=== Saved solution ===");
    eprintln!("Path:            {}", path.display());
    eprintln!("Waypoints:       {}", saved.n);
    eprintln!("Fitness:         {:.4}", saved.best_fit);
    eprintln!();
    eprintln!("Search params used to produce it:");
    eprintln!("  time_weight:     {}", saved.time_weight);
    eprintln!("  fuel_weight:     {}", saved.fuel_weight);
    eprintln!("  particles_space: {}", saved.particles_space);
    eprintln!("  particles_time:  {}", saved.particles_time);
    eprintln!("  iter_space:      {}", saved.iter_space);
    eprintln!("  iter_time:       {}", saved.iter_time);
    eprintln!("  topology:        {}", saved.topology);
    if let Some(seed) = saved.seed {
        eprintln!("  seed:            {seed}");
    }
    eprintln!(
        "  path_kick_probability:        {}",
        saved.path_kick_probability
    );
    eprintln!(
        "  path_kick_gamma_0_fraction:   {}",
        saved.path_kick_gamma_0_fraction
    );
    eprintln!(
        "  path_kick_gamma_min_fraction: {}",
        saved.path_kick_gamma_min_fraction
    );
    if let (Some(&start_x), Some(&start_y), Some(&end_x), Some(&end_y)) = (
        saved.xs.first(),
        saved.ys.first(),
        saved.xs.last(),
        saved.ys.last(),
    ) {
        eprintln!();
        eprintln!("Route endpoints: ({start_x:.3}, {start_y:.3}) -> ({end_x:.3}, {end_y:.3})",);
    }
}

fn print_route_summary(saved: &SavedSolution, segment_stats: &[SegmentMetrics]) {
    let total_time: f64 = segment_stats.iter().map(|m| m.time).sum();
    let total_fuel: f64 = segment_stats.iter().map(|m| m.fuel).sum();
    let total_land_metres: f64 = segment_stats.iter().map(|m| m.land_metres).sum();

    eprintln!();
    eprintln!("=== Route (re-scored against --map) ===");
    eprintln!("Total time: {}", format_duration_breakdown(total_time));
    eprintln!("Total fuel: {}", format_fuel_auto(total_fuel));
    eprintln!("Total land: {}", format_land_km(total_land_metres));
    eprintln!(
        "Fitness:    {:.4}  (from JSON; not recomputed)",
        saved.best_fit
    );

    eprintln!();
    eprintln!("=== Segments ===");
    print_segment_table(segment_stats);
}
