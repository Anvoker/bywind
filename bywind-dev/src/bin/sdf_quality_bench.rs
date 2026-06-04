//! Cross-resolution route-quality benchmark for `sdf_resolution_deg`.
//!
//! For each (scenario, resolution, seed), runs the full PSO search and
//! records the gbest's totals (time, fuel, land, fitness) and the
//! benchmark route's totals. Aggregates over seeds so we can see whether
//! finer SDF resolution actually changes the optimum or just shuffles
//! the noise.
//!
//! Wind map + bake are reused across all runs in a scenario; the
//! per-resolution `LandmassGrid` is cached in `OnceLock` so the build
//! cost lands on the first seed only.
//!
//! Run with:
//! ```
//! cargo run -p bywind-dev --release --bin sdf_quality_bench <map.grib2>
//! ```

use std::time::Instant;

use bywind::{
    BakedWindMap, BoatConfig, LonLatBbox, MapBounds, RouteBounds, SearchConfig, SearchResult,
    SearchWeights, TimedWindMap, Topology, WaypointCount, gbest_segment_metrics,
    run_search_blocking_with_baked,
};
use swarmkit_sailing::spherical::LatLon;

const SEEDS: &[u64] = &[1, 2, 3, 4, 5, 6, 7, 8];
const RESOLUTIONS_DEG: &[f64] = &[0.5, 0.2];
const BAKE_STEP_DEG: f64 = 0.25;

struct Scenario {
    name: &'static str,
    start: LatLon,
    end: LatLon,
    bbox: LonLatBbox,
    waypoints: WaypointCount,
    topology: Topology,
}

fn scenarios() -> [Scenario; 2] {
    [
        Scenario {
            name: "Sinai (Med -> Arabian Sea)",
            start: LatLon::new(12.894705772399902, 36.113094329833984),
            end: LatLon::new(55.21006393432617, 10.023117065429688),
            bbox: LonLatBbox {
                lon_min: -32.35,
                lon_max: 69.85,
                lat_min: -49.85,
                lat_max: 52.35,
            },
            waypoints: WaypointCount::N40,
            topology: Topology::VonNeumann,
        },
        Scenario {
            name: "Black Sea -> Bay of Biscay",
            start: LatLon::new(30.451622009277344, 42.87483596801758),
            end: LatLon::new(-3.651212692260742, 45.15494155883789),
            bbox: LonLatBbox {
                lon_min: -32.35,
                lon_max: 69.85,
                lat_min: -49.85,
                lat_max: 52.35,
            },
            waypoints: WaypointCount::N60,
            topology: Topology::Ring,
        },
    ]
}

fn boat_cfg() -> BoatConfig {
    BoatConfig {
        mcr_kw: 1000.0,
        k: 4000.0,
        polar_c: 1.5,
        polar_sin_power: 1.0,
        fuel_a: 0.0875,
        fuel_b: -0.0555,
        fuel_c: 0.0347,
    }
}

fn search_cfg_for(scenario: &Scenario, sdf_resolution_deg: f64, seed: u64) -> SearchConfig {
    SearchConfig {
        waypoint_count: scenario.waypoints,
        time_weight: 1.0,
        fuel_weight: 10.0,
        land_weight: 2000.0,
        particles_space: 100,
        particles_time: 40,
        iter_space: 80,
        iter_time: 30,
        inertia: 0.33,
        cognitive_coeff: 2.0,
        social_coeff: 2.0,
        path_kick_probability: 0.1,
        path_kick_gamma_0_fraction: 0.05,
        path_kick_gamma_min_fraction: 0.005,
        seed: Some(seed),
        sdf_resolution_deg,
        topology: scenario.topology,
        ..SearchConfig::default()
    }
}

#[derive(Clone, Copy)]
struct RunMetrics {
    time_s: f64,
    fuel_kg: f64,
    land_m: f64,
    fitness: f64,
    search_secs: f64,
}

fn aggregate(samples: &[RunMetrics]) -> (f64, f64, f64, f64) {
    // Returns (mean, stddev, min, max) of fitness.
    let n = samples.len() as f64;
    let mean = samples.iter().map(|s| s.fitness).sum::<f64>() / n;
    let var = samples
        .iter()
        .map(|s| {
            let d = s.fitness - mean;
            d * d
        })
        .sum::<f64>()
        / n;
    let stddev = var.sqrt();
    let min = samples
        .iter()
        .map(|s| s.fitness)
        .fold(f64::INFINITY, f64::min);
    let max = samples
        .iter()
        .map(|s| s.fitness)
        .fold(f64::NEG_INFINITY, f64::max);
    (mean, stddev, min, max)
}

fn fmt_duration_d(secs: f64) -> String {
    let total = secs as i64;
    let days = total / 86400;
    let hours = (total % 86400) / 3600;
    let minutes = (total % 3600) / 60;
    format!("{days}d {hours:02}h {minutes:02}m")
}

fn run_scenario(scenario: &Scenario, wind: &TimedWindMap) {
    println!("\n## {}\n", scenario.name);

    let wind_bounds = MapBounds::from_wind_map(wind).expect("wind has no rows");
    // Bake covers the route bbox (we trust the user's bbox sits inside the
    // wind extent for these hard-coded scenarios — the GFS map is global).
    let _ = wind_bounds;
    let bake_bounds = MapBounds {
        bbox: scenario.bbox,
    }
    .to_bake_bounds(BAKE_STEP_DEG);
    let bake_start = Instant::now();
    let mut baked: Option<BakedWindMap> = Some(wind.bake(bake_bounds));
    println!("  bake: {:.2}s", bake_start.elapsed().as_secs_f64());

    let route_bounds = RouteBounds::new(scenario.start, scenario.end, scenario.bbox);

    println!(
        "\n| res | seed | search time | PSO time | PSO fuel | PSO land | PSO fitness | bench fit | PSO better |"
    );
    println!("|---|---|---|---|---|---|---|---|---|");

    let mut by_res: Vec<(f64, Vec<RunMetrics>)> = Vec::new();
    for &res in RESOLUTIONS_DEG {
        let mut samples: Vec<RunMetrics> = Vec::new();
        for &seed in SEEDS {
            let cfg = search_cfg_for(scenario, res, seed);
            let settings = cfg.to_search_settings();
            let weights = SearchWeights {
                time_weight: cfg.time_weight,
                fuel_weight: cfg.fuel_weight,
                land_weight: cfg.land_weight,
            };
            let ship = boat_cfg().to_boat();
            let start = Instant::now();
            let result = run_search_blocking_with_baked(
                bywind::WindInput::single(baked.take().expect("baked threaded")),
                route_bounds,
                cfg.waypoint_count,
                settings,
                ship,
                weights,
                res,
                // Single-tier for the bench so the resolution sweep is
                // apples-to-apples — adding a fine tier per resolution
                // would conflate two effects.
                None,
                &mut |_| {},
            )
            .expect("search failed");
            let search_secs = start.elapsed().as_secs_f64();
            let SearchResult {
                route_evolution,
                route_bounds: route_bounds_out,
                baked: baked_back,
                boat,
                benchmark,
                ..
            } = result;
            baked = Some(baked_back);

            let last_iter = route_evolution.iter_count().saturating_sub(1);
            let stats = gbest_segment_metrics(
                &route_evolution,
                last_iter,
                &boat,
                baked.as_ref().expect("baked"),
                route_bounds_out.step_distance_max,
            )
            .expect("no iters");
            let total_time: f64 = stats.iter().map(|s| s.time).sum();
            let total_fuel: f64 = stats.iter().map(|s| s.fuel).sum();
            let total_land: f64 = stats.iter().map(|s| s.land_metres).sum();
            // Final gbest fitness = the value the search converged on.
            let fitness = route_evolution
                .gbest_at(last_iter)
                .map(|g| g.best_fit)
                .expect("gbest");
            let bench_fitness = benchmark.as_ref().map(|b| b.fitness);
            let pso_better = bench_fitness.map_or(0.0, |bf| {
                if bf.abs() < 1e-9 {
                    0.0
                } else {
                    (fitness - bf) / bf.abs() * 100.0
                }
            });

            println!(
                "| {res}° | {seed} | {:.1}s | {} | {:.1} t | {:.1} km | {:.0} | {} | {:+.1}% |",
                search_secs,
                fmt_duration_d(total_time),
                total_fuel / 1000.0,
                total_land / 1000.0,
                fitness,
                bench_fitness.map_or("—".to_string(), |b| format!("{b:.0}")),
                pso_better,
            );
            samples.push(RunMetrics {
                time_s: total_time,
                fuel_kg: total_fuel,
                land_m: total_land,
                fitness,
                search_secs,
            });
        }
        by_res.push((res, samples));
    }

    println!("\n### Aggregate over {} seeds\n", SEEDS.len());
    println!(
        "| res | mean fit | stddev | min | max | mean time | mean fuel | mean land | mean search |"
    );
    println!("|---|---|---|---|---|---|---|---|---|");
    for (res, samples) in &by_res {
        let (mean, stddev, min, max) = aggregate(samples);
        let mean_time: f64 = samples.iter().map(|s| s.time_s).sum::<f64>() / samples.len() as f64;
        let mean_fuel: f64 = samples.iter().map(|s| s.fuel_kg).sum::<f64>() / samples.len() as f64;
        let mean_land: f64 = samples.iter().map(|s| s.land_m).sum::<f64>() / samples.len() as f64;
        let mean_search: f64 =
            samples.iter().map(|s| s.search_secs).sum::<f64>() / samples.len() as f64;
        println!(
            "| {res}° | {mean:.0} | {stddev:.0} | {min:.0} | {max:.0} | {} | {:.1} t | {:.1} km | {mean_search:.1}s |",
            fmt_duration_d(mean_time),
            mean_fuel / 1000.0,
            mean_land / 1000.0,
        );
    }
}

fn main() {
    let map_path = std::env::args().nth(1).unwrap_or_else(|| {
        "C:/Projects/GameDev/grib-data/gfs.t06z.0p25.f000-f024.grib2".to_string()
    });
    println!("# SDF resolution × route quality benchmark\n");
    println!("Loading wind map: {map_path}");
    let load_start = Instant::now();
    let wind = bywind::io::load(std::path::Path::new(&map_path), 1, None).expect("load wind");
    println!("  loaded in {:.2}s\n", load_start.elapsed().as_secs_f64());

    for scenario in scenarios() {
        run_scenario(&scenario, &wind);
    }
}
