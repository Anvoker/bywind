//! Benchmark the cost of finer `sdf_resolution_deg`.
//!
//! Builds the landmass grid at several resolutions and times:
//! - Grid construction (rasterise + carve-outs + 8SSEDT distance transform)
//! - A* sea-path search for two real scenarios
//! - Bulk signed-distance queries (proxy for the PSO inner loop)
//!
//! Run with:
//! ```
//! cargo run -p bywind-dev --release --bin sdf_resolution_bench
//! ```

use std::time::Instant;

use bywind::landmass::{LandmassGrid, landmass_grid_at_resolution};
use bywind::{LonLatBbox, RouteBounds};
// The bench needs `find_sea_path` / `signed_distance_m`, which live on the
// `LandmassSource` trait. `SeaPathBias` and `LatLon` aren't re-exported at
// the bywind crate root, so reach into swarmkit-sailing through the
// transitive dependency that `bywind`'s public API already exposes.
use swarmkit_sailing::spherical::LatLon;
use swarmkit_sailing::{LandmassSource, SeaPathBias};

const RESOLUTIONS_DEG: &[f64] = &[0.5, 0.25, 0.2, 0.1];

const QUERY_COUNT: usize = 1_000_000;

struct Scenario {
    name: &'static str,
    origin: LatLon,
    destination: LatLon,
    bbox: LonLatBbox,
}

const SCENARIOS: &[Scenario] = &[
    Scenario {
        name: "Sinai (Med -> Arabian Sea, around Africa)",
        origin: LatLon::new(12.894705772399902, 36.113094329833984),
        destination: LatLon::new(55.21006393432617, 10.023117065429688),
        bbox: LonLatBbox {
            lon_min: -32.35,
            lon_max: 69.85,
            lat_min: -49.85,
            lat_max: 52.35,
        },
    },
    Scenario {
        name: "Black Sea -> Bay of Biscay",
        origin: LatLon::new(30.451622009277344, 42.87483596801758),
        destination: LatLon::new(-3.651212692260742, 45.15494155883789),
        bbox: LonLatBbox {
            lon_min: -32.35,
            lon_max: 69.85,
            lat_min: -49.85,
            lat_max: 52.35,
        },
    },
];

fn memory_bytes(cell_deg: f64) -> usize {
    let width = (360.0 / cell_deg).round() as usize;
    let height = (180.0 / cell_deg).round() as usize;
    // sdf_m: f32 (4 bytes) + grad_en: (f32, f32) (8 bytes) per cell.
    width * height * 12
}

fn fmt_bytes(bytes: usize) -> String {
    if bytes >= 1024 * 1024 * 1024 {
        format!("{:.2} GiB", bytes as f64 / (1024.0 * 1024.0 * 1024.0))
    } else if bytes >= 1024 * 1024 {
        format!("{:.2} MiB", bytes as f64 / (1024.0 * 1024.0))
    } else if bytes >= 1024 {
        format!("{:.2} KiB", bytes as f64 / 1024.0)
    } else {
        format!("{bytes} B")
    }
}

fn bench_build(cell_deg: f64) -> std::time::Duration {
    let polygons = bywind::landmass::raw_polygons();
    let start = Instant::now();
    let _grid = LandmassGrid::build(polygons, cell_deg);
    start.elapsed()
}

fn bench_astar(grid: &LandmassGrid, scenario: &Scenario) -> (std::time::Duration, usize) {
    let bounds = RouteBounds::new(scenario.origin, scenario.destination, scenario.bbox);
    let start = Instant::now();
    let path = grid.find_sea_path(
        scenario.origin,
        scenario.destination,
        &bounds,
        SeaPathBias::None,
    );
    let elapsed = start.elapsed();
    let vertices = path.as_ref().map(|p| p.len()).unwrap_or(0);
    (elapsed, vertices)
}

fn bench_bulk_queries(grid: &LandmassGrid, bbox: &LonLatBbox, count: usize) -> std::time::Duration {
    let lon_min = bbox.lon_min;
    let lon_max = bbox.lon_max;
    let lat_min = bbox.lat_min;
    let lat_max = bbox.lat_max;
    // Deterministic linear-congruential pseudo-random points so different
    // runs are comparable.
    let mut state: u64 = 0xDEAD_BEEF_CAFE_F00D;
    let mut sum = 0.0_f64;
    let start = Instant::now();
    for _ in 0..count {
        state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let r1 = ((state >> 32) as f64) / (u32::MAX as f64);
        state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let r2 = ((state >> 32) as f64) / (u32::MAX as f64);
        let lon = lon_min + r1 * (lon_max - lon_min);
        let lat = lat_min + r2 * (lat_max - lat_min);
        sum += grid.signed_distance_m(LatLon::new(lon, lat));
    }
    // Prevent dead-code elimination.
    std::hint::black_box(sum);
    start.elapsed()
}

fn main() {
    println!("# SDF resolution benchmark\n");
    println!("Polygon count: {}", bywind::landmass::raw_polygons().len());
    println!();

    println!("## Grid construction + memory footprint\n");
    println!("| resolution | width × height | est. SDF memory | build time |");
    println!("|---|---|---|---|");
    for &res in RESOLUTIONS_DEG {
        let width = (360.0 / res).round() as usize;
        let height = (180.0 / res).round() as usize;
        let mem = memory_bytes(res);
        let build = bench_build(res);
        println!(
            "| {res}° | {width} × {height} = {} cells | {} | {build:?} |",
            width * height,
            fmt_bytes(mem),
        );
    }
    println!();

    println!("## A* sea-path search (cached grids)\n");
    println!("| resolution | scenario | vertices | A* time |");
    println!("|---|---|---|---|");
    for &res in RESOLUTIONS_DEG {
        let grid = landmass_grid_at_resolution(res);
        for scenario in SCENARIOS {
            let (elapsed, vertices) = bench_astar(grid, scenario);
            println!("| {res}° | {} | {vertices} | {elapsed:?} |", scenario.name);
        }
    }
    println!();

    println!("## Bulk `signed_distance_m` queries ({QUERY_COUNT} random points in Med bbox)\n");
    let med = LonLatBbox {
        lon_min: -10.0,
        lon_max: 40.0,
        lat_min: 30.0,
        lat_max: 46.0,
    };
    println!("| resolution | total time | per-query (ns) |");
    println!("|---|---|---|");
    for &res in RESOLUTIONS_DEG {
        let grid = landmass_grid_at_resolution(res);
        let elapsed = bench_bulk_queries(grid, &med, QUERY_COUNT);
        let per_query_ns = elapsed.as_nanos() as f64 / QUERY_COUNT as f64;
        println!("| {res}° | {elapsed:?} | {per_query_ns:.1} ns |");
    }
}
