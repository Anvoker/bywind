#![allow(
    clippy::float_cmp,
    reason = "tests rely on bit-exact comparisons of constant or stored f32/f64 values."
)]

use crate::{TimedWeatherRow, TimedWindMap, WeatherRow, WindMap, WindSample};

#[test]
fn round_trip_serialization() {
    let original = vec![
        WeatherRow {
            lon: 1.5,
            lat: 2.5,
            sample: WindSample {
                speed: 12.3,
                direction: 270.0,
            },
        },
        WeatherRow {
            lon: 3.0,
            lat: 4.0,
            sample: WindSample {
                speed: 7.8,
                direction: 90.0,
            },
        },
        WeatherRow {
            lon: 5.5,
            lat: 6.5,
            sample: WindSample {
                speed: 20.1,
                direction: 180.0,
            },
        },
    ];

    let json = serde_json::to_string(&original).unwrap();
    let deserialized: Vec<WeatherRow> = serde_json::from_str(&json).unwrap();

    assert_eq!(original, deserialized);
}

#[test]
fn interpolation_midpoint() {
    let map = WindMap::new(vec![
        WeatherRow {
            lon: 0.0,
            lat: 0.0,
            sample: WindSample {
                speed: 10.0,
                direction: 0.0,
            },
        },
        WeatherRow {
            lon: 0.0,
            lat: 2.0,
            sample: WindSample {
                speed: 20.0,
                direction: 90.0,
            },
        },
    ]);

    let result = map.query(0.0, 1.0);

    let epsilon = 1e-4;
    assert!(
        (result.speed - 15.0).abs() < epsilon,
        "speed was {}",
        result.speed
    );
    assert!(
        (result.direction - 45.0).abs() < epsilon,
        "direction was {}",
        result.direction
    );
}

fn single_point_frame(speed: f32, direction: f32) -> WindMap {
    WindMap::new(vec![WeatherRow {
        lon: 0.0,
        lat: 0.0,
        sample: WindSample { speed, direction },
    }])
}

#[test]
fn timed_query_at_frame_boundary_returns_frame() {
    let map = TimedWindMap::new(
        vec![
            single_point_frame(10.0, 0.0),
            single_point_frame(20.0, 90.0),
            single_point_frame(30.0, 180.0),
        ],
        3600.0,
    );

    for (t, expected_speed, expected_dir) in [
        (0.0, 10.0, 0.0),
        (3600.0, 20.0, 90.0),
        (7200.0, 30.0, 180.0),
    ] {
        let s = map.query(0.0, 0.0, t);
        assert!(
            (s.speed - expected_speed).abs() < 1e-4,
            "speed at t={t}: {}",
            s.speed
        );
        assert!(
            (s.direction - expected_dir).abs() < 1e-4,
            "dir at t={t}: {}",
            s.direction
        );
    }
}

#[test]
fn timed_query_midpoint_interpolates() {
    let map = TimedWindMap::new(
        vec![
            single_point_frame(10.0, 0.0),
            single_point_frame(20.0, 90.0),
        ],
        3600.0,
    );

    let s = map.query(0.0, 0.0, 1800.0);
    assert!((s.speed - 15.0).abs() < 1e-4, "speed was {}", s.speed);
    assert!(
        (s.direction - 45.0).abs() < 1e-4,
        "direction was {}",
        s.direction
    );
}

#[test]
fn timed_query_wraps_out_of_range_t() {
    // Pre-crossfade behaviour clamped out-of-range `t` to the first /
    // last frame. The current rule wraps via `rem_euclid` over
    // `cycle_seconds = duration + crossfade`. The crossfade endpoint
    // tests under `wind_map::crossfade_tests` cover the seam shape;
    // here we just pin that any `t` is reachable (no panic, no
    // clamp), and that wrapping past the cycle is idempotent.
    let map = TimedWindMap::new(
        vec![
            single_point_frame(10.0, 0.0),
            single_point_frame(20.0, 90.0),
        ],
        3600.0,
    );

    let baseline = map.query(0.0, 0.0, 1800.0);
    let one_cycle_later = map.query(0.0, 0.0, 1800.0 + map.cycle_seconds());
    assert!(
        (baseline.speed - one_cycle_later.speed).abs() < 1e-4,
        "speed: {} vs {}",
        baseline.speed,
        one_cycle_later.speed,
    );
    let dir_diff = ((baseline.direction - one_cycle_later.direction + 540.0) % 360.0) - 180.0;
    assert!(dir_diff.abs() < 1e-3);
}

#[test]
fn timed_round_trip_via_timed_rows() {
    let original = TimedWindMap::new(
        vec![
            WindMap::new(vec![
                WeatherRow {
                    lon: 0.0,
                    lat: 0.0,
                    sample: WindSample {
                        speed: 1.0,
                        direction: 10.0,
                    },
                },
                WeatherRow {
                    lon: 1.0,
                    lat: 0.0,
                    sample: WindSample {
                        speed: 2.0,
                        direction: 20.0,
                    },
                },
            ]),
            WindMap::new(vec![
                WeatherRow {
                    lon: 0.0,
                    lat: 0.0,
                    sample: WindSample {
                        speed: 3.0,
                        direction: 30.0,
                    },
                },
                WeatherRow {
                    lon: 1.0,
                    lat: 0.0,
                    sample: WindSample {
                        speed: 4.0,
                        direction: 40.0,
                    },
                },
            ]),
        ],
        1800.0,
    );

    let rows = original.to_timed_rows();
    let json = serde_json::to_string(&rows).unwrap();
    let read: Vec<TimedWeatherRow> = serde_json::from_str(&json).unwrap();
    let rebuilt = TimedWindMap::from_timed_rows(read).expect("non-empty");

    assert_eq!(rebuilt.frame_count(), 2);
    assert_eq!(rebuilt.step_seconds(), 1800.0);
    assert_eq!(
        rebuilt.frame(0).unwrap().rows(),
        original.frame(0).unwrap().rows()
    );
    assert_eq!(
        rebuilt.frame(1).unwrap().rows(),
        original.frame(1).unwrap().rows()
    );
}

/// Build a 2D grid of `nx × ny` samples at spacing `step`. Speed encodes
/// `(i, j)` so tests can assert specific cells came back.
fn grid_map(nx: usize, ny: usize, step: f32) -> WindMap {
    let mut rows = Vec::with_capacity(nx * ny);
    for i in 0..nx {
        for j in 0..ny {
            rows.push(WeatherRow {
                lon: i as f32 * step,
                lat: j as f32 * step,
                sample: WindSample {
                    speed: (i * 100 + j) as f32,
                    direction: 0.0,
                },
            });
        }
    }
    WindMap::new(rows)
}

#[test]
fn grid_query_at_grid_point_returns_exact_sample() {
    let map = grid_map(4, 3, 10.0);
    let s = map.query(20.0, 10.0); // (i=2, j=1)
    assert_eq!(s.speed, 201.0);
}

#[test]
fn grid_query_at_cell_center_idw_blends_four_corners() {
    // 2×2 grid at 10-unit spacing. Center is equidistant from all 4 corners,
    // so IDW gives the plain mean of their speeds.
    let map = WindMap::new(vec![
        WeatherRow {
            lon: 0.0,
            lat: 0.0,
            sample: WindSample {
                speed: 10.0,
                direction: 0.0,
            },
        },
        WeatherRow {
            lon: 10.0,
            lat: 0.0,
            sample: WindSample {
                speed: 20.0,
                direction: 0.0,
            },
        },
        WeatherRow {
            lon: 0.0,
            lat: 10.0,
            sample: WindSample {
                speed: 30.0,
                direction: 0.0,
            },
        },
        WeatherRow {
            lon: 10.0,
            lat: 10.0,
            sample: WindSample {
                speed: 40.0,
                direction: 0.0,
            },
        },
    ]);
    let s = map.query(5.0, 5.0);
    assert!((s.speed - 25.0).abs() < 1e-4, "speed was {}", s.speed);
}

#[test]
fn grid_detection_handles_shuffled_input_order() {
    // Same 4 corners as above but in a non-canonical order. Detection should
    // still recognise the grid and reorder rows so queries land on the right cell.
    let map = WindMap::new(vec![
        WeatherRow {
            lon: 10.0,
            lat: 10.0,
            sample: WindSample {
                speed: 40.0,
                direction: 0.0,
            },
        },
        WeatherRow {
            lon: 0.0,
            lat: 0.0,
            sample: WindSample {
                speed: 10.0,
                direction: 0.0,
            },
        },
        WeatherRow {
            lon: 10.0,
            lat: 0.0,
            sample: WindSample {
                speed: 20.0,
                direction: 0.0,
            },
        },
        WeatherRow {
            lon: 0.0,
            lat: 10.0,
            sample: WindSample {
                speed: 30.0,
                direction: 0.0,
            },
        },
    ]);
    assert_eq!(map.query(0.0, 0.0).speed, 10.0);
    assert_eq!(map.query(10.0, 10.0).speed, 40.0);
}

#[test]
fn grid_query_circle_indices_returns_points_within_radius() {
    let map = grid_map(5, 5, 10.0); // points at (0,0),(0,10),...,(40,40)
    let mut indices = map.query_circle_indices(20.0, 20.0, 12.0);
    indices.sort_unstable();
    // The 5 points within 12 units of (20,20): (20,20) itself + the 4 axis-aligned
    // neighbours at distance 10. The diagonals at distance ≈14.14 are excluded.
    let expected: Vec<usize> = [
        2 * 5 + 2, // (20, 20)
        5 + 2,     // (10, 20)
        3 * 5 + 2, // (30, 20)
        2 * 5 + 1, // (20, 10)
        2 * 5 + 3, // (20, 30)
    ]
    .into_iter()
    .collect::<Vec<_>>();
    let mut expected = expected;
    expected.sort_unstable();
    assert_eq!(indices, expected);
}

#[test]
fn grid_construction_does_not_panic_for_size_that_breaks_kdtree() {
    // 200×200 = 40 000 collinear-per-axis points. With either kiddo backend in
    // play this would either overflow the bucket size (KdTree) or trip the
    // shift-overflow in optimize_stems (ImmutableKdTree). Grid detection must
    // bypass both.
    let map = grid_map(200, 200, 1.0);
    let s = map.query(150.5, 80.5);
    assert!(s.speed.is_finite());
}

#[test]
fn grid_construction_handles_global_scale_coordinates() {
    // Reproduces the GFS-loader scenario: a regular lat/lon grid projected
    // to metres puts row positions at ±2e7 m, where f32 has only ~2 m of
    // precision. The cumulative `origin + i*step` predictor that
    // detect_grid_layout used to apply drifted by kilometres over thousands
    // of columns and rejected the grid, falling through to the kd-tree path
    // and tripping kiddo's `optimize_stems` overflow. The local-step check
    // we use instead must accept this grid.
    let nx = 1440usize;
    let ny = 721usize;
    let step_lon_deg = 0.25_f32;
    let step_lat_deg = 0.25_f32;
    let metres_per_degree = 111_320.0_f32;
    let mut rows = Vec::with_capacity(nx * ny);
    for i in 0..nx {
        for j in 0..ny {
            let lon = -180.0 + (i as f32) * step_lon_deg;
            let lat = -90.0 + (j as f32) * step_lat_deg;
            rows.push(WeatherRow {
                lon: lon * metres_per_degree,
                lat: lat * metres_per_degree,
                sample: WindSample {
                    speed: 1.0,
                    direction: 0.0,
                },
            });
        }
    }
    let map = WindMap::new(rows);
    let s = map.query(0.0, 0.0);
    assert!(
        s.speed.is_finite(),
        "global-scale grid must take the grid path, not kd-tree"
    );
}

#[test]
fn from_timed_rows_single_frame_uses_default_step() {
    let rows = vec![
        TimedWeatherRow {
            lon: 0.0,
            lat: 0.0,
            t_seconds: 0.0,
            sample: WindSample {
                speed: 5.0,
                direction: 0.0,
            },
        },
        TimedWeatherRow {
            lon: 1.0,
            lat: 0.0,
            t_seconds: 0.0,
            sample: WindSample {
                speed: 6.0,
                direction: 0.0,
            },
        },
    ];
    let map = TimedWindMap::from_timed_rows(rows).expect("non-empty");
    assert_eq!(map.frame_count(), 1);
    assert!(map.step_seconds() > 0.0, "step_seconds must be positive");
}
