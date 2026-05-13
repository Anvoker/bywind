use kiddo::{ImmutableKdTree, SquaredEuclidean};
use rayon::prelude::*;
use std::f64::consts::PI;
use swarmkit_sailing::WindSource;
use swarmkit_sailing::spherical::{LatLon, LonLatBbox, Wind};

use crate::{TimedWeatherRow, WeatherRow, WindSample};

/// Canonicalise to `(−180, 180]`. Indexes use this canonical form, so
/// antimeridian-wrap queries (e.g. `lon = 185°`) must too — otherwise
/// they clamp to the edge instead of hitting the right cell.
#[expect(
    clippy::float_cmp,
    reason = "antimeridian boundary check on an exactly-representable -180.0."
)]
fn wrap_lon_query(lon: f32) -> f32 {
    let wrapped = ((lon + 540.0).rem_euclid(360.0)) - 180.0;
    if wrapped == -180.0 { 180.0 } else { wrapped }
}

/// Spatial index over `WindMap` samples. Regular grids — the common
/// case from GRIB2 / generated data — break kd-trees (kiddo panics at
/// sufficient grid size: every column shares an x), so we detect grid
/// structure at construction and switch to direct indexing.
#[derive(Clone)]
enum SpatialIndex {
    /// `(i, j)` cell at `(origin + (i, j) * step)`, stored at
    /// `rows[i * ny + j]`.
    Grid {
        origin_x: f32,
        origin_y: f32,
        step_x: f32,
        step_y: f32,
        nx: usize,
        ny: usize,
    },
    Tree(ImmutableKdTree<f32, 2>),
}

/// Regular-grid metadata for direct sub-rect iteration via
/// `rows()[i * ny + j]`. Returned by [`WindMap::grid_layout`] when the
/// map is grid-backed.
#[derive(Clone, Copy, Debug)]
pub struct GridLayout {
    /// Longitude (degrees) of the `i = 0` column.
    pub origin_x: f32,
    /// Latitude (degrees) of the `j = 0` row.
    pub origin_y: f32,
    /// Longitudinal spacing between adjacent columns (degrees).
    pub step_x: f32,
    /// Latitudinal spacing between adjacent rows (degrees).
    pub step_y: f32,
    /// Number of columns (longitude samples).
    pub nx: usize,
    /// Number of rows (latitude samples).
    pub ny: usize,
}

/// 2D wind field at one instant, queryable at arbitrary `(lon°, lat°)`.
///
/// [`WindMap::new`] auto-detects uniform-grid input and stores
/// `(origin, step, nx, ny)` for O(1) cell lookup; otherwise falls back
/// to a kd-tree. [`WindMap::query`] returns an IDW-interpolated sample
/// over the four nearest neighbours; direction is interpolated as a
/// circular quantity (sin/cos + `atan2`) so the 0°/360° boundary stitches.
/// For time-varying wind see [`TimedWindMap`].
#[derive(Clone)]
pub struct WindMap {
    index: SpatialIndex,
    /// `Grid`: column-major `[i * ny + j]`. `Tree`: construction order.
    rows: Vec<WeatherRow>,
}

impl WindMap {
    pub fn generate(size_x: f32, size_y: f32, density: f32) -> Self {
        let cols = (size_x / density).floor() as usize + 1;
        let rows_count = (size_y / density).floor() as usize + 1;
        let mut rows = Vec::with_capacity(cols * rows_count);
        for i in 0..cols {
            for j in 0..rows_count {
                rows.push(WeatherRow {
                    lon: i as f32 * density,
                    lat: j as f32 * density,
                    sample: WindSample {
                        speed: 0.0,
                        direction: 0.0,
                    },
                });
            }
        }
        Self::new(rows)
    }

    pub fn generate_random(
        size_x: f32,
        size_y: f32,
        density: f32,
        speed_range: std::ops::Range<f32>,
    ) -> Self {
        Self::generate_random_with_rng(size_x, size_y, density, speed_range, &mut rand::rng())
    }

    /// Same as [`Self::generate_random`] but takes a caller-supplied RNG.
    /// Use a seeded `SmallRng` for reproducible synthetic maps in tests.
    pub fn generate_random_with_rng<R: rand::Rng + rand::RngExt>(
        size_x: f32,
        size_y: f32,
        density: f32,
        speed_range: std::ops::Range<f32>,
        rng: &mut R,
    ) -> Self {
        let cols = (size_x / density).floor() as usize + 1;
        let rows_count = (size_y / density).floor() as usize + 1;
        let mut rows = Vec::with_capacity(cols * rows_count);
        for i in 0..cols {
            for j in 0..rows_count {
                rows.push(WeatherRow {
                    lon: i as f32 * density,
                    lat: j as f32 * density,
                    sample: WindSample {
                        speed: rng.random_range(speed_range.clone()),
                        direction: rng.random_range(0.0..360.0),
                    },
                });
            }
        }
        Self::new(rows)
    }

    pub fn new(rows: Vec<WeatherRow>) -> Self {
        if let Some(layout) = detect_grid_layout(&rows) {
            let rows = reorder_to_grid(rows, &layout);
            return Self {
                index: SpatialIndex::Grid {
                    origin_x: layout.origin_x,
                    origin_y: layout.origin_y,
                    step_x: layout.step_x,
                    step_y: layout.step_y,
                    nx: layout.nx,
                    ny: layout.ny,
                },
                rows,
            };
        }
        let positions: Vec<[f32; 2]> = rows.iter().map(|r| [r.lon, r.lat]).collect();
        let tree = ImmutableKdTree::new_from_slice(&positions);
        Self {
            index: SpatialIndex::Tree(tree),
            rows,
        }
    }

    /// Fast path for callers that already have rows in column-major
    /// grid order. ~50× faster than [`Self::new`] on million-cell
    /// frames; debug-asserts the row count.
    pub fn from_grid(rows: Vec<WeatherRow>, layout: GridLayout) -> Self {
        debug_assert_eq!(
            rows.len(),
            layout.nx * layout.ny,
            "row count must match grid layout",
        );
        Self {
            index: SpatialIndex::Grid {
                origin_x: layout.origin_x,
                origin_y: layout.origin_y,
                step_x: layout.step_x,
                step_y: layout.step_y,
                nx: layout.nx,
                ny: layout.ny,
            },
            rows,
        }
    }

    pub fn query(&self, x: f32, y: f32) -> WindSample {
        // Canonicalise lon so antimeridian-wrap queries hit the right
        // cell instead of clamping to the edge.
        let x = wrap_lon_query(x);
        match &self.index {
            SpatialIndex::Grid {
                origin_x,
                origin_y,
                step_x,
                step_y,
                nx,
                ny,
            } => {
                let nx = *nx;
                let ny = *ny;
                let fx = (x - origin_x) / step_x;
                let fy = (y - origin_y) / step_y;
                // Cell corners clamped to the grid. `detect_grid_layout`
                // enforces nx, ny ≥ 2.
                let i_lo = fx.floor().clamp(0.0, (nx - 1) as f32) as usize;
                let i_hi = fx.ceil().clamp(0.0, (nx - 1) as f32) as usize;
                let j_lo = fy.floor().clamp(0.0, (ny - 1) as f32) as usize;
                let j_hi = fy.ceil().clamp(0.0, (ny - 1) as f32) as usize;
                let corners = [
                    i_lo * ny + j_lo,
                    i_hi * ny + j_lo,
                    i_lo * ny + j_hi,
                    i_hi * ny + j_hi,
                ];
                idw_blend(&self.rows, &corners, x, y)
            }
            SpatialIndex::Tree(tree) => {
                // Empty WindMap → zero sample. `nearest_n` requires
                // `NonZero<usize>` (kiddo 5+).
                let Some(k) = std::num::NonZero::new(self.rows.len().min(4)) else {
                    return WindSample {
                        speed: 0.0,
                        direction: 0.0,
                    };
                };
                let nearest = tree.nearest_n::<SquaredEuclidean>(&[x, y], k);
                let mut weight_sum = 0.0f32;
                let mut speed_sum = 0.0f32;
                let mut sin_sum = 0.0f32;
                let mut cos_sum = 0.0f32;
                for neighbor in &nearest {
                    // distance is squared, so 1/d² is IDW with power=2.
                    if neighbor.distance == 0.0 {
                        let sample = &self.rows[neighbor.item as usize].sample;
                        return WindSample {
                            speed: sample.speed,
                            direction: sample.direction,
                        };
                    }
                    let w = 1.0 / neighbor.distance;
                    let sample = &self.rows[neighbor.item as usize].sample;
                    weight_sum += w;
                    speed_sum += w * sample.speed;
                    let dir_rad = sample.direction.to_radians();
                    sin_sum += w * dir_rad.sin();
                    cos_sum += w * dir_rad.cos();
                }
                WindSample {
                    speed: speed_sum / weight_sum,
                    direction: sin_sum.atan2(cos_sum).to_degrees().rem_euclid(360.0),
                }
            }
        }
    }

    pub fn set_sample(&mut self, index: usize, speed: f32, direction: f32) -> Option<()> {
        let row = self.rows.get_mut(index)?;
        row.sample.speed = speed;
        row.sample.direction = direction;
        Some(())
    }

    pub fn query_circle(&self, x: f32, y: f32, radius: f32) -> Vec<&WeatherRow> {
        self.query_circle_indices(x, y, radius)
            .into_iter()
            .map(|i| &self.rows[i])
            .collect()
    }

    pub fn rows(&self) -> &[WeatherRow] {
        &self.rows
    }

    /// Regular-grid metadata when this map's spatial index is grid-backed,
    /// `None` for kd-tree-backed maps. Callers can use the layout to walk
    /// only the cells in a sub-rectangle via `rows()[i * ny + j]` indexing
    /// instead of scanning every row.
    pub fn grid_layout(&self) -> Option<GridLayout> {
        match self.index {
            SpatialIndex::Grid {
                origin_x,
                origin_y,
                step_x,
                step_y,
                nx,
                ny,
            } => Some(GridLayout {
                origin_x,
                origin_y,
                step_x,
                step_y,
                nx,
                ny,
            }),
            SpatialIndex::Tree(_) => None,
        }
    }

    /// The smaller of the two grid step sizes when this map is a regular
    /// uniform grid; `None` for kd-tree-backed maps. Renderers can use
    /// this to size per-cell decorations (e.g. wind barbs) so visual
    /// density stays sensible across zoom levels.
    pub fn grid_step(&self) -> Option<f32> {
        match self.index {
            SpatialIndex::Grid { step_x, step_y, .. } => Some(step_x.min(step_y)),
            SpatialIndex::Tree(_) => None,
        }
    }

    pub fn query_circle_indices(&self, x: f32, y: f32, radius: f32) -> Vec<usize> {
        let x = wrap_lon_query(x);
        match &self.index {
            SpatialIndex::Grid {
                origin_x,
                origin_y,
                step_x,
                step_y,
                nx,
                ny,
            } => {
                let r = radius.abs();
                let r2 = r * r;
                let nx = *nx;
                let ny = *ny;
                let i_min = ((x - r - origin_x) / step_x)
                    .floor()
                    .clamp(0.0, (nx - 1) as f32) as usize;
                let i_max = ((x + r - origin_x) / step_x)
                    .ceil()
                    .clamp(0.0, (nx - 1) as f32) as usize;
                let j_min = ((y - r - origin_y) / step_y)
                    .floor()
                    .clamp(0.0, (ny - 1) as f32) as usize;
                let j_max = ((y + r - origin_y) / step_y)
                    .ceil()
                    .clamp(0.0, (ny - 1) as f32) as usize;
                let mut out = Vec::new();
                for i in i_min..=i_max {
                    for j in j_min..=j_max {
                        let idx = i * ny + j;
                        let row = &self.rows[idx];
                        let dx = x - row.lon;
                        let dy = y - row.lat;
                        if dx * dx + dy * dy <= r2 {
                            out.push(idx);
                        }
                    }
                }
                out
            }
            SpatialIndex::Tree(tree) => tree
                .within::<SquaredEuclidean>(&[x, y], radius * radius)
                .iter()
                .map(|n| n.item as usize)
                .collect(),
        }
    }
}

/// Inverse-distance-weighted blend of the wind samples at the given row indices,
/// evaluated at `(x, y)`. Direction is blended via sin/cos to handle the 0°/360°
/// wraparound. An exact coordinate hit short-circuits to that row's sample.
fn idw_blend(rows: &[WeatherRow], indices: &[usize], x: f32, y: f32) -> WindSample {
    let mut weight_sum = 0.0f32;
    let mut speed_sum = 0.0f32;
    let mut sin_sum = 0.0f32;
    let mut cos_sum = 0.0f32;
    for &idx in indices {
        let r = &rows[idx];
        let dx = x - r.lon;
        let dy = y - r.lat;
        let d2 = dx * dx + dy * dy;
        if d2 == 0.0 {
            return WindSample {
                speed: r.sample.speed,
                direction: r.sample.direction,
            };
        }
        let w = 1.0 / d2;
        weight_sum += w;
        speed_sum += w * r.sample.speed;
        let dir_rad = r.sample.direction.to_radians();
        sin_sum += w * dir_rad.sin();
        cos_sum += w * dir_rad.cos();
    }
    WindSample {
        speed: speed_sum / weight_sum,
        direction: sin_sum.atan2(cos_sum).to_degrees().rem_euclid(360.0),
    }
}

/// Detect whether `rows` form a complete, uniformly spaced 2D grid (every
/// `(xi, yj)` cell present exactly once with a constant step on each axis).
/// Returns `None` if not, in which case the caller should fall back to a
/// kd-tree.
fn detect_grid_layout(rows: &[WeatherRow]) -> Option<GridLayout> {
    if rows.len() < 4 {
        return None;
    }

    let cmp = |a: &f32, b: &f32| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal);
    let mut xs: Vec<f32> = rows.iter().map(|r| r.lon).collect();
    let mut ys: Vec<f32> = rows.iter().map(|r| r.lat).collect();
    xs.sort_by(cmp);
    ys.sort_by(cmp);
    xs.dedup();
    ys.dedup();

    let nx = xs.len();
    let ny = ys.len();
    if nx < 2 || ny < 2 {
        return None;
    }
    if nx.checked_mul(ny)? != rows.len() {
        return None;
    }

    let origin_x = xs[0];
    let origin_y = ys[0];
    let step_x = xs[1] - xs[0];
    let step_y = ys[1] - ys[0];
    if step_x <= 0.0 || step_y <= 0.0 || !step_x.is_finite() || !step_y.is_finite() {
        return None;
    }

    // Verify uniform spacing per consecutive pair. Tolerance is relative to
    // the step so it scales with maps from tiny (density 0.1) to huge
    // (density 100000) without tuning. We deliberately compare each pair's
    // local step against `step_x` rather than predicting `origin + i*step_x`
    // and comparing to `xs[i]` — at global geographic scale the row positions
    // sit at ±2e7 m where f32 precision is ~2 m, and a 1-2 m round-off in
    // `step_x` accumulates to kilometres of drift over thousands of columns
    // even when the data is genuinely uniform.
    let eps_x = step_x * 1e-3;
    let eps_y = step_y * 1e-3;
    for w in xs.windows(2) {
        if (w[1] - w[0] - step_x).abs() > eps_x {
            return None;
        }
    }
    for w in ys.windows(2) {
        if (w[1] - w[0] - step_y).abs() > eps_y {
            return None;
        }
    }

    // Verify each (i, j) cell is present exactly once. Combined with
    // `nx*ny == rows.len()`, this proves complete coverage (no duplicates,
    // no holes).
    let mut seen = vec![false; nx * ny];
    for row in rows {
        // f32 → i64 saturates (NaN → 0, ±inf → ±i64::MAX), so the
        // `try_from` correctly rejects negative / NaN coordinates by
        // refusing to narrow them into usize.
        let Ok(i) = usize::try_from(((row.lon - origin_x) / step_x).round() as i64) else {
            return None;
        };
        let Ok(j) = usize::try_from(((row.lat - origin_y) / step_y).round() as i64) else {
            return None;
        };
        if i >= nx || j >= ny {
            return None;
        }
        let slot = i * ny + j;
        if seen[slot] {
            return None;
        }
        seen[slot] = true;
    }

    Some(GridLayout {
        origin_x,
        origin_y,
        step_x,
        step_y,
        nx,
        ny,
    })
}

/// Permute `rows` into canonical column-major grid order. Pre-condition:
/// `detect_grid_layout` returned `Some(layout)` for these rows, which proves
/// every cell is filled exactly once.
fn reorder_to_grid(rows: Vec<WeatherRow>, layout: &GridLayout) -> Vec<WeatherRow> {
    let mut slots: Vec<Option<WeatherRow>> = (0..layout.nx * layout.ny).map(|_| None).collect();
    for row in rows {
        let i = ((row.lon - layout.origin_x) / layout.step_x).round() as usize;
        let j = ((row.lat - layout.origin_y) / layout.step_y).round() as usize;
        slots[i * layout.ny + j] = Some(row);
    }
    slots
        .into_iter()
        .map(|o| o.expect("detect_grid_layout proved complete coverage"))
        .collect()
}

/// Default crossfade window for time-axis wrap, expressed in source
/// frame counts. With the 1 h GFS sample we ship that's 5 hours of
/// smooth transition between frame N-1 and the looped frame 0; with
/// any cadence the rule yields a 5-frame blend, scaling sensibly.
const DEFAULT_CROSSFADE_FRAMES: f32 = 5.0;

/// Linear blend of two [`WindSample`]s. Speed is interpolated
/// directly; direction is interpolated via sin/cos so the 0°/360°
/// boundary doesn't cause a discontinuity (the canonical trick for
/// blending circular quantities).
fn blend_samples(lo: &WindSample, hi: &WindSample, alpha: f32) -> WindSample {
    let inv = 1.0 - alpha;
    let speed = lo.speed * inv + hi.speed * alpha;
    let lo_rad = lo.direction.to_radians();
    let hi_rad = hi.direction.to_radians();
    let sin_blend = lo_rad.sin() * inv + hi_rad.sin() * alpha;
    let cos_blend = lo_rad.cos() * inv + hi_rad.cos() * alpha;
    let direction = sin_blend.atan2(cos_blend).to_degrees().rem_euclid(360.0);
    WindSample { speed, direction }
}

/// A stack of [`WindMap`] frames separated by a fixed time step (in seconds).
///
/// Frame `k` represents the wind field at time `k * step_seconds`. Spatial
/// queries between frames are linearly interpolated; queries past the last
/// frame loop with a smooth crossfade (see [`Self::crossfade_seconds`])
/// rather than snapping back to frame 0.
#[derive(Clone)]
pub struct TimedWindMap {
    step_seconds: f32,
    /// Length of the synthesised blend window between the last frame and
    /// the looped first frame. See `Self::query` for the math. Defaults
    /// to `5 × step_seconds` in `Self::new`; downstream callers can
    /// override via [`Self::with_crossfade_seconds`].
    crossfade_seconds: f32,
    /// Optional real-world UTC range the dataset covers. `None` means
    /// "we don't know" — synthetic generators leave it unset, and v1
    /// `wind_av1` files (predating the v2 schema bump) also decode
    /// without it. When `Some`, `.0 ≤ .1` and consumers can format
    /// them for display.
    time_range: Option<(chrono::DateTime<chrono::Utc>, chrono::DateTime<chrono::Utc>)>,
    frames: Vec<WindMap>,
}

impl TimedWindMap {
    pub fn new(frames: Vec<WindMap>, step_seconds: f32) -> Self {
        assert!(
            !frames.is_empty(),
            "TimedWindMap must have at least one frame"
        );
        assert!(
            step_seconds > 0.0,
            "TimedWindMap step_seconds must be > 0, got {step_seconds}"
        );
        Self {
            step_seconds,
            crossfade_seconds: DEFAULT_CROSSFADE_FRAMES * step_seconds,
            time_range: None,
            frames,
        }
    }

    /// Override the wrap-crossfade window (see [`Self::query`]). `secs`
    /// is clamped to ≥ 0; passing 0 reverts to a hard wrap (frame N-1
    /// → frame 0 in one step). Larger values give a smoother blend at
    /// the cost of more synthesised end-of-data.
    pub fn with_crossfade_seconds(mut self, secs: f32) -> Self {
        self.crossfade_seconds = secs.max(0.0);
        self
    }

    /// Attach the dataset's UTC `(start, end)` time bounds. Consumers
    /// surface them in `bywind-cli info` and the GUI's Time section.
    /// The two timestamps are kept in order — if `start > end` the
    /// pair is swapped to keep the invariant.
    pub fn with_time_range(
        mut self,
        start: chrono::DateTime<chrono::Utc>,
        end: chrono::DateTime<chrono::Utc>,
    ) -> Self {
        let (a, b) = if start <= end {
            (start, end)
        } else {
            (end, start)
        };
        self.time_range = Some((a, b));
        self
    }

    /// UTC `(start, end)` bounds of the dataset's coverage, or `None`
    /// when the source (synthetic generator, v1 codec file) didn't
    /// carry them. The pair is always non-decreasing.
    pub fn time_range(
        &self,
    ) -> Option<(chrono::DateTime<chrono::Utc>, chrono::DateTime<chrono::Utc>)> {
        self.time_range
    }

    /// Length of the wrap-crossfade window in seconds (see [`Self::query`]).
    pub fn crossfade_seconds(&self) -> f32 {
        self.crossfade_seconds
    }

    /// Length of one looped cycle in seconds: data extent
    /// `(N-1) · step_seconds` plus the crossfade window. Past this
    /// the time axis wraps via `rem_euclid`.
    pub fn cycle_seconds(&self) -> f32 {
        self.duration_seconds() + self.crossfade_seconds
    }

    pub fn generate(
        size_x: f32,
        size_y: f32,
        density: f32,
        frame_count: usize,
        step_seconds: f32,
    ) -> Self {
        let frames = (0..frame_count)
            .map(|_| WindMap::generate(size_x, size_y, density))
            .collect();
        Self::new(frames, step_seconds)
    }

    pub fn generate_random(
        size_x: f32,
        size_y: f32,
        density: f32,
        frame_count: usize,
        step_seconds: f32,
        speed_range: std::ops::Range<f32>,
    ) -> Self {
        Self::generate_random_with_rng(
            size_x,
            size_y,
            density,
            frame_count,
            step_seconds,
            speed_range,
            &mut rand::rng(),
        )
    }

    /// Same as [`Self::generate_random`] but takes a caller-supplied RNG so
    /// every frame's wind field can be deterministic for tests.
    pub fn generate_random_with_rng<R: rand::Rng + rand::RngExt>(
        size_x: f32,
        size_y: f32,
        density: f32,
        frame_count: usize,
        step_seconds: f32,
        speed_range: std::ops::Range<f32>,
        rng: &mut R,
    ) -> Self {
        let frames = (0..frame_count)
            .map(|_| {
                WindMap::generate_random_with_rng(size_x, size_y, density, speed_range.clone(), rng)
            })
            .collect();
        Self::new(frames, step_seconds)
    }

    /// Group `rows` by their `t_seconds` into one [`WindMap`] per unique time
    /// value, sorted ascending. The time step is inferred from the gap between
    /// the first two unique times; with only one unique time, `step_seconds`
    /// defaults to `1.0` (irrelevant since no interpolation is possible).
    /// Returns `None` if `rows` is empty.
    #[expect(
        clippy::float_cmp,
        reason = "timestamps are copied byte-for-byte from input rows; \
                  exact equality determines whether two rows belong to the \
                  same frame."
    )]
    pub fn from_timed_rows(mut rows: Vec<TimedWeatherRow>) -> Option<Self> {
        if rows.is_empty() {
            return None;
        }

        rows.sort_by(|a, b| {
            a.t_seconds
                .partial_cmp(&b.t_seconds)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut frame_rows: Vec<Vec<WeatherRow>> = vec![Vec::new()];
        let mut frame_times: Vec<f32> = vec![rows[0].t_seconds];
        let mut current_t = rows[0].t_seconds;
        // Track the current frame's index explicitly so we can use
        // direct `frame_rows[idx]` indexing instead of
        // `.last_mut().unwrap()`. Invariant: `frame_rows.len() == idx + 1`
        // — every `push(Vec::new())` is paired with `idx += 1`.
        let mut idx = 0usize;

        for trow in rows {
            if trow.t_seconds != current_t {
                frame_rows.push(Vec::new());
                frame_times.push(trow.t_seconds);
                current_t = trow.t_seconds;
                idx += 1;
            }
            frame_rows[idx].push(WeatherRow {
                lon: trow.lon,
                lat: trow.lat,
                sample: trow.sample,
            });
        }

        let step_seconds = if frame_times.len() < 2 {
            1.0
        } else {
            frame_times[1] - frame_times[0]
        };

        let frames = frame_rows.into_iter().map(WindMap::new).collect();
        Some(Self::new(frames, step_seconds))
    }

    pub fn frames(&self) -> &[WindMap] {
        &self.frames
    }
    pub fn frame(&self, idx: usize) -> Option<&WindMap> {
        self.frames.get(idx)
    }
    pub fn frame_mut(&mut self, idx: usize) -> Option<&mut WindMap> {
        self.frames.get_mut(idx)
    }
    pub fn frame_count(&self) -> usize {
        self.frames.len()
    }
    pub fn step_seconds(&self) -> f32 {
        self.step_seconds
    }

    pub fn duration_seconds(&self) -> f32 {
        self.frames.len().saturating_sub(1) as f32 * self.step_seconds
    }

    /// Flatten to a list of `(x, y, t_seconds, sample)` rows, ordered frame by frame.
    pub fn to_timed_rows(&self) -> Vec<TimedWeatherRow> {
        let mut out = Vec::with_capacity(self.frames.iter().map(|f| f.rows().len()).sum());
        for (k, frame) in self.frames.iter().enumerate() {
            let t = k as f32 * self.step_seconds;
            for row in frame.rows() {
                out.push(TimedWeatherRow {
                    lon: row.lon,
                    lat: row.lat,
                    t_seconds: t,
                    sample: row.sample.clone(),
                });
            }
        }
        out
    }

    /// Spatially queries the bracketing frames for time `t_seconds` and
    /// linearly interpolates the result. Direction is interpolated via
    /// sin/cos blend so the 0/360° wraparound doesn't cause a discontinuity.
    ///
    /// Time-axis behaviour: `t_seconds` is taken modulo
    /// [`Self::cycle_seconds`], so queries past the data end loop. The
    /// cycle is split into two regions:
    ///
    /// * `t_mod ∈ [0, (N-1)·step]` — normal interpolation between
    ///   adjacent frames.
    /// * `t_mod ∈ ((N-1)·step, cycle]` — crossfade region. Linearly
    ///   blends frame N-1 (at `t_mod = (N-1)·step`) toward frame 0 (at
    ///   `t_mod = cycle`). The width is [`Self::crossfade_seconds`].
    ///
    /// With the default 5-frame crossfade, hitting the data end is a
    /// smooth multi-frame transition into the loop instead of a hard
    /// snap.
    pub fn query(&self, x: f32, y: f32, t_seconds: f32) -> WindSample {
        let n = self.frames.len();
        if n == 1 {
            return self.frames[0].query(x, y);
        }
        let data_end = self.duration_seconds();
        let cycle = data_end + self.crossfade_seconds;
        // `rem_euclid` wraps both directions and never returns negatives,
        // so callers can pass any finite `t_seconds`.
        let t_mod = if cycle > 0.0 {
            t_seconds.rem_euclid(cycle)
        } else {
            0.0
        };

        let (lo_idx, hi_idx, alpha) = if t_mod <= data_end {
            // Inside the data: bracketing frames are `floor(t/step)` and
            // its successor. Clamp the upper index so the data-end edge
            // (where `frame_idx_f` lands exactly on `n - 1`) still picks
            // a valid frame.
            let frame_idx_f = t_mod / self.step_seconds;
            let lo = (frame_idx_f.floor() as usize).min(n - 1);
            let hi = (lo + 1).min(n - 1);
            (lo, hi, frame_idx_f - lo as f32)
        } else {
            // In the crossfade tail: linearly blend frame N-1 → frame 0
            // across `crossfade_seconds`. `alpha = 0` at `t_mod =
            // data_end`, `alpha = 1` at `t_mod = cycle`.
            let alpha = (t_mod - data_end) / self.crossfade_seconds;
            (n - 1, 0, alpha)
        };

        let s_lo = self.frames[lo_idx].query(x, y);
        if lo_idx == hi_idx || alpha == 0.0 {
            return s_lo;
        }
        let s_hi = self.frames[hi_idx].query(x, y);
        blend_samples(&s_lo, &s_hi, alpha)
    }

    /// Build a synthetic `WindMap` representing the wind field at
    /// `t_seconds`. Used by the GUI to render time points past the
    /// data end (in the crossfade / wrap region) where no underlying
    /// frame exists. Each cell's value comes from
    /// [`Self::query`] at that cell's `(lon, lat)`, so the same
    /// crossfade rule the search uses also drives what's drawn.
    ///
    /// Reuses the frame-0 grid layout and row positions; just
    /// resamples the per-cell wind. Cost is one `query` per cell
    /// — for a 1440×721 global grid that's ~1 M queries, ~100 ms on
    /// a modern CPU. Callers should cache the result when scrubbing.
    pub fn synthesize_frame_at(&self, t_seconds: f32) -> WindMap {
        let template = &self.frames[0];
        let rows: Vec<WeatherRow> = template
            .rows()
            .iter()
            .map(|row| WeatherRow {
                lon: row.lon,
                lat: row.lat,
                sample: self.query(row.lon, row.lat, t_seconds),
            })
            .collect();
        match template.grid_layout() {
            Some(layout) => WindMap::from_grid(rows, layout),
            None => WindMap::new(rows),
        }
    }

    /// Precompute a regular spatial grid of wind vectors over `bounds`, one slice
    /// per source frame, returning a [`BakedWindMap`] suitable for fast,
    /// `Sync`-safe sampling (e.g. parallel PSO).
    pub fn bake(&self, bounds: BakeBounds) -> BakedWindMap {
        BakedWindMap::from_timed_map(self, bounds)
    }
}

/// Specification for baking a [`TimedWindMap`] onto a regular spatial grid.
///
/// `bbox` carries the wrap encoding (`lon_min > lon_max` ⇒ crosses the
/// antimeridian). The bake-time `+360` extension that keeps the lon
/// axis monotonic is applied at use time inside [`TimedWindMap::bake`]
/// via [`LonLatBbox::lon_max_unwrapped`], so the stored bbox stays
/// canonical and round-trips through [`crate::MapBounds::to_bake_bounds`]
/// without losing the wrap information.
///
/// The time grid is inherited from the source map (one bake slice per
/// source frame).
#[derive(Copy, Clone, Debug)]
pub struct BakeBounds {
    pub bbox: LonLatBbox,
    /// Spatial grid spacing. Smaller means higher-resolution at quadratic memory cost.
    pub step: f64,
    /// Divisor applied to incoming `sample_wind` coordinates before grid lookup.
    /// Match this to whatever factor the search uses to scale its coordinates.
    pub coord_scale: f64,
}

/// Precomputed time-varying wind field on a regular spatial grid. `Sync`,
/// lock-free, O(1) lookup with linear time interpolation.
///
/// Layout: `grid[(iy * nx + ix) * nt + it]`. Time is the innermost axis so the
/// two values needed for time interpolation share a cacheline (the per-`(x,y)`
/// time series is contiguous).
#[derive(Clone, Debug)]
pub struct BakedWindMap {
    pub(crate) grid: Vec<Wind>,
    pub(crate) nx: usize,
    pub(crate) ny: usize,
    pub(crate) nt: usize,
    pub(crate) x_min: f64,
    pub(crate) y_min: f64,
    pub(crate) step: f64,
    pub(crate) t_step_seconds: f64,
    /// Length of the wrap-crossfade window. Routes past `(nt-1) ·
    /// t_step_seconds` blend frame N-1 → frame 0 over this many
    /// seconds instead of snapping in one step. Defaults to
    /// `DEFAULT_CROSSFADE_FRAMES × t_step_seconds`; populated from the
    /// source [`TimedWindMap`] when baking and from `t_step_seconds`
    /// alone when decoding (`baked_codec` doesn't carry the value on
    /// disk yet).
    pub(crate) crossfade_seconds: f64,
    pub(crate) coord_scale: f64,
}

impl BakedWindMap {
    /// Number of cells along the longitude axis.
    pub fn nx(&self) -> usize {
        self.nx
    }
    /// Number of cells along the latitude axis.
    pub fn ny(&self) -> usize {
        self.ny
    }
    /// Number of time slices (frames).
    pub fn nt(&self) -> usize {
        self.nt
    }
    /// Longitude of cell `(0, *)` in degrees.
    pub fn x_min(&self) -> f64 {
        self.x_min
    }
    /// Latitude of cell `(*, 0)` in degrees.
    pub fn y_min(&self) -> f64 {
        self.y_min
    }
    /// Spatial grid spacing in degrees (same on both axes — the bake grid
    /// is square in lon/lat, even though the underlying ground distance
    /// varies with latitude).
    pub fn step(&self) -> f64 {
        self.step
    }
    /// Time step between frames in seconds.
    pub fn t_step_seconds(&self) -> f64 {
        self.t_step_seconds
    }

    fn from_timed_map(map: &TimedWindMap, bounds: BakeBounds) -> Self {
        assert!(bounds.step > 0.0, "BakeBounds::step must be > 0");
        assert!(
            bounds.coord_scale > 0.0,
            "BakeBounds::coord_scale must be > 0"
        );
        assert!(
            bounds.bbox.lat_max >= bounds.bbox.lat_min,
            "BakeBounds: lat_max < lat_min",
        );

        // Lon axis runs monotonically from `lon_min` to `lon_max_unwrapped()`
        // — for non-wrapping bboxes that's just `lon_max`; for wrapping
        // bboxes (lon_min > lon_max) it's `lon_max + 360` so the axis
        // doesn't fold back on itself. The sample sites below are
        // pre-canonicalised via `WindMap::query`, so storage and lookup
        // agree on the convention.
        let lon_min = bounds.bbox.lon_min;
        let lon_max_unwrapped = bounds.bbox.lon_max_unwrapped();
        let lat_min = bounds.bbox.lat_min;
        let lat_max = bounds.bbox.lat_max;
        let nx = ((lon_max_unwrapped - lon_min) / bounds.step).ceil() as usize + 1;
        let ny = ((lat_max - lat_min) / bounds.step).ceil() as usize + 1;
        let nt = map.frame_count();
        let t_step_seconds = f64::from(map.step_seconds());

        // Wind map speeds are in knots (meteorological convention used by the
        // barb renderer in bywind-viz); the sailing physics in swarmkit operates
        // in SI units (m/s). Convert once here at the bake boundary so all
        // downstream physics sees consistent units. 1 knot = 1852/3600 m/s.
        const KNOTS_TO_MS: f64 = 1852.0 / 3600.0;

        let frames = map.frames();
        let mut grid = vec![Wind::zero(); nx * ny * nt];
        // Each (j, i) cell owns a contiguous span of `nt` entries at offset
        // `(j * nx + i) * nt`, so chunks are non-overlapping and can be filled
        // in parallel. Frames are read immutably; WindMap::query takes &self.
        grid.par_chunks_mut(nt)
            .enumerate()
            .for_each(|(cell_idx, chunk)| {
                let j = cell_idx / nx;
                let i = cell_idx % nx;
                let x = lon_min + i as f64 * bounds.step;
                let y = lat_min + j as f64 * bounds.step;
                for (k, frame) in frames.iter().enumerate() {
                    let sample = frame.query(x as f32, y as f32);
                    // `direction` is the meteorological from-bearing (compass:
                    // 0 = from-N, π/2 = from-E). The wind velocity vector
                    // points the *opposite* way (toward bearing β + π). In the
                    // local east-north tangent frame:
                    //   u_east  = speed · sin(β + π) = −speed · sin(β)
                    //   v_north = speed · cos(β + π) = −speed · cos(β)
                    let dir_rad = f64::from(sample.direction) * PI / 180.0;
                    let speed_ms = f64::from(sample.speed) * KNOTS_TO_MS;
                    chunk[k] = Wind::new(-speed_ms * dir_rad.sin(), -speed_ms * dir_rad.cos());
                }
            });

        Self {
            grid,
            nx,
            ny,
            nt,
            x_min: lon_min,
            y_min: lat_min,
            step: bounds.step,
            t_step_seconds,
            // Carry the source map's crossfade through to the baked
            // grid so viz queries (TimedWindMap) and search queries
            // (BakedWindMap) agree on what happens at the time-axis
            // wrap.
            crossfade_seconds: f64::from(map.crossfade_seconds()),
            coord_scale: bounds.coord_scale,
        }
    }
}

impl WindSource for BakedWindMap {
    /// Returns the wind velocity vector at `location` and time `t` (seconds).
    ///
    /// Spatial lookup snaps to the nearest grid cell; time lookup linearly
    /// interpolates between the two bracketing baked frames. `t` wraps modularly
    /// across the loaded period `nt * t_step_seconds`, so a route that runs
    /// longer than the loaded data sees the field repeat (frame `nt` ≡ frame 0).
    /// Negative `t` wraps the same way.
    ///
    /// `WindSample::direction` is meteorological: degrees the wind blows FROM,
    /// 0° = north, clockwise. The returned vector points in the direction the
    /// wind is blowing TO, in screen coordinates (y-down):
    ///   `vx = -speed × sin(dir_rad)`,  `vy = +speed × cos(dir_rad)`.
    fn sample_wind(&self, location: LatLon, t: f64) -> Wind {
        // The pre-LatLon API took a `Vector2<f64>` of `(lon, lat)` in the
        // bake grid's coordinate space — i.e. *post-`coord_scale` divide*.
        // `coord_scale` is currently always `1.0` for native-(lon, lat)
        // wind maps; the divide was a leftover from the pre-projection
        // era and is preserved here only so any future scaled bake stays
        // consistent.
        let scaled_lon = location.lon / self.coord_scale;
        let scaled_lat = location.lat / self.coord_scale;
        // Wrap-aware lon: if the bake bounds spanned the antimeridian
        // (`x_max > 180°`), an incoming canonical-lon query (e.g. PSO
        // particle at lon = −175°) won't be inside `[x_min, x_max]`
        // even though the same physical point IS in the bake grid.
        // Shift by ±360° to bring it into range. The bake step
        // pre-canonicalises samples via `WindMap::query`, so storage
        // and lookup agree on this convention.
        let mut x = scaled_lon;
        let x_max = self.x_min + (self.nx as f64 - 1.0) * self.step;
        if x < self.x_min {
            x += 360.0;
        } else if x > x_max {
            x -= 360.0;
        }
        let ix = ((x - self.x_min) / self.step)
            .round()
            .clamp(0.0, (self.nx - 1) as f64) as usize;
        let iy = ((scaled_lat - self.y_min) / self.step)
            .round()
            .clamp(0.0, (self.ny - 1) as f64) as usize;

        let nt = self.nt.max(1);
        let base = (iy * self.nx + ix) * self.nt;
        if nt == 1 || self.t_step_seconds <= 0.0 {
            return self.grid[base];
        }
        // Same crossfade rule TimedWindMap::query uses: `t_mod` lands
        // in either the data span `[0, (nt-1)·step]` or the crossfade
        // tail `((nt-1)·step, cycle]`. The cycle accounts for the
        // crossfade window so consecutive loops don't snap.
        let data_end = (nt as f64 - 1.0) * self.t_step_seconds;
        let cycle = data_end + self.crossfade_seconds;
        let t_mod = if cycle > 0.0 {
            t.rem_euclid(cycle)
        } else {
            0.0
        };

        let (it, it_hi, alpha) = if t_mod <= data_end {
            let t_idx_f = t_mod / self.t_step_seconds;
            let it_floor = t_idx_f.floor();
            let it = (it_floor as i64).clamp(0, (nt - 1) as i64) as usize;
            let it_hi = (it + 1).min(nt - 1);
            (it, it_hi, t_idx_f - it_floor)
        } else {
            // Crossfade tail: blend frame nt-1 → frame 0.
            let alpha = (t_mod - data_end) / self.crossfade_seconds;
            (nt - 1, 0, alpha)
        };

        let v_lo = self.grid[base + it];
        if it == it_hi || alpha == 0.0 {
            return v_lo;
        }
        let v_hi = self.grid[base + it_hi];
        let inv = 1.0 - alpha;
        Wind::new(
            v_lo.east_mps * inv + v_hi.east_mps * alpha,
            v_lo.north_mps * inv + v_hi.north_mps * alpha,
        )
    }
}

#[cfg(test)]
mod crossfade_tests {
    //! Endpoint and continuity coverage for the time-axis wrap
    //! crossfade. Both `TimedWindMap::query` and
    //! `BakedWindMap::sample_wind` adopt the same rule (`t mod cycle`
    //! splits the cycle into a data span and a crossfade tail), and
    //! these tests pin both sides so they can't drift apart.
    use super::*;
    use crate::WindSample;

    /// Two-frame `TimedWindMap` on a 1×1 grid: frame 0 = wind from
    /// north at 10 kt, frame 1 = wind from east at 20 kt. Step =
    /// 3600 s, so a 5-frame crossfade default puts the cycle at
    /// `data_end + 5·step = 1·3600 + 5·3600 = 21600 s`.
    fn fixture() -> TimedWindMap {
        let row = |speed, direction| {
            vec![WeatherRow {
                lon: 0.0,
                lat: 0.0,
                sample: WindSample { speed, direction },
            }]
        };
        let frames = vec![WindMap::new(row(10.0, 0.0)), WindMap::new(row(20.0, 90.0))];
        TimedWindMap::new(frames, 3600.0)
    }

    #[test]
    #[expect(
        clippy::float_cmp,
        reason = "exact equality is the contract being tested"
    )]
    fn cycle_and_crossfade_defaults() {
        let m = fixture();
        assert_eq!(m.step_seconds(), 3600.0);
        assert_eq!(m.duration_seconds(), 3600.0);
        // 5-frame crossfade default.
        assert_eq!(m.crossfade_seconds(), 5.0 * 3600.0);
        assert_eq!(m.cycle_seconds(), 3600.0 + 5.0 * 3600.0);
    }

    #[test]
    fn at_data_end_returns_last_frame_exactly() {
        let m = fixture();
        let s = m.query(0.0, 0.0, m.duration_seconds());
        // At t = (N-1)·step we're exactly on frame N-1 (the
        // crossfade hasn't started).
        assert!((s.speed - 20.0).abs() < 1e-4);
        assert!((s.direction - 90.0).abs() < 1e-4);
    }

    #[test]
    fn at_cycle_returns_first_frame_exactly() {
        let m = fixture();
        // t = cycle wraps back to t = 0 via rem_euclid, which is
        // exactly frame 0 (the wrap completes here).
        let s = m.query(0.0, 0.0, m.cycle_seconds());
        assert!((s.speed - 10.0).abs() < 1e-4);
        assert!((s.direction - 0.0).abs() < 1e-4);
    }

    #[test]
    fn midpoint_of_crossfade_blends_50_50() {
        let m = fixture();
        // Halfway through the crossfade tail: 50/50 blend of frame
        // N-1 (20 kt from E) and frame 0 (10 kt from N).
        let t = m.duration_seconds() + m.crossfade_seconds() / 2.0;
        let s = m.query(0.0, 0.0, t);
        assert!((s.speed - 15.0).abs() < 1e-3, "speed = {}", s.speed);
        // Direction blends sin/cos: 0° (N) and 90° (E) → 45° (NE).
        let wrapped = ((s.direction - 45.0 + 540.0) % 360.0) - 180.0;
        assert!(wrapped.abs() < 1e-3, "direction = {}", s.direction);
    }

    #[test]
    fn continuity_across_cycle_boundary() {
        // Querying t = cycle + ε should match querying t = ε
        // because rem_euclid wraps both back to the same point. This
        // pins that the cycle is the right length (no off-by-one).
        let m = fixture();
        let eps = 12.0_f32;
        let s_near_start = m.query(0.0, 0.0, eps);
        let s_after_cycle = m.query(0.0, 0.0, m.cycle_seconds() + eps);
        assert!((s_near_start.speed - s_after_cycle.speed).abs() < 1e-3);
        let dir_wrapped =
            ((s_near_start.direction - s_after_cycle.direction + 540.0) % 360.0) - 180.0;
        assert!(dir_wrapped.abs() < 1e-3);
    }

    #[test]
    fn baked_crossfade_matches_timed_at_endpoints() {
        // Bake a one-cell grid from the same fixture and verify the
        // search-side and viz-side queries agree at the seam. Both
        // paths drive the same crossfade math; this guards against
        // future drift between the two implementations.
        let m = fixture();
        let bbox = swarmkit_sailing::spherical::LonLatBbox::new(0.0, 0.0, 0.0, 0.0);
        let baked = m.bake(BakeBounds {
            bbox,
            step: 0.25,
            coord_scale: 1.0,
        });
        let loc = swarmkit_sailing::spherical::LatLon::new(0.0, 0.0);
        let knots_to_ms = 1852.0 / 3600.0;

        // At t = 0: frame 0 is 10 kt from north — wind blows TOWARD
        // south so v_north < 0, u_east ≈ 0.
        let w0 = baked.sample_wind(loc, 0.0);
        assert!(w0.east_mps.abs() < 1e-3);
        assert!((w0.north_mps + 10.0 * knots_to_ms).abs() < 1e-3);

        // At t = mid-crossfade: 50/50 blend of frame N-1 and frame 0.
        let t_mid = f64::from(m.duration_seconds()) + f64::from(m.crossfade_seconds()) / 2.0;
        let w_mid = baked.sample_wind(loc, t_mid);
        // Frame N-1: 20 kt from east → u_east = -20·KTS, v_north = 0.
        // Frame 0:   10 kt from north → u_east = 0,         v_north = -10·KTS.
        // 50/50 blend: u_east = -10·KTS, v_north = -5·KTS.
        assert!((w_mid.east_mps + 10.0 * knots_to_ms).abs() < 1e-2);
        assert!((w_mid.north_mps + 5.0 * knots_to_ms).abs() < 1e-2);
    }
}
