use crate::spherical::Segment;
use crate::units::PathXY;
use crate::{Sailboat, WindSource};

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

#[cfg(feature = "probe-stats")]
use std::sync::atomic::{AtomicU64, Ordering};

/// `locate`-call counters plus out-of-range probe count (when the
/// queried `dep` falls outside the tabulated range, `locate` clamps
/// silently — see item D in the accuracy writeup). Feature-gated to
/// avoid atomic traffic in normal builds; per-instance so concurrent
/// searches don't share counters.
#[cfg(feature = "probe-stats")]
#[derive(Default, Debug)]
pub struct ProbeCounters {
    total: AtomicU64,
    out_of_range: AtomicU64,
}

#[cfg(feature = "probe-stats")]
impl ProbeCounters {
    pub fn snapshot(&self) -> ProbeStats {
        ProbeStats {
            total: self.total.load(Ordering::Relaxed),
            out_of_range: self.out_of_range.load(Ordering::Relaxed),
        }
    }

    pub fn reset(&self) {
        self.total.store(0, Ordering::Relaxed);
        self.out_of_range.store(0, Ordering::Relaxed);
    }
}

#[cfg(feature = "probe-stats")]
#[derive(Clone, Copy, Debug, Default)]
pub struct ProbeStats {
    pub total: u64,
    pub out_of_range: u64,
}

/// Default departure-time samples per segment. 8 gives sub-1%
/// interpolation error on smooth weather fields. Overridden at call
/// site via `SegmentRangeTables::build`'s `range_k` arg, which
/// `SearchSettings::range_k` plumbs from `SearchConfig`.
pub const DEFAULT_RANGE_K: usize = 8;

/// Default `mcr_01` samples per departure-time bucket. 8 tracks the
/// smooth-monotone `travel_time(mcr)` curve within ~1%. Sensitivity
/// study (`bywind/profiling/stage2-counters.md`, iter 6) — worst-case
/// regression on the large scenario:
///
/// - `k_mcr=6` → ~6% wallclock saving, ≤1.3% fitness loss.
/// - `k_mcr=4` → ~11% wallclock saving, ≤2.1% fitness loss.
///
/// Geographic path is xy-determined, so `k_mcr` only moves the inner
/// PSO's `t` lookup-table resolution; land-crossing is unaffected.
/// Overridden at call site via `SegmentRangeTables::build`'s `k_mcr`
/// arg, which `SearchSettings::k_mcr` plumbs from `SearchConfig`.
pub const DEFAULT_K_MCR: usize = 8;

/// Tabulation-range padding. Inner-PSO particles can briefly probe past
/// the forward-propagated `best`/`worst` extrema before the boundary
/// clamp pulls them back; padding avoids out-of-range queries in the
/// common case (clamping catches the rest).
const RANGE_PADDING: f64 = 0.15;

#[derive(Clone, Debug)]
struct SegmentTable {
    dep_min: f64,
    dep_max: f64,
    /// Lower endpoint of the mcr sampling range. **Uniform across all
    /// buckets of this segment** so `times[..][k]` shares one absolute
    /// mcr value across `b` — lerping along the dep axis is then
    /// well-defined (see `query_mcr_for_delta_time`). 0.0 when every
    /// probed dep yields finite travel time at `mcr=0`, 0.1 otherwise.
    worst_mcr: f64,
    /// Row-major `times[b * k_mcr + k]` (length `range_k * k_mcr`),
    /// monotonically decreasing in `k` within each row. `Vec` rather
    /// than fixed arrays so resolution stays runtime-tunable.
    times: Vec<f64>,
}

/// Per-segment `travel_time(departure_time, mcr)` lookup tables.
/// Replaces full wind integrations with O(1) interpolated lookups in
/// the inner time-PSO hot path. Rebuilt once per outer `xy` change.
/// Segment `i` covers `xy[i] -> xy[i+1]` and arrives at `t[i+1]`.
#[derive(Debug)]
pub struct SegmentRangeTables<const N: usize> {
    tables: Vec<SegmentTable>,
    range_k: usize,
    k_mcr: usize,
    #[cfg(feature = "probe-stats")]
    probes: ProbeCounters,
}

impl<const N: usize> Clone for SegmentRangeTables<N> {
    fn clone(&self) -> Self {
        Self {
            tables: self.tables.clone(),
            range_k: self.range_k,
            k_mcr: self.k_mcr,
            // Snapshot atomic counts so each instance owns its own.
            #[cfg(feature = "probe-stats")]
            probes: ProbeCounters {
                total: AtomicU64::new(self.probes.total.load(Ordering::Relaxed)),
                out_of_range: AtomicU64::new(self.probes.out_of_range.load(Ordering::Relaxed)),
            },
        }
    }
}

impl<const N: usize> SegmentRangeTables<N> {
    /// Forward pass through `xy` from `departure_time`. Each segment's
    /// tabulation range comes from the previous segment's best/worst
    /// extrema, padded by `RANGE_PADDING`. Segment 0's range is
    /// degenerate (departure time is fixed). `range_k` and `k_mcr` must
    /// be ≥ 2; see [`DEFAULT_RANGE_K`] / [`DEFAULT_K_MCR`].
    pub fn build<SB: Sailboat, WS: WindSource>(
        xy: &PathXY<N>,
        boat: &SB,
        wind: &WS,
        step_distance_max: f64,
        departure_time: f64,
        range_k: usize,
        k_mcr: usize,
    ) -> Self {
        #[cfg(feature = "profile-timers")]
        let __profile_start = std::time::Instant::now();

        assert!(
            range_k >= 2,
            "SegmentRangeTables::build requires range_k >= 2, got {range_k}",
        );
        assert!(
            k_mcr >= 2,
            "SegmentRangeTables::build requires k_mcr >= 2, got {k_mcr}",
        );

        let seg_count = N.saturating_sub(1);
        let mut tables = Vec::with_capacity(seg_count);

        let mut dep_nominal_lo = departure_time;
        let mut dep_nominal_hi = departure_time;

        let mut per_bucket_wmcr = vec![0.0f64; range_k];
        let mut mcr_grid = vec![0.0f64; k_mcr];
        let mut mcr_grid_01 = vec![0.0f64; k_mcr];

        for seg_i in 0..seg_count {
            let origin = xy.lat_lon(seg_i);
            let destination = xy.lat_lon(seg_i + 1);

            let (dep_min, dep_max) = pad_range(dep_nominal_lo, dep_nominal_hi);

            // Reset per-bucket scratch state; reused across segments.
            per_bucket_wmcr.fill(0.0);
            let mut times = vec![0.0f64; range_k * k_mcr];

            // Pass 1: sample each bucket on its natural mcr grid. Probe
            // `mcr=0.0` first to detect a bucket that needs the 0.1
            // fallback (zero-wind / pole-locked / etc.), then call the
            // batched `get_travel_times_for_mcrs` once per bucket to fill
            // every `times[b * k_mcr + k]` from a single shared substep walk.
            for b in 0..range_k {
                let dep = dep_at(dep_min, dep_max, b, range_k);
                let segment = Segment {
                    origin,
                    destination,
                    origin_time: dep,
                    step_distance_max,
                };

                let t_at_zero = boat.get_travel_time(wind, segment, 0.0);
                let wm = if t_at_zero.is_infinite() { 0.1 } else { 0.0 };
                per_bucket_wmcr[b] = wm;

                for (k, slot) in mcr_grid.iter_mut().enumerate() {
                    *slot = mcr_at(wm, k, k_mcr);
                }
                let row = &mut times[b * k_mcr..(b + 1) * k_mcr];
                boat.get_travel_times_for_mcrs(wind, segment, &mcr_grid, row);
            }

            // Pass 2 (mixed only): re-sample the wmcr=0.0 buckets on
            // [0.1, 1.0] so every `times[..][k]` shares one mcr value
            // across `b`. Buckets already at 0.1 stay put.
            let worst_mcr = if per_bucket_wmcr.iter().any(|&w| w > 0.0) {
                for (k, slot) in mcr_grid_01.iter_mut().enumerate() {
                    *slot = mcr_at(0.1, k, k_mcr);
                }
                for b in 0..range_k {
                    if per_bucket_wmcr[b] < 0.1 {
                        let dep = dep_at(dep_min, dep_max, b, range_k);
                        let segment = Segment {
                            origin,
                            destination,
                            origin_time: dep,
                            step_distance_max,
                        };
                        let row = &mut times[b * k_mcr..(b + 1) * k_mcr];
                        boat.get_travel_times_for_mcrs(wind, segment, &mcr_grid_01, row);
                    }
                }
                0.1
            } else {
                0.0
            };

            let best_min = (0..range_k)
                .map(|b| times[b * k_mcr + (k_mcr - 1)])
                .fold(f64::INFINITY, f64::min);
            let worst_max = (0..range_k)
                .map(|b| times[b * k_mcr])
                .fold(f64::NEG_INFINITY, f64::max);
            dep_nominal_lo = dep_min + best_min;
            dep_nominal_hi = dep_max + worst_max;

            tables.push(SegmentTable {
                dep_min,
                dep_max,
                worst_mcr,
                times,
            });
        }

        let result = Self {
            tables,
            range_k,
            k_mcr,
            #[cfg(feature = "probe-stats")]
            probes: ProbeCounters::default(),
        };

        #[cfg(feature = "profile-timers")]
        crate::profile_timers::SEGMENT_CACHE_BUILD
            .record(__profile_start.elapsed().as_nanos() as u64);

        result
    }

    /// In-range vs. out-of-range probe counts.
    #[cfg(feature = "probe-stats")]
    pub fn probe_stats(&self) -> ProbeStats {
        self.probes.snapshot()
    }

    /// Reset counters; use between measurement windows on one cache.
    #[cfg(feature = "probe-stats")]
    pub fn reset_probe_stats(&self) {
        self.probes.reset();
    }

    /// `dep ≈ grid[idx] + frac * (grid[idx+1] - grid[idx])`. `idx`
    /// clamped to `[0, range_k - 2]`. Returns the unclamped `rel`
    /// alongside so `probe-stats` callers can tally out-of-range hits.
    fn locate(t: &SegmentTable, range_k: usize, dep: f64) -> (usize, f64, f64) {
        if t.dep_max <= t.dep_min {
            return (0, 0.0, 0.0);
        }
        let span = t.dep_max - t.dep_min;
        let step = span / (range_k - 1) as f64;
        let rel = (dep - t.dep_min) / step;
        let rel_clamped = rel.clamp(0.0, (range_k - 1) as f64);
        let idx = (rel_clamped.floor() as usize).min(range_k - 2);
        let frac = (rel_clamped - idx as f64).clamp(0.0, 1.0);
        (idx, frac, rel)
    }

    #[cfg(feature = "probe-stats")]
    fn record_probe(&self, rel: f64) {
        self.probes.total.fetch_add(1, Ordering::Relaxed);
        if rel < 0.0 || rel > (self.range_k - 1) as f64 {
            self.probes.out_of_range.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// `(t_min, t_max)` for segment `seg_i` at `dep`, ordered. Scans
    /// all `k_mcr` samples because in time-variant wind
    /// `travel_time(mcr)` isn't monotonic (each mcr samples wind at a
    /// different timestamp). On an all-INF row (e.g. pole-locked
    /// segment) the lerp produces NaN; we catch that with
    /// `!(t_min <= t_max)` and return `(INF, INF)` so `f64::clamp`
    /// accepts it and the infeasibility propagates as a poor fitness
    /// instead of panicking.
    pub fn query_range(&self, seg_i: usize, dep: f64) -> (f64, f64) {
        let t = &self.tables[seg_i];
        let (idx, frac, _rel) = Self::locate(t, self.range_k, dep);
        #[cfg(feature = "probe-stats")]
        self.record_probe(_rel);
        let mut t_min = f64::INFINITY;
        let mut t_max = f64::NEG_INFINITY;
        let k_mcr = self.k_mcr;
        for k in 0..k_mcr {
            let v = lerp(
                t.times[idx * k_mcr + k],
                t.times[(idx + 1) * k_mcr + k],
                frac,
            );
            if v < t_min {
                t_min = v;
            }
            if v > t_max {
                t_max = v;
            }
        }
        if t_min.is_nan() || t_max.is_nan() || t_max < t_min {
            return (f64::INFINITY, f64::INFINITY);
        }
        (t_min, t_max)
    }

    /// Invert: `mcr_01` whose travel time matches `delta_time`. Clamps
    /// to `[worst_mcr, 1.0]` when out of tabulated range. Replaces a
    /// 12-iter bisection in the inner fitness path; error is two
    /// lerps of a smooth monotone curve — see [`DEFAULT_K_MCR`].
    pub fn query_mcr_for_delta_time(&self, seg_i: usize, dep: f64, delta_time: f64) -> f64 {
        let t = &self.tables[seg_i];
        let (idx, frac, _rel) = Self::locate(t, self.range_k, dep);
        #[cfg(feature = "probe-stats")]
        self.record_probe(_rel);

        let k_mcr = self.k_mcr;
        let wmcr = t.worst_mcr;
        let t_worst = lerp(t.times[idx * k_mcr], t.times[(idx + 1) * k_mcr], frac);
        let t_best = lerp(
            t.times[idx * k_mcr + (k_mcr - 1)],
            t.times[(idx + 1) * k_mcr + (k_mcr - 1)],
            frac,
        );

        if delta_time >= t_worst {
            return wmcr;
        }
        if delta_time <= t_best {
            return 1.0;
        }

        // Linear scan beats binary search for the typical k_mcr=8;
        // `times` is monotone and small enough to stay cache-hot.
        let mut k_hi = 0;
        let mut k_lo = k_mcr - 1;
        let mut t_at_hi = t_worst;
        let mut t_at_lo = t_best;
        for k in 1..k_mcr {
            let t_k = lerp(
                t.times[idx * k_mcr + k],
                t.times[(idx + 1) * k_mcr + k],
                frac,
            );
            if t_k <= delta_time {
                k_hi = k - 1;
                k_lo = k;
                t_at_hi = lerp(
                    t.times[idx * k_mcr + (k - 1)],
                    t.times[(idx + 1) * k_mcr + (k - 1)],
                    frac,
                );
                t_at_lo = t_k;
                break;
            }
        }

        let mcr_at_hi = mcr_at(wmcr, k_hi, k_mcr);
        let mcr_at_lo = mcr_at(wmcr, k_lo, k_mcr);

        let span = t_at_hi - t_at_lo;
        let frac_t = if span > 0.0 {
            (t_at_hi - delta_time) / span
        } else {
            0.0
        };
        lerp(mcr_at_hi, mcr_at_lo, frac_t)
    }
}

fn pad_range(lo: f64, hi: f64) -> (f64, f64) {
    let span = hi - lo;
    if span <= 0.0 {
        return (lo, hi);
    }
    let pad = span * RANGE_PADDING;
    (lo - pad, hi + pad)
}

#[expect(
    clippy::float_cmp,
    reason = "exact equality is the precise degenerate-range check."
)]
fn dep_at(dep_min: f64, dep_max: f64, k: usize, range_k: usize) -> f64 {
    if range_k <= 1 || dep_max == dep_min {
        return dep_min;
    }
    let frac = k as f64 / (range_k - 1) as f64;
    dep_min + frac * (dep_max - dep_min)
}

/// `[wmcr, 1.0]` uniformly in `k_mcr` steps.
fn mcr_at(wmcr: f64, k: usize, k_mcr: usize) -> f64 {
    if k_mcr <= 1 {
        return wmcr;
    }
    let frac = k as f64 / (k_mcr - 1) as f64;
    wmcr + frac * (1.0 - wmcr)
}
