use crate::spherical::{LatLon, Segment, Wind, destination_point, haversine, initial_bearing};
use crate::{Sailboat, WindSource};

/// Sailboat physics model: motor + wind-driven speed and fuel-burn cubic.
///
/// Implements [`Sailboat`] using a polar curve for wind-driven speed and a
/// cubic SFC for fuel rate. Used as the boat type for the bywind sailing
/// search; see [`Boat::default`] for the calibrated mid-size-marine-diesel
/// preset that the rest of the workspace exercises in tests and examples.
pub struct Boat {
    /// Maximum continuous engine rating (watts).
    pub mcr: f64,
    /// Hydrodynamic drag coefficient in `P = 0.5 · k · v³`.
    pub k: f64,
    /// Polar curve scale: speed = `polar_c` · (1 + |sin φ|^`polar_sin_power`) / 2 · TWS,
    /// where φ is the angle between heading and wind. Endpoints are unaffected:
    /// 0.5·`polar_c`·TWS at downwind/upwind (|sin φ| = 0), `polar_c`·TWS at beam reach
    /// (|sin φ| = 1).
    pub polar_c: f64,
    /// Polar curve sharpness exponent on |sin φ|. At 1 the curve is linear in
    /// |sin φ| (gentle peak); larger values flatten off-beam angles and concentrate
    /// speed near 90°.
    pub polar_sin_power: f64,
    /// Cubic SFC coefficients in the fuel-rate model
    /// `mcr_01 · (fuel_a + fuel_b·mcr_01 + fuel_c·mcr_01²)`. Pinning `fuel_b = -1.6·fuel_c`
    /// places the SFC minimum at -`fuel_b/(2·fuel_c)` = 0.8 (80% load).
    pub fuel_a: f64,
    pub fuel_b: f64,
    pub fuel_c: f64,
}

impl Default for Boat {
    /// Mid-size marine diesel at MCR = 1 MW. The fuel-rate cubic produces a
    /// U-shaped SFC with minimum ~235 g/kWh at 80% load, rising to ~296 g/kWh
    /// at 10% load and 240 g/kWh at full load (= 240 kg/h).
    fn default() -> Self {
        Self {
            mcr: 1_000_000.0,
            k: 4000.0,
            polar_c: 1.5,
            polar_sin_power: 1.0,
            fuel_a: 0.0875,
            fuel_b: -0.0555,
            fuel_c: 0.0347,
        }
    }
}

impl Boat {
    /// Polar response: speed gained from wind given a heading bearing
    /// (compass radians, `0 = north`) and a wind sample in the local
    /// east-north tangent frame at the segment evaluation point.
    ///
    /// Computes `|sin φ|` (where φ is the angle between heading and
    /// wind) directly from the bearing trig and the wind components,
    /// avoiding an explicit heading vector: the heading unit vector
    /// would be `(sin(bearing), cos(bearing))` in (east, north), so
    /// `heading × wind = sin(bearing)·wind.north_mps − cos(bearing)·wind.east_mps`,
    /// and `|sin φ| = |that| / tws`.
    pub fn get_wind_speed(&self, bearing_rad: f64, wind: Wind) -> f64 {
        let (sin_b, cos_b) = bearing_rad.sin_cos();
        self.get_wind_speed_with_sin_cos(sin_b, cos_b, wind)
    }

    /// Like [`Self::get_wind_speed`] but takes precomputed
    /// `(sin(bearing), cos(bearing))`. Useful inside the substep loops
    /// of `get_travel_time` / `get_travel_times_for_mcrs`, where the
    /// same bearing feeds both the start-of-substep and mid-substep
    /// wind samples (so the sin/cos can be computed once per substep
    /// and reused).
    fn get_wind_speed_with_sin_cos(&self, sin_b: f64, cos_b: f64, wind: Wind) -> f64 {
        let tws = wind.speed();
        if tws == 0.0 {
            return 0.0;
        }
        let cross = sin_b * wind.north_mps - cos_b * wind.east_mps;
        let abs_sin_phi = cross.abs() / tws;
        self.polar_c * (1.0 + abs_sin_phi.powf(self.polar_sin_power)) / 2.0 * tws
    }

    /// Fuel burn rate (kg/s) at load `mcr_01 ∈ [0, 1]`.
    pub fn get_fuel_rate(&self, mcr_01: f64) -> f64 {
        if mcr_01 <= 0.0 {
            0.0
        } else {
            mcr_01 * (self.fuel_a + self.fuel_b * mcr_01 + self.fuel_c * mcr_01 * mcr_01)
        }
    }

    fn get_speed_from_wind_and_motor(&self, windborne_speed: f64, mcr_01: f64) -> f64 {
        self.speed_from_motor_power(windborne_speed, mcr_01 * self.mcr)
    }

    fn speed_from_motor_power(&self, windborne_speed: f64, motor_power: f64) -> f64 {
        (windborne_speed.powi(3) + motor_power / (0.5 * self.k)).cbrt()
    }
}

impl Sailboat for Boat {
    fn get_fuel_consumed(&self, mcr_01: f64, delta_time: f64) -> f64 {
        if delta_time <= 0.0 || mcr_01 <= 0.0 || self.mcr <= 0.0 {
            return 0.0;
        }
        self.get_fuel_rate(mcr_01) * delta_time
    }

    fn get_travel_time<WS: WindSource>(
        &self,
        wind_source: &WS,
        segment: Segment,
        mcr_01: f64,
    ) -> f64 {
        let Segment {
            origin,
            destination,
            origin_time,
            step_distance_max,
        } = segment;
        // `origin` and `destination` are `(lon°, lat°)`; `step_distance_max`
        // is real ground metres. We integrate along the great-circle path
        // from `origin` toward `destination`, recomputing the bearing each
        // step so floating-point drift can't cause us to miss the
        // destination. The heading vector handed to the polar curve is in
        // the local east-north tangent frame at the segment, matching the
        // wind vector's frame from `wind_source`.
        let total_distance = haversine(origin, destination);
        if total_distance <= 0.0 {
            return 0.0;
        }

        let step_count = (total_distance / step_distance_max).ceil().max(1.0) as usize;
        let step_distance = total_distance / step_count as f64;
        let mut position = origin;
        let mut time = origin_time;

        for _ in 0..step_count {
            let Some(bearing) = initial_bearing(position, destination) else {
                // We're at a pole; the bearing is undefined. Mark the
                // path infeasible — same outcome as the `speed <= 0.0`
                // branches below.
                return f64::INFINITY;
            };
            // Compute (sin, cos) once per substep and reuse for both
            // wind-speed evaluations below. `f64::sin_cos` is one
            // hardware op pair instead of two.
            let (sin_b, cos_b) = bearing.sin_cos();

            let wind_start = wind_source.sample_wind(position, time);
            let windborne_start = self.get_wind_speed_with_sin_cos(sin_b, cos_b, wind_start);
            let speed_start = self.get_speed_from_wind_and_motor(windborne_start, mcr_01);
            // Spell out the NaN case explicitly: NaN — possible when the
            // wind sampler returns NaN at a degenerate position — must also
            // flag the path infeasible, otherwise it would propagate into
            // `step_distance / speed` and then into the final delta.
            if speed_start.is_nan() || speed_start <= 0.0 {
                return f64::INFINITY;
            }
            let half_dt = 0.5 * step_distance / speed_start;

            let Some(mid_position) = destination_point(position, step_distance * 0.5, bearing)
            else {
                return f64::INFINITY;
            };
            let mid_time = time + half_dt;
            let wind_mid = wind_source.sample_wind(mid_position, mid_time);
            let windborne_mid = self.get_wind_speed_with_sin_cos(sin_b, cos_b, wind_mid);
            let speed = self.get_speed_from_wind_and_motor(windborne_mid, mcr_01);
            // See `speed_start` check above — same NaN / non-positive guard.
            if speed.is_nan() || speed <= 0.0 {
                return f64::INFINITY;
            }

            let Some(next_position) = destination_point(position, step_distance, bearing) else {
                return f64::INFINITY;
            };
            position = next_position;
            time += step_distance / speed;
        }

        let delta = time - origin_time;
        // Defence in depth: the per-step `!(speed > 0.0)` checks above
        // already filter NaN, but if some new failure mode leaks NaN into
        // `time` we'd rather flag the path infeasible than panic the worker.
        if delta.is_nan() {
            return f64::INFINITY;
        }
        delta
    }

    fn get_travel_times_for_mcrs<WS: WindSource>(
        &self,
        wind_source: &WS,
        segment: Segment,
        mcr_01_values: &[f64],
        out: &mut [f64],
    ) {
        let Segment {
            origin,
            destination,
            origin_time,
            step_distance_max,
        } = segment;
        debug_assert_eq!(
            mcr_01_values.len(),
            out.len(),
            "out buffer must match input mcr_01 length",
        );
        if mcr_01_values.is_empty() {
            return;
        }

        let total_distance = haversine(origin, destination);
        if total_distance <= 0.0 {
            out.fill(0.0);
            return;
        }

        let step_count = (total_distance / step_distance_max).ceil().max(1.0) as usize;
        let step_distance = total_distance / step_count as f64;

        // Pre-compute the great-circle substep geometry. The walk from
        // `origin` toward `destination` is mcr-independent — every mcr
        // visits the same sequence of substep positions and therefore
        // shares the same per-substep `bearing`, midpoint, and next
        // point. Doing this once per segment avoids repeating
        // `initial_bearing` + 2 × `destination_point` per substep per
        // mcr, which in `SegmentRangeTables::build` is 8× redundant
        // (one geometry walk shared across `K_MCR` mcr samples).
        //
        // If the walk hits a pole-degenerate substep, mark every mcr's
        // result `f64::INFINITY` and bail — same semantics as the
        // per-mcr loop in `get_travel_time`.
        let mut positions: Vec<LatLon> = Vec::with_capacity(step_count);
        let mut mid_positions: Vec<LatLon> = Vec::with_capacity(step_count);
        let mut bearing_sin_cos: Vec<(f64, f64)> = Vec::with_capacity(step_count);
        let mut walker = origin;
        for _ in 0..step_count {
            let Some(bearing) = initial_bearing(walker, destination) else {
                out.fill(f64::INFINITY);
                return;
            };
            let Some(mid) = destination_point(walker, step_distance * 0.5, bearing) else {
                out.fill(f64::INFINITY);
                return;
            };
            let Some(next) = destination_point(walker, step_distance, bearing) else {
                out.fill(f64::INFINITY);
                return;
            };
            positions.push(walker);
            mid_positions.push(mid);
            bearing_sin_cos.push(bearing.sin_cos());
            walker = next;
        }

        // Replay through the precomputed geometry for each mcr. Time
        // accumulation diverges per mcr (faster boats reach the same
        // position sooner), so the wind samples differ by their
        // `(position, time)` even though `position` is shared.
        for (i, &mcr_01) in mcr_01_values.iter().enumerate() {
            let mut time = origin_time;
            let mut bad = false;
            for k in 0..step_count {
                let position = positions[k];
                let mid_position = mid_positions[k];
                let (sin_b, cos_b) = bearing_sin_cos[k];

                let wind_start = wind_source.sample_wind(position, time);
                let windborne_start = self.get_wind_speed_with_sin_cos(sin_b, cos_b, wind_start);
                let speed_start = self.get_speed_from_wind_and_motor(windborne_start, mcr_01);
                if speed_start.is_nan() || speed_start <= 0.0 {
                    bad = true;
                    break;
                }
                let half_dt = 0.5 * step_distance / speed_start;

                let mid_time = time + half_dt;
                let wind_mid = wind_source.sample_wind(mid_position, mid_time);
                let windborne_mid = self.get_wind_speed_with_sin_cos(sin_b, cos_b, wind_mid);
                let speed = self.get_speed_from_wind_and_motor(windborne_mid, mcr_01);
                if speed.is_nan() || speed <= 0.0 {
                    bad = true;
                    break;
                }

                time += step_distance / speed;
            }
            out[i] = if bad {
                f64::INFINITY
            } else {
                let delta = time - origin_time;
                if delta.is_nan() { f64::INFINITY } else { delta }
            };
        }
    }
}
