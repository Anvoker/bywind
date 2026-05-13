//! Human-readable formatters shared by the GUI's stats panel and the
//! CLI's summary output.
//!
//! Lifted out of both `bywind-viz` and `bywind-cli` so the on-screen
//! and on-stderr renderings of the same numbers stay byte-identical
//! without copy-paste.
//!
//! Pure value-producing helpers — no I/O. Callers are responsible for
//! routing the resulting strings to the right surface (egui label, stderr
//! `eprintln!`, etc.).

/// Format a duration in seconds as `Nd Nh Nm Ns`, dropping leading
/// zero units so a one-minute route reads `30s` rather than
/// `0d 0h 0m 30s`.
///
/// Sub-second precision is dropped — the granularity matches the visual
/// scale this format is meant for (long routes, hours+).
pub fn format_duration_breakdown(total_seconds: f64) -> String {
    let total = total_seconds.max(0.0) as u64;
    let days = total / 86_400;
    let hours = (total % 86_400) / 3600;
    let minutes = (total % 3600) / 60;
    let seconds = total % 60;
    if days > 0 {
        format!("{days}d {hours}h {minutes}m {seconds}s")
    } else if hours > 0 {
        format!("{hours}h {minutes}m {seconds}s")
    } else if minutes > 0 {
        format!("{minutes}m {seconds}s")
    } else {
        format!("{seconds}s")
    }
}

/// Format a fuel mass (kilograms), auto-switching units: tonnes
/// (`12.345 t`) at or above 1 t, kilograms (`12345.67 kg`) otherwise.
///
/// Used where the same number can span a wide dynamic range — e.g. the
/// CLI's totals row, where short test routes might be under a tonne and
/// long real-world routes are tens of tonnes.
pub fn format_fuel_auto(kg: f64) -> String {
    if kg >= 1000.0 {
        format!("{:.3} t", kg / 1000.0)
    } else {
        format!("{kg:.2} kg")
    }
}

/// Format an over-land distance (metres) as kilometres with two
/// decimals.
///
/// A correctly-routed search reads `0.00 km` here — non-zero is a
/// sanity flag that the route is crossing landmass.
pub fn format_land_km(metres: f64) -> String {
    format!("{:.2} km", metres / 1000.0)
}

/// Format a fitness value as a compact magnitude with a `K` / `M`
/// suffix and three decimals — e.g. `-1825302.28` → `"1.825M"`.
///
/// Sign is dropped (in this system fitness is `-(time + fuel + land)`
/// and almost always negative; the magnitude is what readers compare).
/// The `M` cutoff is `|value| >= 1.0M`; below that, `K` is used:
/// `-173235.24` → `"173.235K"`.
pub fn format_fitness_magnitude(value: f64) -> String {
    let abs = value.abs();
    if abs >= 1_000_000.0 {
        format!("{:.3}M", abs / 1_000_000.0)
    } else {
        format!("{:.3}K", abs / 1_000.0)
    }
}

/// Format the PSO-vs-benchmark delta as a human-readable phrase like
/// `"PSO 23.4% better"` / `"PSO 5.1% worse"` / `"PSO equal"` /
/// `"PSO N/A"`.
///
/// `larger_is_better` flips the comparison: pass `true` for fitness
/// (higher is better), `false` for time / fuel / land (lower is better).
/// Pure ASCII so it renders in any font the egui frontend ends up
/// loading.
pub fn format_pso_delta(pso: f64, bench: f64, larger_is_better: bool) -> String {
    if bench.abs() < 1e-12 {
        return "PSO N/A".to_owned();
    }
    let diff = pso - bench;
    if diff.abs() < 1e-9 {
        return "PSO equal".to_owned();
    }
    let pct = (diff.abs() / bench.abs()) * 100.0;
    let pso_better = if larger_is_better {
        diff > 0.0
    } else {
        diff < 0.0
    };
    if pso_better {
        format!("PSO {pct:.1}% better")
    } else {
        format!("PSO {pct:.1}% worse")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_duration_breakdown_drops_leading_zero_units() {
        assert_eq!(format_duration_breakdown(45.0), "45s");
        assert_eq!(format_duration_breakdown(125.0), "2m 5s");
        assert_eq!(format_duration_breakdown(3.0 * 3600.0 + 4.0), "3h 0m 4s");
        assert_eq!(
            format_duration_breakdown(2.0 * 86_400.0 + 3.0 * 3600.0),
            "2d 3h 0m 0s",
        );
    }

    #[test]
    fn format_fuel_auto_picks_unit_by_threshold() {
        assert_eq!(format_fuel_auto(0.0), "0.00 kg");
        assert_eq!(format_fuel_auto(999.99), "999.99 kg");
        assert_eq!(format_fuel_auto(1000.0), "1.000 t");
        assert_eq!(format_fuel_auto(12_345.0), "12.345 t");
    }

    #[test]
    fn format_land_km_renders_metres_as_two_decimal_km() {
        assert_eq!(format_land_km(0.0), "0.00 km");
        assert_eq!(format_land_km(1500.0), "1.50 km");
        assert_eq!(format_land_km(12_345.0), "12.35 km");
    }

    #[test]
    fn format_fitness_magnitude_uses_m_above_one_million() {
        assert_eq!(format_fitness_magnitude(-1_825_302.276_8), "1.825M");
        assert_eq!(format_fitness_magnitude(-1_000_000.0), "1.000M");
        // Just under the breakpoint flips to K.
        assert_eq!(format_fitness_magnitude(-999_999.999), "1000.000K");
        assert_eq!(format_fitness_magnitude(-173_235.238_9), "173.235K");
        assert_eq!(format_fitness_magnitude(0.0), "0.000K");
    }

    #[test]
    fn format_pso_delta_handles_signs_and_edges() {
        assert_eq!(format_pso_delta(80.0, 100.0, false), "PSO 20.0% better");
        assert_eq!(format_pso_delta(120.0, 100.0, false), "PSO 20.0% worse");
        assert_eq!(format_pso_delta(1.2, 1.0, true), "PSO 20.0% better");
        assert_eq!(format_pso_delta(100.0, 100.0, false), "PSO equal");
        assert_eq!(format_pso_delta(50.0, 0.0, false), "PSO N/A");
    }
}
