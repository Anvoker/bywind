//! Shared CLI parsing helpers. The per-flag wrappers in `convert.rs` and
//! `search.rs` (`parse_bbox` / `parse_lonlat` / `parse_bounds_4`) all
//! parse a comma-separated list of floats — only the float type, the
//! component count, and the field names differ. They build their typed
//! value (`Grib2Bbox`, `[f64; 2]`, `[f64; 4]`) on top of
//! [`parse_n_floats`] here so error messages and edge-case behaviour
//! stay consistent across flags.

use std::str::FromStr;

use anyhow::{Context as _, Result, bail};

/// Split `s` on commas, trim each component, parse as `T`, and require
/// exactly `N` components. `names` is the per-component label list used
/// in the per-component parse-error message (so a bad `--bounds` value
/// reads `parsing lat_min from --bounds: invalid float literal` rather
/// than `parsing field 1 from --bounds`); `flag` is the CLI-flag name
/// used in both the arity- and the per-component-error messages.
pub fn parse_n_floats<T, const N: usize>(s: &str, names: [&str; N], flag: &str) -> Result<[T; N]>
where
    T: FromStr,
    T::Err: std::error::Error + Send + Sync + 'static,
{
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != N {
        bail!(
            "{flag} expects {N} comma-separated values ({}), got {}",
            names.join(","),
            parts.len(),
        );
    }
    let mut out: [Option<T>; N] = std::array::from_fn(|_| None);
    for ((raw, name), slot) in parts.iter().zip(names.iter()).zip(out.iter_mut()) {
        let value = raw
            .trim()
            .parse::<T>()
            .with_context(|| format!("parsing {name} from {flag}"))?;
        *slot = Some(value);
    }
    // `out` is fully populated by the len-N traversal above; the option
    // wrappers fall away here. `expect` is reachable only if a future edit
    // breaks that invariant.
    Ok(out.map(|v| v.expect("len-N traversal filled every slot")))
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "tests rely on bit-exact comparisons of constant or stored f32/f64 values."
    )]
    use super::*;

    #[test]
    fn parses_four_f64() {
        let r: [f64; 4] = parse_n_floats("1,2,3,4", ["a", "b", "c", "d"], "--flag")
            .expect("valid four-float input");
        assert_eq!(r, [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn parses_f32_just_as_well() {
        let r: [f32; 4] = parse_n_floats("1,2,3,4", ["a", "b", "c", "d"], "--grib-bbox")
            .expect("valid four-float input");
        assert_eq!(r, [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn tolerates_whitespace_around_components() {
        let r: [f64; 2] = parse_n_floats(" 30 , -60 ", ["lon", "lat"], "--flag")
            .expect("whitespace should be tolerated");
        assert_eq!(r, [30.0, -60.0]);
    }

    #[test]
    fn arity_mismatch_too_few_reports_expected_and_actual() {
        let r: Result<[f64; 4]> = parse_n_floats("1,2,3", ["a", "b", "c", "d"], "--flag");
        let err = r.expect_err("3 components, expected 4").to_string();
        assert!(err.contains("--flag"));
        assert!(err.contains("expects 4"));
        assert!(err.contains("got 3"));
    }

    #[test]
    fn arity_mismatch_too_many_is_rejected() {
        let r: Result<[f64; 4]> = parse_n_floats("1,2,3,4,5", ["a", "b", "c", "d"], "--flag");
        r.unwrap_err();
    }

    #[test]
    fn non_numeric_error_carries_field_and_flag_context() {
        let r: Result<[f64; 2]> = parse_n_floats("north,40", ["lon", "lat"], "--start");
        // anyhow `{:#}` walks the source chain so the per-component
        // context is visible alongside the underlying `ParseFloatError`.
        let err = format!("{:#}", r.expect_err("non-numeric input"));
        assert!(err.contains("--start"));
        assert!(err.contains("lon"));
    }
}
