//! Categorised CLI error type so `main` can return graded exit codes.
//!
//! Most errors flow through the codebase as `anyhow::Error` and are treated
//! as `BadInput` by default (the user's file / flags / TOML had a problem).
//! A few sites explicitly construct `NoResult` (the inputs were valid but
//! couldn't yield a route — empty wind map, degenerate bounds, search
//! returned no iterations) or `Internal` (an invariant violation that
//! shouldn't happen in normal use).
//!
//! Exit codes for shell scripting:
//! - `0` — success
//! - `1` — bad input (file not found, parse error, validation failure)
//! - `2` — no result possible (data was valid but produces no route)
//! - `3` — internal / unexpected (probable bug)

use std::fmt;
use std::process::ExitCode;

/// Top-level error reported back to `main`. Wraps an `anyhow::Error` for the
/// formatted message and carries an `ExitClass` for the exit-code mapping.
#[derive(Debug)]
pub enum AppError {
    BadInput(anyhow::Error),
    NoResult(anyhow::Error),
    Internal(anyhow::Error),
}

impl AppError {
    /// Construct a `NoResult` error from anything convertible to `anyhow::Error`.
    pub fn no_result(e: impl Into<anyhow::Error>) -> Self {
        Self::NoResult(e.into())
    }

    /// Construct an `Internal` error from anything convertible to `anyhow::Error`.
    pub fn internal(e: impl Into<anyhow::Error>) -> Self {
        Self::Internal(e.into())
    }

    /// Map this error to its corresponding exit code.
    pub fn exit_code(&self) -> ExitCode {
        match self {
            Self::BadInput(_) => ExitCode::from(1),
            Self::NoResult(_) => ExitCode::from(2),
            Self::Internal(_) => ExitCode::from(3),
        }
    }

    /// Borrow the wrapped error for formatting.
    fn inner(&self) -> &anyhow::Error {
        match self {
            Self::BadInput(e) | Self::NoResult(e) | Self::Internal(e) => e,
        }
    }
}

impl fmt::Display for AppError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // `{:#}` on anyhow::Error walks the source chain. Same convention as
        // the Phase 1 dispatcher that just printed `{e:#}`.
        write!(f, "{:#}", self.inner())
    }
}

impl std::error::Error for AppError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(self.inner().as_ref())
    }
}

/// Default conversion: any unannotated `anyhow::Error` is treated as
/// `BadInput`. Subcommand entry points keep using `?` exactly as before;
/// only specific sites that mean "valid input, no result" or "internal
/// invariant violation" need to construct `NoResult` / `Internal`
/// explicitly. Other concrete error types convert via
/// `.map_err(anyhow::Error::from)?` (or equivalent) at the call site.
impl From<anyhow::Error> for AppError {
    fn from(e: anyhow::Error) -> Self {
        Self::BadInput(e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_anyhow_defaults_to_bad_input() {
        let err: AppError = anyhow::anyhow!("bad path").into();
        assert!(matches!(err, AppError::BadInput(_)));
    }

    #[test]
    fn no_result_constructor_categorises_correctly() {
        let err = AppError::no_result(anyhow::anyhow!("empty map"));
        assert!(matches!(err, AppError::NoResult(_)));
    }

    #[test]
    fn internal_constructor_categorises_correctly() {
        let err = AppError::internal(anyhow::anyhow!("invariant broken"));
        assert!(matches!(err, AppError::Internal(_)));
    }

    #[test]
    fn display_walks_anyhow_source_chain() {
        let inner = anyhow::anyhow!("root cause")
            .context("middle")
            .context("top");
        let err: AppError = inner.into();
        let s = format!("{err}");
        // anyhow's `{:#}` is `top: middle: root cause`.
        assert!(s.contains("top"));
        assert!(s.contains("root cause"));
    }
}
