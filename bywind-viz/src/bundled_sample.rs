//! Optional embedded `wind_av1` sample dataset.
//!
//! `build.rs` sets the `bundled_sample_available` cfg flag whenever
//! `assets/sample_wind.wcav` exists at compile time; this module then
//! `include_bytes!`'s it into the binary and the viz app spawns a
//! worker thread on startup that decodes it and slots the resulting
//! `TimedWindMap` in. No cargo feature toggle involved — present file
//! = bundled sample, missing file = empty slice and `wind_map` stays
//! `None` until the user loads or generates one.
//!
//! Decoding takes ~12 s for the 720-hour file on a modern CPU
//! (pure-Rust rav1d, no NASM). The async pattern (thread +
//! `AsyncJob`) keeps the GUI interactive while it runs; on failure we
//! surface a toast and `wind_map` stays `None`.
//!
//! Gated on `not(wasm32)`: wasm has no `wind_av1` decoder, so
//! embedding ~25 MB into the wasm bundle for no payoff isn't a trade
//! we'd make.

// The whole point of this module is the large include — the lint is
// a warning so this `expect` attribute documents the intent.
#[expect(
    clippy::large_include_file,
    reason = "embedded sample dataset, intentionally large"
)]
#[cfg(all(bundled_sample_available, not(target_arch = "wasm32")))]
pub(crate) const BUNDLED_WCAV: &[u8] = include_bytes!("../assets/sample_wind.wcav");

#[cfg(not(all(bundled_sample_available, not(target_arch = "wasm32"))))]
pub(crate) const BUNDLED_WCAV: &[u8] = &[];

/// `true` if a `wind_av1` sample is embedded in this build. Used by
/// the app to decide whether to spawn the startup decoder.
pub(crate) const fn has_bundled_sample() -> bool {
    !BUNDLED_WCAV.is_empty()
}
