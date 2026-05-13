//! Loading mechanism for the optional `wind_av1` sample dataset.
//!
//! Three layers, in priority order, all driving the same
//! `bundled_sample_job` worker spawned at startup:
//!
//! 1. **Embedded.** `build.rs` sets `bundled_sample_available` whenever
//!    `assets/sample_wind.wcav` is present at compile time, and this
//!    module `include_bytes!`s it. The cargo-dist GitHub-release
//!    binaries follow this path — the file is tracked in git, so it's
//!    there when CI runs.
//! 2. **Cached.** When no embed is available, look in the OS cache dir
//!    (`directories::ProjectDirs`) for a previously-downloaded copy.
//!    Subsequent launches of a `cargo install`ed binary hit this path.
//! 3. **Downloaded.** First launch of a `cargo install`ed binary
//!    fetches the sample from `raw.githubusercontent.com` pinned to
//!    the binary's `CARGO_PKG_VERSION` tag, then writes it to the
//!    cache for #2 on the next launch.
//!
//! The crates.io-published `bywind-viz` excludes `sample_wind.wcav`
//! from its `include` list (the file is ~26 MB, over the registry's
//! 10 MiB limit). The bundled-sample mechanism therefore exists in
//! two flavours: embedded for the cargo-dist binary releases,
//! cache-or-download for the crates.io install path.
//!
//! Gated on `not(target_arch = "wasm32")` — wasm has no `wind_av1`
//! decoder and no native HTTP client, so the whole flow is native-only.

#[expect(
    clippy::large_include_file,
    reason = "embedded sample dataset, intentionally large"
)]
#[cfg(all(bundled_sample_available, not(target_arch = "wasm32")))]
pub(crate) const BUNDLED_WCAV: &[u8] = include_bytes!("../assets/sample_wind.wcav");

#[cfg(not(all(bundled_sample_available, not(target_arch = "wasm32"))))]
pub(crate) const BUNDLED_WCAV: &[u8] = &[];

/// `true` if a `wind_av1` sample is embedded in this build. Used to
/// distinguish the "instant" embed path from the "may download" cache
/// path in the menu hover text.
pub(crate) const fn has_bundled_sample() -> bool {
    !BUNDLED_WCAV.is_empty()
}

/// `true` if the running build can load a sample at all — embed,
/// cache, or download. Always `true` on native; `false` on wasm (no
/// decoder, no HTTP client).
pub(crate) const fn can_load_sample() -> bool {
    !cfg!(target_arch = "wasm32")
}

#[cfg(not(target_arch = "wasm32"))]
pub(crate) use native::load_sample_bytes;

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use std::path::PathBuf;

    /// Where the non-embedded path downloads from. Version-pinned via
    /// `CARGO_PKG_VERSION` so an old `cargo install`ed binary can't
    /// accidentally pick up a future sample format from a newer release.
    pub(crate) const SAMPLE_URL: &str = concat!(
        "https://raw.githubusercontent.com/Anvoker/bywind/v",
        env!("CARGO_PKG_VERSION"),
        "/bywind-viz/assets/sample_wind.wcav",
    );

    /// On-disk cache location for the downloaded sample. The filename
    /// includes `CARGO_PKG_VERSION` so a newer binary downloads its
    /// own pinned sample rather than reusing a stale one from a
    /// previous version. Returns `None` only when the platform has no
    /// resolvable home/cache dir (rare; some headless environments).
    fn cache_path() -> Option<PathBuf> {
        let dirs = directories::ProjectDirs::from("dev", "Anvoker", "bywind-viz")?;
        let name = concat!("sample_wind-v", env!("CARGO_PKG_VERSION"), ".wcav");
        Some(dirs.cache_dir().join(name))
    }

    /// Returns the raw `.wcav` bytes for the sample, walking
    /// embed → cache → download in priority order. The download is
    /// best-effort-cached for next launch; cache write failures are
    /// logged but don't propagate.
    ///
    /// Error strings are caller-displayable; the worker thread surfaces
    /// them directly to the UI toast.
    pub(crate) fn load_sample_bytes() -> Result<Vec<u8>, String> {
        if !super::BUNDLED_WCAV.is_empty() {
            return Ok(super::BUNDLED_WCAV.to_vec());
        }
        if let Some(path) = cache_path() {
            if path.is_file() {
                match std::fs::read(&path) {
                    Ok(bytes) if !bytes.is_empty() => return Ok(bytes),
                    Ok(_) => log::warn!(
                        "cached sample at {} was empty; redownloading",
                        path.display(),
                    ),
                    Err(e) => log::warn!(
                        "cached sample at {} unreadable ({e}); redownloading",
                        path.display(),
                    ),
                }
            }
        }
        let bytes = download().map_err(|e| format!("download from {SAMPLE_URL}: {e}"))?;
        if let Some(path) = cache_path() {
            if let Some(parent) = path.parent() {
                drop(std::fs::create_dir_all(parent));
            }
            if let Err(e) = std::fs::write(&path, &bytes) {
                log::warn!("failed to cache sample at {}: {e}", path.display());
            }
        }
        Ok(bytes)
    }

    fn download() -> Result<Vec<u8>, String> {
        use std::io::Read as _;
        let mut resp = ureq::get(SAMPLE_URL).call().map_err(|e| e.to_string())?;
        let mut bytes = Vec::with_capacity(26 * 1024 * 1024);
        resp.body_mut()
            .as_reader()
            .read_to_end(&mut bytes)
            .map_err(|e| e.to_string())?;
        Ok(bytes)
    }
}
