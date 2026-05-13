//! Detects the optional bundled wind dataset and sets the
//! `bundled_sample_available` cfg flag when present. The flag drives a
//! conditional `include_bytes!` in `src/bundled_sample.rs`, so dropping
//! `assets/sample_wind.wcav` into place is all it takes to enable the
//! embedded sample — no cargo feature toggle, no manifest edit.
//!
//! When the file is absent, the cfg stays unset and the embedded slice
//! is empty. `bundled_sample.rs` then falls back to its cache + download
//! path on native (the crates.io-published crate excludes the asset for
//! size, so `cargo install` builds always take this path). Plain
//! `cargo build` works on fresh clones with or without the asset.

fn main() {
    // Rust 1.80 requires this declaration before `rustc-cfg` lines are
    // accepted without a `unexpected_cfgs` warning.
    println!("cargo::rustc-check-cfg=cfg(bundled_sample_available)");

    let manifest_dir =
        std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR set by cargo");
    let asset = std::path::Path::new(&manifest_dir).join("assets/sample_wind.wcav");
    if asset.is_file() {
        println!("cargo::rustc-cfg=bundled_sample_available");
    }
    // Rebuild whenever the asset is added, removed, or replaced so the
    // bundled state stays in sync without a manual `cargo clean`.
    println!("cargo::rerun-if-changed=assets/sample_wind.wcav");
}
