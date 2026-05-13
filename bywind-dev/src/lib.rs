//! Internal diagnostic / benchmarking crate. Excluded from the
//! workspace's `default-members` so plain `cargo build` skips it;
//! binaries are invoked explicitly with
//! `cargo run -p bywind-dev --release --bin <name>`.
//!
//! Binaries live under `src/bin/`:
//!
//! * `av1_round_trip` — encode a GRIB2 through `wind_av1`, decode it
//!   back, and report per-cell `(u, v)` drift between the source
//!   and the round-tripped result. Used to keep the encoder honest
//!   when we tune `rav1e` settings or change the codec format.
//! * `wcav_drift` — compare an existing `.wcav` against a source
//!   GRIB2 cell-by-cell, bucketed by source wind speed. Direction
//!   drift is the headline metric here because `atan2(u, v)` is
//!   pathological in low-wind cells.
//!
//! Anything in this crate is dev-only; production code lives in the
//! `bywind` / `bywind-cli` / `bywind-viz` crates. Tests that are too
//! slow / expensive for the regular `cargo test` run can also live
//! here in `bywind-dev/tests/` (still gated by `cargo test -p
//! bywind-dev`).
