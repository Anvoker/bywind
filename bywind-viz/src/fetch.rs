//! Background worker for the `File → Fetch from AWS…` dialog.
//!
//! Wraps `bywind::fetch::fetch_to_grib2` in an OS thread, streams
//! [`FetchEvent`]s back to the UI thread over an mpsc channel, and
//! exposes a shared cancel flag the dialog flips when the user clicks
//! Cancel. On success the worker also performs the GRIB2 → `.wcav`
//! transcode in-thread (when the output path's extension warrants it)
//! so the UI only sees the final artifact.

use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
    mpsc::{Receiver, Sender},
};

use bywind::{
    TimedWindMap,
    fetch::{
        FetchProgress, FetchSpec, fetch_to_grib2, parse_yyyymmddhh as parse_yyyymmddhh_lib,
        transcode_grib2_to_wcav,
    },
    io::Format,
};
use chrono::{DateTime, Timelike as _, Utc};

/// One event from the fetch worker. The receiver drains all available
/// events each UI frame and walks `Done` once to close the loop.
pub(crate) enum FetchEvent {
    /// Per-frame status from the GFS bucket.
    Progress(FetchProgress),
    /// Started the GRIB2 → `.wcav` transcode (only fires when the output
    /// extension is `.wcav`). The UI uses this to show the encode phase
    /// separately from the network phase.
    EncodingStarted,
    /// Terminal event: either the loaded wind map on success or the
    /// error string on failure. The UI uses this to dismiss the
    /// "in-progress" state and slot the map into `wind_map` when present.
    Done(Result<TimedWindMap, String>),
}

/// State held by [`crate::app::BywindApp`] for the fetch dialog +
/// background worker. The dialog renderer reads `log` / `phase`; the
/// update loop drains `rx` per frame.
#[derive(Default)]
pub(crate) struct FetchJob {
    /// Lines shown in the dialog's log area. Capped at
    /// [`MAX_LOG_LINES`] so a long fetch doesn't grow the buffer
    /// unboundedly.
    pub(crate) log: Vec<String>,
    /// Phase indicator. The UI uses it to label the "running"
    /// state and to disable Start while a worker is alive.
    pub(crate) phase: FetchPhase,
    rx: Option<Receiver<FetchEvent>>,
    cancel: Option<Arc<AtomicBool>>,
}

/// Maximum lines kept in [`FetchJob::log`]. The cap is far higher than
/// any sane fetch produces (1 line per frame × at most a few hundred
/// frames) — it exists only to bound runaway error spam.
const MAX_LOG_LINES: usize = 500;

#[derive(Default, PartialEq, Eq, Clone, Copy)]
pub(crate) enum FetchPhase {
    #[default]
    Idle,
    Fetching,
    Encoding,
    Cancelling,
}

impl FetchJob {
    pub(crate) fn is_running(&self) -> bool {
        !matches!(self.phase, FetchPhase::Idle)
    }

    /// Bind a freshly-spawned worker to this job. The dialog reads
    /// `phase` to decide which controls to enable, so we transition
    /// straight to `Fetching` here.
    pub(crate) fn attach(&mut self, rx: Receiver<FetchEvent>, cancel: Arc<AtomicBool>) {
        self.rx = Some(rx);
        self.cancel = Some(cancel);
        self.phase = FetchPhase::Fetching;
    }

    /// Flip the shared cancel flag and remember we're in the
    /// post-cancel waiting window. The worker checks the flag at every
    /// `FetchProgress` event so the next per-frame turnaround terminates.
    pub(crate) fn request_cancel(&mut self) {
        if let Some(flag) = &self.cancel {
            flag.store(true, Ordering::Release);
        }
        if self.phase == FetchPhase::Fetching {
            self.phase = FetchPhase::Cancelling;
        }
    }

    /// Drain any pending events from the worker and apply them. Returns
    /// the decoded `TimedWindMap` when the worker has finished
    /// successfully so the caller can swap it into the app's
    /// `wind_map`. Drops the channel on terminal events.
    pub(crate) fn poll(&mut self) -> Option<TimedWindMap> {
        // Drain into a local Vec first so we don't hold an immutable
        // borrow on `self.rx` while pushing into `self.log`. Per-frame
        // event counts are tiny (at most a few hundred over a multi-
        // minute fetch), so the extra allocation is irrelevant.
        let events = {
            let rx = self.rx.as_ref()?;
            let mut buf = Vec::new();
            // Track whether we've drained the worker's terminal event.
            // When the worker exits cleanly, `Done(_)` lands first and
            // then the channel disconnects — without this flag we'd
            // mistake the graceful close for a panic and push a spurious
            // error after the real result.
            let mut saw_done = false;
            loop {
                match rx.try_recv() {
                    Ok(ev) => {
                        if matches!(&ev, FetchEvent::Done(_)) {
                            saw_done = true;
                        }
                        buf.push(ev);
                    }
                    Err(std::sync::mpsc::TryRecvError::Empty) => break,
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                        if !saw_done {
                            buf.push(FetchEvent::Done(Err(
                                "worker disconnected without a Done event".to_owned(),
                            )));
                        }
                        break;
                    }
                }
            }
            buf
        };

        let mut delivered = None;
        for ev in events {
            match ev {
                FetchEvent::Progress(p) => self.append_log(format_progress(&p)),
                FetchEvent::EncodingStarted => {
                    self.phase = FetchPhase::Encoding;
                    self.append_log("encoding to .wcav…".to_owned());
                }
                FetchEvent::Done(Ok(map)) => {
                    self.append_log("done".to_owned());
                    self.phase = FetchPhase::Idle;
                    delivered = Some(map);
                    self.rx = None;
                    self.cancel = None;
                }
                FetchEvent::Done(Err(msg)) => {
                    self.append_log(format!("error: {msg}"));
                    self.phase = FetchPhase::Idle;
                    self.rx = None;
                    self.cancel = None;
                }
            }
        }
        delivered
    }

    /// Reset the log and phase so a re-run starts from a clean slate.
    pub(crate) fn reset_log(&mut self) {
        self.log.clear();
    }

    fn append_log(&mut self, line: String) {
        self.log.push(line);
        if self.log.len() > MAX_LOG_LINES {
            let excess = self.log.len() - MAX_LOG_LINES;
            self.log.drain(..excess);
        }
    }
}

/// Spawn a worker that runs the spec and writes to `out_path`. On
/// success the worker re-opens the artifact, decodes it, and ships the
/// `TimedWindMap` over the channel so the UI thread can swap it in.
///
/// The `cancel` flag is the same one [`FetchJob::request_cancel`] flips;
/// the worker honours it at every progress event.
pub(crate) fn spawn_worker(
    spec: FetchSpec,
    out_path: PathBuf,
    ctx: egui::Context,
) -> (Receiver<FetchEvent>, Arc<AtomicBool>) {
    let (tx, rx) = std::sync::mpsc::channel();
    let cancel = Arc::new(AtomicBool::new(false));
    let cancel_for_worker = Arc::clone(&cancel);
    std::thread::spawn(move || {
        let result = run_worker(&spec, &out_path, &tx, &cancel_for_worker, &ctx);
        drop(tx.send(FetchEvent::Done(result)));
        ctx.request_repaint();
    });
    (rx, cancel)
}

/// Same flow as `bywind-cli`'s `fetch` subcommand, but instead of
/// `eprintln!` we send events down `tx`.
fn run_worker(
    spec: &FetchSpec,
    out_path: &Path,
    tx: &Sender<FetchEvent>,
    cancel: &Arc<AtomicBool>,
    ctx: &egui::Context,
) -> Result<TimedWindMap, String> {
    let out_fmt = Format::from_path(out_path).map_err(|e| format!("{e}"))?;

    let staging = match out_fmt {
        Format::Grib2 => out_path.to_path_buf(),
        Format::WindAv1 => out_path.with_extension("grib2.tmp"),
    };

    {
        let file =
            File::create(&staging).map_err(|e| format!("creating {}: {e}", staging.display()))?;
        let mut writer = BufWriter::new(file);
        let tx_ref = tx.clone();
        let ctx_ref = ctx.clone();
        let cancel_ref = Arc::clone(cancel);
        fetch_to_grib2(spec, &mut writer, |event| {
            drop(tx_ref.send(FetchEvent::Progress(event)));
            ctx_ref.request_repaint();
            if cancel_ref.load(Ordering::Acquire) {
                std::ops::ControlFlow::Break(())
            } else {
                std::ops::ControlFlow::Continue(())
            }
        })
        .map_err(|e| format!("{e}"))?;
    }

    if cancel.load(Ordering::Acquire) {
        // The user cancelled mid-fetch; leave the partial staging file
        // on disk only if it's the user-named output (.grib2 case).
        // Otherwise clean up the tmp.
        if out_fmt == Format::WindAv1 {
            drop(std::fs::remove_file(&staging));
        }
        return Err("cancelled".to_owned());
    }

    if out_fmt == Format::Grib2 {
        // Re-open and decode so the UI can swap the map in.
        use std::io::BufReader;
        let reader = BufReader::new(
            File::open(&staging).map_err(|e| format!("opening {}: {e}", staging.display()))?,
        );
        return TimedWindMap::from_grib2_reader(reader, 1, None)
            .map_err(|e| format!("decoding fetched GRIB2: {e}"));
    }

    // `.wcav` path: decode the staged GRIB2, re-encode as wcav, then
    // also keep the in-memory map so the UI swap doesn't have to decode
    // the file we just wrote.
    drop(tx.send(FetchEvent::EncodingStarted));
    ctx.request_repaint();
    let map = transcode_grib2_to_wcav(&staging, out_path).map_err(|e| e.to_string())?;
    drop(std::fs::remove_file(&staging));
    Ok(map)
}

fn format_progress(p: &FetchProgress) -> String {
    match p {
        FetchProgress::Fetched {
            idx,
            total,
            timestamp,
            bytes,
        } => format!(
            "[{idx:3}/{total:3}] {}  ok ({} KB)",
            timestamp.format("%Y-%m-%d %H:%M UTC"),
            bytes / 1024,
        ),
        FetchProgress::Skipped {
            idx,
            total,
            timestamp,
            reason,
        } => format!(
            "[{idx:3}/{total:3}] {}  skipped: {reason}",
            timestamp.format("%Y-%m-%d %H:%M UTC"),
        ),
    }
}

/// Format a `DateTime<Utc>` as `YYYYMMDDHH` for the dialog's text
/// fields. Matches the `bywind-cli fetch` argument shape exactly.
pub(crate) fn format_yyyymmddhh(t: DateTime<Utc>) -> String {
    t.format("%Y%m%d%H").to_string()
}

/// Thin `String`-error wrapper over [`bywind::fetch::parse_yyyymmddhh`]
/// so the dialog's inline error path can splice the message straight
/// into the toast.
pub(crate) fn parse_yyyymmddhh(s: &str) -> Result<DateTime<Utc>, String> {
    parse_yyyymmddhh_lib(s).map_err(|e| e.to_string())
}

/// Snap `t` down to the most recent 6 h GFS cycle (00 / 06 / 12 / 18 UTC).
pub(crate) fn snap_to_cycle(t: DateTime<Utc>) -> DateTime<Utc> {
    let hour = t.hour() / 6 * 6;
    t.with_hour(hour)
        .and_then(|t| t.with_minute(0))
        .and_then(|t| t.with_second(0))
        .and_then(|t| t.with_nanosecond(0))
        .expect("the resulting (h, 0, 0, 0) is always a valid time")
}
