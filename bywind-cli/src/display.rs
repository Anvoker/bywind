//! CLI-side output sinks for route summaries. The value-producing
//! formatters (durations, fuel, land distance, PSO-vs-benchmark deltas)
//! live in [`bywind::fmt`] so the GUI's stats panel and the CLI's stderr
//! output stay byte-identical. This module only owns the bits that
//! actually write to stderr.

use bywind::{
    SegmentMetrics,
    fmt::{format_duration_breakdown, format_fuel_auto, format_land_km},
};

/// Print the per-segment table with `|` separators column-aligned. Each
/// cell is pre-formatted, then per-column widths are computed across the
/// whole table so the output stays readable even when (for example)
/// adjacent segments span very different durations.
pub fn print_segment_table(segment_stats: &[SegmentMetrics]) {
    if segment_stats.is_empty() {
        return;
    }
    // Columns: time | motor | fuel | speed | land. Build the cells once
    // so both the width pass and the print pass see the same strings.
    let cells: Vec<[String; 5]> = segment_stats
        .iter()
        .map(|m| {
            [
                format_duration_breakdown(m.time),
                format!("motor={:.2}", m.mcr_01),
                format!("fuel={}", format_fuel_auto(m.fuel)),
                format!("speed={:.1} km/h", m.speed_kmh),
                format!("land={}", format_land_km(m.land_metres)),
            ]
        })
        .collect();
    let mut widths = [0_usize; 5];
    for row in &cells {
        for (cell, w) in row.iter().zip(widths.iter_mut()) {
            if cell.len() > *w {
                *w = cell.len();
            }
        }
    }
    let idx_width = (segment_stats.len().saturating_sub(1)).to_string().len();
    for (i, row) in cells.iter().enumerate() {
        eprintln!(
            "[{:>iw$}] {:<w0$} | {:<w1$} | {:<w2$} | {:<w3$} | {:<w4$}",
            i,
            row[0],
            row[1],
            row[2],
            row[3],
            row[4],
            iw = idx_width,
            w0 = widths[0],
            w1 = widths[1],
            w2 = widths[2],
            w3 = widths[3],
            w4 = widths[4],
        );
    }
}
