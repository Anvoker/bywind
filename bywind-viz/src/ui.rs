use crate::config::Tool;
use crate::draw::{
    draw_benchmark_route, draw_coastlines, draw_endpoint_markers, draw_minimap, draw_route_bounds,
    draw_windmap, render_route_evolution,
};
use crate::view::ViewTransform;
use bywind::{
    BoatConfig, LonLatBbox, MapBounds, TimedWindMap, Topology, WaypointCount,
    fmt::{format_duration_breakdown, format_fitness_magnitude, format_land_km, format_pso_delta},
    route_evolution_match,
};

mod central;
mod dialogs;
mod menu;
mod panels;

/// Render-scale slider bounds. The lower end is a backstop for the
/// first frame (before the central panel has reported its height); the
/// effective minimum used at runtime is dynamic — see
/// [`min_render_scale`].
pub(super) const SCALE_MIN: f32 = 1e-5;
pub(super) const SCALE_MAX: f32 = 4.0;
/// Pixel padding around the wind map inside the central panel; shared by
/// `render_central_panel` and the autoscale fit so they agree on the
/// available drawing area.
pub(super) const MAP_PADDING: f32 = 20.0;

/// Smallest render scale that still keeps 180° of latitude (the full
/// world height) covering at least `panel_height` pixels. Falls back to
/// [`SCALE_MIN`] until the central panel has reported a height (first
/// frame, before layout has run).
pub(super) fn min_render_scale(panel_height: f32) -> f32 {
    if panel_height > 0.0 {
        (panel_height / (180.0 * crate::view::METRES_PER_DEGREE)).max(SCALE_MIN)
    } else {
        SCALE_MIN
    }
}

/// Format `render_scale` (pixels per ground metre) as a map-style
/// "1 px : N km / m / cm" readout. Unit is auto-picked from magnitude so
/// the number stays in a readable range across the full slider span.
pub(super) fn format_scale_value(render_scale: f32) -> String {
    let m_per_px = f64::from(1.0 / render_scale.max(f32::EPSILON));
    if m_per_px >= 1000.0 {
        format!("1 px : {} km", format_quantity(m_per_px / 1000.0))
    } else if m_per_px >= 1.0 {
        format!("1 px : {} m", format_quantity(m_per_px))
    } else {
        format!("1 px : {} cm", format_quantity(m_per_px * 100.0))
    }
}

/// Inverse of [`format_scale_value`] for the slider's text-input edit
/// path. Accepts "1 px : N km|m|cm" (whitespace and the "1 px :" prefix
/// optional) and returns the corresponding pixels-per-metre value, or
/// `None` if the input doesn't match.
pub(super) fn parse_scale_value(s: &str) -> Option<f64> {
    let body = s
        .trim()
        .strip_prefix("1 px :")
        .or_else(|| s.trim().strip_prefix("1px:"))
        .unwrap_or(s)
        .trim();
    let (num_str, metres_per_unit) = if let Some(n) = body.strip_suffix("km") {
        (n.trim(), 1000.0_f64)
    } else if let Some(n) = body.strip_suffix("cm") {
        (n.trim(), 0.01_f64)
    } else if let Some(n) = body.strip_suffix('m') {
        (n.trim(), 1.0_f64)
    } else {
        return None;
    };
    let n: f64 = num_str.parse().ok()?;
    if n <= 0.0 {
        return None;
    }
    Some(1.0 / (n * metres_per_unit))
}

/// Pretty-print a positive quantity with decimals scaled to its magnitude
/// so a 1-decimal readout never reads as "153.7 km" but also doesn't lose
/// precision at the small end.
pub(super) fn format_quantity(v: f64) -> String {
    if v >= 10.0 {
        format!("{v:.0}")
    } else if v >= 1.0 {
        format!("{v:.1}")
    } else {
        format!("{v:.2}")
    }
}

/// Visual cap on the slider+stepper row width so a wide left panel
/// doesn't render a monstrously long bar.
pub(super) const SLIDER_ROW_MAX_WIDTH: f32 = 240.0;

/// Paint a soft pulsing outline around the Set Endpoints selectable,
/// used as the "you forgot to set endpoints" prompt. The pulse is a
/// 1.5-Hz sine on the stroke's alpha + thickness so it's noticeable
/// without being seizure-inducing; the warm-orange matches the route-
/// node colour so the cue ties back to placing waypoints on the map.
///
/// `StrokeKind::Outside` keeps the outline outside the selectable's
/// rect so it never overlaps the label text — important because
/// painting happens after `selectable_value` and any fill would
/// otherwise cover the glyphs.
pub(super) fn paint_endpoint_highlight(ui: &egui::Ui, rect: egui::Rect) {
    let t = ui.input(|i| i.time);
    let pulse = 0.5 + 0.5 * (t * std::f64::consts::TAU * 1.5).sin();
    let alpha = (160.0 + 95.0 * pulse) as u8;
    let width = 1.5 + 1.5 * pulse as f32;
    let stroke = egui::Stroke::new(
        width,
        egui::Color32::from_rgba_unmultiplied(255, 140, 40, alpha),
    );
    ui.painter()
        .rect_stroke(rect, 2.0, stroke, egui::StrokeKind::Outside);
}

/// Linear `usize` slider over `0..=max` with `[−]` `[+]` step-by-1 buttons.
/// The slider hides its numeric readout (matches the existing time- and
/// iteration-scrubber UX); callers render the human-readable label
/// alongside if they want it. Width grows with the panel up to
/// [`SLIDER_ROW_MAX_WIDTH`].
pub(super) fn int_slider_with_steppers(ui: &mut egui::Ui, value: &mut usize, max: usize) {
    ui.horizontal(|ui| {
        if ui.small_button("−").clicked() && *value > 0 {
            *value -= 1;
        }
        expand_slider_width(ui);
        ui.add(egui::Slider::new(value, 0..=max).show_value(false));
        if ui.small_button("+").clicked() && *value < max {
            *value += 1;
        }
    });
}

/// Logarithmic `f32` slider over `start..=end` with `[−]` `[+]` buttons
/// that scale the value by ±10% (the multiplicative step matches the
/// log-scale layout — additive steps would feel huge at one end of the
/// range and invisible at the other). Slider hides its built-in
/// readout; callers render a `DragValue` above for typed input. Width
/// grows with the panel up to [`SLIDER_ROW_MAX_WIDTH`].
pub(super) fn log_slider_with_steppers(ui: &mut egui::Ui, value: &mut f32, start: f32, end: f32) {
    const STEP_FACTOR: f32 = 1.1;
    ui.horizontal(|ui| {
        if ui.small_button("−").clicked() {
            *value = (*value / STEP_FACTOR).clamp(start, end);
        }
        expand_slider_width(ui);
        ui.add(
            egui::Slider::new(value, start..=end)
                .logarithmic(true)
                .show_value(false),
        );
        if ui.small_button("+").clicked() {
            *value = (*value * STEP_FACTOR).clamp(start, end);
        }
    });
}

/// Set `slider_width` so the slider fills the row's remaining width
/// (after the leading "−" button) minus a reserve for the trailing
/// "+" button + spacing, clamped to [40, [`SLIDER_ROW_MAX_WIDTH`]].
/// Shared by `int_slider_with_steppers` and `log_slider_with_steppers`
/// so both rows stay visually aligned.
pub(super) fn expand_slider_width(ui: &mut egui::Ui) {
    let trailing_button_reserve = 24.0;
    let target = (ui.available_width() - trailing_button_reserve).clamp(40.0, SLIDER_ROW_MAX_WIDTH);
    ui.spacing_mut().slider_width = target;
}

/// Format a fuel mass in kilograms as either tonnes (`12.345 t`) or
/// kilograms (`12345.67 kg`), depending on the user's Summary cycle state.
/// Distinct from [`bywind::fmt::format_fuel_auto`] (which auto-switches by
/// magnitude) because the GUI keeps consistent units across the panel
/// rather than swapping per-row.
pub(super) fn format_fuel(kg: f64, tonnes: bool) -> String {
    if tonnes {
        format!("{:.3} t", kg / 1000.0)
    } else {
        format!("{kg:.2} kg")
    }
}

/// Number of polyline segments used to draw the fuel-rate curve. The
/// underlying function is at most cubic, so this resolution is overkill for
/// smoothness — the only concern is keeping the polyline visually crisp when
/// the panel is wide.
pub(super) const FUEL_CURVE_SAMPLES: usize = 64;

/// Inline plot of `fuel_rate(mcr_01) = mcr_01 · (a + b·mcr_01 + c·mcr_01²)`
/// over `mcr_01 ∈ [0, 1]`. Auto-scales the y-axis to the curve's own range so
/// users can see the shape regardless of the SFC magnitude. Drawn directly
/// with the painter to avoid pulling in `egui_plot` for a single plot.
pub(super) fn draw_fuel_curve(ui: &mut egui::Ui, fuel_a: f64, fuel_b: f64, fuel_c: f64) {
    let width = ui.available_width().clamp(80.0, 220.0);
    let desired_size = egui::Vec2::new(width, 80.0);
    let (rect, _) = ui.allocate_exact_size(desired_size, egui::Sense::hover());

    let fuel_rate = |mcr_01: f64| -> f64 {
        if mcr_01 <= 0.0 {
            0.0
        } else {
            mcr_01 * (fuel_a + fuel_b * mcr_01 + fuel_c * mcr_01 * mcr_01)
        }
    };

    let mut samples: Vec<(f64, f64)> = Vec::with_capacity(FUEL_CURVE_SAMPLES + 1);
    let mut y_min = 0.0_f64;
    let mut y_max = 0.0_f64;
    for i in 0..=FUEL_CURVE_SAMPLES {
        let mcr = i as f64 / FUEL_CURVE_SAMPLES as f64;
        let y = fuel_rate(mcr);
        y_min = y_min.min(y);
        y_max = y_max.max(y);
        samples.push((mcr, y));
    }
    // Avoid divide-by-zero when the curve is flat (e.g. all coefficients zero);
    // the visual range is meaningless then but the widget still needs to render.
    let y_span = (y_max - y_min).max(1e-12);

    let visuals = ui.visuals();
    let painter = ui.painter_at(rect);
    painter.rect(
        rect,
        2.0,
        visuals.extreme_bg_color,
        egui::Stroke::new(1.0, visuals.weak_text_color()),
        egui::StrokeKind::Inside,
    );

    let to_screen = |mcr: f64, y: f64| -> egui::Pos2 {
        let t_x = mcr.clamp(0.0, 1.0) as f32;
        let t_y = ((y - y_min) / y_span) as f32;
        egui::Pos2::new(
            rect.left() + t_x * rect.width(),
            rect.bottom() - t_y * rect.height(),
        )
    };

    // Zero baseline only when the curve straddles zero — for the default
    // coefficients y_min == 0 and the baseline coincides with the bottom edge,
    // so drawing it would just thicken the frame.
    if y_min < 0.0 && y_max > 0.0 {
        let zero_y = to_screen(0.0, 0.0).y;
        painter.line_segment(
            [
                egui::Pos2::new(rect.left(), zero_y),
                egui::Pos2::new(rect.right(), zero_y),
            ],
            egui::Stroke::new(1.0, visuals.weak_text_color()),
        );
    }

    let curve_points: Vec<egui::Pos2> = samples.iter().map(|&(mcr, y)| to_screen(mcr, y)).collect();
    painter.add(egui::Shape::line(
        curve_points,
        egui::Stroke::new(1.5, visuals.text_color()),
    ));
}

pub(super) fn powered_by_egui_and_eframe(ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 0.0;
        ui.label("Powered by ");
        ui.hyperlink_to("egui", "https://github.com/emilk/egui");
        ui.label(" and ");
        ui.hyperlink_to(
            "eframe",
            "https://github.com/emilk/egui/tree/master/crates/eframe",
        );
        ui.label(".");
    });
}

/// Replace the trailing extension on `path` with `new_ext` (without
/// the leading dot). Empty paths or paths without an extension get
/// `.<new_ext>` appended instead. Used by the fetch dialog to keep
/// `out_path` in step with the Format combo.
#[cfg(not(target_arch = "wasm32"))]
pub(super) fn sync_out_path_extension(path: &mut String, new_ext: &str) {
    if path.is_empty() {
        return;
    }
    let pb = std::path::PathBuf::from(&*path);
    let updated = pb.with_extension(new_ext);
    *path = updated.to_string_lossy().into_owned();
}
