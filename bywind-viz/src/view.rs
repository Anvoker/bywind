/// View / camera transform for the central panel: maps between screen
/// pixels and `(lon°, lat°)` map coordinates via an internal
/// equirectangular projection. Built fresh each frame in `update()` from
/// `panel_rect`, [`crate::config::ViewState::pan_offset`], and
/// [`crate::config::ViewState::render_scale`].
///
/// All map-space callers pass `(lon, lat)` directly — this struct hides
/// the projection. The projection origin (`lat0`, `lon0`) is whichever
/// point lands at screen offset `self.offset` and is typically the
/// centre of the loaded wind map's bounding box, picked by the central
/// panel before constructing the transform.
#[derive(Clone, Copy)]
pub(crate) struct ViewTransform {
    /// Equirectangular projection origin: latitude (degrees). Sets the
    /// `cos(lat0)` longitude-scale factor.
    pub(crate) lat0: f32,
    /// Equirectangular projection origin: longitude (degrees). Subtracted
    /// from each point's longitude before scaling.
    pub(crate) lon0: f32,
    /// Pre-computed `cos(lat0)` in radians, kept on the struct so every
    /// `latlon_to_screen` call is a few multiplies rather than a
    /// trig hit.
    pub(crate) cos_lat0: f32,
    /// Pixels per real ground metre at `lat0`. Replaces the previous
    /// unitless `render_scale`.
    pub(crate) render_scale: f32,
    /// Pixel offset of the projected origin within the panel.
    pub(crate) offset: egui::Vec2,
}

/// Projected metres per degree of latitude on the sphere — derived from the
/// same `EARTH_RADIUS_M = 6_371_000` that `swarmkit-sailing::spherical`
/// uses, so distances reported by the search and pixel sizes drawn here
/// stay self-consistent. Kept as `f32` because `egui::Pos2`'s components
/// are `f32`.
pub(crate) const METRES_PER_DEGREE: f32 = (std::f64::consts::PI * 6_371_000.0 / 180.0) as f32;

impl ViewTransform {
    /// Project `(lon°, lat°)` → screen pixel via equirectangular at
    /// `(lat0, lon0)`. North maps to negative-y on screen (egui's
    /// y-down convention).
    pub(crate) fn map_to_screen(&self, p: egui::Pos2) -> egui::Pos2 {
        let dx_m = (p.x - self.lon0) * self.cos_lat0 * METRES_PER_DEGREE;
        let dy_m = -(p.y - self.lat0) * METRES_PER_DEGREE;
        egui::Pos2::new(
            dx_m * self.render_scale + self.offset.x,
            dy_m * self.render_scale + self.offset.y,
        )
    }

    /// Inverse of [`Self::map_to_screen`]: screen pixel → `(lon°, lat°)`.
    pub(crate) fn screen_to_map(&self, p: egui::Pos2) -> egui::Pos2 {
        let dx_px = p.x - self.offset.x;
        let dy_px = p.y - self.offset.y;
        let lon = self.lon0 + dx_px / (self.render_scale * self.cos_lat0 * METRES_PER_DEGREE);
        let lat = self.lat0 - dy_px / (self.render_scale * METRES_PER_DEGREE);
        egui::Pos2::new(lon, lat)
    }

    /// Pixel-space delta → map-space `(Δlon°, Δlat°)`. Position-independent
    /// because equirectangular scales uniformly: the same pixel delta
    /// gives the same `(Δlon, Δlat)` regardless of *where* the cursor is.
    /// (This is the property that makes equirectangular nice for
    /// click-and-drag interactions; the latitude-dependent distortion is
    /// already baked into `cos_lat0` at construction time.)
    pub(crate) fn screen_delta_to_map(&self, d: egui::Vec2) -> egui::Vec2 {
        egui::Vec2::new(
            d.x / (self.render_scale * self.cos_lat0 * METRES_PER_DEGREE),
            -d.y / (self.render_scale * METRES_PER_DEGREE),
        )
    }

    /// `(lon, lat)` → screen, identical to [`Self::map_to_screen`] except
    /// it accepts `f64` route coordinates (PSO uses `f64` throughout).
    pub(crate) fn route_to_screen(&self, lon: f64, lat: f64) -> egui::Pos2 {
        self.map_to_screen(egui::Pos2::new(lon as f32, lat as f32))
    }

    /// Pixel-space delta → route-space `(Δlon°, Δlat°)` as `f64`. Used
    /// by waypoint dragging, where the new waypoint position is
    /// `current + delta`.
    pub(crate) fn screen_delta_to_route(&self, d: egui::Vec2) -> (f64, f64) {
        let m = self.screen_delta_to_map(d);
        (f64::from(m.x), f64::from(m.y))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::{Pos2, Vec2};

    const EPS: f32 = 1e-3;

    fn approx(a: f32, b: f32) -> bool {
        (a - b).abs() <= EPS
    }

    /// Relative tolerance for values whose magnitude approaches
    /// `METRES_PER_DEGREE` (~1.1e5). f32's ~6 significant digits leave
    /// ~0.1 of absolute slack at that scale; trig like `cos(60°)`
    /// alone produces ~3e-3 of error after multiplication.
    fn approx_rel(a: f32, b: f32) -> bool {
        let scale = a.abs().max(b.abs()).max(1.0);
        (a - b).abs() <= scale * 1e-5
    }

    fn approx_pos(a: Pos2, b: Pos2) -> bool {
        approx(a.x, b.x) && approx(a.y, b.y)
    }

    fn view_at(lat0: f32, lon0: f32, scale: f32, offset: Vec2) -> ViewTransform {
        ViewTransform {
            lat0,
            lon0,
            cos_lat0: lat0.to_radians().cos(),
            render_scale: scale,
            offset,
        }
    }

    #[test]
    fn map_origin_lands_at_offset() {
        let view = view_at(45.0, -10.0, 1e-4, Vec2::new(100.0, 50.0));
        // The projection origin is `(lon0, lat0)` = `(-10, 45)`. It maps
        // to the offset, regardless of scale.
        let screen = view.map_to_screen(Pos2::new(-10.0, 45.0));
        assert!(approx_pos(screen, Pos2::new(100.0, 50.0)));
    }

    #[test]
    fn one_degree_north_maps_negative_y() {
        // Moving 1° north of the origin moves toward smaller screen y
        // (egui convention: y-down). At 1e-3 px/m, 1° lat ≈ 111195 m
        // ≈ 111 px, and the y component is negative.
        let view = view_at(0.0, 0.0, 1e-3, Vec2::ZERO);
        let screen = view.map_to_screen(Pos2::new(0.0, 1.0));
        assert!(approx(screen.x, 0.0));
        assert!(screen.y < 0.0);
        assert!(approx_rel(screen.y, -METRES_PER_DEGREE * 1e-3));
    }

    #[test]
    fn one_degree_east_at_equator_is_full_metres_per_degree() {
        // cos(0°) = 1, so 1° east at the equator = METRES_PER_DEGREE in x.
        let view = view_at(0.0, 0.0, 1.0, Vec2::ZERO);
        let screen = view.map_to_screen(Pos2::new(1.0, 0.0));
        assert!(approx_rel(screen.x, METRES_PER_DEGREE));
        assert!(approx(screen.y, 0.0));
    }

    #[test]
    fn one_degree_east_at_60n_is_half_metres_per_degree() {
        // cos(60°) = 0.5, so 1° east at lat=60° = METRES_PER_DEGREE * 0.5.
        let view = view_at(60.0, 0.0, 1.0, Vec2::ZERO);
        let screen = view.map_to_screen(Pos2::new(1.0, 60.0));
        assert!(approx_rel(screen.x, METRES_PER_DEGREE * 0.5));
    }

    #[test]
    fn map_to_screen_then_back_is_identity() {
        let view = view_at(45.0, -10.0, 1e-4, Vec2::new(123.0, -456.0));
        let p = Pos2::new(-7.5, 47.0);
        let round = view.screen_to_map(view.map_to_screen(p));
        assert!(approx_pos(round, p));
    }

    #[test]
    fn screen_to_map_then_back_is_identity() {
        let view = view_at(0.0, 0.0, 1e-2, Vec2::new(50.0, 50.0));
        let s = Pos2::new(312.5, -78.25);
        let round = view.map_to_screen(view.screen_to_map(s));
        assert!(approx_pos(round, s));
    }

    #[test]
    fn route_to_screen_matches_map_to_screen() {
        let view = view_at(30.0, 15.0, 1e-3, Vec2::new(8.0, -2.0));
        let route = view.route_to_screen(20.0, 32.0);
        let map = view.map_to_screen(Pos2::new(20.0, 32.0));
        assert!(approx_pos(route, map));
    }

    #[test]
    fn screen_delta_to_map_is_translation_invariant() {
        // Position-independent because equirectangular has uniform scale.
        let view_a = view_at(45.0, 0.0, 2.0, Vec2::ZERO);
        let view_b = view_at(45.0, 0.0, 2.0, Vec2::new(999.0, -42.0));
        let d = Vec2::new(100.0, 50.0);
        let a = view_a.screen_delta_to_map(d);
        let b = view_b.screen_delta_to_map(d);
        assert!(approx(a.x, b.x));
        assert!(approx(a.y, b.y));
    }

    #[test]
    fn screen_delta_to_route_round_trips_through_map() {
        let view = view_at(45.0, 0.0, 1e-3, Vec2::new(20.0, 30.0));
        let (dx, dy) = view.screen_delta_to_route(Vec2::new(40.0, -80.0));
        let m = view.screen_delta_to_map(Vec2::new(40.0, -80.0));
        assert!((dx - f64::from(m.x)).abs() < 1e-9);
        assert!((dy - f64::from(m.y)).abs() < 1e-9);
    }

    #[test]
    fn lon_projection_is_purely_linear() {
        // The ViewTransform does no antimeridian wrapping — callers do it
        // themselves if they care. Two longitudes 360° apart in the raw
        // input space are projected as a 360° difference in screen space,
        // even though they describe the same geographic point. This is
        // intentional: the projection is a pure linear function in lon
        // and lat, and antimeridian handling is the caller's concern.
        let view = view_at(0.0, 0.0, 1.0, Vec2::ZERO);
        let a = view.map_to_screen(Pos2::new(179.0, 0.0));
        let b = view.map_to_screen(Pos2::new(-181.0, 0.0));
        let dx = (a.x - b.x).abs();
        let expected = 360.0 * METRES_PER_DEGREE;
        assert!(approx_rel(dx, expected));
    }
}
