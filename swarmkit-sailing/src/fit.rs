use crate::spherical::Segment;
use crate::units::Path;
use crate::{EnsembleWindSource, LandmassSource, Sailboat, SailboatFitData, WindSource, dynamics};
use swarmkit::{Contextful, FitCalc};

/// Combine route totals into the scalar fitness score.
///
/// Sign convention: higher is better, so minimising cost reads as
/// `-cost`. Single source of truth for the formula; both
/// [`SailboatFitCalc::calculate_fit`] and downstream consumers that
/// recompute fitness from cached segment metrics (e.g. the GUI's
/// drag-edit recompute) route through here.
#[inline]
#[must_use]
pub fn weighted_fitness(
    travel_time_s: f64,
    fuel_kg: f64,
    land_metres: f64,
    time_weight: f64,
    fuel_weight: f64,
    land_weight: f64,
) -> f64 {
    -(travel_time_s * time_weight + fuel_kg * fuel_weight + land_metres * land_weight)
}

pub struct SailboatFitCalc<'a, const N: usize, SB: Sailboat, WS: WindSource, LS: LandmassSource> {
    pub time_weight: f64,
    pub fuel_weight: f64,
    /// Penalty per metre of segment that lies inside a landmass. Zero
    /// recovers the pre-landmass-support fitness (no penalty).
    pub land_weight: f64,
    pub departure_time: f64,
    pub step_distance_max: f64,
    pub ship: &'a SB,
    pub wind_source: &'a WS,
    pub landmass: &'a LS,
}

impl<const N: usize, SB: Sailboat, WS: WindSource, LS: LandmassSource> SailboatFitData
    for SailboatFitCalc<'_, N, SB, WS, LS>
{
    type Ship = SB;
    type Wind = WS;
    type Land = LS;

    fn ship(&self) -> &Self::Ship {
        self.ship
    }
    fn wind_source(&self) -> &Self::Wind {
        self.wind_source
    }
    fn landmass(&self) -> &Self::Land {
        self.landmass
    }
    fn step_distance_max(&self) -> f64 {
        self.step_distance_max
    }
    fn departure_time(&self) -> f64 {
        self.departure_time
    }
    fn time_weight(&self) -> f64 {
        self.time_weight
    }
    fn fuel_weight(&self) -> f64 {
        self.fuel_weight
    }
    fn land_weight(&self) -> f64 {
        self.land_weight
    }
}

impl<const N: usize, SB: Sailboat, WS: WindSource, LS: LandmassSource> Contextful
    for SailboatFitCalc<'_, N, SB, WS, LS>
{
    type TContext = Path<N>;
}

impl<const N: usize, SB: Sailboat, WS: WindSource, LS: LandmassSource> FitCalc
    for SailboatFitCalc<'_, N, SB, WS, LS>
{
    type T = Path<N>;

    fn calculate_fit(&self, position: Self::T) -> f64 {
        #[cfg(feature = "profile-timers")]
        let __profile_start = std::time::Instant::now();

        let (travel_time, fuel_consumed) = walk_segments_against_wind(
            self.ship,
            self.wind_source,
            position,
            self.departure_time,
            self.step_distance_max,
        );
        // The land-metres product with `land_weight == 0` is zero, so
        // the segment walk is wasted work in that mode. Skip it. This
        // preserves the pre-landmass-support hot-path cost exactly for
        // callers that pass `land_weight = 0` and a `LandmassSourceDummy`.
        let land_metres = if self.land_weight == 0.0 {
            0.0
        } else {
            get_route_land_metres(self.landmass, position, self.step_distance_max)
        };
        let fit = weighted_fitness(
            travel_time,
            fuel_consumed,
            land_metres,
            self.time_weight,
            self.fuel_weight,
            self.land_weight,
        );

        #[cfg(feature = "profile-timers")]
        crate::profile_timers::OUTER_FIT.record(__profile_start.elapsed().as_nanos() as u64);

        fit
    }
}

/// Walk every segment of `path` against `wind_source`.
///
/// Returns `(travel_time, fuel_consumed)` summed across the route.
/// Factored out so the ensemble fit calc can reuse the per-member
/// loop without reaching into `SailboatFitCalc` internals.
pub fn walk_segments_against_wind<const N: usize, SB: Sailboat, WS: WindSource>(
    ship: &SB,
    wind_source: &WS,
    path: Path<N>,
    departure_time: f64,
    step_distance_max: f64,
) -> (f64, f64) {
    let mut fuel_consumed = 0.0;
    let mut travel_time = 0.0;
    for seg in path.iter_with_running_clock(departure_time) {
        let (_mcr_01, fuel) = dynamics::get_segment_metrics(
            ship,
            wind_source,
            Segment {
                origin: seg.origin,
                destination: seg.destination,
                origin_time: seg.t_depart,
                step_distance_max,
            },
            seg.t_arrive,
        );
        fuel_consumed += fuel;
        travel_time += seg.segment_time;
    }
    (travel_time, fuel_consumed)
}

/// Sum of [`dynamics::get_segment_land_metres`] across every segment of
/// `path`. Returns `f64::INFINITY` if any segment crosses a pole; `0.0`
/// for a path entirely over open water.
fn get_route_land_metres<const N: usize, LS: LandmassSource>(
    landmass: &LS,
    path: Path<N>,
    step_distance_max: f64,
) -> f64 {
    let mut total = 0.0;
    for seg in path.iter_with_running_clock(0.0) {
        let m = dynamics::get_segment_land_metres(
            landmass,
            seg.origin,
            seg.destination,
            step_distance_max,
        );
        if m.is_infinite() {
            return f64::INFINITY;
        }
        total += m;
    }
    total
}

pub(crate) struct PathFitCalc<'a, const N: usize, TFit>
where
    TFit: FitCalc<T = Path<N>>,
{
    fit_calc: &'a TFit,
}

impl<'a, const N: usize, TFit> PathFitCalc<'a, N, TFit>
where
    TFit: FitCalc<T = Path<N>>,
{
    pub fn new(fit_calc: &'a TFit) -> Self {
        PathFitCalc { fit_calc }
    }
}

impl<const N: usize, TFit> Contextful for PathFitCalc<'_, N, TFit>
where
    TFit: FitCalc<T = Path<N>>,
{
    type TContext = Path<N>;
}

impl<const N: usize, TFit> FitCalc for PathFitCalc<'_, N, TFit>
where
    TFit: FitCalc<T = Path<N>>,
{
    type T = Path<N>;

    const PAR_LEAF_SIZE: usize = TFit::PAR_LEAF_SIZE;

    fn calculate_fit(&self, position: Self::T) -> f64 {
        self.fit_calc.calculate_fit(position)
    }
}

impl<const N: usize, TFit> SailboatFitData for PathFitCalc<'_, N, TFit>
where
    TFit: FitCalc<T = Path<N>> + SailboatFitData,
{
    type Ship = TFit::Ship;
    type Wind = TFit::Wind;
    type Land = TFit::Land;

    fn ship(&self) -> &Self::Ship {
        self.fit_calc.ship()
    }
    fn wind_source(&self) -> &Self::Wind {
        self.fit_calc.wind_source()
    }
    fn landmass(&self) -> &Self::Land {
        self.fit_calc.landmass()
    }
    fn step_distance_max(&self) -> f64 {
        self.fit_calc.step_distance_max()
    }
    fn departure_time(&self) -> f64 {
        self.fit_calc.departure_time()
    }
    fn time_weight(&self) -> f64 {
        self.fit_calc.time_weight()
    }
    fn fuel_weight(&self) -> f64 {
        self.fit_calc.fuel_weight()
    }
    fn land_weight(&self) -> f64 {
        self.fit_calc.land_weight()
    }
}

/// Scalar-reduction strategy applied to a `K`-element vector of
/// per-member fitness samples.
///
/// The first cut ships [`Self::Mean`]; `CVaR` / mean+`CVaR` /
/// worst-case are sketched as future arms.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RobustObjective {
    /// Arithmetic mean of per-member fitness. Cheapest and most
    /// forgiving of forecast outliers in either direction.
    Mean,
    // Reserved for the next phase, not implemented yet:
    // Cvar { alpha_x1000: u32 },              // CVaR(α)
    // MeanPlusCvar { alpha_x1000: u32, lambda_x1000: u32 },
    // WorstCase,
}

/// Ensemble counterpart of [`SailboatFitCalc`].
///
/// Per-particle fitness is the [`RobustObjective`]-reduced scalar
/// over `K` per-member evaluations. Landmass penalty is computed once
/// and reused across members (it's wind-independent), so the K-fold
/// cost is purely the wind-dependent segment walk.
pub struct EnsembleSailboatFitCalc<
    'a,
    const N: usize,
    SB: Sailboat,
    EWS: EnsembleWindSource,
    LS: LandmassSource,
> {
    pub time_weight: f64,
    pub fuel_weight: f64,
    /// Penalty per metre of segment that lies inside a landmass. Zero
    /// recovers the pre-landmass-support fitness (no penalty).
    pub land_weight: f64,
    pub departure_time: f64,
    pub step_distance_max: f64,
    pub ship: &'a SB,
    pub wind_ensemble: &'a EWS,
    pub landmass: &'a LS,
    pub robust_objective: RobustObjective,
}

impl<const N: usize, SB: Sailboat, EWS: EnsembleWindSource, LS: LandmassSource> SailboatFitData
    for EnsembleSailboatFitCalc<'_, N, SB, EWS, LS>
{
    type Ship = SB;
    type Wind = EWS::Member;
    type Land = LS;

    fn ship(&self) -> &Self::Ship {
        self.ship
    }
    /// Returns the *first* member as the canonical wind source for
    /// downstream code paths that haven't yet been ensemble-aware.
    /// They get a representative, not a full picture. The PSO fitness
    /// itself does loop the ensemble in `calculate_fit`.
    fn wind_source(&self) -> &Self::Wind {
        assert!(
            self.wind_ensemble.member_count() > 0,
            "ensemble must have at least one member"
        );
        self.wind_ensemble.member(0)
    }
    fn landmass(&self) -> &Self::Land {
        self.landmass
    }
    fn step_distance_max(&self) -> f64 {
        self.step_distance_max
    }
    fn departure_time(&self) -> f64 {
        self.departure_time
    }
    fn time_weight(&self) -> f64 {
        self.time_weight
    }
    fn fuel_weight(&self) -> f64 {
        self.fuel_weight
    }
    fn land_weight(&self) -> f64 {
        self.land_weight
    }
}

impl<const N: usize, SB: Sailboat, EWS: EnsembleWindSource, LS: LandmassSource> Contextful
    for EnsembleSailboatFitCalc<'_, N, SB, EWS, LS>
{
    type TContext = Path<N>;
}

impl<const N: usize, SB: Sailboat, EWS: EnsembleWindSource, LS: LandmassSource> FitCalc
    for EnsembleSailboatFitCalc<'_, N, SB, EWS, LS>
{
    type T = Path<N>;

    fn calculate_fit(&self, position: Self::T) -> f64 {
        let k = self.wind_ensemble.member_count();
        assert!(k > 0, "ensemble must have at least one member");
        // Land penalty is wind-independent. Compute it once and reuse
        // across members — saves K-1 redundant SDF walks.
        let land_metres = if self.land_weight == 0.0 {
            0.0
        } else {
            get_route_land_metres(self.landmass, position, self.step_distance_max)
        };

        let mut sum_time = 0.0;
        let mut sum_fuel = 0.0;
        for k_idx in 0..k {
            let member = self.wind_ensemble.member(k_idx);
            let (t, f) = walk_segments_against_wind(
                self.ship,
                member,
                position,
                self.departure_time,
                self.step_distance_max,
            );
            sum_time += t;
            sum_fuel += f;
        }

        match self.robust_objective {
            RobustObjective::Mean => {
                let k_f = k as f64;
                weighted_fitness(
                    sum_time / k_f,
                    sum_fuel / k_f,
                    land_metres,
                    self.time_weight,
                    self.fuel_weight,
                    self.land_weight,
                )
            }
        }
    }
}
