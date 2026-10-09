//! Runtime load limits driven by `config.toml` → `[performance]` and `[render]`.
//!
//! Two independent things live here:
//!
//! * [`limit_frame_rate`] paces the main loop when `performance.max_fps` is set.
//!   `window.present_mode` only chooses between vsync and "as fast as possible",
//!   so an explicit cap has to be slept out at the end of the frame.
//! * [`apply_performance_config`] pushes the switches that lower render cost
//!   (virtual-clock clamp, camera far planes, shadow cascade budget) onto the
//!   entities that already exist. It runs every frame and only writes what
//!   actually differs, so a late-spawned capture camera picks the budget up and a
//!   hot-reloaded `config.toml` takes effect on the next frame.
//!
//! The systems that *use* the fields — `physics.substep_count` /
//! `physics.fixed_hz` in [`crate::config`], and `camera.pip_*` in
//! [`crate::systems::camera`] — are unaffected; they read the config directly.

use bevy::light::{CascadeShadowConfig, CascadeShadowConfigBuilder};
use bevy::prelude::*;
use std::time::{Duration, Instant};

use crate::capture::CaptureCamera;
use crate::capture::depth::DepthCaptureCamera;
use crate::components::{MainCamera, PictureInPicture};
use crate::config::{RenderConfig, SimulationConfig};

/// Shadow cascades beyond four stop paying for themselves on the shadow map
/// resolutions this simulator uses.
const MAX_SHADOW_CASCADES: u32 = 4;
/// A cascade budget below a meter is useless; above 10 km it is the engine default.
const MIN_SHADOW_DISTANCE: f32 = 1.0;
const MAX_SHADOW_DISTANCE: f32 = 10_000.0;
/// Camera far planes must stay beyond the 0.1 m near plane of both cameras.
const MIN_VIEW_DISTANCE: f32 = 1.0;
const MAX_VIEW_DISTANCE: f32 = 500_000_000.0;

/// Frame pacing state for `performance.max_fps`. Wall-clock based and anchored to
/// the previous deadline, so the average rate stays on target instead of drifting
/// by however long the frame's own work happened to take.
#[derive(Resource, Default)]
pub struct FrameLimiter {
    deadline: Option<Instant>,
}

/// The `Time<Virtual>` clamp the app started with, so a config that turns
/// `max_frame_delta` back to `0` restores the engine default instead of freezing
/// whatever value was last written.
#[derive(Resource, Deref)]
struct EngineFrameDelta(Duration);

pub struct PerformancePlugin;

impl Plugin for PerformancePlugin {
    fn build(&self, app: &mut App) {
        let engine_delta = app
            .world()
            .get_resource::<Time<Virtual>>()
            .map(|time| time.max_delta())
            .unwrap_or(Duration::from_millis(250));

        app.init_resource::<FrameLimiter>()
            .insert_resource(EngineFrameDelta(engine_delta))
            .add_systems(PostUpdate, apply_performance_config)
            .add_systems(Last, limit_frame_rate);
    }
}

/// Deadline for the next frame and how long to sleep before it.
///
/// `previous` is the deadline the limiter is currently pacing toward — it was
/// computed at the end of the last frame, so it may still lie ahead of `now`.
/// `None` means the limiter was off (or just turned on) and only starts pacing
/// from now. A frame that overran its whole budget resyncs to `now` instead of
/// trying to catch up with a burst of zero-sleep frames.
fn schedule_frame(
    now: Instant,
    previous: Option<Instant>,
    interval: Duration,
) -> (Instant, Duration) {
    let deadline = previous.unwrap_or(now);
    let sleep = deadline.saturating_duration_since(now);
    let target = deadline + interval;
    let next = if target > now { target } else { now + interval };

    (next, sleep)
}

/// Sleeps out the rest of the frame budget. Runs in `Last`, after every system
/// that does work, so the sleep is what actually caps the loop.
fn limit_frame_rate(config: Res<SimulationConfig>, mut limiter: ResMut<FrameLimiter>) {
    let Some(interval) = config.performance.frame_interval() else {
        // Uncapped: forget the deadline so re-enabling starts a fresh cadence.
        limiter.deadline = None;
        return;
    };

    let now = Instant::now();
    let (next, sleep) = schedule_frame(now, limiter.deadline, interval);
    limiter.deadline = Some(next);

    if !sleep.is_zero() {
        std::thread::sleep(sleep);
    }
}

/// Shadow cascade budget, or `None` to leave the engine default in place.
fn cascade_budget(render: &RenderConfig) -> Option<CascadeShadowConfig> {
    let distance = (render.shadow_max_distance.is_finite() && render.shadow_max_distance > 0.0)
        .then(|| {
            render
                .shadow_max_distance
                .clamp(MIN_SHADOW_DISTANCE, MAX_SHADOW_DISTANCE)
        });

    if distance.is_none() && render.shadow_cascades == 0 {
        return None;
    }

    let defaults = CascadeShadowConfigBuilder::default();
    let maximum_distance = distance.unwrap_or(defaults.maximum_distance);
    let num_cascades = if render.shadow_cascades == 0 {
        defaults.num_cascades
    } else {
        render.shadow_cascades.clamp(1, MAX_SHADOW_CASCADES) as usize
    };

    Some(
        CascadeShadowConfigBuilder {
            num_cascades,
            maximum_distance,
            // Keep the first cascade at ~10% of the budget, which reproduces the
            // engine's 10 m / 150 m defaults exactly and scales down with it.
            first_cascade_far_bound: (maximum_distance * 0.1).clamp(1.0, 10.0),
            ..default()
        }
        .build(),
    )
}

/// Camera far plane for the main and capture views, or `None` to keep the
/// built-in value. The depth and inset cameras carry their own far planes
/// (`capture.depth.far`, `camera.pip_far`) and are deliberately left alone.
fn view_distance(render: &RenderConfig) -> Option<f32> {
    (render.view_distance.is_finite() && render.view_distance > 0.0).then(|| {
        render
            .view_distance
            .clamp(MIN_VIEW_DISTANCE, MAX_VIEW_DISTANCE)
    })
}

/// Applies the far plane and the cascade budget to one camera view.
fn apply_view_budget(
    commands: &mut Commands,
    entity: Entity,
    projection: &mut Projection,
    cascades: Option<&mut CascadeShadowConfig>,
    far: Option<f32>,
    budget: Option<&CascadeShadowConfig>,
) {
    if let Some(far) = far
        && let Projection::Perspective(perspective) = projection
        && perspective.far != far
    {
        perspective.far = far;
    }

    let Some(budget) = budget else {
        return;
    };

    match cascades {
        Some(cascades) => {
            if cascades.bounds != budget.bounds
                || cascades.minimum_distance != budget.minimum_distance
                || cascades.overlap_proportion != budget.overlap_proportion
            {
                *cascades = budget.clone();
            }
        }
        // No component yet: inserting one is what switches the view off the
        // engine default, and it happens once because it is present afterwards.
        None => {
            commands.entity(entity).insert(budget.clone());
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn apply_performance_config(
    config: Res<SimulationConfig>,
    engine_delta: Res<EngineFrameDelta>,
    mut virtual_time: ResMut<Time<Virtual>>,
    mut commands: Commands,
    mut main_cameras: Query<
        (Entity, &mut Projection, Option<&mut CascadeShadowConfig>),
        With<MainCamera>,
    >,
    mut capture_cameras: Query<
        (Entity, &mut Projection, Option<&mut CascadeShadowConfig>),
        (With<CaptureCamera>, Without<MainCamera>),
    >,
    mut inset_cameras: Query<
        (Entity, &mut Projection, Option<&mut CascadeShadowConfig>),
        (
            With<PictureInPicture>,
            Without<MainCamera>,
            Without<CaptureCamera>,
        ),
    >,
    mut depth_cameras: Query<
        (Entity, &mut Projection, Option<&mut CascadeShadowConfig>),
        (
            With<DepthCaptureCamera>,
            Without<MainCamera>,
            Without<CaptureCamera>,
            Without<PictureInPicture>,
        ),
    >,
    mut lights: Query<&mut DirectionalLight>,
) {
    let delta_cap = config
        .performance
        .frame_delta_cap()
        .unwrap_or(engine_delta.0);
    if virtual_time.max_delta() != delta_cap {
        virtual_time.set_max_delta(delta_cap);
    }

    let far = view_distance(&config.render);
    let budget = cascade_budget(&config.render);

    for (entity, mut projection, mut cascades) in &mut main_cameras {
        apply_view_budget(
            &mut commands,
            entity,
            &mut projection,
            cascades.as_deref_mut(),
            far,
            budget.as_ref(),
        );
    }

    for (entity, mut projection, mut cascades) in &mut capture_cameras {
        apply_view_budget(
            &mut commands,
            entity,
            &mut projection,
            cascades.as_deref_mut(),
            far,
            budget.as_ref(),
        );
    }

    for (entity, mut projection, mut cascades) in &mut inset_cameras {
        // The inset keeps `camera.pip_far`; only the shadow budget is shared.
        apply_view_budget(
            &mut commands,
            entity,
            &mut projection,
            cascades.as_deref_mut(),
            None,
            budget.as_ref(),
        );
    }

    for (entity, mut projection, mut cascades) in &mut depth_cameras {
        // Depth-only livox view: same deal, `capture.depth.far` stays its own.
        apply_view_budget(
            &mut commands,
            entity,
            &mut projection,
            cascades.as_deref_mut(),
            None,
            budget.as_ref(),
        );
    }

    for mut light in &mut lights {
        if light.illuminance != config.render.illuminance {
            light.illuminance = config.render.illuminance;
        }
        if light.shadow_maps_enabled != config.render.shadows {
            light.shadow_maps_enabled = config.render.shadows;
        }
        if light.contact_shadows_enabled != config.render.shadows {
            light.contact_shadows_enabled = config.render.shadows;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn interval() -> Duration {
        Duration::from_millis(16)
    }

    #[test]
    fn first_frame_only_starts_the_clock() {
        let now = Instant::now();
        let (next, sleep) = schedule_frame(now, None, interval());

        assert_eq!(sleep, Duration::ZERO);
        assert_eq!(next, now + interval());
    }

    #[test]
    fn frame_that_finished_early_sleeps_out_the_budget() {
        let now = Instant::now();
        let deadline = now + Duration::from_millis(6);
        let (next, sleep) = schedule_frame(now, Some(deadline), interval());

        assert_eq!(sleep, Duration::from_millis(6));
        assert_eq!(
            next,
            deadline + interval(),
            "the deadline stays on the original cadence"
        );
    }

    #[test]
    fn overrunning_frame_resyncs_instead_of_catching_up() {
        let now = Instant::now();
        let missed_deadline = now - Duration::from_millis(200);
        let (next, sleep) = schedule_frame(now, Some(missed_deadline), interval());

        assert_eq!(sleep, Duration::ZERO, "already late, nothing to sleep for");
        assert_eq!(
            next,
            now + interval(),
            "a late frame resyncs to now; it must not queue a burst"
        );
    }

    #[test]
    fn uncapped_config_asks_for_neither_interval_nor_clamp() {
        let performance = crate::config::PerformanceConfig::default();

        assert_eq!(performance.frame_interval(), None);
        assert_eq!(performance.frame_delta_cap(), None);
    }

    #[test]
    fn limits_are_clamped_to_what_the_loop_can_deliver() {
        let slow = crate::config::PerformanceConfig {
            max_fps: 0.1,
            max_frame_delta: 900.0,
        };
        let fast = crate::config::PerformanceConfig {
            max_fps: 100_000.0,
            max_frame_delta: 0.000_1,
        };

        assert_eq!(slow.frame_interval(), Some(Duration::from_secs(1)));
        assert_eq!(slow.frame_delta_cap(), Some(Duration::from_secs(5)));
        assert_eq!(fast.frame_interval(), Some(Duration::from_millis(1)));
        assert_eq!(fast.frame_delta_cap(), Some(Duration::from_millis(1)));
    }

    #[test]
    fn non_finite_limits_are_treated_as_unset() {
        let performance = crate::config::PerformanceConfig {
            max_fps: f32::NAN,
            max_frame_delta: f32::INFINITY,
        };

        assert_eq!(performance.frame_interval(), None);
        assert_eq!(performance.frame_delta_cap(), None);
    }

    #[test]
    fn render_budget_is_absent_until_configured() {
        assert!(cascade_budget(&RenderConfig::default()).is_none());
        assert_eq!(view_distance(&RenderConfig::default()), None);
    }

    #[test]
    fn shadow_budget_scales_the_cascades_with_the_distance() {
        let render = RenderConfig {
            shadow_max_distance: 150.0,
            shadow_cascades: 4,
            ..RenderConfig::default()
        };
        let budget = cascade_budget(&render).expect("configured shadows");

        assert_eq!(budget.bounds.len(), 4);
        assert_eq!(budget.minimum_distance, 0.1);
        assert!(
            (*budget.bounds.last().unwrap() - 150.0).abs() < 0.001,
            "the last cascade ends at the configured distance, got {:?}",
            budget.bounds
        );
    }

    #[test]
    fn shadow_budget_survives_hostile_values() {
        let tiny = RenderConfig {
            shadow_max_distance: 0.01,
            shadow_cascades: 99,
            ..RenderConfig::default()
        };
        let budget = cascade_budget(&tiny).expect("configured shadows");

        assert_eq!(budget.bounds.len(), MAX_SHADOW_CASCADES as usize);
        assert!(
            (*budget.bounds.last().unwrap() - MIN_SHADOW_DISTANCE).abs() < 0.001,
            "a sub-meter budget is clamped up, got {:?}",
            budget.bounds
        );

        let one_cascade = RenderConfig {
            shadow_cascades: 1,
            ..RenderConfig::default()
        };
        let budget = cascade_budget(&one_cascade).expect("cascades configured alone");
        assert_eq!(budget.bounds.len(), 1);
        assert_eq!(
            budget.bounds[0], 150.0,
            "distance falls back to the default"
        );
    }

    #[test]
    fn view_distance_is_clamped_and_zero_means_untouched() {
        assert_eq!(
            view_distance(&RenderConfig {
                view_distance: 250.0,
                ..RenderConfig::default()
            }),
            Some(250.0)
        );
        assert_eq!(
            view_distance(&RenderConfig {
                view_distance: 0.01,
                ..RenderConfig::default()
            }),
            Some(MIN_VIEW_DISTANCE)
        );
        assert_eq!(
            view_distance(&RenderConfig {
                view_distance: f32::NAN,
                ..RenderConfig::default()
            }),
            None
        );
    }

    fn headless_app(config: SimulationConfig) -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .insert_resource(config)
            .add_plugins(PerformancePlugin);
        app
    }

    fn camera(app: &mut App, marker: impl Bundle) -> Entity {
        app.world_mut()
            .spawn((
                Camera3d::default(),
                Projection::Perspective(PerspectiveProjection {
                    far: 10_000.0,
                    ..default()
                }),
                marker,
            ))
            .id()
    }

    fn far_of(app: &App, entity: Entity) -> f32 {
        match app.world().entity(entity).get::<Projection>() {
            Some(Projection::Perspective(perspective)) => perspective.far,
            _ => panic!("the test cameras are perspective"),
        }
    }

    fn cascade_count(app: &App, entity: Entity) -> usize {
        app.world()
            .entity(entity)
            .get::<CascadeShadowConfig>()
            .expect("the cascade budget is applied on the first frame")
            .bounds
            .len()
    }

    #[test]
    fn config_reaches_the_live_cameras_and_hot_reloads() {
        let mut config = SimulationConfig::default();
        config.performance.max_frame_delta = 0.05;
        config.render.view_distance = 250.0;
        config.render.shadow_max_distance = 80.0;
        config.render.shadow_cascades = 2;

        let mut app = headless_app(config);
        let main = camera(
            &mut app,
            MainCamera {
                follow_offset: Vec3::ZERO,
            },
        );
        let inset = camera(&mut app, PictureInPicture::new());
        let depth = camera(&mut app, DepthCaptureCamera);

        app.update();

        let cap = app.world().resource::<Time<Virtual>>().max_delta();
        assert!(
            (cap.as_secs_f32() - 0.05).abs() < 1e-6,
            "max_frame_delta clamps virtual time (which also bounds fixed steps), got {cap:?}"
        );
        assert_eq!(far_of(&app, main), 250.0);
        assert_eq!(
            far_of(&app, inset),
            10_000.0,
            "the inset keeps camera.pip_far; only the shadow budget is shared"
        );
        assert_eq!(
            far_of(&app, depth),
            10_000.0,
            "the livox depth view keeps capture.depth.far"
        );
        assert_eq!(cascade_count(&app, main), 2);
        assert_eq!(cascade_count(&app, inset), 2);
        assert_eq!(
            cascade_count(&app, depth),
            2,
            "every 3D view shares the shadow budget"
        );

        // Hot reload: a changed config must reach the entities that already exist,
        // including turning the limits back off.
        let engine_default = app.world().resource::<EngineFrameDelta>().0;
        {
            let mut config = app.world_mut().resource_mut::<SimulationConfig>();
            config.render.view_distance = 120.0;
            config.render.shadow_cascades = 1;
            config.performance.max_frame_delta = 0.0;
        }

        app.update();

        assert_eq!(far_of(&app, main), 120.0);
        assert_eq!(cascade_count(&app, main), 1);
        assert_eq!(
            app.world().resource::<Time<Virtual>>().max_delta(),
            engine_default,
            "max_frame_delta = 0 restores the engine default instead of freezing the cap"
        );
    }
}
