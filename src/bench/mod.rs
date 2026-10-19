//! Benchmark v2 support (`--features bench`).
//!
//! The benchmark needs three things the interactive simulator never did:
//!
//! 1. **Scripted, reproducible motion for more than one robot.** The v1 scenes
//!    drive at most two vehicles, both off the keyboard, and the ego
//!    (`Controlled`) and the slapper (`ActiveSlapper`) were addressed by *query
//!    singletons*. A moving-target benchmark needs "which robot does this input
//!    belong to", which is a [`crate::components::RobotId`], not a marker.
//! 2. **Ground truth as a topic.** `/tf` already carries every armor plate, but
//!    it carries no ownership, no robot state and no fixed-step scenario clock.
//! 3. **Per-shot events.** Hits are already detected
//!    (`crate::robomaster::armor::collision`), they are just not attributable.
//!
//! Everything here is behind `#[cfg(feature = "bench")]`, and the systems are
//! additionally gated at runtime by `[bench] enabled` in `config.toml`, so a
//! `bench` build with the shipped config behaves like a normal run.
//!
//! Module layout:
//! - [`motion`] — scenario primitives to `(movement, chassis_yaw, boost)`
//! - [`scenario`] — scenario file, fixed-step clock, `/sim/scenario/status`
//! - [`servo`] — bench-only velocity loop, so a scripted `v` is a real speed
//! - [`gt`] — `/sim/gt/robots` and `/sim/gt/armors`
//! - [`events`] — `/sim/events/fire` and `/sim/events/hit`
//!
//! Deliberately *not* here yet (P1): visibility ground truth, scenario
//! services, `/sim/robot_cmd`, fault injection, the other eleven primitives.

use bevy::prelude::*;

use crate::config::SimulationConfig;
use crate::ros2::plugin::capture_rune;

pub mod events;
pub mod gt;
pub mod motion;
pub mod scenario;
pub mod services;
pub mod servo;

/// `[bench] enabled`, read every frame so a hot-reloaded `config.toml` can still
/// turn the benchmark off (or on) between runs.
fn bench_enabled(config: Res<SimulationConfig>) -> bool {
    config.bench.enabled
}

pub struct BenchPlugin;

impl Plugin for BenchPlugin {
    fn build(&self, app: &mut App) {
        let config = app
            .world()
            .get_resource::<SimulationConfig>()
            .cloned()
            .unwrap_or_default();

        // Collision diagnostics are opt-in: `DAEDALUS_HIT_DEBUG=1` adds an observer
        // that logs every collision *start* of a live projectile, which is what tells
        // a real armour impact apart from an environment contact that resolves the
        // shot on the same step. With the variable unset nothing is registered.
        if std::env::var_os("DAEDALUS_HIT_DEBUG").is_some() {
            app.add_observer(events::debug_shot_start);
        }

        // Armour contact tracking is *not* optional: `handle_shot_end` needs it to
        // keep avian's speculative environment contact from swallowing a real
        // plate hit (see `events::ArmorContactSet`).
        app.add_observer(events::track_armor_contact_start)
            .add_observer(events::track_armor_contact_end);

        app.init_resource::<gt::ArmorIndex>()
            .init_resource::<gt::ChassisYawTracker>()
            .init_resource::<servo::SpeedServoStates>()
            .insert_resource(gt::GtRobotsLimiter::new(config.bench.gt_robots_period()))
            .init_resource::<events::ShotSequence>()
            .init_resource::<events::ResolvedShots>()
            .init_resource::<events::ArmorContactSet>()
            // Keeps a plate contact actionable for a couple of steps after it
            // ends, so the reverse sweep is not gated off on a contact that
            // starts and ends inside one physics step (task-76).
            .init_resource::<events::RecentArmorContact>()
            // Pristine scene poses, so a runtime `reset` can put a robot that has
            // no `spawn` back where the scene placed it.
            .init_resource::<scenario::SceneBaselines>()
            // Read the scenario file once `[bench] enabled` is on.
            .add_systems(Update, scenario::load_scenario.run_if(bench_enabled))
            // `DAEDALUS_SPAWN_DEBUG=1` only: per-frame `Position`/`Transform` of
            // every robot root, for chasing spawn teleports that do not stick.
            .add_systems(
                Update,
                scenario::debug_spawn_tracking.run_if(bench_enabled),
            )
            // Runtime scenario services (P1-B): the ROS threads only queue, the
            // state change happens here, before the system that owns the physical
            // reset.
            .add_systems(
                Update,
                services::poll_scenario_services
                    .before(scenario::scenario_control)
                    .run_if(bench_enabled),
            )
            // The baseline read must happen before any scenario teleport, which
            // is inside `scenario_control`.
            .add_systems(
                Update,
                scenario::capture_scene_baselines
                    .before(scenario::scenario_control)
                    .run_if(bench_enabled),
            )
            // Plate slots/counts must exist before the first GT frame.
            .add_systems(
                Update,
                gt::build_armor_index
                    .before(gt::publish_armor_gt)
                    .run_if(bench_enabled),
            )
            // The scenario writes the scripted robots' inputs between the
            // keyboard routing and the vehicle system, so both drivers end up in
            // the same table and the same physics path.
            .add_systems(
                Update,
                scenario::scenario_control
                    .after(crate::systems::route_robot_inputs)
                    .before(crate::systems::robot_vehicle_controls)
                    .run_if(bench_enabled),
            )
            // Occluder boxes (task-67): after the runtime command has cleared
            // `occluders_installed`, so a `restart` respawns in the same frame
            // the reset list despawned the old generation.
            .add_systems(
                Update,
                scenario::install_scenario_occluders
                    .after(scenario::scenario_control)
                    .run_if(bench_enabled),
            )
            // Close the speed loop on the input `scenario_control` just wrote,
            // still before the vehicle system applies it, so the servo and the
            // physics see each other with one frame of delay — the same loop a
            // real chassis closes internally.
            .add_systems(
                Update,
                servo::speed_servo_control
                    .after(scenario::scenario_control)
                    .before(crate::systems::robot_vehicle_controls)
                    .run_if(bench_enabled),
            )
            // Ground truth reuses `/tf`'s stamp, so it must run after the system
            // that samples the clock and builds the TF tree.
            .add_systems(
                Update,
                (
                    gt::publish_armor_gt.after(capture_rune),
                    gt::publish_robot_gt.after(capture_rune),
                    scenario::publish_scenario_status.after(capture_rune),
                )
                    .run_if(bench_enabled),
            )
            // Fire events: `projectile_launch` spawns in `PostUpdate`, so the
            // `Added<Projectile>` scan runs in `Last`, after it and after every
            // other spawner (darts/UAVs are filtered out by component).
            .add_systems(Last, events::publish_fire_events.run_if(bench_enabled))
            // True contact points: right after every physics step, sweep the
            // projectile sphere over the segment that step travelled and cache
            // the first plate intersection, so the `CollisionEnd` observer can
            // publish a contact point instead of a post-bounce position.
            .add_systems(
                FixedPostUpdate,
                events::record_shot_contacts.run_if(bench_enabled),
            );

        if config.bench.enabled {
            match config.bench.scenario_path() {
                Some(path) => info!(
                    "[bench] enabled: scenario={} gt_robots_hz={}",
                    path.display(),
                    config.bench.gt_robots_hz
                ),
                None => warn!("[bench] enabled but [bench] scenario_path is empty"),
            }
        } else {
            info!("[bench] disabled ([bench] enabled = false): benchmark systems idle");
        }
    }
}
