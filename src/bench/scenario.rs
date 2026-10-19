//! Scenario file, scenario clock and primitive scheduling.
//!
//! # What this owns
//!
//! - **The scenario clock.** `t_scenario` is accumulated from `Time<Fixed>`,
//!   not from wall time and not from the render loop, so a scenario replays on
//!   the same timeline whatever the frame rate does. `t_scenario = 0` is the
//!   trigger instant, not process start and not load time.
//! - **The trigger.** `on_load` starts as soon as every planned robot exists;
//!   `on_first_gimbal_cmd` waits for the first *solution* on `/rm_gimbal/cmd`,
//!   which is what makes the measurement window start when the auto-aim stack
//!   is actually alive (v1's t0).
//! - **The inputs.** Each frame, every planned robot's primitive is evaluated
//!   and written into [`RobotInputs`], which
//!   [`crate::systems::robot_vehicle_controls`] then applies through the normal
//!   vehicle dynamics. Nothing here touches a `Transform`, `Position` or a
//!   velocity — with one deliberate exception: the optional initial `spawn`
//!   teleport, which happens *once*, at load, before the scenario starts.
//! - **The deviation metric.** `max_track_deviation_m` is the **cross-track**
//!   error between where the robot actually is and where the primitive's ideal
//!   trajectory says it should be, compared at the same `t_scenario`. The
//!   along-track (lag) part is logged but deliberately *not* reported: it is the
//!   unavoidable consequence of `linear_acceleration` and `rotation_speed`, and
//!   a scenario is not misbehaving because a 4 m/s reversal takes a moment.
//!   Task-59 (`daedalus-bench:p1dev`) made the split plane *persistent*: through
//!   a `cross_los` coast and through the `static` tail after a leg it stays the
//!   last commanded direction, because the instantaneous command is zero there
//!   and the old `|delta|` fallback turned the honest ramp/coast lag into a
//!   metre-scale cross-track spike (2.0858 m on a healthy `ego_move_rung` leg,
//!   which then failed `scenario_deviation_m <= 0.5`). A robot that is commanded
//!   to move and does not is caught by the stall watchdog instead, which folds
//!   the positional error back into the same gate.
//!
//! # Failure policy
//!
//! A scenario that cannot be honoured is a failed run, so it exits the process
//! with a non-zero code (`error!` + `AppExit::error()`) instead of degrading
//! quietly. That applies to unknown primitives, missing primitive parameters,
//! malformed segments, and scripting the ego (P1's `ego_move`).

use avian3d::prelude::{
    AngularVelocity, Collider, ColliderDisabled, LinearVelocity, Position, RigidBody,
    RigidBodyDisabled, Rotation,
};
use bevy::prelude::*;
use r2r::rm_interfaces::msg::SimScenarioStatus;
use r2r::std_msgs::msg::Header;
use serde::Deserialize;
use std::collections::HashMap;
use std::path::Path;

use crate::bench::motion::{self, MotionFrame, MotionLimits, Primitive};
use crate::components::{
    ActiveSlapper, GameLayer, Infantry, InfantryChassis, MotionSource, RobotId, ScenarioAbsent,
};
use crate::config::SimulationConfig;
use crate::ros2::plugin::FirstGimbalCmd;
use crate::ros2::topic::TopicPublisher;
use crate::ros2::topic::bench_topics::SimScenarioStatusTopic;
use crate::systems::{ControllerInput, RobotInputs};

/// How far above the requested `z` a spawned robot is dropped from.
///
/// `scene.rs` spawns every robot 1 m above its ground position and lets the
/// contact solver settle it; reusing that number here means a scenario spawn
/// settles the same way the default scene does instead of clipping into the
/// ground or teleporting on top of it.
const SPAWN_DROP_HEIGHT_M: f32 = 1.0;

/// How long a requested spawn may stay pending before it is reported as failed.
///
/// The scene loads in well under a second, so this only fires when something is
/// genuinely wrong (robot entity missing, chassis node missing). The robot keeps
/// retrying afterwards, and the status error clears if it eventually lands.
const SPAWN_TIMEOUT_SECS: f64 = 5.0;

/// Update frames skipped by the trajectory metric right after a spawn teleport
/// (one propagation pass is not always enough under a heavy frame).
const SPAWN_DEVIATION_SKIP_FRAMES: u8 = 2;

/// Commanded speed above which a scripted robot is expected to actually move.
const STALL_MIN_COMMAND_MPS: f32 = 0.5;

/// A robot counts as stalled when it achieves less than this fraction of the
/// commanded speed.
const STALL_MIN_ACHIEVED_FRACTION: f32 = 0.25;

/// How long the command and the motion may disagree before it is an error.
///
/// Covers the honest ramp-up: the servo reaches 25% of the command within
/// ~0.5 s from rest even in the slowest (un-boosted) regime, so one second
/// cannot fire on a healthy leg.
const STALL_GRACE_SECS: f32 = 1.0;

/// `/sim/scenario/status` rate (`IMPLEMENTATION_PLAN.md` §3.3).
const STATUS_HZ: f64 = 10.0;

/// The frame `/tf`, `/sim/gt/*` and the status message all publish in.
const MAP_FRAME: &str = "map";

/// How the scenario clock starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Trigger {
    /// As soon as every planned robot exists in the scene.
    OnLoad,
    /// On the first solver solution received on `/rm_gimbal/cmd`.
    OnFirstGimbalCmd,
}

impl Trigger {
    fn parse(name: &str) -> Result<Self, String> {
        match name {
            "on_load" => Ok(Self::OnLoad),
            "on_first_gimbal_cmd" => Ok(Self::OnFirstGimbalCmd),
            other => Err(format!(
                "unknown trigger '{other}' (P0 supports on_load, on_first_gimbal_cmd)"
            )),
        }
    }

    fn name(self) -> &'static str {
        match self {
            Self::OnLoad => "on_load",
            Self::OnFirstGimbalCmd => "on_first_gimbal_cmd",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Idle,
    Running,
    Done,
}

impl Phase {
    pub(crate) fn wire(self) -> u8 {
        match self {
            Self::Idle => 0,
            Self::Running => 1,
            Self::Done => 2,
        }
    }
}

/// Runtime command from `/sim/scenario/*` (P1-B).
///
/// The service listener only stores this; [`scenario_control`] applies it, so the
/// world is only ever mutated from inside the Bevy schedule.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ScenarioCommand {
    #[default]
    None,
    /// A new scenario was loaded: reset the vehicles to their initial pose,
    /// drop the scripted drivers, keep the clock idle until an explicit `start`.
    Load,
    /// Idempotent reset of the current scenario: same physical reset, clock idle.
    Reset,
    /// Full restart: same physical reset, then `Running` at `t_scenario = 0`.
    Restart,
}

impl ScenarioCommand {
    fn label(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Load => "load",
            Self::Reset => "reset",
            Self::Restart => "restart",
        }
    }
}

/// `spawn`, in the **map (ROS) frame** — the frame `/tf`, `/sim/gt/*` and the
/// auto-aim stack share. `x` and `y` are the horizontal axes (`x` east, `y`
/// north) and `z` is height above the ground; `z = 0` therefore means ground
/// level, and `z` is the only height entry. `yaw_deg` is a ROS heading about +Z
/// (0 = facing +X).
#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
pub struct SpawnSpec {
    pub x: f32,
    pub y: f32,
    #[serde(default)]
    pub z: f32,
    #[serde(default)]
    pub yaw_deg: f32,
}

impl SpawnSpec {
    /// Bevy world position and yaw for this spawn.
    ///
    /// The `/tf` remap is `ros = (-z, -x, y)`, so its inverse is
    /// `bevy = (-ros_y, ros_z, -ros_x)`; a ROS heading of θ is also the Bevy
    /// yaw θ because the same remap maps Bevy `Y` to ROS `Z`.
    pub fn to_bevy(self) -> (Vec3, f32) {
        (
            Vec3::new(-self.y, self.z + SPAWN_DROP_HEIGHT_M, -self.x),
            self.yaw_deg.to_radians(),
        )
    }
}

/// A plain `x`/`y`/`z` triple in the **map (ROS) frame**, used by the occluder
/// box tables where the axes mean different things (`pos` = the box **bottom**
/// centre, `size` = full extents).
#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
pub struct Vec3Spec {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// A scene-side static occluder (task-67), in the **map (ROS) frame**:
/// `pos` is the **bottom** centre of the box (`pos.z` = the lowest face, so a
/// box standing on the plateau is `pos.z = 0.0543`), `size` the full extents
/// (`x` along, `y` wide, `z` tall) and `yaw_deg` a ROS heading about +Z.
///
/// `visible` selects which axis the occluder probes and is the whole point of
/// the two modes:
///
/// * `visible = false` (default) — **collider only**: [`RigidBody::Static`] +
///   [`Collider::cuboid`] + [`GameLayer::environment_collision_layers`], and no
///   `Mesh3d`/material at all, so nothing is drawn. The projectile is stopped
///   while the plates stay **fully visible** and the lock is untouched: the
///   "blind fire" case, which exercises the `hit` axis and has **no** vertical
///   window constraint.
/// * `visible = true` — an extra visible box (a built-in [`Cuboid`] mesh) is
///   spawned on top of the collider, so it blocks the bore **and** the sight
///   line: the `det`/visibility case, where the box top must sit inside the
///   `z_o` window of `NOTES.md §32`.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
pub struct OccluderSpec {
    /// Logging/attribution handle only; when omitted the table index is used.
    #[serde(default)]
    pub id: Option<u32>,
    pub pos: Vec3Spec,
    pub size: Vec3Spec,
    #[serde(default)]
    pub yaw_deg: f32,
    #[serde(default)]
    pub visible: bool,
}

/// Largest allowed occluder extent, to catch unit mistakes (m).
const MAX_OCCLUDER_EXTENT_M: f32 = 50.0;

impl OccluderSpec {
    /// Bevy centre, rotation and full Bevy-frame extents for this box.
    ///
    /// The box maps with the same `/tf` remap as everything else
    /// (`bevy = (-ros_y, ros_z, -ros_x)`), so the Bevy axes are
    /// `x = ros_y`, `y = ros_z`, `z = ros_x`; a ROS heading θ is also the Bevy
    /// yaw θ. `pos.z` is the **bottom**, hence the extra half-extent on `y`.
    pub fn to_bevy(&self) -> (Vec3, Quat, Vec3) {
        let extent = Vec3::new(self.size.y, self.size.z, self.size.x);
        let centre = Vec3::new(
            -self.pos.y,
            self.pos.z + self.size.z * 0.5,
            -self.pos.x,
        );
        (centre, Quat::from_rotation_y(self.yaw_deg.to_radians()), extent)
    }

    /// Fail-closed validation: a bad entry is a load error, never a silent skip.
    fn validate(&self) -> Result<(), String> {
        for (axis, value) in [("x", self.size.x), ("y", self.size.y), ("z", self.size.z)] {
            if !value.is_finite() || value <= 0.0 {
                return Err(format!(
                    "occluder {}: size.{axis} must be a positive length, got {value}",
                    self.id.map_or_else(|| "<auto>".to_string(), |id| id.to_string())
                ));
            }
            if value > MAX_OCCLUDER_EXTENT_M {
                return Err(format!(
                    "occluder {}: size.{axis} = {value} m exceeds the {MAX_OCCLUDER_EXTENT_M} m sanity cap",
                    self.id.map_or_else(|| "<auto>".to_string(), |id| id.to_string())
                ));
            }
        }
        for (axis, value) in [("x", self.pos.x), ("y", self.pos.y), ("z", self.pos.z)] {
            if !value.is_finite() {
                return Err(format!(
                    "occluder {}: pos.{axis} must be a finite map coordinate, got {value}",
                    self.id.map_or_else(|| "<auto>".to_string(), |id| id.to_string())
                ));
            }
        }
        if !self.yaw_deg.is_finite() {
            return Err(format!(
                "occluder {}: yaw_deg must be finite, got {}",
                self.id.map_or_else(|| "<auto>".to_string(), |id| id.to_string()),
                self.yaw_deg
            ));
        }
        Ok(())
    }

    /// The id this entry is addressed by: the declared one, else the table index.
    pub fn resolved_id(&self, index: usize) -> u32 {
        self.id.unwrap_or(index as u32)
    }
}

#[derive(Debug, Deserialize)]
struct ScenarioFile {
    id: String,
    #[serde(default)]
    seed: u32,
    duration: f32,
    #[serde(default = "default_trigger_name")]
    trigger: String,
    #[serde(default)]
    robot: Vec<RobotSpec>,
    #[serde(default)]
    occluder: Vec<OccluderSpec>,
}

fn default_trigger_name() -> String {
    "on_first_gimbal_cmd".to_string()
}

/// The robots `scene.rs` creates. A scenario that does not list one of them
/// declares it **absent** (task-44), rather than leaving it parked at its scene
/// pose where the sensor can still see and lock it.
pub const SCENE_ROBOT_IDS: [u8; 2] = [1, 2];

fn default_present() -> bool {
    true
}

#[derive(Debug, Deserialize)]
struct RobotSpec {
    id: u8,
    /// `false` means "this robot is not in the world for this scenario": the
    /// vehicle is hidden, its colliders and rigid body are disabled and it is
    /// dropped from GT, `/tf` and input routing. Omitted ⇒ `true`; not listing
    /// the robot at all is equivalent to `present = false`.
    #[serde(default = "default_present")]
    present: bool,
    /// Opt-in to the legacy behaviour of a listed robot **without** a `spawn`:
    /// keep whatever pose `scene.rs` gave it. Without this flag such a robot is
    /// reported (`warn!`) instead of silently keeping the scene pose.
    #[serde(default)]
    keep_default: bool,
    #[serde(default)]
    spawn: Option<SpawnSpec>,
    #[serde(default)]
    segment: Vec<SegmentSpec>,
}

#[derive(Debug, Deserialize)]
struct SegmentSpec {
    t0: f32,
    t1: f32,
    #[serde(rename = "type")]
    kind: String,
    #[serde(default)]
    v: Option<f32>,
    #[serde(default)]
    dir_deg: Option<f32>,
    #[serde(default)]
    omega: Option<f32>,
    #[serde(default)]
    freq_hz: Option<f32>,
    /// `sine` only: a linear frequency sweep `freq_start_hz → freq_end_hz` over
    /// the leg (a chirp). `freq_hz` above is then the constant-frequency
    /// shorthand (both ends equal).
    #[serde(default)]
    freq_start_hz: Option<f32>,
    #[serde(default)]
    freq_end_hz: Option<f32>,
    #[serde(default)]
    amp_m: Option<f32>,
    /// `spin` sweep only (P2 chirp): the chassis yaw oscillates as
    /// `yaw_amp_deg * sin(phi(t))`, so the angular rate is
    /// `yaw_amp_deg * 2*pi*f(t) * cos(phi(t))` with the same frequency-pair
    /// spelling as `sine` (`freq_hz`, or `freq_start_hz`/`freq_end_hz`). When
    /// present, `omega` must be absent — the sweep defines the rate — and the
    /// amplitude is capped so the rate peak fits `rotation_speed`.
    #[serde(default)]
    yaw_amp_deg: Option<f32>,
    /// `cross_los` only: keep sliding for the whole leg (`true`) or stop at the
    /// leg's midpoint, i.e. at the line-of-sight crossing (`false`, the
    /// default — a crossing target that keeps going leaves the sensor behind).
    #[serde(default)]
    pass_through: Option<bool>,
}

impl SegmentSpec {
    fn required(&self, key: &str, value: Option<f32>) -> Result<f32, String> {
        value.ok_or_else(|| {
            format!(
                "segment [{:.3}, {:.3}] of type '{}' needs '{key}'",
                self.t0, self.t1, self.kind
            )
        })
    }

    /// `sine` and the `spin` sweep share the frequency-pair spelling: two ends,
    /// a single end (constant), or the `freq_hz` shorthand. `kind` only names the
    /// primitive in the error message.
    fn sweep_freqs(&self, kind: &str) -> Result<(f32, f32), String> {
        match (self.freq_start_hz, self.freq_end_hz, self.freq_hz) {
            (Some(f0), Some(f1), _) => Ok((f0, f1)),
            (Some(f0), None, _) => Ok((f0, f0)),
            (None, Some(f1), _) => Ok((f1, f1)),
            (None, None, Some(f)) => Ok((f, f)),
            (None, None, None) => Err(format!(
                "segment [{:.3}, {:.3}] of type '{kind}' needs 'freq_hz' \
                 (constant) or 'freq_start_hz' + 'freq_end_hz' (sweep)",
                self.t0, self.t1
            )),
        }
    }

    fn to_primitive(&self) -> Result<Primitive, String> {
        match self.kind.as_str() {
            "static" => Ok(Primitive::Static),
            "line" => Ok(Primitive::Line {
                v: self.required("v", self.v)?,
                dir_deg: self.required("dir_deg", self.dir_deg)?,
            }),
            "strafe" => Ok(Primitive::Strafe {
                v: self.required("v", self.v)?,
                dir_deg: self.required("dir_deg", self.dir_deg)?,
            }),
            // A near-perpendicular crossing of the line of sight. Same slide as
            // `strafe` (the chassis heading is untouched, so the armour ring
            // does not swing with the travel direction), with `pass_through`
            // deciding whether the target keeps sliding after the crossing.
            "cross_los" => Ok(Primitive::CrossLos {
                v: self.required("v", self.v)?,
                dir_deg: self.required("dir_deg", self.dir_deg)?,
                pass_through: self.pass_through.unwrap_or(false),
                duration_s: (self.t1 - self.t0).max(0.0),
            }),
            "spin" => match self.yaw_amp_deg {
                // The P2 chirp: `type = "spin"` plus `yaw_amp_deg` and a
                // frequency pair. Refusing `omega` alongside is deliberate — a
                // silently ignored key is exactly the class of bug this benchmark
                // keeps tripping over.
                Some(yaw_amp_deg) => {
                    if self.omega.is_some() {
                        return Err(format!(
                            "segment [{:.3}, {:.3}] of type 'spin' has both 'omega' and \
                             'yaw_amp_deg': a sweep defines its own rate, so remove 'omega'",
                            self.t0, self.t1
                        ));
                    }
                    let (start, end) = self.sweep_freqs("spin")?;
                    Ok(Primitive::SpinSweep {
                        yaw_amp_deg,
                        freq_start_hz: start,
                        freq_end_hz: end,
                        duration_s: (self.t1 - self.t0).max(0.0),
                    })
                }
                None => Ok(Primitive::Spin {
                    omega: self.required("omega", self.omega)?,
                }),
            },
            "weave" => Ok(Primitive::Weave {
                v: self.required("v", self.v)?,
                dir_deg: self.required("dir_deg", self.dir_deg)?,
                freq_hz: self.required("freq_hz", self.freq_hz)?,
                amp_m: self.required("amp_m", self.amp_m)?,
            }),
            // P2 chirp: sinusoidal lateral sweep with a linear frequency sweep.
            // `freq_hz` is the constant-frequency shorthand; otherwise both ends
            // must be given (a missing end defaults to the other one).
            "sine" => {
                let (start, end) = self.sweep_freqs("sine")?;
                Ok(Primitive::Sine {
                    v: self.required("v", self.v)?,
                    dir_deg: self.required("dir_deg", self.dir_deg)?,
                    amp_m: self.required("amp_m", self.amp_m)?,
                    freq_start_hz: start,
                    freq_end_hz: end,
                    duration_s: (self.t1 - self.t0).max(0.0),
                })
            }
            other => Err(format!(
                "unsupported primitive '{other}' at [{:.3}, {:.3}]; P0/P1 supports \
                 static, line, strafe, cross_los, spin, weave, sine",
                self.t0, self.t1
            )),
        }
    }
}

/// A primitive pinned to a `t_scenario` window.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimedSegment {
    pub t0: f32,
    pub t1: f32,
    pub primitive: Primitive,
}

impl TimedSegment {
    fn contains(&self, t: f32) -> bool {
        t >= self.t0 && t < self.t1
    }
}

/// One scripted robot: where it starts, what it does, and how far it drifted.
pub struct PlannedRobot {
    pub id: RobotId,
    pub spawn: Option<SpawnSpec>,
    pub segments: Vec<TimedSegment>,
    /// MotionSource installed / spawn applied. Cleared until the robot exists.
    pub installed: bool,
    /// Actual ground-plane position at `t_scenario = 0`, the anchor the nominal
    /// trajectory is integrated from. `None` until the trigger fires.
    pub origin: Option<Vec3>,
    pub max_cross_track_m: f32,
    pub max_along_track_m: f32,
    /// Segment timeline end, for a sanity check against `duration`.
    pub last_segment_end: f32,
    /// `d(world heading) / d(chassis local yaw)` for this model: `+1` for
    /// `vehicle.glb`, `-1` for `HERO.glb` (its scene root mirrors the body).
    /// Resolved once the chassis node exists; see [`motion::local_chassis_yaw`].
    pub yaw_sign: f32,
    /// Frames to skip before the deviation metric trusts `GlobalTransform`, which
    /// still holds the pre-spawn pose on the teleport frame.
    pub deviation_skip_frames: u8,
    /// Last non-zero commanded direction, so a `static` tail after a moving leg
    /// keeps splitting its lag into along-track instead of cross-track.
    pub last_reference_velocity: Vec3,
    /// Scenario time at which the command and the motion started disagreeing.
    pub stall_since: Option<f32>,
    /// The stall error was logged once.
    pub stall_logged: bool,
}

impl PlannedRobot {
    /// The primitive active at `t_scenario`, or `Static` outside every window.
    pub fn primitive_at(&self, t: f32) -> Primitive {
        self.segments
            .iter()
            .find(|segment| segment.contains(t))
            .map(|segment| segment.primitive)
            .unwrap_or(Primitive::Static)
    }

    /// Segment-relative time, so a `weave`'s sine starts at its own `t0`.
    fn local_time(&self, t: f32) -> f32 {
        self.segments
            .iter()
            .find(|segment| segment.contains(t))
            .map(|segment| t - segment.t0)
            .unwrap_or(0.0)
    }

    /// Where the primitive's ideal (limit-free) trajectory says the robot is.
    pub fn nominal_position(&self, t: f32, origin: Vec3) -> Vec3 {
        let mut position = origin;
        for segment in &self.segments {
            let from = segment.t0;
            let to = segment.t1.min(t);
            if to > from {
                position += segment.primitive.nominal_displacement(
                    (from - segment.t0).max(0.0),
                    to - segment.t0,
                );
            }
            if t <= segment.t1 {
                break;
            }
        }
        position
    }

    /// Ideal velocity of the active primitive, in the map frame.
    pub fn nominal_velocity(&self, t: f32) -> Vec3 {
        self.primitive_at(t).nominal_velocity(self.local_time(t))
    }

    /// The active primitive's nominal velocity at its own segment start.
    ///
    /// Used only when the instantaneous nominal velocity is zero, to keep the
    /// cross/along split meaningful: a `cross_los` coast still has a direction
    /// (the leg's), while `static`/`spin` return zero here too, which is exactly
    /// the old `|delta|` fallback. For `line`/`strafe`/`weave` the instantaneous
    /// value is non-zero anyway, so the reference is never consulted.
    pub fn reference_nominal_velocity(&self, t: f32) -> Vec3 {
        self.primitive_at(t).nominal_velocity(0.0)
    }
}

/// The scenario currently loaded, if any.
#[derive(Resource)]
pub struct Scenario {
    pub id: String,
    pub seed: u32,
    pub duration: f32,
    pub trigger: Trigger,
    pub robots: Vec<PlannedRobot>,
    /// Scene robots this scenario declares absent (task-44): not listed, or
    /// listed with `present = false`. They are hidden, physically disabled and
    /// filtered out of GT/`/tf`/input while the scenario is loaded.
    pub absent: Vec<RobotId>,
    /// Static occluder boxes this scenario asks for (task-67). Empty for every
    /// shipped scenario, in which case no entity is created at all.
    pub occluders: Vec<OccluderSpec>,
    /// Whether the occluder entities for the *current* `occluders` list exist.
    /// Cleared by [`Scenario::apply_command`] so a restart respawns them; a
    /// runtime `load` carries a freshly parsed scenario with the flag down.
    occluders_installed: bool,
    pub phase: Phase,
    pub t_scenario: f32,
    pub max_track_deviation_m: f32,
    /// The same measurement decomposed **along** the commanded direction: start-up
    /// lag and post-leg coast are expected physics, so this component never gates
    /// validity — it is published so "the robot moved slower than commanded" stays
    /// visible next to the cross-track number that does gate.
    pub max_along_track_m: f32,
    /// Why a requested spawn is not applied (empty/`None` when everything is
    /// installed). Published on `/sim/scenario/status` and logged as `ERROR`, so
    /// a scenario that silently kept the scene pose can no longer look healthy.
    pub spawn_error: Option<String>,
    origin_fixed_secs: Option<f64>,
    status_last_secs: f64,
    /// Fixed-clock deadline for the pending spawns, `None` when nothing waits.
    spawn_deadline_secs: Option<f64>,
    /// Guards the `ERROR` line so a stuck spawn reports once, not every frame.
    spawn_error_logged: bool,
    /// Runtime command queued by `/sim/scenario/*`, applied at the top of
    /// [`scenario_control`] (never from the r2r listener thread).
    pub pending: ScenarioCommand,
    /// Set by a runtime `load`/`reset`: the TOML `trigger` must not start the
    /// clock on its own any more, because the caller owns the start decision.
    /// Cleared by an explicit `start`, and never set by the start-up bootstrap
    /// path — a run that never calls the services behaves exactly as before.
    auto_start_suppressed: bool,
}

impl Scenario {
    pub fn parse(text: &str, limits: &MotionLimits) -> Result<Self, String> {
        let file: ScenarioFile =
            toml::from_str(text).map_err(|error| format!("scenario TOML does not parse: {error}"))?;

        if !(file.duration.is_finite() && file.duration > 0.0) {
            return Err(format!("duration must be positive, got {}", file.duration));
        }
        let trigger = Trigger::parse(&file.trigger)?;

        let mut robots = Vec::with_capacity(file.robot.len());
        let mut seen = Vec::new();
        let mut present_ids: Vec<u8> = Vec::new();
        for spec in &file.robot {
            // P1 `ego_move`: the ego may be *scripted* — listed like any other
            // robot and driven through the bench primitives — and, because the
            // ego's default pose has less than 3 m of clear runway in every
            // direction, it may also carry a `spawn` to place it on a certified
            // corridor before the run starts. It always exists: it hosts the
            // camera, so `present = false` is an error rather than a silent hole.
            let is_ego = spec.id == 0;
            if is_ego && !spec.present {
                return Err(
                    "robot 0 is the ego: it cannot be absent (`present = false`); drop the entry \
                     instead to leave it hand-driven"
                        .to_string(),
                );
            }
            if seen.contains(&spec.id) {
                return Err(format!("robot id {} appears twice", spec.id));
            }
            seen.push(spec.id);

            if !spec.present {
                if spec.spawn.is_some() || !spec.segment.is_empty() {
                    return Err(format!(
                        "robot {}: `present = false` conflicts with its spawn/segments; drop the \
                         entry entirely to declare the robot absent",
                        spec.id
                    ));
                }
                info!(
                    "[bench] scenario '{}' declares robot {} absent (`present = false`)",
                    file.id, spec.id
                );
                continue;
            }
            present_ids.push(spec.id);

            // The ego has no spawn by construction (`parse` rejects one above), so
            // the "you forgot a spawn" warning would be noise for it.
            if spec.spawn.is_none() && !is_ego {
                if spec.keep_default {
                    info!(
                        "[bench] robot {}: no spawn; keeping its scene pose (`keep_default = true`)",
                        spec.id
                    );
                } else {
                    warn!(
                        "[bench] robot {}: no spawn and no `keep_default = true`, so it keeps the \
                         pose the scene loaded it with. Write `keep_default = true` to say that on \
                         purpose, or a `spawn` to place it.",
                        spec.id
                    );
                }
            }

            let mut segments = Vec::with_capacity(spec.segment.len());
            let mut previous_end = 0.0f32;
            for segment in &spec.segment {
                if !(segment.t0.is_finite() && segment.t1.is_finite()) || segment.t1 <= segment.t0 {
                    return Err(format!(
                        "robot {}: segment [{:.3}, {:.3}] must have t1 > t0",
                        spec.id, segment.t0, segment.t1
                    ));
                }
                if segment.t0 < previous_end - 1e-4 {
                    return Err(format!(
                        "robot {}: segment [{:.3}, {:.3}] overlaps the previous one (ends {:.3})",
                        spec.id, segment.t0, segment.t1, previous_end
                    ));
                }
                if segment.t0 > previous_end + 1e-3 && !segments.is_empty() {
                    warn!(
                        "[bench] robot {}: gap between {:.3}s and {:.3}s (zero input, not an error)",
                        spec.id, previous_end, segment.t0
                    );
                }
                let primitive = segment.to_primitive()?;
                let (primitive, clamped) = primitive.clamped(limits);
                if clamped {
                    warn!(
                        "[bench] robot {} segment [{:.3}, {:.3}] '{}' exceeded the vehicle limits \
                         (max_speed={:.2}, rotation_speed={:.2}) and was clamped",
                        spec.id,
                        segment.t0,
                        segment.t1,
                        primitive.kind(),
                        limits.max_speed,
                        limits.rotation_speed
                    );
                }
                segments.push(TimedSegment {
                    t0: segment.t0,
                    t1: segment.t1,
                    primitive,
                });
                previous_end = segment.t1;
            }
            let last_segment_end = segments.last().map(|segment| segment.t1).unwrap_or(0.0);
            if last_segment_end > file.duration + 1e-3 {
                warn!(
                    "[bench] robot {} drives until {:.3}s but the scenario ends at {:.3}s",
                    spec.id, last_segment_end, file.duration
                );
            }
            robots.push(PlannedRobot {
                id: RobotId(spec.id),
                spawn: spec.spawn,
                segments,
                installed: false,
                origin: None,
                max_cross_track_m: 0.0,
                max_along_track_m: 0.0,
                last_segment_end,
                yaw_sign: 1.0,
                deviation_skip_frames: 0,
                last_reference_velocity: Vec3::ZERO,
                stall_since: None,
                stall_logged: false,
            });
        }

        if robots.is_empty() {
            warn!("[bench] scenario '{}' plans no robots", file.id);
        }

        // Task-67: occluder boxes are optional, and an entry that cannot be
        // turned into a sane box is a load error — a silently dropped occluder
        // would look exactly like "the shot went through a wall".
        let mut seen_occluder_ids: std::collections::HashSet<u32> = std::collections::HashSet::new();
        for (index, spec) in file.occluder.iter().enumerate() {
            spec.validate()?;
            let id = spec.resolved_id(index);
            if !seen_occluder_ids.insert(id) {
                return Err(format!(
                    "occluder {id}: duplicate id; give every box a distinct `id` (or omit it and \
                     let the table index be used)"
                ));
            }
        }
        if !file.occluder.is_empty() {
            info!(
                "[bench] scenario '{}' declares {} occluder box(es): {} collider-only (blind fire), \
                 {} visible (blocks the sight line too)",
                file.id,
                file.occluder.len(),
                file.occluder.iter().filter(|spec| !spec.visible).count(),
                file.occluder.iter().filter(|spec| spec.visible).count(),
            );
        }

        // Task-44: not listing a scene robot means it does not exist for this
        // run. Silence here is exactly what let `smoke_far`'s second vehicle sit
        // at its scene pose, inside the camera's field of view, and steal the
        // lock from the intended target.
        let mut absent: Vec<RobotId> = SCENE_ROBOT_IDS
            .iter()
            .filter(|id| !present_ids.contains(id))
            .map(|id| RobotId(*id))
            .collect();
        absent.sort();
        if !absent.is_empty() {
            info!(
                "[bench] scenario '{}' leaves robot(s) {} out of the world: hidden, rigid body and \
                 colliders disabled, excluded from GT//tf/input",
                file.id,
                absent
                    .iter()
                    .map(|id| id.0.to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }

        Ok(Self {
            id: file.id,
            seed: file.seed,
            duration: file.duration,
            trigger,
            robots,
            absent,
            occluders: file.occluder,
            occluders_installed: false,
            phase: Phase::Idle,
            t_scenario: 0.0,
            max_track_deviation_m: 0.0,
            max_along_track_m: 0.0,
            spawn_error: None,
            origin_fixed_secs: None,
            status_last_secs: f64::NEG_INFINITY,
            spawn_deadline_secs: None,
            spawn_error_logged: false,
            pending: ScenarioCommand::None,
            auto_start_suppressed: false,
        })
    }

    pub fn load(path: &Path, limits: &MotionLimits) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|error| format!("cannot read scenario {}: {error}", path.display()))?;
        Self::parse(&text, limits).map_err(|error| format!("{}: {error}", path.display()))
    }

    /// P1-B `load`: parse + validate, without touching the running scenario.
    ///
    /// The caller installs the result, so a failure here leaves whatever is
    /// running untouched (a half-applied load would be the one unrecoverable
    /// state). `seed_override != 0` replaces the TOML seed.
    pub fn load_for_runtime(
        path: &Path,
        seed_override: i64,
        limits: &MotionLimits,
    ) -> Result<Self, String> {
        let mut scenario = Self::load(path, limits)?;
        if seed_override != 0 {
            scenario.seed = seed_override.clamp(0, u32::MAX as i64) as u32;
        }
        // The caller decides when the clock starts: a runtime load must not be
        // started by the TOML `trigger` behind its back.
        scenario.auto_start_suppressed = true;
        scenario.pending = ScenarioCommand::Load;
        Ok(scenario)
    }

    /// `installed` means the spawn really landed (pose *and* chassis heading), so
    /// this is the honest answer to "did my scenario geometry apply?" — used both
    /// by `/sim/scenario/status` and by the service replies. A scenario with no
    /// spawns trivially qualifies.
    pub fn spawn_applied(&self) -> bool {
        self.robots
            .iter()
            .all(|robot| robot.spawn.is_none() || robot.installed)
    }

    /// Bookkeeping half of a runtime `load`/`reset`/`restart`.
    ///
    /// The physical half (poses, velocities, chassis yaw, scripted drivers) lives
    /// in [`scenario_control`], which owns those queries; this only touches the
    /// scenario's own state, so the two halves cannot drift apart.
    pub fn apply_command(&mut self, command: ScenarioCommand, fixed_now: f64) {
        let restart = command == ScenarioCommand::Restart;
        self.origin_fixed_secs = if restart { Some(fixed_now) } else { None };
        self.t_scenario = 0.0;
        self.phase = if restart { Phase::Running } else { Phase::Idle };
        self.max_track_deviation_m = 0.0;
        self.max_along_track_m = 0.0;
        self.spawn_error = None;
        self.spawn_deadline_secs = None;
        self.spawn_error_logged = false;
        self.auto_start_suppressed = !restart;
        // Task-67: the occluder entities were despawned by the reset half
        // (`bench::services`, and the same list on `restart`), so this scenario
        // must spawn its own boxes again. A runtime `load` replaces the whole
        // resource and carries an already-cleared flag.
        self.occluders_installed = false;
        for robot in self.robots.iter_mut() {
            robot.origin = None;
            robot.max_cross_track_m = 0.0;
            robot.max_along_track_m = 0.0;
            // The poses written by this command are only visible to the deviation
            // metric after the next transform propagation, so skip the same
            // number of frames a fresh spawn skips.
            robot.deviation_skip_frames = SPAWN_DEVIATION_SKIP_FRAMES;
            robot.yaw_sign = 1.0;
            // The split plane and the stall watchdog describe the run that just
            // ended, so they must not leak into the next scenario.
            robot.last_reference_velocity = Vec3::ZERO;
            robot.stall_since = None;
            robot.stall_logged = false;
            // A restart re-installs (spawn + scripted driver); a plain reset has
            // already restored the pose itself and must not re-install, or the
            // Idle scenario would quietly keep driving its robots.
            robot.installed = !restart;
        }
        info!(
            "[bench] scenario '{}' {}: phase={:?}, t_scenario=0, \
             deviation cleared, drivers reinstalled={}",
            self.id,
            command.label(),
            self.phase,
            restart
        );
    }

    pub fn ready(&self) -> bool {
        !self.robots.is_empty() && self.robots.iter().all(|robot| robot.installed)
    }

    /// Scenario time in nanoseconds, as the frozen wire format asks for it.
    ///
    /// `uint64`: a `uint32` of nanoseconds caps at 4.29 s, which would wrap
    /// (and silently mis-pair fire/hit lead computations) four times inside a
    /// 20 s P0 scenario. See `IMPLEMENTATION_PLAN.md` §3.2.
    pub fn sim_time_ns(&self) -> u64 {
        ((self.t_scenario.max(0.0) as f64) * 1e9).max(0.0) as u64
    }

    fn advance_clock(&mut self, fixed_now: f64, first_cmd_seen: bool) {
        match self.phase {
            Phase::Idle => {
                let triggered = !self.auto_start_suppressed
                    && self.ready()
                    && match self.trigger {
                        Trigger::OnLoad => true,
                        Trigger::OnFirstGimbalCmd => first_cmd_seen,
                    };
                if triggered {
                    self.origin_fixed_secs = Some(fixed_now);
                    self.t_scenario = 0.0;
                    self.phase = Phase::Running;
                    info!(
                        "[bench] scenario '{}' started (trigger={}, fixed t={:.3}s)",
                        self.id,
                        self.trigger.name(),
                        fixed_now
                    );
                }
            }
            Phase::Running => {
                let origin = self.origin_fixed_secs.unwrap_or(fixed_now);
                self.t_scenario = ((fixed_now - origin) as f32).max(0.0);
                if self.t_scenario >= self.duration {
                    self.t_scenario = self.duration;
                    self.phase = Phase::Done;
                    info!(
                        "[bench] scenario '{}' done at t_scenario={:.3}s; \
                         holding last state (zero input, no reset)",
                        self.id, self.t_scenario
                    );
                }
            }
            // Hold: no reset, no rewind. The robots stop where they are.
            Phase::Done => self.t_scenario = self.duration,
        }
    }
}

/// Marker for a static occluder entity spawned for the current scenario
/// (task-67). `id` is the TOML `id`, or the table index when it was omitted,
/// and is what the `[bench][hitdbg]` entity name shows.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScenarioOccluder(pub u32);

/// Spawn the current scenario's occluder boxes, once per scenario generation.
///
/// Deliberately a separate system rather than part of [`scenario_control`]: it
/// is the only scenario-side code that needs the asset handles, and it must run
/// *after* the runtime command of the same frame, which is what clears
/// `occluders_installed`.
///
/// The despawn pass is a safety net, not the normal path: `bench::services`
/// despawns every [`ScenarioOccluder`] on `load`/`reset`/`restart`, so two
/// generations of boxes can never coexist even if a reset is missed.
pub fn install_scenario_occluders(
    mut commands: Commands,
    scenario: Option<ResMut<Scenario>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    existing: Query<Entity, With<ScenarioOccluder>>,
) {
    let Some(mut scenario) = scenario else {
        return;
    };
    if scenario.occluders_installed {
        return;
    }
    for entity in &existing {
        commands.entity(entity).despawn();
    }
    for (index, spec) in scenario.occluders.iter().enumerate() {
        let id = spec.resolved_id(index);
        let (centre, rotation, extent) = spec.to_bevy();
        let mut entity = commands.spawn((
            Name::new(format!("occluder_{id}")),
            ScenarioOccluder(id),
            Transform::from_translation(centre).with_rotation(rotation),
            RigidBody::Static,
            // Full extents, not half: avian's `Collider::cuboid` halves its
            // arguments internally (`SharedShape::cuboid(x*0.5, ..)`), so the
            // collider is exactly the declared `size` box.
            Collider::cuboid(extent.x, extent.y, extent.z),
            // The environment layer is the one projectiles already interact
            // with, so this box stops shots without touching a vehicle's own
            // collision response.
            GameLayer::environment_collision_layers(),
        ));
        if spec.visible {
            // `visible = true`: an opaque box is drawn as well, so it blocks the
            // *sight line* too (`det`/visibility axis, `NOTES.md §32` window).
            // A built-in cuboid needs no new asset.
            entity.insert((
                Mesh3d(meshes.add(Cuboid::new(extent.x, extent.y, extent.z))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: Color::srgb(0.42, 0.45, 0.50),
                    perceptual_roughness: 0.9,
                    ..default()
                })),
            ));
        }
    }
    if !scenario.occluders.is_empty() {
        info!(
            "[bench] scenario '{}': spawned {} occluder box(es) ({} collider-only)",
            scenario.id,
            scenario.occluders.len(),
            scenario.occluders.iter().filter(|spec| !spec.visible).count(),
        );
    }
    scenario.occluders_installed = true;
}

/// Reads `[bench] scenario_path` once the benchmark is enabled.
///
/// Runs in `Update` rather than `Startup` so a hot-reloaded `bench.enabled`
/// still works; it no-ops as soon as the resource exists. A scenario that does
/// not load is fatal: the run must not continue with an unknown motion.
pub fn load_scenario(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    existing: Option<Res<Scenario>>,
    mut exit: MessageWriter<AppExit>,
) {
    if !config.bench.enabled || existing.is_some() {
        return;
    }
    let limits = motion_limits(&config);
    let Some(path) = config.bench.scenario_path() else {
        error!("[bench] enabled but [bench] scenario_path is empty; exiting");
        exit.write(AppExit::error());
        return;
    };
    match Scenario::load(path, &limits) {
        Ok(scenario) => {
            info!(
                "[bench] loaded scenario '{}' (seed={}, duration={:.1}s, trigger={}, robots={})",
                scenario.id,
                scenario.seed,
                scenario.duration,
                scenario.trigger.name(),
                scenario.robots.len()
            );
            for robot in &scenario.robots {
                info!(
                    "[bench]   robot {}: {} segments, ends {:.1}s{}",
                    robot.id.0,
                    robot.segments.len(),
                    robot.last_segment_end,
                    if robot.spawn.is_some() {
                        " (repositioned at load)"
                    } else {
                        ""
                    }
                );
            }
            commands.insert_resource(scenario);
        }
        Err(error) => {
            error!("[bench] scenario load failed: {error}; exiting (non-zero)");
            exit.write(AppExit::error());
        }
    }
}

pub(crate) fn motion_limits(config: &SimulationConfig) -> MotionLimits {
    MotionLimits {
        max_speed: config.vehicle.max_speed,
        rotation_speed: config.vehicle.rotation_speed,
    }
}

/// Pristine pose of every robot, captured the first frame its body exists.
///
/// A scenario robot without a `spawn` is reset to this pose — the scene's own
/// placement, before any scenario teleport and before the settle drop that
/// follows it. Robots that do have a `spawn` are reset to the spawn, so this is
/// only the fallback.
#[derive(Resource, Default)]
pub struct SceneBaselines {
    poses: HashMap<RobotId, (Vec3, Quat)>,
    chassis_yaw: HashMap<RobotId, f32>,
}

impl SceneBaselines {
    pub fn pose(&self, id: RobotId) -> Option<(Vec3, Quat)> {
        self.poses.get(&id).copied()
    }

    pub fn chassis_yaw(&self, id: RobotId) -> Option<f32> {
        self.chassis_yaw.get(&id).copied()
    }
}

/// Captures [`SceneBaselines`], ordered before [`scenario_control`].
///
/// The spawn teleport runs inside `scenario_control`, so the pose read here on
/// the first frame the body exists is still the scene's own.
pub fn capture_scene_baselines(
    mut baselines: ResMut<SceneBaselines>,
    roots: Query<(Entity, &RobotId, Option<&Position>, Option<&Rotation>), With<Infantry>>,
    chassis: Query<&InfantryChassis, Without<Infantry>>,
    children: Query<&Children>,
) {
    for (root, id, position, rotation) in &roots {
        if !baselines.poses.contains_key(id) {
            if let (Some(position), Some(rotation)) = (position, rotation) {
                baselines.poses.insert(*id, (position.0, rotation.0));
                debug!("[bench] baseline pose for robot {}: {:?}", id.0, position.0);
            }
        }
        if !baselines.chassis_yaw.contains_key(id) {
            let found = children
                .iter_descendants(root)
                .find(|entity| chassis.contains(*entity));
            if let Some(data) = found.and_then(|entity| chassis.get(entity).ok()) {
                baselines.chassis_yaw.insert(*id, data.yaw);
            }
        }
    }
}

/// Physical half of a runtime `load`/`reset`/`restart` command.
///
/// Runs in the same frame as the bookkeeping in [`Scenario::apply_command`], and
/// before `install_planned_robots`, so a restart re-installs from a clean slate.
/// Everything a previous scenario could leave behind is undone here:
///
/// * every scripted driver is removed (with `ActiveSlapper` restored), so a robot
///   that only existed in the previous plan stops being driven;
/// * every robot of the new plan goes back to its initial pose — the `spawn` when
///   the scenario asks for one (the same `to_bevy` mapping the install pass uses,
///   including the chassis-heading compensation for mirrored models), the scene's
///   captured baseline otherwise — with both velocities zeroed;
/// * its queued inputs are dropped, so no stale stick value survives the reset.
///
/// Projectiles, shot state and rate limiters are reset separately in
/// `bench::services`, which owns those resources.
#[allow(clippy::too_many_arguments)]
fn apply_runtime_command(
    scenario: &mut Scenario,
    command: ScenarioCommand,
    fixed_now: f64,
    commands: &mut Commands,
    roots: &mut Query<
        (
            Entity,
            &RobotId,
            &GlobalTransform,
            Option<&mut Position>,
            Option<&mut Rotation>,
            Option<&mut LinearVelocity>,
            Option<&mut AngularVelocity>,
            Option<&mut Transform>,
        ),
        With<Infantry>,
    >,
    chassis: &mut Query<(
        &mut InfantryChassis,
        &mut Transform,
        &GlobalTransform,
    ), Without<Infantry>>,
    children: &Query<&Children>,
    baselines: &SceneBaselines,
    scripted: &Query<(Entity, &RobotId), (With<Infantry>, With<MotionSource>)>,
    robot_inputs: &mut RobotInputs,
) {
    // 1. Drop every scripted driver in the world.
    for (entity, id) in scripted.iter() {
        commands.entity(entity).remove::<MotionSource>();
        // These robots were taken over from the keyboard's *remote* slot, so hand
        // it back. The ego (id 0) belongs to the primary `Controlled` slot and
        // never had `ActiveSlapper`; adding it here would point the remote
        // bindings at the player's own vehicle after a reset.
        if id.0 != 0 {
            commands.entity(entity).insert(ActiveSlapper);
        }
    }

    // 2. Put every planned robot back on its starting pose.
    let wanted: Vec<(RobotId, Option<(Vec3, f32)>)> = scenario
        .robots
        .iter()
        .map(|robot| (robot.id, robot.spawn.map(|spawn| spawn.to_bevy())))
        .collect();
    for (id, spawn_pose) in wanted {
        let Some((
            root,
            _,
            _,
            position,
            rotation,
            linear_velocity,
            angular_velocity,
            transform,
        )) = roots.iter_mut().find(|(_, robot_id, ..)| **robot_id == id)
        else {
            continue;
        };
        let target = match spawn_pose {
            Some((position, yaw)) => Some((position, Quat::IDENTITY, Some(yaw))),
            None => baselines
                .pose(id)
                .map(|(position, rotation)| (position, rotation, None)),
        };
        let Some((position_value, rotation_value, world_yaw)) = target else {
            continue;
        };
        if let (Some(mut position), Some(mut rotation)) = (position, rotation) {
            position.0 = position_value;
            rotation.0 = rotation_value;
            if let Some(mut velocity) = linear_velocity {
                velocity.0 = Vec3::ZERO;
            }
            if let Some(mut velocity) = angular_velocity {
                velocity.0 = Vec3::ZERO;
            }
            // A teleport must move the transform too: on the frame the scenario
            // writes the pose, `Transform` still holds the scene pose, so any
            // code that syncs the transform back into the body would undo the
            // move.
            if let Some(mut transform) = transform {
                transform.translation = position_value;
                transform.rotation = rotation_value;
            }
            if let Some(chassis_entity) = children
                .iter_descendants(root)
                .find(|entity| chassis.contains(*entity))
            {
                if let Ok((mut data, mut transform, global)) = chassis.get_mut(chassis_entity) {
                    let parent = chassis_parent_frame(&global, &data);
                    let local_yaw = match world_yaw {
                        Some(yaw) => motion::local_chassis_yaw(parent, yaw),
                        None => baselines.chassis_yaw(id).unwrap_or(data.yaw),
                    };
                    data.yaw = local_yaw;
                    data.yaw_velocity = 0.0;
                    data.roll = 0.0;
                    data.pitch = 0.0;
                    transform.rotation = Quat::from_euler(EulerRot::YXZ, local_yaw, 0.0, 0.0);
                }
            }
        }
        robot_inputs.remove(id);
    }

    // 3. Bookkeeping last: it clears `installed` for a restart, which is what
    //    makes the install pass re-teleport and re-arm the scripted driver.
    scenario.apply_command(command, fixed_now);
}

/// Installs `MotionSource::Scripted` (and the optional spawn teleport) on every
/// planned robot, once the scene has finished spawning it.
///
/// The teleport is the one place a scenario writes pose directly, and it is
/// deliberately outside the dynamics: it happens *before* the trigger, so the
/// settle transient (the same 1 m drop `scene.rs` uses) is over before
/// `t_scenario = 0`. Everything after that goes through the vehicle forces.
fn install_planned_robots(
    scenario: &mut Scenario,
    commands: &mut Commands,
    roots: &mut Query<
        (
            Entity,
            &RobotId,
            &GlobalTransform,
            Option<&mut Position>,
            Option<&mut Rotation>,
            Option<&mut LinearVelocity>,
            Option<&mut AngularVelocity>,
            Option<&mut Transform>,
        ),
        With<Infantry>,
    >,
    chassis: &mut Query<(
        &mut InfantryChassis,
        &mut Transform,
        &GlobalTransform,
    ), Without<Infantry>>,
    children: &Query<&Children>,
) {
    if scenario.robots.iter().all(|robot| robot.installed) {
        return;
    }
    let found: HashMap<RobotId, Entity> = roots
        .iter()
        .map(|(entity, id, ..)| (*id, entity))
        .collect();

    for robot in scenario.robots.iter_mut() {
        if robot.installed {
            continue;
        }
        let Some(&root) = found.get(&robot.id) else {
            continue;
        };
        let Ok((
            _,
            _,
            _,
            position,
            rotation,
            linear_velocity,
            angular_velocity,
            transform,
        )) = roots.get_mut(root)
        else {
            continue;
        };

        // Everything the requested spawn implies, resolved before touching the
        // chassis: `None` means "leave the pose alone".
        let spawn_pose = if let Some(spawn) = robot.spawn {
            // `setup_vehicle` adds `RigidBody` — and with it avian's required
            // `Position`/`Rotation` — through deferred commands, so on the first
            // frame after the scene loads the robot exists but its body
            // components do not yet. Retry instead of silently keeping the
            // scene pose: a scenario that asks for a spawn must get one.
            let (Some(mut position), Some(mut rotation)) = (position, rotation) else {
                continue;
            };
            let (position_value, yaw) = spawn.to_bevy();
            position.0 = position_value;
            // The scenario heading lives in the chassis node (see
            // `update_chassis_rotation`), so the body root stays level.
            rotation.0 = Quat::IDENTITY;
            if let Some(mut velocity) = linear_velocity {
                velocity.0 = Vec3::ZERO;
            }
            if let Some(mut velocity) = angular_velocity {
                velocity.0 = Vec3::ZERO;
            }
            // Task-44 regression: the body teleport alone is not enough. On this
            // frame `Transform` still holds the scene pose, and the transform is
            // what the published `GlobalTransform` is computed from — with the
            // body left to the sync the move can be undone silently (the HERO
            // kept its scene position while its chassis yaw *did* come from the
            // spawn). Move both.
            if let Some(mut transform) = transform {
                transform.translation = position_value;
                transform.rotation = Quat::IDENTITY;
            }
            Some((spawn, position_value, yaw))
        } else {
            None
        };

        // A spawn is only "applied" once the chassis node exists too: the
        // scenario heading lives on that node, and `installed = true` (which the
        // status topic publishes as `spawn_applied`) must mean the whole spawn
        // landed. Otherwise retry next frame instead of reporting success.
        let chassis_written = if let Some(chassis_entity) = children
            .iter_descendants(root)
            .find(|entity| chassis.contains(*entity))
        {
            if let Ok((mut data, mut transform, global)) = chassis.get_mut(chassis_entity) {
                // `update_chassis_rotation` writes a *local* rotation, and not
                // every model's root frame agrees with the map frame: HERO.glb's
                // scene root carries a 180° rotation, so its world heading is
                // the negative of `data.yaw`. `parent` is the frame the local
                // rotation lives in, recovered from the world rotation with the
                // local one divided out — no model-specific constants.
                let parent = chassis_parent_frame(&global, &data);
                let wanted_heading = spawn_pose
                    .map(|(_, _, yaw)| yaw)
                    .unwrap_or(data.yaw * robot.yaw_sign);
                robot.yaw_sign = motion::chassis_yaw_sign(parent, wanted_heading);
                if let Some((_, _, yaw)) = spawn_pose {
                    let local_yaw = motion::local_chassis_yaw(parent, yaw);
                    data.yaw = local_yaw;
                    data.yaw_velocity = 0.0;
                    data.roll = 0.0;
                    data.pitch = 0.0;
                    transform.rotation = Quat::from_euler(EulerRot::YXZ, local_yaw, 0.0, 0.0);
                }
                true
            } else {
                false
            }
        } else {
            false
        };

        if spawn_pose.is_some() && !chassis_written {
            continue;
        }

        if let Some((spawn, position_value, _)) = spawn_pose {
            // The planned trajectory is measured against the scenario's own
            // frame of reference, so the origin is the spawn — not wherever the
            // scene happened to drop the robot (whose transform is not even
            // propagated yet on this frame).
            robot.origin = Some(position_value);
            // Ignore the teleport frame(s) in the deviation metric: the transform
            // propagation that publishes the new pose runs after this system.
            robot.deviation_skip_frames = SPAWN_DEVIATION_SKIP_FRAMES;
            info!(
                "[bench] robot {} spawned at map ({:.3}, {:.3}, {:.3}) yaw {:.1} deg",
                robot.id.0, spawn.x, spawn.y, spawn.z, spawn.yaw_deg
            );
        }

        // The scenario owns this robot: it must not also be driven by the
        // keyboard/gamepad. Dropping `ActiveSlapper` from the whole subtree is
        // what stops the IJKL bindings reaching a scripted robot's gimbal.
        commands.entity(root).insert(MotionSource::SCRIPTED);
        commands.entity(root).remove::<ActiveSlapper>();
        for descendant in children.iter_descendants(root) {
            commands.entity(descendant).remove::<ActiveSlapper>();
        }
        robot.installed = true;
        info!("[bench] robot {} is now scripted", robot.id.0);
    }
}

/// World yaw of a robot's chassis node, or `None` if it has none.
///
/// Deliberately reads through the *same* query the spawn teleport mutates: Bevy
/// rejects a system that holds both `Query<&mut InfantryChassis>` and
/// `Query<&InfantryChassis>` with a `B0001` panic, and `Query::get` hands back
/// the read-only projection of a mutable query, so one query serves both.
fn chassis_heading(
    root: Entity,
    children: &Query<&Children>,
    chassis: &Query<(
        &mut InfantryChassis,
        &mut Transform,
        &GlobalTransform,
    ), Without<Infantry>>,
) -> Option<f32> {
    let entity = children
        .iter_descendants(root)
        .find(|descendant| chassis.contains(*descendant))?;
    chassis.get(entity).ok().map(|(data, _, _)| data.yaw)
}

/// The frame a chassis' local rotation is expressed in: its world rotation with
/// its own local rotation divided out.
///
/// This is what makes the yaw mirroring in [`motion::local_chassis_yaw`]
/// model-agnostic: `HERO.glb`'s scene root rotates the body by 180°, so the
/// recovered frame differs from `vehicle.glb`'s by exactly that rotation.
fn chassis_parent_frame(global: &GlobalTransform, data: &InfantryChassis) -> Quat {
    let local = Quat::from_euler(EulerRot::YXZ, data.yaw, data.pitch, data.roll);
    global.rotation() * local.inverse()
}

/// The benchmark's main Update system: install robots, advance the clock, drive
/// the scripted robots and accumulate trajectory deviation.
#[allow(clippy::too_many_arguments)]
/// Task-44: make the scenario's declared presence real, every frame.
///
/// A robot the scenario does not list (or lists with `present = false`) must not
/// exist for the run: it stays where the scene put it, but it is hidden from the
/// camera, its rigid body and colliders are disabled, and it carries
/// [`ScenarioAbsent`] so GT, `/tf` and the input router can skip it with a single
/// filter. Marking is idempotent and fully reversible — the pose is never
/// touched, so a robot that becomes present again is exactly where it was.
fn apply_scenario_presence(
    scenario: &Scenario,
    commands: &mut Commands,
    robots: &mut Query<
        (
            Entity,
            &RobotId,
            &mut Visibility,
            Has<ScenarioAbsent>,
            &Children,
        ),
        With<Infantry>,
    >,
    children: &Query<&Children>,
    colliders: &Query<Entity, With<Collider>>,
) {
    for (entity, id, mut visibility, absent, _) in robots.iter_mut() {
        let wanted = scenario.absent.contains(id);
        if wanted {
            if !absent {
                commands.entity(entity).insert((
                    ScenarioAbsent,
                    RigidBodyDisabled,
                    LinearVelocity(Vec3::ZERO),
                    AngularVelocity(Vec3::ZERO),
                ));
                for descendant in children.iter_descendants(entity) {
                    commands.entity(descendant).insert(ScenarioAbsent);
                    if colliders.contains(descendant) {
                        commands.entity(descendant).insert(ColliderDisabled);
                    }
                }
                info!(
                    "[bench] robot {} is absent: hidden, rigid body and colliders disabled, \
                     excluded from GT//tf/input",
                    id.0
                );
            }
            if *visibility != Visibility::Hidden {
                *visibility = Visibility::Hidden;
            }
        } else if absent {
            commands
                .entity(entity)
                .remove::<(ScenarioAbsent, RigidBodyDisabled)>();
            for descendant in children.iter_descendants(entity) {
                commands
                    .entity(descendant)
                    .remove::<(ScenarioAbsent, ColliderDisabled)>();
            }
            *visibility = Visibility::Visible;
            info!(
                "[bench] robot {} is present again: visible, colliders re-enabled",
                id.0
            );
        }
    }
}

/// `DAEDALUS_SPAWN_DEBUG=1`: print the pose of every robot root for the first
/// 150 frames of a run, straight from the two sources that can disagree —
/// avian's `Position` and the `Transform` the published `GlobalTransform` is
/// built from. Added while chasing the task-44 teleport regression, where the
/// install log proved a spawn was written yet the robot stayed on its scene
/// pose; the per-frame view says whether the write never landed on this entity
/// or was overwritten afterwards. Costs one env lookup per frame when off.
pub fn debug_spawn_tracking(
    mut frames: Local<u32>,
    robots: Query<(Entity, &RobotId, &Position, &Transform, &GlobalTransform), With<Infantry>>,
) {
    if *frames >= 150 || std::env::var_os("DAEDALUS_SPAWN_DEBUG").is_none() {
        return;
    }
    *frames += 1;
    for (entity, id, position, transform, global) in &robots {
        info!(
            "[bench][spawndbg] frame={} robot={} entity={:?} position={:?} \
             transform={:?} global={:?}",
            *frames,
            id.0,
            entity,
            position.0,
            transform.translation,
            global.translation()
        );
    }
}

pub fn scenario_control(
    time_fixed: Res<Time<Fixed>>,
    mut commands: Commands,
    mut robot_inputs: ResMut<RobotInputs>,
    mut scenario: Option<ResMut<Scenario>>,
    config: Res<SimulationConfig>,
    first_cmd: Res<FirstGimbalCmd>,
    baselines: Res<SceneBaselines>,
    scripted: Query<(Entity, &RobotId), (With<Infantry>, With<MotionSource>)>,
    mut roots: Query<
        (
            Entity,
            &RobotId,
            &GlobalTransform,
            Option<&mut Position>,
            Option<&mut Rotation>,
            Option<&mut LinearVelocity>,
            Option<&mut AngularVelocity>,
            Option<&mut Transform>,
        ),
        With<Infantry>,
    >,
    robot_poses: Query<(Entity, &RobotId, &GlobalTransform), With<Infantry>>,
    mut presence: Query<
        (
            Entity,
            &RobotId,
            &mut Visibility,
            Has<ScenarioAbsent>,
            &Children,
        ),
        With<Infantry>,
    >,
    colliders: Query<Entity, With<Collider>>,
    mut chassis: Query<(
        &mut InfantryChassis,
        &mut Transform,
        &GlobalTransform,
    ), Without<Infantry>>,
    children: Query<&Children>,
) {
    let Some(scenario) = scenario.as_mut() else {
        return;
    };
    let limits = motion_limits(&config);
    let fixed_now = time_fixed.elapsed_secs_f64();

    // Task-44: honour the scenario's declared presence before anything else, so
    // a robot the scenario leaves out is already hidden and physically disabled
    // on the first frame (the camera must not see it during warm-up).
    apply_scenario_presence(
        scenario,
        &mut commands,
        &mut presence,
        &children,
        &colliders,
    );

    // P1-B: apply a queued load/reset/restart physically before the install pass,
    // so a restart rebuilds the spawn and the scripted driver from a clean slate.
    let command = std::mem::take(&mut scenario.pending);
    if command != ScenarioCommand::None {
        apply_runtime_command(
            scenario,
            command,
            fixed_now,
            &mut commands,
            &mut roots,
            &mut chassis,
            &children,
            &baselines,
            &scripted,
            &mut robot_inputs,
        );
    }

    install_planned_robots(scenario, &mut commands, &mut roots, &mut chassis, &children);

    // Loud spawn diagnostics: a scenario that asks for a spawn must get one, and
    // `/sim/scenario/status` has to say so. `installed` is only set once both the
    // pose and the chassis heading were written (see `install_planned_robots`), so
    // a robot still pending here is a real problem, not a reporting detail.
    let pending: Vec<u8> = scenario
        .robots
        .iter()
        .filter(|robot| robot.spawn.is_some() && !robot.installed)
        .map(|robot| robot.id.0)
        .collect();
    if pending.is_empty() {
        scenario.spawn_deadline_secs = None;
        if scenario.spawn_error.take().is_some() {
            info!("[bench] all requested spawns are applied; status healthy again");
        }
        scenario.spawn_error_logged = false;
    } else {
        let deadline = *scenario
            .spawn_deadline_secs
            .get_or_insert(fixed_now + SPAWN_TIMEOUT_SECS);
        if fixed_now > deadline && !scenario.spawn_error_logged {
            let ids = pending
                .iter()
                .map(|id| id.to_string())
                .collect::<Vec<_>>()
                .join(", ");
            let message = format!(
                "spawn not applied within {SPAWN_TIMEOUT_SECS:.1}s for robot(s) {ids}; \
                 they keep their scene pose and are not driven"
            );
            error!("[bench] {message}");
            scenario.spawn_error = Some(message);
            scenario.spawn_error_logged = true;
        }
    }
    // A runtime `restart` is already Running when `apply_command` returns, so it
    // must not count as "was running" or the origin capture below (the anchor of
    // the deviation metric) would be skipped for this run.
    let was_running =
        scenario.phase == Phase::Running && command == ScenarioCommand::None;
    scenario.advance_clock(fixed_now, first_cmd.seen());

    if scenario.phase == Phase::Idle {
        // Before the trigger nothing drives the planned robots: they keep the
        // pose the scene (or their spawn) gave them.
        return;
    }

    let mut positions: HashMap<RobotId, Vec3> = HashMap::new();
    let mut frames: HashMap<RobotId, MotionFrame> = HashMap::new();
    let mut headings: HashMap<RobotId, f32> = HashMap::new();
    let mut entities: HashMap<RobotId, Entity> = HashMap::new();
    let signs: HashMap<RobotId, f32> = scenario
        .robots
        .iter()
        .map(|robot| (robot.id, robot.yaw_sign))
        .collect();
    for (entity, id, transform) in &robot_poses {
        positions.insert(*id, transform.translation());
        frames.insert(*id, MotionFrame::from_transform(transform));
        entities.insert(*id, entity);
        if let Some(heading) = chassis_heading(entity, &children, &chassis) {
            // `chassis_heading` is the chassis' *local* yaw; the primitives and
            // the controller both work in the map frame, which is one mirror
            // away on a model like HERO.
            headings.insert(*id, heading * signs.get(id).copied().unwrap_or(1.0));
        }
    }

    if scenario.phase == Phase::Running && !was_running {
        for robot in scenario.robots.iter_mut() {
            // A requested spawn already pinned the origin to the scenario's
            // frame of reference; without one, the robot's pose at the moment
            // the scenario starts is the reference.
            if robot.origin.is_none() {
                robot.origin = positions.get(&robot.id).copied();
            }
            info!(
                "[bench] robot {} nominal origin (map): {:?}",
                robot.id.0,
                robot.origin.map(ros_map_of_bevy)
            );
        }
    }

    let t_scenario = scenario.t_scenario;
    let running = scenario.phase == Phase::Running;
    for robot in scenario.robots.iter_mut() {
        let Some(&actual) = positions.get(&robot.id) else {
            continue;
        };
        // A requested spawn whose body does not exist yet: driving or measuring
        // it before it is teleported would be meaningless, and the install pass
        // above retries on the next frame.
        if robot.spawn.is_some() && !robot.installed {
            continue;
        }
        if !running {
            // After `duration`: zero input, hold position, no reset.
            robot_inputs.set(
                robot.id,
                ControllerInput {
                    boost: false,
                    ..Default::default()
                },
            );
            continue;
        }

        let primitive = robot.primitive_at(t_scenario);
        let frame = frames.get(&robot.id).copied().unwrap_or(MotionFrame {
            forward: Vec3::NEG_Z,
            right: Vec3::X,
        });
        let heading = headings.get(&robot.id).copied().unwrap_or(0.0);
        let command = primitive.evaluate(robot.local_time(t_scenario), heading, &frame, &limits);
        robot_inputs.set(
            robot.id,
            ControllerInput {
                movement: command.movement,
                // The evaluator reasons in the map frame; `update_chassis_rotation`
                // integrates the chassis' *local* yaw, so on a mirrored model the
                // commanded world yaw rate has to be scaled by the same sign.
                chassis_yaw: command.chassis_yaw * robot.yaw_sign,
                // `ControllerInput::boost` is the keyboard's Shift (a x2
                // multiplier), so the command's multiplier maps onto it.
                boost: command.boost > 1.0,
                ..Default::default()
            },
        );

        let Some(origin) = robot.origin else {
            continue;
        };
        // The teleport lands on the install frame, but `GlobalTransform` keeps
        // carrying the pre-spawn pose until the next propagation pass. Measuring
        // then latches a spurious cross-track spike — the distance from the scene
        // default pose, up to several metres — into the scenario maximum, which
        // would make an otherwise perfect spawn look like a failed trajectory.
        if robot.deviation_skip_frames > 0 {
            robot.deviation_skip_frames -= 1;
            robot.max_cross_track_m = 0.0;
            robot.max_along_track_m = 0.0;
            continue;
        }
        let nominal = robot.nominal_position(t_scenario, origin);
        let velocity = robot.nominal_velocity(t_scenario);
        // Task-59: the split plane is the commanded velocity while a leg is
        // powered, and — because `split_direction` remembers it — the last leg's
        // direction through a `cross_los` coast and through the `static` tail that
        // follows. Without that memory the tail falls back to `|delta|` and counts
        // the honest ramp/coast lag as cross-track, which latched a metre-scale
        // spike (2.0858 m on a healthy ego `line` leg) into the validity gate.
        // `reference_nominal_velocity` still supplies the current segment's own
        // direction; a scenario that never moves keeps the old fallback.
        let reference = if velocity.length_squared() > 1e-12 {
            velocity
        } else {
            robot.reference_nominal_velocity(t_scenario)
        };
        let split = split_direction(reference, &mut robot.last_reference_velocity);
        let (cross, along) = track_error(actual, nominal, split);
        robot.max_cross_track_m = robot.max_cross_track_m.max(cross);
        robot.max_along_track_m = robot.max_along_track_m.max(along.abs());

        // Task-59 stall watchdog. The cross-track gate cannot see a robot that was
        // told to move and did not: its error is *along* the commanded direction,
        // which the split deliberately ignores. Keep the detection power by
        // watching the speeds, then fold the positional error back into the gated
        // metric so the run fails validity loudly rather than passing clean.
        let commanded = velocity.length();
        let achieved = entities
            .get(&robot.id)
            .and_then(|entity| roots.get(*entity).ok())
            .and_then(|(_, _, _, _, _, linear, _, _)| {
                linear.map(|linear| linear.0.length())
            })
            .unwrap_or(0.0);
        if is_stalled(commanded, achieved) {
            let since = *robot.stall_since.get_or_insert(t_scenario);
            if t_scenario - since >= STALL_GRACE_SECS {
                let error = (actual - nominal).with_y(0.0).length();
                robot.max_cross_track_m = robot.max_cross_track_m.max(error);
                if !robot.stall_logged {
                    robot.stall_logged = true;
                    error!(
                        "[bench] robot {} is commanded {:.2} m/s but measures {:.2} m/s \
                         for {:.1}s: folding the {:.3}m positional error into the \
                         deviation gate",
                        robot.id.0,
                        commanded,
                        achieved,
                        t_scenario - since,
                        error
                    );
                }
            }
        } else {
            robot.stall_since = None;
        }
    }

    scenario.max_track_deviation_m = scenario
        .robots
        .iter()
        .map(|robot| robot.max_cross_track_m)
        .fold(0.0, f32::max);
    // Reported, never gated: it is the "how far behind the ideal path" number.
    scenario.max_along_track_m = scenario
        .robots
        .iter()
        .map(|robot| robot.max_along_track_m)
        .fold(0.0, f32::max);

    if running && let Some(worst) = scenario
        .robots
        .iter()
        .max_by(|a, b| a.max_cross_track_m.total_cmp(&b.max_cross_track_m))
    {
        debug!(
            "[bench] t={:.3}s cross-track max {:.3}m (robot {}), along-track max {:.3}m",
            t_scenario, worst.max_cross_track_m, worst.id.0, worst.max_along_track_m
        );
    }
}

/// Decompose "actual minus ideal" into the component along the ideal velocity
/// and the component across it (ground plane only).
///
/// Only the cross-track part is reported: an along-track lag is what
/// `linear_acceleration` and `rotation_speed` *do*, so counting it as scenario
/// deviation would fail every high-speed reversal for being physically possible.
fn track_error(actual: Vec3, nominal: Vec3, nominal_velocity: Vec3) -> (f32, f32) {
    let delta = (actual - nominal).with_y(0.0);
    let speed = nominal_velocity.length();
    if speed <= 1e-6 {
        return (delta.length(), 0.0);
    }
    let direction = nominal_velocity / speed;
    let along = delta.dot(direction);
    ((delta - direction * along).length(), along)
}

/// Bevy → map (ROS) position, for logging only.
fn ros_map_of_bevy(bevy: Vec3) -> [f32; 3] {
    [-bevy.z, -bevy.x, bevy.y]
}

/// The plane the cross/along split uses, remembered across segments.
///
/// The instantaneous commanded velocity is the natural split plane while a leg
/// is powered, but it is zero on the `static` tail (and through a `cross_los`
/// coast), and then `track_error` would fall back to `|delta|` and count the
/// accumulated ramp/coast lag as cross-track — a metre-scale false positive.
/// Task-59: keep the last non-zero direction instead, so the tail splits by the
/// leg it came from. A scenario that never moves leaves this zero, which keeps
/// the old fallback for pure `static`/`spin` runs.
fn split_direction(nominal_velocity: Vec3, remembered: &mut Vec3) -> Vec3 {
    if nominal_velocity.length_squared() > 1e-12 {
        *remembered = nominal_velocity;
    }
    *remembered
}

/// A scripted robot is stalled when it was told to move and did not.
///
/// The cross-track gate cannot see this: a robot that never moves deviates
/// *along* the commanded direction, which the split deliberately ignores (that
/// is the honest ramp). Task-59 keeps the detection power by watching the speeds
/// instead of the positions.
fn is_stalled(commanded_speed: f32, achieved_speed: f32) -> bool {
    commanded_speed > STALL_MIN_COMMAND_MPS
        && achieved_speed < STALL_MIN_ACHIEVED_FRACTION * commanded_speed
}

pub fn publish_scenario_status(
    time_fixed: Res<Time<Fixed>>,
    mut scenario: Option<ResMut<Scenario>>,
    ros_stamp: Res<crate::ros2::plugin::RosFrameStamp>,
    status_pub: Res<TopicPublisher<SimScenarioStatusTopic>>,
) {
    let Some(scenario) = scenario.as_mut() else {
        return;
    };
    let fixed_now = time_fixed.elapsed_secs_f64();
    if fixed_now - scenario.status_last_secs < 1.0 / STATUS_HZ {
        return;
    }
    scenario.status_last_secs = fixed_now;

    // `installed` means the spawn really landed (pose *and* chassis heading), so
    // this is the honest on-wire answer to "did my scenario geometry apply?".
    // A scenario with no spawns trivially qualifies.
    let spawn_applied = scenario.spawn_applied();
    // `t_scenario` stays `float32` seconds on the wire on purpose: at 20 s a
    // float32 resolves ~2e-6 s, and this field is for humans and window edges.
    // The nanosecond field is the one that had to be widened to u64.
    status_pub.publish(SimScenarioStatus {
        header: Header {
            stamp: ros_stamp.0.clone(),
            frame_id: MAP_FRAME.to_string(),
        },
        scenario_id: scenario.id.clone(),
        seed: scenario.seed,
        t_scenario: scenario.t_scenario,
        phase: scenario.phase.wire(),
        max_track_deviation_m: scenario.max_track_deviation_m,
        max_along_track_m: scenario.max_along_track_m,
        spawn_applied,
        spawn_error: scenario.spawn_error.clone().unwrap_or_default(),
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn limits() -> MotionLimits {
        MotionLimits::default()
    }

    /// `Scenario::parse` returns a `String` error; `expect_err` would need
    /// `Scenario: Debug`, and printing a whole scenario is not worth deriving.
    fn parse_err(text: &str) -> String {
        match Scenario::parse(text, &limits()) {
            Ok(_) => panic!("expected a parse error"),
            Err(error) => error,
        }
    }

    const SMOKE: &str = r#"
id = "smoke_static"
seed = 20261014
duration = 20.0
trigger = "on_first_gimbal_cmd"

[[robot]]
id = 1

[[robot.segment]]
t0 = 0.0
t1 = 20.0
type = "static"
"#;

    #[test]
    fn parses_a_static_scenario() {
        let scenario = Scenario::parse(SMOKE, &limits()).expect("smoke scenario parses");

        assert_eq!(scenario.id, "smoke_static");
        assert_eq!(scenario.seed, 20261014);
        assert_eq!(scenario.trigger, Trigger::OnFirstGimbalCmd);
        assert_eq!(scenario.phase, Phase::Idle);
        assert_eq!(scenario.robots.len(), 1);
        assert_eq!(scenario.robots[0].id, RobotId(1));
        assert!(scenario.robots[0].spawn.is_none());
        assert_eq!(scenario.robots[0].primitive_at(5.0), Primitive::Static);
    }

    #[test]
    fn trigger_defaults_to_the_first_gimbal_command() {
        // The v1-compatible t0: no solver, no clock.
        let scenario = Scenario::parse(
            "id = \"x\"\nduration = 1.0\n[[robot]]\nid = 1\n",
            &limits(),
        )
        .unwrap();

        assert_eq!(scenario.trigger, Trigger::OnFirstGimbalCmd);
    }

    #[test]
    fn a_static_or_spin_scenario_never_remembers_a_split_direction() {
        // Task-59 inertness proof: static and spin legs command no velocity at all,
        // so `split_direction` keeps its zero memory and `track_error` keeps the old
        // `|delta|` fallback => every static/spin-only scenario reads exactly as it
        // did before the fix.
        let scenario = Scenario::parse(
            "id = \"still\"\nduration = 4.0\n\n[[robot]]\nid = 1\n\
             spawn = { x = -2.55, y = -2.243, z = 0.0543 }\n\n\
             [[robot.segment]]\nt0 = 0.0\nt1 = 2.0\ntype = \"static\"\n\n\
             [[robot.segment]]\nt0 = 2.0\nt1 = 4.0\ntype = \"spin\"\nomega = 6.0\n",
            &limits(),
        )
        .expect("scenario parses");

        let robot = &scenario.robots[0];
        let mut remembered = Vec3::ZERO;
        for t in [0.0, 1.0, 2.5, 3.999] {
            let reference = robot.reference_nominal_velocity(t);
            assert_eq!(reference, Vec3::ZERO, "static/spin command no velocity (t={t})");
            assert_eq!(split_direction(reference, &mut remembered), Vec3::ZERO);
        }
        assert_eq!(remembered, Vec3::ZERO, "no direction was ever remembered");

        // A zero split plane still means the legacy `|delta|` fallback.
        let (cross, along) = track_error(Vec3::new(0.4, 0.0, 0.0), Vec3::ZERO, remembered);
        assert!((cross - 0.4).abs() < 1e-6, "cross={cross}");
        assert_eq!(along, 0.0);
    }

    #[test]
    fn a_leg_tail_keeps_splitting_along_the_last_leg_and_reports_it() {
        // A leg that ramps up and then sits on a static tail: the tail is lag along
        // the leg, reported through `along`, never through the cross-track gate.
        let scenario = Scenario::parse(
            "id = \"tail\"\nduration = 6.0\n\n[[robot]]\nid = 1\n\
             spawn = { x = -2.55, y = -2.243, z = 0.0543 }\n\n\
             [[robot.segment]]\nt0 = 0.0\nt1 = 2.0\ntype = \"line\"\nv = 2.0\ndir_deg = 0.0\n\n\
             [[robot.segment]]\nt0 = 2.0\nt1 = 6.0\ntype = \"static\"\n",
            &limits(),
        )
        .expect("scenario parses");

        let robot = &scenario.robots[0];
        let leg = robot.reference_nominal_velocity(1.0);
        assert!((leg.length() - 2.0).abs() < 1e-4, "the leg commands 2 m/s, got {leg:?}");
        let mut remembered = Vec3::ZERO;
        assert_eq!(split_direction(leg, &mut remembered), leg);
        assert_eq!(robot.reference_nominal_velocity(5.0), Vec3::ZERO);

        // 1.5 m behind the ideal point: cross-track ~0, along-track -1.5.
        let (cross, along) = track_error(leg * 0.0 + Vec3::new(0.0, 0.0, 1.5), Vec3::ZERO, remembered);
        assert!(cross.abs() < 1e-4, "cross={cross}");
        assert!((along + 1.5).abs() < 1e-4, "along={along}");

        // A stalled robot is a different signal: it never moves, so the watchdog -
        // not the cross-track gate - has to catch it.
        assert!(is_stalled(2.0, 0.0));
        assert!(!is_stalled(2.0, 0.6));
        assert!(!is_stalled(0.2, 0.0), "no command, no stall");
    }

    #[test]
    fn a_runtime_reset_clears_run_state_and_leaves_the_clock_idle() {
        let mut scenario = Scenario::parse(SMOKE, &limits()).expect("scenario parses");
        scenario.t_scenario = 12.5;
        scenario.max_track_deviation_m = 3.5;
        scenario.max_along_track_m = 9.5;
        scenario.spawn_error = Some("boom".to_string());
        scenario.phase = Phase::Done;
        scenario.robots[0].installed = false;
        scenario.robots[0].max_cross_track_m = 1.5;

        scenario.apply_command(ScenarioCommand::Reset, 42.0);

        assert_eq!(scenario.t_scenario, 0.0);
        assert_eq!(scenario.phase, Phase::Idle);
        assert_eq!(scenario.max_track_deviation_m, 0.0);
        assert_eq!(scenario.max_along_track_m, 0.0);
        assert!(scenario.spawn_error.is_none());
        assert_eq!(scenario.origin_fixed_secs, None);
        // A reset restores the pose itself, so the install pass must stay away
        // (`installed = true`) or the idle scenario would re-teleport and re-arm.
        assert!(scenario.robots[0].installed);
        assert_eq!(scenario.robots[0].max_cross_track_m, 0.0);
        assert_eq!(scenario.robots[0].origin, None);
        assert_eq!(
            scenario.robots[0].deviation_skip_frames,
            SPAWN_DEVIATION_SKIP_FRAMES
        );
    }

    #[test]
    fn a_runtime_restart_reinstalls_and_runs_from_zero() {
        let mut scenario = Scenario::parse(SMOKE, &limits()).expect("scenario parses");
        scenario.t_scenario = 7.0;
        scenario.phase = Phase::Done;
        scenario.robots[0].installed = true;

        scenario.apply_command(ScenarioCommand::Restart, 42.0);

        assert_eq!(scenario.t_scenario, 0.0);
        assert_eq!(scenario.phase, Phase::Running);
        // The origin is re-captured on this run, from the restart instant.
        assert_eq!(scenario.origin_fixed_secs, Some(42.0));
        // `installed = false` is what makes the install pass re-teleport a spawn
        // and re-arm the scripted driver.
        assert!(!scenario.robots[0].installed);
    }

    #[test]
    fn a_runtime_load_cannot_be_started_by_the_toml_trigger() {
        // `on_load` starts as soon as the robots exist; a runtime `load` must wait
        // for an explicit `start`, so the trigger is suppressed until then.
        let on_load = SMOKE.replace("on_first_gimbal_cmd", "on_load");
        let mut scenario = Scenario::parse(&on_load, &limits()).expect("scenario parses");

        scenario.apply_command(ScenarioCommand::Load, 3.0);
        scenario.advance_clock(10.0, true);

        assert_eq!(scenario.phase, Phase::Idle);
        assert_eq!(scenario.t_scenario, 0.0);
    }

    #[test]
    fn unsupported_primitives_are_an_error_not_a_fallback() {
        let text = r#"
id = "bad"
duration = 5.0

[[robot]]
id = 1

[[robot.segment]]
t0 = 0.0
t1 = 5.0
type = "circle"
radius_m = 1.5
v = 3.0
"#;
        let error = parse_err(text);

        assert!(error.contains("unsupported primitive 'circle'"), "{error}");
    }

    #[test]
    fn missing_primitive_parameters_are_an_error() {
        let text = r#"
id = "bad"
duration = 5.0

[[robot]]
id = 1

[[robot.segment]]
t0 = 0.0
t1 = 5.0
type = "line"
v = 2.0
"#;
        let error = parse_err(text);

        assert!(error.contains("needs 'dir_deg'"), "{error}");
    }

    #[test]
    fn the_ego_can_be_placed_on_a_corridor() {
        // P1 `ego_move`: the ego's default pose has less than 3 m of clear runway
        // in every direction (the central platform is ~1.4 m east and the plateau
        // rim 0.7-2.6 m elsewhere), so a scripted ego may be placed with the same
        // `spawn` key the other robots use. It is still never *absent*: it hosts
        // the camera.
        let text = r#"
id = "ego_placed"
duration = 5.0
trigger = "on_load"

[[robot]]
id = 0
spawn = { x = -6.5, y = 11.3, z = -0.2465, yaw_deg = 0.0 }

[[robot.segment]]
t0 = 0.0
t1 = 5.0
type = "line"
v = 2.0
dir_deg = 0.0
"#;
        let scenario = Scenario::parse(text, &limits()).unwrap();

        assert_eq!(scenario.robots.len(), 1);
        assert_eq!(scenario.robots[0].id, RobotId(0));
        assert!(scenario.robots[0].spawn.is_some());
    }

    #[test]
    fn overlapping_segments_are_rejected() {
        let text = r#"
id = "bad"
duration = 5.0

[[robot]]
id = 1

[[robot.segment]]
t0 = 0.0
t1 = 3.0
type = "static"

[[robot.segment]]
t0 = 2.0
t1 = 5.0
type = "static"
"#;
        let error = parse_err(text);

        assert!(error.contains("overlaps"), "{error}");
    }

    #[test]
    fn primitive_parameters_are_clamped_to_the_vehicle() {
        let text = r#"
id = "fast"
duration = 5.0

[[robot]]
id = 1

[[robot.segment]]
t0 = 0.0
t1 = 5.0
type = "line"
v = 9.0
dir_deg = 0.0
"#;
        let scenario = Scenario::parse(text, &limits()).unwrap();

        assert_eq!(scenario.robots[0].segments[0].primitive.v(), Some(6.0));
    }

    #[test]
    fn segments_tile_the_scenario_and_outside_them_the_robot_holds() {
        let text = r#"
id = "tiles"
duration = 20.0

[[robot]]
id = 1

[[robot.segment]]
t0 = 0.0
t1 = 5.0
type = "static"

[[robot.segment]]
t0 = 5.0
t1 = 10.0
type = "line"
v = 2.0
dir_deg = 90.0
"#;
        let scenario = Scenario::parse(text, &limits()).unwrap();
        let robot = &scenario.robots[0];

        assert_eq!(robot.primitive_at(0.0), Primitive::Static);
        assert_eq!(robot.primitive_at(4.999), Primitive::Static);
        assert_eq!(
            robot.primitive_at(5.0),
            Primitive::Line {
                v: 2.0,
                dir_deg: 90.0
            }
        );
        assert_eq!(
            robot.primitive_at(19.9),
            Primitive::Static,
            "after the last segment the robot holds"
        );
    }

    #[test]
    fn nominal_trajectory_is_the_limit_free_path() {
        let text = r#"
id = "nominal"
duration = 10.0

[[robot]]
id = 1

[[robot.segment]]
t0 = 0.0
t1 = 4.0
type = "line"
v = 2.0
dir_deg = 0.0

[[robot.segment]]
t0 = 4.0
t1 = 10.0
type = "static"
"#;
        let scenario = Scenario::parse(text, &limits()).unwrap();
        let robot = &scenario.robots[0];
        let origin = Vec3::new(1.0, 0.5, 2.0);

        // ROS +X is Bevy -Z.
        let after_2s = robot.nominal_position(2.0, origin);
        assert!(
            (after_2s - (origin + Vec3::new(0.0, 0.0, -4.0))).length() < 1e-4,
            "{after_2s:?}"
        );
        // Then it holds: the static segment adds nothing.
        let after_8s = robot.nominal_position(8.0, origin);
        assert!(
            (after_8s - (origin + Vec3::new(0.0, 0.0, -8.0))).length() < 1e-4,
            "{after_8s:?}"
        );
        assert_eq!(robot.nominal_velocity(6.0), Vec3::ZERO);
    }

    #[test]
    fn cross_track_ignores_along_track_lag() {
        let nominal = Vec3::new(0.0, 0.0, 0.0);
        // Ideal motion along -Z at 4 m/s.
        let velocity = Vec3::new(0.0, 0.0, -4.0);

        // 30 cm behind (pure lag) and 5 cm to the side.
        let actual = Vec3::new(0.05, 0.0, 0.3);
        let (cross, along) = track_error(actual, nominal, velocity);
        assert!((cross - 0.05).abs() < 1e-5, "cross={cross}");
        assert!((along + 0.3).abs() < 1e-5, "along={along}");

        // Standing still (static/spin): the whole distance is the error.
        let (cross, along) = track_error(Vec3::new(0.3, 0.0, 0.4), nominal, Vec3::ZERO);
        assert!((cross - 0.5).abs() < 1e-5, "cross={cross}");
        assert_eq!(along, 0.0);

        // The 1 m spawn drop is vertical: it is not a trajectory error.
        let (cross, _) = track_error(Vec3::new(0.0, -1.0, 0.0), nominal, velocity);
        assert_eq!(cross, 0.0);
    }

    #[test]
    fn spawn_is_read_in_the_map_frame() {
        // ROS (-1, -1, 0) is the default Blue slapper's map position, which is
        // Bevy (1, 1, 1) in the v1 scene.
        let (position, yaw) = SpawnSpec {
            x: -1.0,
            y: -1.0,
            z: 0.0,
            yaw_deg: 90.0,
        }
        .to_bevy();

        assert!((position - Vec3::new(1.0, 1.0, 1.0)).length() < 1e-5, "{position:?}");
        assert!((yaw - std::f32::consts::FRAC_PI_2).abs() < 1e-6);
    }

    #[test]
    fn clock_only_starts_on_its_trigger() {
        let mut scenario = Scenario::parse(SMOKE, &limits()).unwrap();
        scenario.robots[0].installed = true;

        scenario.advance_clock(100.0, false);
        assert_eq!(scenario.phase, Phase::Idle);
        assert_eq!(scenario.t_scenario, 0.0);

        scenario.advance_clock(100.5, true);
        assert_eq!(scenario.phase, Phase::Running);
        assert_eq!(scenario.t_scenario, 0.0);

        scenario.advance_clock(105.5, true);
        assert!((scenario.t_scenario - 5.0).abs() < 1e-4);

        // Past `duration` the clock stops and the phase holds.
        scenario.advance_clock(130.0, true);
        assert_eq!(scenario.phase, Phase::Done);
        assert_eq!(scenario.t_scenario, 20.0);
        scenario.advance_clock(140.0, true);
        assert_eq!(scenario.t_scenario, 20.0, "done never rewinds");
    }

    #[test]
    fn on_load_starts_without_a_solver() {
        let text = "id = \"x\"\nduration = 2.0\ntrigger = \"on_load\"\n\n[[robot]]\nid = 1\n";
        let mut scenario = Scenario::parse(text, &limits()).unwrap();
        scenario.robots[0].installed = true;

        scenario.advance_clock(10.0, false);
        assert_eq!(scenario.phase, Phase::Running);
    }

    #[test]
    fn scenario_waits_for_every_robot_before_starting() {
        let text = "id = \"x\"\nduration = 2.0\ntrigger = \"on_load\"\n\n[[robot]]\nid = 1\n\n[[robot]]\nid = 2\n";
        let mut scenario = Scenario::parse(text, &limits()).unwrap();

        scenario.robots[0].installed = true;
        scenario.advance_clock(10.0, false);
        assert_eq!(scenario.phase, Phase::Idle, "robot 2 is not in the scene yet");

        scenario.robots[1].installed = true;
        scenario.advance_clock(11.0, false);
        assert_eq!(scenario.phase, Phase::Running);
    }

    #[test]
    fn sim_time_ns_is_the_scenario_clock() {
        let mut scenario = Scenario::parse(SMOKE, &limits()).unwrap();

        scenario.t_scenario = 1.5;
        assert_eq!(scenario.sim_time_ns(), 1_500_000_000);
        scenario.t_scenario = 0.0;
        assert_eq!(scenario.sim_time_ns(), 0);
    }

    #[test]
    fn omitting_a_robot_declares_it_absent() {
        // Task-44: only robot 1 is listed, so robot 2 must not be in the world.
        // Before this, robot 2 kept its scene pose — visible, collidable and in
        // GT — and stole the auto-aim lock from the intended target.
        let text = "id = \"one\"\nduration = 5.0\ntrigger = \"on_load\"\n\n\
                    [[robot]]\nid = 1\nkeep_default = true\n";
        let scenario = Scenario::parse(text, &limits()).unwrap();

        assert_eq!(scenario.robots.len(), 1);
        assert_eq!(scenario.absent, vec![RobotId(2)]);
        assert!(scenario.absent.contains(&RobotId(2)));
    }

    #[test]
    fn listing_both_robots_leaves_none_absent() {
        let text = "id = \"both\"\nduration = 5.0\ntrigger = \"on_load\"\n\n\
                    [[robot]]\nid = 1\nkeep_default = true\n\n\
                    [[robot]]\nid = 2\nkeep_default = true\n";
        let scenario = Scenario::parse(text, &limits()).unwrap();

        assert_eq!(scenario.robots.len(), 2);
        assert!(scenario.absent.is_empty(), "{:?}", scenario.absent);
    }

    #[test]
    fn present_false_must_not_carry_spawn_or_segments() {
        let text = "id = \"conflict\"\nduration = 5.0\ntrigger = \"on_load\"\n\n\
                    [[robot]]\nid = 1\npresent = false\nkeep_default = true\n\n\
                    [[robot.segment]]\nt0 = 0.0\nt1 = 1.0\ntype = \"static\"\n";
        // `unwrap_err` needs `Scenario: Debug`; match instead so the test does not
        // force a `Debug` impl on the scenario graph.
        let Err(error) = Scenario::parse(text, &limits()) else {
            panic!("`present = false` with segments must be rejected");
        };

        assert!(error.contains("present = false"), "{error}");
        assert!(error.contains("conflicts"), "{error}");
    }

    #[test]
    fn present_false_entry_is_the_same_as_omitting_the_robot() {
        let text = "id = \"explicit\"\nduration = 5.0\ntrigger = \"on_load\"\n\n\
                    [[robot]]\nid = 1\nkeep_default = true\n\n\
                    [[robot]]\nid = 2\npresent = false\n";
        let scenario = Scenario::parse(text, &limits()).unwrap();

        assert_eq!(scenario.robots.len(), 1);
        assert_eq!(scenario.absent, vec![RobotId(2)]);
    }

    #[test]
    fn the_ego_can_be_scripted() {
        // P1 `ego_move`: listing robot 0 with segments hands the ego to the
        // scenario evaluator (the bench speed servo drives it; see
        // `the_ego_can_be_placed_on_a_corridor` for the optional placement).
        let text = "id = \"ego_line\"\nduration = 5.0\ntrigger = \"on_load\"\n\n\
                    [[robot]]\nid = 0\n\n\
                    [[robot.segment]]\nt0 = 0.0\nt1 = 5.0\ntype = \"static\"\n";
        let scenario = Scenario::parse(text, &limits()).unwrap();

        assert_eq!(scenario.robots.len(), 1);
        assert_eq!(scenario.robots[0].id, RobotId(0));
        // Listing only the ego means the two scene robots do not exist for the run
        // (task-44 semantics); the ego itself is never "absent" — it hosts the
        // camera and is always part of the world.
        assert_eq!(scenario.absent, vec![RobotId(1), RobotId(2)]);
    }

    #[test]
    fn the_ego_must_stay_present() {
        let text = "id = \"ego_absent\"\nduration = 5.0\ntrigger = \"on_load\"\n\n\
                    [[robot]]\nid = 0\npresent = false\n";
        let error = parse_err(text);
        assert!(error.contains("cannot be absent"), "{error}");
    }

    #[test]
    fn a_coast_keeps_the_segment_direction_as_its_split_plane() {
        // `cross_los` with `pass_through = false` stops at the leg midpoint, so
        // the instantaneous nominal velocity is zero for the rest of the leg.
        // Splitting the error against that zero vector (the old behaviour) put
        // the whole along-track ramp lag into the cross-track gate; the leg's own
        // direction is the correct split plane.
        let text = "id = \"coast\"\nduration = 10.0\ntrigger = \"on_load\"\n\n\
                    [[robot]]\nid = 1\nkeep_default = true\n\n\
                    [[robot.segment]]\nt0 = 0.0\nt1 = 4.0\ntype = \"cross_los\"\n\
                    v = 2.0\ndir_deg = 0.0\n";
        let scenario = Scenario::parse(text, &limits()).unwrap();
        let robot = &scenario.robots[0];
        let t = 3.5;
        let velocity = robot.nominal_velocity(t);
        assert_eq!(velocity, Vec3::ZERO, "the leg coasts after its midpoint");

        let reference = robot.reference_nominal_velocity(t);
        assert!(reference.length() > 1.0, "the leg direction survives");

        let direction = reference.normalize();
        let nominal = Vec3::ZERO;
        let actual = nominal + direction * 1.8; // purely along-track lag
        let (cross, along) = track_error(actual, nominal, reference);
        assert!(cross < 1e-4, "along-track lag must not reach the gate: {cross}");
        assert!((along - 1.8).abs() < 1e-4, "{along}");

        // …which is exactly what the old zero-split fallback got wrong.
        let (old_cross, _) = track_error(actual, nominal, velocity);
        assert!((old_cross - 1.8).abs() < 1e-4, "{old_cross}");

        // Static and spin legs have no direction at all, so they keep the old
        // `|delta|` fallback — that is why this change is inert for every
        // scenario that has no coast.
        let text = "id = \"static\"\nduration = 10.0\ntrigger = \"on_load\"\n\n\
                    [[robot]]\nid = 1\nkeep_default = true\n\n\
                    [[robot.segment]]\nt0 = 0.0\nt1 = 4.0\ntype = \"static\"\n";
        let scenario = Scenario::parse(text, &limits()).unwrap();
        let robot = &scenario.robots[0];
        assert_eq!(robot.reference_nominal_velocity(3.5), Vec3::ZERO);
        let (cross, along) = track_error(Vec3::new(0.0, 0.0, 1.8), Vec3::ZERO, Vec3::ZERO);
        assert!((cross - 1.8).abs() < 1e-4 && along == 0.0);
    }

    #[test]
    fn a_static_tail_keeps_the_last_legs_split_direction() {
        // A scenario that never moves must keep the old `|delta|` fallback.
        let mut remembered = Vec3::ZERO;
        assert_eq!(split_direction(Vec3::ZERO, &mut remembered), Vec3::ZERO);

        // A powered leg teaches the plane…
        let leg = Vec3::new(2.0, 0.0, 0.0);
        assert_eq!(split_direction(leg, &mut remembered), leg);

        // …and the `static` tail after it still splits by that leg, so the ramp
        // and the coast lag stay along-track instead of becoming a metre-scale
        // cross-track spike (task-59).
        let after = split_direction(Vec3::ZERO, &mut remembered);
        assert_eq!(after, leg);
        let direction = leg.normalize();
        let (cross, along) = track_error(direction * 2.086, Vec3::ZERO, after);
        assert!(cross < 1e-4, "the tail lag must stay along-track: {cross}");
        assert!((along - 2.086).abs() < 1e-4, "{along}");
    }

    #[test]
    fn a_robot_that_is_told_to_move_and_does_not_is_stalled() {
        // Commanded but still: stalled.
        assert!(is_stalled(2.0, 0.0));
        assert!(is_stalled(1.0, 0.2));
        // The honest ramp-up is not a stall (the servo crosses 25% quickly), and
        // neither is a robot that is simply not asked to move.
        assert!(!is_stalled(2.0, 1.5));
        assert!(!is_stalled(0.0, 0.0));
        assert!(!is_stalled(0.2, 0.0), "below the command threshold");
    }

    #[test]
    fn a_sine_segment_parses_with_a_constant_frequency() {
        let text = "id = \"sine\"\nduration = 10.0\ntrigger = \"on_load\"\n\n\
                    [[robot]]\nid = 1\nkeep_default = true\n\n\
                    [[robot.segment]]\nt0 = 1.0\nt1 = 5.0\ntype = \"sine\"\n\
                    v = 2.0\ndir_deg = 0.0\namp_m = 0.5\nfreq_hz = 0.75\n";
        let scenario = Scenario::parse(text, &limits()).unwrap();
        let primitive = scenario.robots[0].primitive_at(3.0);
        assert_eq!(primitive.kind(), "sine");
        match primitive {
            Primitive::Sine {
                v,
                dir_deg,
                amp_m,
                freq_start_hz,
                freq_end_hz,
                duration_s,
            } => {
                assert_eq!((v, dir_deg, amp_m), (2.0, 0.0, 0.5));
                assert_eq!(freq_start_hz, 0.75);
                assert_eq!(freq_end_hz, 0.75, "freq_hz is the constant shorthand");
                assert_eq!(duration_s, 4.0, "duration_s comes from the leg window");
            }
            other => panic!("expected a sine primitive: {other:?}"),
        }
    }

    #[test]
    fn a_sine_segment_parses_a_sweep_and_defaults_a_missing_end() {
        let sweep = "id = \"sine\"\nduration = 10.0\ntrigger = \"on_load\"\n\n\
                     [[robot]]\nid = 1\nkeep_default = true\n\n\
                     [[robot.segment]]\nt0 = 0.0\nt1 = 2.0\ntype = \"sine\"\n\
                     v = 1.0\ndir_deg = 90.0\namp_m = 1.0\n\
                     freq_start_hz = 0.5\nfreq_end_hz = 3.0\n";
        let scenario = Scenario::parse(sweep, &limits()).unwrap();
        match scenario.robots[0].primitive_at(1.0) {
            Primitive::Sine {
                freq_start_hz,
                freq_end_hz,
                ..
            } => assert_eq!((freq_start_hz, freq_end_hz), (0.5, 3.0)),
            other => panic!("expected a sine primitive: {other:?}"),
        }

        // One end only: the other defaults to it, which is still a legal sweep.
        let one_end = sweep.replace("freq_start_hz = 0.5\nfreq_end_hz = 3.0", "freq_end_hz = 2.0");
        let scenario = Scenario::parse(&one_end, &limits()).unwrap();
        match scenario.robots[0].primitive_at(1.0) {
            Primitive::Sine {
                freq_start_hz,
                freq_end_hz,
                ..
            } => assert_eq!((freq_start_hz, freq_end_hz), (2.0, 2.0)),
            other => panic!("expected a sine primitive: {other:?}"),
        }
    }

    #[test]
    fn a_sine_segment_without_any_frequency_is_rejected() {
        let text = "id = \"sine\"\nduration = 10.0\ntrigger = \"on_load\"\n\n\
                    [[robot]]\nid = 1\nkeep_default = true\n\n\
                    [[robot.segment]]\nt0 = 0.0\nt1 = 2.0\ntype = \"sine\"\n\
                    v = 1.0\ndir_deg = 0.0\namp_m = 1.0\n";
        let error = parse_err(text);
        assert!(error.contains("sine"), "{error}");
        assert!(error.contains("freq_hz"), "{error}");
        assert!(error.contains("freq_start_hz"), "{error}");
    }

    #[test]
    fn a_spin_segment_with_yaw_amp_parses_as_a_sweep() {
        let text = "id = \"spin-chirp\"\nduration = 12.0\ntrigger = \"on_load\"\n\n\
                    [[robot]]\nid = 1\nkeep_default = true\n\n\
                    [[robot.segment]]\nt0 = 2.0\nt1 = 8.0\ntype = \"spin\"\n\
                    yaw_amp_deg = 54.4\nfreq_start_hz = 0.2\nfreq_end_hz = 1.2\n";
        let scenario = Scenario::parse(text, &limits()).unwrap();
        let primitive = scenario.robots[0].primitive_at(3.0);
        assert_eq!(
            primitive.kind(),
            "spin",
            "the scene-file type of a sweep is still `spin`"
        );
        match primitive {
            Primitive::SpinSweep {
                yaw_amp_deg,
                freq_start_hz,
                freq_end_hz,
                duration_s,
            } => {
                assert_eq!(yaw_amp_deg, 54.4);
                assert_eq!((freq_start_hz, freq_end_hz), (0.2, 1.2));
                assert_eq!(duration_s, 6.0, "duration_s comes from the leg window");
            }
            other => panic!("expected a spin sweep: {other:?}"),
        }

        // A single frequency is the constant-frequency shorthand, same as `sine`.
        let one_end = text.replace("freq_start_hz = 0.2\nfreq_end_hz = 1.2", "freq_hz = 0.8");
        let scenario = Scenario::parse(&one_end, &limits()).unwrap();
        match scenario.robots[0].primitive_at(3.0) {
            Primitive::SpinSweep {
                freq_start_hz,
                freq_end_hz,
                ..
            } => assert_eq!((freq_start_hz, freq_end_hz), (0.8, 0.8)),
            other => panic!("expected a spin sweep: {other:?}"),
        }
    }

    #[test]
    fn a_spin_segment_without_yaw_amp_keeps_the_constant_rate() {
        let text = "id = \"spin\"\nduration = 10.0\ntrigger = \"on_load\"\n\n\
                    [[robot]]\nid = 1\nkeep_default = true\n\n\
                    [[robot.segment]]\nt0 = 1.0\nt1 = 3.0\ntype = \"spin\"\n\
                    omega = 6.0\n";
        let scenario = Scenario::parse(text, &limits()).unwrap();
        match scenario.robots[0].primitive_at(2.0) {
            Primitive::Spin { omega } => assert_eq!(omega, 6.0),
            other => panic!("expected a constant-rate spin: {other:?}"),
        }
    }

    #[test]
    fn a_spin_segment_with_both_omega_and_yaw_amp_is_rejected() {
        let text = "id = \"spin-clash\"\nduration = 10.0\ntrigger = \"on_load\"\n\n\
                    [[robot]]\nid = 1\nkeep_default = true\n\n\
                    [[robot.segment]]\nt0 = 0.0\nt1 = 4.0\ntype = \"spin\"\n\
                    omega = 3.0\nyaw_amp_deg = 20.0\nfreq_hz = 0.5\n";
        let error = parse_err(text);
        assert!(error.contains("omega"), "{error}");
        assert!(error.contains("yaw_amp_deg"), "{error}");
    }

    // ---------------------------------------------------------------------
    // task-67: scene-side static occluders.
    // ---------------------------------------------------------------------

    /// A scenario with one collider-only occluder box, `pos.z` = box bottom.
    const OCCLUDED: &str = r#"
id = "occluded"
seed = 7
duration = 12.0
trigger = "on_load"

[[robot]]
id = 1
keep_default = true

[[robot.segment]]
t0 = 0.0
t1 = 12.0
type = "static"

[[occluder]]
id = 3
pos = { x = 2.5, y = -1.0, z = 0.0543 }
size = { x = 0.40, y = 0.30, z = 0.60 }
yaw_deg = 30.0
"#;

    /// Directories a shipped-scenario sweep may find the case files in, most
    /// specific first. The container `check`/`test` mounts only the simulator
    /// repo, so the sweep is skipped there rather than failing.
    fn scenario_dirs() -> Vec<std::path::PathBuf> {
        let mut dirs = Vec::new();
        if let Ok(dir) = std::env::var("DAEDALUS_SCENARIOS_DIR") {
            dirs.push(std::path::PathBuf::from(dir));
        }
        dirs.push(std::path::PathBuf::from("/bench_scenarios"));
        dirs.push(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../rm_auto_aim_ws/tools/benchmark/scenarios"),
        );
        dirs.push(std::path::PathBuf::from(
            "/home/crp6/workspace/RM/rm_auto_aim_ws/tools/benchmark/scenarios",
        ));
        dirs
    }

    #[test]
    fn an_occluder_box_maps_the_ros_frame_into_bevy() {
        let scenario = Scenario::parse(OCCLUDED, &limits()).expect("occluded scenario parses");
        assert_eq!(scenario.occluders.len(), 1);
        let spec = scenario.occluders[0];
        // `visible` is the one thing that decides whether the box is drawn.
        assert!(!spec.visible);
        assert_eq!(spec.resolved_id(0), 3);
        assert!(
            !scenario.occluders_installed,
            "a freshly parsed scenario owns no entities yet"
        );

        let (centre, rotation, extent) = spec.to_bevy();
        // ROS -> Bevy is `bevy = (-ros_y, ros_z, -ros_x)`; `pos.z` is the bottom,
        // so the Bevy centre carries another half-extent of height.
        assert!(
            (centre - Vec3::new(1.0, 0.3543, -2.5)).length() < 1e-5,
            "{centre:?}"
        );
        // Bevy extents follow the axis remap: x = ros_y, y = ros_z, z = ros_x.
        assert!((extent - Vec3::new(0.30, 0.60, 0.40)).length() < 1e-5, "{extent:?}");
        let expected = Quat::from_rotation_y(30.0_f32.to_radians());
        assert!((rotation.dot(expected).abs() - 1.0).abs() < 1e-6, "{rotation:?}");
    }

    #[test]
    fn occluder_tables_are_optional_and_start_empty() {
        let scenario = Scenario::parse(SMOKE, &limits()).expect("smoke scenario parses");
        assert!(scenario.occluders.is_empty());
        assert!(!scenario.occluders_installed);
    }

    #[test]
    fn a_visible_occluder_keeps_the_same_geometry() {
        let text = OCCLUDED.replace("yaw_deg = 30.0", "yaw_deg = 30.0\nvisible = true");
        let scenario = Scenario::parse(&text, &limits()).expect("visible occluder parses");
        assert!(scenario.occluders[0].visible);
        let (centre, _, extent) = scenario.occluders[0].to_bevy();
        assert!((centre.y - 0.3543).abs() < 1e-5, "{centre:?}");
        assert!((extent.y - 0.60).abs() < 1e-5, "{extent:?}");
    }

    #[test]
    fn a_bad_occluder_fails_the_load_instead_of_being_skipped() {
        for (label, text) in [
            (
                "zero height",
                OCCLUDED.replace("z = 0.60 }", "z = 0.0 }"),
            ),
            (
                "negative width",
                OCCLUDED.replace("y = 0.30, z", "y = -0.30, z"),
            ),
            (
                "absurd size",
                OCCLUDED.replace("x = 0.40, y", "x = 400.0, y"),
            ),
            (
                "non-finite position",
                OCCLUDED.replace("x = 2.5, y", "x = inf, y"),
            ),
            (
                "duplicate id",
                format!("{OCCLUDED}\n[[occluder]]\nid = 3\npos = {{ x = 0.0, y = 0.0, z = 0.0 }}\nsize = {{ x = 0.1, y = 0.1, z = 0.1 }}\n"),
            ),
        ] {
            let error = parse_err(&text);
            assert!(
                error.contains("occluder"),
                "{label}: the error must name the occluder table, got: {error}"
            );
        }
    }

    #[test]
    fn shipped_scenarios_declare_no_occluders() {
        let Some(dir) = scenario_dirs().into_iter().find(|dir| dir.is_dir()) else {
            println!(
                "[bench] no shipped scenario directory visible from this build; \
                 occluder laziness sweep skipped"
            );
            return;
        };
        let mut checked = 0usize;
        let mut entries: Vec<std::path::PathBuf> = std::fs::read_dir(&dir)
            .expect("scenario directory is readable")
            .map(|entry| entry.expect("directory entry").path())
            .collect();
        entries.sort();
        for path in entries {
            if path.extension().and_then(|ext| ext.to_str()) != Some("toml") {
                continue;
            }
            let text = std::fs::read_to_string(&path).expect("scenario file is readable");
            let scenario = match Scenario::parse(&text, &limits()) {
                Ok(scenario) => scenario,
                Err(error) => panic!("shipped scenario {} does not parse: {error}", path.display()),
            };
            assert!(
                scenario.occluders.is_empty(),
                "shipped scenario {} declares an occluder: the task-67 laziness claim (zero \
                 occluder tables in the fleet => zero new entities) no longer holds",
                path.display()
            );
            checked += 1;
        }
        assert!(checked > 0, "no .toml scenario found in {}", dir.display());
        println!("[bench] occluder laziness sweep: {checked} shipped scenarios, none declare an occluder");
    }
}
