use bevy::prelude::*;

use crate::robomaster::prelude::{RobotConfig, Team};

#[derive(Component)]
pub struct Controlled;

#[derive(Component)]
pub struct Infantry {
    pub team: Team,
    pub config: RobotConfig,
}

impl Infantry {
    pub const fn new(team: Team, config: RobotConfig) -> Self {
        Self { team, config }
    }
}

#[derive(Component, Default)]
pub struct InfantryChassis {
    pub yaw: f32,
    pub yaw_velocity: f32,
    pub roll: f32,
    pub pitch: f32,
}

#[derive(Component, Default)]
pub struct InfantryGimbal {
    pub local_yaw: f32,
    pub pitch: f32,
}

#[derive(Component)]
pub struct InfantryViewOffset;

#[derive(Component)]
pub struct InfantryLaunchOffset;

#[derive(Component)]
pub struct SlapperInfantry;

/// Marker for the currently active (controlled) SlapperInfantry
#[derive(Component)]
pub struct ActiveSlapper;

/// Stable identity of a robot in the scene.
///
/// `0` is the ego (the `Controlled` robot the auto-aim stack aims from), `1..n`
/// are targets. Everything that needs to say "which vehicle" — scripted motion,
/// ground-truth ownership, hit attribution — keys on this instead of on entity
/// order or `ArmorId`: `ArmorId` is a construction-order counter
/// (`armor/construct.rs`), so it can never answer "whose plate is this".
///
/// It lives on the robot *root* (the entity that carries `Infantry` and
/// `VehicleDynamic`); descendants find their robot by walking `ChildOf` up.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct RobotId(pub u8);

/// Where a robot's chassis/gimbal input comes from.
///
/// `Gamepad` is the default and means "exactly what the interactive simulator
/// always did": the keyboard/gamepad bindings. `Scripted` means the benchmark
/// scenario evaluator (`src/bench/`) owns this robot for the run. The distinction
/// exists so the two drivers can never fight over the same vehicle.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MotionSource(pub MotionDrive);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MotionDrive {
    #[default]
    Gamepad,
    Scripted,
}

impl MotionSource {
    pub const GAMEPAD: Self = Self(MotionDrive::Gamepad);
    pub const SCRIPTED: Self = Self(MotionDrive::Scripted);

    pub const fn is_scripted(self) -> bool {
        matches!(self.0, MotionDrive::Scripted)
    }
}

impl Default for RobotId {
    fn default() -> Self {
        Self(0)
    }
}

/// A robot the loaded scenario declares **absent from the world**.
///
/// P1 task-44: a scenario that does not list a robot (or lists it with
/// `present = false`) means that vehicle does not exist for the run, instead of
/// silently sitting at its scene pose. While the scenario is loaded the marker
/// sits on the robot root *and* on every descendant, together with
/// `Visibility::Hidden` (camera), `RigidBodyDisabled` (simulation) and
/// `ColliderDisabled` on the collider descendants (physics), so a single
/// `Without<ScenarioAbsent>` filter excludes the whole vehicle from GT, `/tf`,
/// input routing and collisions. Removing the markers restores the robot.
#[derive(Component, Clone, Copy, Debug)]
pub struct ScenarioAbsent;
