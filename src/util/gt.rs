//! Shared ground-truth sampling.
//!
//! One place that answers "where is this robot, and which way is it facing, in
//! the frame the rest of the stack uses". Both consumers read from here:
//!
//! - the Talos shared-memory ground truth (`src/talos/ground_truth.rs`), and
//! - the benchmark's ROS 2 topics (`src/bench/gt.rs`).
//!
//! Keeping it in one module is not tidiness: if the two disagreed, a benchmark
//! number and a Talos number about the same run would be quietly different, and
//! the `/tf`-vs-GT cross-check could pass against a self-consistent but wrong
//! convention.

use bevy::prelude::*;

use crate::robomaster::prelude::{
    ArmorType, ENGINEER_ROBOT_CONFIG, HERO_ROBOT_CONFIG, INFANTRY_FOUR_CONFIG,
    INFANTRY_THREE_CONFIG, RobotConfig, Team,
};

/// Bevy (Y-up) → ROS (Z-up) alignment, `ros = (-z, -x, y)`.
///
/// This is the same matrix `/tf` applies in
/// [`crate::ros2::prelude::transform`], and the two *must* stay equal: the
/// benchmark cross-checks ground truth against `/tf` frame by frame at
/// millimetre scale.
pub const ALIGN: Mat3 = Mat3::from_cols(
    Vec3::new(0.0, -1.0, 0.0),
    Vec3::new(0.0, 0.0, 1.0),
    Vec3::new(-1.0, 0.0, 0.0),
);

/// Align a Bevy vector into the map (ROS) frame.
#[inline]
pub fn to_ros_vec3(vector: Vec3) -> Vec3 {
    ALIGN * vector
}

/// Align a Bevy rotation into the map (ROS) frame.
#[inline]
pub fn to_ros_rotation(rotation: Quat) -> Quat {
    let align = Quat::from_mat3(&ALIGN);
    align * rotation * align.inverse()
}

/// `float32[3]` on the wire. r2r binds *both* fixed-size and variable-length
/// float arrays to `Vec<f32>`, so every `[3]` field in the new `Sim*` messages
/// goes through this one conversion.
#[inline]
pub fn ros_vec3(vector: Vec3) -> Vec<f32> {
    vec![vector.x, vector.y, vector.z]
}

/// Yaw about the map frame's +Z axis, in radians.
///
/// `to_euler(EulerRot::ZYX)` returns its angles **in the enum's axis order**,
/// i.e. `(z, y, x)` = `(yaw, pitch, roll)`, so the Z angle is the *first*
/// element. The pre-existing copy of this function in
/// `talos/ground_truth.rs` read the third element (the X/roll angle), which is
/// why the Talos ground truth's `yaw` was ~0 for any robot on flat ground. The
/// unit test below pins the convention with a pure Z rotation.
#[inline]
pub fn ros_yaw(rotation: Quat) -> f32 {
    let (yaw, _, _) = to_ros_rotation(rotation).to_euler(EulerRot::ZYX);
    yaw
}

/// `SimRobotState.team` / `GroundTruthTarget.team`: 0 = red, 1 = blue.
pub const fn team_u8(team: Team) -> u8 {
    match team {
        Team::Red => 0,
        Team::Blue => 1,
    }
}

/// `SimRobotState.robot_type`: 0 = infantry3, 1 = infantry4, 2 = hero,
/// 3 = sentry, 4 = engineer.
///
/// The scene carries a `RobotConfig`, not a type tag, so the mapping goes
/// through the only thing that distinguishes the configs: their armour spec.
/// `ArmorLabel::Sentry` is the legacy engineer label (see
/// `robomaster::common`), which is why it maps to 4 rather than 3; nothing in
/// P0 spawns an actual sentry.
pub fn robot_type_u8(config: RobotConfig) -> u8 {
    if config == HERO_ROBOT_CONFIG {
        2
    } else if config == INFANTRY_THREE_CONFIG {
        0
    } else if config == INFANTRY_FOUR_CONFIG {
        1
    } else if config == ENGINEER_ROBOT_CONFIG {
        4
    } else if config.armor.armor_type() == ArmorType::Large {
        2
    } else {
        3
    }
}

/// Plate width/height in meters, measured from the shipped models' armour
/// meshes (not from the RM rule book):
///
/// - `vehicle.glb` (small armour): 0.1349 m wide, 0.1249 m high,
/// - `HERO.glb` (large armour): 0.2300 m wide, 0.1272 m high.
///
/// `SimArmorState.plate_w/plate_h` report these so a scorer can ask "is this
/// impact inside the plate" without knowing which model a robot is.
pub const fn plate_size(armor_type: ArmorType) -> (f32, f32) {
    match armor_type {
        ArmorType::Small => (0.1349, 0.1249),
        ArmorType::Large => (0.2300, 0.1272),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn alignment_matches_the_tf_remap() {
        // ros = (-z, -x, y)
        assert!((to_ros_vec3(Vec3::new(1.0, 0.0, 0.0)) - Vec3::new(0.0, -1.0, 0.0)).length() < 1e-6);
        assert!((to_ros_vec3(Vec3::new(0.0, 1.0, 0.0)) - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-6);
        assert!((to_ros_vec3(Vec3::new(0.0, 0.0, 1.0)) - Vec3::new(-1.0, 0.0, 0.0)).length() < 1e-6);

        // The v1 player pose: Bevy (0.1433, 0.4199, 3.1117) must land at map
        // (-3.1117, -0.1433, 0.4199).
        let mapped = to_ros_vec3(Vec3::new(0.1433, 0.4199, 3.1117));
        assert!((mapped - Vec3::new(-3.1117, -0.1433, 0.4199)).length() < 1e-5, "{mapped:?}");
    }

    #[test]
    fn yaw_is_the_heading_about_map_z() {
        // A Bevy yaw of +90 deg about +Y is a ROS yaw of +90 deg about +Z.
        let yaw = ros_yaw(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2));
        assert!((yaw - std::f32::consts::FRAC_PI_2).abs() < 1e-4, "{yaw}");
        assert!(ros_yaw(Quat::IDENTITY).abs() < 1e-6);
    }

    #[test]
    fn robot_types_are_distinguishable_per_config() {
        assert_eq!(robot_type_u8(INFANTRY_THREE_CONFIG), 0);
        assert_eq!(robot_type_u8(INFANTRY_FOUR_CONFIG), 1);
        assert_eq!(robot_type_u8(HERO_ROBOT_CONFIG), 2);
        assert_eq!(robot_type_u8(ENGINEER_ROBOT_CONFIG), 4);
    }

    #[test]
    fn plate_sizes_are_the_measured_model_sizes() {
        assert_eq!(plate_size(ArmorType::Small), (0.1349, 0.1249));
        assert_eq!(plate_size(ArmorType::Large), (0.2300, 0.1272));
    }
}
