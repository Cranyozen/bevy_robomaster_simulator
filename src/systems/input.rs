use bevy::prelude::*;
use std::collections::HashMap;
use std::sync::atomic::Ordering;

use crate::components::{
    ActiveSlapper, CameraMode, Controlled, FollowingType, Infantry, InfantryChassis, InfantryGimbal,
    MotionSource, RobotId, ScenarioAbsent, SlapperInfantry, SubscribeAutoAim,
};
use crate::config::SimulationConfig;
use crate::robomaster::vehicle::movement::VehicleDynamic;
use crate::systems::{ControllerInput, ControllerState};
use avian3d::prelude::*;

const CHASSIS_ROTATION_STOP_EPSILON: f32 = 1e-3;
const CHASSIS_TILT_LIMIT: f32 = 20.0 * std::f32::consts::PI / 180.0;

/// Per-robot chassis/gimbal input, keyed by [`RobotId`].
///
/// The interactive simulator only ever had two input slots
/// (`ControllerState::controlled` and `::remote`) and handed them to two
/// hardcoded *single* vehicles. A benchmark that drives several robots at once
/// needs the mapping to be explicit, so input is routed here once per frame and
/// the vehicle system reads it back by id. Two writers, one each:
///
/// - [`route_robot_inputs`] writes the keyboard/gamepad slots (`MotionSource`
///   `Gamepad`),
/// - the benchmark scenario evaluator (`src/bench/scenario.rs`) writes the
///   `Scripted` robots.
///
/// An id with no entry is a robot nothing drives this frame — which is exactly
/// what "no input authority" meant before this table existed.
#[derive(Resource, Default)]
pub struct RobotInputs {
    inputs: HashMap<RobotId, ControllerInput>,
}

impl RobotInputs {
    pub fn clear(&mut self) {
        self.inputs.clear();
    }

    pub fn set(&mut self, id: RobotId, input: ControllerInput) {
        self.inputs.insert(id, input);
    }

    pub fn get(&self, id: RobotId) -> Option<ControllerInput> {
        self.inputs.get(&id).copied()
    }

    pub fn remove(&mut self, id: RobotId) -> Option<ControllerInput> {
        self.inputs.remove(&id)
    }

    pub fn len(&self) -> usize {
        self.inputs.len()
    }

    pub fn is_empty(&self) -> bool {
        self.inputs.is_empty()
    }
}

fn update_chassis_rotation(
    chassis_transform: &mut Transform,
    chassis_data: &mut InfantryChassis,
    yaw_input: f32,
    roll_input: f32,
    pitch_input: f32,
    yaw_rotation_speed: f32,
    yaw_acceleration: f32,
    tilt_rotation_speed: f32,
    dt: f32,
) {
    let target_yaw_velocity = yaw_input * yaw_rotation_speed;
    let max_velocity_delta = yaw_acceleration * dt;
    chassis_data.yaw_velocity = move_towards(
        chassis_data.yaw_velocity,
        target_yaw_velocity,
        max_velocity_delta,
    );

    if chassis_data.yaw_velocity.abs() < CHASSIS_ROTATION_STOP_EPSILON
        && target_yaw_velocity.abs() < CHASSIS_ROTATION_STOP_EPSILON
    {
        chassis_data.yaw_velocity = 0.0;
    }

    chassis_data.yaw += chassis_data.yaw_velocity * dt;
    chassis_data.roll = (chassis_data.roll + roll_input * tilt_rotation_speed * dt)
        .clamp(-CHASSIS_TILT_LIMIT, CHASSIS_TILT_LIMIT);
    chassis_data.pitch = (chassis_data.pitch + pitch_input * tilt_rotation_speed * dt)
        .clamp(-CHASSIS_TILT_LIMIT, CHASSIS_TILT_LIMIT);
    chassis_transform.rotation = Quat::from_euler(
        EulerRot::YXZ,
        chassis_data.yaw,
        chassis_data.pitch,
        chassis_data.roll,
    );
}

fn move_towards(current: f32, target: f32, max_delta: f32) -> f32 {
    current + (target - current).clamp(-max_delta, max_delta)
}

/// Route the keyboard/gamepad slots into the per-robot table.
///
/// This is the whole of the old `ControllerState::controlled` /
/// `ControllerState::remote` split, kept in one pure mapping so the "same
/// controls, same robot" property can be unit-tested:
///
/// - `Controlled`    -> the primary keyboard/gamepad input (WASD + arrows),
/// - `ActiveSlapper` -> the IJKL/d-pad "remote" input (the *second* vehicle the
///   keyboard can drive),
/// - neither         -> no input authority at all. Such a robot is a prop as far
///   as input is concerned, exactly as before: an id with no table entry is not
///   touched by [`robot_vehicle_controls`].
///
/// `Scripted` robots are left out on purpose — the benchmark scenario evaluator
/// owns those.
pub fn route_robot_inputs(
    mut robot_inputs: ResMut<RobotInputs>,
    controller: Res<ControllerState>,
    robots: Query<
        (
            &RobotId,
            Option<&MotionSource>,
            Option<&Controlled>,
            Option<&ActiveSlapper>,
        ),
        (With<Infantry>, Without<ScenarioAbsent>),
    >,
) {
    robot_inputs.clear();
    for (id, source, controlled, active_slapper) in &robots {
        if source.is_some_and(|source| source.is_scripted()) {
            continue;
        }
        if let Some(input) = gamepad_input_for(
            controlled.is_some(),
            active_slapper.is_some(),
            &controller,
        ) {
            robot_inputs.set(*id, input);
        }
    }
}

/// Which of the two hand-driven slots a robot gets, if any.
fn gamepad_input_for(
    controlled: bool,
    active_slapper: bool,
    controller: &ControllerState,
) -> Option<ControllerInput> {
    if controlled {
        Some(controller.controlled)
    } else if active_slapper {
        Some(controller.remote)
    } else {
        None
    }
}

/// Drives every robot from the per-robot input table.
///
/// This replaces the pair of `Single<..>` systems (`vehicle_controls` and
/// `remote_vehicle_controls`) that could only ever drive one vehicle each. The
/// physics is unchanged — the same [`VehicleDynamic::linear`] and the same
/// [`update_chassis_rotation`] — and so is the choice of movement frame:
///
/// - the ego (`Controlled`) moves relative to its **gimbal** (the auto-aim view
///   direction), which is what `vehicle_controls` always did,
/// - every other robot moves relative to its own **body**,
/// - a **scripted** ego (`MotionSource::Scripted`, P1 `ego_move`) moves relative
///   to its **body** as well: a scenario leg says "cross the line at 338°", and
///   in the gimbal frame that direction would turn with the turret while the
///   auto-aim tracks a target. A hand-driven ego never has a `MotionSource`, so
///   the interactive behaviour is untouched.
/// - the ego is released while the free camera is active, which used to be a
///   `run_if` on `vehicle_controls` and is now a per-robot skip so the other
///   vehicles keep being driven.
///
/// A robot whose id is absent from [`RobotInputs`] is skipped entirely — not
/// driven with zero input — so an untouched robot's `Transform` is never
/// rewritten by the Euler reconstruction inside `update_chassis_rotation`.
pub fn robot_vehicle_controls(
    time: Res<Time>,
    config: Res<SimulationConfig>,
    robot_inputs: Res<RobotInputs>,
    camera_mode: Res<CameraMode>,
    mut robots: Query<
        (
            Entity,
            &RobotId,
            Forces,
            &Mass,
            &mut VehicleDynamic,
            &GlobalTransform,
            Option<&Controlled>,
            Option<&MotionSource>,
        ),
        (With<Infantry>, Without<ScenarioAbsent>),
    >,
    gimbals: Query<&GlobalTransform, With<InfantryGimbal>>,
    mut chassis: Query<(&mut Transform, &mut InfantryChassis)>,
    children: Query<&Children>,
) {
    let free_camera = camera_mode.0 == FollowingType::Free;
    let dt = time.delta_secs();

    for (root, id, mut forces, &Mass(mass), mut dynamic, root_transform, controlled, source) in
        &mut robots
    {
        let Some(input) = robot_inputs.get(*id) else {
            continue;
        };
        let is_ego = controlled.is_some();
        let is_scripted = source.is_some_and(|source| source.is_scripted());
        // The free camera releases the *player's* vehicle; a scripted ego has no
        // player, so the scenario keeps owning it.
        if is_ego && free_camera && !is_scripted {
            continue;
        }

        let movement_frame = if is_ego && !is_scripted {
            first_descendant_with(root, &children, |entity| gimbals.contains(entity))
                .and_then(|entity| gimbals.get(entity).ok())
                .unwrap_or(root_transform)
        } else {
            root_transform
        };

        let boost = input.boost_multiplier();
        dynamic.linear(
            &mut forces,
            mass,
            movement_frame,
            input.movement,
            dt,
            boost,
        );

        let Some(chassis_entity) =
            first_descendant_with(root, &children, |entity| chassis.contains(entity))
        else {
            continue;
        };
        let Ok((mut chassis_transform, mut chassis_data)) = chassis.get_mut(chassis_entity) else {
            continue;
        };
        update_chassis_rotation(
            &mut chassis_transform,
            &mut chassis_data,
            input.chassis_yaw,
            input.chassis_roll,
            input.chassis_pitch,
            config.vehicle.rotation_speed,
            config.vehicle.yaw_acceleration,
            config.vehicle.tilt_rotation_speed,
            dt,
        );
    }
}

/// First descendant of `root` (excluding `root` itself) accepted by `matches`.
fn first_descendant_with(
    root: Entity,
    children: &Query<&Children>,
    matches: impl Fn(Entity) -> bool,
) -> Option<Entity> {
    children.iter_descendants(root).find(|entity| matches(*entity))
}

pub fn gimbal_controls(
    time: Res<Time>,
    controller: Res<ControllerState>,
    enabled: Res<SubscribeAutoAim>,
    config: Res<SimulationConfig>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal),
        (With<Controlled>, Without<InfantryChassis>),
    >,
) {
    if enabled.load(Ordering::Acquire) {
        return;
    }

    let dt = time.delta_secs();
    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();

    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);

    let controller = controller.controlled;
    let rotation_speed = config.vehicle.gimbal_rotation_speed * controller.gimbal_scale() * dt;
    gimbal_data.local_yaw += controller.gimbal.x * rotation_speed;
    gimbal_data.pitch += controller.gimbal.y * rotation_speed;

    gimbal_data.pitch = gimbal_data.pitch.clamp(
        -config.vehicle.gimbal_pitch_limit,
        config.vehicle.gimbal_pitch_limit,
    );

    let gimbal_rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);

    gimbal_transform.rotation = gimbal_rotation;
}

pub fn remote_gimbal_controls(
    time: Res<Time>,
    controller: Res<ControllerState>,
    config: Res<SimulationConfig>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal),
        (With<ActiveSlapper>, Without<InfantryChassis>),
    >,
) {
    let dt = time.delta_secs();
    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();

    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);

    let controller = controller.remote;
    let rotation_speed = config.vehicle.gimbal_rotation_speed * controller.gimbal_scale() * dt;
    gimbal_data.local_yaw += controller.gimbal.x * rotation_speed;
    gimbal_data.pitch += controller.gimbal.y * rotation_speed;
    gimbal_data.pitch = gimbal_data.pitch.clamp(
        -config.vehicle.gimbal_pitch_limit,
        config.vehicle.gimbal_pitch_limit,
    );

    let gimbal_rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);

    gimbal_transform.rotation = gimbal_rotation;
}

pub fn switch_slapper_control(
    mut commands: Commands,
    controller: Res<ControllerState>,
    children: Query<&Children>,
    slapper_roots: Query<Entity, (With<Infantry>, With<SlapperInfantry>)>,
    active_root: Query<Entity, (With<Infantry>, With<SlapperInfantry>, With<ActiveSlapper>)>,
) {
    if !controller.controlled.switch_slapper_just_pressed {
        return;
    }

    let roots: Vec<Entity> = slapper_roots.iter().collect();
    if roots.len() <= 1 {
        return;
    }

    let current = active_root.single().ok();
    let current_idx = current.and_then(|e| roots.iter().position(|&r| r == e));
    let next_idx = match current_idx {
        Some(idx) => (idx + 1) % roots.len(),
        None => 0,
    };

    // Remove ActiveSlapper from current
    if let Some(current_root) = current {
        commands.entity(current_root).remove::<ActiveSlapper>();
        for descendant in children.iter_descendants(current_root) {
            commands.entity(descendant).remove::<ActiveSlapper>();
        }
    }

    // Add ActiveSlapper to next
    let next_root = roots[next_idx];
    commands.entity(next_root).insert(ActiveSlapper);
    for descendant in children.iter_descendants(next_root) {
        commands.entity(descendant).insert(ActiveSlapper);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gamepad_routing_matches_the_old_two_slots() {
        let mut controller = ControllerState::default();
        controller.controlled.movement = Vec2::new(1.0, 0.0);
        controller.remote.movement = Vec2::new(-1.0, 0.0);

        // The ego gets what WASD/left-stick produce.
        assert_eq!(
            gamepad_input_for(true, false, &controller).unwrap().movement,
            Vec2::new(1.0, 0.0)
        );
        // The remote vehicle gets what IJKL/d-pad produce.
        assert_eq!(
            gamepad_input_for(false, true, &controller).unwrap().movement,
            Vec2::new(-1.0, 0.0)
        );
        // Both markers: the primary slot wins, and the old
        // `remote_vehicle_controls` excluded `Controlled` anyway, so such a robot
        // could never be driven twice — the new routing keeps that.
        assert_eq!(
            gamepad_input_for(true, true, &controller).unwrap().movement,
            Vec2::new(1.0, 0.0)
        );
        // Neither: no input authority. The old systems never touched such a robot.
        assert!(gamepad_input_for(false, false, &controller).is_none());
    }

    #[test]
    fn robot_inputs_table_is_keyed_by_robot_id() {
        let mut inputs = RobotInputs::default();
        inputs.set(
            RobotId(2),
            ControllerInput {
                chassis_yaw: 1.0,
                ..Default::default()
            },
        );

        assert_eq!(inputs.get(RobotId(2)).unwrap().chassis_yaw, 1.0);
        assert!(
            inputs.get(RobotId(1)).is_none(),
            "an absent id means \"nothing drives this robot\""
        );

        inputs.clear();
        assert!(inputs.is_empty());
    }

    #[test]
    fn chassis_rotation_smoothly_ramps_towards_target_speed() {
        let mut transform = Transform::default();
        let mut chassis = InfantryChassis::default();

        update_chassis_rotation(
            &mut transform,
            &mut chassis,
            1.0,
            0.0,
            0.0,
            9.42,
            60.0,
            2.0,
            0.016,
        );

        assert!(chassis.yaw_velocity > 0.0);
        assert!(chassis.yaw_velocity < 9.42);
        assert!(chassis.yaw > 0.0);
    }

    #[test]
    fn chassis_rotation_uses_independent_yaw_and_tilt_speeds() {
        let mut transform = Transform::default();
        let mut chassis = InfantryChassis::default();

        update_chassis_rotation(
            &mut transform,
            &mut chassis,
            1.0,
            1.0,
            -1.0,
            8.0,
            1.0,
            0.25,
            1.0,
        );

        assert_eq!(chassis.yaw_velocity, 1.0);
        assert_eq!(chassis.roll, 0.25);
        assert_eq!(chassis.pitch, -0.25);
    }

    #[test]
    fn chassis_rotation_smoothly_brakes_to_stop() {
        let mut transform = Transform::default();
        let mut chassis = InfantryChassis {
            yaw: 0.0,
            yaw_velocity: 9.42,
            ..default()
        };

        for _ in 0..60 {
            update_chassis_rotation(
                &mut transform,
                &mut chassis,
                0.0,
                0.0,
                0.0,
                9.42,
                60.0,
                2.0,
                0.016,
            );
        }

        assert!(chassis.yaw_velocity.abs() < 1e-2);
    }

    #[test]
    fn chassis_rotation_bounds_roll_and_pitch_as_swing_angles() {
        let mut transform = Transform::default();
        let mut chassis = InfantryChassis::default();

        update_chassis_rotation(
            &mut transform,
            &mut chassis,
            0.0,
            1.0,
            -1.0,
            2.0,
            60.0,
            2.0,
            10.0,
        );
        assert_eq!(chassis.roll, CHASSIS_TILT_LIMIT);
        assert_eq!(chassis.pitch, -CHASSIS_TILT_LIMIT);

        update_chassis_rotation(
            &mut transform,
            &mut chassis,
            1.0,
            -1.0,
            1.0,
            2.0,
            60.0,
            2.0,
            10.0,
        );

        assert_eq!(chassis.roll, -CHASSIS_TILT_LIMIT);
        assert_eq!(chassis.pitch, CHASSIS_TILT_LIMIT);
        let (_, pitch, roll) = transform.rotation.to_euler(EulerRot::YXZ);
        assert!((roll + CHASSIS_TILT_LIMIT).abs() < 1e-5);
        assert!((pitch - CHASSIS_TILT_LIMIT).abs() < 1e-5);
    }
}
