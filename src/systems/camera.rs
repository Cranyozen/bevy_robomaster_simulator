use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use std::f32::consts::PI;

use crate::components::{
    ActiveSlapper, CameraFocus, CameraMode, Controlled, FollowingType, Infantry, InfantryGimbal,
    InfantryLaunchOffset, InfantryViewOffset, MainCamera,
};
use crate::config::SimulationConfig;
use crate::systems::ControllerState;

pub fn following_controls(mut mode: ResMut<CameraMode>, controller: Res<ControllerState>) {
    if controller.controlled.switch_camera_just_pressed {
        mode.0 = match mode.0 {
            FollowingType::Free => FollowingType::Robot,
            FollowingType::Robot => FollowingType::ThirdPerson,
            FollowingType::ThirdPerson => FollowingType::Free,
        };
    }
}

pub fn focus_controls(controller: Res<ControllerState>, mut focus: ResMut<CameraFocus>) {
    if controller.controlled.switch_focus_just_pressed {
        focus.toggle();
    }
}

/// Frames the camera on one robot. `infantry`, `gimbal`, `view_offset` and
/// `launch_offset` all belong to the same robot hierarchy.
fn apply_follow(
    camera: &mut Transform,
    follow_offset: Vec3,
    mode: FollowingType,
    infantry: &Transform,
    gimbal: &Transform,
    view_offset: &Transform,
    launch_offset: &Transform,
) {
    match mode {
        FollowingType::Robot => {
            let gimbal_world_rotation = infantry.rotation * gimbal.rotation;
            let view_offset_world = gimbal_world_rotation * view_offset.translation;

            camera.translation = infantry.translation + view_offset_world;
            camera.rotation = (gimbal_world_rotation
                * launch_offset.rotation
                * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, PI / 2.0))
            .normalize()
        }
        FollowingType::ThirdPerson => {
            let offset = infantry.rotation * follow_offset;
            camera.translation = infantry.translation + offset;
            camera.look_at(infantry.translation, Vec3::Y);
        }
        FollowingType::Free => {}
    }
}

pub fn update_camera_follow(
    camera_query: Single<(&mut Transform, &MainCamera), Without<Controlled>>,
    infantry: Single<&Transform, (With<Infantry>, With<Controlled>)>,
    gimbal: Single<&Transform, (With<Controlled>, With<InfantryGimbal>)>,
    view_offset: Single<&Transform, (With<Controlled>, With<InfantryViewOffset>)>,
    launch_offset: Single<&Transform, (With<Controlled>, With<InfantryLaunchOffset>)>,
    mode: Res<CameraMode>,
) {
    let (mut camera, main_camera) = camera_query.into_inner();
    apply_follow(
        &mut camera,
        main_camera.follow_offset,
        mode.0,
        infantry.into_inner(),
        gimbal.into_inner(),
        view_offset.into_inner(),
        launch_offset.into_inner(),
    );
}

/// Follows the active slapper. Queries instead of `Single` so that a scene
/// without an active slapper (or one that is still loading) keeps the camera
/// where it is instead of panicking.
pub fn update_camera_follow_slapper(
    camera_query: Single<(&mut Transform, &MainCamera), Without<ActiveSlapper>>,
    infantry: Query<&Transform, (With<Infantry>, With<ActiveSlapper>)>,
    gimbal: Query<&Transform, (With<ActiveSlapper>, With<InfantryGimbal>)>,
    view_offset: Query<&Transform, (With<ActiveSlapper>, With<InfantryViewOffset>)>,
    launch_offset: Query<&Transform, (With<ActiveSlapper>, With<InfantryLaunchOffset>)>,
    mode: Res<CameraMode>,
) {
    let (Ok(infantry), Ok(gimbal), Ok(view_offset), Ok(launch_offset)) = (
        infantry.single(),
        gimbal.single(),
        view_offset.single(),
        launch_offset.single(),
    ) else {
        return;
    };

    let (mut camera, main_camera) = camera_query.into_inner();
    apply_follow(
        &mut camera,
        main_camera.follow_offset,
        mode.0,
        infantry,
        gimbal,
        view_offset,
        launch_offset,
    );
}

pub fn freecam_controls(
    time: Res<Time>,
    mode: Res<CameraMode>,
    config: Res<SimulationConfig>,
    mut mouse_motion_events: MessageReader<MouseMotion>,
    keyboard: Res<ButtonInput<KeyCode>>,
    camera_query: Single<&mut Transform, (With<MainCamera>, Without<Infantry>)>,
) {
    if mode.0 != FollowingType::Free {
        return;
    }

    let delta = time.delta_secs();
    let mut camera_transform = camera_query.into_inner();

    let mut mouse_delta = Vec2::ZERO;
    for event in mouse_motion_events.read() {
        mouse_delta += event.delta;
    }

    if mouse_delta != Vec2::ZERO {
        let (yaw, pitch, roll) = camera_transform.rotation.to_euler(EulerRot::YXZ);

        let new_yaw = yaw - mouse_delta.x * config.camera.mouse_sensitivity;
        let new_pitch = (pitch - mouse_delta.y * config.camera.mouse_sensitivity).clamp(-1.4, 1.4);

        camera_transform.rotation = Quat::from_euler(EulerRot::YXZ, new_yaw, new_pitch, roll);
    }

    let speed = config.camera.free_move_speed * delta;
    let forward = camera_transform.forward();
    let right = camera_transform.right();
    let up = camera_transform.up();

    if keyboard.pressed(KeyCode::KeyW) {
        camera_transform.translation += forward * speed;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        camera_transform.translation -= forward * speed;
    }
    if keyboard.pressed(KeyCode::KeyA) {
        camera_transform.translation -= right * speed;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        camera_transform.translation += right * speed;
    }
    if keyboard.pressed(KeyCode::KeyN) {
        camera_transform.translation += up * speed;
    }
    if keyboard.pressed(KeyCode::KeyJ) {
        camera_transform.translation -= up * speed;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn robot(at: Vec3) -> (Transform, Transform, Transform, Transform) {
        (
            Transform::from_translation(at),
            Transform::IDENTITY,
            Transform::from_xyz(0.1, 0.2, 0.3),
            Transform::IDENTITY,
        )
    }

    #[test]
    fn first_person_camera_lands_on_the_view_offset() {
        let (infantry, gimbal, view_offset, launch_offset) = robot(Vec3::new(4.0, 1.0, -2.0));
        let mut camera = Transform::IDENTITY;

        apply_follow(
            &mut camera,
            Vec3::new(0.0, 3.0, 2.0),
            FollowingType::Robot,
            &infantry,
            &gimbal,
            &view_offset,
            &launch_offset,
        );

        assert_eq!(
            camera.translation,
            infantry.translation + view_offset.translation
        );
    }

    #[test]
    fn third_person_camera_orbits_the_follow_offset_around_the_robot() {
        let (infantry, gimbal, view_offset, launch_offset) = robot(Vec3::new(1.0, 1.0, 1.0));
        let follow_offset = Vec3::new(0.0, 3.0, 2.0);
        let mut camera = Transform::IDENTITY;

        apply_follow(
            &mut camera,
            follow_offset,
            FollowingType::ThirdPerson,
            &infantry,
            &gimbal,
            &view_offset,
            &launch_offset,
        );

        assert_eq!(camera.translation, infantry.translation + follow_offset);
    }

    #[test]
    fn free_camera_is_left_untouched() {
        let (infantry, gimbal, view_offset, launch_offset) = robot(Vec3::new(1.0, 1.0, 1.0));
        let mut camera = Transform::from_xyz(9.0, 9.0, 9.0);

        apply_follow(
            &mut camera,
            Vec3::new(0.0, 3.0, 2.0),
            FollowingType::Free,
            &infantry,
            &gimbal,
            &view_offset,
            &launch_offset,
        );

        assert_eq!(camera.translation, Vec3::new(9.0, 9.0, 9.0));
    }
}
