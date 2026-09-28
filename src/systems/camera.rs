use bevy::camera::RenderTarget;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use std::f32::consts::PI;

use crate::components::{
    ActiveSlapper, CameraFocus, CameraMode, Controlled, FollowingType, Infantry, InfantryGimbal,
    InfantryLaunchOffset, InfantryViewOffset, MainCamera, PictureInPicture, PictureInPictureNode,
    PictureInPictureState,
};
use crate::config::{CameraConfig, SimulationConfig};
use crate::systems::ControllerState;

const INSET_BORDER_COLOR: Color = Color::srgba(1.0, 1.0, 1.0, 0.35);

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

pub fn inset_controls(controller: Res<ControllerState>, mut inset: ResMut<PictureInPictureState>) {
    if controller.controlled.toggle_inset_just_pressed {
        inset.toggle();
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

/// Off-screen third-person camera for the first-person inset, plus the UI node
/// that shows its render target. Only the render target size is fixed for the
/// session; visibility, margin and opacity stay live.
pub fn setup_picture_in_picture(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    mut images: ResMut<Assets<Image>>,
) {
    let size = inset_size(&config.camera);
    let target = images.add(Image::new_target_texture(
        size.x,
        size.y,
        TextureFormat::Bgra8UnormSrgb,
        Some(TextureFormat::Bgra8UnormSrgb),
    ));

    commands.spawn((
        Camera3d::default(),
        Camera {
            order: 2,
            is_active: false,
            ..default()
        },
        RenderTarget::Image(target.clone().into()),
        Projection::Perspective(PerspectiveProjection {
            fov: config.camera.fov.to_radians(),
            far: config.camera.pip_far.max(1.0),
            ..default()
        }),
        Tonemapping::None,
        Msaa::Off,
        Transform::from_xyz(0.0, 3.0, 2.0).looking_at(Vec3::ZERO, Vec3::Y),
        PictureInPicture::new(),
    ));

    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            right: Val::Px(config.camera.pip_margin),
            bottom: Val::Px(config.camera.pip_margin),
            width: Val::Px(size.x as f32),
            height: Val::Px(size.y as f32),
            border: UiRect::all(Val::Px(1.0)),
            border_radius: BorderRadius::all(Val::Px(6.0)),
            ..default()
        },
        BorderColor::all(INSET_BORDER_COLOR),
        ImageNode {
            image: target,
            color: inset_tint(&config.camera),
            ..default()
        },
        PictureInPictureNode,
    ));
}

fn inset_size(camera: &CameraConfig) -> UVec2 {
    UVec2::new(
        camera.pip_size[0].max(1.0) as u32,
        camera.pip_size[1].max(1.0) as u32,
    )
}

fn inset_tint(camera: &CameraConfig) -> Color {
    Color::srgba(1.0, 1.0, 1.0, camera.pip_opacity.clamp(0.0, 1.0))
}

/// First-person view direction of the robot, i.e. where the gimbal aims. Matches
/// the main camera's `FollowingType::Robot` orientation and `projectile_launch`.
fn inset_aim(root: &Transform, gimbal: &Transform, launch_offset: &Transform) -> Vec3 {
    (root.rotation * gimbal.rotation * launch_offset.rotation)
        .mul_vec3(Vec3::Y)
        .normalize_or_zero()
}

/// Camera offset from the robot root as `[lateral, up, behind]`, measured against
/// the gimbal's **horizontal** aim so the boom swings with the turret without
/// dipping into the ground when the gimbal pitches.
fn inset_boom(aim: Vec3, offset: Vec3) -> Vec3 {
    let horizontal = Vec3::new(aim.x, 0.0, aim.z).normalize_or_zero();
    let forward = if horizontal == Vec3::ZERO {
        Vec3::NEG_Z
    } else {
        horizontal
    };
    let right = forward.cross(Vec3::Y);

    Vec3::Y * offset.y + right * offset.x - forward * offset.z
}

/// One node of a robot hierarchy: the root, its gimbal or its muzzle, together
/// with the marker that says which camera focus owns it.
type RobotPart<'a> = (
    &'a Transform,
    Option<&'a Infantry>,
    Option<&'a InfantryGimbal>,
    Option<&'a InfantryLaunchOffset>,
    Option<&'a Controlled>,
    Option<&'a ActiveSlapper>,
);

/// Shows the inset only while the main view is first-person and follows the same
/// robot as the main camera (own robot, or the active slapper). The inset pass is
/// skipped on frames outside `pip_refresh_divisor` so it only costs a fraction of
/// a second scene render.
pub fn update_picture_in_picture(
    config: Res<SimulationConfig>,
    mode: Res<CameraMode>,
    focus: Res<CameraFocus>,
    state: Res<PictureInPictureState>,
    camera: Single<(&mut PictureInPicture, &mut Transform, &mut Camera), With<PictureInPicture>>,
    parts: Query<
        RobotPart,
        (
            Without<PictureInPicture>,
            Or<(With<Controlled>, With<ActiveSlapper>)>,
        ),
    >,
    inset: Single<(&mut Node, &mut ImageNode), With<PictureInPictureNode>>,
) {
    let (mut pip, mut camera_transform, mut camera) = camera.into_inner();
    let (mut node, mut image) = inset.into_inner();

    let visible = state.enabled && config.camera.pip_enabled && mode.0 == FollowingType::Robot;
    let render = pip.tick(visible, config.camera.pip_refresh_divisor);

    let display = if visible {
        Display::Flex
    } else {
        Display::None
    };
    if node.display != display {
        node.display = display;
    }
    if camera.is_active != render {
        camera.is_active = render;
    }

    let margin = Val::Px(config.camera.pip_margin);
    if node.right != margin {
        node.right = margin;
    }
    if node.bottom != margin {
        node.bottom = margin;
    }

    let tint = inset_tint(&config.camera);
    if image.color != tint {
        image.color = tint;
    }

    if !render {
        return;
    }

    let mut root = None;
    let mut gimbal = None;
    let mut launch_offset = None;
    for (transform, infantry, infantry_gimbal, launch, controlled, slapper) in &parts {
        let follows_focus = match *focus {
            CameraFocus::Own => controlled.is_some(),
            CameraFocus::Slapper => slapper.is_some(),
        };
        if !follows_focus {
            continue;
        }
        if infantry.is_some() {
            root = Some(transform);
        } else if infantry_gimbal.is_some() {
            gimbal = Some(transform);
        } else if launch.is_some() {
            launch_offset = Some(transform);
        }
    }
    let (Some(root), Some(gimbal), Some(launch_offset)) = (root, gimbal, launch_offset) else {
        return;
    };

    // The inset hangs behind the gimbal's aim and looks where the first-person
    // view looks, so turning the turret turns the inset view too.
    let aim = inset_aim(root, gimbal, launch_offset);
    let boom = inset_boom(aim, Vec3::from_array(config.camera.pip_offset));
    camera_transform.translation = root.translation + boom;
    camera_transform.look_at(root.translation + boom + aim, Vec3::Y);
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

    #[test]
    fn inset_defaults_to_eighty_percent_opacity() {
        let camera = CameraConfig::default();

        assert_eq!(inset_size(&camera), UVec2::new(320, 180));
        assert_eq!(
            inset_tint(&camera),
            Color::srgba(1.0, 1.0, 1.0, 0.8),
            "an inset alpha of 0.8 means 20% see-through"
        );
    }

    #[test]
    fn inset_size_and_opacity_are_clamped() {
        let oversized = CameraConfig {
            pip_size: [0.0, -8.0],
            pip_opacity: 2.0,
            ..CameraConfig::default()
        };

        assert_eq!(inset_size(&oversized), UVec2::new(1, 1));
        assert_eq!(inset_tint(&oversized), Color::srgba(1.0, 1.0, 1.0, 1.0));
    }

    #[test]
    fn inset_aim_matches_the_first_person_view_direction() {
        let root = Transform::from_rotation(Quat::from_rotation_y(0.7));
        let gimbal = Transform::from_rotation(Quat::from_euler(EulerRot::YXZ, 0.4, -0.3, 0.0));
        let launch_offset = Transform::from_rotation(Quat::from_rotation_x(-0.2));

        let gimbal_world_rotation = root.rotation * gimbal.rotation;
        let mut first_person = Transform::IDENTITY;
        first_person.rotation = (gimbal_world_rotation
            * launch_offset.rotation
            * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, PI / 2.0))
        .normalize();

        let aim = inset_aim(&root, &gimbal, &launch_offset);

        assert!(
            first_person.forward().dot(aim) > 0.9999,
            "the inset must look where the first-person view looks"
        );
    }

    #[test]
    fn inset_boom_sits_behind_the_horizontal_aim() {
        let offset = Vec3::new(0.0, 1.0, 4.5);

        assert_eq!(inset_boom(Vec3::NEG_Z, offset), Vec3::new(0.0, 1.0, 4.5));
        assert_eq!(inset_boom(Vec3::X, offset), Vec3::new(-4.5, 1.0, 0.0));
        assert_eq!(
            inset_boom(Vec3::NEG_Z, Vec3::new(1.0, 1.0, 4.5)),
            Vec3::new(1.0, 1.0, 4.5)
        );
        // Aiming straight up keeps a sane boom instead of collapsing it.
        assert_eq!(inset_boom(Vec3::Y, offset), Vec3::new(0.0, 1.0, 4.5));
    }
}
