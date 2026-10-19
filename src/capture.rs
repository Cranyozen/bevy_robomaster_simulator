pub mod depth;
pub mod driver;
pub mod effect;
pub mod view_copy;

use bevy::camera::RenderTarget;
use bevy::core_pipeline::prepass::DepthPrepass;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;

use crate::config::SimulationConfig;
use bevy_metalfx::{MetalFxTemporalUpscaling, UpscaleFactor};

pub use driver::CaptureBundle;

#[derive(Component)]
pub struct CaptureSource;

#[derive(Component)]
pub struct CaptureCamera;

#[derive(Resource, Deref, Clone)]
pub struct ImageHandle(pub Handle<Image>);

#[derive(Resource, Clone, Copy)]
pub struct CameraFov(pub f32);

pub const CAPTURE_CAMERA_ORDER: isize = -100;

pub fn setup_capture_camera(world: &mut World) {
    let capture_camera_exists = {
        let mut query = world.query_filtered::<Entity, With<CaptureCamera>>();
        query.iter(world).next().is_some()
    };
    if capture_camera_exists {
        return;
    }

    let render_target_handle = world.resource::<ImageHandle>().0.clone();
    let fov = world.resource::<CameraFov>().0;

    // `[render] capture_msaa` (0 = the shipped `Msaa::Off`). Only this camera is
    // affected: its image is the one the effect pass reads and the encoder
    // publishes, which is what the SUT sees.
    let msaa = capture_msaa(
        world
            .get_resource::<SimulationConfig>()
            .map_or(0, |config| config.render.capture_msaa),
    );

    // The clean copy of this view that the lens-simulation preview toggles to.
    // Created here because it has to match the capture target's size and format
    // exactly; see `effect::insert_raw_preview_target`.
    effect::insert_raw_preview_target(world, &render_target_handle);

    let metalfx = world
        .get_resource::<SimulationConfig>()
        .filter(|config| cfg!(target_os = "macos") && config.render.metalfx_temporal)
        .map(|config| MetalFxTemporalUpscaling {
            factor: UpscaleFactor::clamped(config.render.metalfx_scale),
            frame_generation: config.render.metalfx_frame_generation,
        });

    let mut capture_camera = world.spawn((
        Camera3d::default(),
        Tonemapping::None,
        RenderTarget::Image(render_target_handle.into()),
        Camera {
            order: CAPTURE_CAMERA_ORDER,
            // clear_color: ClearColorConfig::Custom(Color::BLACK),
            ..default()
        },
        Projection::Perspective(PerspectiveProjection {
            fov,
            near: 0.1,
            far: 10000.0,
            ..default()
        }),
        msaa,
        DepthPrepass,
        CaptureCamera,
    ));
    if let Some(metalfx) = metalfx {
        capture_camera.insert(metalfx);
    }
}

/// Maps `[render] capture_msaa` onto Bevy's sample counts.
///
/// `0` (the default) and `1` both mean `Msaa::Off`; `2`, `4` and `8` select
/// `Msaa::Sample2/4/8`. Anything else is a configuration mistake and
/// `Msaa::from_samples` panics on it: warn and keep the shipped behaviour
/// instead of taking the simulator down over a typo.
fn capture_msaa(samples: u32) -> Msaa {
    match samples {
        0 | 1 => Msaa::Off,
        2 | 4 | 8 => Msaa::from_samples(samples),
        other => {
            bevy::log::warn!(
                "[config] render.capture_msaa = {other} is not one of 0/1/2/4/8; using Msaa::Off"
            );
            Msaa::Off
        }
    }
}

#[derive(Component)]
pub struct PreviewCamera;

#[derive(Component)]
pub struct PreviewImageNode;

pub fn setup_preview_window(world: &mut World) {
    let preview_enabled = world
        .resource::<crate::config::SimulationConfig>()
        .preview
        .enabled;
    if !preview_enabled {
        return;
    }

    let render_target_handle = world.resource::<ImageHandle>().0.clone();
    let preview_camera_exists = {
        let mut query = world.query_filtered::<Entity, With<PreviewCamera>>();
        query.iter(world).next().is_some()
    };
    if !preview_camera_exists {
        world.spawn((
            Camera2d::default(),
            Camera {
                order: 1,
                ..default()
            },
            PreviewCamera,
        ));
    }

    let preview_node_exists = {
        let mut query = world.query_filtered::<Entity, With<PreviewImageNode>>();
        query.iter(world).next().is_some()
    };
    if !preview_node_exists {
        // Spawned with the capture image (the simulated camera, i.e. what ROS 2
        // and Talos receive). `capture::effect::show_lens_preview` retargets this
        // node to the clean copy while `F7` asks for it; the published image is
        // never involved.
        world.spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                ..default()
            },
            // Render as a background; help text UI remains on top.
            GlobalZIndex(-1),
            ImageNode::new(render_target_handle),
            PreviewImageNode,
        ));
    }
}

pub fn copy_transform(target: &Transform, our: &mut Transform) {
    our.translation = target.translation;
    our.scale = target.scale;
    our.rotation = target.rotation;
}

pub fn sync_capture_camera(
    target: Single<&Transform, (With<CaptureSource>, Without<CaptureCamera>)>,
    mut our: Single<&mut Transform, (With<CaptureCamera>, Without<CaptureSource>)>,
) {
    copy_transform(&target, &mut our);
}

#[derive(Clone, Copy, Debug)]
pub struct CameraIntrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub width: u32,
    pub height: u32,
}

pub fn compute_camera_intrinsics(width: u32, height: u32, fov_y: f32) -> CameraIntrinsics {
    let fov_y = fov_y as f64;
    let aspect = width as f64 / height as f64;
    let fov_x = 2.0 * ((fov_y / 2.0).tan() * aspect).atan();

    let fx = width as f64 / (2.0 * (fov_x / 2.0).tan());
    let fy = height as f64 / (2.0 * (fov_y / 2.0).tan());

    let cx = width as f64 / 2.0;
    let cy = height as f64 / 2.0;

    CameraIntrinsics {
        fx,
        fy,
        cx,
        cy,
        width,
        height,
    }
}

pub const IMAGE_WIDTH: u32 = 1440;
pub const IMAGE_HEIGHT: u32 = 1080;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_msaa_maps_the_configured_sample_count() {
        assert_eq!(capture_msaa(0), Msaa::Off);
        assert_eq!(capture_msaa(1), Msaa::Off);
        assert_eq!(capture_msaa(2), Msaa::Sample2);
        assert_eq!(capture_msaa(4), Msaa::Sample4);
        assert_eq!(capture_msaa(8), Msaa::Sample8);
        // Unsupported counts fall back to the shipped behaviour rather than
        // panicking in `Msaa::from_samples`.
        assert_eq!(capture_msaa(3), Msaa::Off);
        // The default path is bit-identical to what the simulator shipped before
        // `[render] capture_msaa` existed: the default config value maps to Off,
        // and `Msaa::Off` is what the capture camera used to hard-code.
        let default_samples = crate::config::RenderConfig::default().capture_msaa;
        assert_eq!(default_samples, 0, "the shipped default must stay `Msaa::Off`");
        assert_eq!(capture_msaa(default_samples), Msaa::Off);
    }
}
