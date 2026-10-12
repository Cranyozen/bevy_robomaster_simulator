//! Simulated camera lens and sensor for the capture camera.
//!
//! WHY: the auto-aim stack is tuned against real camera frames, but the capture
//! camera renders a clean, noise-free, perfectly rectilinear image with no lens
//! falloff — the one thing the robot's own camera has and this one did not, and
//! exactly where a detector that works in the simulator stops working on the
//! robot. A threshold-based light-bar detector is sensitive to vignetting and
//! grain; PnP is sensitive to distortion. This module puts a lens and a sensor in
//! front of the capture camera so the simulated feed is as awkward as the real one.
//!
//! WHAT IT DOES NOT TOUCH: the main (operator) camera, the inset camera and the
//! depth/livox camera are left alone. Only the capture camera — the one whose
//! image goes to ROS 2 / Talos and is mirrored in the on-screen preview — gets
//! the layer.
//!
//! THE ROS INVARIANT: the layer runs *inside* the capture camera's own view (a
//! [`FullscreenMaterial`] pass, the same mechanism Bevy's bloom and tonemapping
//! use), so the pixels the readback publishes are the pixels the camera produced;
//! no second copy can drift out of sync. `F7` therefore cannot change the
//! published image at all — it only picks which of two images the *preview
//! window* shows: the simulated one (what ROS 2/Talos receive, and the default) or
//! a clean copy taken from the view before the pass ran. The only switch that
//! removes the layer from the published image is `[capture.effect] enabled =
//! false`, which removes the pass itself.
//!
//! COST: one fullscreen pass per captured frame on the capture camera, plus a
//! 32-byte uniform upload (the grain seed changes every frame). The clean preview
//! copy is a plain texture-to-texture blit and only runs on frames where the
//! preview is showing the clean image, so the default preview costs nothing.

use bevy::core_pipeline::fullscreen_material::{FullscreenMaterial, FullscreenMaterialPlugin};
use bevy::core_pipeline::{Core3dSystems, schedule::Core3d};
use bevy::prelude::*;
use bevy::render::camera::ExtractedCamera;
use bevy::render::extract_component::ExtractComponent;
use bevy::render::render_asset::RenderAssets;
use bevy::render::render_resource::{
    Extent3d, Origin3d, ShaderType, TexelCopyTextureInfo, TextureAspect, TextureUsages,
};
use bevy::render::renderer::{RenderContext, ViewQuery};
use bevy::render::texture::GpuImage;
use bevy::render::view::ViewTarget;
use bevy::render::{Extract, ExtractSchedule, RenderApp};
use bevy::shader::ShaderRef;

use crate::capture::{CAPTURE_CAMERA_ORDER, CaptureCamera, ImageHandle, PreviewImageNode};
use crate::config::{CaptureEffectConfig, SimulationConfig};
use crate::systems::GameplaySystems;

/// The lens/sensor parameters uploaded to `assets/shaders/camera_lens.wgsl`.
///
/// WHY A COMPONENT ON THE CAMERA: it is the pass's whole gate. `FullscreenMaterial`
/// only runs for views whose extracted component exists, so removing this component
/// removes the pass — that is how `[capture.effect] enabled = false` costs nothing
/// rather than running a pass that does nothing.
///
/// The field order is the shader's uniform layout; keep them in sync.
#[derive(Component, ExtractComponent, Clone, Copy, Debug, Default, PartialEq, ShaderType)]
pub struct CameraLensEffect {
    vignette: f32,
    vignette_radius: f32,
    vignette_smoothness: f32,
    chromatic_aberration: f32,
    lens_distortion: f32,
    lens_distortion_scale: f32,
    grain: f32,
    /// Grain offset for this frame, so the noise pattern is new every frame
    /// instead of a fixed texture stuck to the sensor.
    seed: f32,
}

impl CameraLensEffect {
    /// The uniform for one frame, or `None` when the layer is switched off (in
    /// which case the caller must make sure the component is *absent*, not
    /// zeroed).
    pub fn from_config(config: &CaptureEffectConfig, seed: f32) -> Option<Self> {
        config.enabled.then(|| Self {
            vignette: config.vignette(),
            vignette_radius: config.vignette_radius(),
            vignette_smoothness: config.vignette_smoothness(),
            chromatic_aberration: config.chromatic_aberration(),
            lens_distortion: config.lens_distortion(),
            lens_distortion_scale: config.lens_distortion_scale(),
            grain: config.grain(),
            seed,
        })
    }
}

impl FullscreenMaterial for CameraLensEffect {
    fn fragment_shader() -> ShaderRef {
        "shaders/camera_lens.wgsl".into()
    }
}

/// Clean, pre-effect copy of the capture view, shown by the preview when `F7`
/// switches the simulated camera off.
///
/// One extra capture-sized texture (6 MB at the default 1440x1080), allocated with
/// the capture camera whether or not the preview is currently showing it, so that
/// the toggle never has to allocate mid-frame.
///
/// Created next to the capture target rather than here, because only the capture
/// target knows the size and format the copy has to match: a texture-to-texture
/// blit requires both, and a mismatch is a wgpu validation error, not a soft
/// failure. See [`insert_raw_preview_target`].
#[derive(Resource, Deref, Clone)]
pub struct RawPreviewHandle(pub Handle<Image>);

/// Creates [`RawPreviewHandle`] from the capture target's own descriptor.
///
/// Safe to call when there is no such handle (an app with no capture plugin at
/// all): it then does nothing, and every system here degrades to a no-op.
pub fn insert_raw_preview_target(world: &mut World, capture: &Handle<Image>) {
    let Some((size, format)) = world.resource::<Assets<Image>>().get(capture).map(|image| {
        (
            image.texture_descriptor.size,
            image.texture_descriptor.format,
        )
    }) else {
        return;
    };

    let mut raw = Image::new_target_texture(size.width, size.height, format, Some(format));
    // A copy *destination* needs `COPY_DST` (already set by `new_target_texture`);
    // `COPY_SRC` is added so the clean image stays readable by anything else that
    // ever wants to grab it.
    raw.texture_descriptor.usage |= TextureUsages::COPY_SRC;

    let handle = world.resource_mut::<Assets<Image>>().add(raw);
    world.insert_resource(RawPreviewHandle(handle));
}

/// Which camera the preview window is showing: the simulated one (what ROS 2 and
/// Talos get) or the clean render. Runtime state of `F7`; `config.toml` only
/// supplies the value it starts at, and the master switch.
#[derive(Resource, Debug, PartialEq, Eq)]
pub struct LensPreview {
    show_effect: bool,
}

impl Default for LensPreview {
    fn default() -> Self {
        Self { show_effect: true }
    }
}

impl LensPreview {
    /// Whether the preview is showing the simulated camera. Never applies to the
    /// published image: see the module docs.
    pub fn show_effect(&self) -> bool {
        self.show_effect
    }

    fn toggle(&mut self) {
        self.show_effect = !self.show_effect;
    }

    /// Explicit form of [`Self::toggle`], for callers that know the state they
    /// want (the config re-seed, and the HUD's tests).
    pub(crate) fn set(&mut self, show_effect: bool) {
        self.show_effect = show_effect;
    }
}

/// Whether the preview window should show the clean copy instead of the camera's
/// own (simulated) image.
///
/// ONE DEFINITION, TWO SIDES: this is both the condition the preview UI switches
/// on and the condition the render world copies the clean image for, so the two
/// cannot disagree — showing a copy nobody refreshes would leave a stale frame on
/// screen, and copying one nobody shows would burn a blit every frame. Two more
/// ways to be "no clean copy": the layer switched off (there is no simulated image
/// to compare against, so the toggle stops mattering instead of latching) and no
/// preview window at all (`[preview] enabled = false`).
fn shows_clean_preview(config: &SimulationConfig, preview: &LensPreview) -> bool {
    config.capture.effect.enabled && config.preview.enabled && !preview.show_effect
}

/// Simulated lens/sensor on the capture camera, with the `F7` preview toggle.
pub struct CameraLensEffectPlugin;

impl Plugin for CameraLensEffectPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FullscreenMaterialPlugin::<CameraLensEffect>::default())
            .init_resource::<LensPreview>()
            .add_systems(
                Update,
                (
                    // Config first: a hot-reloaded `preview_effect` must not
                    // overwrite an `F7` press made in the same frame.
                    apply_lens_preview_config,
                    toggle_lens_preview,
                    sync_camera_lens_effect,
                    show_lens_preview,
                )
                    .chain()
                    .in_set(GameplaySystems::GameLogic),
            );

        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };

        render_app
            .init_resource::<RawPreviewCopy>()
            .add_systems(ExtractSchedule, extract_raw_preview_copy)
            // After the main pass, so the copy holds the scene the camera just
            // drew, and before the post-process set, which is where the lens pass
            // itself runs (`FullscreenMaterial` defaults to
            // `Core3dSystems::PostProcess`). Anything later would copy the
            // already-simulated image and make the toggle a no-op.
            .add_systems(
                Core3d,
                copy_clean_preview
                    .after(Core3dSystems::MainPass)
                    .before(Core3dSystems::PostProcess),
            );
    }
}

/// Seeds the runtime toggle from `[capture.effect] preview_effect`, and re-seeds
/// it on every config (re)load. `config.toml` is hot-reloaded as a whole resource,
/// so an edit — of this field or any other — restores the configured starting
/// state; `F7` then moves it again.
fn apply_lens_preview_config(config: Res<SimulationConfig>, mut preview: ResMut<LensPreview>) {
    if config.is_changed() {
        preview.set(config.capture.effect.preview_effect);
    }
}

/// `F7` flips what the preview shows. Deliberately ignored while the layer is
/// switched off: the same rule as `F4` with `[debug] fps = false`, so the key
/// never latches a state that surprises the user once the gate is on again.
fn toggle_lens_preview(
    keyboard: Res<ButtonInput<KeyCode>>,
    config: Res<SimulationConfig>,
    mut preview: ResMut<LensPreview>,
) {
    if config.capture.effect.enabled && keyboard.just_pressed(KeyCode::F7) {
        preview.toggle();
    }
}

/// Keeps the capture camera's lens uniform in sync with `config.toml` and advances
/// the grain seed every frame.
///
/// The seed is taken from the *real* clock, not the virtual one: grain is a sensor
/// property, and it must keep moving even if the simulation clock is paused or
/// clamped. Two runs are therefore not pixel-identical, which is the honest
/// behaviour for a sensor model.
fn sync_camera_lens_effect(
    config: Res<SimulationConfig>,
    time: Res<Time<Real>>,
    mut commands: Commands,
    capture: Query<Entity, With<CaptureCamera>>,
    mut effect: Query<&mut CameraLensEffect>,
) {
    let Ok(camera) = capture.single() else {
        return;
    };

    let seed = (time.elapsed_secs_f64() * 1000.0).rem_euclid(4096.0) as f32;
    let Some(next) = CameraLensEffect::from_config(&config.capture.effect, seed) else {
        // Switched off: the component has to disappear, not go to zero, or the
        // pass would keep running for nothing.
        if effect.contains(camera) {
            info!("Lens simulation OFF: the published image is clean again");
            commands.entity(camera).remove::<CameraLensEffect>();
        }
        return;
    };

    match effect.get_mut(camera) {
        Ok(mut current) => {
            if *current != next {
                *current = next;
            }
        }
        Err(_) => {
            info!(
                "Lens simulation ON: the capture camera renders through it, so /image_raw* is \
                 affected too; F7 only switches the preview window"
            );
            commands.entity(camera).insert(next);
        }
    }
}

/// Points the preview UI node at the simulated image or the clean copy.
///
/// This is the *only* thing `F7` changes. Both handles are ordinary `Image`s, so
/// the published stream is untouched — there is no path from this system to the
/// readback that ROS 2 and Talos use.
fn show_lens_preview(
    config: Res<SimulationConfig>,
    preview: Res<LensPreview>,
    capture: Option<Res<ImageHandle>>,
    raw: Option<Res<RawPreviewHandle>>,
    mut node: Query<&mut ImageNode, With<PreviewImageNode>>,
) {
    let (Some(capture), Some(raw)) = (capture, raw) else {
        return;
    };
    let Ok(mut node) = node.single_mut() else {
        return;
    };

    let next = if shows_clean_preview(&config, &preview) {
        raw.0.clone()
    } else {
        capture.0.clone()
    };
    // Only touch the node on a real change: writing it re-runs UI layout.
    if node.image != next {
        node.image = next;
    }
}

/// Whether the clean copy is wanted this frame, and where it goes. Render-world
/// state, refreshed from the main world by [`extract_raw_preview_copy`].
#[derive(Resource, Default)]
struct RawPreviewCopy {
    target: Option<Handle<Image>>,
    wanted: bool,
}

fn extract_raw_preview_copy(
    raw: Extract<Option<Res<RawPreviewHandle>>>,
    preview: Extract<Option<Res<LensPreview>>>,
    config: Extract<Option<Res<SimulationConfig>>>,
    mut copy: ResMut<RawPreviewCopy>,
) {
    copy.target = raw.as_ref().map(|raw| raw.0.clone());
    copy.wanted = match (config.as_ref(), preview.as_ref()) {
        (Some(config), Some(preview)) => shows_clean_preview(config, preview),
        _ => false,
    };
}

/// Copies the capture view into the clean preview target, then lets the lens pass
/// run over the view. Runs once per camera, so it filters on the capture camera's
/// order like `capture::view_copy` does for the depth target.
fn copy_clean_preview(
    view: ViewQuery<(&ExtractedCamera, &ViewTarget)>,
    copy: Res<RawPreviewCopy>,
    targets: Res<RenderAssets<GpuImage>>,
    mut ctx: RenderContext,
) {
    if !copy.wanted {
        return;
    }

    let (camera, view_target) = view.into_inner();
    if camera.order != CAPTURE_CAMERA_ORDER {
        return;
    }

    let Some(handle) = copy.target.as_ref() else {
        return;
    };
    let Some(destination) = targets.get(handle) else {
        return;
    };

    // Both textures come from the same capture target, so this is a plain blit.
    // If a future change (a compositing space, an HDR target, a resized preview)
    // ever makes them disagree, skipping is better than a validation error every
    // frame — but it must not be silent, or the preview would just look stale.
    let source = view_target.main_texture();
    if source.format() != destination.texture.format()
        || source.width() != destination.texture.width()
        || source.height() != destination.texture.height()
    {
        warn_once!(
            "lens: capture view is {}x{} {:?} but the clean preview copy is {}x{} {:?}: \
             skipping the copy",
            source.width(),
            source.height(),
            source.format(),
            destination.texture.width(),
            destination.texture.height(),
            destination.texture.format(),
        );
        return;
    }

    ctx.command_encoder().copy_texture_to_texture(
        TexelCopyTextureInfo {
            texture: source,
            mip_level: 0,
            origin: Origin3d::ZERO,
            aspect: TextureAspect::All,
        },
        TexelCopyTextureInfo {
            texture: &destination.texture,
            mip_level: 0,
            origin: Origin3d::ZERO,
            aspect: TextureAspect::All,
        },
        Extent3d {
            width: source.width(),
            height: source.height(),
            depth_or_array_layers: 1,
        },
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::render::render_resource::TextureFormat;
    use bevy::time::TimeUpdateStrategy;
    use std::time::Duration;

    fn config_with(effect: CaptureEffectConfig) -> SimulationConfig {
        let mut config = SimulationConfig::default();
        config.capture.effect = effect;
        config
    }

    fn app_with(effect: CaptureEffectConfig) -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            // Without `RenderPlugin` there is no render world to sync to, but the
            // component hooks `ExtractComponentPlugin` installs still run (and
            // still want this resource) when the layer is switched on or off.
            .add_plugins(bevy::render::sync_world::SyncWorldPlugin)
            .init_resource::<ButtonInput<KeyCode>>()
            .init_resource::<Assets<Image>>()
            .add_plugins(CameraLensEffectPlugin)
            .insert_resource(config_with(effect))
            .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_millis(
                16,
            )));
        app
    }

    fn image_handle(app: &mut App) -> Handle<Image> {
        app.world_mut()
            .resource_mut::<Assets<Image>>()
            .add(Image::new_target_texture(
                8,
                8,
                TextureFormat::Bgra8UnormSrgb,
                None,
            ))
    }

    /// The capture camera plus the two resources the plugin needs, wired the way
    /// `setup_capture_camera` and `setup_preview_window` wire the real ones.
    fn spawn_capture(app: &mut App, preview_image: Handle<Image>) {
        let capture = image_handle(app);
        let raw = image_handle(app);

        app.world_mut().spawn((Camera3d::default(), CaptureCamera));
        app.world_mut().insert_resource(ImageHandle(capture));
        app.world_mut().insert_resource(RawPreviewHandle(raw));
        app.world_mut().spawn((
            Node::default(),
            ImageNode::new(preview_image),
            PreviewImageNode,
        ));
    }

    /// [`spawn_capture`] with a placeholder preview handle, for the tests that
    /// never look at it.
    fn spawn_capture_here(app: &mut App) {
        let preview = image_handle(app);
        spawn_capture(app, preview);
    }

    fn press_f7(app: &mut App) {
        {
            let mut keys = app.world_mut().resource_mut::<ButtonInput<KeyCode>>();
            keys.reset_all();
            keys.press(KeyCode::F7);
        }
        app.update();
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .reset_all();
    }

    fn preview_image(app: &mut App) -> Handle<Image> {
        let mut query = app
            .world_mut()
            .query_filtered::<&ImageNode, With<PreviewImageNode>>();
        let nodes: Vec<Handle<Image>> = query
            .iter(app.world())
            .map(|node| node.image.clone())
            .collect();
        assert_eq!(nodes.len(), 1, "preview has exactly one node");
        nodes[0].clone()
    }

    fn lens_effect(app: &mut App) -> Option<CameraLensEffect> {
        let mut query = app
            .world_mut()
            .query_filtered::<&CameraLensEffect, With<CaptureCamera>>();
        query.iter(app.world()).next().copied()
    }

    #[test]
    fn from_config_clamps_every_knob_and_disables_the_pass_entirely() {
        let extreme = CaptureEffectConfig {
            enabled: true,
            vignette: 5.0,
            vignette_radius: -3.0,
            vignette_smoothness: 99.0,
            chromatic_aberration: 1.0,
            lens_distortion: -9.0,
            lens_distortion_scale: 0.1,
            grain: 8.0,
            ..CaptureEffectConfig::default()
        };
        let effect = CameraLensEffect::from_config(&extreme, 0.0).expect("enabled");

        assert_eq!(effect.vignette, 1.0, "vignette saturates at full black");
        assert_eq!(effect.vignette_radius, 0.0);
        assert_eq!(effect.chromatic_aberration, 0.02);
        assert_eq!(effect.lens_distortion, -0.5);
        assert_eq!(
            effect.lens_distortion_scale, 1.0,
            "never below 1: that samples outside the frame and smears the border"
        );
        assert_eq!(effect.grain, 0.25);

        let off = CaptureEffectConfig {
            enabled: false,
            ..CaptureEffectConfig::default()
        };
        assert!(
            CameraLensEffect::from_config(&off, 0.0).is_none(),
            "关闭时连 pass 都不该存在"
        );
    }

    #[test]
    fn defaults_are_a_visible_but_subtle_lens() {
        let effect = CameraLensEffect::from_config(&CaptureEffectConfig::default(), 0.0)
            .expect("shipped default is enabled");

        assert!((0.05..=0.6).contains(&effect.vignette), "{effect:?}");
        assert!(
            (0.0..0.01).contains(&effect.chromatic_aberration),
            "{effect:?}"
        );
        assert_eq!(
            effect.lens_distortion, 0.0,
            "distortion is off by default: /camera_info publishes zero distortion coefficients, \
             so turning it on by default would make the detector's PnP wrong out of the box"
        );
        assert!((0.0..=0.15).contains(&effect.grain), "{effect:?}");
    }

    #[test]
    fn the_layer_follows_the_config_and_the_grain_seed_moves() {
        let mut app = app_with(CaptureEffectConfig::default());
        spawn_capture_here(&mut app);

        app.update();
        let first = lens_effect(&mut app).expect("the pass is on by default");

        app.update();
        let second = lens_effect(&mut app).expect("still on");
        assert_ne!(first.seed, second.seed, "每帧的噪点种子都要变");
        assert_eq!(
            first.vignette, second.vignette,
            "种子之外的一切都来自 config，不应每帧抖动"
        );

        // Hot reload: the same resource replacement `config_hot_reload` performs.
        app.world_mut()
            .resource_mut::<SimulationConfig>()
            .capture
            .effect
            .enabled = false;
        app.update();
        assert!(
            lens_effect(&mut app).is_none(),
            "关闭后组件必须被移除，否则 pass 仍在跑"
        );

        app.world_mut()
            .resource_mut::<SimulationConfig>()
            .capture
            .effect
            .enabled = true;
        app.update();
        assert!(lens_effect(&mut app).is_some(), "重新打开后立即恢复");
    }

    #[test]
    fn the_layer_only_ever_lands_on_the_capture_camera() {
        let mut app = app_with(CaptureEffectConfig::default());
        let other = app.world_mut().spawn(Camera3d::default()).id();
        spawn_capture_here(&mut app);

        app.update();

        assert!(
            lens_effect(&mut app).is_some(),
            "capture camera gets the layer"
        );
        assert!(
            app.world().get::<CameraLensEffect>(other).is_none(),
            "主视角相机不该被套上模拟镜头"
        );
    }

    #[test]
    fn f7_switches_the_preview_between_the_simulated_and_the_clean_image() {
        let mut app = app_with(CaptureEffectConfig::default());
        spawn_capture_here(&mut app);
        let capture = app.world().resource::<ImageHandle>().0.clone();
        let raw = app.world().resource::<RawPreviewHandle>().0.clone();

        app.update();
        assert_eq!(
            preview_image(&mut app),
            capture,
            "默认显示模拟画面（与 ROS 一致）"
        );

        press_f7(&mut app);
        assert_eq!(preview_image(&mut app), raw, "F7 切到干净画面");

        press_f7(&mut app);
        assert_eq!(preview_image(&mut app), capture, "再按一次切回来");
    }

    #[test]
    fn the_toggle_is_ignored_while_the_layer_is_switched_off() {
        let mut app = app_with(CaptureEffectConfig {
            enabled: false,
            ..CaptureEffectConfig::default()
        });
        spawn_capture_here(&mut app);
        let capture = app.world().resource::<ImageHandle>().0.clone();

        app.update();
        assert_eq!(preview_image(&mut app), capture);

        press_f7(&mut app);
        assert_eq!(
            preview_image(&mut app),
            capture,
            "整层关掉时 F7 不该锁存一个状态"
        );
        assert!(app.world().resource::<LensPreview>().show_effect());
    }

    #[test]
    fn the_config_picks_the_starting_side_of_the_toggle() {
        let mut app = app_with(CaptureEffectConfig {
            preview_effect: false,
            ..CaptureEffectConfig::default()
        });
        spawn_capture_here(&mut app);
        let raw = app.world().resource::<RawPreviewHandle>().0.clone();

        app.update();
        assert_eq!(
            preview_image(&mut app),
            raw,
            "preview_effect = false 从干净画面开始"
        );

        // A config reload re-seeds it, the same way it re-seeds every other
        // `config.toml`-driven value.
        press_f7(&mut app);
        let capture = app.world().resource::<ImageHandle>().0.clone();
        assert_eq!(preview_image(&mut app), capture);

        app.world_mut()
            .resource_mut::<SimulationConfig>()
            .capture
            .effect
            .preview_effect = false;
        app.update();
        assert_eq!(preview_image(&mut app), raw, "热重载后回到配置的状态");
    }

    #[test]
    fn a_missing_capture_setup_degrades_to_a_no_op() {
        // No capture plugin: no camera, no `ImageHandle`, no preview node. The
        // plugin must still build and step without panicking.
        let mut app = app_with(CaptureEffectConfig::default());

        for _ in 0..3 {
            app.update();
        }
        assert!(lens_effect(&mut app).is_none());
    }
}
