use avian3d::prelude::SubstepCount;
use bevy::prelude::*;
use crossbeam_channel::{Receiver, Sender, unbounded};
use notify::{Event, RecommendedWatcher, RecursiveMode, Watcher};
use serde::Deserialize;
use std::path::Path;
use std::time::Duration;

#[derive(Resource, Deserialize, Reflect, Clone)]
#[reflect(Resource)]
pub struct SimulationConfig {
    #[serde(default)]
    pub window: WindowConfig,
    #[serde(default)]
    pub performance: PerformanceConfig,
    #[serde(default)]
    pub debug: DebugConfig,
    #[serde(default)]
    pub preview: PreviewConfig,
    #[serde(default)]
    pub render: RenderConfig,
    #[serde(default)]
    pub capture: CapturePipelineConfig,
    #[serde(default)]
    pub livox_ros: LivoxRosConfig,
    pub physics: PhysicsConfig,
    pub vehicle: VehicleConfig,
    #[serde(default)]
    pub mecanum: MecanumConfig,
    pub projectile: ProjectileConfig,
    pub camera: CameraConfig,
    #[serde(default)]
    pub bench: BenchConfig,
}

/// Benchmark v2 knobs, from `config.toml` → `[bench]`.
///
/// Only the scenario *path* lives here, never the scenario itself: the runner
/// edits `config.toml` with line-level scalar overrides (`tools/benchmark/plan.py`),
/// which cannot express tables or arrays. The scenario is a separate TOML file
/// (see `src/bench/scenario.rs`) so one config can point at any case.
#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct BenchConfig {
    /// Master switch. `false` is the shipped value, and means the benchmark
    /// systems are registered but do nothing — a normal simulator run is
    /// unaffected even in a `bench` build.
    pub enabled: bool,
    /// Scenario file to run, e.g. `/scenarios/dyn_line_2.toml`. Relative paths
    /// resolve against the process working directory.
    pub scenario_path: String,
    /// `/sim/gt/robots` publish rate in Hz. `0`/invalid publishes every frame;
    /// 50 Hz is the contract default (`IMPLEMENTATION_PLAN.md` §3.3).
    pub gt_robots_hz: f32,
    /// Close an explicit velocity loop on the **scripted** robots, so a
    /// scenario's `v` becomes a real target speed. The shared vehicle model is a
    /// direction actuator — `linear_accelerate` normalises
    /// `forward * stick.y + right * stick.x`, so the stick's magnitude never
    /// reaches the physics and `line v=1` and `line v=6` produce the same
    /// thrust (measured: 1.7 m/s vs 1.2 m/s). The servo only rewrites the
    /// movement/boost of inputs the scenario wrote, never the keyboard/gamepad
    /// path, and `false` restores the raw open-loop stick for A/B runs.
    pub speed_servo: bool,
    /// Attribution radius for `hit_armor` events, in metres.
    ///
    /// A plate hit is attributed to the plate whose **centre** is closest to the
    /// sweep-recovered `contact_point` (see `bench/events.rs`), not to the
    /// collider avian happened to report: with the unbounded speculative margin
    /// the collision can start on a neighbouring plate ~0.2 m early. When the
    /// nearest centre is farther than this radius the event keeps the collider's
    /// plate and logs the fallback (`[bench][hitid]`).
    ///
    /// 0.25 m sits between the two populations measured on the phase bed: a
    /// genuine contact misses its plate centre by 15–110 mm, a mis-attributed
    /// neighbour by 230–344 mm. The radius of the plate ring itself is ≈0.212 m,
    /// so a tighter value would start rejecting real hits on the far side of a
    /// plate; a looser one would re-admit neighbours.
    pub hit_armor_id_max_offset_m: f32,
}

impl Default for BenchConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            scenario_path: String::new(),
            gt_robots_hz: 50.0,
            speed_servo: true,
            hit_armor_id_max_offset_m: 0.25,
        }
    }
}

impl BenchConfig {
    /// Period between `/sim/gt/robots` messages, or `None` for every frame.
    pub fn gt_robots_period(&self) -> Option<Duration> {
        period_from_hz(self.gt_robots_hz)
    }

    /// The scenario file to load, or `None` when the path is unset/blank.
    pub fn scenario_path(&self) -> Option<&Path> {
        let path = self.scenario_path.trim();
        (!path.is_empty()).then(|| Path::new(path))
    }
}

#[derive(Deserialize, Reflect, Clone)]
pub struct WindowConfig {
    pub present_mode: String,
}

impl Default for WindowConfig {
    fn default() -> Self {
        Self {
            // Uncap rendering by default so off-screen capture (Talos/ROS2) can exceed 60Hz.
            present_mode: "auto_no_vsync".to_string(),
        }
    }
}

/// Load limits for the main loop itself, from `config.toml` → `[performance]`.
///
/// Both knobs default to `0.0`, i.e. "no limit", so a config file without this
/// section behaves exactly like the engine defaults did.
#[derive(Deserialize, Reflect, Clone, Default)]
#[serde(default)]
pub struct PerformanceConfig {
    /// Upper bound on the main loop frame rate. `0` leaves the loop uncapped,
    /// which is what off-screen Talos/ROS 2 capture wants; a positive value makes
    /// the loop sleep out the rest of each frame (see `systems::performance`).
    pub max_fps: f32,
    /// Upper bound on a single frame's delta in seconds. `0` keeps the engine
    /// default (0.25 s). This also bounds how many fixed steps one frame may run,
    /// since those are driven by virtual time: `max_frame_delta * fixed_hz`.
    pub max_frame_delta: f32,
}

/// The limiter sleeps in whole milliseconds, so sub-1 fps would just stall input,
/// and above 1 kHz the OS timer cannot pace frames anyway.
const MIN_MAX_FPS: f32 = 1.0;
const MAX_MAX_FPS: f32 = 1000.0;
/// The virtual-clock clamp has the same bounds, expressed as seconds.
const MIN_FRAME_DELTA: f32 = 1.0 / MAX_MAX_FPS;
const MAX_FRAME_DELTA: f32 = 5.0;

impl PerformanceConfig {
    /// Time one frame should take, or `None` when the loop stays uncapped.
    pub fn frame_interval(&self) -> Option<Duration> {
        (self.max_fps.is_finite() && self.max_fps > 0.0).then(|| {
            Duration::from_secs_f64(1.0 / self.max_fps.clamp(MIN_MAX_FPS, MAX_MAX_FPS) as f64)
        })
    }

    /// Clamp for the virtual clock, or `None` to keep the engine default.
    pub fn frame_delta_cap(&self) -> Option<Duration> {
        (self.max_frame_delta.is_finite() && self.max_frame_delta > 0.0).then(|| {
            Duration::from_secs_f64(
                self.max_frame_delta.clamp(MIN_FRAME_DELTA, MAX_FRAME_DELTA) as f64
            )
        })
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct DebugConfig {
    pub egui: bool,
    pub inspector: bool,
    pub diagnostics: bool,
    /// On-screen FPS / frame-time readout in the top-right corner. Also gates the
    /// runtime `F4` toggle: with this off the overlay never renders and the key is
    /// ignored. Hot-reloadable.
    pub fps: bool,
    /// Top-center toast after `config.toml` is hot-reloaded, saying whether the
    /// edit was applied or rejected. Set to `false` to keep the HUD quiet; the
    /// console log is written either way. Hot-reloadable.
    pub reload_notice: bool,
}

impl Default for DebugConfig {
    fn default() -> Self {
        Self {
            egui: false,
            inspector: false,
            diagnostics: false,
            fps: true,
            reload_notice: true,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
pub struct PreviewConfig {
    pub enabled: bool,
}

impl Default for PreviewConfig {
    fn default() -> Self {
        Self { enabled: true }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct RenderConfig {
    pub illuminance: f32,
    pub shadows: bool,
    pub main_camera_fxaa: bool,
    #[serde(alias = "main_camera_metalfx_temporal")]
    pub metalfx_temporal: bool,
    #[serde(alias = "main_camera_metalfx_frame_generation")]
    pub metalfx_frame_generation: bool,
    #[serde(alias = "main_camera_metalfx_scale")]
    pub metalfx_scale: f32,
    /// Far plane of the main and capture cameras, in meters. `0` keeps the
    /// built-in values (the main camera renders out to 5e8 m, the capture camera
    /// to 10 km); a positive value culls early and sharpens depth precision.
    pub view_distance: f32,
    /// Far bound of the shadow cascades, in meters. `0` keeps the engine default
    /// (150 m); a smaller value shrinks the shadow map area, which is the
    /// cheapest way to make shadows affordable.
    pub shadow_max_distance: f32,
    /// Number of shadow cascades, clamped to `1..=4`. `0` keeps the engine
    /// default (4). Fewer cascades means fewer shadow passes per light.
    pub shadow_cascades: u32,
    /// MSAA sample count for the **capture** camera, i.e. the camera whose image
    /// is encoded and published to the SUT. `0` (and `1`) keeps the shipped
    /// behaviour (`Msaa::Off`); `2`, `4` and `8` select `Msaa::Sample2/4/8`.
    /// Anything else falls back to `Msaa::Off` with a warning.
    ///
    /// Only the antialiasing of the intermediate that the effect pass reads
    /// changes; the on-screen camera is unaffected (`main_camera_fxaa` covers
    /// it). Added for the detection-envelope separation experiment: at the
    /// 4.04 m probe the armour number is 20.8 x 32.6 px with ~5 px strokes, so
    /// whether edge antialiasing moves the SUT's number classification has to be
    /// separated from `[capture.color] jpeg_quality` and `[capture.effect]
    /// grain`.
    pub capture_msaa: u32,
}

impl Default for RenderConfig {
    fn default() -> Self {
        Self {
            illuminance: 50.0,
            shadows: false,
            main_camera_fxaa: false,
            metalfx_temporal: cfg!(target_os = "macos"),
            metalfx_frame_generation: false,
            metalfx_scale: 2.0,
            view_distance: 0.0,
            shadow_max_distance: 0.0,
            shadow_cascades: 0,
            capture_msaa: 0,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct PhysicsConfig {
    pub substep_count: u32,
    pub fixed_hz: f64,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            substep_count: 8,
            fixed_hz: 120.0,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct VehicleConfig {
    pub rotation_speed: f32,
    pub yaw_acceleration: f32,
    pub tilt_rotation_speed: f32,
    pub gimbal_rotation_speed: f32,
    pub gimbal_pitch_limit: f32,
    pub max_speed: f32,
    pub linear_acceleration: f32,
    pub acceleration_exponent: f32,
    #[serde(default)]
    pub gimbal_pid: GimbalPidConfig,
}

impl Default for VehicleConfig {
    fn default() -> Self {
        Self {
            rotation_speed: 3.0,
            yaw_acceleration: 24.0,
            tilt_rotation_speed: 3.0,
            gimbal_rotation_speed: 3.0,
            gimbal_pitch_limit: 0.785,
            max_speed: 4.0,
            linear_acceleration: 8.0,
            acceleration_exponent: 10.0,
            gimbal_pid: GimbalPidConfig::default(),
        }
    }
}

/// Gains for the closed-loop gimbal tracking used by every remote/auto-aim command.
/// The two axes carry different inertia and travel limits, so they are tuned apart.
#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct GimbalPidConfig {
    pub yaw: GimbalAxisPidConfig,
    pub pitch: GimbalAxisPidConfig,
}

impl Default for GimbalPidConfig {
    fn default() -> Self {
        Self {
            yaw: GimbalAxisPidConfig {
                kp: 12.0,
                ki: 0.5,
                kd: 0.35,
                integral_limit: 0.5,
                max_rate: 20.0,
            },
            pitch: GimbalAxisPidConfig {
                kp: 10.0,
                ki: 0.5,
                kd: 0.3,
                integral_limit: 0.5,
                max_rate: 12.0,
            },
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct GimbalAxisPidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Bound on the accumulated error term (rad·s), guards against windup.
    pub integral_limit: f32,
    /// Saturation of the commanded angular rate for this axis (rad/s).
    pub max_rate: f32,
}

impl Default for GimbalAxisPidConfig {
    fn default() -> Self {
        Self {
            kp: 12.0,
            ki: 0.5,
            kd: 0.35,
            integral_limit: 0.5,
            max_rate: 20.0,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct MecanumConfig {
    pub wheel_radius_m: f32,
    pub half_wheelbase_m: f32,
    pub half_trackwidth_m: f32,
}

impl Default for MecanumConfig {
    fn default() -> Self {
        Self {
            wheel_radius_m: 0.076,
            half_wheelbase_m: 0.18,
            half_trackwidth_m: 0.15,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
pub struct ProjectileConfig {
    pub lifetime: f32,
    pub speed: f32,
    pub cooldown: f32,
    pub diameter: f32,
    pub uav_size: f32,
    pub uav_vel: f32,
    pub mass: f32,
    pub friction: f32,
    pub linear_damping: f32,
    #[serde(default)]
    pub aerodynamics: ProjectileAerodynamicsConfig,
}

#[derive(Deserialize, Reflect, Clone)]
pub struct ProjectileAerodynamicsConfig {
    pub enabled: bool,
    pub air_density: f32,
    pub drag_coefficient: f32,
    pub wind: [f32; 3],
}

impl Default for ProjectileAerodynamicsConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            // kg/m^3 - air density at sea level (15°C)
            air_density: 1.225,
            // Drag coefficient for a smooth sphere, typical Re for 17mm @ ~25m/s.
            drag_coefficient: 0.47,
            // m/s - wind velocity in world coordinates.
            wind: [0.0, 0.0, 0.0],
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct CameraConfig {
    pub fov: f32,
    pub free_move_speed: f32,
    pub follow_offset: [f32; 3],
    pub mouse_sensitivity: f32,
    /// Third-person inset shown while the main view is first-person.
    pub pip_enabled: bool,
    /// Inset render target size in pixels as `[width, height]` (needs a restart).
    pub pip_size: [f32; 2],
    /// Inset camera offset as `[lateral, up, behind]` measured against the
    /// gimbal's horizontal aim.
    pub pip_offset: [f32; 3],
    /// Distance from the bottom-right corner in pixels.
    pub pip_margin: f32,
    /// Inset opacity, `0.0..=1.0`.
    pub pip_opacity: f32,
    /// Render the inset every N frames (`1` = every frame).
    pub pip_refresh_divisor: u32,
    /// Far plane of the inset camera in meters; culls distant scenery.
    pub pip_far: f32,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            fov: 45.0,
            free_move_speed: 8.0,
            follow_offset: [0.0, 3.0, 2.0],
            mouse_sensitivity: 0.003,
            pip_enabled: true,
            pip_size: [320.0, 180.0],
            pip_offset: [0.0, 1.0, 4.5],
            pip_margin: 16.0,
            pip_opacity: 0.8,
            pip_refresh_divisor: 2,
            pip_far: 60.0,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct CapturePipelineConfig {
    pub color: CaptureStreamConfig,
    pub depth: DepthCaptureConfig,
    /// Simulated lens and sensor in front of the capture camera. See
    /// [`CaptureEffectConfig`].
    pub effect: CaptureEffectConfig,
}

impl Default for CapturePipelineConfig {
    fn default() -> Self {
        Self {
            color: CaptureStreamConfig::default(),
            depth: DepthCaptureConfig::default(),
            effect: CaptureEffectConfig::default(),
        }
    }
}

/// The simulated camera lens and sensor the *capture* camera renders through, from
/// `config.toml` → `[capture.effect]`.
///
/// The capture camera is the robot's camera: its image is what ROS 2 publishes on
/// `/image_raw*`, what the Talos shm link sends, and what the on-screen preview
/// mirrors. Putting the layer there is what makes it impossible for the published
/// image and the preview to disagree — and what makes the `F7` preview toggle
/// unable to change the published image.
///
/// DEFAULTS: a visible but gentle lens. Enough vignetting and grain that a
/// threshold-based detector meets the same noise it will meet on the robot, but
/// not so much that it fails here for reasons it will not hit there. Every knob is
/// clamped by the accessors below, so a typo cannot produce a black or smeared
/// frame.
///
/// WHAT TO RAISE FIRST: `grain` (the sensor half of the simulation) and
/// `vignette`. Leave `lens_distortion` at zero unless the consumer expects it —
/// see that field.
#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct CaptureEffectConfig {
    /// Master switch for the whole layer. `false` leaves the capture camera — and
    /// therefore the published image — completely clean, and turns `F7` into a
    /// no-op. Easier to reach than the individual knobs when a detector is being
    /// calibrated and every unknown should be removed at once.
    pub enabled: bool,
    /// What the on-screen preview starts with: `true` = the simulated camera,
    /// i.e. exactly what ROS 2/Talos receive, `false` = the clean render. `F7`
    /// flips it at runtime while `enabled`; the published image is unaffected
    /// either way.
    pub preview_effect: bool,
    /// Lens falloff at the corners, `0.0..=1.0` (`1.0` = corners fully black).
    pub vignette: f32,
    /// Normalized radius where the darkening starts: `0.0` = at the centre,
    /// `1.0` = at the image corners. Clamped to `0.0..=2.0`.
    pub vignette_radius: f32,
    /// How far past `vignette_radius` the falloff takes to reach full strength.
    /// Smaller is a harder, more obviously artificial edge. Clamped to `0.0..=2.0`.
    pub vignette_smoothness: f32,
    /// Lateral chromatic aberration, as a fraction of the image the red and blue
    /// channels are displaced by at the edges, clamped to `0.0..=0.02`. `0.005`
    /// moves them about 4 px apart at the corners of a 1440-wide frame. Green is
    /// never displaced, so the luminance an armour detector thresholds keeps its
    /// geometry and only the colour fringes move.
    pub chromatic_aberration: f32,
    /// Radial distortion coefficient `k1` of the Brown-Conrady model, `-0.5..=0.5`
    /// (positive = barrel, negative = pincushion).
    ///
    /// OFF BY DEFAULT ON PURPOSE: the published `/camera_info` describes an
    /// undistorted pinhole (`d` is all zeros), so enabling this makes the image
    /// and the intrinsics disagree and PnP's translation estimates drift by an
    /// amount that grows towards the edges. Turn it on when the consumer is
    /// expected to undistort for itself; leave it off when the goal is to test a
    /// detector that trusts `camera_info`.
    pub lens_distortion: f32,
    /// Zoom applied after distortion, `1.0..=2.0`. Barrel distortion pushes the
    /// frame's edge pixels outwards; zooming crops them back off. Never below 1.0:
    /// that samples outside the rendered frame and smears the border.
    pub lens_distortion_scale: f32,
    /// Sensor grain, as a fraction of the local signal (`0.0..=0.25`), redrawn
    /// every frame. This is multiplicative gain noise, so it vanishes in true
    /// black rather than turning it grey. Set to `0.0` to switch the sensor off
    /// and keep the lens: grain raises the entropy of every frame, which at
    /// `[capture.color] jpeg_quality` costs image bytes and encode time.
    pub grain: f32,
}

impl Default for CaptureEffectConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            preview_effect: true,
            vignette: 0.35,
            vignette_radius: 0.75,
            vignette_smoothness: 0.45,
            chromatic_aberration: 0.005,
            lens_distortion: 0.0,
            lens_distortion_scale: 1.0,
            grain: 0.05,
        }
    }
}

impl CaptureEffectConfig {
    /// Vignette strength, clamped so a typo cannot black out the frame.
    pub fn vignette(&self) -> f32 {
        finite_or_zero(self.vignette).clamp(0.0, 1.0)
    }

    /// Vignette inner radius. Past `1.0` the corners are no longer dark.
    pub fn vignette_radius(&self) -> f32 {
        finite_or_zero(self.vignette_radius).clamp(0.0, 2.0)
    }

    /// Vignette falloff width.
    pub fn vignette_smoothness(&self) -> f32 {
        finite_or_zero(self.vignette_smoothness).clamp(0.0, 2.0)
    }

    /// Chromatic aberration. Above `0.02` the fringes are wider than any real
    /// lens and eat the light bars the detector looks for.
    pub fn chromatic_aberration(&self) -> f32 {
        finite_or_zero(self.chromatic_aberration).clamp(0.0, 0.02)
    }

    /// Radial distortion coefficient.
    pub fn lens_distortion(&self) -> f32 {
        finite_or_zero(self.lens_distortion).clamp(-0.5, 0.5)
    }

    /// Distortion zoom. Below `1.0` the sampler would read outside the frame.
    pub fn lens_distortion_scale(&self) -> f32 {
        finite_or_one(self.lens_distortion_scale).clamp(1.0, 2.0)
    }

    /// Sensor grain amplitude.
    pub fn grain(&self) -> f32 {
        finite_or_zero(self.grain).clamp(0.0, 0.25)
    }
}

/// `NaN`/infinite input means "no effect" rather than "undefined pixels".
fn finite_or_zero(value: f32) -> f32 {
    if value.is_finite() { value } else { 0.0 }
}

/// Same, for the fields where `0` would be an invalid value rather than "off".
fn finite_or_one(value: f32) -> f32 {
    if value.is_finite() { value } else { 1.0 }
}

/// The color stream that a `--features ros2` build hands to the auto-aim stack.
///
/// Everything here exists to keep the image *transport* from eating the control
/// loop: an uncompressed 1440x1080 `rgb8` frame is 4.66 MB, so the old
/// "publish every captured frame as `/image_raw`" path put ~230 MB/s through
/// DDS and made the detector work on frames that were already stale. The knobs
/// below bound the rate, the size and the encoding independently.
#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct CaptureStreamConfig {
    pub width: u32,
    pub height: u32,
    /// Upper bound on the image publish rate, in Hz. `0` publishes every
    /// captured frame. Frames that are skipped are skipped *before* the GPU
    /// readback, so they cost nothing at all (no copy, no conversion, no encode).
    pub publish_hz: f32,
    /// Upper bound on the `/camera_info` publish rate, in Hz. The intrinsics are
    /// constant while the resolution is, and the detector latches the first
    /// message it sees, so there is no reason to repeat them per frame.
    pub camera_info_hz: f32,
    /// Publish `sensor_msgs/CompressedImage` (`jpeg`) on `/image_raw/compressed`.
    pub publish_compressed: bool,
    /// Publish `sensor_msgs/Image` (`rgb8`) on `/image_raw`. Off by default:
    /// 4.66 MB/frame is what used to saturate DDS. Turn it on only to drive a
    /// consumer that cannot read compressed images.
    pub publish_raw: bool,
    /// JPEG quality for `publish_compressed`, clamped to `1..=100`. Lower is
    /// faster to encode and cheaper on the wire; the armor detector only needs
    /// enough fidelity to threshold blue light bars.
    pub jpeg_quality: u8,
    /// How many captures may be in flight at once, clamped to `1..=8`.
    ///
    /// A capture spends most of its life *waiting* for the GPU readback to
    /// become mappable (~100 ms on an integrated GPU), during which it holds no
    /// useful CPU. With one slot the image rate collapses to ~1/(that latency);
    /// a few overlapping slots keep the stream fed. Higher costs latency, since
    /// a frame waits behind the ones accepted before it.
    pub max_in_flight: u32,
    /// How many threads encode JPEG frames in parallel, clamped to `1..=8`.
    ///
    /// `image`'s JPEG encoder is scalar — measured ~21 Mpix/s, i.e. ~47 ms for one
    /// 1440x1080 frame — so encoding every frame on the single capture-readback
    /// thread pins the image rate at ~21 Hz and leaves that thread at 100% of one
    /// core while the rest of the machine idles. Encoding on `N` threads lifts the
    /// ceiling to roughly `N x` that, at the cost of `N-1` more cores' worth of
    /// CPU during the encode. Frames are still *published* strictly in capture
    /// order (the detector feeds header stamps to an EKF), so this changes
    /// throughput only, not the message sequence.
    pub encode_threads: u32,
    /// Downscale factor applied before publishing, clamped to `0.1..=1.0`.
    /// `/camera_info` is scaled to match, so PnP stays correct; `1.0` publishes
    /// the capture resolution untouched.
    pub scale: f32,
}

impl Default for CaptureStreamConfig {
    fn default() -> Self {
        Self {
            width: 1440,
            height: 1080,
            publish_hz: 30.0,
            camera_info_hz: 1.0,
            publish_compressed: true,
            publish_raw: false,
            jpeg_quality: 80,
            max_in_flight: 3,
            encode_threads: 3,
            scale: 1.0,
        }
    }
}

impl CaptureStreamConfig {
    /// Period between published images, or `None` to publish every frame.
    pub fn publish_period(&self) -> Option<Duration> {
        period_from_hz(self.publish_hz)
    }

    /// Period between `/camera_info` messages, or `None` to publish every frame.
    pub fn camera_info_period(&self) -> Option<Duration> {
        period_from_hz(self.camera_info_hz)
    }

    /// Publish scale, clamped: sub-pixel resolutions are not representable.
    pub fn scale(&self) -> f32 {
        if self.scale.is_finite() {
            self.scale.clamp(MIN_CAPTURE_SCALE, 1.0)
        } else {
            1.0
        }
    }

    /// Published image size after [`Self::scale`], never zero.
    pub fn published_size(&self) -> (u32, u32) {
        let scale = self.scale() as f64;
        (
            ((self.width as f64 * scale).round().max(1.0)) as u32,
            ((self.height as f64 * scale).round().max(1.0)) as u32,
        )
    }

    /// Capture slots, clamped so a config typo cannot starve or flood the pipeline.
    pub fn max_in_flight(&self) -> usize {
        self.max_in_flight.clamp(1, 8) as usize
    }

    /// JPEG quality, clamped to the encoder's valid range.
    pub fn jpeg_quality(&self) -> u8 {
        self.jpeg_quality.clamp(1, 100)
    }

    /// Encoder threads, clamped so a config typo cannot spawn a pool or disable
    /// the pipeline: 1 thread means "encode inline on the capture worker".
    pub fn encode_threads(&self) -> usize {
        self.encode_threads.clamp(1, 8) as usize
    }

    /// Whether any image message is published at all.
    pub fn wants_image(&self) -> bool {
        self.publish_raw || self.publish_compressed
    }
}

/// Below this the published image is too small for the detector's light-bar
/// geometry to survive; above 1.0 would be an upsample and is never useful.
const MIN_CAPTURE_SCALE: f32 = 0.1;

/// `0`/NaN/negative means "no limit"; otherwise one message per `1/hz` seconds.
fn period_from_hz(hz: f32) -> Option<Duration> {
    if !hz.is_finite() || hz <= 0.0 {
        return None;
    }
    Some(Duration::from_secs_f64(1.0 / hz.clamp(1e-3, 1000.0) as f64))
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct DepthCaptureConfig {
    pub width: u32,
    pub height: u32,
    pub near: f32,
    pub far: f32,
}

impl Default for DepthCaptureConfig {
    fn default() -> Self {
        Self {
            width: 640,
            height: 480,
            near: 0.1,
            far: 80.0,
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct LivoxRosConfig {
    pub enabled: bool,
    pub frame_id: String,
    pub publish_freq: f32,
    pub points_per_second: u32,
    pub line_num: u8,
    pub tag_default: u8,
    pub intensity_default: f32,
}

impl Default for LivoxRosConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            frame_id: "livox_frame".to_string(),
            publish_freq: 10.0,
            points_per_second: 100_000,
            line_num: 6,
            tag_default: 0,
            intensity_default: 100.0,
        }
    }
}

impl SimulationConfig {
    pub fn load() -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let content = std::fs::read_to_string("config.toml")?;
        Ok(toml::from_str(&content)?)
    }
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self::load().unwrap_or_else(|e| {
            warn!("Failed to load config.toml: {}, using defaults", e);
            Self {
                window: WindowConfig::default(),
                performance: PerformanceConfig::default(),
                debug: DebugConfig::default(),
                preview: PreviewConfig::default(),
                render: RenderConfig::default(),
                capture: CapturePipelineConfig::default(),
                livox_ros: LivoxRosConfig::default(),
                physics: PhysicsConfig::default(),
                vehicle: VehicleConfig::default(),
                mecanum: MecanumConfig::default(),
                projectile: ProjectileConfig {
                    lifetime: 5.0,
                    speed: 25.0,
                    cooldown: 0.1,
                    diameter: 0.017,
                    mass: 0.017,
                    friction: 1.1,
                    linear_damping: 0.0,
                    aerodynamics: ProjectileAerodynamicsConfig::default(),
                    uav_size: 1.0,
                    uav_vel: 2.0,
                },
                camera: CameraConfig::default(),
                bench: BenchConfig::default(),
            }
        })
    }
}

#[derive(Resource)]
pub struct ConfigWatcher {
    _watcher: RecommendedWatcher,
    receiver: Receiver<Result<Event, notify::Error>>,
}

/// The file the watcher above follows. Kept in one place so the on-screen notice
/// names the same file the watcher actually reads.
pub const CONFIG_PATH: &str = "config.toml";

/// How one hot-reload attempt ended.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConfigReloadOutcome {
    /// The edited file parsed and replaced the whole `SimulationConfig`.
    Applied,
    /// The edit did not parse, so the previously running config stays in effect.
    Failed(String),
}

/// Broadcast by `config_hot_reload` after every attempt, so the HUD can tell the
/// user what the console already logged. The config side stays free of UI types:
/// whoever wants to show it just reads this message.
#[derive(Message, Clone, Debug, PartialEq, Eq)]
pub struct ConfigReloaded {
    pub outcome: ConfigReloadOutcome,
}

pub struct ConfigPlugin;

impl Plugin for ConfigPlugin {
    fn build(&self, app: &mut App) {
        let config = SimulationConfig::default();

        // Registered up front and unconditionally: `config_hot_reload` writes into
        // it, and a reader in another plugin would panic at startup if the writer
        // only ever got registered on the branch where the watcher came up.
        app.add_message::<ConfigReloaded>();

        // Set up file watcher using crossbeam-channel for thread safety
        let (tx, rx): (
            Sender<Result<Event, notify::Error>>,
            Receiver<Result<Event, notify::Error>>,
        ) = unbounded();
        let watcher_result = RecommendedWatcher::new(
            move |res| {
                let _ = tx.send(res);
            },
            notify::Config::default(),
        );

        match watcher_result {
            Ok(mut watcher) => {
                if let Err(e) = watcher.watch(Path::new(CONFIG_PATH), RecursiveMode::NonRecursive) {
                    warn!("Failed to watch config.toml: {}", e);
                } else {
                    info!("Config hot-reload enabled for config.toml");
                    app.insert_resource(ConfigWatcher {
                        _watcher: watcher,
                        receiver: rx,
                    });
                    app.add_systems(Update, config_hot_reload);
                }
            }
            Err(e) => {
                warn!("Failed to create config watcher: {}", e);
            }
        }

        app.insert_resource(config)
            .register_type::<SimulationConfig>();
    }
}

fn config_hot_reload(
    mut config: ResMut<SimulationConfig>,
    watcher: Option<Res<ConfigWatcher>>,
    mut substeps: Option<ResMut<SubstepCount>>,
    mut fixed_time: Option<ResMut<Time<Fixed>>>,
    mut reloaded: MessageWriter<ConfigReloaded>,
) {
    let Some(watcher) = watcher else {
        return;
    };

    // A single save usually arrives as several `Modify` events (editors truncate,
    // write, then rename), and they are all drained in this one frame. The HUD has
    // exactly one line to show, so only the last attempt is reported — one notice
    // per save instead of one per event.
    let mut outcome = None;

    // Non-blocking check for file changes
    while let Ok(Ok(event)) = watcher.receiver.try_recv() {
        if event.kind.is_modify() {
            outcome = Some(match SimulationConfig::load() {
                Ok(new_config) => {
                    info!("Config reloaded successfully");
                    if let Some(substeps) = substeps.as_deref_mut() {
                        substeps.0 = new_config.physics.substep_count;
                    }
                    if let Some(fixed_time) = fixed_time.as_deref_mut() {
                        *fixed_time = Time::<Fixed>::from_hz(new_config.physics.fixed_hz.max(1.0));
                    }
                    *config = new_config;
                    ConfigReloadOutcome::Applied
                }
                Err(e) => {
                    warn!("Failed to reload config: {}", e);
                    ConfigReloadOutcome::Failed(e.to_string())
                }
            });
        }
    }

    if let Some(outcome) = outcome {
        reloaded.write(ConfigReloaded { outcome });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn camera_section_tolerates_missing_fields() {
        let camera: CameraConfig =
            toml::from_str("fov = 30.0\n").expect("partial [camera] section");

        assert_eq!(camera.fov, 30.0);
        assert_eq!(camera.pip_opacity, 0.8);
        assert!(camera.pip_enabled);
        assert_eq!(camera.pip_refresh_divisor, 2);
    }

    #[test]
    fn inset_defaults_keep_the_extra_pass_cheap() {
        let camera = CameraConfig::default();

        assert_eq!(camera.pip_size, [320.0, 180.0]);
        assert_eq!(camera.pip_offset, [0.0, 1.0, 4.5]);
        assert_eq!(camera.pip_refresh_divisor, 2, "每 2 帧渲染一次，减半开销");
        assert_eq!(camera.pip_far, 60.0);
    }

    #[test]
    fn repository_config_parses() {
        let config = SimulationConfig::load().expect("config.toml 必须能解析");

        assert!(config.camera.pip_refresh_divisor >= 1);
        assert!(config.camera.pip_far > 0.0);
    }

    #[test]
    fn debug_section_tolerates_missing_fields() {
        let debug: DebugConfig =
            toml::from_str("inspector = true\n").expect("partial [debug] section");

        assert!(debug.inspector);
        assert!(!debug.egui);
        assert!(!debug.diagnostics);
        assert!(debug.fps, "FPS 读数默认开启");
        assert!(debug.reload_notice, "热重载提示默认开启");
    }

    #[test]
    fn debug_fps_can_be_turned_off() {
        let debug: DebugConfig = toml::from_str("fps = false\n").expect("partial [debug] section");

        assert!(!debug.fps);
    }

    #[test]
    fn debug_reload_notice_can_be_turned_off() {
        let debug: DebugConfig =
            toml::from_str("reload_notice = false\n").expect("partial [debug] section");

        assert!(!debug.reload_notice);
    }

    #[test]
    fn limit_sections_tolerate_missing_fields() {
        let performance: PerformanceConfig =
            toml::from_str("max_fps = 60.0\n").expect("partial [performance] section");
        let render: RenderConfig =
            toml::from_str("shadow_cascades = 2\n").expect("partial [render] section");

        assert_eq!(performance.max_fps, 60.0);
        assert_eq!(
            performance.max_frame_delta, 0.0,
            "an unset hitch guard stays off"
        );
        assert_eq!(render.shadow_cascades, 2);
        assert_eq!(render.view_distance, 0.0);
        assert_eq!(render.shadow_max_distance, 0.0);
    }

    #[test]
    fn limits_default_to_no_limit() {
        let performance = PerformanceConfig::default();
        let render = RenderConfig::default();

        assert_eq!(performance.max_fps, 0.0);
        assert_eq!(performance.max_frame_delta, 0.0);
        assert_eq!(render.view_distance, 0.0);
        assert_eq!(render.shadow_max_distance, 0.0);
        assert_eq!(render.shadow_cascades, 0);
        assert_eq!(
            performance.frame_interval(),
            None,
            "0 max_fps leaves the loop uncapped for Talos/ROS2 capture"
        );
    }

    #[test]
    fn repository_config_keeps_the_limits_off() {
        let config = SimulationConfig::load().expect("config.toml 必须能解析");

        assert_eq!(config.performance.frame_interval(), None);
        assert_eq!(config.performance.frame_delta_cap(), None);
        assert_eq!(
            config.render.view_distance, 0.0,
            "the shipped config must not change the default far planes"
        );
        assert_eq!(config.render.shadow_cascades, 0);
    }

    #[test]
    fn capture_stream_tolerates_missing_fields() {
        let color: CaptureStreamConfig =
            toml::from_str("width = 640\nheight = 480\n").expect("partial [capture.color]");

        assert_eq!(color.published_size(), (640, 480));
        assert_eq!(
            color.publish_period(),
            CaptureStreamConfig::default().publish_period(),
            "an older config.toml keeps the default publish rate"
        );
    }

    #[test]
    fn capture_rate_zero_or_invalid_means_every_frame() {
        let unlimited = |hz: f32| CaptureStreamConfig {
            publish_hz: hz,
            ..Default::default()
        };

        for hz in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            assert_eq!(
                unlimited(hz).publish_period(),
                None,
                "hz={hz} must not rate limit"
            );
        }

        let period = unlimited(30.0)
            .publish_period()
            .expect("30 Hz limits the rate");
        assert!((period.as_secs_f64() - 1.0 / 30.0).abs() < 1e-6);
    }

    #[test]
    fn capture_scale_is_clamped_and_never_yields_zero_pixels() {
        let at_scale = |scale: f32| CaptureStreamConfig {
            width: 1440,
            height: 1080,
            scale,
            ..Default::default()
        };

        assert_eq!(at_scale(0.5).published_size(), (720, 540));
        assert_eq!(
            at_scale(0.0).published_size(),
            (144, 108),
            "clamped to the 0.1 floor"
        );
        assert_eq!(
            at_scale(4.0).published_size(),
            (1440, 1080),
            "never upsamples"
        );
        assert_eq!(at_scale(f32::NAN).published_size(), (1440, 1080));
    }

    #[test]
    fn capture_in_flight_is_clamped_to_a_usable_range() {
        let slots = |max_in_flight: u32| CaptureStreamConfig {
            max_in_flight,
            ..Default::default()
        };

        assert_eq!(slots(0).max_in_flight(), 1, "never zero: that would stall");
        assert_eq!(slots(99).max_in_flight(), 8);
        assert_eq!(slots(3).max_in_flight(), 3);
    }

    #[test]
    fn capture_jpeg_quality_is_clamped_to_the_encoder_range() {
        let at_quality = |jpeg_quality: u8| CaptureStreamConfig {
            jpeg_quality,
            ..Default::default()
        };

        assert_eq!(at_quality(0).jpeg_quality(), 1);
        assert_eq!(at_quality(200).jpeg_quality(), 100);
        assert_eq!(at_quality(80).jpeg_quality(), 80);
    }

    #[test]
    fn shipped_config_publishes_compressed_only() {
        let config = SimulationConfig::load().expect("config.toml 必须能解析");
        let color = &config.capture.color;

        assert!(color.wants_image());
        assert!(
            color.publish_compressed,
            "the compressed stream is what keeps DDS affordable"
        );
        assert!(
            color.publish_period().is_some(),
            "the shipped config rate limits the image stream"
        );
    }

    #[test]
    fn capture_effect_tolerates_missing_fields() {
        // An older config.toml has no `[capture.effect]` section at all: the layer
        // defaults to on, which is what the section is for.
        let effect: CaptureEffectConfig =
            toml::from_str("grain = 0.2\n").expect("partial [capture.effect]");

        assert_eq!(effect.grain(), 0.2);
        assert!(effect.enabled);
        assert!(effect.preview_effect);
        assert_eq!(
            effect.lens_distortion(),
            0.0,
            "畸变默认关闭，否则 /camera_info 的零畸变假设立刻失效"
        );
    }

    #[test]
    fn capture_effect_is_clamped_into_a_visible_range() {
        let extreme = CaptureEffectConfig {
            vignette: f32::NAN,
            vignette_radius: 99.0,
            vignette_smoothness: -1.0,
            chromatic_aberration: 9.0,
            lens_distortion: -7.0,
            lens_distortion_scale: 0.0,
            grain: f32::INFINITY,
            ..CaptureEffectConfig::default()
        };

        assert_eq!(extreme.vignette(), 0.0, "NaN 视为无效果");
        assert_eq!(extreme.vignette_radius(), 2.0);
        assert_eq!(extreme.vignette_smoothness(), 0.0);
        assert_eq!(extreme.chromatic_aberration(), 0.02);
        assert_eq!(extreme.lens_distortion(), -0.5);
        assert_eq!(
            extreme.lens_distortion_scale(),
            1.0,
            "小于 1 会把采样点推到画面外"
        );
        assert_eq!(extreme.grain(), 0.0, "非有限值不能变成全屏噪声");
    }

    #[test]
    fn repository_config_ships_the_lens_on_and_distortion_off() {
        let config = SimulationConfig::load().expect("config.toml 必须能解析");
        let effect = &config.capture.effect;

        assert!(effect.enabled, "出厂配置应带上模拟镜头");
        assert!(effect.grain() > 0.0, "传感器噪点也要在");
        assert_eq!(
            effect.lens_distortion(),
            0.0,
            "出厂配置必须保持零畸变，PnP 才对得上 camera_info"
        );
    }

    #[test]
    fn bench_section_tolerates_missing_fields_and_defaults_off() {
        // An older config.toml has no `[bench]` at all, and the shipped one must
        // not switch a benchmark scenario on by itself.
        let bench: BenchConfig =
            toml::from_str("scenario_path = \"/scenarios/x.toml\"\n").expect("partial [bench]");

        assert!(!bench.enabled, "出厂默认关闭");
        assert_eq!(bench.gt_robots_hz, 50.0, "契约默认 50 Hz");
        assert!(bench.speed_servo, "伺服默认开启：脚本 v 就是目标速度");
        assert_eq!(bench.scenario_path(), Some(Path::new("/scenarios/x.toml")));
        assert!(
            (bench.gt_robots_period().unwrap().as_secs_f64() - 0.02).abs() < 1e-9,
            "50 Hz = 20 ms"
        );
    }

    #[test]
    fn bench_scenario_path_treats_blank_as_unset() {
        let unset = BenchConfig::default();
        let blank = BenchConfig {
            scenario_path: "   ".to_string(),
            ..Default::default()
        };

        assert!(!unset.enabled);
        assert_eq!(unset.scenario_path(), None, "空路径不是场景");
        assert_eq!(blank.scenario_path(), None, "空白也不算路径");
    }
}
