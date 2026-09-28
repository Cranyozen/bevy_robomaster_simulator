use avian3d::prelude::SubstepCount;
use bevy::prelude::*;
use crossbeam_channel::{Receiver, Sender, unbounded};
use notify::{Event, RecommendedWatcher, RecursiveMode, Watcher};
use serde::Deserialize;
use std::path::Path;

#[derive(Resource, Deserialize, Reflect, Clone)]
#[reflect(Resource)]
pub struct SimulationConfig {
    #[serde(default)]
    pub window: WindowConfig,
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

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct DebugConfig {
    pub egui: bool,
    pub inspector: bool,
    pub diagnostics: bool,
}

impl Default for DebugConfig {
    fn default() -> Self {
        Self {
            egui: false,
            inspector: false,
            diagnostics: false,
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
}

impl Default for CapturePipelineConfig {
    fn default() -> Self {
        Self {
            color: CaptureStreamConfig::default(),
            depth: DepthCaptureConfig::default(),
        }
    }
}

#[derive(Deserialize, Reflect, Clone)]
#[serde(default)]
pub struct CaptureStreamConfig {
    pub width: u32,
    pub height: u32,
}

impl Default for CaptureStreamConfig {
    fn default() -> Self {
        Self {
            width: 1440,
            height: 1080,
        }
    }
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
            }
        })
    }
}

#[derive(Resource)]
pub struct ConfigWatcher {
    _watcher: RecommendedWatcher,
    receiver: Receiver<Result<Event, notify::Error>>,
}

pub struct ConfigPlugin;

impl Plugin for ConfigPlugin {
    fn build(&self, app: &mut App) {
        let config = SimulationConfig::default();

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
                if let Err(e) = watcher.watch(Path::new("config.toml"), RecursiveMode::NonRecursive)
                {
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
) {
    let Some(watcher) = watcher else {
        return;
    };

    // Non-blocking check for file changes
    while let Ok(Ok(event)) = watcher.receiver.try_recv() {
        if event.kind.is_modify() {
            match SimulationConfig::load() {
                Ok(new_config) => {
                    info!("Config reloaded successfully");
                    if let Some(substeps) = substeps.as_deref_mut() {
                        substeps.0 = new_config.physics.substep_count;
                    }
                    if let Some(fixed_time) = fixed_time.as_deref_mut() {
                        *fixed_time = Time::<Fixed>::from_hz(new_config.physics.fixed_hz.max(1.0));
                    }
                    *config = new_config;
                }
                Err(e) => {
                    warn!("Failed to reload config: {}", e);
                }
            }
        }
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
        assert_eq!(camera.pip_refresh_divisor, 2, "每 2 帧渲染一次，减半开销");
        assert_eq!(camera.pip_far, 60.0);
    }

    #[test]
    fn repository_config_parses() {
        let config = SimulationConfig::load().expect("config.toml 必须能解析");

        assert!(config.camera.pip_refresh_divisor >= 1);
        assert!(config.camera.pip_far > 0.0);
    }
}
