use crate::arc_mutex;
use crate::capture::CaptureSource;
use crate::capture::driver::{CaptureConfig, CapturedFrameKind};
use crate::components::{
    Controlled, InfantryChassis, InfantryGimbal, InfantryLaunchOffset, ProjectileCooldown,
    ScenarioAbsent, SubscribeAutoAim,
};
use crate::config::SimulationConfig;
use crate::robomaster::prelude::{ArmorRoot, PowerRune, RuneIndex, TechCore, tech_core_state_json};
use crate::ros2::capture::{CaptureGates, EncodePool, RosCaptureContext, RosCapturePlugin};
use crate::ros2::livox::{RosLivoxContext, RosLivoxPlugin};
use crate::ros2::prelude::AverageRateLimiter;
use crate::ros2::prelude::transform;
use crate::ros2::topic::*;
use crate::statistic::ProjectileStatistics;
use crate::systems::{GimbalAimTarget, GimbalAimTracker, projectile_launch};
use crate::util::entity_query::HierarchyQuery;
use bevy::ecs::system::RunSystemOnce;
use bevy::image::BevyDefault;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use r2r::ClockType::SystemTime;
use r2r::geometry_msgs::msg::{Point, Pose, Vector3};
use r2r::std_msgs::msg::{ColorRGBA, String as RosString};
use r2r::visualization_msgs::msg::Marker;
use r2r::{Clock, Context, Node, std_msgs::msg::Header, tf2_msgs::msg::TFMessage};
use std::collections::HashMap;
use std::f32::consts::PI;
use std::time::Duration;
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    thread::{self, JoinHandle},
};

macro_rules! res_unwrap {
    ($res:tt) => {
        $res.0.lock().unwrap()
    };
}

#[derive(Resource, Deref, DerefMut)]
struct StopSignal(Arc<AtomicBool>);

#[derive(Resource, Deref, DerefMut)]
struct SpinThreadHandle(Option<JoinHandle<()>>);

#[derive(Resource, Deref, DerefMut)]
pub struct RoboMasterClock(pub Arc<Mutex<Clock>>);

/// The wall-clock instant `/tf` was built at, for the current frame.
///
/// `capture_rune` is the one system that samples [`RoboMasterClock`], and it
/// stores the value it stamped `/tf` with here. The benchmark ground-truth
/// publishers reuse this exact value instead of reading the clock again, so a
/// GT message and the TF frame describing the *same world state* carry bitwise
/// equal stamps (`IMPLEMENTATION_PLAN.md` §3.4.1) — which is what turns
/// "GT vs `/tf armor_<N>`" into a real cross-check.
#[derive(Resource, Clone)]
pub struct RosFrameStamp(pub r2r::builtin_interfaces::msg::Time);

/// Set the first time a solver solution arrives on `/rm_gimbal/cmd`.
///
/// A benchmark scenario with `trigger = "on_first_gimbal_cmd"` uses this as its
/// `t_scenario = 0`, i.e. the moment the auto-aim stack is actually alive (the
/// same t0 v1 used). `distance == -1.0` means "no solution", so it does not
/// count: the clock must not start on a frame where nothing was solved.
#[derive(Resource, Default)]
pub struct FirstGimbalCmd(pub AtomicBool);

impl FirstGimbalCmd {
    pub fn mark(&self) {
        self.0.store(true, Ordering::Release);
    }

    pub fn seen(&self) -> bool {
        self.0.load(Ordering::Acquire)
    }
}

#[derive(Resource, Deref, DerefMut)]
pub(crate) struct FireRateLimiter(AverageRateLimiter);

/// Firing cadence the ROS command path allows.
///
/// Shared with `bench::services` so a runtime scenario reset restores exactly the
/// cadence the node was built with instead of duplicating the number.
pub(crate) const FIRE_RATE_HZ: f32 = 10.0;

#[derive(Resource, Deref, DerefMut)]
struct TechCoreStateRateLimiter(AverageRateLimiter);

macro_rules! tf_tree {
    (stamp: $stamp:expr;$root:literal { $($content:tt)* }) => {{
        let stamp = $stamp;
        let mut transform_stamped = vec![];
        let _parent = $root;
        let _current = $root;
        tf_tree!(@frame transform_stamped, stamp, _parent, _current, $($content)*);

        transform_stamped
    }};

    (@header $stamp:ident, $current:ident) => {
        Header {
            stamp: $stamp.clone(),
            frame_id: $current.to_string(),
        }
    };

    (@frame $tf_vec:ident, $stamp:ident, $parent:ident, $current:ident,
        $curr_name:literal as ($translation:expr, $rotation:expr) $(for $pub_:ident)?
        {$($children:tt)*}
        $($remaining:tt)*
    ) => {
        {
            let $parent = &$current;
            let $current = $curr_name;
            $crate::add_tf_frame!($tf_vec, tf_tree!(@header $stamp, $parent), $current, $translation, $rotation);
            $(
                $pub_.publish($crate::pose!(tf_tree!(@header $stamp, $current)));
            )*
            tf_tree!(@frame $tf_vec, $stamp, $parent, $current, $($children)*);
        }
        tf_tree!(@frame $tf_vec, $stamp, $parent, $current, $($remaining)*);
    };

    (@frame $tf_vec:ident, $stamp:ident, $parent:ident, $current:ident,
    $(let $p_name:ident = $p_expr:expr;)*
        for ($($elem:tt),+$(,)?) in $iter:ident {
            $(let $name:ident = $expr:expr;)*
            pub $curr_name:ident as ($translation:expr, $rotation:expr) $(for $pub_:ident)?;
            $($children:tt)*
        }
        $($remaining:tt)*
    ) => {
        $(let $p_name = $p_expr;)*
        for ($($elem),+) in $iter {
            $(let $name = $expr;)*
            let $parent = &$current;
            let $current = $curr_name;
            $crate::add_tf_frame!($tf_vec, tf_tree!(@header $stamp, $parent), $current, $translation, $rotation);
            $(
                $pub_.publish($crate::pose!(tf_tree!(@header $stamp, $current)));
            )*
            tf_tree!(@frame $tf_vec, $stamp, $parent, $current, $($children)*);
        }
        tf_tree!(@frame $tf_vec, $stamp, $parent, $current, $($remaining)*);
    };

    (@frame $tf_vec:ident, $stamp:ident, $parent:ident, $current:ident, $(;)? $(,)? $({})?) => { };
}

pub(crate) fn capture_rune(
    camera: Single<&GlobalTransform, With<CaptureSource>>,
    gimbal: Single<&GlobalTransform, (With<Controlled>, With<InfantryGimbal>)>,
    muzzle_offset: Single<
        (&GlobalTransform, &Transform),
        (With<InfantryLaunchOffset>, With<Controlled>),
    >,

    runes: Query<(Entity, &GlobalTransform, &PowerRune)>,
    targets: Query<(&GlobalTransform, &RuneIndex, &Name)>,

    clock: ResMut<RoboMasterClock>,
    tf_publisher: ResMut<TopicPublisher<GlobalTransformTopic>>,
    gimbal_pose_pub: ResMut<TopicPublisher<GimbalPoseTopic>>,
    odom_pose_pub: ResMut<TopicPublisher<OdomPoseTopic>>,
    muzzle_pose_pub: ResMut<TopicPublisher<MuzzlePoseTopic>>,
    camera_pose_pub: ResMut<TopicPublisher<CameraPoseTopic>>,
    center: Query<(Entity, &GlobalTransform)>,
    qq: HierarchyQuery,
    armor: Query<(Entity, &GlobalTransform, &ArmorRoot), Without<ScenarioAbsent>>,
    marker_pub: ResMut<TopicPublisher<OutpostMarkerTopic>>,
    mut frame_stamp: ResMut<RosFrameStamp>,
) {
    // One clock read for the whole frame: `/tf`, the armour marker and the
    // benchmark ground truth all describe this same instant.
    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());
    frame_stamp.0 = stamp.clone();

    let cam_transform = camera.into_inner();
    let gimbal = gimbal.into_inner();
    let cam_rel = cam_transform.reparented_to(gimbal);
    let muzzle_rel = muzzle_offset.0.reparented_to(gimbal);

    // Avoid per-frame `info!` logging (it will cap FPS under ROS2 builds).
    debug!(
        "[ROS2] ODOM pos: [{:.4}, {:.4}, {:.4}]",
        gimbal.translation().x,
        gimbal.translation().y,
        gimbal.translation().z
    );
    debug!(
        "[ROS2] CAMERA_REL pos: [{:.4}, {:.4}, {:.4}]",
        cam_rel.translation.x, cam_rel.translation.y, cam_rel.translation.z
    );
    let mut targets = targets.into_iter().fold(
        HashMap::<Entity, Vec<(String, Transform)>>::new(),
        |mut map, (tf, target, name)| {
            // only use one target
            if !name.contains("_ACTIVATED") {
                return map;
            }
            let Ok((rune_entity, rune_tf, rune)) = runes.get(target.rune) else {
                return map;
            };
            map.entry(rune_entity).or_default().push((
                format!("power_rune_{:?}_{:?}", rune.mode(), target.target)
                    .to_string()
                    .to_lowercase(),
                tf.reparented_to(rune_tf),
            ));
            map
        },
    );

    debug!(
        "[ROS2] MUZZLE pos: [{:.4}, {:.4}, {:.4}]",
        muzzle_rel.translation.x, muzzle_rel.translation.y, muzzle_rel.translation.z
    );
    let rot = (gimbal.rotation()
        * muzzle_offset.1.rotation
        * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, PI / 2.0))
    .to_euler(EulerRot::ZXY);
    debug!(
        "[ROS2] GIMBAL rpy: [{:.4}, {:.4}, {:.4}]",
        rot.0.to_degrees(),
        rot.1.to_degrees(),
        rot.2.to_degrees()
    );

    // The `*CENTER` child is NOT present in every armor asset. GROUND.glb
    // (`_ARMOR_CENTER`), HERO.glb (`1__L_ARMOR_CENTER`) and vehicle.glb
    // (`*_ARMOR_CENTER`) have one, but OUTPOST.glb's `A..F_ARMOR_ROOT` and
    // `AT|BT_ARMOR_ROOT` have no `*_CENTER` node at all — their children are
    // `*_ARMOR`, `*_ARMOR_BASE`, `*_ARMOR_C`, `*_ARMOR_MARKER`, ... Unconditionally
    // unwrapping the hierarchy lookup therefore crashed the whole simulator with
    // `called Option::unwrap() on a None value` in `capture_rune` as soon as an
    // outpost was loaded in a ROS2 build. (`ros2` is the only feature that registers
    // `capture_rune`, which is why a `default`/`talos` build never hit it.)
    // Fall back to the armor root's own global transform — that is exactly what the
    // otherwise-unused `GlobalTransform` in the `armor` query is there for.
    let armor_center = |entity: Entity, fallback: &GlobalTransform| -> Transform {
        qq.of(entity)
            .suffix("CENTER")
            .any()
            .one()
            .and_then(|center_entity| center.get(center_entity).ok())
            .map(|(_, center_tf)| center_tf.compute_transform())
            .unwrap_or_else(|| fallback.compute_transform())
    };

    let transform_stamped = tf_tree! {
        stamp: stamp.clone();

        "map" {
            "odom" as (gimbal.translation(), Quat::IDENTITY) for odom_pose_pub {
                "gimbal_link" as (Vec3::ZERO, gimbal.rotation() * muzzle_offset.1.rotation * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, PI / 2.0)) for gimbal_pose_pub {
                    "muzzle" as (muzzle_rel.translation, Quat::IDENTITY) {
                        "muzzle_link" as (Vec3::ZERO, Quat::IDENTITY) for muzzle_pose_pub{}
                    }
                    "camera_link" as (cam_rel.translation, Quat::IDENTITY) for camera_pose_pub {
                        "camera_optical_frame" as (Vec3::ZERO, Quat::from_euler(EulerRot::ZYX, -PI / 2.0, PI, PI / 2.0)) {}
                    }
                }
            }
            for (rune_entity, transform, rune) in runes {
                let name = format!("power_rune_{:?}", rune.mode()).to_string().to_lowercase();
                let tf = transform.compute_transform();
                pub name as (tf.translation, tf.rotation);
                let targets = targets.remove(&rune_entity).unwrap_or_default();
                for (name, tf) in targets {
                    pub name as (tf.translation, tf.rotation);
                }
            }
            for (entity, armor_tf, armor) in armor {
                let name = format!("armor_{:?}", armor.id.as_usize())
                    .to_string()
                    .to_lowercase();
                let tf = armor_center(entity, armor_tf);
                pub name as (tf.translation, tf.rotation);
            }
        }
    };

    for (entity, armor_root_tf, armor) in armor {
        let mut tff = armor_center(entity, armor_root_tf);
        tff.rotation =
            armor_root_tf.rotation() * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, -PI / 2.0);
        let tf = transform(tff);
        marker_pub.publish(Marker {
            header: Header {
                stamp: stamp.clone(),
                frame_id: "map".to_string(),
            }
            .clone(),
            ns: "armors".to_string(),
            id: armor.id.as_usize() as i32,
            type_: Marker::CUBE as i32,
            action: Marker::ADD as i32,
            pose: Pose {
                position: Point {
                    x: tf.translation.x,
                    y: tf.translation.y,
                    z: tf.translation.z,
                },
                orientation: tf.rotation,
            },
            scale: Vector3 {
                x: 0.03,
                y: 0.15,
                z: 0.125,
            },
            color: ColorRGBA {
                r: 0.0,
                g: 1.0,
                b: 0.0,
                a: 0.0,
            },
            lifetime: r2r::builtin_interfaces::msg::Duration {
                sec: 0,
                nanosec: 300000000,
            },
            frame_locked: false,
            points: vec![],
            colors: vec![],
            texture_resource: "".to_string(),
            texture: Default::default(),
            uv_coordinates: vec![],
            text: "".to_string(),
            mesh_resource: "".to_string(),
            mesh_file: Default::default(),
            mesh_use_embedded_materials: false,
        });
    }

    tf_publisher.publish(TFMessage {
        transforms: transform_stamped,
    });
}

/// A commanded fire stays latched until a shot actually leaves the barrel.
///
/// **A refused attempt must never consume the pulse**: the latch is cleared only by
/// a real launch (`ProjectileStatistics::launch_count` growth), never by the
/// attempt itself.
///
/// `/rm_gimbal/cmd` reports intent, not a guarantee: an attempt can be refused by
/// the 10 Hz [`FireRateLimiter`] or by `ProjectileCooldown`, and the cooldown only
/// advances while `projectile_launch` is running, so it lags a burst. Each attempt
/// costs one rate-limiter ticket, which is why draining a burst takes a few frames.
/// Consuming the request with the attempt itself let a refusal eat the commanded
/// pulse (measured on the bench: 17 commanded pulses produced 2 shots). Latching it
/// — and clearing it only once [`ProjectileStatistics::launch_count`] has grown
/// past the value seen when the request was made — keeps every pulse while the
/// limiter still bounds the cadence at ~10 Hz.
#[derive(Resource, Default)]
pub(crate) struct PendingFire {
    requested: bool,
    /// `launch_count` at the moment the request was latched, so a launch that
    /// happened *before* the request cannot satisfy it.
    launches_at_request: u32,
    /// Attempts spent on the current request; the cooldown drains in a couple of
    /// attempts, so a much larger count means the attempt never launches (e.g. the
    /// fire path was disabled) and retrying would only burn limiter tickets.
    attempts: u32,
}

impl PendingFire {
    /// P1-B: drop a latched request, so a scenario reset cannot replay a pulse
    /// that was commanded before the reset.
    pub(crate) fn reset(&mut self) {
        *self = Self::default();
    }
}

/// Retries per commanded pulse: at 10 Hz each attempt costs ~0.1 s, and the
/// projectile cooldown (0.05 s of accumulated frame time) is satisfied after a
/// couple of attempts.
const MAX_FIRE_ATTEMPTS: u32 = 10;

fn process_subscription(
    time: Res<Time>,
    mut commands: Commands,
    gimbal_cmd: ResMut<TopicSubscriber<GimbalCmdTopic>>,
    first_cmd: Res<FirstGimbalCmd>,
    mut fire_rate_limiter: ResMut<FireRateLimiter>,
    mut pending_fire: ResMut<PendingFire>,
    mut cooldown: ResMut<ProjectileCooldown>,
    statistics: Res<ProjectileStatistics>,
    gimbal: Single<
        (Entity, Option<&mut GimbalAimTracker>),
        (
            With<Controlled>,
            With<InfantryGimbal>,
            Without<InfantryChassis>,
            Without<InfantryLaunchOffset>,
        ),
    >,
) {
    let (gimbal_entity, mut tracker) = gimbal.into_inner();
    fire_rate_limiter.tick(time.delta());

    // A launch after the request was latched means the shot is out: stop retrying.
    if pending_fire.requested && statistics.launch_count > pending_fire.launches_at_request {
        pending_fire.requested = false;
    }

    // Drain everything that arrived since the last frame, newest state last: the
    // queue keeps a whole `fire_advice` pulse instead of replacing it with the
    // next message (see `TopicSubscriber`).
    loop {
        let Some(cmd) = gimbal_cmd.try_recv() else {
            break;
        };
        // No solution from the solver: drop the target so the PID loop stops driving.
        // The latched request survives: a brief `distance == -1` is a state update,
        // not a cancellation of an order the solver itself just issued.
        if cmd.distance == -1.0 {
            commands.entity(gimbal_entity).remove::<GimbalAimTracker>();
            break;
        }
        first_cmd.mark();
        if cmd.fire_advice && !pending_fire.requested {
            pending_fire.requested = true;
            pending_fire.launches_at_request = statistics.launch_count;
            pending_fire.attempts = 0;
        }

        let target = GimbalAimTarget::from_solver_degrees(cmd.yaw as f32, cmd.pitch as f32);
        match tracker.as_mut() {
            Some(tracker) => tracker.retarget(target),
            None => {
                commands
                    .entity(gimbal_entity)
                    .insert(GimbalAimTracker::new(target));
            }
        }
    }

    // `ProjectileCooldown` only advances while `projectile_launch` runs, so on the
    // ROS path (sparse attempts) it measured attempts instead of time: measured, a
    // 17-pulse burst realized ~2 shots at ~1.7 Hz although the limiter allows 10 Hz
    // and the cooldown is 0.05 s. Advance it with frame time while a request is
    // pending. The Gamepad path is untouched — it has no request, so nothing ticks
    // the cooldown here and its own per-frame call keeps its cadence — and the
    // limiter below stays the binding gate, so the <=10 Hz contract is unchanged.
    if pending_fire.requested {
        cooldown.tick(time.delta());
    }

    // Service the latched request: one attempt per rate-limiter ticket. A refused
    // attempt (no ticket this frame) does not lose the pulse, because the request is
    // only cleared once a launch is observed.
    if pending_fire.requested && fire_rate_limiter.allow() {
        pending_fire.attempts += 1;
        if pending_fire.attempts > MAX_FIRE_ATTEMPTS {
            pending_fire.requested = false;
        } else {
            commands.queue(|w: &mut World| {
                w.run_system_once(projectile_launch).unwrap();
            });
        }
    }
}

fn publish_tech_core_state(
    time: Res<Time>,
    clock: Res<RoboMasterClock>,
    mut limiter: ResMut<TechCoreStateRateLimiter>,
    cores: Query<&TechCore>,
    state_pub: Res<TopicPublisher<TechCoreStateTopic>>,
) {
    limiter.tick(time.delta());
    if !limiter.allow() {
        return;
    }

    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());
    state_pub.publish(RosString {
        data: tech_core_state_json(
            stamp.sec,
            stamp.nanosec,
            time.elapsed_secs_f64(),
            cores.iter(),
        ),
    });
}

fn cleanup_ros2_system(
    mut exit: MessageReader<AppExit>,
    stop_signal: Res<StopSignal>,
    mut handle_res: ResMut<SpinThreadHandle>,
) {
    if exit.read().len() > 0 {
        stop_signal.store(true, Ordering::Release);
        if let Some(handle) = handle_res.take() {
            info!("Waiting for ROS 2 spin thread to join...");
            match handle.join() {
                Ok(_) => info!("ROS 2 thread successfully joined. Safe to exit."),
                Err(_) => error!("WARNING: ROS 2 thread panicked or failed to join."),
            }
        }
    }
}

#[derive(Default)]
pub struct ROS2Plugin {}

impl Plugin for ROS2Plugin {
    fn build(&self, app: &mut App) {
        let sim_config = app
            .world()
            .get_resource::<SimulationConfig>()
            .cloned()
            .unwrap_or_default();
        let mut node = Node::create(Context::create().unwrap(), "simulator", "robomaster").unwrap();
        let signal_arc = Arc::new(AtomicBool::new(false));

        register_pub(signal_arc.clone(), app, &mut node);
        register_sub(signal_arc.clone(), app, &mut node);

        // Benchmark v2, P1-B: `/sim/scenario/{load,start,reset}`. Same reason as
        // the bench topics below — the DDS node lives here, and the service
        // listener threads only enqueue (`poll_scenario_services` does the work
        // inside the Bevy schedule).
        #[cfg(feature = "bench")]
        crate::bench::services::register_services(signal_arc.clone(), app, &mut node);

        // Benchmark v2 topics. Registered here rather than in `BenchPlugin`
        // because this is where the DDS node lives; `BenchPlugin` (added after
        // this plugin) only consumes the publisher resources.
        #[cfg(feature = "bench")]
        crate::ros2::topic::bench_topics::register_pub(signal_arc.clone(), app, &mut node);

        let camera_info = app
            .world_mut()
            .remove_resource::<TopicPublisher<CameraInfoTopic>>()
            .unwrap();
        let image_raw = app
            .world_mut()
            .remove_resource::<TopicPublisher<ImageRawTopic>>()
            .unwrap();
        let image_compressed = app
            .world_mut()
            .remove_resource::<TopicPublisher<ImageCompressedTopic>>()
            .unwrap();
        let livox_pointcloud = app
            .world_mut()
            .remove_resource::<TopicPublisher<LivoxPointCloudTopic>>()
            .unwrap();

        let clock = arc_mutex!(Clock::create(SystemTime).unwrap());
        let fov_y = sim_config.camera.fov.to_radians();
        let color_stream = &sim_config.capture.color;
        let color_capture_config = CaptureConfig {
            width: color_stream.width,
            height: color_stream.height,
            texture_format: TextureFormat::bevy_default(),
            frame_kind: CapturedFrameKind::Rgb8,
        };
        // What actually goes on the wire: the capture resolution, unless
        // `[capture.color] scale` shrinks it. `/camera_info` is derived from the
        // same numbers, so a downscaled stream still solves PnP correctly.
        let published_size = color_stream.published_size();
        let image_period = color_stream.publish_period();
        let camera_info_period = color_stream.camera_info_period();
        if !color_stream.wants_image() {
            warn!(
                "[ROS2] [capture.color] disables both publish_raw and publish_compressed: \
                 no image topic will be published"
            );
        } else {
            let rate = match image_period {
                Some(period) => format!("{:.1} Hz", 1.0 / period.as_secs_f64()),
                None => "every captured frame".to_string(),
            };
            let topics = match (color_stream.publish_compressed, color_stream.publish_raw) {
                (true, true) => "/image_raw/compressed + /image_raw",
                (true, false) => "/image_raw/compressed",
                _ => "/image_raw",
            };
            info!(
                "[ROS2] color stream: {}x{} -> {}x{} on {} at {} (jpeg quality {}, {} encode threads)",
                color_stream.width,
                color_stream.height,
                published_size.0,
                published_size.1,
                topics,
                rate,
                color_stream.jpeg_quality(),
                color_stream.encode_threads(),
            );
        }
        let depth_capture_config = CaptureConfig {
            width: sim_config.capture.depth.width,
            height: sim_config.capture.depth.height,
            texture_format: TextureFormat::Depth32Float,
            frame_kind: CapturedFrameKind::Depth32F,
        };
        let publish_freq = sim_config.livox_ros.publish_freq.max(0.1);
        let points_per_publish =
            ((sim_config.livox_ros.points_per_second as f32) / publish_freq).max(1.0) as usize;

        app.insert_resource(RoboMasterClock(clock.clone()))
            .insert_resource(RosFrameStamp(r2r::builtin_interfaces::msg::Time {
                sec: 0,
                nanosec: 0,
            }))
            .insert_resource(FirstGimbalCmd::default())
            .insert_resource(StopSignal(signal_arc.clone()))
            .insert_resource(FireRateLimiter(AverageRateLimiter::from_hz(FIRE_RATE_HZ)))
            .init_resource::<PendingFire>()
            .insert_resource(TechCoreStateRateLimiter(AverageRateLimiter::from_hz(20.0)))
            .add_plugins(RosCapturePlugin {
                config: color_capture_config,
                context: RosCaptureContext {
                    clock: clock.clone(),
                    fov_y,
                    publish_compressed: color_stream.publish_compressed,
                    publish_raw: color_stream.publish_raw,
                    jpeg_quality: color_stream.jpeg_quality(),
                    published_size,
                    gates: Arc::new(CaptureGates::new(
                        image_period,
                        camera_info_period,
                        color_stream.max_in_flight(),
                    )),
                    // The job channel is sized by the capture budget: a frame
                    // holds its slot until it is published, so this many can be
                    // in the pool at once and no more. The publisher thread owns
                    // the topic handles and releases the slots.
                    encode: Arc::new(EncodePool::new(
                        color_stream.encode_threads(),
                        color_stream.max_in_flight(),
                        {
                            let image_raw = image_raw.clone();
                            let image_compressed = image_compressed.clone();
                            move |frame| {
                                if let Some(message) = frame.raw {
                                    image_raw.publish(message);
                                }
                                if let Some(message) = frame.compressed {
                                    image_compressed.publish(message);
                                }
                                // Dropping the slot here is what admits the next
                                // capture.
                                drop(frame.slot);
                            }
                        },
                    )),
                    camera_info,
                    image_raw,
                    image_compressed,
                },
            })
            .add_systems(Last, cleanup_ros2_system)
            .add_systems(
                Update,
                process_subscription
                    .run_if(|enabled: Res<SubscribeAutoAim>| enabled.load(Ordering::Acquire)),
            )
            .add_systems(Update, capture_rune.after(TransformSystems::Propagate))
            .add_systems(Update, publish_tech_core_state)
            .insert_resource(SpinThreadHandle(Some(thread::spawn(move || {
                while !signal_arc.load(Ordering::Acquire) {
                    node.spin_once(Duration::from_millis(1));
                }
            }))));

        if sim_config.livox_ros.enabled {
            app.add_plugins(RosLivoxPlugin {
                config: depth_capture_config,
                context: RosLivoxContext {
                    clock,
                    frame_id: sim_config.livox_ros.frame_id.clone(),
                    fov_y,
                    near: sim_config.capture.depth.near,
                    far: sim_config.capture.depth.far,
                    publish_period_ns: (1_000_000_000.0 / publish_freq) as u64,
                    points_per_publish,
                    line_num: sim_config.livox_ros.line_num.max(1),
                    tag_default: sim_config.livox_ros.tag_default,
                    intensity_default: sim_config.livox_ros.intensity_default,
                    pointcloud: livox_pointcloud,
                    last_publish_ns: Arc::new(AtomicU64::new(0)),
                },
            });
        }
    }
}
