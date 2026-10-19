//! Ground-truth publishers: `/sim/gt/robots` and `/sim/gt/armors`.
//!
//! Both describe the same world state `/tf` describes, in the same frame, with
//! the **same `header.stamp`**: [`RosFrameStamp`] is written by `capture_rune`
//! (the one system that samples `RoboMasterClock`) and read here, and these
//! systems are ordered after it. That is what makes the acceptance check
//! "GT vs `/tf armor_<N>`: position < 1 mm, direction < 0.1°" a real check of
//! two independently published messages rather than of two clocks that happen
//! to agree.
//!
//! `/sim/gt/robots` is rate limited by `[bench] gt_robots_hz` (50 Hz per the
//! contract); `/sim/gt/armors` publishes every frame, matching `/tf`, because
//! the plate positions are what a per-shot lead computation interpolates.

use avian3d::prelude::LinearVelocity;
use bevy::prelude::*;
use r2r::rm_interfaces::msg::{SimArmorState, SimArmorStates, SimRobotState, SimRobotStates};
use r2r::std_msgs::msg::Header;
use std::collections::HashMap;
use std::f32::consts::FRAC_PI_2;

use crate::bench::scenario::Scenario;
use crate::components::{Infantry, InfantryChassis, InfantryGimbal, RobotId, ScenarioAbsent};
use crate::robomaster::prelude::{Armor, ArmorRoot};
use crate::ros2::plugin::RosFrameStamp;
use crate::ros2::prelude::WallRateLimiter;
use crate::ros2::topic::TopicPublisher;
use crate::ros2::topic::bench_topics::{SimGtArmorsTopic, SimGtRobotsTopic};
use crate::util::entity_query::HierarchyQuery;
use crate::bench::motion::wrap_pi;
use crate::util::gt::{
    plate_size, robot_type_u8, ros_vec3, ros_yaw, team_u8, to_ros_rotation, to_ros_vec3,
};

/// The frame every `/tf` transform and every GT position is expressed in.
const MAP_FRAME: &str = "map";

/// Which robot a descendant entity belongs to, by walking `ChildOf`.
pub(crate) fn robot_of(
    entity: Entity,
    robot_ids: &Query<&RobotId>,
    child_of: &Query<&ChildOf>,
) -> Option<RobotId> {
    if let Ok(id) = robot_ids.get(entity) {
        return Some(*id);
    }
    child_of
        .iter_ancestors(entity)
        .find_map(|ancestor| robot_ids.get(ancestor).ok().copied())
}

/// Previous `(yaw, time)` per robot, so `yaw_rate` is the derivative of the
/// **published** heading series rather than whatever the physics root happens to
/// be doing.
#[derive(Resource, Default)]
pub struct ChassisYawTracker {
    last: HashMap<u8, (f32, f64)>,
}

/// Plate `slot` and per-robot armour counts, computed once from the scene.
///
/// `slot` is the rank of a plate's `ArmorId` within its robot, ascending.
/// `ArmorId` itself is a global construction counter, so it identifies a plate
/// but says nothing about ownership; the rank is what turns it into "this car's
/// third plate" for plate-phase analysis.
#[derive(Resource, Default)]
pub struct ArmorIndex {
    slot_by_armor_id: HashMap<u32, u8>,
    count_by_robot: HashMap<u8, u8>,
    /// How many `ArmorRoot` entities the index was built from.
    ///
    /// The scene spawns plates in several waves — **outposts first**, and outpost
    /// plates belong to no robot — so latching on the first non-empty query
    /// recorded "0 plates across 0 robots" and kept it for the whole run
    /// (`armor_count` came out 0 and every `slot` 0). Rebuild while the wave
    /// count keeps changing: it is O(plates) and settles once the scene is up.
    seen_roots: usize,
    built: bool,
}

impl ArmorIndex {
    fn build(
        &mut self,
        armor_roots: &Query<(Entity, &ArmorRoot), Without<ScenarioAbsent>>,
        robot_ids: &Query<&RobotId>,
        child_of: &Query<&ChildOf>,
        seen_roots: usize,
    ) {
        self.slot_by_armor_id.clear();
        self.count_by_robot.clear();
        let mut per_robot: HashMap<u8, Vec<u32>> = HashMap::new();
        for (entity, armor) in armor_roots.iter() {
            let Some(robot) = robot_of(entity, robot_ids, child_of) else {
                continue;
            };
            per_robot
                .entry(robot.0)
                .or_default()
                .push(armor.id.as_usize() as u32);
        }
        for (robot, mut armor_ids) in per_robot {
            armor_ids.sort_unstable();
            self.count_by_robot.insert(robot, armor_ids.len() as u8);
            for (slot, armor_id) in armor_ids.into_iter().enumerate() {
                self.slot_by_armor_id.insert(armor_id, slot as u8);
            }
        }
        self.seen_roots = seen_roots;
        self.built = true;
        if !self.count_by_robot.is_empty() {
            info!(
                "[bench] armour index: {} plates across {} robots",
                self.slot_by_armor_id.len(),
                self.count_by_robot.len()
            );
        }
    }
}

/// The actual build, with the queries it needs.
pub fn build_armor_index(
    mut index: ResMut<ArmorIndex>,
    armor_roots: Query<(Entity, &ArmorRoot), Without<ScenarioAbsent>>,
    robot_ids: Query<&RobotId>,
    child_of: Query<&ChildOf>,
) {
    let roots = armor_roots.iter().count();
    if index.built && index.seen_roots == roots {
        return;
    }
    index.build(&armor_roots, &robot_ids, &child_of, roots);
}

/// The state of one robot that both GT topics need.
struct RobotSample {
    id: RobotId,
    team: u8,
    robot_type: u8,
    position: Vec3,
    velocity: Vec3,
    yaw: f32,
    yaw_rate: f32,
    gimbal_rpy: [f32; 3],
}

fn header(stamp: &RosFrameStamp) -> Header {
    Header {
        stamp: stamp.0.clone(),
        frame_id: MAP_FRAME.to_string(),
    }
}

fn scenario_time_ns(scenario: Option<&Scenario>) -> u64 {
    scenario.map(Scenario::sim_time_ns).unwrap_or(0)
}

/// `/sim/gt/robots`, gated to `[bench] gt_robots_hz`.
#[allow(clippy::too_many_arguments)]
pub fn publish_robot_gt(
    mut limiter: ResMut<GtRobotsLimiter>,
    time: Res<Time>,
    scenario: Option<Res<Scenario>>,
    stamp: Res<RosFrameStamp>,
    publisher: Res<TopicPublisher<SimGtRobotsTopic>>,
    index: Res<ArmorIndex>,
    mut yaw_tracker: ResMut<ChassisYawTracker>,
    chassis: Query<&GlobalTransform, With<InfantryChassis>>,
    robots: Query<(
        Entity,
        &RobotId,
        &GlobalTransform,
        &Infantry,
        Option<&LinearVelocity>,
    ), Without<ScenarioAbsent>>,
    gimbals: Query<(&GlobalTransform, &InfantryGimbal)>,
    children: Query<&Children>,
) {
    if !limiter.0.allow() {
        return;
    }

    // `yaw`/`yaw_rate` describe the **chassis** (the `InfantryChassis` node), not
    // the physics root. `update_chassis_rotation` turns that child — it is the
    // node the armour plates hang from and therefore the heading a consumer can
    // actually see — while the root never receives a torque. Publishing the
    // root's yaw (as this did at first) meant that in a `spin` scenario the
    // plates swept around while `yaw_rate` stayed ~0: a silent, plausible-looking
    // wrong number for exactly the primitive that exists to test plate phase.
    let now = time.elapsed_secs_f64();
    // A robot the scenario leaves out is only tagged with `ScenarioAbsent` on the
    // frame *after* the presence pass, so the component filter alone would leak
    // one frame of a robot that must not appear at all (the camera cannot see it,
    // but a consumer reading GT would). Filter on the declaration too.
    let absent = scenario
        .as_deref()
        .map(|scenario| scenario.absent.as_slice())
        .unwrap_or(&[]);
    let samples: Vec<RobotSample> = robots
        .iter()
        .filter(|(_, id, ..)| !absent.contains(id))
        .map(|(entity, id, transform, infantry, velocity)| {
            let chassis_rotation = children
                .iter_descendants(entity)
                .find_map(|descendant| chassis.get(descendant).ok())
                .map(|transform| transform.rotation())
                .unwrap_or(transform.rotation());
            let gimbal_rotation = children
                .iter_descendants(entity)
                .find_map(|descendant| gimbals.get(descendant).ok())
                .map(|(transform, _)| transform.rotation())
                .unwrap_or(transform.rotation());
            let (_, pitch, roll) = to_ros_rotation(gimbal_rotation).to_euler(EulerRot::ZYX);
            let yaw = ros_yaw(chassis_rotation);
            // Derivative of the published series; wrapped so crossing ±π does not
            // produce a 2π/dt spike.
            let yaw_rate = match yaw_tracker.last.get(&id.0).copied() {
                Some((previous_yaw, previous_time)) if now > previous_time => {
                    (wrap_pi(yaw - previous_yaw) as f64 / (now - previous_time)) as f32
                }
                _ => 0.0,
            };
            yaw_tracker.last.insert(id.0, (yaw, now));
            RobotSample {
                id: *id,
                team: team_u8(infantry.team),
                robot_type: robot_type_u8(infantry.config),
                position: to_ros_vec3(transform.translation()),
                // The body's linear velocity, in the map frame. `LinearVelocity`
                // is the rigid body's, i.e. exactly what `/odom_pose` moves with.
                velocity: to_ros_vec3(velocity.map(|v| v.0).unwrap_or(Vec3::ZERO)),
                yaw,
                yaw_rate,
                // [roll, pitch, yaw] about the map axes.
                gimbal_rpy: [roll, pitch, ros_yaw(gimbal_rotation)],
            }
        })
        .collect();

    // A stable, readable order: the ego first, then by id.
    let mut samples = samples;
    samples.sort_by_key(|sample| sample.id);

    let robots_msg: Vec<SimRobotState> = samples
        .iter()
        .map(|sample| SimRobotState {
            header: header(&stamp),
            sim_time_ns: scenario_time_ns(scenario.as_deref()),
            robot_id: sample.id.0,
            team: sample.team,
            robot_type: sample.robot_type,
            // No damage model in P0: a robot that is in the scene is alive.
            alive: true,
            pos: ros_vec3(sample.position),
            vel: ros_vec3(sample.velocity),
            yaw: sample.yaw,
            yaw_rate: sample.yaw_rate,
            gimbal_rpy: sample.gimbal_rpy.to_vec(),
            armor_count: index
                .count_by_robot
                .get(&sample.id.0)
                .copied()
                .unwrap_or(0),
        })
        .collect();

    publisher.publish(SimRobotStates {
        header: header(&stamp),
        robots: robots_msg,
    });
}

/// `/sim/gt/armors`, every frame (same cadence as `/tf`).
#[allow(clippy::too_many_arguments)]
pub fn publish_armor_gt(
    scenario: Option<Res<Scenario>>,
    stamp: Res<RosFrameStamp>,
    publisher: Res<TopicPublisher<SimGtArmorsTopic>>,
    index: Res<ArmorIndex>,
    armor_roots: Query<(Entity, &GlobalTransform, &ArmorRoot, &Armor), Without<ScenarioAbsent>>,
    robot_ids: Query<&RobotId>,
    child_of: Query<&ChildOf>,
    hierarchy: HierarchyQuery,
    centers: Query<&GlobalTransform>,
) {
    let sim_time_ns = scenario_time_ns(scenario.as_deref());
    // Same one-frame leak as `/sim/gt/robots`: a robot that is declared absent
    // gets its `ScenarioAbsent` tag on the next frame, so filter on the
    // declaration as well.
    let absent = scenario
        .as_deref()
        .map(|scenario| scenario.absent.as_slice())
        .unwrap_or(&[]);
    let mut armors = Vec::with_capacity(armor_roots.iter().len());
    for (entity, armor_transform, armor_root, armor) in &armor_roots {
        let Some(robot_id) = robot_of(entity, &robot_ids, &child_of) else {
            continue;
        };
        if absent.contains(&robot_id) {
            continue;
        }
        // The plate centre is *the same point* `/tf` publishes for `armor_<N>`:
        // the `*_CENTER` child when the model has one, else the root itself. If
        // these two ever diverged, the GT-vs-TF cross-check would be meaningless.
        let center = hierarchy
            .of(entity)
            .suffix("CENTER")
            .any()
            .one()
            .and_then(|center| centers.get(center).ok())
            .map(|center| center.compute_transform())
            .unwrap_or_else(|| armor_transform.compute_transform());

        // Outward plate normal, in the simulator's own armour-marker convention
        // (`capture_rune` publishes the marker cube as
        // `armor_root_rotation * Rz(-90°)`, with +X being the plate's thickness
        // axis). Verified against both shipped models: this points away from the
        // vehicle for all eight plates of `vehicle.glb` and `HERO.glb`, whereas
        // the `*_CENTER` frame's axes do not (its +X is tangential).
        let normal_bevy = (armor_transform.rotation() * Quat::from_rotation_z(-FRAC_PI_2))
            .mul_vec3(Vec3::X);
        let (plate_w, plate_h) = plate_size(armor.spec.armor_type());

        armors.push(SimArmorState {
            header: header(&stamp),
            sim_time_ns,
            armor_id: armor_root.id.as_usize() as u32,
            robot_id: robot_id.0,
            slot: index
                .slot_by_armor_id
                .get(&(armor_root.id.as_usize() as u32))
                .copied()
                .unwrap_or(0),
            pos: ros_vec3(to_ros_vec3(center.translation)),
            normal: ros_vec3(to_ros_vec3(normal_bevy)),
            plate_w,
            plate_h,
        });
    }

    // Deterministic order (armour construction order is stable, but sorting
    // makes the cross-check trivially indexable).
    armors.sort_by_key(|armor| armor.armor_id);
    publisher.publish(SimArmorStates {
        header: header(&stamp),
        armors,
    });
}

/// `[bench] gt_robots_hz` gate. Kept as a resource so the period is read once,
/// not per frame.
#[derive(Resource)]
pub struct GtRobotsLimiter(pub WallRateLimiter);

impl GtRobotsLimiter {
    pub fn new(period: Option<std::time::Duration>) -> Self {
        Self(WallRateLimiter::from_period(period))
    }
}
