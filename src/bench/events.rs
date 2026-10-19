//! Fire and hit events: per-shot attribution for `/sim/events/*`.
//!
//! The simulator already detects armour hits
//! (`crate::robomaster::armor::collision`): a projectile's `CollisionEnd`
//! against an armour plate calls `ProjectileStatistics::increase_accurate()`.
//! What it never had was *which shot* hit *which plate* *where*, which is what
//! makes a hit rate auditable rather than merely counted.
//!
//! `shot_seq` is the primary key: [`publish_fire_events`] allocates it when the
//! projectile appears and stores it on the projectile as [`ShotTag`], so the
//! matching `hit_armor`/`hit_env` can be paired without touching images or
//! timestamps.
//!
//! # Why the hooks live here and not in `systems/projectile.rs`
//!
//! `projectile_launch` and `handle_armor_collision` are *not* feature-gated: they
//! run in every build, including the v1 measurement surface, and they own
//! `ProjectileStatistics`, which the v1 green-dot cross-check reads. Threading a
//! bench-only publisher through them would put `#[cfg]` branches in the normal
//! firing path. Instead:
//!
//! - fire events come from `Added<Projectile>` (same frame the projectile spawns,
//!   same muzzle transform and initial velocity),
//! - hit events come from a second `CollisionEnd` observer registered only in
//!   bench builds (`armor/collision.rs`), evaluating **the same condition** the
//!   v1 counter does.
//!
//! So `hit.hits` and the green-dot `accurate_count` are two views of one fact,
//! not two implementations that can drift.

use avian3d::prelude::{
    Collider, ColliderAabb, CollisionEnd, CollisionStart, LinearVelocity, ShapeCastConfig,
    SpatialQuery, SpatialQueryFilter,
};
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use r2r::rm_interfaces::msg::SimEvent;
use r2r::std_msgs::msg::Header;
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;

use crate::bench::scenario::Scenario;
use crate::components::{Controlled, DartProjectile, Infantry, RobotId};
use crate::config::SimulationConfig;
use crate::robomaster::prelude::{Armor, ArmorRoot, Projectile};
use crate::ros2::plugin::RosFrameStamp;
use crate::ros2::topic::TopicPublisher;
use crate::ros2::topic::bench_topics::{SimFireEventTopic, SimHitEventTopic};
use crate::util::entity_query::HierarchyQuery;
use crate::util::gt::{ros_vec3, to_ros_vec3};

/// `SimEvent.kind`.
pub const KIND_FIRE: u8 = 0;
pub const KIND_HIT_ARMOR: u8 = 1;
pub const KIND_HIT_ENV: u8 = 2;

/// `SimEvent.armor_id` / `target_id` when the field does not apply.
///
/// `255` rather than `0` for `target_id`: robot id 0 is the ego, i.e. a perfectly
/// valid target value, so it cannot double as "unset".
pub const INVALID_ARMOR_ID: u32 = u32::MAX;
pub const INVALID_ROBOT_ID: u8 = u8::MAX;

/// The frame the events' world points and velocities are expressed in.
const MAP_FRAME: &str = "map";

/// Global, monotonically increasing shot counter.
///
/// Global rather than per-shooter: P0 has exactly one shooter, and a single
/// counter cannot be misread as "shot N of robot 0" by a consumer.
#[derive(Resource, Default)]
pub struct ShotSequence(AtomicU32);

impl ShotSequence {
    /// Allocate the next shot id. Starts at 1 so `0` can mean "unset" on the
    /// wire without colliding with a real shot.
    pub fn next(&self) -> u32 {
        self.0.fetch_add(1, Ordering::Relaxed) + 1
    }

    pub fn allocated(&self) -> u32 {
        self.0.load(Ordering::Relaxed)
    }

    /// P1-B: restart the numbering at 1 (the next [`Self::next`] returns 1).
    ///
    /// Only meaningful together with a scenario reset, which also despawns the
    /// projectiles in flight and clears [`ResolvedShots`] / [`ArmorContactSet`];
    /// otherwise a late event from the previous generation could collide with a
    /// fresh `shot_seq`.
    pub fn reset(&self) {
        self.0.store(0, Ordering::Relaxed);
    }
}

/// Everything a shot needs to stay attributable after it leaves the barrel.
#[derive(Component, Clone, Copy, Debug)]
pub struct ShotTag {
    pub shot_seq: u32,
    pub shooter_id: u8,
    /// Scenario clock at fire time, for the event's `sim_time_ns`.
    pub fire_sim_time_ns: u64,
    /// `Time::elapsed_secs_f64()` at fire time. Kept *in addition* to the
    /// scenario clock so `flight_time_s` is a real duration even when there is
    /// no scenario clock yet (bench enabled, scenario not triggered).
    pub fire_elapsed_s: f64,
}

/// Terminal marker: this shot already produced its **armour** hit event.
///
/// The projectile keeps flying after a hit (the simulator does not despawn it),
/// so without this a bounce would emit several hit events for one shot and break
/// the one-`fire`-one-`hit` pairing `shot_seq` promises.
///
/// The component is inserted through `Commands`, and Bevy applies an observer's
/// commands only at the next flush (an observer runs against a `DeferredWorld`),
/// so two `CollisionEnd` events for one projectile inside the same schedule run
/// would both still read an unmarked projectile. [`ResolvedShots`] is the
/// synchronous half of this guard.
#[derive(Component, Clone, Copy, Debug)]
pub struct ShotResolved;

/// This shot already published its `hit_env` event.
///
/// Deliberately *not* [`ShotResolved`]: a shot that clips the environment may
/// still reach a plate, and the P1 rule is that the armour event is published
/// whenever it happens. This marker (plus the synchronous [`ResolvedShots`] set)
/// only stops one shot from emitting the same environment event twice.
#[derive(Component, Clone, Copy, Debug)]
pub struct ShotEnvHit;

/// Per-shot sweep state behind `SimEvent.contact_point`.
///
/// `CollisionEnd` can only say *when* to publish: by then the projectile has
/// already bounced and sits up to one physics step (208 mm at 120 Hz / 25 m/s)
/// away from the plate it touched. Measured on the static infantry target, using
/// that frame's `Transform` as the impact point reads 125.9 mm off the true
/// contact (75.9 mm late along the trajectory, 90.4 mm along the plate normal),
/// while the real aim offset was 27.2 mm.
///
/// The contact point therefore has to be recovered explicitly: [`ShotContact`]
/// carries the position at the start of the current physics step, and
/// [`record_shot_contacts`] sweeps the projectile sphere over that segment,
/// caching the first armour intersection once per shot.
#[derive(Component, Clone, Copy, Debug)]
pub struct ShotContact {
    /// World position at the *start* of the current physics step.
    pub prev: Vec3,
    /// First true intersection with a plate (world frame), once found.
    pub contact: Option<Vec3>,
    /// Collider entity that intersection was found on (diagnostics).
    pub collider: Option<Entity>,
}

impl ShotContact {
    /// A freshly spawned projectile: the next step sweeps from `origin`.
    pub fn at(origin: Vec3) -> Self {
        Self {
            prev: origin,
            contact: None,
            collider: None,
        }
    }
}

/// Projectiles that are **currently touching** an armour collider.
///
/// `avian3d` runs with an unbounded speculative margin, so a 25 m/s projectile
/// (a 208 mm swept AABB per 120 Hz step) generates *phantom* environment
/// contacts up to ~0.22 m away from the surface it is about to hit. Measured on
/// the infantry target: the phantom floor contact starts one physics step before
/// the plate contact, and its `CollisionEnd` — caused by the plate bounce
/// reversing the velocity — arrives first, which used to settle the shot as
/// `hit_env` and swallow the real `armor_id`. A live armour contact must
/// therefore veto an environment resolution: at the moment that phantom contact
/// ends, the armour contact is active by construction.
#[derive(Resource, Default)]
pub struct ArmorContactSet(HashSet<Entity>);

impl ArmorContactSet {
    /// `true` while this projectile touches an armour collider.
    pub fn contains(&self, projectile: Entity) -> bool {
        self.0.contains(&projectile)
    }

    /// P1-B: forget the live contacts. A scenario reset despawns every projectile
    /// in flight, so the old entities can never end their contact.
    pub fn clear(&mut self) {
        self.0.clear();
    }
}

/// Projectiles that touched an armour collider within the last
/// [`REVERSE_SWEEP_STEPS`] physics steps.
///
/// [`ArmorContactSet`] only knows the *live* contacts, but a fast projectile's
/// plate contact can start **and end inside one physics step** (a glancing or
/// spinning hit), and `CollisionEnd` removes it from the live set before
/// `record_shot_contacts` runs in the same step. The reverse sweep would then be
/// gated off exactly when it is needed: that is the residue after task-73
/// (measured 10% of kind=1 hits on the phase bed, 5/52, all of them with a low
/// post-bounce residual speed). This registry keeps a contact actionable for
/// [`REVERSE_SWEEP_STEPS`] steps after it ended.
///
/// The stamp is the **end** of the contact: a long contact stays live in
/// [`ArmorContactSet`] anyway, so what matters is how long ago it stopped.
#[derive(Resource, Default)]
pub struct RecentArmorContact {
    ended_at: HashMap<Entity, Duration>,
}

impl RecentArmorContact {
    /// Remember that this projectile just stopped touching armour.
    pub fn note_contact_end(&mut self, projectile: Entity, now: Duration) {
        self.ended_at.insert(projectile, now);
    }

    /// `true` while a contact that ended within `window` of `now` exists.
    pub fn is_recent(&self, projectile: Entity, now: Duration, window: Duration) -> bool {
        self.ended_at
            .get(&projectile)
            .is_some_and(|ended| now.saturating_sub(*ended) <= window)
    }

    /// Drop every entry older than `window`. Called once per fixed step, so the
    /// map never grows past the last two steps' contacts.
    pub fn prune(&mut self, now: Duration, window: Duration) {
        self.ended_at
            .retain(|_, ended| now.saturating_sub(*ended) <= window);
    }

    /// P1-B: scenario reset (the projectiles are despawned, so no end will come).
    pub fn clear(&mut self) {
        self.ended_at.clear();
    }

    /// Number of remembered contacts (diagnostics/tests).
    pub fn len(&self) -> usize {
        self.ended_at.len()
    }
}

/// Shots that already published their hit event in this process.
///
/// `shot_seq` is allocated once per launch and only increases, so a repeated
/// insert is exactly "this shot is already resolved". This closes the window
/// [`ShotResolved`] leaves open between the collision event and the command
/// flush; it is a process-local set because a single simulator process runs one
/// scenario (or a sequence of them, with `shot_seq` still increasing).
///
/// Two sets, because armouring and environment events no longer exclude each
/// other (P1): `armour` is the terminal set (`resolved()`), `env` only guards
/// against a repeated `hit_env` for the same shot (`env_only()` counts the shots
/// that never reached a plate — the `hit.env_only_fires` figure).
#[derive(Resource, Default)]
pub struct ResolvedShots {
    armour: HashSet<u32>,
    env: HashSet<u32>,
}

impl ResolvedShots {
    /// `true` the first time this shot resolves against armour, `false` later.
    fn claim(&mut self, shot_seq: u32) -> bool {
        self.armour.insert(shot_seq)
    }

    /// `true` the first time this shot reports an environment contact.
    fn claim_env(&mut self, shot_seq: u32) -> bool {
        self.env.insert(shot_seq)
    }

    /// Number of shots resolved against armour so far (diagnostics/tests).
    pub fn resolved(&self) -> usize {
        self.armour.len()
    }

    /// Shots that reported an environment contact and no armour hit.
    pub fn env_only(&self) -> usize {
        self.env.difference(&self.armour).count()
    }

    /// P1-B: forget every resolved shot, so a restarted scenario starts clean.
    pub fn clear(&mut self) {
        self.armour.clear();
        self.env.clear();
    }
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

/// Placeholder for "this event has no true contact point".
///
/// `NaN` rather than zeros on purpose: `(0, 0, 0)` is a perfectly valid map
/// position, so a consumer could not tell the placeholder from a real hit.
fn no_contact_point() -> Vec<f64> {
    vec![f64::NAN; 3]
}

/// The contact point as `float64`, in the same already-aligned map frame as
/// [`ros_vec3`] (`ros_vec3` is the identity widening: the builders receive points
/// that the caller has already run through `to_ros_vec3`).
fn contact_point_f64(point: Vec3) -> Vec<f64> {
    vec![point.x as f64, point.y as f64, point.z as f64]
}

/// A shot leaving the barrel.
///
/// `world_point` is the muzzle position and `velocity` the projectile's initial
/// world velocity, both in the map frame — the same frame the GT armour centres
/// use, so the scorer's muzzle-ray reconstruction needs no extra transform.
pub fn fire_event(
    stamp: &RosFrameStamp,
    sim_time_ns: u64,
    shot_seq: u32,
    shooter_id: u8,
    world_point: Vec3,
    velocity: Vec3,
) -> SimEvent {
    SimEvent {
        header: header(stamp),
        sim_time_ns,
        kind: KIND_FIRE,
        shot_seq,
        shooter_id,
        target_id: INVALID_ROBOT_ID,
        armor_id: INVALID_ARMOR_ID,
        world_point: ros_vec3(world_point),
        contact_point: no_contact_point(),
        velocity: ros_vec3(velocity),
        flight_time_s: 0.0,
    }
}

/// A projectile that ended on an armour plate.
///
/// `armor_id` is the same integer as the `/tf` frame `armor_<N>`, so a hit can
/// be checked against the plate the detector reported without a lookup table.
///
/// Since task-72 it is **the plate whose centre is nearest the sweep
/// `contact_point`**, restricted to the hit robot's own plates — not the plate
/// the collision event named. avian's unbounded speculative margin lets a
/// contact start on the neighbouring plate (~0.212 m ring ⇒ 230–344 mm off the
/// true contact) while the touched plate sits 15–110 mm away, which used to
/// pollute `impact_err` and every per-shot plate-phase reference. When the
/// nearest centre is farther than `[bench] hit_armor_id_max_offset_m` (0.25 m)
/// the event keeps the collider's plate and logs `[bench][hitid]`.
///
/// `contact_point` is the *true* contact point on the plate (from the sweep in
/// [`record_shot_contacts`]); `world_point` stays "the projectile's position in
/// the resolving frame" for compatibility. A `None` contact point is published
/// as all-`NaN`, never as zeros.
#[allow(clippy::too_many_arguments)]
pub fn hit_armor_event(
    stamp: &RosFrameStamp,
    sim_time_ns: u64,
    shot_seq: u32,
    shooter_id: u8,
    target_id: u8,
    armor_id: u32,
    world_point: Vec3,
    contact_point: Option<Vec3>,
    velocity: Vec3,
    flight_time_s: f32,
) -> SimEvent {
    SimEvent {
        header: header(stamp),
        sim_time_ns,
        kind: KIND_HIT_ARMOR,
        shot_seq,
        shooter_id,
        target_id,
        armor_id,
        world_point: ros_vec3(world_point),
        contact_point: contact_point
            .map(contact_point_f64)
            .unwrap_or_else(no_contact_point),
        velocity: ros_vec3(velocity),
        flight_time_s,
    }
}

/// A projectile that ended on the environment: the "missed" half of a shot, so
/// `env_hit_ratio` is measurable instead of inferred from a missing hit.
pub fn hit_env_event(
    stamp: &RosFrameStamp,
    sim_time_ns: u64,
    shot_seq: u32,
    shooter_id: u8,
    world_point: Vec3,
    velocity: Vec3,
    flight_time_s: f32,
) -> SimEvent {
    SimEvent {
        header: header(stamp),
        sim_time_ns,
        kind: KIND_HIT_ENV,
        shot_seq,
        shooter_id,
        target_id: INVALID_ROBOT_ID,
        armor_id: INVALID_ARMOR_ID,
        world_point: ros_vec3(world_point),
        contact_point: no_contact_point(),
        velocity: ros_vec3(velocity),
        flight_time_s,
    }
}

/// Publishes `/sim/events/fire` the frame a projectile spawns.
///
/// Darts are excluded: they are a different weapon (`DartProjectile`) with a
/// different launcher, and mixing them into `shots_fired` would corrupt the
/// 17 mm hit rate. UAVs never carry `Projectile` at all.
pub fn publish_fire_events(
    mut commands: Commands,
    sequence: Res<ShotSequence>,
    time: Res<Time>,
    scenario: Option<Res<Scenario>>,
    stamp: Res<RosFrameStamp>,
    fire_pub: Res<TopicPublisher<SimFireEventTopic>>,
    shooters: Query<&RobotId, (With<Controlled>, With<Infantry>)>,
    projectiles: Query<
        (Entity, &Transform, Option<&LinearVelocity>),
        (Added<Projectile>, Without<DartProjectile>),
    >,
) {
    // P0 has a single shooter (the ego, `RobotId` 0). A second shooter is P1 and
    // would need a per-projectile owner lookup here.
    let Ok(shooter_id) = shooters.single().map(|id| id.0) else {
        return;
    };

    for (entity, transform, velocity) in &projectiles {
        let shot_seq = sequence.next();
        let sim_time_ns = scenario_time_ns(scenario.as_deref());
        let point = to_ros_vec3(transform.translation);
        let velocity = to_ros_vec3(velocity.map(|velocity| velocity.0).unwrap_or(Vec3::ZERO));

        fire_pub.publish(fire_event(
            &stamp, sim_time_ns, shot_seq, shooter_id, point, velocity,
        ));
        commands.entity(entity).insert((
            ShotTag {
                shot_seq,
                shooter_id,
                fire_sim_time_ns: sim_time_ns,
                fire_elapsed_s: time.elapsed_secs_f64(),
            },
            // The sweep starts at the muzzle: the first physics step after this
            // frame casts from here to wherever gravity took the projectile.
            ShotContact::at(transform.translation),
        ));
    }
}

/// `/sim/events/hit`: attributes a projectile collision to a shot.
///
/// Registered (bench builds only) alongside the v1 handler in
/// `robomaster::armor::collision`, which keeps owning `ProjectileStatistics`.
/// The condition that publishes `hit_armor` is deliberately the same condition
/// that increments the green-dot counter, so the two numbers are two views of
/// one fact rather than two guesses.
///
/// An environment collision publishes `hit_env` at most once per projectile
/// (`ShotEnvHit`, plus `ResolvedShots::claim_env` as the synchronous guard). It
/// no longer *ends* the shot: a later plate contact still publishes `hit_armor`,
/// so for a shot that lands first the pair `hits`/green-dot counter can differ —
/// the P1 rule; the legacy counter is untouched.
///
/// A projectile that is currently touching armour is exempt from the
/// environment branch altogether. avian's unbounded speculative margin gives a
/// 25 m/s projectile a ~0.22 m contact envelope, so a low shot reports a phantom
/// floor contact whose `CollisionEnd` (caused by the real plate bounce) arrives
/// *before* the plate's own resolution — that is what made every static infantry
/// shot read `hit_env`. See [`ArmorContactSet`].
/// `DAEDALUS_HIT_DEBUG=1` turns on the collision journal: every projectile
/// `CollisionEnd` prints both colliders, their ancestor `Name` chain and whether
/// an `Armor` ancestor was found. Off by default, so the shipped bench path stays
/// silent; this exists because `world_point` is sampled when the contact *ends*
/// and therefore cannot tell which surface was touched.
fn hit_debug_enabled() -> bool {
    std::env::var_os("DAEDALUS_HIT_DEBUG").is_some()
}

/// Renders `entity < parent < ...` with an `ARMOR` tag on every link that carries
/// an `Armor` component.
fn describe_chain(
    entity: Entity,
    names: &Query<&Name>,
    armor_components: &Query<(), With<Armor>>,
    child_of: &Query<&ChildOf>,
) -> String {
    std::iter::once(entity)
        .chain(child_of.iter_ancestors(entity))
        .map(|current| {
            let name = names
                .get(current)
                .map(|name| name.as_str())
                .unwrap_or("<unnamed>");
            let tag = if armor_components.contains(current) {
                "ARMOR"
            } else {
                "-"
            };
            format!("{name}[{tag}]")
        })
        .collect::<Vec<_>>()
        .join(" < ")
}

/// Shipped attribution radius, used when `[bench] hit_armor_id_max_offset_m` is
/// missing or nonsensical (`0`/negative/`NaN` would disable re-attribution
/// silently).
pub(crate) const DEFAULT_HIT_ARMOR_ID_MAX_OFFSET_M: f32 = 0.25;

fn sanitized_max_offset(configured: f32) -> f32 {
    if configured.is_finite() && configured > 0.0 {
        configured
    } else {
        DEFAULT_HIT_ARMOR_ID_MAX_OFFSET_M
    }
}

/// The plate whose centre is closest to a recovered contact point, with that
/// distance.
///
/// `plates` must yield the **hit robot's own** plates only — `(armor_id, centre)`
/// in the world frame, the same set `/sim/gt/armors` publishes for that robot.
/// `prefer` is the plate the collision event named; it wins a tie (within
/// 0.1 mm) so the attribution stays deterministic.
///
/// Returns `None` only when the robot has no plates at all.
pub(crate) fn nearest_plate_center(
    contact: Vec3,
    plates: impl IntoIterator<Item = (u32, Vec3)>,
    prefer: u32,
) -> Option<(u32, f32)> {
    let mut best: Option<(u32, f32)> = None;
    for (armor_id, center) in plates {
        let distance = center.distance(contact);
        let replace = match best {
            None => true,
            Some((_, best_distance)) => {
                distance < best_distance - 1e-4
                    || ((distance - best_distance).abs() <= 1e-4 && armor_id == prefer)
            }
        };
        if replace {
            best = Some((armor_id, distance));
        }
    }
    best
}

/// The plate ring of whichever robot a hit landed on, for `armor_id`
/// attribution (task-72).
///
/// Bundled as one `SystemParam` because `handle_shot_end` already sits at Bevy's
/// observer-parameter arity limit. The plate centre is read exactly like
/// `/sim/gt/armors` and `/tf armor_<N>` read it: the `*_CENTER` child when the
/// model has one, else the armour root itself.
#[derive(SystemParam)]
pub struct PlateRing<'w, 's> {
    /// Every armour root with the world transform of the root itself.
    pub roots: Query<'w, 's, (Entity, &'static GlobalTransform, &'static ArmorRoot)>,
    pub robot_ids: Query<'w, 's, &'static RobotId>,
    pub hierarchy: HierarchyQuery<'w, 's>,
    pub centers: Query<'w, 's, &'static GlobalTransform>,
}

impl PlateRing<'_, '_> {
    /// World-frame centre of one plate.
    fn center_of(&self, entity: Entity, fallback: Vec3) -> Vec3 {
        self.hierarchy
            .of(entity)
            .suffix("CENTER")
            .any()
            .one()
            .and_then(|center| self.centers.get(center).ok())
            .map(|center| center.translation())
            .unwrap_or(fallback)
    }
}

#[allow(clippy::too_many_arguments)]
pub fn handle_shot_end(
    event: On<CollisionEnd>,
    mut commands: Commands,
    time: Res<Time>,
    scenario: Option<Res<Scenario>>,
    stamp: Res<RosFrameStamp>,
    hit_pub: Res<TopicPublisher<SimHitEventTopic>>,
    projectiles: Query<
        (
            Entity,
            &Transform,
            Option<&LinearVelocity>,
            &ShotTag,
            Option<&ShotContact>,
        ),
        (
            With<Projectile>,
            Without<DartProjectile>,
            Without<ShotResolved>,
        ),
    >,
    armor_components: Query<(), With<Armor>>,
    plates: PlateRing,
    names: Query<&Name>,
    collider_aabbs: Query<(&GlobalTransform, Option<&ColliderAabb>)>,
    child_of: Query<&ChildOf>,
    config: Res<SimulationConfig>,
    armor_contacts: Res<ArmorContactSet>,
    mut resolved: ResMut<ResolvedShots>,
) {
    let projectile_body1 = event.body1.and_then(|body| projectiles.get(body).ok());
    let projectile_body2 = event.body2.and_then(|body| projectiles.get(body).ok());
    // Pick the *other* collider from the side that is **not** the projectile.
    // `event.body1.is_some()` would not do: it only says "collider1's entity has a
    // rigid body", so when the robot happens to be collider1 (its root is a rigid
    // body too) that test hands us the projectile's own collider — the `Armor`
    // ancestor search then fails and a real armour hit is reported as `hit_env`.
    // Whichever body resolves to a live projectile entity decides the other side.
    let (projectile, transform, velocity, shot, contact, other_collider) =
        match (projectile_body1, projectile_body2) {
            (Some((projectile, transform, velocity, shot, contact)), _) => (
                projectile,
                transform,
                velocity,
                shot,
                contact,
                event.collider2,
            ),
            (_, Some((projectile, transform, velocity, shot, contact))) => (
                projectile,
                transform,
                velocity,
                shot,
                contact,
                event.collider1,
            ),
            _ => return,
        };

    if hit_debug_enabled() {
        let initial = velocity.map(|velocity| to_ros_vec3(velocity.0));
        let (other_gt, other_aabb) = collider_aabbs
            .get(other_collider)
            .map(|(gt, aabb)| {
                (
                    Some(gt.translation()),
                    aabb.map(|aabb| (aabb.min, aabb.max)),
                )
            })
            .unwrap_or((None, None));
        info!(
            "[bench][hitdbg] seq={} proj={:?}[{}] point={:?} velocity={:?} body1={:?} body2={:?} c1={:?} c2={:?} other={:?} other_gt={:?} other_aabb={:?} chain={}",
            shot.shot_seq,
            projectile,
            names.get(projectile).map(|n| n.as_str()).unwrap_or("<unnamed>"),
            to_ros_vec3(transform.translation),
            initial,
            event.body1,
            event.body2,
            event.collider1,
            event.collider2,
            other_collider,
            other_gt,
            other_aabb,
            describe_chain(other_collider, &names, &armor_components, &child_of),
        );
    }

    let sim_time_ns = scenario_time_ns(scenario.as_deref());
    // `Time` (the app clock), not the scenario's fixed clock; both fire and
    // resolve are sampled from the same clock, so the difference is flight time.
    // There is no timeout path: a projectile that never collides publishes
    // nothing at all (it despawns after its 5 s lifetime), which is why an
    // unmatched `fire` sequence is the only evidence of a clean miss.
    let flight_time_s = (time.elapsed_secs_f64() - shot.fire_elapsed_s).max(0.0) as f32;
    // `CollisionEnd` carries no contact point; the projectile's own position at
    // the event is the honest answer, and its velocity is what the scorer needs
    // for the impact direction. The event fires when the contact *ends*, so this
    // position lags the true contact by up to one physics step — ≈208 mm at the
    // 120 Hz fixed rate and the 25 m/s muzzle speed (25 / 120). It is the same
    // event the legacy green-dot counter is built on, so the two readings share
    // one source of truth.
    let point = to_ros_vec3(transform.translation);
    let velocity = to_ros_vec3(velocity.map(|velocity| velocity.0).unwrap_or(Vec3::ZERO));

    // Armour test: the collider itself, or any ancestor, carries `Armor`
    // (`armor/construct.rs` tags the whole subtree of an armour root).
    let armor_entity = std::iter::once(other_collider)
        .chain(child_of.iter_ancestors(other_collider))
        .find(|entity| armor_components.contains(*entity));

    let Some(armor_entity) = armor_entity else {
        // A projectile that is *touching* a plate must not be settled by an
        // environment contact: the phantom speculative contact that avian
        // reports ends exactly when the plate bounce reverses the velocity (see
        // [`ArmorContactSet`]), so the real armour hit would otherwise be
        // swallowed here. The armour `CollisionEnd` publishes it a step later.
        if armor_contacts.contains(projectile) {
            return;
        }
        // `claim_env` is the synchronous guard: `ShotEnvHit` only becomes
        // visible after the next command flush, so a second environment
        // `CollisionEnd` in the same schedule run must be rejected here.
        if !resolved.claim_env(shot.shot_seq) {
            return;
        }
        // Not `ShotResolved`: an environment contact does not end the shot, a
        // later plate hit still publishes `hit_armor` (P1 semantics).
        commands.entity(projectile).insert(ShotEnvHit);
        hit_pub.publish(hit_env_event(
            &stamp,
            sim_time_ns,
            shot.shot_seq,
            shot.shooter_id,
            point,
            velocity,
            flight_time_s,
        ));
        return;
    };

    // The plate's `ArmorRoot` is the entity whose `ArmorId` names the `/tf` frame
    // `armor_<N>`, so the event and the TF frame agree by construction.
    let Some(armor_root_entity) = std::iter::once(armor_entity)
        .chain(child_of.iter_ancestors(armor_entity))
        .find(|entity| plates.roots.contains(*entity))
    else {
        return;
    };
    let Ok((_, _, armor_root)) = plates.roots.get(armor_root_entity) else {
        return;
    };
    // The plate the *collision event* named. Kept as the fallback: it is what
    // the pre-task-72 semantics published, and the only answer available when
    // the sweep has no contact point to judge against.
    let collider_armor_id = armor_root.id.as_usize() as u32;
    let target_id = crate::bench::gt::robot_of(armor_root_entity, &plates.robot_ids, &child_of)
        .map(|id| id.0)
        .unwrap_or(INVALID_ROBOT_ID);

    if !resolved.claim(shot.shot_seq) {
        return;
    }
    commands.entity(projectile).insert(ShotResolved);
    // The true contact point, recovered by the per-step sweep in
    // [`record_shot_contacts`]. `CollisionStart` cannot be used for this: with
    // avian's unbounded speculative margin it fires up to ~0.22 m early, which is
    // exactly why the sweep exists. A missing contact is published as all-NaN and
    // logged: it would mean the sweep and the collision event disagree.
    // `ShotContact` stores the point in the world (Bevy) frame; the event wants
    // map (ROS) coordinates, like `world_point` above.
    let contact_bevy = contact.and_then(|trace| trace.contact);
    let contact_point = contact_bevy.map(to_ros_vec3);
    if contact_point.is_none() {
        warn!(
            "[bench] shot {} resolved against armour {} without a recorded contact point",
            shot.shot_seq, collider_armor_id
        );
    }
    // Attribution (task-72). The collider is *not* authoritative: with the
    // unbounded speculative margin the collision can start on the neighbouring
    // plate — 90° away on the ≈0.212 m plate ring, i.e. 230–344 mm from where the
    // projectile really touched, while the touched plate is only 15–110 mm from
    // the sweep contact point (22-shot phase-bed measurement). Attribute by the
    // contact point instead, over the hit robot's own plates only (the same set
    // `/sim/gt/armors` publishes), and fall back to the collider when the nearest
    // centre is farther than `[bench] hit_armor_id_max_offset_m`.
    let max_offset = sanitized_max_offset(config.bench.hit_armor_id_max_offset_m);
    let armor_id = match contact_bevy {
        None => collider_armor_id,
        Some(point) => {
            let candidates = plates
                .roots
                .iter()
                .filter_map(|(entity, transform, root)| {
                    let owner =
                        crate::bench::gt::robot_of(entity, &plates.robot_ids, &child_of)
                            .map(|id| id.0);
                    if owner != Some(target_id) {
                        return None;
                    }
                    // Same point `/tf armor_<N>` and `/sim/gt/armors` publish: the
                    // `*_CENTER` child when the model has one, else the root.
                    let center = plates.center_of(entity, transform.translation());
                    Some((root.id.as_usize() as u32, center))
                });
            match nearest_plate_center(point, candidates, collider_armor_id) {
                Some((nearest, offset)) if offset <= max_offset => nearest,
                Some((nearest, offset)) => {
                    warn!(
                        "[bench][hitid] shot {}: nearest plate armour_{} is {:.3} m from the contact point (radius {:.2} m); keeping the collider's armour_{}",
                        shot.shot_seq, nearest, offset, max_offset, collider_armor_id
                    );
                    collider_armor_id
                }
                None => collider_armor_id,
            }
        }
    };
    if hit_debug_enabled() && armor_id != collider_armor_id {
        info!(
            "[bench][hitid] shot {}: contact point re-attributed armour_{} -> armour_{}",
            shot.shot_seq, collider_armor_id, armor_id
        );
    }
    hit_pub.publish(hit_armor_event(
        &stamp,
        sim_time_ns,
        shot.shot_seq,
        shot.shooter_id,
        target_id,
        armor_id,
        point,
        contact_point,
        velocity,
        flight_time_s,
    ));
}

/// Resolves the projectile side of a collision event and returns it with the
/// *other* collider (the same rule `handle_shot_end` uses).
fn projectile_and_other(
    body1: Option<Entity>,
    body2: Option<Entity>,
    collider1: Entity,
    collider2: Entity,
    projectiles: &Query<(), (With<Projectile>, Without<DartProjectile>)>,
) -> Option<(Entity, Entity)> {
    match (body1, body2) {
        (Some(body1), _) if projectiles.contains(body1) => Some((body1, collider2)),
        (_, Some(body2)) if projectiles.contains(body2) => Some((body2, collider1)),
        _ => None,
    }
}

/// Extra travel allowed by the **gated reverse sweep**, in physics steps.
///
/// The forward sweep only looks one step ahead of the *post-step* position
/// (208 mm at 25 m/s and 120 Hz). `avian3d` runs with an unbounded speculative
/// margin, so a contact can already be resolved while the plate is still up to
/// ~0.22 m away; on a glancing or spinning hit there is then no step at which a
/// still-forward velocity and the one-step window overlap, the forward sweep
/// never sees the plate, and the shot keeps a NaN `contact_point` (measured on
/// the phase bed: 25/54 kind=1 hits). Two steps (416 mm) cover that speculative
/// lead with margin.
///
/// [`RecentArmorContact`]'s TTL is **derived from this constant**
/// (`time.delta() * REVERSE_SWEEP_STEPS`): the gate must stay open exactly as long
/// as the reverse pass is willing to reach back, so the two change together.
pub(crate) const REVERSE_SWEEP_STEPS: f32 = 2.0;

/// Floor for the reverse sweep's travel budget, in metres.
///
/// The budget must **not** shrink with the post-bounce residual speed: on a
/// glancing or spinning hit the residual can be as low as 4.4 m/s (a 36 mm step),
/// which would cap two steps at 73 mm — while the plate the projectile just
/// bounced off is still up to one *pre-collision* step (~0.21 m at 25 m/s) behind
/// it. Scaling the reach with the post-collision speed is exactly the NaN tail
/// task-73 left behind (the five residual hits measured 4.4–7.5 m/s). The gate
/// already guarantees that a *real* plate contact happened a moment ago, so a
/// fixed one-lead-plus-margin budget is both safe and sufficient.
pub(crate) const REVERSE_SWEEP_MIN_DISTANCE_M: f32 = 0.30;

/// Travel budget of the gated reverse sweep, in metres.
///
/// `speed` is the projectile's *current* (post-bounce) speed and `step_secs` the
/// fixed timestep; a non-finite input falls back to the floor.
pub(crate) fn reverse_sweep_distance(speed: f32, step_secs: f32) -> f32 {
    let scaled = if speed.is_finite() && step_secs.is_finite() {
        speed * step_secs * REVERSE_SWEEP_STEPS
    } else {
        0.0
    };
    scaled.max(REVERSE_SWEEP_MIN_DISTANCE_M)
}

/// One recovered sweep candidate: the point on the plate, the collider entity
/// and the distance from the sweep origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SweepHit {
    pub point: Vec3,
    pub entity: Entity,
    pub distance: f32,
}

/// Picks the closer of the forward and the gated-reverse sweep hit.
///
/// The forward hit wins a tie (within 0.1 mm), mirroring
/// [`nearest_plate_center`]: recovering the same contact must not flip with
/// query order. `None` means neither direction found a plate, so the shot keeps
/// the pre-task-72 `collider` attribution.
pub(crate) fn closest_sweep_hit(
    forward: Option<SweepHit>,
    reverse: Option<SweepHit>,
) -> Option<SweepHit> {
    match (forward, reverse) {
        (Some(forward), None) => Some(forward),
        (None, Some(reverse)) => Some(reverse),
        (Some(forward), Some(reverse)) => {
            if reverse.distance < forward.distance - 1e-4 {
                Some(reverse)
            } else {
                Some(forward)
            }
        }
        (None, None) => None,
    }
}

/// The reverse sweep direction for a projectile, or `None` when the gate is
/// closed.
///
/// `in_armor_contact` is the gate: [`ArmorContactSet::contains`] **or** a recent
/// contact from [`RecentArmorContact`] (task-76 — a contact can end inside the
/// step it started, so "currently touching" alone is too strict). The gate is
/// what makes the reverse pass safe: its failure mode would be "attributed a
/// plate the projectile never touched", and that is structurally impossible while
/// the projectile touched a plate a step ago. `None` also covers a degenerate
/// velocity.
pub(crate) fn reverse_sweep_direction(velocity: Vec3, in_armor_contact: bool) -> Option<Dir3> {
    if !in_armor_contact {
        return None;
    }
    let speed = velocity.length();
    if !speed.is_finite() || speed <= 0.0 {
        return None;
    }
    Dir3::new(-velocity / speed).ok()
}

/// Recovers the true contact point of every projectile still in flight.
///
/// Runs after the physics step (`FixedPostUpdate`): each projectile carries the
/// position it had at the end of the previous step ([`ShotContact::prev`]), so the
/// segment `prev -> Transform.translation` is exactly what this step swept. A
/// shape cast of the projectile's own sphere along that segment, filtered to
/// armour colliders (the collider tree is asked for the closest armour hit; the
/// predicate uses the `Armor` tag `armor/construct.rs` puts on the plate's whole
/// subtree), gives the first true surface intersection. `point1` is the point on
/// the plate, i.e. the contact point the scorer wants; it is cached once per shot
/// ([`ShotContact::contact`]) and published with the `hit_armor` event.
///
/// A single forward cast is not always enough: `avian3d`'s unbounded speculative
/// margin can reverse the velocity while the plate is still outside the one-step
/// window, so a glancing or spinning hit is never seen (task-73). When the
/// projectile is currently touching armour — [`ArmorContactSet`], i.e. the
/// contact provably exists — a second cast is made back along `-velocity` and the
/// closer of the two candidates is kept ([`closest_sweep_hit`]).
///
/// Not behind `DAEDALUS_HIT_DEBUG`: the event's `contact_point` depends on it.
///
/// A contact that ends inside the step it started is no longer live by the time
/// this system runs, which is why the gate also accepts a recent contact
/// ([`RecentArmorContact`]); the cache is otherwise populated a step before
/// `handle_shot_end` can publish for that contact.
pub fn record_shot_contacts(
    spatial: SpatialQuery,
    mut commands: Commands,
    time: Res<Time<Fixed>>,
    contacts: Res<ArmorContactSet>,
    mut recent: ResMut<RecentArmorContact>,
    armor_components: Query<(), With<Armor>>,
    mut projectiles: Query<
        (
            Entity,
            &Transform,
            &Collider,
            &LinearVelocity,
            Option<&mut ShotContact>,
        ),
        (With<Projectile>, Without<DartProjectile>),
    >,
) {
    // One fixed step of flight. The sweep must use the *pre-collision*
    // velocity: on the step where the discrete solver already resolved the
    // hit, the position delta `cur - prev` is post-bounce and points away from
    // the plate, so the cast would miss it (measured on `daedalus-bench:p1r`:
    // the same cast along the projectile's velocity found the infantry plate
    // 0.1288 m ahead on that very step).
    let step = time.delta_secs();
    let now = time.elapsed();
    // A contact that ends inside the step it started is already gone from
    // `contacts` when this system runs (the collision events are dispatched
    // during the physics schedule, this system runs in `FixedPostUpdate`), so
    // the gate also honours a contact that ended within the last two steps.
    let recent_window = time.delta().saturating_mul(REVERSE_SWEEP_STEPS as u32);
    recent.prune(now, recent_window);
    for (entity, transform, collider, velocity, trace) in &mut projectiles {
        let current = transform.translation;
        // A projectile spawned after the last run of this system has no trace
        // yet: start it here so the next step has a segment to sweep.
        let Some(mut trace) = trace else {
            commands.entity(entity).insert(ShotContact::at(current));
            continue;
        };
        if trace.contact.is_none() {
            let speed = velocity.0.length();
            if speed > 1.0 {
                if let Ok(direction) = Dir3::new(velocity.0 / speed) {
                    let sweep = |direction: Dir3, max_distance: f32| {
                        let config = ShapeCastConfig {
                            max_distance,
                            target_distance: 0.0,
                            compute_contact_on_penetration: true,
                            ignore_origin_penetration: false,
                        };
                        spatial.cast_shape_predicate(
                            collider,
                            current,
                            Quat::IDENTITY,
                            direction,
                            &config,
                            &SpatialQueryFilter::default(),
                            &|candidate| armor_components.contains(candidate),
                        )
                    };
                    let forward_raw = sweep(direction, speed * step);
                    // Gated reverse pass (task-73, widened in task-76): the
                    // post-collision velocity points away from the plate, so a
                    // projectile that touched armour a moment ago can be swept
                    // backwards to recover the surface the forward pass can no
                    // longer see. The gate — live contact *or* a contact that
                    // ended within the last two steps — is what keeps this free
                    // of false positives, and the budget has a floor so a low
                    // post-bounce residual cannot shrink the reach.
                    let gate = contacts.contains(entity)
                        || recent.is_recent(entity, now, recent_window);
                    let reverse_budget = reverse_sweep_distance(speed, step);
                    let reverse_raw = reverse_sweep_direction(velocity.0, gate)
                        .and_then(|direction| sweep(direction, reverse_budget));
                    if hit_debug_enabled() {
                        for (label, hit) in [
                            ("fwd", forward_raw.as_ref()),
                            ("rev", reverse_raw.as_ref()),
                        ] {
                            if let Some(hit) = hit {
                                info!(
                                    "[bench][castdbg] proj={entity} {label} speed={speed:.2} step={step:.5} max={:.4} gate={gate} dist={:.5} point1={:?} point2={:?} n1={:?} origin={:?} target={:?}",
                                    if label == "fwd" {
                                        speed * step
                                    } else {
                                        reverse_budget
                                    },
                                    hit.distance,
                                    hit.point1,
                                    hit.point2,
                                    hit.normal1,
                                    current,
                                    hit.entity
                                );
                            }
                        }
                    }
                    let forward = forward_raw.map(|hit| SweepHit {
                        point: hit.point1,
                        entity: hit.entity,
                        distance: hit.distance,
                    });
                    let reverse = reverse_raw.map(|hit| SweepHit {
                        point: hit.point1,
                        entity: hit.entity,
                        distance: hit.distance,
                    });
                    if let Some(hit) = closest_sweep_hit(forward, reverse) {
                        trace.contact = Some(hit.point);
                        trace.collider = Some(hit.entity);
                        if hit_debug_enabled() {
                            info!(
                                "[bench][castdbg] proj={entity} chose dist={:.5} point1={:?} target={:?}",
                                hit.distance, hit.point, hit.entity
                            );
                        }
                    }
                }
            }
        }
        trace.prev = current;
    }
}

/// Adds a projectile to [`ArmorContactSet`] when it starts touching a plate.
///
/// Registered in every bench build — the environment veto in `handle_shot_end`
/// depends on it, so it is not behind `DAEDALUS_HIT_DEBUG`. The armour collider
/// entity carries `Armor` itself (the whole armour-root subtree is tagged by
/// `armor/construct.rs`), so a direct component check is enough.
pub fn track_armor_contact_start(
    event: On<CollisionStart>,
    mut contacts: ResMut<ArmorContactSet>,
    projectiles: Query<(), (With<Projectile>, Without<DartProjectile>)>,
    armor_components: Query<(), With<Armor>>,
) {
    if let Some((projectile, other)) = projectile_and_other(
        event.body1,
        event.body2,
        event.collider1,
        event.collider2,
        &projectiles,
    ) {
        if armor_components.contains(other) {
            contacts.0.insert(projectile);
        }
    }
}

/// Removes a projectile from [`ArmorContactSet`] when the plate contact ends, and
/// notes the moment in [`RecentArmorContact`] so the reverse sweep stays gated on
/// for [`REVERSE_SWEEP_STEPS`] more steps (task-76: a contact can end inside the
/// step it started, and then this event lands before `record_shot_contacts` runs).
pub fn track_armor_contact_end(
    event: On<CollisionEnd>,
    mut contacts: ResMut<ArmorContactSet>,
    mut recent: ResMut<RecentArmorContact>,
    time: Res<Time<Fixed>>,
    projectiles: Query<(), (With<Projectile>, Without<DartProjectile>)>,
    armor_components: Query<(), With<Armor>>,
) {
    if let Some((projectile, other)) = projectile_and_other(
        event.body1,
        event.body2,
        event.collider1,
        event.collider2,
        &projectiles,
    ) {
        if armor_components.contains(other) {
            contacts.0.remove(&projectile);
            recent.note_contact_end(projectile, time.elapsed());
        }
    }
}

/// Debug-only: log every collision **start** that involves a live projectile.
///
/// Registered only when `DAEDALUS_HIT_DEBUG` is set (see `BenchPlugin`), so it costs
/// nothing in a normal run. It answers the one question the `CollisionEnd` stream
/// cannot: did an *armour* contact start for this shot at all, and on which step did
/// it start relative to the environment contact that resolved the shot?
pub fn debug_shot_start(
    event: On<CollisionStart>,
    projectiles: Query<Entity, (With<Projectile>, Without<DartProjectile>)>,
    shot_tags: Query<&ShotTag>,
    armor_components: Query<(), With<Armor>>,
    names: Query<&Name>,
    child_of: Query<&ChildOf>,
) {
    let (projectile, other_collider) = match (event.body1, event.body2) {
        (Some(body1), _) if projectiles.contains(body1) => (body1, event.collider2),
        (_, Some(body2)) if projectiles.contains(body2) => (body2, event.collider1),
        _ => return,
    };
    let seq = shot_tags.get(projectile).map(|tag| tag.shot_seq).unwrap_or(0);
    info!(
        "[bench][hitdbg] START seq={} other={:?} chain={}",
        seq,
        other_collider,
        describe_chain(other_collider, &names, &armor_components, &child_of),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stamp() -> RosFrameStamp {
        RosFrameStamp(r2r::builtin_interfaces::msg::Time {
            sec: 12,
            nanosec: 345,
        })
    }

    #[test]
    fn an_environment_hit_does_not_close_the_shot_for_armour() {
        let mut resolved = ResolvedShots::default();

        // `hit_env` may only be published once per shot...
        assert!(resolved.claim_env(7), "first environment contact of shot 7");
        assert!(
            !resolved.claim_env(7),
            "a second environment contact of the same shot must not publish again"
        );
        // ...but it must not stop the later armour hit (P1), and an
        // environment-only shot is counted separately from a resolved one.
        assert!(resolved.claim(7), "the plate contact still resolves shot 7");
        assert_eq!(resolved.resolved(), 1);
        assert_eq!(resolved.env_only(), 0);

        assert!(resolved.claim_env(8), "shot 8 clipped the floor");
        assert_eq!(resolved.env_only(), 1, "shot 8 never reached a plate");
    }

    #[test]
    fn a_shot_resolves_exactly_once() {
        let mut resolved = ResolvedShots::default();

        assert!(resolved.claim(7), "first resolution of shot 7");
        assert!(
            !resolved.claim(7),
            "a second collision of the same shot must not publish again"
        );
        assert!(resolved.claim(8), "the next shot still resolves");
        assert_eq!(resolved.resolved(), 2);
    }

    #[test]
    fn shot_sequence_starts_at_one_and_only_increases() {
        let sequence = ShotSequence::default();

        assert_eq!(sequence.next(), 1);
        assert_eq!(sequence.next(), 2);
        assert_eq!(sequence.allocated(), 2);
    }

    #[test]
    fn fire_carries_the_muzzle_and_the_initial_velocity() {
        let event = fire_event(
            &stamp(),
            1_500_000_000,
            7,
            0,
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(0.0, 0.0, -25.0),
        );

        assert_eq!(event.kind, KIND_FIRE);
        assert_eq!(event.shot_seq, 7);
        assert_eq!(event.sim_time_ns, 1_500_000_000);
        assert_eq!(event.world_point, vec![1.0, 2.0, 3.0]);
        assert_eq!(event.velocity, vec![0.0, 0.0, -25.0]);
        assert_eq!(event.armor_id, INVALID_ARMOR_ID);
        assert_eq!(
            event.contact_point.len(),
            3,
            "contact_point is a fixed 3-element array"
        );
        assert!(
            event.contact_point.iter().all(|value| value.is_nan()),
            "kind=0 has no contact point: NaN, never zeros (0 is a real coordinate)"
        );
        assert_eq!(
            event.target_id, INVALID_ROBOT_ID,
            "kind=0 carries no target; 0 would name the ego"
        );
        assert_eq!(event.header.stamp.sec, 12);
        assert_eq!(event.header.frame_id, "map");
    }

    #[test]
    fn hit_armor_names_the_plate_and_its_owner() {
        let event = hit_armor_event(
            &stamp(),
            2_000_000_000,
            7,
            0,
            2,
            15,
            Vec3::new(1.0, 0.5, 2.0),
            Some(Vec3::new(3.0, 0.25, -1.5)),
            Vec3::new(0.0, 0.0, -24.0),
            0.081,
        );

        assert_eq!(event.kind, KIND_HIT_ARMOR);
        assert_eq!(event.shot_seq, 7, "the fire's shot_seq, for pairing");
        assert_eq!(event.target_id, 2);
        assert_eq!(event.armor_id, 15);
        assert!((event.flight_time_s - 0.081).abs() < 1e-6);
        assert_eq!(
            event.world_point,
            vec![1.0, 0.5, 2.0],
            "world_point stays the resolving frame's projectile position"
        );
        assert_eq!(
            event.contact_point,
            vec![3.0, 0.25, -1.5],
            "contact_point is the true plate contact, already in the map frame"
        );
    }

    #[test]
    fn a_hit_without_a_recovered_contact_point_is_nan() {
        let event = hit_armor_event(
            &stamp(),
            2_000_000_000,
            7,
            0,
            2,
            15,
            Vec3::new(1.0, 0.5, 2.0),
            None,
            Vec3::new(0.0, 0.0, -24.0),
            0.081,
        );

        assert_eq!(event.kind, KIND_HIT_ARMOR);
        assert!(
            event.contact_point.iter().all(|value| value.is_nan()),
            "an unrecoverable contact point must not look like (0, 0, 0)"
        );
    }

    #[test]
    fn hit_env_has_no_target_and_no_armour() {
        let event = hit_env_event(
            &stamp(),
            3_000_000_000,
            9,
            0,
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::ZERO,
            0.4,
        );

        assert_eq!(event.kind, KIND_HIT_ENV);
        assert_eq!(event.target_id, INVALID_ROBOT_ID);
        assert_eq!(event.armor_id, INVALID_ARMOR_ID);
        assert!(
            event.contact_point.iter().all(|value| value.is_nan()),
            "kind=2 has no contact point: NaN, never zeros"
        );
    }

    /// The infantry plate ring in the numbers `/sim/gt/armors` publishes:
    /// armour ids 10..13 (construction order), centres ≈0.212 m from the axis.
    fn infantry_ring() -> Vec<(u32, Vec3)> {
        let radius = 0.212_f32;
        vec![
            (10, Vec3::new(0.0, 0.0, radius)),  // front
            (11, Vec3::new(0.0, 0.0, -radius)), // rear
            (12, Vec3::new(-radius, 0.0, 0.0)), // left
            (13, Vec3::new(radius, 0.0, 0.0)),  // right
        ]
    }

    fn distance_to(plates: &[(u32, Vec3)], armor_id: u32, point: Vec3) -> f32 {
        plates
            .iter()
            .find(|(id, _)| *id == armor_id)
            .expect("plate exists")
            .1
            .distance(point)
    }

    #[test]
    fn a_contact_on_a_plate_face_is_attributed_to_that_plate() {
        let plates = infantry_ring();
        // 20 mm inside the front plate's centre, along that plate's normal.
        let contact = Vec3::new(0.0, 0.0, 0.192);

        let (armor_id, offset) =
            nearest_plate_center(contact, plates, 13).expect("the robot has plates");

        assert_eq!(armor_id, 10, "the contact sits on the front plate");
        assert!((offset - 0.020).abs() < 1e-5, "offset {offset}");
        assert!(
            offset <= DEFAULT_HIT_ARMOR_ID_MAX_OFFSET_M,
            "a real hit stays inside the attribution radius"
        );
    }

    #[test]
    fn a_glancing_contact_picks_the_near_plate_not_the_early_collider_one() {
        let plates = infantry_ring();
        // Phase-bed geometry: the speculative collision starts on the 90°
        // neighbour (armour 13, reported by the collider) ≈0.27 m from where the
        // projectile really touches the front plate (armour 10, 35 mm away).
        let contact = Vec3::new(0.030, 0.0, 0.195);
        assert!(
            distance_to(&plates, 13, contact) > 0.25,
            "the collider's neighbour really is outside the radius"
        );

        let (armor_id, offset) =
            nearest_plate_center(contact, plates.clone(), 13).expect("the robot has plates");

        assert_eq!(
            armor_id, 10,
            "the contact point, not the collision entity, decides the plate"
        );
        assert!(offset < 0.05, "offset {offset}");
    }

    #[test]
    fn a_tie_keeps_the_collider_plate_so_attribution_stays_deterministic() {
        // Two plates equidistant from the contact: the collider's own plate wins.
        let plates = vec![(13, Vec3::new(0.1, 0.0, 0.0)), (10, Vec3::new(-0.1, 0.0, 0.0))];

        let (armor_id, _) = nearest_plate_center(Vec3::ZERO, plates, 10).expect("plates");

        assert_eq!(armor_id, 10);
    }

    #[test]
    fn a_contact_far_from_every_plate_falls_back_to_the_collider() {
        let plates = infantry_ring();
        // 0.4 m in front of the whole ring: nothing is within the radius, so
        // `handle_shot_end` publishes the collider's plate and logs the fallback.
        let contact = Vec3::new(0.0, 0.0, 0.612);

        let (armor_id, offset) =
            nearest_plate_center(contact, plates, 13).expect("the robot has plates");

        assert_eq!(armor_id, 10, "the nearest plate is still reported for the log");
        assert!(
            offset > DEFAULT_HIT_ARMOR_ID_MAX_OFFSET_M,
            "outside the radius ⇒ fallback: offset {offset}"
        );
    }

    #[test]
    fn a_robot_without_plates_yields_no_attribution() {
        assert!(nearest_plate_center(Vec3::ZERO, Vec::new(), 10).is_none());
    }

    #[test]
    fn a_nonsensical_attribution_radius_falls_back_to_the_shipped_value() {
        assert_eq!(
            sanitized_max_offset(0.0),
            DEFAULT_HIT_ARMOR_ID_MAX_OFFSET_M
        );
        assert_eq!(
            sanitized_max_offset(-1.0),
            DEFAULT_HIT_ARMOR_ID_MAX_OFFSET_M
        );
        assert_eq!(
            sanitized_max_offset(f32::NAN),
            DEFAULT_HIT_ARMOR_ID_MAX_OFFSET_M
        );
        assert_eq!(sanitized_max_offset(0.4), 0.4);
    }

    /// A throwaway entity id for the sweep-candidate tests.
    fn sweep_target() -> Entity {
        World::new().spawn_empty().id()
    }

    #[test]
    fn a_forward_sweep_hit_is_kept_when_the_reverse_pass_is_farther() {
        let entity = sweep_target();
        let forward = SweepHit {
            point: Vec3::new(0.0, 0.2, 2.0),
            entity,
            distance: 0.05,
        };
        let reverse = SweepHit {
            point: Vec3::new(0.0, 0.2, 2.4),
            entity,
            distance: 0.41,
        };

        let hit = closest_sweep_hit(Some(forward), Some(reverse)).expect("a recovered contact");

        assert_eq!(hit, forward, "the forward pass is closer");
    }

    #[test]
    fn a_closer_reverse_hit_wins_over_the_forward_one() {
        let entity = sweep_target();
        let forward = SweepHit {
            point: Vec3::new(0.0, 0.2, 1.9),
            entity,
            distance: 0.31,
        };
        // The task-73 case: the velocity was already reversed while the plate
        // was still 0.3 m out, so only the reverse pass reaches the surface.
        let reverse = SweepHit {
            point: Vec3::new(0.0, 0.2, 2.0),
            entity,
            distance: 0.02,
        };

        let hit = closest_sweep_hit(Some(forward), Some(reverse)).expect("a recovered contact");

        assert_eq!(hit, reverse, "the plate the projectile is touching wins");
    }

    #[test]
    fn a_tie_between_the_two_directions_keeps_the_forward_hit() {
        let entity = sweep_target();
        let forward = SweepHit {
            point: Vec3::new(0.0, 0.2, 2.0),
            entity,
            distance: 0.2,
        };
        let reverse = SweepHit {
            point: Vec3::new(0.0, 0.2, 2.0),
            entity,
            distance: 0.2 + 5e-5,
        };

        let hit = closest_sweep_hit(Some(forward), Some(reverse)).expect("a recovered contact");

        assert_eq!(hit, forward, "a sub-0.1 mm difference is a tie");
    }

    #[test]
    fn a_reverse_only_hit_is_used_when_the_forward_pass_missed() {
        let reverse = SweepHit {
            point: Vec3::new(0.0, 0.2, 2.0),
            entity: sweep_target(),
            distance: 0.1,
        };

        assert_eq!(closest_sweep_hit(None, Some(reverse)), Some(reverse));
    }

    #[test]
    fn neither_sweep_direction_found_a_plate_so_the_collider_attribution_stands() {
        assert!(closest_sweep_hit(None, None).is_none());
    }

    #[test]
    fn the_reverse_pass_needs_a_live_armour_contact() {
        let velocity = Vec3::new(0.0, 0.0, 25.0);

        assert!(
            reverse_sweep_direction(velocity, false).is_none(),
            "a projectile that is not touching a plate must never sweep backwards"
        );

        let direction = reverse_sweep_direction(velocity, true).expect("the gate is open");
        assert!(
            direction.as_vec3().dot(Vec3::Z) < -0.999,
            "the reverse pass looks back along the incoming path"
        );

        assert!(
            reverse_sweep_direction(Vec3::ZERO, true).is_none(),
            "a degenerate velocity has no direction"
        );
    }

    #[test]
    fn the_reverse_sweep_allowance_covers_the_speculative_lead() {
        // 25 m/s at 120 Hz = 208 mm per step, and `avian3d`'s unbounded
        // speculative margin resolves armour contacts up to ~0.22 m early: one
        // step is not enough to reach back to the surface (the measured 46% NaN
        // rate on the phase bed).
        const SPEED_MPS: f32 = 25.0;
        const STEP_S: f32 = 1.0 / 120.0;
        const SPECULATIVE_LEAD_M: f32 = 0.22;

        assert!(
            SPEED_MPS * STEP_S * REVERSE_SWEEP_STEPS > SPECULATIVE_LEAD_M,
            "two steps must reach past the speculative lead"
        );
    }

    #[test]
    fn a_contact_that_ended_a_step_ago_still_opens_the_reverse_gate() {
        const STEP: Duration = Duration::from_nanos(8_333_333);
        let window = STEP * (REVERSE_SWEEP_STEPS as u32);
        let projectile = sweep_target();
        let ended = Duration::from_secs(1);
        let mut recent = RecentArmorContact::default();
        recent.note_contact_end(projectile, ended);

        // A contact that starts and ends inside one physics step is already gone
        // from `ArmorContactSet` when the sweep runs, but the reverse pass must
        // still be allowed on that step and the next one.
        assert!(
            recent.is_recent(projectile, ended, window),
            "the step the contact ended on is inside the TTL"
        );
        assert!(
            recent.is_recent(projectile, ended + STEP, window),
            "one step after the contact ended the reverse sweep must still run"
        );
        assert!(
            reverse_sweep_direction(Vec3::new(0.0, 0.0, -4.4), recent.is_recent(projectile, ended + STEP, window))
                .is_some(),
            "the widened gate opens the reverse direction for a low residual hit"
        );
        // Past the TTL the shot must fall back to the collider attribution.
        assert!(
            !recent.is_recent(projectile, ended + STEP * 3, window),
            "past the TTL the collider attribution stands"
        );
        assert!(
            reverse_sweep_direction(Vec3::new(0.0, 0.0, -4.4), false).is_none(),
            "a closed gate yields no reverse direction"
        );
    }

    #[test]
    fn pruning_forgets_contacts_past_the_ttl() {
        const STEP: Duration = Duration::from_nanos(8_333_333);
        let window = STEP * (REVERSE_SWEEP_STEPS as u32);
        let projectile = sweep_target();
        let mut recent = RecentArmorContact::default();
        recent.note_contact_end(projectile, Duration::from_secs(1));

        recent.prune(Duration::from_secs(1) + STEP, window);
        assert_eq!(recent.len(), 1, "an entry inside the TTL survives");

        recent.prune(Duration::from_secs(1) + STEP * 4, window);
        assert_eq!(recent.len(), 0, "an entry past the TTL is dropped");
    }

    #[test]
    fn the_reverse_budget_has_a_floor_for_low_residual_speeds() {
        const STEP_S: f32 = 1.0 / 120.0;
        const SPECULATIVE_LEAD_M: f32 = 0.22;

        // The five residual NaN shots of the phase bed measured 4.38 .. 7.53 m/s
        // *after* the bounce: scaling the budget with that speed would cap two
        // steps at 73..125 mm and never reach the plate.
        for residual in [4.38_f32, 5.27, 6.27, 6.79, 7.53] {
            let budget = reverse_sweep_distance(residual, STEP_S);
            assert_eq!(
                budget, REVERSE_SWEEP_MIN_DISTANCE_M,
                "the floor is what makes {residual} m/s reach the plate"
            );
            assert!(
                budget > SPECULATIVE_LEAD_M,
                "the budget must still clear the speculative lead at {residual} m/s"
            );
        }

        assert!(
            reverse_sweep_distance(25.0, STEP_S) > REVERSE_SWEEP_MIN_DISTANCE_M,
            "a full-speed projectile keeps its scaled budget"
        );
        assert_eq!(
            reverse_sweep_distance(f32::NAN, STEP_S),
            REVERSE_SWEEP_MIN_DISTANCE_M,
            "a non-finite speed falls back to the floor, never to NaN"
        );
        assert_eq!(
            reverse_sweep_distance(25.0, f32::INFINITY),
            REVERSE_SWEEP_MIN_DISTANCE_M,
            "a non-finite step falls back to the floor"
        );
    }
}
