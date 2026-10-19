//! Scene load order.
//!
//! glTF worlds become ready on whatever frame their asset happens to finish loading, so spawning
//! everything at once lets `setup_vehicle` turn a robot into a dynamic rigid body before the
//! environment has any colliders — and the robot falls through the floor. Loading is therefore one
//! linear task: each `spawn` resolves when that world's instance is ready, and the robots are
//! spawned last.
//!
//! Per-world setup (naming, collision layers, gimbal wiring) stays in the `WorldInstanceReady`
//! observers, because worlds spawned at runtime — projectiles, UAVs — need it too. What used to be
//! implicit ordering *between* those observers is now explicit here.

use avian3d::prelude::*;
use bevy::prelude::*;
use bevy::world_serialization::{InstanceId, WorldAssetRoot, WorldInstance};
use std::collections::HashMap;

use crate::components::{
    ActiveSlapper, Controlled, GameLayer, GroundRoot, Infantry, PreciousCollision, RobotId,
    SlapperInfantry,
};
use crate::robomaster::power_rune::construct::setup_power_rune;
use crate::robomaster::prelude::{
    HERO_ROBOT_CONFIG, INFANTRY_THREE_CONFIG, PowerRuneRoot, Team, TechCoreRoot,
};
use crate::robomaster::tech_core::construct::setup_tech_core;
use crate::setup::{
    ScanOutpost, setup_collision, setup_dart_launch, setup_outposts, setup_vehicle,
};
use crate::util::async_world::{AsyncWorld, AsyncWorldTask, drive_async_world};

/// Resolves once Avian has built every collider `setup_collision` queued under `root`.
///
/// `setup_collision` puts a [`ColliderConstructorHierarchy`] on the named descendants of a world as
/// soon as its instance is ready, so by the time this runs the queue for `root` is complete.
/// Selecting and observing happen in one world job, so constructors finishing early are simply not
/// selected rather than awaited forever.
async fn colliders_ready(w: &AsyncWorld, root: Entity) {
    w.observe_all::<ColliderConstructorHierarchyReady, _>(move |world| {
        let mut queued = Vec::new();
        collect_descendants(world, root, &mut queued);
        queued.retain(|entity| world.get::<ColliderConstructorHierarchy>(*entity).is_some());
        queued
    })
    .await;
}

/// The instance id a loaded world spawned under, needed to enumerate its entities by name.
async fn instance_of(w: &AsyncWorld, root: Entity) -> InstanceId {
    w.with_world(move |world| {
        **world
            .get::<WorldInstance>(root)
            .expect("world instance is ready, so its id must exist")
    })
    .await
}

fn collect_descendants(world: &World, entity: Entity, out: &mut Vec<Entity>) {
    let Some(children) = world.get::<Children>(entity) else {
        return;
    };
    let children: Vec<Entity> = children.iter().collect();
    for child in children {
        out.push(child);
        collect_descendants(world, child, out);
    }
}

pub struct ScenePlugin;

impl Plugin for ScenePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(AsyncWorldTask::new(load_scene))
            .add_systems(
                PreUpdate,
                drive_async_world.run_if(resource_exists::<AsyncWorldTask>),
            );
    }
}

async fn load_scene(w: AsyncWorld) {
    let assets = w
        .with_world(|world| world.resource::<AssetServer>().clone())
        .await;
    let scene =
        |path: &'static str| WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset(path)));

    let layers = GameLayer::environment_collision_layers();
    let trimesh = || {
        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMeshWithConfig(
            TrimeshFlags::all(),
        ))
        .with_default_layers(layers)
    };
    let static_trimesh = || {
        (
            trimesh(),
            layers,
            Visibility::Visible,
            Some(RigidBody::Static),
        )
    };

    // Environment first. Every robot below lands on what these build.
    let ground = w
        .spawn(scene("GROUND.glb"), (GroundRoot, Friction::new(0.5)))
        .await;
    w.run(setup_dart_launch, ground).await;
    w.run(
        setup_collision,
        (
            ground,
            PreciousCollision(HashMap::from([(
                "GROUND_DENSE".to_string(),
                static_trimesh(),
            )])),
        ),
    )
    .await;
    colliders_ready(&w, ground).await;

    w.spawn(
        scene("CALIB.glb"),
        Transform::IDENTITY.with_translation(Vec3::new(1.0, 2.5, 1.0)),
    )
    .await;

    w.spawn(
        scene("CALIB.glb"),
        Transform::IDENTITY.with_translation(Vec3::new(2.0, 0.5, 2.0)),
    )
    .await;

    let outpost = w
        .spawn(scene("OUTPOST.glb"), (RigidBody::Static, ScanOutpost))
        .await;
    w.run(setup_outposts, outpost).await;

    let tech_core = w.spawn(scene("TECH_CORE.glb"), TechCoreRoot).await;
    w.run(
        setup_tech_core,
        (tech_core, instance_of(&w, tech_core).await),
    )
    .await;
    w.run(
        setup_collision,
        (
            tech_core,
            PreciousCollision(HashMap::from([("GROUND".to_string(), static_trimesh())])),
        ),
    )
    .await;
    colliders_ready(&w, tech_core).await;

    let power_rune = w
        .spawn(
            scene("POWER.glb"),
            (
                RigidBody::Static,
                CollisionMargin(0.001),
                Restitution::ZERO,
                PowerRuneRoot,
            ),
        )
        .await;
    w.run(
        setup_power_rune,
        (power_rune, instance_of(&w, power_rune).await),
    )
    .await;
    w.run(
        setup_collision,
        (power_rune, PreciousCollision(power_rune_collision(layers))),
    )
    .await;
    colliders_ready(&w, power_rune).await;

    // Robots last, so they can never become dynamic over an empty world.
    //
    // Player start pose, tuned for the ROS 2 auto-aim loop: from here two enemy
    // robots (armour numbers "1" and "3") sit ~1.8-2.0 m ahead and square in the
    // camera, so `armor_detector` sees them immediately without anyone having to
    // drive anywhere first.
    //
    // Captured from a live session rather than guessed: the world pose
    // (gimbal translation + gimbal/camera +Z) was read off `/tf`, and the chassis
    // pose solved from it using the gimbal mount offset measured at the previous
    // spawn. Position is Bevy world (y up); the rotation is a quaternion on
    // purpose — the chassis there is pitched, so Euler angles would have to be
    // re-derived against Bevy's rotation order.
    let player = w
        .spawn(
            scene("vehicle.glb"),
            (
                Transform::from_xyz(0.1433, 0.4199, 3.1117).with_rotation(Quat::from_xyzw(
                    -0.26040781,
                    -0.21265314,
                    -0.05930884,
                    0.93991959,
                )),
                Infantry::new(Team::Red, INFANTRY_THREE_CONFIG),
                Controlled,
                RobotId(0),
            ),
        )
        .await;
    w.run(setup_vehicle, player).await;

    let slapper = w
        .spawn(
            scene("vehicle.glb"),
            (
                Transform::from_xyz(1.0, 1.0, 1.0),
                Infantry::new(Team::Blue, INFANTRY_THREE_CONFIG),
                SlapperInfantry,
                RobotId(1),
            ),
        )
        .await;
    w.run(setup_vehicle, slapper).await;

    let hero = w
        .spawn(
            scene("HERO.glb"),
            (
                Transform::from_xyz(2.0, 1.0, 1.0),
                Infantry::new(Team::Blue, HERO_ROBOT_CONFIG),
                SlapperInfantry,
                ActiveSlapper,
                RobotId(2),
            ),
        )
        .await;
    w.run(setup_vehicle, hero).await;

    info!("scene loaded");
}

/// Every rune target gets a voxelized collider so hits register on the spinning arms.
fn power_rune_collision(
    layers: CollisionLayers,
) -> HashMap<
    String,
    (
        ColliderConstructorHierarchy,
        CollisionLayers,
        Visibility,
        Option<RigidBody>,
    ),
> {
    let voxel = |size| {
        ColliderConstructorHierarchy::new(ColliderConstructor::VoxelizedTrimeshFromMesh {
            voxel_size: size,
            fill_mode: FillMode::FloodFill {
                detect_cavities: true,
            },
        })
        .with_default_layers(layers)
    };

    let mut collision = HashMap::from([(
        "BASE".to_string(),
        (
            ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMeshWithConfig(
                TrimeshFlags::all(),
            ))
            .with_default_layers(layers),
            layers,
            Visibility::Visible,
            Some(RigidBody::Static),
        ),
    )]);
    for face in 1..=2 {
        for target in 1..=5 {
            for state in ["ACTIVATED", "ACTIVE", "COMPLETED", "DISABLED"] {
                collision.insert(
                    format!("FACE_{face}_TARGET_{target}_{state}"),
                    (voxel(0.015), layers, Visibility::Visible, None),
                );
            }
        }
    }
    collision
}
