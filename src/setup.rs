use avian3d::prelude::*;
use bevy::anti_alias::fxaa::Fxaa;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::{EguiGlobalSettings, PrimaryEguiContext};

use crate::components::{
    ActiveSlapper, Controlled, DartLaunch, GameLayer, Infantry, InfantryChassis, InfantryGimbal,
    InfantryLaunchOffset, InfantryViewOffset, MainCamera, PreciousCollision, SlapperInfantry,
};
use crate::config::SimulationConfig;
use crate::robomaster::prelude::{OutpostRoot, ScanArmor, Team};
use crate::robomaster::vehicle::movement::VehicleDynamic;
use crate::util::entity_query::HierarchyQuery;
use bevy_metalfx::{MetalFxTemporalUpscaling, UpscaleFactor};

#[derive(Component)]
pub struct ScanOutpost;

pub fn setup(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    egui_global_settings: Option<ResMut<EguiGlobalSettings>>,
) {
    if let Some(mut egui_global_settings) = egui_global_settings {
        egui_global_settings.auto_create_primary_context = false;
    }
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(0.9, 0.95, 1.0),
            illuminance: config.render.illuminance,
            shadow_maps_enabled: config.render.shadows,
            contact_shadows_enabled: config.render.shadows,
            ..default()
        },
        Transform::from_xyz(0.0, 4.0, 0.0).looking_at(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0)),
    ));

    // Scene assets are spawned in load order by `ScenePlugin`, not here.

    let mut main_camera = commands.spawn((
        Camera3d::default(),
        Camera {
            // When Talos/ROS2 capture is enabled, the actual on-screen preview is a UI blit of the
            // off-screen capture texture. Keep this camera inactive to avoid rendering twice.
            #[cfg(any(feature = "ros2", feature = "talos"))]
            is_active: false,
            #[cfg(not(any(feature = "ros2", feature = "talos")))]
            is_active: config.preview.enabled,
            // clear_color: ClearColorConfig::Custom(Color::BLACK),
            ..default()
        },
        Projection::Perspective(PerspectiveProjection {
            fov: config.camera.fov.to_radians(),
            near: 0.1,
            far: 500000000.0,
            ..default()
        }),
        Tonemapping::None,
        Msaa::Off,
        Transform::from_xyz(0.0, 10.0, 15.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
        MainCamera {
            follow_offset: Vec3::from_array(config.camera.follow_offset),
        },
    ));
    if cfg!(target_os = "macos") && config.render.metalfx_temporal {
        main_camera.insert(MetalFxTemporalUpscaling {
            factor: UpscaleFactor::clamped(config.render.metalfx_scale),
            frame_generation: config.render.metalfx_frame_generation,
        });
    } else if config.render.main_camera_fxaa {
        main_camera.insert(Fxaa::default());
    }
    if config.debug.egui {
        main_camera.insert(PrimaryEguiContext);
    }
    #[cfg(any(feature = "ros2", feature = "talos"))]
    main_camera.insert(crate::capture::CaptureSource);
}

/// Tags the two outposts inside `OUTPOST.glb`.
pub fn setup_outposts(
    In(root): In<Entity>,
    mut commands: Commands,
    children: Query<&Children>,
    name: Query<&Name>,
) {
    children.iter_descendants(root).for_each(|e| {
        let Ok(name) = name.get(e) else {
            return;
        };
        if name.as_str() == "OUTPOST_1" {
            commands.entity(e).insert(OutpostRoot::new(Team::Red));
        }
        if name.as_str() == "OUTPOST_2" {
            commands.entity(e).insert(OutpostRoot::new(Team::Blue));
        }
    })
}

/// Finds the dart launch marker inside `GROUND.glb`.
pub fn setup_dart_launch(
    In(root): In<Entity>,
    mut commands: Commands,
    children: Query<&Children>,
    name: Query<&Name>,
) {
    for entity in children.iter_descendants(root) {
        let Ok(name) = name.get(entity) else {
            continue;
        };
        if name.as_str() == "DART_LAUNCH_DIRECTION" {
            commands.entity(entity).insert(DartLaunch);
            return;
        }
    }

    warn!("GROUND.glb is missing DART_LAUNCH_DIRECTION");
}

/// Turns a loaded robot world into a driveable vehicle: body, armor layers, chassis and gimbal.
pub fn setup_vehicle(
    In(root): In<Entity>,
    mut commands: Commands,
    query: HierarchyQuery,
    root_query: Query<(
        Entity,
        &Infantry,
        Option<&Controlled>,
        Option<&ActiveSlapper>,
    )>,
    sim_config: Res<SimulationConfig>,
) {
    let (root, infantry, is_local, is_active) = root_query
        .get(root)
        .expect("setup_vehicle called on an entity that is not an Infantry root");
    let team = infantry.team;
    let config = infantry.config;
    let is_local = is_local.is_some();
    let is_active = is_active.is_some();
    if is_local {
        query.children.iter_descendants(root).for_each(|e| {
            commands.entity(e).insert(Controlled);
        });
    } else {
        query.children.iter_descendants(root).for_each(|e| {
            commands.entity(e).insert(SlapperInfantry);
            if is_active {
                commands.entity(e).insert(ActiveSlapper);
            }
        });
    }
    let vehicle_body_collision_layers = GameLayer::vehicle_body_collision_layers(is_local);
    let vehicle_armor_collision_layers = GameLayer::vehicle_armor_collision_layers(is_local);

    commands.entity(root).insert((
        RigidBody::Dynamic,
        VehicleDynamic::new(
            sim_config.vehicle.max_speed,
            sim_config.vehicle.linear_acceleration,
            sim_config.vehicle.acceleration_exponent,
        ),
        Collider::compound(vec![(
            Vec3::new(0.0, -0.115649, 0.0),
            Quat::IDENTITY,
            Collider::cylinder(0.2593615, 0.231298),
        )]),
        CollisionMargin(0.005),
        vehicle_body_collision_layers,
        Mass(15.0),
        Restitution::new(0.01),
        AngularDamping(50.0),
    ));

    query.children.iter_descendants(root).for_each(|e| {
        commands.entity(e).insert(vehicle_armor_collision_layers);
    });

    let iter = query.of(root).any().exact("VEHICLE").flatten();
    let base = iter.clone().exact("BASE").one().unwrap();
    commands.entity(base).insert((
        InfantryChassis::default(),
        ScanArmor::new(team, config.armor),
    ));
    let gimbal = iter.exact("GIMBAL").one().unwrap();
    commands.entity(gimbal).insert(InfantryGimbal::default());
    // Slappers carry these markers too so the camera can follow them (see
    // `CameraFocus`); both robot models expose the nodes under `GIMBAL`.
    let gimbal_markers = query.of(gimbal).flatten();
    commands
        .entity(
            gimbal_markers
                .clone()
                .exact("SHOT_DIRECTION")
                .one()
                .unwrap(),
        )
        .insert(InfantryLaunchOffset);
    commands
        .entity(gimbal_markers.exact("CAM_DIRECTION").one().unwrap())
        .insert(InfantryViewOffset);
}

/// Queues Avian collider construction for the named nodes of a loaded world.
///
/// The map is passed in rather than parked on the entity as a component: it is an instruction for
/// one moment in the load, not state the world should keep.
pub fn setup_collision(
    In((root, map)): In<(Entity, PreciousCollision)>,
    mut commands: Commands,
    children: Query<&Children>,
    name: Query<&Name, With<Children>>,
) {
    for e in children.iter_descendants(root) {
        let Ok(name) = name.get(e) else {
            continue;
        };
        let Some((constructor, layer, visibility, rigid)) = map.get(name.as_str()) else {
            continue;
        };
        if let Some(rigid) = rigid {
            commands
                .entity(e)
                .insert((*rigid, constructor.clone(), *layer));
        } else {
            commands.entity(e).insert((constructor.clone(), *layer));
        }
        if visibility == &Visibility::Hidden {
            commands.entity(e).insert(*visibility);
        }
    }
}
