//! P1-B runtime scenario services: `/sim/scenario/{load,start,reset}`.
//!
//! # Why services, not topics
//!
//! `load`/`start`/`reset` need an answer ("did it work, and what is running
//! now?"), and a topic would turn every failure into a silent timeout.
//!
//! # Threading
//!
//! The r2r callbacks run on the ROS executor thread; they only **enqueue** a
//! request. The state change happens in [`poll_scenario_services`] on the main
//! schedule, so the physics world is never touched from another thread and a
//! slow load cannot stall driving. The services answer within a frame (well under
//! the ~1 s the runner allows) and never wait for a spawn: spawn confirmation
//! travels through `/sim/scenario/status`.
//!
//! # Failure policy
//!
//! A `load` that cannot be parsed leaves the running scenario *completely*
//! untouched and answers `success=false` with the parser's own error text. A
//! half-applied load is the one state that cannot be recovered from, and a
//! "failed" load that quietly kept the old scenario running would make a batch
//! read the previous scenario's data.
//!
//! # Reset list
//!
//! A runtime `reset`/`start` undoes every piece of state a previous run could
//! leak into the next one:
//!
//! * robot poses, velocities and chassis heading, plus scripted-driver/`installed`
//!   state — in [`crate::bench::scenario::scenario_control`], which owns those
//!   queries;
//! * live projectiles (despawned);
//! * shot bookkeeping: [`ResolvedShots`], [`ArmorContactSet`], [`ShotSequence`];
//! * the fire path: `PendingFire`, `ProjectileCooldown` and the 10 Hz
//!   `FireRateLimiter`;
//! * scenario bookkeeping: `t_scenario`, `phase`, `max_track_deviation_m`,
//!   `max_along_track_m`,
//!   per-robot deviation counters and `spawn_error`;
//! * task-67 occluder boxes (`ScenarioOccluder`), which are then respawned for
//!   whichever scenario is current — so a `load` of scenario B never runs with
//!   scenario A's occluders still in the world.
//!
//! There is no RNG state to reset: the scenario `seed` is parsed, recorded and
//! published, but P0/P1 primitives are deterministic and never draw from it
//! (verified by grep before this module was written).

use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use bevy::prelude::*;
use bevy::tasks::futures_lite;
use bevy::tasks::futures_lite::future::block_on;
use r2r::rm_interfaces::srv::{SimScenarioLoad, SimScenarioReset, SimScenarioStart};
use r2r::QosProfile;

use crate::bench::events::{ArmorContactSet, ResolvedShots, ShotSequence};
use crate::bench::scenario::{self, Phase, Scenario, ScenarioOccluder};
use crate::components::ProjectileCooldown;
use crate::config::SimulationConfig;
use crate::robomaster::prelude::Projectile;
use crate::ros2::plugin::{FireRateLimiter, PendingFire, FIRE_RATE_HZ};
use crate::ros2::prelude::AverageRateLimiter;

pub const LOAD_SERVICE: &str = "/sim/scenario/load";
pub const START_SERVICE: &str = "/sim/scenario/start";
pub const RESET_SERVICE: &str = "/sim/scenario/reset";

/// One queued service call. Holds the reply handle so the answer is sent from the
/// main schedule, after the state change actually happened.
pub enum ScenarioRequest {
    Load {
        scenario_path: String,
        seed: i64,
        reply: r2r::ServiceRequest<SimScenarioLoad::Service>,
    },
    Start {
        reply: r2r::ServiceRequest<SimScenarioStart::Service>,
    },
    Reset {
        reply: r2r::ServiceRequest<SimScenarioReset::Service>,
    },
}

/// Requests waiting for the main schedule. The ROS thread only ever appends.
#[derive(Resource, Clone, Default)]
pub struct ScenarioServiceQueue(Arc<Mutex<Vec<ScenarioRequest>>>);

impl ScenarioServiceQueue {
    fn push(&self, request: ScenarioRequest) {
        match self.0.lock() {
            Ok(mut queue) => queue.push(request),
            Err(_) => error!("[bench] scenario service queue is poisoned; dropping request"),
        }
    }

    fn drain(&self) -> Vec<ScenarioRequest> {
        self.0
            .lock()
            .map(|mut queue| std::mem::take(&mut *queue))
            .unwrap_or_default()
    }
}

/// Creates the three services and one listener thread each.
///
/// The threads exit when the node's stop signal flips, exactly like the topic
/// subscribers in [`crate::ros2::topic`].
pub fn register_services(signal: Arc<AtomicBool>, app: &mut App, node: &mut r2r::Node) {
    let queue = ScenarioServiceQueue::default();
    app.insert_resource(queue.clone());

    match node.create_service::<SimScenarioLoad::Service>(LOAD_SERVICE, QosProfile::default()) {
        Ok(mut stream) => {
            let queue = queue.clone();
            let signal = signal.clone();
            std::thread::spawn(move || {
                while !signal.load(Ordering::Acquire) {
                    let Some(request) = block_on(futures_lite::StreamExt::next(&mut stream)) else {
                        break;
                    };
                    queue.push(ScenarioRequest::Load {
                        scenario_path: request.message.scenario_path.clone(),
                        seed: request.message.seed,
                        reply: request,
                    });
                }
            });
        }
        Err(error) => error!("[bench] cannot create {LOAD_SERVICE}: {error}"),
    }

    match node.create_service::<SimScenarioStart::Service>(START_SERVICE, QosProfile::default()) {
        Ok(mut stream) => {
            let queue = queue.clone();
            let signal = signal.clone();
            std::thread::spawn(move || {
                while !signal.load(Ordering::Acquire) {
                    let Some(request) = block_on(futures_lite::StreamExt::next(&mut stream)) else {
                        break;
                    };
                    queue.push(ScenarioRequest::Start { reply: request });
                }
            });
        }
        Err(error) => error!("[bench] cannot create {START_SERVICE}: {error}"),
    }

    match node.create_service::<SimScenarioReset::Service>(RESET_SERVICE, QosProfile::default()) {
        Ok(mut stream) => {
            let queue = queue.clone();
            let signal = signal.clone();
            std::thread::spawn(move || {
                while !signal.load(Ordering::Acquire) {
                    let Some(request) = block_on(futures_lite::StreamExt::next(&mut stream)) else {
                        break;
                    };
                    queue.push(ScenarioRequest::Reset { reply: request });
                }
            });
        }
        Err(error) => error!("[bench] cannot create {RESET_SERVICE}: {error}"),
    }

    info!("[bench] scenario services up: {LOAD_SERVICE}, {START_SERVICE}, {RESET_SERVICE}");
}

/// Applies queued service calls, ordered before `scenario_control`.
#[allow(clippy::too_many_arguments)]
pub fn poll_scenario_services(
    mut commands: Commands,
    services: Res<ScenarioServiceQueue>,
    config: Res<SimulationConfig>,
    scenario: Option<ResMut<Scenario>>,
    mut resolved: ResMut<ResolvedShots>,
    mut contacts: ResMut<ArmorContactSet>,
    shots: Option<Res<ShotSequence>>,
    mut pending_fire: ResMut<PendingFire>,
    mut cooldown: ResMut<ProjectileCooldown>,
    mut limiter: ResMut<FireRateLimiter>,
    projectiles: Query<Entity, With<Projectile>>,
    occluders: Query<Entity, With<ScenarioOccluder>>,
) {
    let requests = services.drain();
    if requests.is_empty() {
        return;
    }
    let mut scenario = scenario;

    for request in requests {
        match request {
            ScenarioRequest::Load {
                scenario_path,
                seed,
                reply,
            } => {
                let limits = scenario::motion_limits(&config);
                match Scenario::load_for_runtime(Path::new(&scenario_path), seed, &limits) {
                    Ok(loaded) => {
                        reset_run_state(
                            &mut commands,
                            &projectiles,
                            &occluders,
                            &mut resolved,
                            &mut contacts,
                            shots.as_deref(),
                            &mut pending_fire,
                            &mut cooldown,
                            &mut limiter,
                        );
                        let response = SimScenarioLoad::Response {
                            success: true,
                            message: String::new(),
                            scenario_id: loaded.id.clone(),
                            seed: loaded.seed as i64,
                            // A freshly loaded scenario is Idle until an explicit
                            // `start`; report exactly that.
                            phase: loaded.phase.wire(),
                            // The value of the scenario this reply refers to: for a
                            // fresh load that is the newly loaded one, i.e. `false`
                            // while its spawns are still pending (trivially `true`
                            // when it asks for none). The settled answer for the
                            // run comes from `/sim/scenario/status` after `start`.
                            spawn_applied: loaded.spawn_applied(),
                        };
                        info!(
                            "[bench] service load: scenario '{}' (seed {}) accepted, phase={}",
                            loaded.id, loaded.seed, response.phase
                        );
                        commands.insert_resource(loaded);
                        respond(reply, response);
                    }
                    Err(error) => {
                        // Leave the running scenario alone — but say so loudly,
                        // because a batch that reads the old scenario's data while
                        // believing the new one loaded is the worst outcome.
                        let (scenario_id, current_seed, phase, spawn_applied) =
                            snapshot(scenario.as_deref());
                        error!("[bench] service load rejected: {error}; keeping '{scenario_id}'");
                        respond(
                            reply,
                            SimScenarioLoad::Response {
                                success: false,
                                message: error,
                                scenario_id,
                                seed: current_seed,
                                phase,
                                spawn_applied,
                            },
                        );
                    }
                }
            }
            ScenarioRequest::Start { reply } => {
                // `seed` is not part of the Start response; `phase` is recomputed
                // below from what the start actually does.
                let (scenario_id, _seed, _phase, spawn_applied) = snapshot(scenario.as_deref());
                let has_scenario = scenario.is_some();
                let (success, message, phase) = match scenario.as_mut() {
                    Some(scenario) => {
                        // Restart: full physical reset, then Running at t = 0. The
                        // run-state reset below is the other half of "start = a
                        // clean run": without it, shots and rate limiters from the
                        // previous run would leak into this one.
                        scenario.pending = scenario::ScenarioCommand::Restart;
                        info!("[bench] service start: scenario '{}' restarting", scenario.id);
                        (true, String::new(), Phase::Running.wire())
                    }
                    None => (
                        false,
                        "no scenario loaded; call /sim/scenario/load first".to_string(),
                        Phase::Idle.wire(),
                    ),
                };
                if has_scenario {
                    reset_run_state(
                        &mut commands,
                        &projectiles,
                        &occluders,
                        &mut resolved,
                        &mut contacts,
                        shots.as_deref(),
                        &mut pending_fire,
                        &mut cooldown,
                        &mut limiter,
                    );
                }
                respond(
                    reply,
                    SimScenarioStart::Response {
                        success,
                        message,
                        scenario_id,
                        phase,
                        spawn_applied,
                    },
                );
            }
            ScenarioRequest::Reset { reply } => {
                // Idempotent: with no scenario running this still succeeds.
                let (scenario_id, phase, spawn_applied) = match scenario.as_mut() {
                    Some(scenario) => {
                        scenario.pending = scenario::ScenarioCommand::Reset;
                        info!("[bench] service reset: scenario '{}'", scenario.id);
                        (
                            scenario.id.clone(),
                            Phase::Idle.wire(),
                            // Current value at reply time; the physical reset lands
                            // later this frame, so poll status for the settled one.
                            scenario.spawn_applied(),
                        )
                    }
                    None => (String::new(), Phase::Idle.wire(), false),
                };
                reset_run_state(
                    &mut commands,
                    &projectiles,
                    &occluders,
                    &mut resolved,
                    &mut contacts,
                    shots.as_deref(),
                    &mut pending_fire,
                    &mut cooldown,
                    &mut limiter,
                );
                respond(
                    reply,
                    SimScenarioReset::Response {
                        success: true,
                        message: String::new(),
                        scenario_id,
                        phase,
                        spawn_applied,
                    },
                );
            }
        }
    }
}

fn respond<T>(reply: r2r::ServiceRequest<T>, response: T::Response)
where
    T: r2r::WrappedServiceTypeSupport + 'static,
{
    if let Err(error) = reply.respond(response) {
        error!("[bench] cannot send scenario service response: {error}");
    }
}

/// Current scenario identity, for answers that must describe what is *actually*
/// running (a rejected load, a start without a scenario, ...).
fn snapshot(scenario: Option<&Scenario>) -> (String, i64, u8, bool) {
    match scenario {
        Some(scenario) => (
            scenario.id.clone(),
            scenario.seed as i64,
            scenario.phase.wire(),
            scenario.spawn_applied(),
        ),
        None => (String::new(), 0, Phase::Idle.wire(), false),
    }
}

/// Despawns live projectiles and clears every per-run counter that is not part of
/// the scenario itself. See the module docs for the full reset list.
///
/// The occluder boxes are part of the scenario's *own* state, so they are
/// despawned here and respawned by
/// [`scenario::install_scenario_occluders`] for whichever scenario is current
/// after the command — that is what makes a `load` of scenario B free of
/// scenario A's boxes (task-67).
fn reset_run_state(
    commands: &mut Commands,
    projectiles: &Query<Entity, With<Projectile>>,
    occluders: &Query<Entity, With<ScenarioOccluder>>,
    resolved: &mut ResolvedShots,
    contacts: &mut ArmorContactSet,
    shots: Option<&ShotSequence>,
    pending_fire: &mut PendingFire,
    cooldown: &mut ProjectileCooldown,
    limiter: &mut FireRateLimiter,
) {
    for entity in projectiles.iter() {
        commands.entity(entity).despawn();
    }
    for entity in occluders.iter() {
        commands.entity(entity).despawn();
    }
    resolved.clear();
    contacts.clear();
    if let Some(shots) = shots {
        shots.reset();
    }
    pending_fire.reset();
    cooldown.0.reset();
    **limiter = AverageRateLimiter::from_hz(FIRE_RATE_HZ);
}
