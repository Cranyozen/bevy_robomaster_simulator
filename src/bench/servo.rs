//! Bench-only velocity servo for scripted robots.
//!
//! Why this exists: [`crate::robomaster::vehicle::movement`] is a *direction*
//! actuator. `linear_accelerate` builds `dirc = (forward * stick.y + right *
//! stick.x).normalize_or_zero()` and scales it by the acceleration budget, so the
//! stick's **magnitude** never reaches the physics: a scenario that asks for
//! `v = 1` and one that asks for `v = 6` produce the same thrust, and the
//! evaluator's `speed_fraction` only survives as the sign. Measured on the real
//! vehicle (`/tmp/probe/freespace3`), a `line v=6` leg reached 1.2 m/s while a
//! `line v=1` leg on the same ground reached 1.7 m/s.
//!
//! A real RoboMaster chassis does not work that way: the operator sends a
//! velocity and the chassis closes its own loop. The benchmark therefore closes
//! the same loop explicitly — **only on the scripted path**, so the keyboard /
//! gamepad behaviour stays bit-identical:
//!
//! - bang-bang: thrust at full stick, along the velocity error
//!   `target_velocity - measured_velocity`;
//! - dead zone: the vehicle coasts only when it is *both* at the right speed
//!   (within the asymmetric band below) *and* not sliding sideways. The band is
//!   deliberately asymmetric — `+5 %` behind, `-15 %` ahead — because the
//!   friction always pulls the speed down: a symmetric band would make the
//!   vehicle sag to its lower edge and hold there (measured: 14 % low at
//!   `v = 4`), while a one-sided one rides within a few percent of the target;
//! - debounce: a thrust reversal inside `SERVO_MIN_FLIP_INTERVAL_S` coasts
//!   instead, which bounds the sign-flip rate the acceptance criterion watches;
//! - budget: targets above [`SERVO_BOOST_THRESHOLD_MPS`] ask for the `x2` boost
//!   budget, because the un-boosted friction limit is
//!   `6 * (1 - (v/6)^2.5) = mu * g` ⇒ **3.04 m/s**, with only 0.04 m/s² of net
//!   acceleration left at 3 m/s. Without it, `v = 3` could not be reached in 3 s
//!   and `v = 4` not at all.
//!
//! The servo rewrites only [`ControllerInput::movement`] and `::boost` of the
//! entry `scenario_control` wrote; the yaw command is untouched.

use avian3d::prelude::*;
use bevy::prelude::*;
use std::collections::HashMap;

use super::motion::MotionFrame;
use super::scenario::{Phase, Scenario};
use crate::components::{Infantry, RobotId};
use crate::config::SimulationConfig;
use crate::systems::{ControllerInput, RobotInputs};

/// How far **ahead** of the target the vehicle may run before the servo brakes,
/// as a fraction of the target speed.
pub const SERVO_AHEAD_BAND_FRAC: f32 = 0.15;
/// Lower bound on the ahead band, so a `v` of a few cm/s still brakes cleanly.
pub const SERVO_MIN_AHEAD_BAND_MPS: f32 = 0.05;
/// How far **behind** the target the vehicle may fall before the servo thrusts.
/// Smaller than the ahead band on purpose: friction pulls the speed down, so a
/// symmetric band lets the vehicle sag to its lower edge (see the module docs).
pub const SERVO_BEHIND_BAND_FRAC: f32 = 0.05;
pub const SERVO_MIN_BEHIND_BAND_MPS: f32 = 0.02;
/// Minimum interval between thrust sign reversals. Bounds the metric at
/// 2 flips/s, which is what the acceptance criterion asks for.
pub const SERVO_MIN_FLIP_INTERVAL_S: f32 = 0.5;
/// Target speeds above this use the boost budget (see the module docs).
pub const SERVO_BOOST_THRESHOLD_MPS: f32 = 2.0;
/// Below this the primitive asks for no translation at all (`static`, `spin`).
const SERVO_ZERO_EPSILON_MPS: f32 = 1e-3;

/// One servo step's verdict on the movement command.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoOutput {
    /// Body-frame stick, full magnitude when thrusting, zero when coasting.
    pub movement: Vec2,
    pub boost: bool,
}

/// Per-robot servo state: what the last thrust direction was and how often it
/// has flipped, for the `<= 2 flips/s` acceptance criterion.
#[derive(Clone, Debug, Default)]
pub struct RobotServo {
    last_dir: Vec3,
    last_flip_t: f32,
    /// Total thrust reversals accepted.
    pub flips: u32,
    recent_flips: Vec<f32>,
    /// Worst 1 s window seen so far.
    pub max_flips_per_second: u32,
    /// Whether the last step asked for thrust at all (for the flip metric:
    /// coasting is not a reversal).
    thrusting: bool,
}

impl RobotServo {
    /// One servo step for a robot whose ideal velocity is `target_velocity`
    /// (world space, from [`super::scenario::PlannedRobot::nominal_velocity`]) and
    /// whose measured velocity is `measured_velocity`.
    pub fn step(
        &mut self,
        now: f32,
        target_velocity: Vec3,
        measured_velocity: Vec3,
        frame: &MotionFrame,
    ) -> ServoOutput {
        let target = target_velocity.with_y(0.0);
        let speed = target.length();
        if speed <= SERVO_ZERO_EPSILON_MPS {
            // No translation requested; nothing to servo.
            self.thrusting = false;
            return ServoOutput {
                movement: Vec2::ZERO,
                boost: false,
            };
        }

        let error = target - measured_velocity.with_y(0.0);
        let ahead_band = (SERVO_AHEAD_BAND_FRAC * speed).max(SERVO_MIN_AHEAD_BAND_MPS);
        let behind_band = (SERVO_BEHIND_BAND_FRAC * speed).max(SERVO_MIN_BEHIND_BAND_MPS);
        // Speed error along the target direction, and whatever is left over
        // sideways (a `weave`'s lateral leg, or a target direction that just
        // rotated): both are worth thrusting for.
        let target_dir = target / speed;
        let along = error.dot(target_dir);
        let lateral = (error - target_dir * along).length();
        let wanted = if along > behind_band || along < -ahead_band || lateral > behind_band {
            error.normalize_or_zero()
        } else {
            Vec3::ZERO
        };

        // Debounce: a reversal inside the dwell window coasts rather than
        // pushing the other way, which is what keeps the flip rate bounded.
        let reversal = wanted != Vec3::ZERO
            && self.last_dir != Vec3::ZERO
            && self.last_dir.dot(wanted) <= 0.0;
        let direction = if wanted == Vec3::ZERO {
            Vec3::ZERO
        } else if reversal && now - self.last_flip_t < SERVO_MIN_FLIP_INTERVAL_S {
            Vec3::ZERO
        } else {
            if reversal {
                self.flips += 1;
                self.last_flip_t = now;
                self.recent_flips.push(now);
            }
            wanted
        };

        if direction != Vec3::ZERO {
            self.last_dir = direction;
        }
        self.thrusting = direction != Vec3::ZERO;
        self.recent_flips.retain(|flip| now - *flip <= 1.0);
        self.max_flips_per_second = self
            .max_flips_per_second
            .max(self.recent_flips.len() as u32);

        ServoOutput {
            movement: frame.input_for_world_direction(direction),
            boost: speed > SERVO_BOOST_THRESHOLD_MPS,
        }
    }

    /// Thrust reversals inside the last second, as of `now`.
    pub fn flips_last_second(&self, now: f32) -> u32 {
        self.recent_flips
            .iter()
            .filter(|flip| now - **flip <= 1.0)
            .count() as u32
    }

    pub fn is_thrusting(&self) -> bool {
        self.thrusting
    }
}

/// Every scripted robot's servo state, keyed by [`RobotId`]. A [`Resource`] so
/// the state survives frame by frame and can be inspected (log, tests).
#[derive(Resource, Default)]
pub struct SpeedServoStates {
    servos: HashMap<RobotId, RobotServo>,
}

impl SpeedServoStates {
    pub fn get(&self, id: RobotId) -> Option<&RobotServo> {
        self.servos.get(&id)
    }

    pub fn entry(&mut self, id: RobotId) -> &mut RobotServo {
        self.servos.entry(id).or_default()
    }

    pub fn len(&self) -> usize {
        self.servos.len()
    }

    pub fn is_empty(&self) -> bool {
        self.servos.is_empty()
    }
}

/// Close the speed loop for every scripted robot.
///
/// Runs between [`super::scenario::scenario_control`] (which writes the raw
/// primitive input) and [`crate::systems::robot_vehicle_controls`] (which turns
/// the stick into an impulse), so the servo sees the measured velocity of the
/// previous step and the vehicle sees the servoed stick in the same frame.
pub fn speed_servo_control(
    time_fixed: Res<Time<Fixed>>,
    config: Res<SimulationConfig>,
    mut states: ResMut<SpeedServoStates>,
    mut robot_inputs: ResMut<RobotInputs>,
    mut scenario: Option<ResMut<Scenario>>,
    robots: Query<(&RobotId, &GlobalTransform, &LinearVelocity), With<Infantry>>,
    mut last_log: Local<f64>,
) {
    if !config.bench.speed_servo {
        return;
    }
    let Some(scenario) = scenario.as_mut() else {
        return;
    };
    if scenario.phase != Phase::Running {
        return;
    }

    let measured: Vec<(RobotId, Vec3, MotionFrame)> = robots
        .iter()
        .map(|(id, transform, velocity)| {
            (
                *id,
                velocity.0,
                MotionFrame::from_transform(transform),
            )
        })
        .collect();

    let now = time_fixed.elapsed_secs();
    let t_scenario = scenario.t_scenario;
    // One log line per second per robot: the acceptance criterion watches the
    // thrust sign-flip rate, and the frozen GT messages have no field for it.
    let fixed_now = time_fixed.elapsed_secs_f64();
    let logging = fixed_now - *last_log >= 1.0;
    if logging {
        *last_log = fixed_now;
    }
    for robot in scenario.robots.iter_mut() {
        // A pending spawn has no body yet, and a robot the scenario does not
        // drive has no entry to rewrite.
        if robot.spawn.is_some() && !robot.installed {
            continue;
        }
        let Some(input) = robot_inputs.get(robot.id) else {
            continue;
        };
        let Some((_, velocity, frame)) = measured
            .iter()
            .find(|(id, _, _)| *id == robot.id)
        else {
            continue;
        };
        let target = robot.nominal_velocity(t_scenario);
        let servo = states.entry(robot.id);
        let output = servo.step(now, target, *velocity, frame);
        if logging {
            info!(
                "[bench] servo robot {}: target {:.2} m/s measured {:.2} m/s \
                 thrusting={} flips={} max_flips_per_second={}",
                robot.id.0,
                target.length(),
                velocity.length(),
                servo.is_thrusting(),
                servo.flips,
                servo.max_flips_per_second,
            );
        }
        robot_inputs.set(
            robot.id,
            ControllerInput {
                movement: output.movement,
                boost: output.boost,
                ..input
            },
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shipped vehicle budget (`config.toml` `[vehicle]`) plus the ground
    /// friction the scene gives the arena.
    const ACCEL: f32 = 6.0;
    const MAX_SPEED: f32 = 6.0;
    const EXPONENT: f32 = 2.5;
    const FRICTION: f32 = 0.5 * 9.81;

    /// Net acceleration of the real vehicle model while thrusting along the
    /// direction of travel: `a = accel * (1 - (v/max)^n) - mu * g`.
    fn plant_acceleration(speed: f32, boost: f32) -> f32 {
        let max_speed = MAX_SPEED * boost;
        ACCEL * boost * (1.0 - (speed / max_speed).powf(EXPONENT)) - FRICTION
    }

    struct Run {
        final_speed: f32,
        max_flips_per_second: u32,
        time_to_band: Option<f32>,
    }

    /// Drive the servo against the analytic plant, exactly as the vehicle does:
    /// full stick in the error direction, no thrust while coasting.
    fn simulate(target: f32, seconds: f32) -> Run {
        let dt = 1.0 / 64.0;
        let frame = MotionFrame {
            forward: Vec3::NEG_Z,
            right: Vec3::X,
        };
        let mut servo = RobotServo::default();
        let mut vel = 0.0f32;
        let mut time = 0.0f32;
        let mut time_to_band = None;
        while time < seconds {
            time += dt;
            // A `line` at signed speed `target` along +X in this frame.
            let target_velocity = Vec3::X * target;
            let out = servo.step(time, target_velocity, Vec3::X * vel, &frame);
            // The stick is normalised by the vehicle layer: only its direction
            // and sign matter.
            let thrusting = out.movement.length() > 0.0;
            let along = out.movement.x.signum();
            let boost = if out.boost { 2.0 } else { 1.0 };
            let accel = if thrusting {
                along * plant_acceleration(vel, boost)
            } else {
                -FRICTION.signum() * vel.signum()
            };
            vel += accel * dt;
            if vel.abs() > MAX_SPEED * boost {
                vel = MAX_SPEED * boost * vel.signum();
            }
            if vel.abs() < 1e-3 {
                vel = 0.0;
            }
            if time_to_band.is_none() && (vel - target).abs() <= 0.25 * target.abs() {
                time_to_band = Some(time);
            }
        }
        Run {
            final_speed: vel,
            max_flips_per_second: servo.max_flips_per_second,
            time_to_band,
        }
    }

    #[test]
    fn servo_reaches_every_p0_target_speed_within_the_tolerance() {
        // The acceptance criterion: within 3 s of the command, |v| is within
        // 25 % of v, for v in {1, 2, 3, 4}.
        for target in [1.0f32, 2.0, 3.0, 4.0] {
            let run = simulate(target, 3.0);
            let error = (run.final_speed - target).abs() / target;
            assert!(
                error <= 0.25,
                "v={target}: reached {} after 3 s ({:.1}% off)",
                run.final_speed,
                error * 100.0
            );
            assert!(
                run.time_to_band.map(|t| t <= 3.0).unwrap_or(false),
                "v={target}: never entered the ±25 % band"
            );
        }
    }

    #[test]
    fn servo_keeps_the_thrust_sign_flip_rate_at_or_below_two_per_second() {
        for target in [1.0f32, 2.0, 3.0, 4.0] {
            let run = simulate(target, 6.0);
            assert!(
                run.max_flips_per_second <= 2,
                "v={target}: {} flips in one second",
                run.max_flips_per_second
            );
        }
    }

    #[test]
    fn servo_reaches_high_targets_only_because_it_asks_for_boost() {
        // Without the boost request the friction limit is 3.04 m/s and a 4 m/s
        // command can never be met — the reason `SERVO_BOOST_THRESHOLD_MPS`
        // exists.
        let ceiling = {
            let mut lo = 0.0f32;
            let mut hi = MAX_SPEED;
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if plant_acceleration(mid, 1.0) > 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            0.5 * (lo + hi)
        };
        assert!(
            (ceiling - 3.04).abs() < 0.05,
            "un-boosted friction ceiling drifted to {ceiling}"
        );

        let frame = MotionFrame {
            forward: Vec3::NEG_Z,
            right: Vec3::X,
        };
        let mut servo = RobotServo::default();
        assert!(!servo.step(0.01, Vec3::X * 1.5, Vec3::ZERO, &frame).boost);
        assert!(servo.step(0.02, Vec3::X * 2.5, Vec3::ZERO, &frame).boost);
    }

    #[test]
    fn servo_coasts_inside_the_dead_zone_and_brakes_when_ahead() {
        let frame = MotionFrame {
            forward: Vec3::NEG_Z,
            right: Vec3::X,
        };
        let mut servo = RobotServo::default();
        // At the mark: coast.
        let out = servo.step(0.01, Vec3::X * 2.0, Vec3::X * 2.0, &frame);
        assert_eq!(out.movement, Vec2::ZERO);
        // Well behind: full stick towards +X (`right` is `Vec3::X`).
        let out = servo.step(0.02, Vec3::X * 2.0, Vec3::ZERO, &frame);
        assert!(out.movement.x > 0.9 && out.movement.y.abs() < 1e-6);
        // Well ahead: full stick the other way.
        let out = servo.step(3.00, Vec3::X * 2.0, Vec3::X * 4.0, &frame);
        assert!(out.movement.x < -0.9);
    }

    #[test]
    fn servo_ignores_primitives_that_do_not_translate() {
        let frame = MotionFrame {
            forward: Vec3::NEG_Z,
            right: Vec3::X,
        };
        let mut servo = RobotServo::default();
        let moving = servo.step(0.01, Vec3::X * 2.0, Vec3::ZERO, &frame);
        assert_ne!(moving.movement, Vec2::ZERO);
        // `static` / `spin`: no target velocity, no thrust, and the coast must
        // not be counted as a reversal.
        let out = servo.step(0.02, Vec3::ZERO, Vec3::ZERO, &frame);
        assert_eq!(out.movement, Vec2::ZERO);
        assert_eq!(servo.flips, 0);
        assert_eq!(servo.max_flips_per_second, 0);
    }

    #[test]
    fn servo_debounce_bounds_rapid_reversals() {
        let frame = MotionFrame {
            forward: Vec3::NEG_Z,
            right: Vec3::X,
        };
        let mut servo = RobotServo::default();
        let mut now = 0.0f32;
        // Alternate "far ahead" (brake) and "far behind" (push) at 100 Hz: the
        // plant is adversarial, the debounce still has to bound the rate.
        for _ in 0..600 {
            now += 0.01;
            let measured = if servo.is_thrusting() {
                Vec3::X * 5.0
            } else {
                Vec3::ZERO
            };
            servo.step(now, Vec3::X * 2.0, measured, &frame);
        }
        assert!(
            servo.max_flips_per_second <= 2,
            "debounce let {} flips through in one second",
            servo.max_flips_per_second
        );
        assert!(servo.flips >= 1, "the adversarial plant never flipped at all");
    }
}
