//! Scenario motion primitives → the inputs the existing vehicle systems already
//! understand.
//!
//! The point of this module is that a scripted robot is **not** special-cased in
//! the physics. `evaluate` answers the only question the vehicle layer asks —
//! "what is the stick input this instant" — as a `(movement, chassis_yaw, boost)`
//! triple, and [`crate::systems::robot_vehicle_controls`] then feeds it through
//! exactly the same [`VehicleDynamic::linear`] and `update_chassis_rotation` the
//! keyboard uses. Nothing here writes a `Transform`: if a primitive asks for
//! 8 m/s or 20 rad/s, the physical limits clamp it and the mismatch shows up in
//! the reported trajectory deviation instead of being silently believed.
//!
//! Frames: the scenario file is authored in the **map (ROS) frame** — the one
//! `/tf`, `/sim/gt/*` and the auto-aim stack use. `dir_deg` is a ROS heading
//! about +Z, and because the simulator's `/tf` alignment maps Bevy `Y` to ROS
//! `Z`, a ROS heading of `θ` is the Bevy yaw `θ` as well. The optional `spawn`
//! position does need the full remap; see [`crate::bench::scenario`].
//!
//! [`VehicleDynamic::linear`]: crate::robomaster::vehicle::movement::VehicleDynamic::linear

use bevy::prelude::*;

/// What one primitive asks the vehicle to do at one instant.
///
/// `movement` is expressed in the frame the caller will pass to
/// `VehicleDynamic::linear` (see [`MotionFrame`]), so it is directly usable and
/// stays inside `[-1, 1]` per axis: it is a stick deflection, never a velocity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionCommand {
    pub movement: Vec2,
    pub chassis_yaw: f32,
    /// Multiplier on the vehicle's speed/acceleration budget: `1.0` is normal,
    /// `2.0` is the keyboard's Shift boost. No P0 primitive boosts, but the field
    /// exists so the evaluator's output shape stays the one P1 primitives (and
    /// the interactive `/sim/robot_cmd` path) will need.
    pub boost: f32,
}

impl Default for MotionCommand {
    fn default() -> Self {
        Self {
            movement: Vec2::ZERO,
            chassis_yaw: 0.0,
            boost: 1.0,
        }
    }
}

/// The physical limits a primitive must respect, mirrored from
/// `[vehicle]` in `config.toml`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionLimits {
    pub max_speed: f32,
    pub rotation_speed: f32,
}

impl Default for MotionLimits {
    fn default() -> Self {
        Self {
            max_speed: 6.0,
            rotation_speed: 7.42,
        }
    }
}

/// The basis `movement` is expressed in: the movement frame's forward/right,
/// flattened onto the ground plane.
#[derive(Clone, Copy, Debug)]
pub struct MotionFrame {
    pub forward: Vec3,
    pub right: Vec3,
}

impl MotionFrame {
    pub fn from_transform(transform: &GlobalTransform) -> Self {
        Self {
            forward: transform.forward().with_y(0.0).normalize_or_zero(),
            right: transform.right().with_y(0.0).normalize_or_zero(),
        }
    }

    /// Stick input that makes `direction` (a world-space vector) come out of
    /// `VehicleDynamic::linear` as a world translation along that direction.
    ///
    /// `linear` rebuilds the direction as `forward * input.y + right * input.x`
    /// and normalises it, so projecting onto an orthonormal basis is exact.
    pub fn input_for_world_direction(&self, direction: Vec3) -> Vec2 {
        let direction = direction.with_y(0.0).normalize_or_zero();
        Vec2::new(direction.dot(self.right), direction.dot(self.forward))
    }

    /// World-space direction of a scenario heading, in radians (ROS +Z heading,
    /// which is numerically the Bevy `Y` yaw the `/tf` alignment produces).
    pub fn world_direction(heading: f32) -> Vec3 {
        let (sin, cos) = heading.sin_cos();
        // Bevy (x, y, z) = (-sin θ, ·, -cos θ) reproduces ROS (cos θ, sin θ, 0)
        // under the `/tf` remap in `ros2::prelude::transform`.
        Vec3::new(-sin, 0.0, -cos)
    }

    /// Right-hand side of a world direction, on the ground plane. Positive
    /// `weave` amplitude offsets to this side.
    pub fn right_of(direction: Vec3) -> Vec3 {
        direction.cross(Vec3::Y).normalize_or_zero()
    }
}

/// Effective displacement per second the primitive asks for, before limits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Primitive {
    /// Parked.
    Static,
    /// Straight line at `v` along the scenario heading `dir_deg`.
    Line { v: f32, dir_deg: f32 },
    /// Mecanum sidestep at `v` along `dir_deg`, body heading unchanged.
    Strafe { v: f32, dir_deg: f32 },
    /// Near-perpendicular crossing of the line of sight: the same slide as
    /// [`Primitive::Strafe`] (mecanum, body heading unchanged, so the armour
    /// ring does *not* swing with the travel direction and the crossing factor
    /// survives), but the motion can stop at the crossing point.
    ///
    /// `pass_through = true` slides for the whole leg; `false` coasts from the
    /// leg's midpoint on, which is where the scenario author should place the
    /// line-of-sight crossing so the target does not run away from the sensor
    /// after passing it. `duration_s` is filled from the segment window
    /// (`t1 - t0`) at parse time — it is not a scene-file key.
    CrossLos {
        v: f32,
        dir_deg: f32,
        pass_through: bool,
        duration_s: f32,
    },
    /// In-place spin at `omega` rad/s (armour phase switching).
    Spin { omega: f32 },
    /// P2 frequency-response axis: the chassis yaw **oscillates** as
    /// `theta(t) = yaw_amp_deg * sin(phi(t))`, so the angular rate is
    /// `omega(t) = yaw_amp_deg * 2*pi*f(t) * cos(phi(t))` with the frequency swept
    /// linearly from `freq_start_hz` to `freq_end_hz` over the leg
    /// (`duration_s`, filled from the segment window at parse time).
    ///
    /// This is the *hull-rotating* counterpart of [`Primitive::Sine`]: it turns
    /// the body on purpose, which is the capability under test (the "robot
    /// rotates its chassis" motion), while `sine`/`cross_los` hold the heading.
    ///
    /// The chassis yaw is kinematic in this simulator
    /// (`update_chassis_rotation` integrates the rate with no torque or friction
    /// limit), so unlike a lateral `sine` there is **no lateral-acceleration
    /// ceiling** here: the sweep reaches frequencies a lateral position sine
    /// cannot. The only budget is the chassis rotation speed, which caps the
    /// rate peak and therefore the amplitude (see [`Primitive::clamped`]).
    ///
    /// Closed form for `plan.py::travel_trace` (there is **no** linear
    /// translation, exactly like [`Primitive::Spin`]):
    /// `f(t) = f0 + (f1 - f0)*t/T`, `phi(t) = 2*pi*(f0*t + (f1-f0)*t^2/(2T))`,
    /// `theta(t) = A*sin(phi(t))`, `omega(t) = A*2*pi*f(t)*cos(phi(t))`,
    /// peak rate `A*2*pi*f_max`, peak angular acceleration `~ A*(2*pi*f_max)^2`.
    ///
    /// A *constant* frequency covers the "sinusoidal angular rate" spelling too:
    /// asking for `omega(t) = omega_max*cos(2*pi*f*t)` is exactly
    /// `A = omega_max/(2*pi*f)` here, so one mode expresses both.
    SpinSweep {
        yaw_amp_deg: f32,
        freq_start_hz: f32,
        freq_end_hz: f32,
        duration_s: f32,
    },
    /// `v` along `dir_deg` plus a sinusoidal lateral velocity of
    /// `amp_m * 2π * freq_hz`, i.e. a real serpentine rather than a wiggle of
    /// the stick.
    Weave {
        v: f32,
        dir_deg: f32,
        freq_hz: f32,
        amp_m: f32,
    },
    /// P2 frequency-response axis: a sinusoidal lateral sweep of `amp_m` on top
    /// of a constant `v` along `dir_deg`, with the frequency **swept linearly**
    /// from `freq_start_hz` to `freq_end_hz` over the leg (`duration_s`).
    ///
    /// This is [`Primitive::Weave`] with a chirp, and with the body heading
    /// **held** (like [`Primitive::Strafe`]): the armour ring keeps pointing the
    /// way the scenario set it while the line-of-sight rate oscillates at a
    /// known, increasing frequency, which is what the chirp axis is for.
    ///
    /// The ideal path is closed-form, so `plan.py::travel_trace` can predict it:
    /// `along(t) = v·t`, `φ(t) = 2π(f₀·t + (f₁−f₀)·t²/(2T))`,
    /// `lateral(t) = amp_m·sin(φ(t))`. `freq_start_hz == freq_end_hz` degenerates
    /// to a pure sine. `duration_s` is filled from the segment window at parse
    /// time — it is not a scene-file key.
    Sine {
        v: f32,
        dir_deg: f32,
        amp_m: f32,
        freq_start_hz: f32,
        freq_end_hz: f32,
        duration_s: f32,
    },
}

impl Primitive {
    /// The four-character kind used in the scene file, for diagnostics.
    pub const fn kind(&self) -> &'static str {
        match self {
            Self::Static => "static",
            Self::Line { .. } => "line",
            Self::Strafe { .. } => "strafe",
            Self::CrossLos { .. } => "cross_los",
            Self::Spin { .. } => "spin",
            // The sweep is authored as `type = "spin"` plus `yaw_amp_deg` and a
            // frequency pair, so it reports the same scene-file kind.
            Self::SpinSweep { .. } => "spin",
            Self::Weave { .. } => "weave",
            Self::Sine { .. } => "sine",
        }
    }

    /// Clamp every parameter into what the vehicle can actually do, reporting
    /// whether anything had to move. Nothing is rejected here: an
    /// out-of-range parameter is a *scenario authoring* mistake that must stay
    /// visible as a trajectory deviation, not a crash.
    pub fn clamped(self, limits: &MotionLimits) -> (Self, bool) {
        let max_speed = limits.max_speed.max(0.0);
        let max_omega = limits.rotation_speed.abs().max(0.0);
        let clamp = |v: f32| v.clamp(-max_speed, max_speed);
        let clamped = |original: f32, new: f32| (original - new).abs() > 1e-6;

        let (primitive, did_clamp) = match self {
            Self::Static => (Self::Static, false),
            Self::Line { v, dir_deg } => {
                let v = clamp(v);
                (Self::Line { v, dir_deg }, clamped(self.v().unwrap_or(0.0), v))
            }
            Self::Strafe { v, dir_deg } => {
                let v = clamp(v);
                (
                    Self::Strafe { v, dir_deg },
                    clamped(self.v().unwrap_or(0.0), v),
                )
            }
            Self::Spin { omega } => {
                let omega = omega.clamp(-max_omega, max_omega);
                (
                    Self::Spin { omega },
                    clamped(self.omega().unwrap_or(0.0), omega),
                )
            }
            Self::SpinSweep {
                yaw_amp_deg,
                freq_start_hz,
                freq_end_hz,
                duration_s,
            } => {
                let start = freq_start_hz.clamp(0.0, MAX_SWEEP_FREQ_HZ);
                let end = freq_end_hz.clamp(0.0, MAX_SWEEP_FREQ_HZ);
                // The rate peak is `A * 2*pi*f_max`: keep it inside the chassis                // rotation budget by shrinking the amplitude, which is the rule
                // the bench asked for (clamp `A`, report `did_clamp`).
                let f_max = start.max(end);
                let rate_cap_deg = if f_max > 1e-6 {
                    max_omega.to_degrees() / (std::f32::consts::TAU * f_max)
                } else {
                    MAX_SWEEP_YAW_AMP_DEG
                };
                let amp = yaw_amp_deg.clamp(0.0, rate_cap_deg.min(MAX_SWEEP_YAW_AMP_DEG));
                (
                    Self::SpinSweep {
                        yaw_amp_deg: amp,
                        freq_start_hz: start,
                        freq_end_hz: end,
                        duration_s,
                    },
                    clamped(yaw_amp_deg, amp)
                        || clamped(freq_start_hz, start)
                        || clamped(freq_end_hz, end),
                )
            }
            Self::CrossLos {
                v,
                dir_deg,
                pass_through,
                duration_s,
            } => {
                let v = clamp(v);
                (
                    Self::CrossLos {
                        v,
                        dir_deg,
                        pass_through,
                        duration_s,
                    },
                    clamped(self.v().unwrap_or(0.0), v),
                )
            }
            Self::Weave {
                v,
                dir_deg,
                freq_hz,
                amp_m,
            } => {
                let v = clamp(v);
                // The lateral *velocity* is what has to fit, and `evaluate`
                // enforces that by clamping the resulting vector; these bounds
                // only stop absurd (and physically meaningless) authoring.
                let freq_hz = freq_hz.clamp(0.0, 10.0);
                let amp_m = amp_m.clamp(0.0, 20.0);
                (
                    Self::Weave {
                        v,
                        dir_deg,
                        freq_hz,
                        amp_m,
                    },
                    clamped(self.v().unwrap_or(0.0), v)
                        || clamped(self.freq_hz().unwrap_or(0.0), freq_hz)
                        || clamped(self.amp_m().unwrap_or(0.0), amp_m),
                )
            }
            Self::Sine {
                v,
                dir_deg,
                amp_m,
                freq_start_hz,
                freq_end_hz,
                duration_s,
            } => {
                let v = clamp(v);
                // Same bounds as `Weave` (they only stop absurd authoring; the
                // reachable part is enforced by the speed clamp in `evaluate`).
                let start = freq_start_hz.clamp(0.0, 10.0);
                let end = freq_end_hz.clamp(0.0, 10.0);
                let amp = amp_m.clamp(0.0, 20.0);
                (
                    Self::Sine {
                        v,
                        dir_deg,
                        amp_m: amp,
                        freq_start_hz: start,
                        freq_end_hz: end,
                        duration_s,
                    },
                    clamped(self.v().unwrap_or(0.0), v)
                        || clamped(amp_m, amp)
                        || clamped(freq_start_hz, start)
                        || clamped(freq_end_hz, end),
                )
            }
        };
        (primitive, did_clamp)
    }

    pub const fn v(&self) -> Option<f32> {
        match *self {
            Self::Line { v, .. }
            | Self::Strafe { v, .. }
            | Self::CrossLos { v, .. }
            | Self::Weave { v, .. }
            | Self::Sine { v, .. } => Some(v),
            _ => None,
        }
    }

    pub const fn dir_deg(&self) -> Option<f32> {
        match *self {
            Self::Line { dir_deg, .. }
            | Self::Strafe { dir_deg, .. }
            | Self::CrossLos { dir_deg, .. }
            | Self::Weave { dir_deg, .. }
            | Self::Sine { dir_deg, .. } => Some(dir_deg),
            _ => None,
        }
    }

    /// Constant chassis rate of [`Primitive::Spin`], in rad/s. A
    /// [`Primitive::SpinSweep`] has a time-varying rate and reports `None`
    /// (use [`Primitive::sweep_peak_omega`] for its peak).
    pub const fn omega(&self) -> Option<f32> {
        match *self {
            Self::Spin { omega } => Some(omega),
            _ => None,
        }
    }

    /// Yaw amplitude of the P2 `spin` sweep, in degrees. The scene-file type of
    /// a sweep is still `spin`; `None` for every other primitive.
    pub const fn yaw_amp_deg(&self) -> Option<f32> {
        match *self {
            Self::SpinSweep { yaw_amp_deg, .. } => Some(yaw_amp_deg),
            _ => None,
        }
    }

    /// Frequency sweep ends of the P2 `spin` sweep, in Hz.
    pub const fn sweep_freqs_hz(&self) -> Option<(f32, f32)> {
        match *self {
            Self::SpinSweep {
                freq_start_hz,
                freq_end_hz,
                ..
            } => Some((freq_start_hz, freq_end_hz)),
            _ => None,
        }
    }

    /// Peak angular rate of the P2 `spin` sweep, in rad/s: `A * 2*pi*f_max`.
    /// `None` unless this is a sweep.
    pub fn sweep_peak_omega(&self) -> Option<f32> {
        match *self {
            Self::SpinSweep {
                yaw_amp_deg,
                freq_start_hz,
                freq_end_hz,
                ..
            } => Some(
                yaw_amp_deg.to_radians()
                    * std::f32::consts::TAU
                    * freq_start_hz.max(freq_end_hz),
            ),
            _ => None,
        }
    }

    pub const fn freq_hz(&self) -> Option<f32> {
        match *self {
            Self::Weave { freq_hz, .. } => Some(freq_hz),
            _ => None,
        }
    }

    pub const fn amp_m(&self) -> Option<f32> {
        match *self {
            Self::Weave { amp_m, .. } | Self::Sine { amp_m, .. } => Some(amp_m),
            _ => None,
        }
    }

    /// Instantaneous ideal (unlimited) velocity in the map frame, in m/s.
    pub fn nominal_velocity(&self, t_local: f32) -> Vec3 {
        match *self {
            Self::Static => Vec3::ZERO,
            Self::Line { v, dir_deg } | Self::Strafe { v, dir_deg } => {
                MotionFrame::world_direction(dir_deg.to_radians()) * v
            }
            Self::CrossLos {
                v,
                dir_deg,
                pass_through,
                duration_s,
            } => {
                if !pass_through && t_local > duration_s * 0.5 {
                    Vec3::ZERO
                } else {
                    MotionFrame::world_direction(dir_deg.to_radians()) * v
                }
            }
            Self::Spin { .. } | Self::SpinSweep { .. } => Vec3::ZERO,
            Self::Weave {
                v,
                dir_deg,
                freq_hz,
                amp_m,
            } => {
                let direction = MotionFrame::world_direction(dir_deg.to_radians());
                let lateral_velocity =
                    amp_m * std::f32::consts::TAU * freq_hz * (std::f32::consts::TAU * freq_hz * t_local).cos();
                direction * v + MotionFrame::right_of(direction) * lateral_velocity
            }
            Self::Sine {
                v,
                dir_deg,
                amp_m,
                freq_start_hz,
                freq_end_hz,
                duration_s,
            } => {
                let direction = MotionFrame::world_direction(dir_deg.to_radians());
                // The instantaneous frequency is what sets the lateral *speed*;
                // the phase (its integral) is what sets the position, so a chirp
                // never teleports sideways at the leg boundary.
                let freq = chirp_freq_hz(freq_start_hz, freq_end_hz, t_local, duration_s);
                let phase = chirp_phase(freq_start_hz, freq_end_hz, t_local, duration_s);
                let lateral_velocity = amp_m * std::f32::consts::TAU * freq * phase.cos();
                direction * v + MotionFrame::right_of(direction) * lateral_velocity
            }
        }
    }

    /// Ideal (unlimited) displacement between two instants of this primitive.
    pub fn nominal_displacement(&self, from: f32, to: f32) -> Vec3 {
        let dt = to - from;
        match *self {
            Self::Static | Self::Spin { .. } | Self::SpinSweep { .. } => Vec3::ZERO,
            Self::Line { v, dir_deg } | Self::Strafe { v, dir_deg } => {
                MotionFrame::world_direction(dir_deg.to_radians()) * (v * dt)
            }
            Self::CrossLos {
                v,
                dir_deg,
                pass_through,
                duration_s,
            } => {
                // Once the target has crossed the line of sight it stops, so the
                // ideal path is the travel up to the leg's midpoint only.
                let covered = if pass_through {
                    dt
                } else {
                    let half = duration_s * 0.5;
                    (to.min(half) - from.min(half)).max(0.0)
                };
                MotionFrame::world_direction(dir_deg.to_radians()) * (v * covered)
            }
            Self::Weave {
                v,
                dir_deg,
                freq_hz,
                amp_m,
            } => {
                let direction = MotionFrame::world_direction(dir_deg.to_radians());
                let omega = std::f32::consts::TAU * freq_hz;
                let lateral = amp_m * ((omega * to).sin() - (omega * from).sin());
                direction * (v * dt) + MotionFrame::right_of(direction) * lateral
            }
            Self::Sine {
                v,
                dir_deg,
                amp_m,
                freq_start_hz,
                freq_end_hz,
                duration_s,
            } => {
                let direction = MotionFrame::world_direction(dir_deg.to_radians());
                let lateral = amp_m
                    * ((chirp_phase(freq_start_hz, freq_end_hz, to, duration_s)).sin()
                        - (chirp_phase(freq_start_hz, freq_end_hz, from, duration_s)).sin());
                direction * (v * dt) + MotionFrame::right_of(direction) * lateral
            }
        }
    }

    /// The stick input this primitive wants at `t_local`.
    ///
    /// `chassis_yaw` is the robot's current world heading (rad); the returned
    /// `chassis_yaw` is a *fraction of `rotation_speed`*, which
    /// `update_chassis_rotation` then ramps with `yaw_acceleration` — so a
    /// reversal turns rather than teleports.
    pub fn evaluate(
        &self,
        t_local: f32,
        chassis_yaw: f32,
        frame: &MotionFrame,
        limits: &MotionLimits,
    ) -> MotionCommand {
        let max_speed = limits.max_speed.max(1e-3);
        match *self {
            Self::Static => MotionCommand::default(),
            Self::Line { v, dir_deg } | Self::Strafe { v, dir_deg } => {
                let heading = dir_deg.to_radians();
                let direction = MotionFrame::world_direction(heading);
                // Signed: `v = -2` is the same heading driven backwards, and the
                // stick is a signed deflection, so the sign must survive.
                let speed_fraction = (v / max_speed).clamp(-1.0, 1.0);
                let motion = frame.input_for_world_direction(direction) * speed_fraction;
                // `line` aligns the chassis with its travel direction; `strafe`
                // keeps the body still and slides (that is the mecanum case the
                // primitive exists for).
                let chassis = match *self {
                    Self::Line { .. } => {
                        yaw_input_towards(chassis_yaw, heading, limits.rotation_speed)
                    }
                    _ => 0.0,
                };
                MotionCommand {
                    movement: clamp_stick(motion),
                    chassis_yaw: chassis,
                    boost: 1.0,
                }
            }
            Self::CrossLos {
                v,
                dir_deg,
                pass_through,
                duration_s,
            } => {
                if !pass_through && t_local > duration_s * 0.5 {
                    return MotionCommand::default();
                }
                let direction = MotionFrame::world_direction(dir_deg.to_radians());
                let speed_fraction = (v / max_speed).clamp(-1.0, 1.0);
                MotionCommand {
                    movement: clamp_stick(
                        frame.input_for_world_direction(direction) * speed_fraction,
                    ),
                    // Exactly `strafe`: the chassis keeps its heading, so the
                    // armour ring does not swing with the travel direction and
                    // the crossing factor this primitive exists for survives.
                    chassis_yaw: 0.0,
                    boost: 1.0,
                }
            }
            Self::Spin { omega } => MotionCommand {
                movement: Vec2::ZERO,
                chassis_yaw: (omega / limits.rotation_speed.abs().max(1e-3)).clamp(-1.0, 1.0),
                boost: 1.0,
            },
            Self::SpinSweep {
                yaw_amp_deg,
                freq_start_hz,
                freq_end_hz,
                duration_s,
            } => {
                // `theta(t) = A*sin(phi(t))` differentiated once gives the rate
                // `omega(t) = A*2*pi*f(t)*cos(phi(t))`. Feeding it through the
                // same rate-to-stick mapping as `spin` means the chassis
                // integration (`update_chassis_rotation`) stays untouched: the
                // sweep only asks for a time-varying rate instead of a constant
                // one. A rate beyond `rotation_speed` would saturate the stick,
                // which is why `clamped` shrinks `A` to the budget.
                let amp_rad = yaw_amp_deg.to_radians();
                let freq = chirp_freq_hz(freq_start_hz, freq_end_hz, t_local, duration_s);
                let phase = chirp_phase(freq_start_hz, freq_end_hz, t_local, duration_s);
                let omega = amp_rad * std::f32::consts::TAU * freq * phase.cos();
                MotionCommand {
                    movement: Vec2::ZERO,
                    chassis_yaw: (omega / limits.rotation_speed.abs().max(1e-3)).clamp(-1.0, 1.0),
                    boost: 1.0,
                }
            }
            Self::Weave { dir_deg, .. } => {
                let velocity = self.nominal_velocity(t_local);
                let speed = velocity.length();
                if speed <= 1e-6 {
                    return MotionCommand {
                        movement: Vec2::ZERO,
                        chassis_yaw: yaw_input_towards(
                            chassis_yaw,
                            dir_deg.to_radians(),
                            limits.rotation_speed,
                        ),
                        boost: 1.0,
                    };
                }
                // The weave's total speed is what gets clamped, not just the
                // forward part: amp/freq that ask for more lateral speed than
                // the vehicle has must show up as path deviation.
                let speed_fraction = (speed / max_speed).clamp(0.0, 1.0);
                MotionCommand {
                    movement: clamp_stick(
                        frame.input_for_world_direction(velocity) * speed_fraction,
                    ),
                    // A serpentine body follows its velocity vector, otherwise it
                    // would crab sideways instead of weaving.
                    chassis_yaw: yaw_input_towards(
                        chassis_yaw,
                        heading_of(velocity),
                        limits.rotation_speed,
                    ),
                    boost: 1.0,
                }
            }
            Self::Sine { .. } => {
                let velocity = self.nominal_velocity(t_local);
                let speed = velocity.length();
                if speed <= 1e-6 {
                    // Standing still inside the sweep: keep the body still rather
                    // than snapping it to `dir_deg` — this primitive never turns
                    // the chassis (the scenario author sets the heading).
                    return MotionCommand::default();
                }
                // Total speed (along + lateral) is what gets clamped, exactly as
                // for `weave`: an amp/freq pair that asks for more lateral speed
                // than the vehicle has must show up as path deviation.
                let speed_fraction = (speed / max_speed).clamp(0.0, 1.0);
                MotionCommand {
                    movement: clamp_stick(
                        frame.input_for_world_direction(velocity) * speed_fraction,
                    ),
                    // Unlike `weave`, the body keeps its heading: the armour ring
                    // stays pointed where the scenario aimed it while the lateral
                    // rate sweeps — that is the P2 chirp axis.
                    chassis_yaw: 0.0,
                    boost: 1.0,
                }
            }
        }
    }
}

/// Guard rails for the P2 `spin` sweep, in the same spirit as the `weave`/`sine`
/// bounds: an amplitude beyond half a turn is not a sweep but an authoring
/// mistake, and 10 Hz is far outside anything the chassis can follow (the
/// rotation-speed cap on `A` bites long before).
const MAX_SWEEP_YAW_AMP_DEG: f32 = 180.0;
const MAX_SWEEP_FREQ_HZ: f32 = 10.0;

/// Instantaneous frequency of a [`Primitive::Sine`] / [`Primitive::SpinSweep`]
/// sweep: linear from `f0` at `t = 0` to `f1` at `t = duration_s` (constant `f0`
/// when the leg is empty).
fn chirp_freq_hz(f0: f32, f1: f32, t: f32, duration_s: f32) -> f32 {
    if duration_s > 1e-6 {
        f0 + (f1 - f0) * (t / duration_s)
    } else {
        f0
    }
}

/// Phase of a [`Primitive::Sine`] sweep, `2π(f₀·t + (f₁−f₀)·t²/(2T))` — the
/// integral of the instantaneous frequency, so the lateral position is
/// continuous and a chirp never jumps sideways at the leg boundary.
fn chirp_phase(f0: f32, f1: f32, t: f32, duration_s: f32) -> f32 {
    let rate = if duration_s > 1e-6 {
        (f1 - f0) / duration_s
    } else {
        0.0
    };
    std::f32::consts::TAU * (f0 * t + 0.5 * rate * t * t)
}

/// First-order heading time constant, in seconds: the commanded yaw rate is
/// `heading_error / tau`, expressed as a fraction of `rotation_speed`.
///
/// 0.5 s is fast enough that a scenario leg which reverses the target (the usual
/// way to keep it near its spawn) completes the turn inside its window, and slow
/// enough that `update_chassis_rotation`'s `yaw_acceleration` ramp is never the
/// bottleneck — so the turn is a smooth slew, never a teleport. With the shipped
/// `rotation_speed = 7.42`, even a 180° error only asks for 85% of the stick;
/// the stick clamp only matters for an artificially slow vehicle.
const YAW_ALIGN_TIME_CONSTANT_S: f32 = 0.5;

fn yaw_input_towards(current_yaw: f32, target_yaw: f32, rotation_speed: f32) -> f32 {
    let error = wrap_pi(target_yaw - current_yaw);
    (error / (rotation_speed.abs().max(1e-3) * YAW_ALIGN_TIME_CONSTANT_S)).clamp(-1.0, 1.0)
}

/// Bevy yaw of a world-space direction on the ground plane: the inverse of
/// [`MotionFrame::world_direction`].
pub fn heading_of(direction: Vec3) -> f32 {
    let flat = direction.with_y(0.0);
    if flat.length_squared() <= 1e-12 {
        return 0.0;
    }
    // world_direction(θ) = (-sin θ, 0, -cos θ), so sin θ = -x and cos θ = -z.
    (-flat.x).atan2(-flat.z)
}

/// Wrap to `(-π, π]`.
pub fn wrap_pi(angle: f32) -> f32 {
    let wrapped = (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU)
        - std::f32::consts::PI;
    if wrapped <= -std::f32::consts::PI {
        wrapped + std::f32::consts::TAU
    } else {
        wrapped
    }
}

/// Local chassis yaw that points the body's front at the ROS `world_heading`.
///
/// The chassis node's rotation is a *local* one, and the model may sit under a
/// parent frame that mirrors it: `HERO.glb`'s scene root (`VEHICLE`) carries a
/// `180°` rotation about Bevy `Z`, so facing azimuth `θ` in the map frame needs
/// the chassis' local yaw `-θ` there, while `vehicle.glb` (an unrotated root)
/// needs `θ`. `parent` is the world rotation of the frame the chassis' local
/// rotation is expressed in, i.e. the chassis' world rotation with its own
/// local rotation divided out.
pub fn local_chassis_yaw(parent: Quat, world_heading: f32) -> f32 {
    // Bevy world forward for ROS heading θ is `(-sin θ, 0, -cos θ)`; the same
    // vector expressed in the parent's frame is what the chassis points at.
    let local = parent.inverse() * MotionFrame::world_direction(world_heading);
    // `Quat::from_euler(YXZ, yaw, 0, 0)` has forward `(-sin yaw, 0, -cos yaw)`.
    (-local.x).atan2(-local.z)
}

/// Sign of `d(world heading) / d(chassis local yaw)` under `parent`.
///
/// `-1.0` when the parent frame mirrors the body (the HERO case), `+1.0`
/// otherwise. The scripted controller reasons in the map frame, so both the
/// heading it reads back and the yaw rate it commands are scaled by this.
pub fn chassis_yaw_sign(parent: Quat, world_heading: f32) -> f32 {
    let base = local_chassis_yaw(parent, world_heading);
    let nudged = local_chassis_yaw(parent, world_heading + 0.05);
    if wrap_pi(nudged - base) < 0.0 { -1.0 } else { 1.0 }
}

fn clamp_stick(input: Vec2) -> Vec2 {
    Vec2::new(input.x.clamp(-1.0, 1.0), input.y.clamp(-1.0, 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The world basis a robot spawned with yaw 0 sees: Bevy `-Z` forward.
    fn frame_yaw0() -> MotionFrame {
        MotionFrame {
            forward: Vec3::NEG_Z,
            right: Vec3::X,
        }
    }

    #[test]
    fn scenario_heading_matches_the_ros_remap() {
        // ROS +X -> Bevy -Z; ROS +Y -> Bevy -X.
        let d0 = MotionFrame::world_direction(0.0);
        assert!((d0 - Vec3::NEG_Z).length() < 1e-6, "{d0:?}");

        let d90 = MotionFrame::world_direction(std::f32::consts::FRAC_PI_2);
        assert!((d90 - Vec3::NEG_X).length() < 1e-6, "{d90:?}");

        let d180 = MotionFrame::world_direction(std::f32::consts::PI);
        assert!((d180 - Vec3::Z).length() < 1e-6, "{d180:?}");
    }

    #[test]
    fn heading_of_inverts_world_direction() {
        for deg in [-170.0f32, -90.0, 0.0, 45.0, 90.0, 179.0] {
            let heading = deg.to_radians();
            let direction = MotionFrame::world_direction(heading);
            let recovered = heading_of(direction);
            assert!(
                (wrap_pi(recovered - heading)).abs() < 1e-5,
                "deg={deg} recovered={recovered}"
            );
        }
    }

    #[test]
    fn line_asks_for_full_speed_along_its_heading_and_aligns_the_body() {
        let limits = MotionLimits::default();
        let frame = frame_yaw0();
        // 6 m/s is exactly max_speed: a full stick, no boost.
        let cmd = Primitive::Line {
            v: 6.0,
            dir_deg: 0.0,
        }
        .evaluate(0.0, 0.0, &frame, &limits);

        assert!((cmd.movement.length() - 1.0).abs() < 1e-5);
        assert!((cmd.movement.y - 1.0).abs() < 1e-5, "forward = +Y stick");
        assert!(cmd.chassis_yaw.abs() < 1e-5, "already aligned");
        assert_eq!(cmd.boost, 1.0);
    }

    #[test]
    fn line_refuses_to_exceed_max_speed_and_turns_towards_its_heading() {
        let limits = MotionLimits::default();
        let frame = frame_yaw0();
        // 12 m/s is twice the vehicle: the stick saturates instead of lying.
        let cmd = Primitive::Line {
            v: 12.0,
            dir_deg: 0.0,
        }
        .evaluate(0.0, 0.0, &frame, &limits);
        assert!((cmd.movement.length() - 1.0).abs() < 1e-5, "clamped to 6 m/s");

        // 90 deg away: commanded yaw rate = error / tau, as a stick fraction.
        let cmd = Primitive::Line {
            v: 2.0,
            dir_deg: 90.0,
        }
        .evaluate(0.0, 0.0, &frame, &limits);
        let expected =
            std::f32::consts::FRAC_PI_2 / (limits.rotation_speed * YAW_ALIGN_TIME_CONSTANT_S);
        assert!((cmd.chassis_yaw - expected).abs() < 1e-5, "{}", cmd.chassis_yaw);

        // 180 deg opposite: a big but bounded stick, never a teleport.
        let cmd = Primitive::Line {
            v: 2.0,
            dir_deg: 180.0,
        }
        .evaluate(0.0, 0.0, &frame, &limits);
        assert!(
            cmd.chassis_yaw > 0.8 && cmd.chassis_yaw < 1.0,
            "180 deg error is 85% of the stick at the shipped rotation speed: {}",
            cmd.chassis_yaw
        );

        // The clamp is real: with a slow vehicle an error this large asks for
        // more than the full stick and is bounded to it.
        let slow = MotionLimits {
            rotation_speed: 2.0,
            ..limits
        };
        let cmd = Primitive::Line {
            v: 2.0,
            dir_deg: 180.0,
        }
        .evaluate(0.0, 0.0, &frame, &slow);
        assert!((cmd.chassis_yaw - 1.0).abs() < 1e-5, "{}", cmd.chassis_yaw);
    }

    #[test]
    fn negative_speed_drives_the_same_heading_backwards() {
        let limits = MotionLimits::default();
        let frame = frame_yaw0();

        let forward = Primitive::Strafe {
            v: 3.0,
            dir_deg: 0.0,
        }
        .evaluate(0.0, 0.0, &frame, &limits);
        let backward = Primitive::Strafe {
            v: -3.0,
            dir_deg: 0.0,
        }
        .evaluate(0.0, 0.0, &frame, &limits);

        assert!(
            (forward.movement + backward.movement).length() < 1e-6,
            "same heading, opposite stick: {forward:?} vs {backward:?}"
        );
    }

    #[test]
    fn strafe_moves_without_turning_the_body() {
        let limits = MotionLimits::default();
        let frame = frame_yaw0();
        let cmd = Primitive::Strafe {
            v: 3.0,
            dir_deg: 0.0,
        }
        .evaluate(5.0, 0.4, &frame, &limits);

        assert_eq!(cmd.chassis_yaw, 0.0, "pure sidestep");
        assert!((cmd.movement.y - 0.5).abs() < 1e-5, "3 of 6 m/s");
        assert!(cmd.movement.x.abs() < 1e-5);
    }

    #[test]
    fn spin_asks_for_a_constant_rate_within_the_vehicle_limit() {
        let limits = MotionLimits::default();
        let frame = frame_yaw0();

        // The physical ceiling is 7.42 rad/s; asking for it is a full stick and
        // the yaw_acceleration ramp in update_chassis_rotation does the rest.
        let cmd = Primitive::Spin { omega: 7.42 }.evaluate(0.0, 0.0, &frame, &limits);
        assert!((cmd.chassis_yaw - 1.0).abs() < 1e-5);
        assert_eq!(cmd.movement, Vec2::ZERO, "in place");

        let cmd = Primitive::Spin { omega: 3.0 }.evaluate(0.0, 0.0, &frame, &limits);
        assert!((cmd.chassis_yaw - 3.0 / 7.42).abs() < 1e-5);
        assert_eq!(cmd.chassis_yaw.signum(), 1.0);

        let cmd = Primitive::Spin { omega: -3.0 }.evaluate(0.0, 0.0, &frame, &limits);
        assert!(cmd.chassis_yaw < 0.0, "reverse spin stays reverse");
    }

    #[test]
    fn weave_is_a_serpentine_not_a_straight_line() {
        let limits = MotionLimits::default();
        let frame = frame_yaw0();
        let weave = Primitive::Weave {
            v: 2.0,
            dir_deg: 0.0,
            freq_hz: 0.5,
            amp_m: 1.0,
        };

        // At t = 0 the lateral velocity is at its maximum (cos(0) = 1).
        let v0 = weave.nominal_velocity(0.0);
        assert!((v0.z + 2.0).abs() < 1e-4, "forward 2 m/s along -Z: {v0:?}");
        assert!(
            v0.x.abs() > 1.0,
            "lateral component must be present at t=0: {v0:?}"
        );

        // A quarter period later the lateral velocity is zero and the robot is
        // at its lateral extreme.
        let quarter = 0.25 / 0.5;
        let v_quarter = weave.nominal_velocity(quarter);
        assert!(
            v_quarter.x.abs() < 1e-3,
            "lateral velocity crosses zero at the extreme: {v_quarter:?}"
        );

        // The displacement integral must equal the offset that generated it.
        let offset = weave.nominal_displacement(0.0, quarter);
        let expected_lateral = 1.0; // amp * sin(2*pi*f*t) at t = quarter period
        let lateral = offset - MotionFrame::world_direction(0.0) * (2.0 * quarter);
        assert!(
            (lateral.length() - expected_lateral).abs() < 1e-3,
            "lateral offset {lateral:?}"
        );

        // And the stick stays inside the physical budget.
        let cmd = weave.evaluate(0.0, 0.0, &frame, &limits);
        assert!(cmd.movement.length() <= 1.0 + 1e-6);
    }

    #[test]
    fn weave_clamps_the_total_speed_including_the_lateral_part() {
        let limits = MotionLimits::default();
        let frame = frame_yaw0();
        // 6 m/s forward plus up to 2*pi*1 = 6.3 m/s of lateral velocity: the
        // vector must be clamped to the 6 m/s budget, not the forward part.
        let cmd = Primitive::Weave {
            v: 6.0,
            dir_deg: 0.0,
            freq_hz: 1.0,
            amp_m: 1.0,
        }
        .evaluate(0.0, 0.0, &frame, &limits);
        assert!(
            (cmd.movement.length() - 1.0).abs() < 1e-5,
            "total speed clamped: {:?}",
            cmd.movement
        );
    }

    #[test]
    fn sine_with_one_frequency_is_a_pure_lateral_sine() {
        let limits = MotionLimits::default();
        let frame = frame_yaw0();
        let sine = Primitive::Sine {
            v: 2.0,
            dir_deg: 0.0,
            amp_m: 1.0,
            freq_start_hz: 0.5,
            freq_end_hz: 0.5,
            duration_s: 4.0,
        };

        // t = 0: lateral velocity is at its maximum, `amp * TAU * f`.
        let v0 = sine.nominal_velocity(0.0);
        assert!((v0.z + 2.0).abs() < 1e-4, "forward 2 m/s along -Z: {v0:?}");
        assert!(
            (v0.x - std::f32::consts::TAU * 0.5).abs() < 1e-4,
            "lateral peak amp*TAU*f: {v0:?}"
        );

        // A full period (2 s) later the lateral offset is back to zero, so the
        // ideal displacement is purely along the travel direction.
        let whole = sine.nominal_displacement(0.0, 2.0);
        assert!(
            whole.x.abs() < 1e-4 && (whole.z + 4.0).abs() < 1e-4,
            "one whole period has no net lateral offset: {whole:?}"
        );

        // A quarter period (0.5 s at 0.5 Hz) later the lateral offset is at its
        // extreme: +amp to the right of the travel direction.
        let quarter = sine.nominal_displacement(0.0, 0.5);
        assert!(
            (quarter.x - 1.0).abs() < 1e-3,
            "a quarter period reaches +amp laterally: {quarter:?}"
        );

        // Half a period later it is back on the travel line.
        let half = sine.nominal_displacement(0.0, 1.0);
        assert!(
            half.x.abs() < 1e-3,
            "half a period is back on the line: {half:?}"
        );

        // The body is never asked to turn, and the stick stays legal.
        for t in [0.0, 0.7, 1.5, 3.9] {
            let cmd = sine.evaluate(t, 0.0, &frame, &limits);
            assert_eq!(cmd.chassis_yaw, 0.0, "sine holds the chassis heading");
            assert!(cmd.movement.length() <= 1.0 + 1e-6, "stick legal at t={t}");
        }
    }

    #[test]
    fn sine_sweep_follows_the_chirp_law() {
        // Independent implementation of φ(t) = 2π(f₀t + (f₁−f₀)t²/(2T)).
        let phase = |f0: f32, f1: f32, t: f32, duration: f32| {
            std::f32::consts::TAU * (f0 * t + (f1 - f0) * t * t / (2.0 * duration))
        };
        let sine = Primitive::Sine {
            v: 0.0,
            dir_deg: 0.0,
            amp_m: 1.0,
            freq_start_hz: 0.5,
            freq_end_hz: 1.0,
            duration_s: 2.0,
        };
        let direction = MotionFrame::world_direction(0.0);
        let right = MotionFrame::right_of(direction);

        for t in [0.25_f32, 0.5, 1.0, 1.5, 2.0] {
            let disp = sine.nominal_displacement(0.0, t);
            let expected = right * phase(0.5, 1.0, t, 2.0).sin();
            assert!(
                (disp - expected).length() < 1e-4,
                "chirp dispatch at t={t}: {disp:?} vs {expected:?}"
            );
        }

        // At t = T the sweep ends: φ(T) = 2π·T·(f₀+f₁)/2 = 3π ⇒ sin = 0, and the
        // instantaneous frequency is the end frequency.
        let v_end = sine.nominal_velocity(2.0);
        let expected_lateral = std::f32::consts::TAU * 1.0 * phase(0.5, 1.0, 2.0, 2.0).cos();
        assert!(
            (v_end.dot(right) - expected_lateral).abs() < 1e-3,
            "end frequency is f₁: {v_end:?}"
        );
    }

    #[test]
    fn sine_clamps_amp_and_both_frequency_ends() {
        let limits = MotionLimits::default();
        let (primitive, clamped) = Primitive::Sine {
            v: 9.0,
            dir_deg: 30.0,
            amp_m: 25.0,
            freq_start_hz: -1.0,
            freq_end_hz: 99.0,
            duration_s: 3.0,
        }
        .clamped(&limits);
        assert!(clamped);
        assert_eq!(primitive.v(), Some(6.0));
        assert_eq!(primitive.amp_m(), Some(20.0));
        assert_eq!(primitive.kind(), "sine");
        match primitive {
            Primitive::Sine {
                freq_start_hz,
                freq_end_hz,
                duration_s,
                ..
            } => {
                assert_eq!(freq_start_hz, 0.0);
                assert_eq!(freq_end_hz, 10.0);
                assert_eq!(duration_s, 3.0, "duration is carried through clamping");
            }
            other => panic!("clamping must preserve the variant: {other:?}"),
        }

        // In-range parameters are left alone.
        let (same, clamped) = Primitive::Sine {
            v: 2.0,
            dir_deg: 0.0,
            amp_m: 0.5,
            freq_start_hz: 0.2,
            freq_end_hz: 3.0,
            duration_s: 5.0,
        }
        .clamped(&limits);
        assert!(!clamped);
        assert_eq!(same.amp_m(), Some(0.5));
        assert_eq!(same.kind(), "sine");
    }

    #[test]
    fn sine_with_no_motion_is_static() {
        let limits = MotionLimits::default();
        let frame = frame_yaw0();
        let cmd = Primitive::Sine {
            v: 0.0,
            dir_deg: 0.0,
            amp_m: 0.0,
            freq_start_hz: 1.0,
            freq_end_hz: 1.0,
            duration_s: 2.0,
        }
        .evaluate(0.5, 0.0, &frame, &limits);
        assert_eq!(cmd.movement, Vec2::ZERO, "no forward speed, no amplitude");
        assert_eq!(cmd.chassis_yaw, 0.0);
    }

    #[test]
    fn clamping_reports_what_it_changed() {
        let limits = MotionLimits::default();

        let (primitive, clamped) = Primitive::Line {
            v: 9.0,
            dir_deg: 0.0,
        }
        .clamped(&limits);
        assert!(clamped);
        assert_eq!(primitive.v(), Some(6.0));

        let (primitive, clamped) = Primitive::Spin { omega: 20.0 }.clamped(&limits);
        assert!(clamped);
        assert_eq!(primitive.omega(), Some(7.42));

        let (_, clamped) = Primitive::Spin { omega: 3.0 }.clamped(&limits);
        assert!(!clamped);
    }

    #[test]
    fn static_wants_nothing() {
        let cmd = Primitive::Static.evaluate(12.0, 1.0, &frame_yaw0(), &MotionLimits::default());

        assert_eq!(cmd, MotionCommand::default());
        assert_eq!(Primitive::Static.nominal_displacement(0.0, 5.0), Vec3::ZERO);
    }

    #[test]
    fn wrap_pi_keeps_errors_short() {
        use std::f32::consts::PI;

        assert!((wrap_pi(0.0) - 0.0).abs() < 1e-6);
        assert!((wrap_pi(PI - 0.1) - (PI - 0.1)).abs() < 1e-5);
        assert!((wrap_pi(PI + 0.1) + (PI - 0.1)).abs() < 1e-5, "wraps across");
        assert!((wrap_pi(-PI - 0.1) - (PI - 0.1)).abs() < 1e-5);
        assert!((wrap_pi(3.0 * PI) - PI).abs() < 1e-5);
    }

    /// `HERO.glb`'s scene root carries a 180° rotation about Bevy `Z`, so the
    /// chassis' local yaw is the *negative* of the map-frame heading it faces.
    /// Both directions have to round-trip, for every model.
    #[test]
    fn chassis_yaw_round_trips_through_a_mirrored_parent() {
        use std::f32::consts::PI;

        // vehicle.glb: unrotated model root — a straight passthrough.
        assert_eq!(chassis_yaw_sign(Quat::IDENTITY, 1.9), 1.0);
        assert!((wrap_pi(local_chassis_yaw(Quat::IDENTITY, 1.9) - 1.9)).abs() < 1e-5);

        // HERO.glb: mirrored parent — the local yaw is negated and so is the loop.
        let mirrored = Quat::from_rotation_z(PI);
        assert_eq!(chassis_yaw_sign(mirrored, 1.9), -1.0);
        assert!((wrap_pi(local_chassis_yaw(mirrored, 1.9) + 1.9)).abs() < 1e-5);

        for deg in [-170.0f32, -108.0, -45.0, 0.0, 45.0, 108.0, 170.0] {
            let heading = deg.to_radians();
            for parent in [Quat::IDENTITY, mirrored] {
                let local = local_chassis_yaw(parent, heading);
                // What `update_chassis_rotation` writes is `from_euler(YXZ, local, 0, 0)`.
                let world = parent * Quat::from_euler(EulerRot::YXZ, local, 0.0, 0.0);
                let faced = heading_of(world * Vec3::NEG_Z);
                assert!(
                    (wrap_pi(faced - heading)).abs() < 1e-4,
                    "{deg}° under {parent:?} faced {}°",
                    faced.to_degrees()
                );
                // The scripted controller reads back the map-frame heading and
                // feeds its rate back into the same mirrored frame.
                let read_back = chassis_yaw_sign(parent, heading) * local;
                assert!((wrap_pi(read_back - heading)).abs() < 1e-4);
            }
        }
    }

    #[test]
    fn cross_los_slides_across_and_never_turns_the_body() {
        let limits = MotionLimits::default();
        let frame = frame_yaw0();
        // Body facing ROS +X, sliding along ROS +Y at half speed: a pure
        // sidestep, and the armour ring must stay where it was.
        let cmd = Primitive::CrossLos {
            v: 3.0,
            dir_deg: 90.0,
            pass_through: true,
            duration_s: 4.0,
        }
        .evaluate(1.0, 1.0, &frame, &limits);

        assert!((cmd.movement.x + 0.5).abs() < 1e-5, "{:?}", cmd.movement);
        assert!(cmd.movement.y.abs() < 1e-5, "no forward stick");
        assert!(cmd.chassis_yaw.abs() < 1e-6, "heading preserved");
        assert_eq!(cmd.boost, 1.0);

        // Even when the body points the other way, `cross_los` asks for zero
        // yaw: that is what keeps the crossing factor (the whole point of the
        // primitive) from being eaten by a body turn.
        let cmd = Primitive::CrossLos {
            v: 3.0,
            dir_deg: 0.0,
            pass_through: true,
            duration_s: 4.0,
        }
        .evaluate(1.0, std::f32::consts::PI, &frame, &limits);
        assert!(cmd.chassis_yaw.abs() < 1e-6);
    }

    #[test]
    fn cross_los_without_pass_through_coasts_from_the_midpoint() {
        let primitive = Primitive::CrossLos {
            v: 3.0,
            dir_deg: 90.0,
            pass_through: false,
            duration_s: 4.0,
        };
        let limits = MotionLimits::default();
        let frame = frame_yaw0();

        let driving = primitive.evaluate(1.9, 1.0, &frame, &limits);
        assert!((driving.movement.length() - 0.5).abs() < 1e-5);

        let coasting = primitive.evaluate(2.1, 1.0, &frame, &limits);
        assert_eq!(coasting, MotionCommand::default(), "no stick after the LOS");

        // The ideal path stops at the crossing too, so the deviation metric
        // measures the author's trajectory and not the coast.
        let full = MotionFrame::world_direction(std::f32::consts::FRAC_PI_2) * (3.0 * 2.0);
        assert!((primitive.nominal_displacement(0.0, 4.0) - full).length() < 1e-5);
        assert!((primitive.nominal_velocity(1.9).length() - 3.0).abs() < 1e-5);
        assert_eq!(primitive.nominal_velocity(2.1), Vec3::ZERO);

        // `pass_through` keeps sliding for the whole window.
        let through = Primitive::CrossLos {
            v: 3.0,
            dir_deg: 90.0,
            pass_through: true,
            duration_s: 4.0,
        };
        assert!((through.nominal_displacement(0.0, 4.0) - full * 2.0).length() < 1e-5);
    }

    #[test]
    fn cross_los_clamps_speed_and_reports_its_kind() {
        let primitive = Primitive::CrossLos {
            v: 12.0,
            dir_deg: 90.0,
            pass_through: false,
            duration_s: 4.0,
        };
        assert_eq!(primitive.kind(), "cross_los");
        assert_eq!(primitive.v(), Some(12.0));
        assert_eq!(primitive.dir_deg(), Some(90.0));

        let (clamped, did_clamp) = primitive.clamped(&MotionLimits::default());
        assert!(did_clamp);
        assert_eq!(clamped.v(), Some(6.0), "clamped to max_speed");
        assert_eq!(clamped.dir_deg(), Some(90.0));
        assert_eq!(clamped.kind(), "cross_los");
    }

    #[test]
    fn spin_keeps_its_constant_rate_path_unchanged() {
        // Regression guard for the P0 path: adding the sweep must not touch the
        // constant-rate `spin` in any way (same command, same clamp, same kind).
        let limits = MotionLimits::default();
        let frame = frame_yaw0();

        let cmd = Primitive::Spin { omega: 3.0 }.evaluate(0.0, 0.0, &frame, &limits);
        assert!((cmd.chassis_yaw - 3.0 / 7.42).abs() < 1e-6);
        assert_eq!(cmd.movement, Vec2::ZERO, "in place");
        assert_eq!(cmd.boost, 1.0);
        assert_eq!(
            Primitive::Spin { omega: 3.0 }.nominal_velocity(1.0),
            Vec3::ZERO
        );
        assert_eq!(
            Primitive::Spin { omega: 3.0 }.nominal_displacement(0.0, 5.0),
            Vec3::ZERO
        );
        assert_eq!(Primitive::Spin { omega: 3.0 }.omega(), Some(3.0));
        assert_eq!(Primitive::Spin { omega: 3.0 }.kind(), "spin");
        assert_eq!(Primitive::Spin { omega: 3.0 }.yaw_amp_deg(), None);

        let (clamped, did_clamp) = Primitive::Spin { omega: 20.0 }.clamped(&limits);
        assert!(did_clamp);
        assert_eq!(clamped.omega(), Some(7.42));
    }

    #[test]
    fn spin_sweep_follows_the_chirp_law() {
        // Independent implementation of ω(t) = A·2π·f(t)·cos φ(t), φ being the
        // integral of f(t) = f₀ + (f₁−f₀)·t/T.
        let phase = |f0: f32, f1: f32, t: f32, duration: f32| {
            std::f32::consts::TAU * (f0 * t + (f1 - f0) * t * t / (2.0 * duration))
        };
        let amplitude_deg = 30.0_f32;
        let sweep = Primitive::SpinSweep {
            yaw_amp_deg: amplitude_deg,
            freq_start_hz: 0.2,
            freq_end_hz: 1.2,
            duration_s: 4.0,
        };
        let limits = MotionLimits::default();
        let frame = frame_yaw0();

        for t in [0.0_f32, 0.5, 1.0, 2.0, 3.5, 4.0] {
            let freq = 0.2 + (1.2 - 0.2) * (t / 4.0);
            let omega = amplitude_deg.to_radians()
                * std::f32::consts::TAU
                * freq
                * phase(0.2, 1.2, t, 4.0).cos();
            let cmd = sweep.evaluate(t, 0.0, &frame, &limits);
            assert_eq!(cmd.movement, Vec2::ZERO, "the hull turns, the robot stays put");
            assert!(
                (cmd.chassis_yaw - (omega / 7.42).clamp(-1.0, 1.0)).abs() < 1e-5,
                "rate at t={t}: {} vs {omega}",
                cmd.chassis_yaw * 7.42
            );
        }

        // No translation and no heading alignment, exactly like the constant-rate
        // spin; the scene-file kind stays `spin`.
        assert_eq!(sweep.nominal_velocity(1.0), Vec3::ZERO);
        assert_eq!(sweep.nominal_displacement(0.0, 4.0), Vec3::ZERO);
        assert_eq!(sweep.kind(), "spin");
        assert_eq!(sweep.omega(), None, "the rate is time-varying");
        assert_eq!(sweep.yaw_amp_deg(), Some(amplitude_deg));
        assert_eq!(sweep.sweep_freqs_hz(), Some((0.2, 1.2)));
        let peak = sweep.sweep_peak_omega().unwrap();
        let expected_peak = amplitude_deg.to_radians() * std::f32::consts::TAU * 1.2;
        assert!((peak - expected_peak).abs() < 1e-5);
    }

    #[test]
    fn spin_sweep_clamps_the_yaw_amplitude_to_the_rate_limit() {
        // The rate peak is A·2π·f_max, so the amplitude has to shrink to
        // `rotation_speed`'s worth: A = ω_max/(2π·f_max), and the clamp is
        // reported through `did_clamp`.
        let limits = MotionLimits::default();
        let (clamped, did_clamp) = Primitive::SpinSweep {
            yaw_amp_deg: 180.0,
            freq_start_hz: 0.2,
            freq_end_hz: 2.0,
            duration_s: 6.0,
        }
        .clamped(&limits);
        assert!(did_clamp);
        let expected_deg = 7.42_f32.to_degrees() / (std::f32::consts::TAU * 2.0);
        let amp = clamped.yaw_amp_deg().unwrap();
        assert!((amp - expected_deg).abs() < 1e-3, "{amp} vs {expected_deg}");
        assert!((clamped.sweep_peak_omega().unwrap() - 7.42).abs() < 1e-3);
        let frame = frame_yaw0();
        for t in [0.0_f32, 0.25, 1.0, 2.0, 3.0] {
            let cmd = clamped.evaluate(t, 0.0, &frame, &limits);
            assert!(cmd.chassis_yaw.abs() <= 1.0 + 1e-6, "the stick never saturates");
        }

        // A peak inside the budget is left alone.
        let (same, did_clamp) = Primitive::SpinSweep {
            yaw_amp_deg: 5.0,
            freq_start_hz: 0.2,
            freq_end_hz: 1.2,
            duration_s: 6.0,
        }
        .clamped(&limits);
        assert!(!did_clamp);
        assert_eq!(same.yaw_amp_deg(), Some(5.0));

        // A single frequency is the "sinusoidal angular rate" spelling:
        // ω(t) = ω_max·cos(2πft) is exactly A = ω_max/(2πf), so one mode covers
        // both ways of asking for the same motion.
        let target_omega = 3.0_f32;
        let freq = 0.5_f32;
        let single = Primitive::SpinSweep {
            yaw_amp_deg: (target_omega / (std::f32::consts::TAU * freq)).to_degrees(),
            freq_start_hz: freq,
            freq_end_hz: freq,
            duration_s: 2.0,
        };
        let cmd = single.evaluate(0.0, 0.0, &frame, &limits);
        assert!(
            (cmd.chassis_yaw * 7.42 - target_omega).abs() < 1e-4,
            "ω(0) is the requested peak"
        );
    }

    #[test]
    fn spin_sweep_turns_the_hull_while_the_lateral_sine_holds_it() {
        // The semantic contrast the bench asked for: the sweep is a *spin*
        // variant, so the armour ring rotates through it, while `sine` is a
        // translation that deliberately keeps the heading (and has a lateral
        // acceleration ceiling the sweep does not have).
        let limits = MotionLimits::default();
        let frame = frame_yaw0();
        let sweep = Primitive::SpinSweep {
            yaw_amp_deg: 20.0,
            freq_start_hz: 0.5,
            freq_end_hz: 0.5,
            duration_s: 2.0,
        };
        let sine = Primitive::Sine {
            v: 0.0,
            dir_deg: 0.0,
            amp_m: 0.5,
            freq_start_hz: 0.5,
            freq_end_hz: 0.5,
            duration_s: 2.0,
        };

        assert_ne!(
            sweep.evaluate(0.0, 0.0, &frame, &limits).chassis_yaw,
            0.0,
            "the sweep turns the hull"
        );
        assert_eq!(sweep.nominal_velocity(0.0), Vec3::ZERO, "no translation");
        assert_eq!(
            sine.evaluate(0.0, 0.0, &frame, &limits).chassis_yaw,
            0.0,
            "the lateral sine holds the heading"
        );
        assert!(sine.nominal_velocity(0.0).length() > 0.0, "and it translates");
    }
}
