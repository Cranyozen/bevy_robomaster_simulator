use bevy::math::{Mat3, Quat, Vec3};
use bevy::prelude::Transform;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub struct AverageRateLimiter {
    period: Duration,
    elapsed: Duration,
}

impl AverageRateLimiter {
    pub fn new(period: Duration) -> Self {
        Self {
            period,
            elapsed: period,
        }
    }

    pub fn from_hz(hz: f32) -> Self {
        assert!(hz.is_finite() && hz > 0.0);
        Self::new(Duration::from_secs_f32(1.0 / hz))
    }

    pub fn tick(&mut self, delta: Duration) {
        self.elapsed = self.elapsed.saturating_add(delta).min(self.period);
    }

    pub fn allow(&mut self) -> bool {
        if self.elapsed < self.period {
            return false;
        }
        self.elapsed = Duration::ZERO;
        true
    }
}

/// Wall-clock rate gate: "has `period` elapsed since the last time I said yes?".
///
/// [`AverageRateLimiter`] is driven by the caller ticking it with a frame delta,
/// which is what the `Update`-driven publishers want. This one is driven by
/// `Instant::now()` instead, because the capture path has to decide *inside a
/// render-graph node* (and inside the render world) whether this frame is worth
/// reading back from the GPU at all — there is no `Res<Time>` to tick with, and
/// the decision has to be made before any work is done.
///
/// `period` of `None` (or [`Duration::ZERO`]) always allows, so "no limit" needs
/// no special case at the call site.
#[derive(Debug, Clone)]
pub struct WallRateLimiter {
    period: Option<Duration>,
    last: Option<Instant>,
}

impl WallRateLimiter {
    pub fn new(period: Option<Duration>) -> Self {
        Self {
            period: period.filter(|p| !p.is_zero()),
            last: None,
        }
    }

    /// `None` disables the gate; `Some(period)` allows at most one call per `period`.
    pub fn from_period(period: Option<Duration>) -> Self {
        Self::new(period)
    }

    /// Whether this call may proceed. The first call after construction always
    /// may — otherwise a 1 Hz `/camera_info` gate would delay startup by a
    /// second, and the detector needs the intrinsics before its first solve.
    pub fn allow(&mut self) -> bool {
        self.allow_at(Instant::now())
    }

    /// [`Self::allow`] against an explicit clock, so the behaviour is testable
    /// without sleeping.
    pub fn allow_at(&mut self, now: Instant) -> bool {
        let Some(period) = self.period else {
            self.last = Some(now);
            return true;
        };
        let due = match self.last {
            None => true,
            Some(last) => now.saturating_duration_since(last) >= period,
        };
        if due {
            self.last = Some(now);
        }
        due
    }
}

pub const M_ALIGN_MAT3: Mat3 = Mat3::from_cols(
    Vec3::new(0.0, -1.0, 0.0), // M[0,0], M[1,0], M[2,0]
    Vec3::new(0.0, 0.0, 1.0),  // M[0,1], M[1,1], M[2,1]
    Vec3::new(-1.0, 0.0, 0.0), // M[0,2], M[1,2], M[2,2]
);

#[inline]
pub fn transform(bevy_transform: Transform) -> r2r::geometry_msgs::msg::Transform {
    let align_rot_mat = M_ALIGN_MAT3;
    let align_quat = Quat::from_mat3(&align_rot_mat);
    let new_rotation = align_quat * bevy_transform.rotation * align_quat.inverse();
    let new_translation = align_rot_mat * bevy_transform.translation;
    r2r::geometry_msgs::msg::Transform {
        translation: r2r::geometry_msgs::msg::Vector3 {
            x: new_translation.x as f64,
            y: new_translation.y as f64,
            z: new_translation.z as f64,
        },
        rotation: r2r::geometry_msgs::msg::Quaternion {
            x: new_rotation.x as f64,
            y: new_rotation.y as f64,
            z: new_rotation.z as f64,
            w: new_rotation.w as f64,
        },
    }
}

#[macro_export]
macro_rules! add_tf_frame {
    ($ls:ident, $hdr:expr, $id:expr, $translation:expr, $rotation:expr) => {
        $ls.push(::r2r::geometry_msgs::msg::TransformStamped {
            header: $hdr.clone(),
            child_frame_id: $id.to_string(),
            transform: $crate::ros2::prelude::transform(
                ::bevy::prelude::Transform::IDENTITY
                    .with_translation($translation)
                    .with_rotation($rotation),
            ),
        });
    };
    ($ls:ident, $hdr:expr, $id:expr, $transform:expr) => {
        $ls.push(::r2r::geometry_msgs::msg::TransformStamped {
            header: $hdr.clone(),
            child_frame_id: $id.to_string(),
            transform: $crate::ros2::prelude::transform($transform),
        });
    };
}

#[macro_export]
macro_rules! pose {
    ($hdr:expr) => {
        ::r2r::geometry_msgs::msg::PoseStamped {
            header: $hdr.clone(),
            pose: ::r2r::geometry_msgs::msg::Pose {
                position: ::r2r::geometry_msgs::msg::Point {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                orientation: ::r2r::geometry_msgs::msg::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unlimited_gate_allows_every_call() {
        let mut gate = WallRateLimiter::from_period(None);
        let t0 = Instant::now();

        for step in 0..5 {
            assert!(
                gate.allow_at(t0 + Duration::from_micros(step)),
                "a disabled gate must not throttle"
            );
        }

        let mut zero = WallRateLimiter::from_period(Some(Duration::ZERO));
        assert!(zero.allow_at(t0));
        assert!(zero.allow_at(t0), "Duration::ZERO is the same as no limit");
    }

    #[test]
    fn gate_allows_the_first_call_then_waits_out_the_period() {
        let t0 = Instant::now();
        let mut gate = WallRateLimiter::from_period(Some(Duration::from_millis(33)));

        assert!(gate.allow_at(t0), "the first frame must not be delayed");
        assert!(!gate.allow_at(t0 + Duration::from_millis(32)));
        assert!(gate.allow_at(t0 + Duration::from_millis(33)));
        assert!(
            !gate.allow_at(t0 + Duration::from_millis(34)),
            "the gate re-anchors on the accepted call, not on every call"
        );
        assert!(gate.allow_at(t0 + Duration::from_millis(66)));
    }

    #[test]
    fn gate_catches_up_without_bursting() {
        let t0 = Instant::now();
        let mut gate = WallRateLimiter::from_period(Some(Duration::from_millis(10)));

        assert!(gate.allow_at(t0));
        // A long stall (a hitch, a paused window) must not queue up a burst of
        // publishes: one frame may go out, the rest of the backlog is dropped.
        assert!(gate.allow_at(t0 + Duration::from_secs(5)));
        assert!(!gate.allow_at(t0 + Duration::from_secs(5)));
    }

    #[test]
    fn average_limiter_still_ticks() {
        // Built from an exact Duration: `from_hz` rounds through f32, which would
        // make a nominal 100 ms period a nanosecond longer than tick(100 ms).
        let mut limiter = AverageRateLimiter::new(Duration::from_millis(100));

        assert!(limiter.allow());
        assert!(!limiter.allow());
        limiter.tick(Duration::from_millis(100));
        assert!(limiter.allow());
    }
}
