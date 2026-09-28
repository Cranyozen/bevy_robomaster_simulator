use bevy::prelude::*;
use std::sync::atomic::AtomicBool;

#[derive(Component)]
pub struct MainCamera {
    pub follow_offset: Vec3,
}

#[derive(Resource, PartialEq, Deref, DerefMut)]
pub struct CameraMode(pub FollowingType);

impl Default for CameraMode {
    fn default() -> Self {
        Self(FollowingType::Robot)
    }
}

#[derive(Resource, Deref, DerefMut)]
pub struct SubscribeAutoAim(pub AtomicBool);

#[derive(PartialEq, Clone, Copy)]
pub enum FollowingType {
    Free,
    Robot,
    ThirdPerson,
}

/// Which robot the follow camera tracks. `F3` picks the view style, `O` picks the
/// robot the view is attached to.
#[derive(Resource, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub enum CameraFocus {
    #[default]
    Own,
    Slapper,
}

impl CameraFocus {
    pub fn toggle(&mut self) {
        *self = match self {
            Self::Own => Self::Slapper,
            Self::Slapper => Self::Own,
        };
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::Own => "own",
            Self::Slapper => "slapper",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn focus_defaults_to_the_own_robot_and_toggles_back_and_forth() {
        let mut focus = CameraFocus::default();
        assert_eq!(focus, CameraFocus::Own);
        assert_eq!(focus.label(), "own");

        focus.toggle();
        assert_eq!(focus, CameraFocus::Slapper);
        assert_eq!(focus.label(), "slapper");

        focus.toggle();
        assert_eq!(focus, CameraFocus::Own);
    }
}
