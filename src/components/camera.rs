use bevy::prelude::*;
use std::sync::atomic::AtomicBool;

#[derive(Component)]
pub struct MainCamera {
    pub follow_offset: Vec3,
}

/// Off-screen camera rendering the gimbal-following inset: it sits behind the
/// robot's aim (see `pip_offset`) and looks exactly where the first-person view
/// looks, so the inset turns with the turret.
#[derive(Component)]
pub struct PictureInPicture {
    frame: u64,
    was_visible: bool,
}

impl PictureInPicture {
    pub fn new() -> Self {
        Self {
            frame: 0,
            was_visible: false,
        }
    }

    /// Advances the inset clock and reports whether the extra pass runs this
    /// frame. Rendering every `divisor` frames halves the cost of a second view;
    /// a freshly shown inset renders immediately so it never appears stale.
    pub fn tick(&mut self, visible: bool, divisor: u32) -> bool {
        let divisor = divisor.max(1) as u64;
        let render =
            visible && (divisor == 1 || self.frame.is_multiple_of(divisor) || !self.was_visible);

        self.frame = self.frame.wrapping_add(1);
        self.was_visible = visible;
        render
    }
}

/// UI node showing the `PictureInPicture` camera's render target.
#[derive(Component)]
pub struct PictureInPictureNode;

/// Runtime on/off for the inset (`F6` / gamepad `B`). `camera.pip_enabled` in
/// `config.toml` stays the master switch.
#[derive(Resource, Debug)]
pub struct PictureInPictureState {
    pub enabled: bool,
}

impl Default for PictureInPictureState {
    fn default() -> Self {
        Self { enabled: true }
    }
}

impl PictureInPictureState {
    pub fn toggle(&mut self) {
        self.enabled = !self.enabled;
    }
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

    #[test]
    fn inset_tick_renders_every_divisor_frames_and_immediately_when_shown() {
        let mut inset = PictureInPicture::new();

        // Hidden: the extra pass never runs.
        assert!(!inset.tick(false, 2));
        assert!(!inset.tick(false, 2));

        // Freshly shown: renders at once, whatever the frame phase is.
        assert!(
            inset.tick(true, 2),
            "first visible frame renders immediately"
        );

        // Afterwards exactly one of any two consecutive frames renders.
        let first = inset.tick(true, 2);
        let second = inset.tick(true, 2);
        assert!(first ^ second, "divisor 2 renders one frame out of two");

        // Hiding stops the pass, showing again renders immediately.
        assert!(!inset.tick(false, 2));
        assert!(inset.tick(true, 2));

        // A divisor of one renders every frame; zero means "every frame".
        assert!(inset.tick(true, 1));
        assert!(inset.tick(true, 0));
    }

    #[test]
    fn inset_starts_enabled_and_toggles() {
        let mut inset = PictureInPictureState::default();
        assert!(inset.enabled);

        inset.toggle();
        assert!(!inset.enabled);

        inset.toggle();
        assert!(inset.enabled);
    }
}
