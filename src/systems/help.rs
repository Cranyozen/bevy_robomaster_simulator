//! Bottom-left control-hint HUD.
//!
//! The tables below are declared once and filtered by the live runtime state:
//! input device (keyboard / gamepad), camera mode (follow / free) and whether the
//! external solver drives the gimbal. Keys are sampled in `systems/controller.rs`,
//! `systems/input.rs` and `systems/camera.rs`; keep the tables in sync when a
//! binding changes. Strings stay ASCII because the bundled default font ships
//! Latin glyphs only.

use bevy::prelude::*;

use crate::components::{CameraFocus, CameraMode, FollowingType};
use crate::statistic::ProjectileStatistics;
use crate::systems::{ControllerState, GameplaySystems, InputSource};

const HEADER_FONT_SIZE: f32 = 14.0;
const ACTION_FONT_SIZE: f32 = 11.5;
const KEY_FONT_SIZE: f32 = 10.5;
const SECTION_FONT_SIZE: f32 = 10.5;

const PANEL_BACKGROUND: Color = Color::srgba(0.03, 0.04, 0.06, 0.82);
const PANEL_BORDER: Color = Color::srgba(1.0, 1.0, 1.0, 0.10);
const KEY_BACKGROUND: Color = Color::srgba(1.0, 1.0, 1.0, 0.10);
const KEY_BORDER: Color = Color::srgba(1.0, 1.0, 1.0, 0.16);
const KEY_TEXT_COLOR: Color = Color::srgb(0.96, 0.98, 1.0);
const ACTION_TEXT_COLOR: Color = Color::srgb(0.70, 0.76, 0.84);
const SECTION_TEXT_COLOR: Color = Color::srgb(0.52, 0.58, 0.67);
const HEADER_TEXT_COLOR: Color = Color::srgb(0.92, 0.95, 1.0);
const ACCENT_TEXT_COLOR: Color = Color::srgb(1.00, 0.72, 0.30);

const ITEM_MIN_WIDTH: f32 = 152.0;
const COLUMN_MIN_WIDTH: f32 = 322.0;

/// Which runtime states a hint applies to. `Any` always matches.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SourceScope {
    Any,
    Keyboard,
    Gamepad,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ViewScope {
    Any,
    Follow,
    Free,
}

/// `Manual` means the local aim input still moves the gimbal; `AutoAim` means the
/// solver fire advice owns it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AimScope {
    Any,
    Manual,
    AutoAim,
}

#[derive(Clone, Copy, PartialEq, Eq)]
struct HintScope {
    source: SourceScope,
    view: ViewScope,
    aim: AimScope,
}

impl HintScope {
    const ANY: Self = Self {
        source: SourceScope::Any,
        view: ViewScope::Any,
        aim: AimScope::Any,
    };

    const fn keyboard(self) -> Self {
        Self {
            source: SourceScope::Keyboard,
            ..self
        }
    }

    const fn gamepad(self) -> Self {
        Self {
            source: SourceScope::Gamepad,
            ..self
        }
    }

    const fn follow(self) -> Self {
        Self {
            view: ViewScope::Follow,
            ..self
        }
    }

    const fn free(self) -> Self {
        Self {
            view: ViewScope::Free,
            ..self
        }
    }

    const fn manual(self) -> Self {
        Self {
            aim: AimScope::Manual,
            ..self
        }
    }

    const fn auto_aim(self) -> Self {
        Self {
            aim: AimScope::AutoAim,
            ..self
        }
    }
}

const KEYBOARD: HintScope = HintScope::ANY.keyboard();
const GAMEPAD: HintScope = HintScope::ANY.gamepad();
const KEYBOARD_FOLLOW: HintScope = HintScope::ANY.keyboard().follow();
const GAMEPAD_FOLLOW: HintScope = HintScope::ANY.gamepad().follow();
const KEYBOARD_MANUAL: HintScope = HintScope::ANY.keyboard().manual();
const GAMEPAD_MANUAL: HintScope = HintScope::ANY.gamepad().manual();
const KEYBOARD_FREE_MANUAL: HintScope = HintScope::ANY.keyboard().free().manual();
const FREE: HintScope = HintScope::ANY.free();
const AUTO_AIM: HintScope = HintScope::ANY.auto_aim();

struct Hint {
    keys: &'static [&'static str],
    action: &'static str,
    scope: HintScope,
}

struct HintRow {
    items: &'static [Hint],
}

struct HintSection {
    title: &'static str,
    rows: &'static [HintRow],
}

const fn hint(keys: &'static [&'static str], action: &'static str, scope: HintScope) -> Hint {
    Hint {
        keys,
        action,
        scope,
    }
}

const fn row(items: &'static [Hint]) -> HintRow {
    HintRow { items }
}

const fn section(title: &'static str, rows: &'static [HintRow]) -> HintSection {
    HintSection { title, rows }
}

const LEFT_SECTIONS: &[HintSection] = &[
    section(
        "CONTROLLED ROBOT",
        &[
            row(&[
                hint(&["W", "A", "S", "D"], "Move", KEYBOARD_FOLLOW),
                hint(&["SHIFT"], "Boost", KEYBOARD_FOLLOW),
            ]),
            row(&[
                hint(&["SPACE"], "Fire 17 mm", KEYBOARD),
                hint(&["G"], "Dart", KEYBOARD),
            ]),
            row(&[
                hint(&["ARROWS"], "Aim gimbal", KEYBOARD_MANUAL),
                hint(&["Q"], "Spin chassis", KEYBOARD),
            ]),
            row(&[
                hint(&["LS"], "Move", GAMEPAD_FOLLOW),
                hint(&["L3"], "Boost", GAMEPAD_FOLLOW),
            ]),
            row(&[
                hint(&["RB"], "Fire 17 mm", GAMEPAD),
                hint(&["X"], "Dart", GAMEPAD),
            ]),
            row(&[
                hint(&["RS"], "Aim gimbal", GAMEPAD_MANUAL),
                hint(&["LB"], "Spin chassis", GAMEPAD),
            ]),
            row(&[
                hint(&["LT"], "Precise aim", GAMEPAD_MANUAL),
                hint(&["RT"], "Hold: auto-aim", GAMEPAD),
            ]),
            row(&[hint(
                &[],
                "Auto-aim: gimbal follows solver fire advice",
                AUTO_AIM,
            )]),
        ],
    ),
    section(
        "FREE CAMERA",
        &[
            row(&[
                hint(&["W", "A", "S", "D"], "Move camera", FREE),
                hint(&["N", "J"], "Up / down", FREE),
            ]),
            row(&[
                hint(&["MOUSE"], "Look around", FREE),
                hint(&["ARROWS"], "Aim gimbal", KEYBOARD_FREE_MANUAL),
            ]),
        ],
    ),
];

const RIGHT_SECTIONS: &[HintSection] = &[
    section(
        "SLAPPER ROBOT",
        &[
            row(&[
                hint(&["I", "J", "K", "L"], "Move", KEYBOARD),
                hint(&["SHIFT R"], "Boost", KEYBOARD),
            ]),
            row(&[
                hint(&["C", "B"], "Aim yaw", KEYBOARD),
                hint(&["F", "V"], "Aim pitch", KEYBOARD),
            ]),
            row(&[
                hint(&["[", "]"], "Chassis roll", KEYBOARD),
                hint(&[";", "'"], "Chassis pitch", KEYBOARD),
            ]),
            row(&[
                hint(&["U"], "Spin chassis", KEYBOARD),
                hint(&["TAB"], "Select slapper", KEYBOARD),
            ]),
            row(&[
                hint(&["DPAD"], "Move", GAMEPAD),
                hint(&["Y"], "Spin chassis", GAMEPAD),
            ]),
            row(&[
                hint(&["R3 + RS"], "Chassis roll / pitch", GAMEPAD),
                hint(&["START"], "Select slapper", GAMEPAD),
            ]),
        ],
    ),
    section(
        "SYSTEM",
        &[
            row(&[
                hint(&["F1"], "Hide panel", HintScope::ANY),
                hint(&["F2"], "Screenshot", HintScope::ANY),
            ]),
            row(&[
                hint(&["F3"], "Camera mode", HintScope::ANY),
                hint(&["F5"], "Auto-aim toggle", HintScope::ANY),
            ]),
            row(&[
                hint(&["O"], "Follow own / slapper", KEYBOARD),
                hint(&["A"], "Follow own / slapper", GAMEPAD),
            ]),
            row(&[
                hint(&["F6"], "Inset on / off", KEYBOARD),
                hint(&["B"], "Inset on / off", GAMEPAD),
            ]),
        ],
    ),
];

#[derive(Component, Clone, Copy)]
enum HintNode {
    /// Panel root: hidden while the panel is collapsed.
    Panel,
    /// Footer shown only while the panel is collapsed.
    Collapsed,
    /// One key/action group: hidden unless its scope matches the runtime state.
    Item(HintScope),
    /// Section container: hidden when none of its items is visible.
    Section(&'static [HintRow]),
}

#[derive(Component, Clone, Copy, PartialEq, Eq)]
enum HintChip {
    Source,
    Mode,
    View,
    Target,
    Gyro,
    SlapperGyro,
    Accuracy,
}

#[derive(Component, Clone, Copy)]
struct HintChipLabel(HintChip);

const CHIPS: [HintChip; 7] = [
    HintChip::Source,
    HintChip::Mode,
    HintChip::View,
    HintChip::Target,
    HintChip::Gyro,
    HintChip::SlapperGyro,
    HintChip::Accuracy,
];

#[derive(Resource)]
struct HelpPanel {
    visible: bool,
}

impl Default for HelpPanel {
    fn default() -> Self {
        Self { visible: true }
    }
}

/// Bottom-left control hints, toggleable with `F1`.
pub struct HelpPlugin;

impl Plugin for HelpPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<HelpPanel>()
            .add_systems(Startup, spawn_help_panel)
            .add_systems(
                Update,
                (toggle_help_panel, update_help_panel, apply_help_visibility)
                    .chain()
                    .in_set(GameplaySystems::GameLogic),
            );
    }
}

#[derive(Clone, Copy)]
struct HintState {
    source: SourceScope,
    view: ViewScope,
    aim: AimScope,
}

impl HintState {
    fn new(controller: &ControllerState, camera: &CameraMode) -> Self {
        Self {
            source: match controller.help_source() {
                InputSource::Keyboard => SourceScope::Keyboard,
                InputSource::Xbox => SourceScope::Gamepad,
            },
            view: match camera.0 {
                FollowingType::Free => ViewScope::Free,
                FollowingType::Robot | FollowingType::ThirdPerson => ViewScope::Follow,
            },
            aim: if controller.auto_aim_active() {
                AimScope::AutoAim
            } else {
                AimScope::Manual
            },
        }
    }

    fn matches(self, scope: HintScope) -> bool {
        let source_matches =
            matches!(scope.source, SourceScope::Any) || scope.source == self.source;
        let view_matches = matches!(scope.view, ViewScope::Any) || scope.view == self.view;
        let aim_matches = matches!(scope.aim, AimScope::Any) || scope.aim == self.aim;
        source_matches && view_matches && aim_matches
    }
}

fn visibility(visible: bool) -> Display {
    if visible {
        Display::Flex
    } else {
        Display::None
    }
}

fn spawn_help_panel(mut commands: Commands) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(12.0),
                bottom: Val::Px(12.0),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(9.0),
                padding: UiRect::all(Val::Px(13.0)),
                border: UiRect::all(Val::Px(1.0)),
                border_radius: BorderRadius::all(Val::Px(10.0)),
                ..default()
            },
            BackgroundColor(PANEL_BACKGROUND),
            BorderColor::all(PANEL_BORDER),
            HintNode::Panel,
        ))
        .with_children(|panel| {
            panel
                .spawn(Node {
                    // Full width comes from the panel's stretch alignment; a
                    // percentage here would resolve against an auto-sized parent.
                    justify_content: JustifyContent::SpaceBetween,
                    align_items: AlignItems::Center,
                    column_gap: Val::Px(18.0),
                    ..default()
                })
                .with_children(|header| {
                    header.spawn((
                        Text::new("CONTROLS"),
                        TextFont::from_font_size(HEADER_FONT_SIZE),
                        TextColor(HEADER_TEXT_COLOR),
                    ));
                    header.spawn((
                        Text::new("F1: HIDE"),
                        TextFont::from_font_size(SECTION_FONT_SIZE),
                        TextColor(SECTION_TEXT_COLOR),
                    ));
                });

            panel
                .spawn(Node {
                    flex_wrap: FlexWrap::Wrap,
                    column_gap: Val::Px(6.0),
                    row_gap: Val::Px(4.0),
                    max_width: Val::Px(COLUMN_MIN_WIDTH * 2.0 + 20.0),
                    ..default()
                })
                .with_children(|chips| {
                    for chip in CHIPS {
                        spawn_chip(chips, chip);
                    }
                });

            panel
                .spawn(Node {
                    flex_direction: FlexDirection::Row,
                    align_items: AlignItems::Start,
                    column_gap: Val::Px(18.0),
                    ..default()
                })
                .with_children(|columns| {
                    for sections in [LEFT_SECTIONS, RIGHT_SECTIONS] {
                        columns
                            .spawn(Node {
                                flex_direction: FlexDirection::Column,
                                row_gap: Val::Px(9.0),
                                min_width: Val::Px(COLUMN_MIN_WIDTH),
                                ..default()
                            })
                            .with_children(|column| {
                                for section in sections {
                                    spawn_section(column, section);
                                }
                            });
                    }
                });
        });

    commands.spawn((
        Text::new("[F1] CONTROLS"),
        TextFont::from_font_size(KEY_FONT_SIZE),
        TextColor(SECTION_TEXT_COLOR),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(12.0),
            bottom: Val::Px(12.0),
            ..default()
        },
        HintNode::Collapsed,
    ));
}

fn spawn_chip(parent: &mut ChildSpawnerCommands<'_>, chip: HintChip) {
    parent
        .spawn((
            Node {
                padding: UiRect::px(8.0, 8.0, 3.0, 3.0),
                border: UiRect::all(Val::Px(1.0)),
                border_radius: BorderRadius::all(Val::Px(9.0)),
                ..default()
            },
            BackgroundColor(PANEL_BACKGROUND),
            BorderColor::all(PANEL_BORDER),
            chip,
        ))
        .with_children(|parent| {
            parent.spawn((
                Text::new(""),
                TextFont::from_font_size(KEY_FONT_SIZE),
                TextColor(ACCENT_TEXT_COLOR),
                HintChipLabel(chip),
            ));
        });
}

fn spawn_section(parent: &mut ChildSpawnerCommands<'_>, section: &'static HintSection) {
    parent
        .spawn((
            Node {
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(3.0),
                ..default()
            },
            HintNode::Section(section.rows),
        ))
        .with_children(|parent| {
            parent.spawn((
                Text::new(section.title),
                TextFont::from_font_size(SECTION_FONT_SIZE),
                TextColor(SECTION_TEXT_COLOR),
            ));
            for row in section.rows {
                spawn_row(parent, row);
            }
        });
}

fn spawn_row(parent: &mut ChildSpawnerCommands<'_>, row: &'static HintRow) {
    parent
        .spawn(Node {
            flex_direction: FlexDirection::Row,
            align_items: AlignItems::Center,
            column_gap: Val::Px(14.0),
            ..default()
        })
        .with_children(|parent| {
            for item in row.items {
                parent
                    .spawn((
                        Node {
                            flex_direction: FlexDirection::Row,
                            align_items: AlignItems::Center,
                            column_gap: Val::Px(4.0),
                            min_width: Val::Px(ITEM_MIN_WIDTH),
                            ..default()
                        },
                        HintNode::Item(item.scope),
                    ))
                    .with_children(|parent| {
                        for key in item.keys {
                            spawn_keycap(parent, key);
                        }
                        let color = if item.keys.is_empty() {
                            ACCENT_TEXT_COLOR
                        } else {
                            ACTION_TEXT_COLOR
                        };
                        parent.spawn((
                            Text::new(item.action),
                            TextFont::from_font_size(ACTION_FONT_SIZE),
                            TextColor(color),
                        ));
                    });
            }
        });
}

fn spawn_keycap(parent: &mut ChildSpawnerCommands<'_>, key: &str) {
    parent
        .spawn((
            Node {
                padding: UiRect::px(6.0, 6.0, 2.0, 2.0),
                border: UiRect::all(Val::Px(1.0)),
                border_radius: BorderRadius::all(Val::Px(4.0)),
                ..default()
            },
            BackgroundColor(KEY_BACKGROUND),
            BorderColor::all(KEY_BORDER),
        ))
        .with_children(|parent| {
            parent.spawn((
                Text::new(key),
                TextFont::from_font_size(KEY_FONT_SIZE),
                TextColor(KEY_TEXT_COLOR),
            ));
        });
}

fn toggle_help_panel(keyboard: Res<ButtonInput<KeyCode>>, mut panel: ResMut<HelpPanel>) {
    if keyboard.just_pressed(KeyCode::F1) {
        panel.visible = !panel.visible;
    }
}

fn apply_help_visibility(
    controller: Res<ControllerState>,
    camera: Res<CameraMode>,
    panel: Res<HelpPanel>,
    mut nodes: Query<(&HintNode, &mut Node)>,
) {
    let state = HintState::new(&controller, &camera);
    for (node, mut layout) in &mut nodes {
        let next = match node {
            HintNode::Panel => visibility(panel.visible),
            HintNode::Collapsed => visibility(!panel.visible),
            HintNode::Item(scope) => visibility(state.matches(*scope)),
            HintNode::Section(rows) => visibility(
                rows.iter()
                    .any(|row| row.items.iter().any(|item| state.matches(item.scope))),
            ),
        };
        // Only touch `Node` on an actual change: writing it re-runs UI layout.
        if layout.display != next {
            layout.display = next;
        }
    }
}

fn update_help_panel(
    controller: Res<ControllerState>,
    camera: Res<CameraMode>,
    focus: Res<CameraFocus>,
    statistics: Res<ProjectileStatistics>,
    mut chips: Query<(&HintChip, &mut BackgroundColor, &mut BorderColor)>,
    mut labels: Query<(&HintChipLabel, &mut Text, &mut TextColor)>,
) {
    for (chip, mut background, mut border) in &mut chips {
        let (_, accent) = chip_content(*chip, &controller, &camera, &focus, &statistics);
        let background_color = accent.with_alpha(0.14);
        if background.0 != background_color {
            background.0 = background_color;
        }
        let border_color = accent.with_alpha(0.42);
        if border.top != border_color {
            *border = BorderColor::all(border_color);
        }
    }

    for (label, mut text, mut color) in &mut labels {
        let (content, accent) = chip_content(label.0, &controller, &camera, &focus, &statistics);
        if text.0 != content {
            text.0 = content;
        }
        if color.0 != accent {
            color.0 = accent;
        }
    }
}

fn chip_content(
    chip: HintChip,
    controller: &ControllerState,
    camera: &CameraMode,
    focus: &CameraFocus,
    statistics: &ProjectileStatistics,
) -> (String, Color) {
    const NEUTRAL: Color = Color::srgb(0.60, 0.66, 0.74);
    const ACTIVE: Color = Color::srgb(0.44, 0.86, 0.56);
    const WARNING: Color = Color::srgb(1.00, 0.68, 0.28);
    const INFO: Color = Color::srgb(0.45, 0.68, 1.00);
    const SKY: Color = Color::srgb(0.55, 0.82, 0.96);

    match chip {
        HintChip::Source => (controller.help_source().label().to_ascii_uppercase(), INFO),
        HintChip::Mode => {
            if controller.auto_aim_active() {
                ("AUTO-AIM".to_string(), WARNING)
            } else {
                ("MANUAL".to_string(), ACTIVE)
            }
        }
        HintChip::View => (
            match camera.0 {
                FollowingType::Free => "FREE CAMERA",
                FollowingType::Robot => "FIRST-PERSON",
                FollowingType::ThirdPerson => "THIRD-PERSON",
            }
            .to_string(),
            SKY,
        ),
        HintChip::Target => (
            format!("TARGET {}", focus.label().to_ascii_uppercase()),
            match focus {
                CameraFocus::Own => INFO,
                CameraFocus::Slapper => WARNING,
            },
        ),
        HintChip::Gyro => spin_label("GYRO", controller.controlled_chassis_spin()),
        HintChip::SlapperGyro => spin_label("SLAPPER GYRO", controller.remote_chassis_spin()),
        HintChip::Accuracy => (
            format!(
                "HIT {:.0}% ({}/{})",
                statistics.accurate_pct() * 100.0,
                statistics.accurate_count,
                statistics.launch_count
            ),
            if statistics.launch_count == 0 {
                NEUTRAL
            } else {
                ACTIVE
            },
        ),
    }
}

fn spin_label(name: &str, spinning: bool) -> (String, Color) {
    const ACTIVE: Color = Color::srgb(0.44, 0.86, 0.56);
    const IDLE: Color = Color::srgb(0.58, 0.63, 0.71);

    if spinning {
        (format!("{name} ON"), ACTIVE)
    } else {
        (format!("{name} OFF"), IDLE)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SOURCES: [SourceScope; 2] = [SourceScope::Keyboard, SourceScope::Gamepad];
    const VIEWS: [ViewScope; 2] = [ViewScope::Follow, ViewScope::Free];
    const AIMS: [AimScope; 2] = [AimScope::Manual, AimScope::AutoAim];

    fn state(source: SourceScope, view: ViewScope, aim: AimScope) -> HintState {
        HintState { source, view, aim }
    }

    fn visible_items(state: HintState) -> Vec<(&'static str, &'static str)> {
        let mut visible = Vec::new();
        for sections in [LEFT_SECTIONS, RIGHT_SECTIONS] {
            for section in sections {
                for row in section.rows {
                    for item in row.items {
                        if state.matches(item.scope) {
                            visible.push((section.title, item.action));
                        }
                    }
                }
            }
        }
        visible
    }

    fn section_has_item(title: &str, state: HintState) -> bool {
        [LEFT_SECTIONS, RIGHT_SECTIONS]
            .iter()
            .flat_map(|sections| sections.iter())
            .filter(|section| section.title == title)
            .any(|section| {
                section
                    .rows
                    .iter()
                    .any(|row| row.items.iter().any(|item| state.matches(item.scope)))
            })
    }

    /// Sections whose hints apply regardless of camera mode or device.
    const ALWAYS_RELEVANT: [&str; 3] = ["CONTROLLED ROBOT", "SLAPPER ROBOT", "SYSTEM"];

    #[test]
    fn always_relevant_sections_keep_a_hint_for_every_runtime_state() {
        for source in SOURCES {
            for view in VIEWS {
                for aim in AIMS {
                    let state = state(source, view, aim);
                    for title in ALWAYS_RELEVANT {
                        assert!(
                            section_has_item(title, state),
                            "section {title:?} has no hint for {source:?}/{view:?}/{aim:?}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn free_camera_section_only_shows_in_free_camera_mode() {
        for source in SOURCES {
            for aim in AIMS {
                assert!(section_has_item(
                    "FREE CAMERA",
                    state(source, ViewScope::Free, aim)
                ));
                assert!(!section_has_item(
                    "FREE CAMERA",
                    state(source, ViewScope::Follow, aim)
                ));
            }
        }
    }

    #[test]
    fn free_camera_swaps_robot_drive_hints_for_camera_hints() {
        let follow = state(SourceScope::Keyboard, ViewScope::Follow, AimScope::Manual);
        let free = state(SourceScope::Keyboard, ViewScope::Free, AimScope::Manual);

        assert!(visible_items(follow).contains(&("CONTROLLED ROBOT", "Move")));
        assert!(!visible_items(follow).contains(&("FREE CAMERA", "Move camera")));

        assert!(!visible_items(free).contains(&("CONTROLLED ROBOT", "Move")));
        assert!(visible_items(free).contains(&("FREE CAMERA", "Move camera")));
        assert!(visible_items(free).contains(&("FREE CAMERA", "Look around")));
        assert!(visible_items(free).contains(&("FREE CAMERA", "Up / down")));
    }

    #[test]
    fn auto_aim_hands_the_gimbal_to_the_solver() {
        let manual = state(SourceScope::Keyboard, ViewScope::Follow, AimScope::Manual);
        let auto_aim = state(SourceScope::Keyboard, ViewScope::Follow, AimScope::AutoAim);

        assert!(visible_items(manual).contains(&("CONTROLLED ROBOT", "Aim gimbal")));
        assert!(!visible_items(auto_aim).contains(&("CONTROLLED ROBOT", "Aim gimbal")));
        assert!(
            !visible_items(manual)
                .iter()
                .any(|(_, action)| action.starts_with("Auto-aim:"))
        );
        assert!(
            visible_items(auto_aim)
                .iter()
                .any(|(_, action)| action.starts_with("Auto-aim:"))
        );
    }

    #[test]
    fn gamepad_state_drops_keyboard_only_hints() {
        let keyboard = state(SourceScope::Keyboard, ViewScope::Follow, AimScope::Manual);
        let gamepad = state(SourceScope::Gamepad, ViewScope::Follow, AimScope::Manual);

        assert!(visible_items(keyboard).contains(&("SLAPPER ROBOT", "Aim yaw")));
        assert!(!visible_items(gamepad).contains(&("SLAPPER ROBOT", "Aim yaw")));
        assert!(visible_items(gamepad).contains(&("SLAPPER ROBOT", "Chassis roll / pitch")));
        assert!(!visible_items(keyboard).contains(&("SLAPPER ROBOT", "Chassis roll / pitch")));
    }

    #[test]
    fn status_chips_track_runtime_state() {
        let mut controller = ControllerState::default();
        let camera = CameraMode::default();
        let mut focus = CameraFocus::default();
        let mut statistics = ProjectileStatistics::default();

        let (label, _) = chip_content(HintChip::Source, &controller, &camera, &focus, &statistics);
        assert_eq!(label, "KEYBOARD");

        let (label, _) = chip_content(HintChip::Mode, &controller, &camera, &focus, &statistics);
        assert_eq!(label, "MANUAL");

        controller.controlled.auto_aim = true;
        let (label, _) = chip_content(HintChip::Mode, &controller, &camera, &focus, &statistics);
        assert_eq!(label, "AUTO-AIM");

        let (label, _) = chip_content(HintChip::Target, &controller, &camera, &focus, &statistics);
        assert_eq!(label, "TARGET OWN");

        focus.toggle();
        let (label, _) = chip_content(HintChip::Target, &controller, &camera, &focus, &statistics);
        assert_eq!(label, "TARGET SLAPPER");

        statistics.launch_count = 2;
        statistics.accurate_count = 1;
        let (label, _) = chip_content(
            HintChip::Accuracy,
            &controller,
            &camera,
            &focus,
            &statistics,
        );
        assert_eq!(label, "HIT 50% (1/2)");

        assert_eq!(spin_label("GYRO", true).0, "GYRO ON");
        assert_eq!(spin_label("GYRO", false).0, "GYRO OFF");
    }

    fn keys_for(state: HintState, action: &str) -> Vec<&'static str> {
        let mut keys = Vec::new();
        for sections in [LEFT_SECTIONS, RIGHT_SECTIONS] {
            for section in sections {
                for row in section.rows {
                    for item in row.items {
                        if item.action == action && state.matches(item.scope) {
                            keys.extend(item.keys.iter().copied());
                        }
                    }
                }
            }
        }
        keys
    }

    #[test]
    fn device_specific_hints_show_the_matching_keys() {
        let keyboard = state(SourceScope::Keyboard, ViewScope::Follow, AimScope::Manual);
        let gamepad = state(SourceScope::Gamepad, ViewScope::Follow, AimScope::Manual);

        assert_eq!(keys_for(keyboard, "Follow own / slapper"), vec!["O"]);
        assert_eq!(keys_for(gamepad, "Follow own / slapper"), vec!["A"]);
        assert_eq!(keys_for(keyboard, "Inset on / off"), vec!["F6"]);
        assert_eq!(keys_for(gamepad, "Inset on / off"), vec!["B"]);
    }
}
