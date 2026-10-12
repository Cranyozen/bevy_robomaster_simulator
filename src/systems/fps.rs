//! Top-right FPS / frame-time readout.
//!
//! The rate comes from Bevy's `FrameTimeDiagnosticsPlugin` `fps` diagnostic — the
//! same measurement the console logger prints — rather than from `Time::delta`, so
//! it stays correct when `performance.max_fps` paces the loop. The frame time next
//! to it is derived as `1000 / fps`: averaging the rate and the frame duration
//! separately gives two numbers that disagree under jitter.
//!
//! The average shown on screen is computed here, not by `Diagnostic::smoothed`:
//! Bevy's built-in EMA has a 16.5 ms smoothing factor, i.e. it only averages when
//! samples arrive closer together than a 60 fps frame. This simulator often runs
//! at 10-40 fps, where that EMA degenerates into "latest sample" and the readout
//! would jump on every refresh. The moving average below is defined over the
//! refresh interval instead, so it settles at any frame rate.
//!
//! Visibility is `config.toml` → `[debug] fps` **and** the runtime `F4` toggle:
//! the config key is hot-reloaded and acts as the feature switch, `F4` flips the
//! overlay within that. The label is only rewritten a few times per second —
//! touching `Text` re-runs UI layout.

use bevy::diagnostic::{DiagnosticPath, DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;

use crate::config::SimulationConfig;
use crate::systems::GameplaySystems;

/// Seconds between label rewrites. Reading the diagnostics faster would only
/// re-layout text for a number that cannot change meaningfully in between.
const REFRESH_INTERVAL: f32 = 0.25;
/// Scaling of the on-screen moving average: each refresh takes
/// `elapsed / SMOOTHING_SECONDS` of the new sample. At the refresh rate above that
/// is half of it per update (a 0.36 s time constant): stable enough to read, quick
/// enough to see a `max_fps` or `[render]` change land.
const SMOOTHING_SECONDS: f32 = 0.5;
const FONT_SIZE: f32 = 13.0;
const TEXT_COLOR: Color = Color::srgb(0.92, 0.95, 1.0);
const PANEL_BACKGROUND: Color = Color::srgba(0.03, 0.04, 0.06, 0.82);
const PANEL_BORDER: Color = Color::srgba(1.0, 1.0, 1.0, 0.10);
const PLACEHOLDER: &str = "FPS --";

/// Runtime state of the overlay: the `F4` toggle, the accumulator that paces
/// label updates against wall-clock time, and the average currently on screen.
#[derive(Resource)]
pub struct FpsOverlay {
    visible: bool,
    elapsed: f32,
    fps: Option<f64>,
}

impl Default for FpsOverlay {
    fn default() -> Self {
        Self {
            visible: true,
            // Start due, so the first frame writes the label (with the placeholder
            // until the diagnostics plugin has taken its first measurement).
            elapsed: REFRESH_INTERVAL,
            fps: None,
        }
    }
}

/// Root node of the overlay; hidden (not despawned) when the readout is off.
#[derive(Component)]
struct FpsOverlayRoot;

#[derive(Component)]
struct FpsLabel;

/// Top-right FPS / frame-time readout, toggleable with `F4`.
pub struct FpsDisplayPlugin;

impl Plugin for FpsDisplayPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FpsOverlay>()
            .add_systems(Startup, spawn_fps_overlay)
            .add_systems(
                Update,
                (toggle_fps_overlay, update_fps_overlay)
                    .chain()
                    .in_set(GameplaySystems::GameLogic),
            );
    }
}

fn spawn_fps_overlay(mut commands: Commands) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(12.0),
                right: Val::Px(12.0),
                padding: UiRect::px(9.0, 9.0, 4.0, 4.0),
                border: UiRect::all(Val::Px(1.0)),
                border_radius: BorderRadius::all(Val::Px(9.0)),
                ..default()
            },
            BackgroundColor(PANEL_BACKGROUND),
            BorderColor::all(PANEL_BORDER),
            FpsOverlayRoot,
        ))
        .with_children(|parent| {
            parent.spawn((
                Text::new(PLACEHOLDER),
                TextFont::from_font_size(FONT_SIZE),
                TextColor(TEXT_COLOR),
                FpsLabel,
            ));
        });
}

/// `F4` flips the overlay, but only while `config.toml` allows the feature at
/// all: with `[debug] fps = false` the key is ignored rather than latching a
/// state that would surprise the user once the gate is switched back on.
fn toggle_fps_overlay(
    keyboard: Res<ButtonInput<KeyCode>>,
    config: Res<SimulationConfig>,
    mut overlay: ResMut<FpsOverlay>,
) {
    if config.debug.fps && keyboard.just_pressed(KeyCode::F4) {
        overlay.visible = !overlay.visible;
    }
}

fn update_fps_overlay(
    time: Res<Time<Real>>,
    config: Res<SimulationConfig>,
    diagnostics: Option<Res<DiagnosticsStore>>,
    mut overlay: ResMut<FpsOverlay>,
    mut roots: Query<&mut Node, With<FpsOverlayRoot>>,
    mut labels: Query<&mut Text, With<FpsLabel>>,
) {
    let display = display_for(overlay_visible(config.debug.fps, overlay.visible));
    for mut layout in &mut roots {
        // Only touch `Node` on an actual change: writing it re-runs UI layout.
        if layout.display != display {
            layout.display = display;
        }
    }

    overlay.elapsed += time.delta_secs();
    if display == Display::None {
        // Drop the average while hidden and keep the refresh armed, so the first
        // visible frame starts from a current sample instead of blending in the
        // rate from before the hide.
        overlay.fps = None;
        overlay.elapsed = REFRESH_INTERVAL;
        return;
    }
    if overlay.elapsed < REFRESH_INTERVAL {
        return;
    }

    // The measurement window actually covered, which is what the average must be
    // weighted by — a slow frame may overshoot `REFRESH_INTERVAL` by a lot.
    let alpha = (overlay.elapsed / SMOOTHING_SECONDS).clamp(0.0, 1.0) as f64;
    overlay.elapsed = 0.0;

    overlay.fps = smooth(
        overlay.fps,
        read_diagnostic(diagnostics.as_deref(), &FrameTimeDiagnosticsPlugin::FPS),
        alpha,
    );

    let content = format_fps(overlay.fps);
    for mut text in &mut labels {
        if text.0 != content {
            text.0 = content.clone();
        }
    }
}

/// The overlay renders only while the config switch allows it *and* `F4` left it on.
fn overlay_visible(config_fps: bool, toggled_on: bool) -> bool {
    config_fps && toggled_on
}

fn display_for(visible: bool) -> Display {
    if visible {
        Display::Flex
    } else {
        Display::None
    }
}

/// One exponential-moving-average step. `alpha` is how much of the new sample to
/// take, derived from the time since the previous refresh; a missing sample (no
/// measurement yet) leaves the average untouched.
fn smooth(current: Option<f64>, sample: Option<f64>, alpha: f64) -> Option<f64> {
    match (current, sample) {
        (Some(current), Some(sample)) => Some(current + alpha.clamp(0.0, 1.0) * (sample - current)),
        (None, sample) => sample,
        (current, None) => current,
    }
}

/// Latest measurement of a diagnostic, or `None` before the first one and while
/// the store is absent (the plugin does not require the diagnostics plugins).
fn read_diagnostic(diagnostics: Option<&DiagnosticsStore>, path: &DiagnosticPath) -> Option<f64> {
    diagnostics?.get(path)?.value()
}

/// `"120.3 FPS  8.31 ms"`, or the placeholder until the first measurement lands.
/// The frame time is `1000 / fps`, so the two numbers on screen always agree.
fn format_fps(fps: Option<f64>) -> String {
    match fps.filter(|value| value.is_finite() && *value > 0.0) {
        Some(fps) => format!("{fps:.1} FPS  {:.2} ms", 1000.0 / fps),
        None => PLACEHOLDER.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::time::TimeUpdateStrategy;
    use std::time::Duration;

    #[test]
    fn placeholder_before_the_first_measurement() {
        for fps in [
            None,
            Some(f64::NAN),
            Some(f64::INFINITY),
            Some(0.0),
            Some(-1.0),
        ] {
            assert_eq!(format_fps(fps), "FPS --", "fps={fps:?}");
        }
    }

    #[test]
    fn shows_the_rate_and_the_frame_time_it_implies() {
        assert_eq!(format_fps(Some(120.34)), "120.3 FPS  8.31 ms");
        assert_eq!(format_fps(Some(59.94)), "59.9 FPS  16.68 ms");
    }

    #[test]
    fn the_two_numbers_are_reciprocal_by_construction() {
        // Averaging the rate and the frame duration separately would let these two
        // drift apart; formatting derives one from the other.
        for fps in [3.3333, 19.53, 60.0, 144.0] {
            let text = format_fps(Some(fps));
            let millis: f64 = text
                .split_whitespace()
                .nth(2)
                .and_then(|value| value.parse().ok())
                .expect("frame time in the label");
            assert!(
                (1000.0 / fps - millis).abs() < 0.006,
                "1000 / {fps} 应等于 {millis} ms，实际标签 {text:?}"
            );
        }
    }

    #[test]
    fn config_and_toggle_both_have_to_allow_the_overlay() {
        assert!(overlay_visible(true, true));
        assert!(
            !overlay_visible(false, true),
            "config.toml 关闭后 F4 不生效"
        );
        assert!(!overlay_visible(true, false), "F4 收起后保持隐藏");
        assert!(!overlay_visible(false, false));
    }

    #[test]
    fn hidden_overlay_is_not_laid_out() {
        assert_eq!(display_for(true), Display::Flex);
        assert_eq!(display_for(false), Display::None);
    }

    #[test]
    fn averaging_seeds_then_moves_towards_the_sample_and_holds_on_gaps() {
        let seeded = smooth(None, Some(10.0), 0.5);
        assert_eq!(seeded, Some(10.0), "第一帧直接采用采样值");

        let half = smooth(seeded, Some(20.0), 0.5).expect("still a value");
        assert!(
            (half - 15.0).abs() < 1e-9,
            "半个时间常数走一半，实际 {half}"
        );

        assert_eq!(
            smooth(Some(15.0), None, 0.5),
            Some(15.0),
            "缺采样时保持不变"
        );
        assert_eq!(smooth(None, None, 0.5), None);
    }

    /// Headless app wired the way `main.rs` wires it: the real diagnostics plugin,
    /// the real config resource (replaced at runtime the way `config_hot_reload`
    /// replaces it) and the real Startup/Update systems. No render world needed —
    /// the overlay is ordinary UI components.
    ///
    /// `ButtonInput` is initialised directly instead of adding `InputPlugin`, so
    /// the test decides exactly which frame sees `F4` as just-pressed.
    fn overlay_app(fps_enabled: bool) -> App {
        let mut config = SimulationConfig::default();
        config.debug.fps = fps_enabled;

        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .init_resource::<ButtonInput<KeyCode>>()
            .add_plugins(FrameTimeDiagnosticsPlugin::default())
            .add_plugins(FpsDisplayPlugin)
            .insert_resource(config)
            // Deterministic 300 ms frames: one real measurement per update.
            .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_millis(
                300,
            )));
        app
    }

    /// Runs exactly one frame with `F4` pressed once, then releases it. The real
    /// app's `InputPlugin` clears the edge-triggered state every frame; without it
    /// the helper has to do that itself.
    fn tap_f4(app: &mut App) {
        {
            let mut keys = app.world_mut().resource_mut::<ButtonInput<KeyCode>>();
            keys.reset_all();
            keys.press(KeyCode::F4);
        }
        app.update();
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .reset_all();
    }

    /// Presses `F4` and leaves it held for one frame, emulating `InputPlugin`: only
    /// the just-pressed flag is cleared, the key stays in `pressed`.
    fn hold_f4(app: &mut App) {
        {
            let mut keys = app.world_mut().resource_mut::<ButtonInput<KeyCode>>();
            keys.reset_all();
            keys.press(KeyCode::F4);
        }
        app.update();
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .clear();
    }

    fn label_text(app: &mut App) -> String {
        let mut query = app.world_mut().query_filtered::<&Text, With<FpsLabel>>();
        let mut texts: Vec<String> = query.iter(app.world()).map(|text| text.0.clone()).collect();
        assert_eq!(texts.len(), 1, "overlay has exactly one label");
        texts.remove(0)
    }

    fn root_display(app: &mut App) -> Display {
        let mut query = app
            .world_mut()
            .query_filtered::<&Node, With<FpsOverlayRoot>>();
        let mut displays: Vec<Display> = query.iter(app.world()).map(|node| node.display).collect();
        assert_eq!(displays.len(), 1, "overlay has exactly one root");
        displays.remove(0)
    }

    fn parse_fps(text: &str) -> f64 {
        text.split_whitespace()
            .next()
            .and_then(|value| value.parse().ok())
            .unwrap_or_else(|| panic!("expected an FPS number, got {text:?}"))
    }

    /// Switches the fake frame duration, i.e. the "machine got faster" step.
    fn set_frame_time(app: &mut App, millis: u64) {
        *app.world_mut().resource_mut::<TimeUpdateStrategy>() =
            TimeUpdateStrategy::ManualDuration(Duration::from_millis(millis));
    }

    #[test]
    fn overlay_shows_the_engine_frame_time_diagnostics() {
        let mut app = overlay_app(true);
        for _ in 0..3 {
            app.update();
        }

        assert_eq!(root_display(&mut app), Display::Flex);
        let text = label_text(&mut app);
        assert_ne!(text, PLACEHOLDER, "占位符应被真实读数替换");
        assert!(
            (3.2..3.5).contains(&parse_fps(&text)),
            "300 ms 帧 → 约 3.33 FPS，实际 {text}"
        );
        assert!(text.ends_with("300.00 ms"), "实际 {text}");
    }

    #[test]
    fn a_step_change_is_averaged_in_rather_than_jumping() {
        // 300 ms frames (~3.33 fps) until the average has settled on that rate.
        let mut app = overlay_app(true);
        for _ in 0..6 {
            app.update();
        }
        let slow = parse_fps(&label_text(&mut app));
        assert!((3.2..3.5).contains(&slow), "实际 {slow}");

        // The loop suddenly gets fast (~33.3 fps). The refresh is due only after
        // 0.25 s has accumulated, so eight 30 ms frames (0.24 s) must not rewrite
        // anything yet.
        set_frame_time(&mut app, 30);
        let mut previous = label_text(&mut app);
        for _ in 0..8 {
            app.update();
            let rewritten = label_text(&mut app);
            assert_eq!(rewritten, previous, "0.24 s 未到刷新间隔，不该重写文本");
            previous = rewritten;
        }

        // The next frame crosses 0.25 s and must blend in roughly half the gap
        // instead of jumping to the raw sample: alpha ≈ 0.27 / 0.5.
        app.update();
        let blended = parse_fps(&label_text(&mut app));
        assert!(
            (13.0..26.0).contains(&blended),
            "第一次刷新应落在 {slow} 与 33.3 之间的中间值，实际 {blended}"
        );

        // …and it keeps converging instead of sticking at either end.
        for _ in 0..40 {
            app.update();
        }
        let settled = parse_fps(&label_text(&mut app));
        assert!(
            (30.0..34.0).contains(&settled),
            "应收敛到约 33.3 FPS，实际 {settled}"
        );
        assert!(settled > blended);
    }

    #[test]
    fn holding_f4_does_not_flip_the_overlay_every_frame() {
        let mut app = overlay_app(true);
        app.update();
        assert_eq!(root_display(&mut app), Display::Flex);

        // Hold F4 down: the real `InputPlugin` clears the edge state every frame
        // while the key stays pressed.
        hold_f4(&mut app);
        assert_eq!(root_display(&mut app), Display::None, "按下时收起读数");

        app.update();
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .clear();
        assert_eq!(
            root_display(&mut app),
            Display::None,
            "按住 F4 只应切换一次"
        );

        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .reset_all();
    }

    #[test]
    fn revealing_the_overlay_starts_from_the_current_rate() {
        let mut app = overlay_app(true);
        // Start on a fast loop, so only ~0.01 s is banked towards the next refresh
        // when the overlay is hidden below.
        set_frame_time(&mut app, 10); // ~100 fps
        for _ in 0..30 {
            app.update();
        }
        let before = parse_fps(&label_text(&mut app));
        assert!((95.0..105.0).contains(&before), "实际 {before}");

        // Hide it, then let the loop get faster while the overlay is off. Every
        // hidden frame is short, so nothing accumulates towards a refresh.
        tap_f4(&mut app);
        assert_eq!(root_display(&mut app), Display::None);
        set_frame_time(&mut app, 6); // ~166.7 fps
        for _ in 0..12 {
            app.update();
        }

        // Revealing must refresh on the spot and seed from the current sample, not
        // keep the pre-hide text.
        tap_f4(&mut app);
        assert_eq!(root_display(&mut app), Display::Flex);
        let shown = parse_fps(&label_text(&mut app));
        assert!(
            (150.0..180.0).contains(&shown),
            "重新显示应直接给出当前帧率，实际 {shown}"
        );
    }

    #[test]
    fn the_displayed_pair_stays_reciprocal_under_jitter() {
        let mut app = overlay_app(true);
        // Frame times incommensurate with the 0.25 s refresh, so both the sampled
        // rate and the displayed average keep moving.
        let mut seed: u64 = 0x2545_F491_4F6C_DD1D;
        let mut checked = 0;
        for _ in 0..600 {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            set_frame_time(&mut app, 5 + (seed >> 33) % 56); // 5..=60 ms
            app.update();

            let text = label_text(&mut app);
            if text == PLACEHOLDER {
                continue;
            }
            let mut fields = text.split_whitespace();
            let fps: f64 = fields
                .next()
                .and_then(|value| value.parse().ok())
                .unwrap_or_else(|| panic!("expected an FPS number, got {text:?}"));
            let millis: f64 = fields
                .nth(1)
                .and_then(|value| value.parse().ok())
                .unwrap_or_else(|| panic!("expected a frame time, got {text:?}"));
            assert!(
                (1000.0 / fps - millis).abs() <= 0.5,
                "帧率与单帧耗时必须互为倒数，实际标签 {text:?}"
            );
            checked += 1;
        }
        assert!(checked > 20, "检查到的刷新次数太少：{checked}");
    }

    #[test]
    fn a_missing_diagnostics_store_degrades_to_the_placeholder() {
        // No `FrameTimeDiagnosticsPlugin` and no `DiagnosticsPlugin`: the overlay
        // must show its placeholder rather than panic on a missing resource.
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .init_resource::<ButtonInput<KeyCode>>()
            .add_plugins(FpsDisplayPlugin)
            .insert_resource(SimulationConfig::default());
        assert!(
            app.world().get_resource::<DiagnosticsStore>().is_none(),
            "本测试要覆盖 store 缺失时的降级路径"
        );

        for _ in 0..3 {
            app.update();
        }
        assert_eq!(label_text(&mut app), PLACEHOLDER);
    }

    #[test]
    fn config_switch_and_the_f4_key_rule_the_overlay() {
        // config.toml → [debug] fps = false: the overlay starts hidden.
        let mut app = overlay_app(false);
        app.update();
        assert_eq!(root_display(&mut app), Display::None);

        // …and the key is ignored rather than latching a hidden state.
        tap_f4(&mut app);
        assert_eq!(root_display(&mut app), Display::None, "关闭时 F4 无效果");

        // `config_hot_reload` replaces the whole resource; flipping the field is
        // the same observable step, and the overlay shows up without a restart.
        app.world_mut().resource_mut::<SimulationConfig>().debug.fps = true;
        app.update();
        assert_eq!(root_display(&mut app), Display::Flex, "重新打开后立即显示");

        // F4 is the binding that actually hides and restores it.
        tap_f4(&mut app);
        assert_eq!(root_display(&mut app), Display::None, "F4 收起读数");

        tap_f4(&mut app);
        assert_eq!(root_display(&mut app), Display::Flex, "F4 重新显示");
    }
}
