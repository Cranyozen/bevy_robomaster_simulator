//! Transient top-center toast for runtime events.
//!
//! The first (and so far only) producer is `config_hot_reload`: it writes a
//! [`ConfigReloaded`] message after every hot-reload attempt, and this module is
//! the only place that turns those messages into pixels. The split keeps the
//! config side free of UI types — it logs to the console and broadcasts an
//! outcome; whoever wants to display it reads the message.
//!
//! The toast lives in the top-center, the one corner no other panel claims (the
//! control hints sit bottom-left, the PiP inset bottom-right, the FPS readout
//! top-right). It is a small pill that removes itself: a reload is a transient
//! event, and a permanent banner would just become furniture. A *failed* edit
//! stays up much longer than a successful one, because "your file did not parse
//! and the old config is still running" is the part that surprises people.
//!
//! Rendered with plain UI components and no extra render pass: while idle the
//! wrapper is `Display::None`, so it costs nothing but the two component writes
//! when the visibility actually changes. `Text` is only rewritten when the words
//! differ (touching `Text` re-runs UI layout); during the hold phase the computed
//! tint stops changing, so the per-frame colour writes stop as well.

use bevy::prelude::*;

use crate::config::{CONFIG_PATH, ConfigReloadOutcome, ConfigReloaded, SimulationConfig};
use crate::systems::GameplaySystems;

/// Fade-in length, in seconds. Short: the toast should feel like a response to
/// the save, not an animation to wait for.
const FADE_IN: f32 = 0.18;
/// Fade-out length, in seconds — longer than the fade-in, because a notice that
/// vanishes abruptly reads as a glitch.
const FADE_OUT: f32 = 0.45;
/// Total lifetime of a successful notice: enough to read one line at a glance.
const APPLIED_DURATION: f32 = 2.6;
/// Total lifetime of a failed notice. A rejection leaves the *previous* config in
/// effect, which is easy to miss, so it is given the time to be noticed and read.
const FAILED_DURATION: f32 = 6.0;
/// Longest reason echoed on screen. A `toml` error is a multi-line report, and the
/// toast is one line, so it is collapsed by [`summarize`] and then capped at this
/// many characters.
const REASON_LIMIT: usize = 96;

const FONT_SIZE: f32 = 14.0;
const TEXT_COLOR: Color = Color::srgb(0.94, 0.96, 1.0);
const PANEL_BACKGROUND: Color = Color::srgba(0.03, 0.04, 0.06, 0.86);
/// Accent of an applied reload — the border it draws.
const APPLIED_ACCENT: Color = Color::srgb(0.46, 0.88, 0.60);
/// Accent of a rejected one. Also the label colour: a failure is the case where
/// the text itself is worth highlighting.
const FAILED_ACCENT: Color = Color::srgb(1.00, 0.47, 0.43);

/// Wrapper spanning the window width at the top; the panel inside is centered by
/// its `justify_content`. Only its `display` changes, which is what keeps the
/// hidden toast out of the layout pass entirely.
#[derive(Component)]
struct NoticeRoot;

/// The visible pill. A marker so the background/border can be tinted without
/// naming the wrapper.
#[derive(Component)]
struct NoticePanel;

#[derive(Component)]
struct NoticeLabel;

/// The toast currently on screen, if any.
#[derive(Resource, Default)]
struct NoticeToast {
    active: Option<ActiveNotice>,
}

/// One notice, with everything needed to draw it and to know when to drop it.
struct ActiveNotice {
    /// Already formatted for display; built once when the notice starts.
    text: String,
    /// Border colour: green when the edit was applied, red when it was rejected.
    accent: Color,
    /// Label colour, matching the accent on failures and neutral otherwise.
    tint: Color,
    /// Seconds since the notice appeared, advanced against `Time<Real>` so a
    /// clamped virtual clock cannot stretch the fade.
    elapsed: f32,
    duration: f32,
}

impl ActiveNotice {
    /// A successful reload says which file was picked up; a failed one says why
    /// it was not, because that is the only actionable half of the message.
    ///
    /// Both forms stay ASCII on purpose. The HUD's font has no middle dot — a
    /// decorative separator would render as a tofu box — and `summarize` turns
    /// anything else it cannot draw into `?`.
    fn new(outcome: &ConfigReloadOutcome) -> Self {
        match outcome {
            ConfigReloadOutcome::Applied => Self {
                text: format!("CONFIG RELOADED: {CONFIG_PATH}"),
                accent: APPLIED_ACCENT,
                tint: TEXT_COLOR,
                elapsed: 0.0,
                duration: APPLIED_DURATION,
            },
            ConfigReloadOutcome::Failed(reason) => Self {
                text: format!("CONFIG RELOAD FAILED: {}", summarize(reason)),
                accent: FAILED_ACCENT,
                tint: FAILED_ACCENT,
                elapsed: 0.0,
                duration: FAILED_DURATION,
            },
        }
    }

    fn alpha(&self) -> f32 {
        fade_alpha(self.elapsed, self.duration)
    }
}

/// Config hot-reload toast, top-center. Gated by `config.toml` → `[debug]
/// reload_notice`.
pub struct NoticePlugin;

impl Plugin for NoticePlugin {
    fn build(&self, app: &mut App) {
        app
            // Registered here as well so the plugin is self-contained; `App` makes
            // this idempotent, and `ConfigPlugin` registers the same message for
            // the writer's sake.
            .add_message::<ConfigReloaded>()
            .init_resource::<NoticeToast>()
            .add_systems(Startup, spawn_notice_toast)
            .add_systems(
                Update,
                drive_notice_toast.in_set(GameplaySystems::GameLogic),
            );
    }
}

fn spawn_notice_toast(mut commands: Commands) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(12.0),
                left: Val::Px(0.0),
                width: Val::Percent(100.0),
                justify_content: JustifyContent::Center,
                // Starts collapsed; the first notice flips this to `Display::Flex`.
                display: Display::None,
                ..default()
            },
            NoticeRoot,
        ))
        .with_children(|wrapper| {
            wrapper
                .spawn((
                    Node {
                        padding: UiRect::px(12.0, 12.0, 6.0, 6.0),
                        border: UiRect::all(Val::Px(1.0)),
                        border_radius: BorderRadius::all(Val::Px(9.0)),
                        max_width: Val::Px(640.0),
                        ..default()
                    },
                    BackgroundColor(PANEL_BACKGROUND),
                    BorderColor::all(APPLIED_ACCENT),
                    NoticePanel,
                ))
                .with_children(|panel| {
                    panel.spawn((
                        Text::new(""),
                        TextFont::from_font_size(FONT_SIZE),
                        TextColor(TEXT_COLOR),
                        NoticeLabel,
                    ));
                });
        });
}

/// Turns queued reload outcomes into a toast, ages it, and fades it out.
fn drive_notice_toast(
    time: Res<Time<Real>>,
    config: Res<SimulationConfig>,
    mut reloaded: MessageReader<ConfigReloaded>,
    mut toast: ResMut<NoticeToast>,
    mut roots: Query<&mut Node, With<NoticeRoot>>,
    mut panels: Query<(&mut BackgroundColor, &mut BorderColor), With<NoticePanel>>,
    mut labels: Query<(&mut Text, &mut TextColor), With<NoticeLabel>>,
) {
    // The newest save is the state of the world; anything it superseded in the
    // same frame is dropped, and a fresh notice restarts the clock rather than
    // queueing behind the one on screen.
    if let Some(message) = reloaded.read().last() {
        toast.active = Some(ActiveNotice::new(&message.outcome));
    }

    // The switch also silences a notice that is already up: turning the feature
    // off must not leave one hanging until its timer runs out.
    if !config.debug.reload_notice {
        toast.active = None;
    }

    if let Some(active) = toast.active.as_mut() {
        active.elapsed += time.delta_secs();
        if active.elapsed >= active.duration {
            toast.active = None;
        }
    }

    let display = display_for(toast.active.is_some());
    for mut layout in &mut roots {
        if layout.display != display {
            layout.display = display;
        }
    }

    let Some(active) = toast.active.as_ref() else {
        return;
    };

    let alpha = active.alpha();
    let tint = active.tint.with_alpha(alpha);
    for (mut text, mut color) in &mut labels {
        // `Text` re-runs UI layout, so it is written on change only.
        if text.0 != active.text {
            text.0.clone_from(&active.text);
        }
        if color.0 != tint {
            color.0 = tint;
        }
    }

    let background = PANEL_BACKGROUND.with_alpha(PANEL_BACKGROUND.alpha() * alpha);
    let border = BorderColor::all(active.accent.with_alpha(active.accent.alpha() * alpha));
    for (mut fill, mut outline) in &mut panels {
        if fill.0 != background {
            fill.0 = background;
        }
        if *outline != border {
            *outline = border;
        }
    }
}

fn display_for(visible: bool) -> Display {
    if visible {
        Display::Flex
    } else {
        Display::None
    }
}

/// Triangular alpha envelope: ramp up over `FADE_IN`, hold at 1.0, ramp down over
/// the last `FADE_OUT` seconds. Combining the two ramps with `min` keeps the
/// result in `0..=1` and monotonic even for a duration barely longer than one
/// fade, where the two ramps overlap.
fn fade_alpha(elapsed: f32, duration: f32) -> f32 {
    if !elapsed.is_finite() || !duration.is_finite() {
        return 0.0;
    }
    let fade_in = elapsed / FADE_IN;
    let fade_out = (duration - elapsed) / FADE_OUT;
    fade_in.clamp(0.0, 1.0).min(fade_out.clamp(0.0, 1.0))
}

/// Collapses a parse error to the one line the toast can show.
///
/// `toml` renders an error as a location line, an annotated snippet, and — for
/// the semantic errors, which are the ones that point at a *value* rather than a
/// typo — the actual complaint last. The toast keeps the location plus that
/// complaint and drops the snippet, so `fps = "on"` comes out as
/// `TOML parse error at line 3, column 1 - invalid type: string "on", expected a
/// boolean` instead of only telling the user where to look.
///
/// The HUD's font has no glyph outside ASCII (and no CJK at all), so anything
/// else becomes `?` rather than a row of tofu boxes; the full report is printed
/// to the console either way. Truncation counts characters, not bytes, so a
/// multi-byte character is never split in half.
fn summarize(reason: &str) -> String {
    let lines: Vec<&str> = reason
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty())
        .collect();

    let Some(&location) = lines.first() else {
        return "unknown error".to_string();
    };
    let detail = match lines.last().copied() {
        Some(last) if last != location && !is_snippet(last) => Some(last),
        _ => None,
    };

    let joined = match detail {
        Some(detail) => format!("{location} - {detail}"),
        None => location.to_string(),
    };

    let flattened: String = joined
        .chars()
        .map(|c| {
            if c.is_ascii_graphic() || c == ' ' {
                c
            } else {
                '?'
            }
        })
        .collect();

    if flattened.chars().count() <= REASON_LIMIT {
        return flattened;
    }
    let mut truncated: String = flattened.chars().take(REASON_LIMIT).collect();
    // ASCII dots, not `…`: see the font note above.
    truncated.push_str("...");
    truncated
}

/// An annotated source line from a `toml` error report: the gutter alone
/// (`  |`), a caret row (`  |   ^`), or source echoed with its line number
/// (`8 | [debug`). These carry no diagnosis, so the toast never quotes them.
fn is_snippet(line: &str) -> bool {
    let rest = line.trim_start();
    if rest.starts_with('|') {
        return true;
    }
    let digits = rest.chars().take_while(char::is_ascii_digit).count();
    digits > 0 && rest[digits..].trim_start().starts_with('|')
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::time::TimeUpdateStrategy;
    use std::time::Duration;

    /// Headless app with the real plugin, the real config resource and a fixed
    /// frame duration, so the fade can be stepped deterministically. No render
    /// world is needed: the toast is ordinary UI components.
    fn notice_app(reload_notice: bool) -> App {
        let mut config = SimulationConfig::default();
        config.debug.reload_notice = reload_notice;

        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .add_plugins(NoticePlugin)
            .insert_resource(config)
            .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_millis(
                100,
            )));
        app
    }

    fn send(app: &mut App, outcome: ConfigReloadOutcome) {
        app.world_mut()
            .write_message(ConfigReloaded { outcome })
            .expect("ConfigReloaded is registered");
    }

    /// Runs one 100 ms frame. The first update also runs the `Startup` spawn.
    fn step(app: &mut App) {
        app.update();
    }

    fn display(app: &mut App) -> Display {
        let mut query = app.world_mut().query_filtered::<&Node, With<NoticeRoot>>();
        let mut found: Vec<Display> = query.iter(app.world()).map(|node| node.display).collect();
        assert_eq!(found.len(), 1, "toast has exactly one root");
        found.remove(0)
    }

    fn label(app: &mut App) -> (String, Color) {
        let mut query = app
            .world_mut()
            .query_filtered::<(&Text, &TextColor), With<NoticeLabel>>();
        let mut found: Vec<(String, Color)> = query
            .iter(app.world())
            .map(|(text, color)| (text.0.clone(), color.0))
            .collect();
        assert_eq!(found.len(), 1, "toast has exactly one label");
        found.remove(0)
    }

    fn border(app: &mut App) -> Color {
        let mut query = app
            .world_mut()
            .query_filtered::<&BorderColor, With<NoticePanel>>();
        let mut found: Vec<Color> = query.iter(app.world()).map(|color| color.top).collect();
        assert_eq!(found.len(), 1, "toast has exactly one panel");
        found.remove(0)
    }

    /// Steps until the given number of seconds has been simulated.
    fn advance(app: &mut App, seconds: f32) {
        for _ in 0..(seconds / 0.1).round() as usize {
            step(app);
        }
    }

    #[test]
    fn an_applied_reload_names_the_file_then_takes_itself_away() {
        let mut app = notice_app(true);
        step(&mut app);
        assert_eq!(display(&mut app), Display::None, "空闲时完全不参与布局");

        send(&mut app, ConfigReloadOutcome::Applied);
        step(&mut app);
        assert_eq!(display(&mut app), Display::Flex);

        let (text, _) = label(&mut app);
        assert!(text.contains(CONFIG_PATH), "提示要点名是哪份配置：{text:?}");
        assert!(text.contains("RELOADED"), "实际 {text:?}");

        // Stays up for its lifetime, then collapses on its own.
        advance(&mut app, APPLIED_DURATION - 0.3);
        assert_eq!(display(&mut app), Display::Flex, "时限内保持可见");
        advance(&mut app, 0.4);
        assert_eq!(display(&mut app), Display::None, "到时自动收起");
    }

    #[test]
    fn a_failed_reload_says_why_and_stays_up_longer() {
        let mut app = notice_app(true);
        send(
            &mut app,
            ConfigReloadOutcome::Failed(
                "TOML parse error at line 12, column 3\n  |\n12 | fps = maybe\n".to_string(),
            ),
        );
        step(&mut app);
        // Past the fade-in, so the colours can be compared at full strength.
        advance(&mut app, 0.3);

        let (text, color) = label(&mut app);
        assert!(text.contains("FAILED"), "实际 {text:?}");
        assert!(text.contains("line 12"), "要保留出错位置：{text:?}");
        assert!(
            !text.contains('\n'),
            "提示只有一行，多行的 TOML 报告必须折叠：{text:?}"
        );
        assert_eq!(color, FAILED_ACCENT, "失败用警示色");
        assert_eq!(border(&mut app), FAILED_ACCENT, "边框同色");

        // A rejection silently keeps the old config running, which is the
        // surprising part, so it is given more time on screen than a success.
        let applied = ActiveNotice::new(&ConfigReloadOutcome::Applied);
        let failed = ActiveNotice::new(&ConfigReloadOutcome::Failed("boom".to_string()));
        assert!(
            failed.duration > applied.duration,
            "失败提示要比成功提示停留更久"
        );

        advance(&mut app, APPLIED_DURATION + 0.5);
        assert_eq!(display(&mut app), Display::Flex, "失败提示停留更久");
    }

    #[test]
    fn the_newest_reload_of_one_save_wins() {
        // One save can produce several events, all drained in the same frame; the
        // HUD must show the state of the world, not a queue.
        let mut app = notice_app(true);
        send(&mut app, ConfigReloadOutcome::Applied);
        send(
            &mut app,
            ConfigReloadOutcome::Failed("TOML parse error at line 4".to_string()),
        );
        step(&mut app);

        let (text, _) = label(&mut app);
        assert!(
            text.starts_with("CONFIG RELOAD FAILED"),
            "后者（最后一次尝试）胜出：{text:?}"
        );
    }

    #[test]
    fn a_second_reload_restarts_the_clock() {
        let mut app = notice_app(true);
        send(&mut app, ConfigReloadOutcome::Applied);
        step(&mut app);

        // Nearly expired…
        advance(&mut app, APPLIED_DURATION - 0.2);
        assert_eq!(display(&mut app), Display::Flex);

        // …and a new save brings it back at full strength instead of letting the
        // old timer finish it off.
        send(&mut app, ConfigReloadOutcome::Applied);
        step(&mut app);
        advance(&mut app, 0.3);
        assert_eq!(display(&mut app), Display::Flex);
        assert_eq!(
            label(&mut app).1,
            TEXT_COLOR.with_alpha(1.0),
            "重新触发应回到不透明"
        );
    }

    #[test]
    fn the_config_switch_silences_the_toast() {
        // `[debug] reload_notice = false`: nothing is ever shown.
        let mut app = notice_app(false);
        send(&mut app, ConfigReloadOutcome::Applied);
        for _ in 0..3 {
            step(&mut app);
        }
        assert_eq!(display(&mut app), Display::None, "关掉后不显示提示");

        // Flipping it at runtime (the way `config_hot_reload` replaces the whole
        // resource) also clears a notice that is already on screen.
        let mut app = notice_app(true);
        send(&mut app, ConfigReloadOutcome::Applied);
        step(&mut app);
        assert_eq!(display(&mut app), Display::Flex);

        app.world_mut()
            .resource_mut::<SimulationConfig>()
            .debug
            .reload_notice = false;
        step(&mut app);
        assert_eq!(display(&mut app), Display::None, "关掉时立即收起");
    }

    #[test]
    fn the_fade_is_a_window_of_opacity_over_the_notice_lifetime() {
        let duration = APPLIED_DURATION;

        assert_eq!(fade_alpha(0.0, duration), 0.0, "第一帧透明");
        assert_eq!(fade_alpha(FADE_IN / 2.0, duration), 0.5, "淡入过半");
        assert_eq!(fade_alpha(FADE_IN, duration), 1.0, "淡入结束");
        assert_eq!(fade_alpha(duration / 2.0, duration), 1.0, "中段保持不透明");
        assert_eq!(fade_alpha(duration, duration), 0.0, "到点透明");
        assert_eq!(fade_alpha(duration * 2.0, duration), 0.0, "超时仍为 0");
        assert_eq!(fade_alpha(-1.0, duration), 0.0, "负时间不应出现负 alpha");

        // Never leaves 0..=1, and never brightens once the fade-in is over — the
        // two ramps overlap on a very short notice, and `min` must still keep the
        // envelope monotonic.
        for duration in [0.1, 0.4, 0.6, 1.0, 3.0, 10.0] {
            let mut previous = 0.0;
            for step in 0..=(duration * 100.0) as usize {
                let elapsed = step as f32 / 100.0;
                let alpha = fade_alpha(elapsed, duration);
                assert!((0.0..=1.0).contains(&alpha), "alpha={alpha}");
                if elapsed > FADE_IN {
                    assert!(
                        alpha <= previous + 1e-6,
                        "淡入结束后只能变暗：{previous} → {alpha}（duration={duration}）"
                    );
                }
                previous = alpha;
            }
        }
    }

    #[test]
    fn a_toml_error_is_collapsed_to_one_short_line() {
        assert_eq!(summarize(""), "unknown error");
        assert_eq!(summarize("\n   \n"), "unknown error");

        // Syntax error: the location line, the snippet dropped, the trailing
        // complaint kept.
        assert_eq!(
            summarize(concat!(
                "TOML parse error at line 1, column 6\n",
                "  |\n",
                "1 | a = 1 2\n",
                "  |      ^\n",
                "expected newline, `#`"
            )),
            "TOML parse error at line 1, column 6 - expected newline, `#`"
        );

        // A report that ends in a snippet keeps the location alone: the caret row
        // is not a diagnosis.
        assert_eq!(
            summarize("TOML parse error at line 3\n  |\n3 | fps = 5\n  | ^^^^^^^\n"),
            "TOML parse error at line 3"
        );
        assert_eq!(
            summarize("\n  TOML parse error at line 3\n  |\n  |   ^\n"),
            "TOML parse error at line 3"
        );

        let long = "x".repeat(REASON_LIMIT + 50);
        let short = summarize(&long);
        assert_eq!(short.chars().count(), REASON_LIMIT + 3, "上限 + 省略号");
        assert!(short.ends_with("..."));

        // Non-ASCII text (a value from the file, echoed by toml) neither gets cut
        // mid-character nor reaches the HUD as a glyph the font does not have.
        let cjk = "错误".repeat(REASON_LIMIT);
        let clamped = summarize(&cjk);
        assert!(clamped.is_ascii(), "实际 {clamped:?}");
        assert!(clamped.starts_with('?'));
        assert_eq!(clamped.chars().count(), REASON_LIMIT + 3);
    }

    #[test]
    fn every_notice_line_survives_the_hud_font() {
        // The default font has no middle dot and no CJK; anything it cannot draw
        // shows up as a tofu box, so the strings handed to `Text` stay ASCII.
        for notice in [
            ActiveNotice::new(&ConfigReloadOutcome::Applied),
            ActiveNotice::new(&ConfigReloadOutcome::Failed(
                concat!(
                    "TOML parse error at line 3, column 1\n",
                    "3 | fps = \"中文\"\n",
                    "invalid type: string \"中文\", expected a boolean"
                )
                .to_string(),
            )),
        ] {
            assert!(
                notice.text.is_ascii(),
                "提示文本必须全部是默认字体能画的字符：{:?}",
                notice.text
            );
        }
    }

    #[test]
    fn the_reason_line_is_what_the_toast_shows() {
        let notice = ActiveNotice::new(&ConfigReloadOutcome::Failed(
            "TOML parse error at line 8, column 1\n  |\n8 | [debug\n".to_string(),
        ));

        assert!(notice.text.contains("line 8, column 1"));
        assert!(!notice.text.contains('\n'));
        assert_eq!(notice.duration, FAILED_DURATION);
        assert_eq!(notice.tint, FAILED_ACCENT);
    }
}
