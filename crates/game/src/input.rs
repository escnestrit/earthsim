//! Keyboard, gamepad and steering wheel. Bevy reads controllers through gilrs (Windows.Gaming.Input / XInput
//! on Windows), so racing wheels show up as gamepads with extra axes. Which axis is the wheel and which are
//! the pedals differs between models, so `controls.txt` next to the exe maps them; F4 shows every axis live.
use crate::Sim;
use bevy::input::gamepad::{Gamepad, GamepadAxis, GamepadButton, GamepadInput};
use bevy::prelude::*;
use earthsim::game::Key;
use phys::Analog;

pub struct InputPlugin;

impl Plugin for InputPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Controls::load()).init_resource::<InputMonitor>().add_systems(Update, read_input);
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Binding {
    pub input: GamepadInput,
    pub invert: bool,
    /// the control reports -1..1 instead of 0..1 (common for wheel pedals)
    pub full_range: bool,
}

#[derive(Resource, Clone, Debug)]
pub struct Controls {
    pub steer: Binding,
    pub throttle: Binding,
    pub brake: Binding,
    pub handbrake: GamepadButton,
    pub deadzone: f32,
    pub source: String,
}

#[derive(Resource, Default)]
pub struct InputMonitor {
    pub show: bool,
    pub lines: Vec<String>,
    pub active: bool,
}

pub const DEFAULT_CONTROLS: &str = "\
# earthsim controls. Inputs: LeftStickX LeftStickY LeftZ RightStickX RightStickY RightZ Other0..Other15 (axes)
# or South East North West LeftTrigger LeftTrigger2 RightTrigger RightTrigger2 ... (buttons, analog value used).
# Add 'invert' to flip a control, 'full' if it reports -1..1 instead of 0..1. Press F4 in the game to see
# all axes of your wheel/pedals live and pick the right ones.
steer = LeftStickX
throttle = RightTrigger2
brake = LeftTrigger2
handbrake = West
deadzone = 0.04
";

fn parse_input(name: &str) -> Option<GamepadInput> {
    use GamepadAxis as A;
    use GamepadButton as B;
    let a = |x| Some(GamepadInput::Axis(x));
    let b = |x| Some(GamepadInput::Button(x));
    match name {
        "LeftStickX" => a(A::LeftStickX),
        "LeftStickY" => a(A::LeftStickY),
        "LeftZ" => a(A::LeftZ),
        "RightStickX" => a(A::RightStickX),
        "RightStickY" => a(A::RightStickY),
        "RightZ" => a(A::RightZ),
        "South" => b(B::South),
        "East" => b(B::East),
        "North" => b(B::North),
        "West" => b(B::West),
        "LeftTrigger" => b(B::LeftTrigger),
        "LeftTrigger2" => b(B::LeftTrigger2),
        "RightTrigger" => b(B::RightTrigger),
        "RightTrigger2" => b(B::RightTrigger2),
        "Select" => b(B::Select),
        "Start" => b(B::Start),
        n => n.strip_prefix("Other").and_then(|k| k.parse().ok()).map(|k| GamepadInput::Axis(A::Other(k))),
    }
}

pub fn input_name(i: &GamepadInput) -> String {
    match i {
        GamepadInput::Axis(GamepadAxis::Other(k)) => format!("Other{k}"),
        GamepadInput::Axis(a) => format!("{a:?}"),
        GamepadInput::Button(b) => format!("{b:?}"),
    }
}

impl Controls {
    pub fn parse(text: &str) -> Result<Controls, String> {
        let mut c = Controls::parse_lines(DEFAULT_CONTROLS)?;
        c.apply(text)?;
        Ok(c)
    }
    fn parse_lines(text: &str) -> Result<Controls, String> {
        let mut c = Controls {
            steer: Binding { input: GamepadInput::Axis(GamepadAxis::LeftStickX), invert: false, full_range: false },
            throttle: Binding { input: GamepadInput::Button(GamepadButton::RightTrigger2), invert: false, full_range: false },
            brake: Binding { input: GamepadInput::Button(GamepadButton::LeftTrigger2), invert: false, full_range: false },
            handbrake: GamepadButton::West,
            deadzone: 0.04,
            source: "built-in defaults".into(),
        };
        c.apply(text)?;
        Ok(c)
    }
    fn apply(&mut self, text: &str) -> Result<(), String> {
        for (ln, line) in text.lines().enumerate() {
            let line = line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            let (k, v) = line.split_once('=').ok_or_else(|| format!("line {}: expected 'name = value'", ln + 1))?;
            let (k, v) = (k.trim(), v.trim());
            let mut words = v.split_whitespace();
            let first = words.next().unwrap_or("");
            let flags: Vec<&str> = words.collect();
            let bind = || -> Result<Binding, String> {
                let input = parse_input(first).ok_or_else(|| format!("line {}: unknown input '{first}'", ln + 1))?;
                Ok(Binding { input, invert: flags.contains(&"invert"), full_range: flags.contains(&"full") })
            };
            match k {
                "steer" => self.steer = bind()?,
                "throttle" => self.throttle = bind()?,
                "brake" => self.brake = bind()?,
                "handbrake" => match parse_input(first) {
                    Some(GamepadInput::Button(b)) => self.handbrake = b,
                    _ => return Err(format!("line {}: handbrake must be a button", ln + 1)),
                },
                "deadzone" => self.deadzone = first.parse().map_err(|_| format!("line {}: bad number", ln + 1))?,
                _ => return Err(format!("line {}: unknown setting '{k}'", ln + 1)),
            }
        }
        Ok(())
    }

    fn load() -> Controls {
        let path = std::env::current_exe().ok().map(|p| p.with_file_name("controls.txt"));
        match path.as_ref().and_then(|p| std::fs::read_to_string(p).ok()) {
            Some(t) => match Controls::parse(&t) {
                Ok(mut c) => {
                    c.source = "controls.txt".into();
                    c
                }
                Err(e) => {
                    world::log::log(&format!("controls.txt: {e}; using defaults"));
                    Controls::parse("").unwrap()
                }
            },
            None => {
                if let Some(p) = path {
                    let _ = std::fs::write(p, DEFAULT_CONTROLS); // leave a template to edit
                }
                Controls::parse("").unwrap()
            }
        }
    }

    /// Normalised value of a binding: steer -1..1, pedals 0..1.
    pub fn value(&self, b: &Binding, raw: Option<f32>, pedal: bool) -> f32 {
        let mut v = raw.unwrap_or(0.0);
        if b.invert {
            v = -v;
        }
        if pedal {
            if b.full_range {
                v = (v + 1.0) * 0.5;
            }
            v = v.clamp(0.0, 1.0);
            if v < self.deadzone {
                0.0
            } else {
                v
            }
        } else {
            v = v.clamp(-1.0, 1.0);
            if v.abs() < self.deadzone {
                0.0
            } else {
                v
            }
        }
    }
}

const KEYS: &[(KeyCode, Key)] = &[
    (KeyCode::KeyW, Key::Up),
    (KeyCode::ArrowUp, Key::Up),
    (KeyCode::KeyS, Key::Down),
    (KeyCode::ArrowDown, Key::Down),
    (KeyCode::KeyA, Key::Left),
    (KeyCode::ArrowLeft, Key::Left),
    (KeyCode::KeyD, Key::Right),
    (KeyCode::ArrowRight, Key::Right),
    (KeyCode::Space, Key::Handbrake),
    (KeyCode::KeyC, Key::Camera),
    (KeyCode::BracketRight, Key::HourUp),
    (KeyCode::BracketLeft, Key::HourDown),
    (KeyCode::KeyL, Key::Lights),
    (KeyCode::KeyR, Key::Respawn),
    (KeyCode::KeyU, Key::Recover),
    (KeyCode::KeyH, Key::Hud),
    (KeyCode::KeyT, Key::Tcs),
    (KeyCode::KeyB, Key::Abs),
    (KeyCode::F1, Key::Help),
    (KeyCode::F2, Key::Teleport),
    (KeyCode::F3, Key::SoftBody),
];

pub fn read_input(sim: ResMut<Sim>, keys: Res<ButtonInput<KeyCode>>, pads: Query<(&Gamepad, Option<&Name>)>, controls: Res<Controls>, mut mon: ResMut<InputMonitor>) {
    let mut g = sim.g();
    for (code, k) in KEYS {
        if keys.just_pressed(*code) {
            g.key_state(*k, true);
        }
        if keys.just_released(*code) {
            g.key_state(*k, false);
        }
    }
    if keys.just_pressed(KeyCode::F4) {
        mon.show = !mon.show;
    }
    // first controller with any input wins
    let mut analog = None;
    let mut hb = false;
    mon.lines.clear();
    mon.active = false;
    for (pad, name) in &pads {
        let c = &*controls;
        let steer = c.value(&c.steer, pad.get(c.steer.input.clone()), false);
        let thr = c.value(&c.throttle, pad.get(c.throttle.input.clone()), true);
        let brk = c.value(&c.brake, pad.get(c.brake.input.clone()), true);
        if mon.show {
            mon.lines.push(format!("CONTROLLER: {}  (MAPPING: {})", name.map(|n| n.as_str().to_uppercase()).unwrap_or_else(|| "UNNAMED".into()), c.source.to_uppercase()));
            let mut all: Vec<(String, f32)> = pad.analog().all_axes_and_values().map(|(i, v)| (input_name(i), v)).collect();
            all.sort_by(|a, b| a.0.cmp(&b.0));
            for chunk in all.chunks(4) {
                mon.lines.push(chunk.iter().map(|(n, v)| format!("{n} {v:+.2}")).collect::<Vec<_>>().join("   "));
            }
            mon.lines.push(format!("-> STEER {steer:+.2}  THROTTLE {thr:.2}  BRAKE {brk:.2}"));
        }
        if analog.is_none() && (steer != 0.0 || thr > 0.0 || brk > 0.0) {
            // steering: Bevy/gilrs report right as positive, the driver model wants left positive
            analog = Some(Analog { steer: -steer as f64, throttle: thr as f64, brake: brk as f64 });
            hb = pad.pressed(c.handbrake);
        }
    }
    mon.active = analog.is_some();
    g.keys.analog = analog;
    if analog.is_some() {
        g.keys.handbrake = hb || keys.pressed(KeyCode::Space);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_parse_and_overrides_apply() {
        let c = Controls::parse("").unwrap();
        assert_eq!(c.steer.input, GamepadInput::Axis(GamepadAxis::LeftStickX));
        let c = Controls::parse("steer = Other3 invert\nthrottle = RightZ full\nbrake = LeftZ full invert # pedals\n").unwrap();
        assert_eq!(c.steer.input, GamepadInput::Axis(GamepadAxis::Other(3)));
        assert!(c.steer.invert && c.throttle.full_range && c.brake.invert);
        assert!(Controls::parse("steer = Nope").is_err());
        assert!(Controls::parse("speed = 3").is_err());
    }

    #[test]
    fn pedal_and_wheel_normalisation() {
        let c = Controls::parse("throttle = RightZ full\nbrake = LeftZ full invert").unwrap();
        assert_eq!(c.value(&c.throttle, Some(-1.0), true), 0.0); // released pedal
        assert_eq!(c.value(&c.throttle, Some(1.0), true), 1.0);
        assert!((c.value(&c.throttle, Some(0.0), true) - 0.5).abs() < 1e-6);
        assert_eq!(c.value(&c.brake, Some(1.0), true), 0.0); // inverted: +1 = released
        assert_eq!(c.value(&c.steer, Some(0.02), false), 0.0); // dead zone
        assert_eq!(c.value(&c.steer, Some(-2.0), false), -1.0);
        assert_eq!(c.value(&c.throttle, None, true), 0.0);
    }
}
