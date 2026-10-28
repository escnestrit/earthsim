//! On-screen text: speed/gear/rpm, the simulation's status lines, help, and the controller monitor (F4).
use crate::input::InputMonitor;
use crate::Sim;
use bevy::prelude::*;

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup).add_systems(Update, update.after(crate::view::sync_world));
    }
}

#[derive(Component)]
struct HudText;
#[derive(Component)]
struct Speedo;

fn setup(mut commands: Commands) {
    let font = |px: f32| TextFont { font_size: FontSize::Px(px), ..default() };
    commands.spawn((
        HudText,
        Text::new(""),
        font(15.0),
        TextColor(Color::srgb(0.95, 0.95, 0.95)),
        TextShadow::default(),
        Node { position_type: PositionType::Absolute, top: Val::Px(10.0), left: Val::Px(12.0), ..default() },
    ));
    commands.spawn((
        Speedo,
        Text::new(""),
        font(30.0),
        TextColor(Color::srgb(1.0, 1.0, 1.0)),
        TextShadow::default(),
        Node { position_type: PositionType::Absolute, bottom: Val::Px(16.0), right: Val::Px(24.0), ..default() },
    ));
}

fn update(sim: Res<Sim>, mon: Res<InputMonitor>, mut text: Query<&mut Text, (With<HudText>, Without<Speedo>)>, mut speedo: Query<&mut Text, (With<Speedo>, Without<HudText>)>) {
    let h = sim.g().hud();
    for mut t in &mut text {
        let mut s = String::new();
        if h.show {
            for l in &h.lines {
                s.push_str(l);
                s.push('\n');
            }
            if !h.warn.is_empty() {
                s.push_str(&h.warn);
                s.push('\n');
            }
        }
        if mon.active {
            s.push_str("CONTROLLER ACTIVE (WHEEL / GAMEPAD)\n");
        }
        if mon.show {
            s.push_str("\nINPUT MONITOR (F4)\n");
            if mon.lines.is_empty() {
                s.push_str("NO CONTROLLERS DETECTED\n");
            }
            for l in &mon.lines {
                s.push_str(l);
                s.push('\n');
            }
        }
        if !h.help.is_empty() {
            s.push('\n');
            s.push_str(&h.help);
        }
        t.0 = s;
    }
    for mut t in &mut speedo {
        let gear = match h.gear {
            g if g < 0 => "R".to_string(),
            0 => "N".to_string(),
            g => g.to_string(),
        };
        t.0 = format!("{:.0} KM/H   {}   {:.0} RPM", h.speed_kmh, gear, h.rpm);
    }
}
