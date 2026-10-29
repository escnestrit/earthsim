//! Graphics quality (`graphics.txt` next to the exe: `quality = low | medium | high`; F5 cycles it in game).
//! It sets what costs the most frame time: sun shadows, anti-aliasing, how far the world is loaded, and how
//! dense generated forests are.
use crate::view::{MainCamera, Sun};
use crate::Sim;
use bevy::light::{CascadeShadowConfig, CascadeShadowConfigBuilder, DirectionalLightShadowMap};
use bevy::prelude::*;
use bevy::render::view::{Msaa, Tonemapping};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quality {
    Low,
    Medium,
    High,
}

impl Quality {
    pub fn parse(s: &str) -> Option<Quality> {
        match s.trim().to_ascii_lowercase().as_str() {
            "low" | "0" => Some(Quality::Low),
            "medium" | "1" => Some(Quality::Medium),
            "high" | "2" => Some(Quality::High),
            _ => None,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Quality::Low => "LOW",
            Quality::Medium => "MEDIUM",
            Quality::High => "HIGH",
        }
    }
    fn next(self) -> Quality {
        match self {
            Quality::Low => Quality::Medium,
            Quality::Medium => Quality::High,
            Quality::High => Quality::Low,
        }
    }
    /// (shadows, shadow distance m, cascades, shadow map size, msaa samples, world radius m, trees per tile)
    pub fn params(self) -> (bool, f32, usize, usize, u32, f64, usize) {
        match self {
            Quality::Low => (false, 0.0, 1, 1024, 1, 800.0, 250),
            Quality::Medium => (true, 150.0, 2, 2048, 2, 1200.0, 600),
            Quality::High => (true, 400.0, 3, 4096, 4, 1600.0, 1200),
        }
    }
}

/// Reads `quality = ...` from the settings file text; anything else is ignored.
pub fn parse_file(text: &str) -> Option<Quality> {
    text.lines().filter_map(|l| l.split('#').next()).filter_map(|l| l.split_once('=')).find(|(k, _)| k.trim().eq_ignore_ascii_case("quality")).and_then(|(_, v)| Quality::parse(v))
}

#[derive(Resource)]
pub struct Graphics {
    pub quality: Quality,
    applied: Option<Quality>,
    /// a post effect failed on this GPU: run without HDR bloom, sky, SSAO and TAA
    pub safe: bool,
}

/// Render errors (an effect this GPU/driver cannot do) switch the post effects off instead of stopping the
/// game; the reason goes to the log.
pub fn on_render_error(e: &bevy::render::error_handler::RenderError, main: &mut World, _render: &mut World) -> bevy::render::error_handler::RenderErrorPolicy {
    if let Some(mut g) = main.get_resource_mut::<Graphics>() {
        if !g.safe {
            g.safe = true;
            g.applied = None;
            world::log::log(&format!("render error, post effects switched off: {}", e.description.lines().take(4).collect::<Vec<_>>().join(" | ")));
        }
    }
    bevy::render::error_handler::RenderErrorPolicy::Ignore
}

pub struct SettingsPlugin;

impl Plugin for SettingsPlugin {
    fn build(&self, app: &mut App) {
        let exe_dir = std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.to_path_buf())).unwrap_or_else(|| ".".into());
        let q = std::env::var("EARTHSIM_QUALITY").ok().and_then(|v| Quality::parse(&v)).or_else(|| std::fs::read_to_string(exe_dir.join("graphics.txt")).ok().and_then(|t| parse_file(&t))).unwrap_or(Quality::Medium);
        // tree density must be known before the first tiles are built
        world::areas::MAX_TREES_PER_TILE.store(q.params().6, std::sync::atomic::Ordering::Relaxed);
        app.insert_resource(Graphics { quality: q, applied: None, safe: false }).add_systems(Update, apply);
    }
}

#[allow(clippy::too_many_arguments)]
fn apply(
    keys: Res<ButtonInput<KeyCode>>,
    mut gfx: ResMut<Graphics>,
    sim: Res<Sim>,
    mut commands: Commands,
    cams: Query<Entity, With<MainCamera>>,
    mut sun: Query<(Entity, &mut DirectionalLight), With<Sun>>,
    tiles: Res<crate::view::Tiles>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut media: ResMut<Assets<bevy::light::atmosphere::ScatteringMedium>>,
    mut atmosphere: Local<Option<Entity>>,
) {
    if keys.just_pressed(KeyCode::F5) {
        gfx.quality = gfx.quality.next();
        let exe_dir = std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.to_path_buf())).unwrap_or_else(|| ".".into());
        let _ = std::fs::write(exe_dir.join("graphics.txt"), format!("# low / medium / high (F5 in game)\nquality = {}\n", gfx.quality.name().to_lowercase()));
    }
    if gfx.applied == Some(gfx.quality) {
        return;
    }
    let (shadows, dist, cascades, map, msaa, radius, trees) = gfx.quality.params();
    for (e, mut l) in &mut sun {
        l.shadow_maps_enabled = shadows;
        let c: CascadeShadowConfig = CascadeShadowConfigBuilder { num_cascades: cascades, first_cascade_far_bound: 25.0f32.min(dist.max(1.0)), maximum_distance: dist.max(30.0), ..default() }.build();
        commands.entity(e).insert(c);
    }
    commands.insert_resource(DirectionalLightShadowMap { size: map });
    // headless test captures run on a software GL without multisampled HDR targets
    let msaa = if std::env::var("EARTHSIM_SHOT").is_ok() { 1 } else { msaa };
    let ms = match msaa {
        1 => Msaa::Off,
        2 => Msaa::Sample2,
        _ => Msaa::Sample4,
    };
    // post-processing per quality: low = plain; medium = HDR, bloom, physical sky, MSAA;
    // high = TAA, SSAO and sharpening instead of MSAA, deeper relief mapping
    let q = gfx.quality;
    let post = !gfx.safe;
    if !post {
        if let Some(a) = atmosphere.take() {
            commands.entity(a).despawn();
        }
    }
    if q != Quality::Low && post && atmosphere.is_none() {
        let medium = media.add(bevy::light::atmosphere::ScatteringMedium::default());
        *atmosphere = Some(commands.spawn(bevy::light::Atmosphere::earth(medium)).id());
    }
    for e in &cams {
        let mut c = commands.entity(e);
        c.insert(Tonemapping::TonyMcMapface);
        c.remove::<(bevy::anti_alias::taa::TemporalAntiAliasing, bevy::pbr::ScreenSpaceAmbientOcclusion, bevy::anti_alias::contrast_adaptive_sharpening::ContrastAdaptiveSharpening)>();
        match q {
            _ if !post => {
                c.remove::<(bevy::post_process::bloom::Bloom, bevy::pbr::AtmosphereSettings, bevy::camera::Hdr)>();
                c.insert(if q == Quality::Low { Msaa::Off } else { ms });
            }
            Quality::Low => {
                c.remove::<(bevy::post_process::bloom::Bloom, bevy::pbr::AtmosphereSettings)>();
                c.insert(Msaa::Off);
            }
            Quality::Medium => {
                c.insert((bevy::camera::Hdr, bevy::post_process::bloom::Bloom { intensity: 0.06, ..bevy::post_process::bloom::Bloom::NATURAL }, bevy::pbr::AtmosphereSettings::default(), ms));
            }
            Quality::High => {
                c.insert((
                    bevy::camera::Hdr,
                    bevy::post_process::bloom::Bloom { intensity: 0.07, ..bevy::post_process::bloom::Bloom::NATURAL },
                    bevy::pbr::AtmosphereSettings::default(),
                    Msaa::Off,
                    bevy::anti_alias::taa::TemporalAntiAliasing::default(),
                    bevy::pbr::ScreenSpaceAmbientOcclusion::default(),
                    bevy::anti_alias::contrast_adaptive_sharpening::ContrastAdaptiveSharpening { enabled: true, sharpening_strength: 0.4, ..default() },
                ));
            }
        }
    }
    // relief-mapped facades: off on low, occlusion mapping on medium, relief mapping on high
    for (h, d) in tiles.facades.iter().zip(&tiles.facade_depth) {
        if let Some(mut m) = materials.get_mut(h) {
            match q {
                Quality::Low => m.depth_map = None,
                Quality::Medium => {
                    m.depth_map = Some(d.clone());
                    m.max_parallax_layer_count = 12.0;
                    m.parallax_mapping_method = bevy::pbr::ParallaxMappingMethod::Occlusion;
                }
                Quality::High => {
                    m.depth_map = Some(d.clone());
                    m.max_parallax_layer_count = 24.0;
                    m.parallax_mapping_method = bevy::pbr::ParallaxMappingMethod::Relief { max_steps: 4 };
                }
            }
        }
    }
    world::areas::MAX_TREES_PER_TILE.store(trees, std::sync::atomic::Ordering::Relaxed);
    let mut g = sim.g();
    g.set_view_radius(radius);
    g.renderer_info = format!("GRAPHICS {}{} (F5)", gfx.quality.name(), if gfx.safe { ", SAFE MODE" } else { "" });
    world::log::log(&format!("graphics quality {}", gfx.quality.name()));
    gfx.applied = Some(gfx.quality);
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reads_the_quality_line() {
        assert_eq!(parse_file("# comment\nquality = High\n"), Some(Quality::High));
        assert_eq!(parse_file("quality=low # fast"), Some(Quality::Low));
        assert_eq!(parse_file("quality = ultra"), None);
        assert_eq!(parse_file(""), None);
    }
}
