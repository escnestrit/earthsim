//! BeamNG mod loading: zip + DEFLATE + lenient JBeam + slot assembly.
pub mod assemble;
pub mod beamng;
pub mod dae;
pub mod expr;
pub mod flex;
pub use scene::inflate;
pub mod lenient;
pub mod materials;
pub mod model;
pub mod props;
pub mod shape;
pub mod table;
pub mod vehicle;
pub mod visual;
pub mod zip;

use std::collections::HashMap;

/// A vehicle mod loaded and ready to drive.
pub struct LoadedVehicle {
    pub vehicle: vehicle::JbeamVehicle,
    pub main_part: String,
    pub nodes: usize,
    pub beams: usize,
    /// assembly/model warnings and vehicle notes (missing parts, defaults used)
    pub notes: Vec<String>,
    /// meshes, materials and textures (None when the mod has no usable .dae or loading them failed)
    pub visual: Option<visual::CarVisual>,
}

/// Load a BeamNG vehicle zip. `main` picks the main part; by default the first part with slotType "main".
pub fn load_vehicle(path: &std::path::Path, main: Option<&str>) -> Result<LoadedVehicle, String> {
    load_vehicle_with(path, main, beamng::find_vehicles_dir().as_deref())
}

/// Like `load_vehicle`, with BeamNG's `content/vehicles` folder given explicitly (None: mod alone).
pub fn load_vehicle_with(path: &std::path::Path, main: Option<&str>, beamng_vehicles: Option<&std::path::Path>) -> Result<LoadedVehicle, String> {
    let mut z = zip::Zip::open(path)?;
    let mut lib = assemble::Library::from_zip(&mut z);
    let mut common = None;
    let mut pre_notes = Vec::new();
    if let Some(dir) = beamng_vehicles {
        let cp = dir.join("common.zip");
        match zip::Zip::open(&cp) {
            Ok(mut cz) => {
                let t = std::time::Instant::now();
                let added = lib.add_zip_fallback(&mut cz);
                pre_notes.push(format!("BeamNG common parts: {added} parts available from {} ({:.1} s)", cp.display(), t.elapsed().as_secs_f64()));
                common = Some(cz);
            }
            Err(e) => pre_notes.push(format!("BeamNG common.zip not readable: {e}")),
        }
    } else {
        pre_notes.push("BeamNG installation not found: parts from its common.zip are missing (set EARTHSIM_BEAMNG)".into());
    }
    let main_part = resolve_main(&lib, main)?;
    let m = model::build(assemble::assemble(&lib, &main_part, &HashMap::new()));
    let mut lv = finish(m, main_part)?;
    lv.notes.splice(0..0, pre_notes);
    match visual::load(&mut z, common.as_mut(), &model::build(assemble::assemble(&lib, &lv.main_part, &HashMap::new())), &lv.vehicle) {
        Ok(v) => {
            lv.notes.push(format!("meshes: {} parts, {} triangles, {} textures", v.parts.len(), v.triangle_count(), v.textures.len()));
            lv.notes.extend(v.notes.iter().cloned());
            lv.visual = Some(v);
        }
        Err(e) => lv.notes.push(format!("meshes not loaded: {e}")),
    }
    Ok(lv)
}

fn resolve_main(lib: &assemble::Library, main: Option<&str>) -> Result<String, String> {
    match main {
        Some(m) => Ok(m.to_string()),
        None => {
            let mut mains: Vec<&String> = lib.by_type.get("main").map(|v| v.iter().collect()).unwrap_or_default();
            mains.sort();
            mains.first().map(|s| s.to_string()).ok_or_else(|| "no part with slotType \"main\" in the mod".to_string())
        }
    }
}

pub fn load_from_library(lib: &assemble::Library, main: Option<&str>) -> Result<LoadedVehicle, String> {
    let main_part = resolve_main(lib, main)?;
    finish(model::build(assemble::assemble(lib, &main_part, &HashMap::new())), main_part)
}

fn finish(m: model::Model, main_part: String) -> Result<LoadedVehicle, String> {
    if m.asm.parts.is_empty() {
        return Err(format!("main part '{main_part}' not found"));
    }
    let v = vehicle::JbeamVehicle::from_model(&m)?;
    let mut notes: Vec<String> = v.notes.clone();
    if !m.asm.missing.is_empty() {
        notes.push(format!("{} parts missing from the mod (BeamNG common files): defaults or empty slots used", m.asm.missing.len()));
    }
    if m.skipped_beams > 0 {
        notes.push(format!("{} beams skipped (their nodes are in missing parts)", m.skipped_beams));
    }
    Ok(LoadedVehicle { nodes: m.nodes.len(), beams: m.beams.len(), vehicle: v, main_part, notes, visual: None })
}
