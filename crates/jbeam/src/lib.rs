//! BeamNG mod loading: zip + DEFLATE + lenient JBeam + slot assembly.
pub mod assemble;
pub mod expr;
pub use scene::inflate;
pub mod lenient;
pub mod model;
pub mod table;
pub mod vehicle;
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
}

/// Load a BeamNG vehicle zip. `main` picks the main part; by default the first part with slotType "main".
pub fn load_vehicle(path: &std::path::Path, main: Option<&str>) -> Result<LoadedVehicle, String> {
    let mut z = zip::Zip::open(path)?;
    let lib = assemble::Library::from_zip(&mut z);
    load_from_library(&lib, main)
}

pub fn load_from_library(lib: &assemble::Library, main: Option<&str>) -> Result<LoadedVehicle, String> {
    let main_part = match main {
        Some(m) => m.to_string(),
        None => {
            let mut mains: Vec<&String> = lib.by_type.get("main").map(|v| v.iter().collect()).unwrap_or_default();
            mains.sort();
            mains.first().map(|s| s.to_string()).ok_or("no part with slotType \"main\" in the mod")?
        }
    };
    let asm = assemble::assemble(lib, &main_part, &HashMap::new());
    if asm.parts.is_empty() {
        return Err(format!("main part '{main_part}' not found"));
    }
    let m = model::build(asm);
    let v = vehicle::JbeamVehicle::from_model(&m)?;
    let mut notes: Vec<String> = v.notes.clone();
    if !m.asm.missing.is_empty() {
        notes.push(format!("{} parts missing from the mod (BeamNG common files): defaults or empty slots used", m.asm.missing.len()));
    }
    if m.skipped_beams > 0 {
        notes.push(format!("{} beams skipped (their nodes are in missing parts)", m.skipped_beams));
    }
    Ok(LoadedVehicle { nodes: m.nodes.len(), beams: m.beams.len(), vehicle: v, main_part, notes })
}
