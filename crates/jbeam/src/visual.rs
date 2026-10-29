//! Everything needed to draw a vehicle mod: flexbody meshes bound to the nodes, materials and texture bytes.
use crate::dae::Dae;
use crate::flex::{self, FlexPart};
use crate::materials::{self, FileIndex, MatDef};
use crate::model::Model;
use crate::vehicle::JbeamVehicle;
use crate::zip::Zip;
use std::collections::HashMap;
use std::io::{Read, Seek};

pub struct CarVisual {
    pub parts: Vec<FlexPart>,
    pub materials: HashMap<String, MatDef>,
    /// texture bytes by zip entry name (only those the materials use)
    pub textures: HashMap<String, Vec<u8>>,
    pub notes: Vec<String>,
}

impl CarVisual {
    pub fn vertex_count(&self) -> usize {
        self.parts.iter().flat_map(|p| p.submeshes.iter()).map(|s| s.positions.len()).sum()
    }
    pub fn triangle_count(&self) -> usize {
        self.parts.iter().flat_map(|p| p.submeshes.iter()).map(|s| s.indices.len() / 3).sum()
    }
}

pub fn load<R: Read + Seek>(z: &mut Zip<R>, m: &Model, v: &JbeamVehicle) -> Result<CarVisual, String> {
    let entries = z.entries.clone();
    let mut daes = Vec::new();
    for e in entries.iter().filter(|e| e.name.to_lowercase().ends_with(".dae")) {
        let bytes = z.read(e)?;
        daes.push(Dae::parse(String::from_utf8_lossy(&bytes).into_owned())?);
    }
    if daes.is_empty() {
        return Err("no .dae meshes in the mod".into());
    }
    let (parts, mut notes) = flex::build(m, v, &daes);
    drop(daes);
    let files = FileIndex::new(entries.iter().map(|e| e.name.as_str()));
    let mut mats = HashMap::new();
    for e in entries.iter().filter(|e| e.name.to_lowercase().ends_with("materials.json")) {
        let text = String::from_utf8_lossy(&z.read(e)?).into_owned();
        if let Err(err) = materials::parse_file(&text, &files, &mut mats) {
            notes.push(format!("{}: {err}", e.name));
        }
    }
    // textures of the materials actually used
    let mut textures = HashMap::new();
    let mut missing_mats = 0;
    let used: std::collections::HashSet<&str> = parts.iter().flat_map(|p| p.submeshes.iter().map(|s| s.material.as_str())).collect();
    for name in &used {
        match mats.get(*name) {
            Some(md) => {
                // normal maps are not used by the renderer yet: only base colour textures are read
                for t in [&md.base_map].into_iter().flatten() {
                    if !textures.contains_key(t) {
                        if let Some(e) = entries.iter().find(|e| &e.name == t) {
                            textures.insert(t.clone(), z.read(e)?);
                        }
                    }
                }
            }
            None => missing_mats += 1,
        }
    }
    if missing_mats > 0 {
        notes.push(format!("{missing_mats} materials not defined in the mod (plain grey)"));
    }
    Ok(CarVisual { parts, materials: mats, textures, notes })
}
