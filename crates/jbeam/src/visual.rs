//! Everything needed to draw a vehicle mod: flexbody meshes bound to the nodes, materials and texture bytes.
//! Meshes, materials and textures are looked up in the mod first, then in BeamNG's own `common.zip` when the
//! player's BeamNG installation was found (parts such as tyres live there).
use crate::dae::Dae;
use crate::flex::{self, FlexPart, MeshFile};
use crate::materials::{self, FileIndex, MatDef};
use crate::model::Model;
use crate::vehicle::JbeamVehicle;
use crate::zip::Zip;
use std::collections::{HashMap, HashSet};
use std::io::{Read, Seek};

pub struct CarVisual {
    pub parts: Vec<FlexPart>,
    pub materials: HashMap<String, MatDef>,
    /// texture bytes by (prefixed) entry name (only those the materials use)
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

fn parse_mesh_file<R: Read + Seek>(z: &mut Zip<R>, e: &crate::zip::Entry) -> Result<MeshFile, String> {
    let l = e.name.to_lowercase();
    let bytes = z.read(e)?;
    if l.ends_with(".cdae") || l.ends_with(".dts") {
        crate::shape::parse(&bytes).map(MeshFile::Shape)
    } else {
        Dae::parse(String::from_utf8_lossy(&bytes).into_owned()).map(MeshFile::Dae)
    }
}

fn is_mesh(l: &str) -> bool {
    l.ends_with(".dae") || l.ends_with(".cdae") || l.ends_with(".dts")
}

/// Texture key: entry name for the mod, "common:" + entry for BeamNG's common.zip.
const COMMON: &str = "common:";

pub fn load<R: Read + Seek>(z: &mut Zip<R>, common: Option<&mut Zip<std::fs::File>>, m: &Model, v: &JbeamVehicle) -> Result<CarVisual, String> {
    let entries = z.entries.clone();
    let mut notes = Vec::new();
    // mesh files of the mod: a binary shape (.cdae) wins over the .dae of the same name, as in BeamNG
    let mut files = Vec::new();
    let lower: Vec<String> = entries.iter().map(|e| e.name.to_lowercase()).collect();
    let lower_set: HashSet<&str> = lower.iter().map(String::as_str).collect();
    for (e, l) in entries.iter().zip(&lower) {
        if !is_mesh(l) || (l.ends_with(".dae") && lower_set.contains(format!("{}.cdae", &l[..l.len() - 4]).as_str())) {
            continue;
        }
        match parse_mesh_file(z, e) {
            Ok(f) => files.push(f),
            Err(err) => notes.push(format!("{}: {err}", e.name)),
        }
    }
    // meshes the mod lacks: search BeamNG's common.zip, reading only files whose name matches the mesh
    let mut common = common;
    if let Some(cz) = common.as_deref_mut() {
        let needed: Vec<String> = m.table("flexbodies").iter().filter_map(|(_, r)| r.str("mesh").map(String::from)).filter(|n| !files.iter().any(|f| f.has(n))).collect();
        let centries: Vec<crate::zip::Entry> = cz.entries.iter().filter(|e| is_mesh(&e.name.to_lowercase())).cloned().collect();
        let mut tried: HashSet<String> = HashSet::new();
        let mut found = 0;
        for name in &needed {
            if files.iter().any(|f| f.has(name)) {
                found += 1;
                continue;
            }
            // candidate files: their stem shares the first word(s) of the mesh name (tire_01a_..., steelwheel_16a_...)
            let n = name.to_lowercase();
            let toks: Vec<&str> = n.split('_').collect();
            let key2 = toks.iter().take(2).copied().collect::<Vec<_>>().join("_");
            let key1 = toks.first().copied().unwrap_or("");
            let mut cands: Vec<&crate::zip::Entry> = centries.iter().filter(|e| {
                let stem = e.name.rsplit('/').next().unwrap_or("").to_lowercase();
                let stem = stem.split('.').next().unwrap_or("");
                !stem.is_empty() && (stem.starts_with(&key2) || key2.starts_with(stem) || (key1.len() >= 4 && stem.starts_with(key1)))
            }).collect();
            cands.sort_by_key(|e| e.size);
            for e in cands.into_iter().take(6) {
                if !tried.insert(e.name.clone()) {
                    continue;
                }
                match parse_mesh_file(cz, e) {
                    Ok(f) => files.push(f),
                    Err(err) => notes.push(format!("common {}: {err}", e.name)),
                }
                if files.iter().any(|f| f.has(name)) {
                    found += 1;
                    break;
                }
            }
        }
        if !needed.is_empty() {
            notes.push(format!("{found} of {} missing meshes found in BeamNG's common.zip", needed.len()));
        }
    }
    if files.is_empty() {
        return Err("no meshes (.dae/.cdae/.dts) in the mod".into());
    }
    let (parts, more) = flex::build(m, v, &files);
    notes.extend(more);
    drop(files);
    // materials: the mod's, then common.zip's for names the mod does not define
    let index = FileIndex::new(entries.iter().map(|e| e.name.as_str()));
    let mut mats = HashMap::new();
    for e in entries.iter().filter(|e| e.name.to_lowercase().ends_with("materials.json")) {
        let text = String::from_utf8_lossy(&z.read(e)?).into_owned();
        if let Err(err) = materials::parse_file(&text, &index, &mut mats) {
            notes.push(format!("{}: {err}", e.name));
        }
    }
    // materials drawn now, plus the damaged ones that deform groups switch to
    let used: HashSet<String> = parts
        .iter()
        .flat_map(|p| p.submeshes.iter().map(|s| s.material.clone()).chain(p.swap.iter().map(|w| w.damaged.clone())))
        .filter(|m| !flex::is_invisible_material(m))
        .collect();
    let swaps = parts.iter().filter(|p| p.swap.is_some()).count();
    if swaps > 0 {
        notes.push(format!("{swaps} meshes change material when damaged (glass cracks, lights)"));
    }
    if let Some(cz) = common.as_deref_mut() {
        if used.iter().any(|u| !mats.contains_key(u)) {
            let cindex = FileIndex::new(cz.entries.iter().map(|e| e.name.as_str()));
            let mut cmats = HashMap::new();
            let mfiles: Vec<crate::zip::Entry> = cz.entries.iter().filter(|e| e.name.to_lowercase().ends_with("materials.json")).cloned().collect();
            for e in mfiles {
                let text = String::from_utf8_lossy(&cz.read(&e)?).into_owned();
                let _ = materials::parse_file(&text, &cindex, &mut cmats);
            }
            for (k, mut md) in cmats {
                if used.contains(&k) && !mats.contains_key(&k) {
                    md.base_map = md.base_map.map(|t| format!("{COMMON}{t}"));
                    md.normal_map = md.normal_map.map(|t| format!("{COMMON}{t}"));
                    mats.insert(k, md);
                }
            }
        }
    }
    // textures of the materials actually used
    let mut textures = HashMap::new();
    let mut missing_mats = 0;
    for name in &used {
        match mats.get(name) {
            Some(md) => {
                // normal maps are not used by the renderer yet: only base colour textures are read
                for t in [&md.base_map].into_iter().flatten() {
                    if textures.contains_key(t) {
                        continue;
                    }
                    if let Some(ct) = t.strip_prefix(COMMON) {
                        if let Some(cz) = common.as_deref_mut() {
                            if let Some(e) = cz.entries.iter().find(|e| e.name == ct).cloned() {
                                textures.insert(t.clone(), cz.read(&e)?);
                            }
                        }
                    } else if let Some(e) = entries.iter().find(|e| &e.name == t) {
                        textures.insert(t.clone(), z.read(e)?);
                    }
                }
            }
            None => missing_mats += 1,
        }
    }
    if missing_mats > 0 {
        notes.push(format!("{missing_mats} materials not defined (plain grey)"));
    }
    Ok(CarVisual { parts, materials: mats, textures, notes })
}
