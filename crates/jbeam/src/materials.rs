//! BeamNG `*.materials.json`: per material the base colour (factor and/or texture), normal map, roughness,
//! metallic and transparency, with texture paths resolved against the files actually in the mod zip
//! (BeamNG accepts a path whose extension differs from the file, e.g. `.png` in the JSON and `.dds` in the zip).

use crate::lenient;
use scene::json::Json;
use std::collections::HashMap;

#[derive(Clone, Debug, PartialEq)]
pub struct MatDef {
    pub name: String,
    pub base_color: [f32; 4],
    /// zip entry names
    pub base_map: Option<String>,
    pub normal_map: Option<String>,
    pub roughness: f32,
    pub metallic: f32,
    pub translucent: bool,
}

/// Index of zip entries by lower-case path, and by lower-case path without extension.
pub struct FileIndex {
    exact: HashMap<String, String>,
    stem: HashMap<String, Vec<String>>,
}

impl FileIndex {
    pub fn new<'a>(names: impl Iterator<Item = &'a str>) -> FileIndex {
        let mut exact = HashMap::new();
        let mut stem: HashMap<String, Vec<String>> = HashMap::new();
        for n in names {
            let l = n.to_lowercase();
            exact.insert(l.clone(), n.to_string());
            let s = match l.rfind('.') {
                Some(i) if !l[i..].contains('/') => l[..i].to_string(),
                _ => l.clone(),
            };
            stem.entry(s).or_default().push(n.to_string());
        }
        FileIndex { exact, stem }
    }

    /// Resolve a texture path from a material file. Prefers DDS, then PNG, then JPG when only the stem matches.
    pub fn resolve(&self, path: &str) -> Option<String> {
        let p = path.trim().trim_start_matches('/').replace('\\', "/").to_lowercase();
        if p.is_empty() {
            return None;
        }
        if let Some(e) = self.exact.get(&p) {
            return Some(e.clone());
        }
        let mut stems = vec![p.clone()];
        if let Some(i) = p.rfind('.') {
            if !p[i..].contains('/') {
                stems.push(p[..i].to_string());
            }
        }
        for s in stems {
            if let Some(c) = self.stem.get(&s) {
                let rank = |n: &String| {
                    let l = n.to_lowercase();
                    if l.ends_with(".dds") { 0 } else if l.ends_with(".png") { 1 } else if l.ends_with(".jpg") || l.ends_with(".jpeg") { 2 } else { 9 }
                };
                if let Some(best) = c.iter().filter(|n| rank(n) < 9).min_by_key(|n| rank(n)) {
                    return Some(best.clone());
                }
            }
        }
        None
    }
}

fn f4(j: Option<&Json>) -> Option<[f32; 4]> {
    let a = j?.as_arr()?;
    let v: Vec<f32> = a.iter().filter_map(|x| x.as_f64().map(|f| f as f32)).collect();
    match v.len() {
        3 => Some([v[0], v[1], v[2], 1.0]),
        n if n >= 4 => Some([v[0], v[1], v[2], v[3]]),
        _ => None,
    }
}

/// Parse one materials file into `out` (keyed by `mapTo`, falling back to the entry name).
pub fn parse_file(text: &str, files: &FileIndex, out: &mut HashMap<String, MatDef>) -> Result<(), String> {
    let j = lenient::parse(text)?;
    let Json::Obj(entries) = j else { return Err("not an object".into()) };
    for (key, m) in entries {
        if !matches!(m, Json::Obj(_)) {
            continue;
        }
        let name = m.get("mapTo").and_then(|v| v.as_str()).unwrap_or(&key).to_string();
        let stage = m.get("Stages").and_then(|s| s.as_arr()).and_then(|a| a.first()).cloned().unwrap_or(Json::Obj(Vec::new()));
        let s = |k: &str| stage.get(k).and_then(|v| v.as_str()).and_then(|p| files.resolve(p));
        let n = |k: &str| stage.get(k).and_then(|v| v.as_f64()).map(|x| x as f32);
        let base_color = f4(stage.get("baseColorFactor")).or(f4(stage.get("diffuseColor"))).unwrap_or([1.0; 4]);
        let def = MatDef {
            name: name.clone(),
            base_color,
            base_map: s("baseColorMap").or(s("colorMap")).or(s("diffuseMap")),
            normal_map: s("normalMap"),
            roughness: n("roughnessFactor").unwrap_or(if stage.get("specularPower").is_some() { 0.4 } else { 0.6 }),
            metallic: n("metallicFactor").unwrap_or(0.0),
            translucent: m.get("translucent").and_then(|v| v.as_bool()).unwrap_or(false) || n("opacityFactor").is_some_and(|o| o > 0.0 && o < 1.0),
        };
        out.insert(name, def);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolves_textures_and_reads_factors() {
        let files = FileIndex::new(["vehicles/car/body_b.color.DDS", "vehicles/car/body_b.color.png", "vehicles/car/glass.png", "vehicles/car/whe.dds"].into_iter());
        assert_eq!(files.resolve("/vehicles/car/body_b.color.png").as_deref(), Some("vehicles/car/body_b.color.png"));
        assert_eq!(files.resolve("vehicles/car/glass.dds").as_deref(), Some("vehicles/car/glass.png"));
        assert_eq!(files.resolve("/vehicles/car/whe").as_deref(), Some("vehicles/car/whe.dds"));
        assert_eq!(files.resolve("vehicles/other/x.png"), None);
        let text = r#"{
          "car_body": {"name":"car_body","mapTo":"car_body","Stages":[{"baseColorFactor":[0.5,0.4,0.3,1],"baseColorMap":"/vehicles/car/body_b.color.png","metallicFactor":1,"roughnessFactor":0.2},{},{},{}]},
          "car_glass": {"mapTo":"car_glass","translucent":true,"Stages":[{"colorMap":"vehicles/car/glass.dds"}]},
        }"#;
        let mut out = HashMap::new();
        parse_file(text, &files, &mut out).unwrap();
        let b = &out["car_body"];
        assert_eq!(b.base_color, [0.5, 0.4, 0.3, 1.0]);
        assert_eq!((b.metallic, b.roughness), (1.0, 0.2));
        assert_eq!(b.base_map.as_deref(), Some("vehicles/car/body_b.color.png"));
        assert!(out["car_glass"].translucent);
        assert_eq!(out["car_glass"].base_map.as_deref(), Some("vehicles/car/glass.png"));
    }
}
