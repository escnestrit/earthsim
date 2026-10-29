//! Part library + slot assembly: starting from the main part, follow `slots`/`slots2` defaults down the tree.

use crate::lenient;
use crate::table::{resolve, rows};
use crate::zip::Zip;
use scene::json::Json;
use std::collections::{BTreeMap, HashMap};
use std::io::{Read, Seek};

pub struct PartDef {
    pub file: String,
    pub body: Json,
}

#[derive(Default)]
pub struct Library {
    pub parts: HashMap<String, PartDef>,
    pub by_type: BTreeMap<String, Vec<String>>,
    pub warnings: Vec<String>,
}

impl Library {
    pub fn from_zip<R: Read + Seek>(z: &mut Zip<R>) -> Library {
        let mut lib = Library::default();
        let names: Vec<_> = z.entries.iter().filter(|e| e.name.ends_with(".jbeam")).cloned().collect();
        for e in names {
            match z.read(&e) {
                Ok(data) => lib.add_file(&e.name, &String::from_utf8_lossy(&data)),
                Err(err) => lib.warnings.push(format!("{}: {err}", e.name)),
            }
        }
        lib
    }

    pub fn add_file(&mut self, file: &str, text: &str) {
        if text.trim().is_empty() {
            self.warnings.push(format!("{file}: empty file"));
            return;
        }
        match lenient::parse(text) {
            Ok(Json::Obj(parts)) => {
                for (name, body) in parts {
                    let ty = body.get("slotType").and_then(|v| v.as_str()).unwrap_or("").to_string();
                    if self.parts.contains_key(&name) {
                        self.warnings.push(format!("{file}: duplicate part '{name}' ignored"));
                        continue;
                    }
                    self.by_type.entry(ty).or_default().push(name.clone());
                    self.parts.insert(name, PartDef { file: file.to_string(), body });
                }
            }
            Ok(_) => self.warnings.push(format!("{file}: top level is not an object")),
            Err(e) => self.warnings.push(format!("{file}: {e}")),
        }
    }
}

pub struct PartInstance {
    pub name: String,
    pub slot: String,
    /// accumulated `nodeOffset` of all parent slots (JBeam axes). Its x is mirrored: applied away from the
    /// centre line (left nodes move left, right nodes right), which is how wheel slots set the track width.
    pub offset: [f64; 3],
    /// accumulated `nodeMove` (a plain translation)
    pub node_move: [f64; 3],
    /// the part body with variables substituted
    pub body: Json,
}

#[derive(Default)]
pub struct Assembly {
    pub parts: Vec<PartInstance>,
    pub vars: HashMap<String, Json>,
    pub warnings: Vec<String>,
    /// slots whose default part does not exist in the library: (slot type, wanted part)
    pub missing: Vec<(String, String)>,
}

struct SlotRef {
    ty: String,
    allow: Vec<String>,
    default: String,
    opts: Option<Json>,
}

fn slots_of(body: &Json) -> Vec<SlotRef> {
    let mut out = Vec::new();
    if let Some(s) = body.get("slots") {
        for r in rows(s) {
            let ty = r.str("type").unwrap_or("").to_string();
            let default = r.str("default").unwrap_or("").to_string();
            let opts = r.get("nodeOffset").map(|v| ("nodeOffset", v)).into_iter().chain(r.get("nodeMove").map(|v| ("nodeMove", v))).fold(Vec::new(), |mut a, (k, v)| {
                a.push((k.to_string(), v.clone()));
                a
            });
            out.push(SlotRef { allow: vec![ty.clone()], ty, default, opts: if opts.is_empty() { None } else { Some(Json::Obj(opts)) } });
        }
    }
    if let Some(s) = body.get("slots2") {
        for r in rows(s) {
            let name = r.str("name").unwrap_or("").to_string();
            let allow = r.strings("allowTypes");
            let default = r.str("default").unwrap_or("").to_string();
            let opts: Vec<(String, Json)> = ["nodeOffset", "nodeMove"].iter().filter_map(|k| r.get(k).map(|v| (k.to_string(), v.clone()))).collect();
            out.push(SlotRef { ty: name, allow, default, opts: if opts.is_empty() { None } else { Some(Json::Obj(opts)) } });
        }
    }
    out
}

fn vec3_of(j: &Json) -> [f64; 3] {
    let g = |k: &str| j.get(k).and_then(|v| v.as_f64()).unwrap_or(0.0);
    [g("x"), g("y"), g("z")]
}

/// `overrides` maps slot type -> part name ("" leaves the slot empty).
pub fn assemble(lib: &Library, root: &str, overrides: &HashMap<String, String>) -> Assembly {
    let mut asm = Assembly::default();
    if !lib.parts.contains_key(root) {
        asm.warnings.push(format!("main part '{root}' not found"));
        return asm;
    }
    // pass 1: choose parts (raw bodies), pass 2: variables, pass 3: resolve
    type Off = ([f64; 3], [f64; 3]);
    let mut chosen: Vec<(String, String, Off)> = Vec::new();
    fn walk(lib: &Library, name: &str, slot: &str, off: ([f64; 3], [f64; 3]), depth: usize, ov: &HashMap<String, String>, asm: &mut Assembly, chosen: &mut Vec<(String, String, ([f64; 3], [f64; 3]))>) {
        if depth > 40 {
            asm.warnings.push(format!("slot tree too deep at '{name}'"));
            return;
        }
        chosen.push((name.to_string(), slot.to_string(), off));
        for s in slots_of(&lib.parts[name].body) {
            let want = ov.get(&s.ty).cloned().unwrap_or_else(|| s.default.clone());
            if want.is_empty() {
                continue;
            }
            let mut pick = want.clone();
            if !lib.parts.contains_key(&pick) {
                asm.missing.push((s.ty.clone(), want.clone()));
                // fall back to any part that fits the slot
                let alt = s.allow.iter().filter_map(|t| lib.by_type.get(t)).flatten().next();
                match alt {
                    Some(a) => {
                        asm.warnings.push(format!("slot '{}': part '{want}' missing, using '{a}'", s.ty));
                        pick = a.clone();
                    }
                    None => {
                        asm.warnings.push(format!("slot '{}': part '{want}' missing, slot left empty", s.ty));
                        continue;
                    }
                }
            }
            let mut o = off;
            if let Some(op) = &s.opts {
                if let Some(v) = op.get("nodeOffset") {
                    let d = vec3_of(v);
                    for i in 0..3 {
                        o.0[i] += d[i];
                    }
                }
                if let Some(v) = op.get("nodeMove") {
                    let d = vec3_of(v);
                    for i in 0..3 {
                        o.1[i] += d[i];
                    }
                }
            }
            walk(lib, &pick, &s.ty, o, depth + 1, ov, asm, chosen);
        }
    }
    // Variables first (defaults of every part in the library that is reachable); slot offsets may use them,
    // so collect from the whole library: names are globally unique in practice.
    for p in lib.parts.values() {
        if let Some(v) = p.body.get("variables") {
            for r in rows(v) {
                if let (Some(n), Some(d)) = (r.str("name"), r.get("default")) {
                    asm.vars.entry(n.to_string()).or_insert_with(|| d.clone());
                }
            }
        }
    }
    // Resolve slot offsets with variables by resolving each part's `slots` lazily: cheapest is to resolve
    // the whole body before walking.
    let mut resolved: HashMap<String, Json> = HashMap::new();
    let mut warn = Vec::new();
    for (n, p) in &lib.parts {
        resolved.insert(n.clone(), resolve(&p.body, &asm.vars, &mut warn));
    }
    asm.warnings.extend(warn);
    let lib2 = Library { parts: resolved.into_iter().map(|(n, b)| { let file = lib.parts[&n].file.clone(); (n, PartDef { file, body: b }) }).collect(), by_type: lib.by_type.clone(), warnings: Vec::new() };
    walk(&lib2, root, "", ([0.0; 3], [0.0; 3]), 0, overrides, &mut asm, &mut chosen);
    for (name, slot, (offset, node_move)) in chosen {
        let body = lib2.parts[&name].body.clone();
        asm.parts.push(PartInstance { name, slot, offset, node_move, body });
    }
    asm
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lib() -> Library {
        let mut l = Library::default();
        l.add_file(
            "a.jbeam",
            r#"{
            "car": {"slotType":"main","slots":[["type","default","description"],["body","body_a","Body"],["wheel","wheel_a","W",{"nodeOffset":{"x":"$=$track+1","y":0.5,"z":0}}],["ghost","ghost_a","missing"],["opt","","none"]],
                    "variables":[["name","type","unit","category","default"],["$track","range","","",0.25]]},
            "body_a": {"slotType":"body","slots2":[["name","allowTypes","denyTypes","default","description"],["door",["door"],[],"door_a","Door",{"nodeMove":{"x":0,"y":0,"z":0.1}}]]},
            "door_a": {"slotType":"door"},
            "door_b": {"slotType":"door"},
            "wheel_a": {"slotType":"wheel"}
        }"#,
        );
        l
    }

    #[test]
    fn follows_defaults_and_offsets() {
        let a = assemble(&lib(), "car", &HashMap::new());
        let names: Vec<_> = a.parts.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(names, ["car", "body_a", "door_a", "wheel_a"]);
        assert_eq!(a.parts[3].offset, [1.25, 0.5, 0.0]);
        assert_eq!(a.parts[2].offset, [0.0, 0.0, 0.0]);
        assert_eq!(a.parts[2].node_move, [0.0, 0.0, 0.1]);
        assert_eq!(a.missing, vec![("ghost".to_string(), "ghost_a".to_string())]);
    }

    #[test]
    fn overrides_and_fallback() {
        let mut ov = HashMap::new();
        ov.insert("door".to_string(), "door_b".to_string());
        ov.insert("wheel".to_string(), String::new());
        let a = assemble(&lib(), "car", &ov);
        let names: Vec<_> = a.parts.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(names, ["car", "body_a", "door_b"]);
        let a = assemble(&lib(), "nope", &HashMap::new());
        assert!(a.parts.is_empty() && !a.warnings.is_empty());
    }

    #[test]
    fn missing_part_falls_back_to_same_type() {
        let mut l = lib();
        l.add_file("b.jbeam", r#"{"ghost_b":{"slotType":"ghost"}}"#);
        let a = assemble(&l, "car", &HashMap::new());
        assert!(a.parts.iter().any(|p| p.name == "ghost_b"));
    }
}
