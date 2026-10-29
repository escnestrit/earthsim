//! Finding the user's BeamNG.drive installation (for its `content/vehicles/common.zip`).
use std::path::{Path, PathBuf};

/// Library folders listed in Steam's `libraryfolders.vdf` ("path" entries; `\\` unescaped).
pub fn steam_libraries(vdf: &str) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for line in vdf.lines() {
        let t = line.trim();
        if !t.starts_with("\"path\"") {
            continue;
        }
        let rest = t["\"path\"".len()..].trim();
        if let Some(s) = rest.strip_prefix('"').and_then(|r| r.rfind('"').map(|e| &r[..e])) {
            out.push(PathBuf::from(s.replace("\\\\", "\\")));
        }
    }
    out
}

fn vehicles_in(install: &Path) -> Option<PathBuf> {
    let v = install.join("content").join("vehicles");
    v.join("common.zip").is_file().then_some(v)
}

/// BeamNG's `content/vehicles` folder: `EARTHSIM_BEAMNG` (install dir or the vehicles dir itself),
/// else every Steam library (default Steam locations + their libraryfolders.vdf).
pub fn find_vehicles_dir() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("EARTHSIM_BEAMNG") {
        let p = PathBuf::from(p);
        if p.join("common.zip").is_file() {
            return Some(p);
        }
        if let Some(v) = vehicles_in(&p) {
            return Some(v);
        }
    }
    let mut steams: Vec<PathBuf> = vec![
        PathBuf::from(r"C:\Program Files (x86)\Steam"),
        PathBuf::from(r"C:\Program Files\Steam"),
    ];
    for d in ['C', 'D', 'E', 'F', 'G'] {
        steams.push(PathBuf::from(format!(r"{d}:\Steam")));
        steams.push(PathBuf::from(format!(r"{d}:\steam")));
        steams.push(PathBuf::from(format!(r"{d}:\SteamLibrary")));
        steams.push(PathBuf::from(format!(r"{d}:\Games\Steam")));
    }
    if let Ok(h) = std::env::var("HOME") {
        steams.push(PathBuf::from(h).join(".steam/steam"));
    }
    let mut libs: Vec<PathBuf> = Vec::new();
    for s in &steams {
        libs.push(s.clone());
        if let Ok(t) = std::fs::read_to_string(s.join("steamapps").join("libraryfolders.vdf")) {
            libs.extend(steam_libraries(&t));
        }
    }
    for l in libs {
        if let Some(v) = vehicles_in(&l.join("steamapps").join("common").join("BeamNG.drive")) {
            return Some(v);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reads_steam_library_paths() {
        let vdf = "\"libraryfolders\"\n{\n\t\"0\"\n\t{\n\t\t\"path\"\t\t\"C:\\\\Program Files (x86)\\\\Steam\"\n\t}\n\t\"1\"\n\t{\n\t\t\"path\"\t\t\"D:\\\\steam\"\n\t}\n}";
        let l = steam_libraries(vdf);
        assert_eq!(l, vec![PathBuf::from(r"C:\Program Files (x86)\Steam"), PathBuf::from(r"D:\steam")]);
    }
    #[test]
    fn env_var_points_at_vehicles_dir() {
        let d = std::env::temp_dir().join("earthsim_bng_test/content/vehicles");
        std::fs::create_dir_all(&d).unwrap();
        std::fs::write(d.join("common.zip"), b"x").unwrap();
        std::env::set_var("EARTHSIM_BEAMNG", d.parent().unwrap().parent().unwrap());
        assert_eq!(find_vehicles_dir(), Some(d));
    }
}
