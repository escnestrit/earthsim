//! Command line: `earthsim [--lat N --lon N] [--demo] [--offline] [--scene file.json] [--cache dir]`.
//! Default: stream the real map (OpenStreetMap through Overpass) around Minsk.
#[derive(Clone, Debug, PartialEq)]
pub enum Mode {
    /// Stream live from Overpass.
    Live,
    /// Stream from the bundled sample dataset instead of the network (tests the whole pipeline offline).
    Offline,
    /// The built-in synthetic city, no streaming.
    Demo,
    /// A scene.json made by the Python tool, no streaming.
    SceneFile(String),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Options {
    pub mode: Mode,
    pub lat: f64,
    pub lon: f64,
    pub cache: Option<String>,
    /// BeamNG vehicle mod (zip) to drive instead of the built-in car, and optionally its main part
    pub car: Option<String>,
    pub car_part: Option<String>,
}

pub const MINSK: (f64, f64) = (53.9045, 27.5615);

pub fn parse<I: IntoIterator<Item = String>>(args: I) -> Result<Options, String> {
    let mut o = Options { mode: Mode::Live, lat: MINSK.0, lon: MINSK.1, cache: None, car: None, car_part: None };
    let mut it = args.into_iter();
    let num = |name: &str, v: Option<String>, lo: f64, hi: f64| -> Result<f64, String> {
        let v = v.ok_or_else(|| format!("{name} needs a value"))?;
        let x: f64 = v.trim().replace(',', ".").parse().map_err(|_| format!("{name}: '{v}' is not a number"))?;
        if !(lo..=hi).contains(&x) {
            return Err(format!("{name}: {x} is outside {lo}..{hi}"));
        }
        Ok(x)
    };
    while let Some(a) = it.next() {
        match a.as_str() {
            "--lat" => o.lat = num("--lat", it.next(), -85.0, 85.0)?,
            "--lon" => o.lon = num("--lon", it.next(), -180.0, 180.0)?,
            "--demo" => o.mode = Mode::Demo,
            "--offline" => o.mode = Mode::Offline,
            "--scene" => o.mode = Mode::SceneFile(it.next().ok_or("--scene needs a file")?),
            "--cache" => o.cache = Some(it.next().ok_or("--cache needs a directory")?),
            "--car" => o.car = Some(it.next().ok_or("--car needs a BeamNG vehicle zip")?),
            "--car-part" => o.car_part = Some(it.next().ok_or("--car-part needs a part name")?),
            x if x.to_ascii_lowercase().ends_with(".zip") => o.car = Some(x.to_string()),
            "--help" | "-h" => return Err(USAGE.to_string()),
            x if x.ends_with(".json") => o.mode = Mode::SceneFile(x.to_string()),
            x => return Err(format!("unknown argument '{x}'\n{USAGE}")),
        }
    }
    Ok(o)
}

pub const USAGE: &str = "earthsim [--lat N --lon N] [--offline] [--demo] [--scene file.json] [--cache dir] [--car mod.zip [--car-part name]]\n  default: stream OpenStreetMap data live around Minsk (needs internet)\n  --offline: stream the bundled sample dataset (no network)\n  --demo: built-in synthetic city\n  env: EARTHSIM_OVERPASS (own Overpass endpoint), EARTHSIM_CACHE (cache directory)";

#[cfg(test)]
mod tests {
    use super::*;
    fn p(a: &[&str]) -> Result<Options, String> {
        parse(a.iter().map(|s| s.to_string()))
    }
    #[test]
    fn defaults_to_live_minsk() {
        let o = p(&[]).unwrap();
        assert_eq!((o.mode, o.lat, o.lon), (Mode::Live, MINSK.0, MINSK.1));
    }
    #[test]
    fn parses_coordinates_and_modes() {
        let o = p(&["--lat", "48,8584", "--lon", "2.2945", "--offline"]).unwrap();
        assert_eq!((o.lat, o.lon, o.mode), (48.8584, 2.2945, Mode::Offline));
        assert_eq!(p(&["scene.json"]).unwrap().mode, Mode::SceneFile("scene.json".into()));
        assert_eq!(p(&["--cache", "d"]).unwrap().cache.as_deref(), Some("d"));
        let o = p(&["--car", "rav4.zip", "--car-part", "ravxa20"]).unwrap();
        assert_eq!((o.car.as_deref(), o.car_part.as_deref()), (Some("rav4.zip"), Some("ravxa20")));
        assert_eq!(p(&["C:\\mods\\Car.ZIP"]).unwrap().car.as_deref(), Some("C:\\mods\\Car.ZIP"));
    }
    #[test]
    fn rejects_garbage() {
        assert!(p(&["--lat"]).is_err());
        assert!(p(&["--lat", "abc"]).is_err());
        assert!(p(&["--lat", "99"]).is_err());
        assert!(p(&["--wat"]).is_err());
    }
}
