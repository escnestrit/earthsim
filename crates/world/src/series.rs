//! Catalogue of typical building designs (standard series and common building types) and the rules that
//! match a mapped building to one. A design sets the facade (material, window layout, colours, storey
//! height) and the exterior elements (balconies/loggias, stairwell windows, entrance canopies, plinth,
//! parapet, technical floor, shops).
//!
//! Matching: the OSM `building:series` tag when present (common on Russian/Belarusian housing), otherwise the
//! region (from latitude/longitude), storeys, corpus depth, year (`start_date`) and building class. The
//! values are typical for each design from public descriptions of the series; they are approximations,
//! not drawings of the real building.
use scene::model::{Balconies, Canopy, Design, StairWindows};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Region {
    Any,
    /// former USSR
    ExUssr,
    Belarus,
    /// East Germany
    Gdr,
    /// Poland, Czechia, Slovakia, Hungary, Romania, Bulgaria
    EasternEurope,
    France,
    Uk,
    WesternEurope,
    NorthAmerica,
    Japan,
    China,
}

pub fn regions_at(lat: f64, lon: f64) -> Vec<Region> {
    let inb = |la0: f64, la1: f64, lo0: f64, lo1: f64| lat >= la0 && lat <= la1 && lon >= lo0 && lon <= lo1;
    let mut r = Vec::new();
    if inb(51.2, 56.2, 23.1, 32.8) {
        r.push(Region::Belarus);
    }
    if inb(41.0, 70.0, 21.0, 60.0) || inb(40.0, 55.0, 46.0, 87.0) || inb(50.0, 72.0, 60.0, 180.0) {
        r.push(Region::ExUssr);
    }
    if inb(50.2, 54.7, 10.5, 15.1) {
        r.push(Region::Gdr);
    }
    if inb(41.0, 55.0, 12.0, 30.0) {
        r.push(Region::EasternEurope);
    }
    if inb(42.0, 51.2, -5.0, 8.2) {
        r.push(Region::France);
    }
    if inb(49.9, 59.0, -8.0, 1.8) {
        r.push(Region::Uk);
    }
    if inb(36.0, 60.0, -10.0, 20.0) {
        r.push(Region::WesternEurope);
    }
    if inb(24.0, 60.0, -130.0, -60.0) {
        r.push(Region::NorthAmerica);
    }
    if inb(30.0, 46.0, 129.0, 146.0) {
        r.push(Region::Japan);
    }
    if inb(18.0, 50.0, 97.0, 123.0) {
        r.push(Region::China);
    }
    r
}

/// Facade style indices of `scene::mesh::mat::facade`.
pub mod style {
    pub const PLASTER: usize = 0;
    pub const BRICK: usize = 1;
    pub const PANEL: usize = 2;
    pub const GLASS: usize = 3;
    pub const WOOD: usize = 4;
    pub const TILE_PANEL: usize = 6;
    pub const SILICATE: usize = 7;
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Use {
    Housing,
    House,
    Office,
}

pub struct Entry {
    pub id: &'static str,
    pub name: &'static str,
    /// normalised `building:series` spellings (lowercase, no spaces/dashes/dots; Cyrillic folded to Latin)
    pub aliases: &'static [&'static str],
    pub regions: &'static [Region],
    pub years: (i32, i32),
    pub levels: (u32, u32),
    pub depth: (f64, f64),
    pub usage: Use,
    pub style: usize,
    pub variant: usize,
    pub palette: &'static [[f32; 3]],
    pub floor_h: f32,
    pub balconies: Balconies,
    pub balcony_every: u32,
    pub balcony_sides: f32,
    pub stair: StairWindows,
    pub canopy: Canopy,
    pub plinth_h: f32,
    pub parapet_h: f32,
    pub tech_floor: bool,
    pub shops: bool,
    pub section_len: f32,
    pub bay: f32,
}

const GREY_PANEL: &[[f32; 3]] = &[[0.78, 0.77, 0.74], [0.84, 0.82, 0.76], [0.72, 0.73, 0.74], [0.80, 0.80, 0.80]];
const KHRUSH: &[[f32; 3]] = &[[0.80, 0.78, 0.72], [0.86, 0.80, 0.62], [0.74, 0.76, 0.78], [0.84, 0.72, 0.62], [0.78, 0.82, 0.78]];
const SILICATE_WHITE: &[[f32; 3]] = &[[0.90, 0.89, 0.86], [0.86, 0.85, 0.82], [0.92, 0.90, 0.84]];
const RED_BRICK: &[[f32; 3]] = &[[0.58, 0.30, 0.22], [0.66, 0.36, 0.26], [0.52, 0.27, 0.20]];
const STALIN: &[[f32; 3]] = &[[0.90, 0.82, 0.64], [0.86, 0.74, 0.58], [0.88, 0.70, 0.62], [0.92, 0.88, 0.78]];
const TILE: &[[f32; 3]] = &[[0.92, 0.92, 0.92], [0.80, 0.86, 0.94], [0.94, 0.88, 0.80], [0.90, 0.80, 0.78]];
const PASTEL: &[[f32; 3]] = &[[0.94, 0.90, 0.78], [0.86, 0.92, 0.88], [0.92, 0.84, 0.80], [0.84, 0.88, 0.94], [0.96, 0.94, 0.90]];
const MODERN: &[[f32; 3]] = &[[0.92, 0.90, 0.86], [0.76, 0.74, 0.70], [0.86, 0.78, 0.66], [0.60, 0.62, 0.66], [0.94, 0.94, 0.94]];
const LIMESTONE: &[[f32; 3]] = &[[0.90, 0.86, 0.76], [0.88, 0.84, 0.74], [0.92, 0.88, 0.80]];
const SIDING: &[[f32; 3]] = &[[0.92, 0.92, 0.88], [0.80, 0.84, 0.86], [0.86, 0.82, 0.72], [0.70, 0.76, 0.70], [0.90, 0.86, 0.80]];
const WHITE_TILE: &[[f32; 3]] = &[[0.94, 0.93, 0.90], [0.88, 0.86, 0.82], [0.80, 0.78, 0.74]];
const GLASSY: &[[f32; 3]] = &[[0.55, 0.62, 0.68], [0.45, 0.55, 0.60], [0.60, 0.60, 0.58]];

use Balconies as B;
use Canopy as C;
use Region as R;
use StairWindows as S;

macro_rules! e {
    ($id:expr, $name:expr, [$($al:expr),*], [$($rg:expr),*], $years:expr, $lv:expr, $depth:expr, $use:expr, $st:expr, $var:expr, $pal:expr, $fh:expr,
     $bal:expr, $every:expr, $sides:expr, $stair:expr, $can:expr, $plinth:expr, $parapet:expr, $tech:expr, $shops:expr, $sec:expr, $bay:expr) => {
        Entry { id: $id, name: $name, aliases: &[$($al),*], regions: &[$($rg),*], years: $years, levels: $lv, depth: $depth, usage: $use, style: $st, variant: $var, palette: $pal, floor_h: $fh,
            balconies: $bal, balcony_every: $every, balcony_sides: $sides, stair: $stair, canopy: $can, plinth_h: $plinth, parapet_h: $parapet, tech_floor: $tech, shops: $shops, section_len: $sec, bay: $bay }
    };
}

pub static CATALOGUE: &[Entry] = &[
    // --- USSR / Belarus housing ---------------------------------------------------------------
    e!("1-464", "1-464 (panel khrushchyovka)", ["1464", "1464a", "1464d"], [R::ExUssr], (1958, 1975), (4, 5), (9.5, 12.0), Use::Housing, style::PANEL, 2, KHRUSH, 2.7,
       B::Open, 2, 0.5, S::Landing, C::Slab, 0.6, 0.25, false, false, 15.0, 3.2),
    e!("m464", "M464 (Minsk panel)", ["m464", "m4641", "464m"], [R::Belarus], (1965, 1990), (5, 9), (10.5, 12.5), Use::Housing, style::PANEL, 0, GREY_PANEL, 2.7,
       B::Open, 2, 0.5, S::Landing, C::Slab, 0.7, 0.3, false, false, 16.0, 3.2),
    e!("1-335", "1-335 (panel khrushchyovka)", ["1335", "1335a"], [R::ExUssr], (1958, 1972), (4, 5), (9.5, 12.0), Use::Housing, style::PANEL, 2, GREY_PANEL, 2.7,
       B::Open, 3, 0.5, S::Landing, C::Slab, 0.6, 0.2, false, false, 15.0, 3.2),
    e!("1-447", "1-447 (brick khrushchyovka)", ["1447", "1447s"], [R::ExUssr], (1958, 1975), (4, 5), (10.0, 12.5), Use::Housing, style::SILICATE, 0, SILICATE_WHITE, 2.8,
       B::Open, 3, 0.5, S::Landing, C::Slab, 0.5, 0.4, false, false, 14.0, 3.0),
    e!("1-515", "1-515/9 (panel 9-storey)", ["15159", "1515"], [R::ExUssr], (1966, 1985), (9, 9), (11.0, 13.0), Use::Housing, style::PANEL, 0, GREY_PANEL, 2.7,
       B::Open, 2, 0.5, S::Landing, C::Slab, 0.8, 0.6, true, false, 24.0, 3.2),
    e!("II-49", "II-49 (panel, tiled)", ["ii49", "249"], [R::ExUssr], (1965, 1985), (9, 12), (11.5, 13.5), Use::Housing, style::TILE_PANEL, 0, TILE, 2.7,
       B::Open, 1, 0.5, S::Landing, C::Slab, 0.8, 0.6, true, false, 24.0, 3.2),
    e!("II-18", "II-18 (brick tower)", ["ii18", "218"], [R::ExUssr], (1960, 1985), (9, 12), (14.0, 26.0), Use::Housing, style::SILICATE, 1, SILICATE_WHITE, 2.8,
       B::Open, 2, 1.0, S::Band, C::Slab, 0.6, 0.6, false, false, 999.0, 3.0),
    e!("II-57", "II-57 (panel tower)", ["ii57", "257"], [R::ExUssr], (1965, 1985), (9, 16), (14.0, 24.0), Use::Housing, style::PANEL, 1, GREY_PANEL, 2.8,
       B::Glazed, 2, 1.0, S::Band, C::Slab, 0.8, 0.6, true, false, 999.0, 3.0),
    e!("90", "90 series (Belarus large panel)", ["90", "9001", "90m"], [R::Belarus], (1975, 2005), (5, 10), (11.5, 13.5), Use::Housing, style::PANEL, 0, PASTEL, 2.8,
       B::Glazed, 2, 0.5, S::Landing, C::Slab, 0.8, 0.6, true, false, 22.0, 3.0),
    e!("111-90", "111-90 (Belarus panel)", ["11190", "111"], [R::Belarus], (1980, 2010), (9, 12), (12.0, 14.0), Use::Housing, style::PANEL, 1, PASTEL, 2.8,
       B::Glazed, 1, 0.5, S::Landing, C::Slab, 0.8, 0.6, true, false, 24.0, 3.0),
    e!("P-44", "P-44 (panel, tiled)", ["p44", "p44t", "p44k", "p46"], [R::ExUssr], (1979, 2005), (10, 17), (12.0, 14.0), Use::Housing, style::TILE_PANEL, 1, TILE, 2.8,
       B::Glazed, 2, 0.5, S::None, C::Slab, 0.9, 0.8, true, false, 30.0, 3.0),
    e!("P-3", "P-3 (panel tower)", ["p3", "p3m"], [R::ExUssr], (1975, 1995), (16, 22), (15.0, 26.0), Use::Housing, style::PANEL, 1, GREY_PANEL, 2.8,
       B::Glazed, 1, 1.0, S::None, C::Slab, 0.9, 0.8, true, false, 999.0, 3.0),
    e!("KOPE", "KOPE (panel tower)", ["kope", "kopemm"], [R::ExUssr], (1985, 2010), (17, 25), (14.0, 26.0), Use::Housing, style::TILE_PANEL, 2, TILE, 2.8,
       B::Glazed, 1, 1.0, S::None, C::Slab, 0.9, 0.8, true, false, 999.0, 3.0),
    e!("stalinka", "Stalinist block (stalinka)", ["stalinka"], [R::ExUssr], (1935, 1958), (3, 8), (12.0, 18.0), Use::Housing, style::PLASTER, 1, STALIN, 3.3,
       B::Railing, 3, 0.5, S::Landing, C::Slab, 1.2, 1.1, false, true, 20.0, 3.3),
    e!("pre-1917", "pre-1917 tenement", [], [R::ExUssr], (1850, 1917), (2, 5), (10.0, 20.0), Use::Housing, style::PLASTER, 2, STALIN, 3.6,
       B::None, 1, 0.0, S::None, C::None, 0.9, 1.0, false, true, 20.0, 3.4),
    e!("monolith", "modern monolithic block", [], [R::ExUssr], (2000, 2035), (9, 30), (12.0, 22.0), Use::Housing, style::PLASTER, 0, MODERN, 3.0,
       B::Glazed, 1, 0.5, S::Band, C::Slab, 1.0, 0.9, false, true, 28.0, 3.0),
    // --- Central / Eastern Europe ----------------------------------------------------------------
    e!("WBS 70", "WBS 70 (GDR panel)", ["wbs70", "p2", "qp71"], [R::Gdr], (1972, 1990), (5, 11), (11.0, 13.0), Use::Housing, style::PANEL, 0, PASTEL, 2.8,
       B::Open, 2, 0.5, S::Band, C::Slab, 0.9, 0.5, false, false, 24.0, 3.0),
    e!("panelak", "panel block (paneláks / wielka płyta)", ["t06b", "vvu", "wk70", "wufs"], [R::EasternEurope], (1960, 1992), (4, 13), (10.0, 14.0), Use::Housing, style::PANEL, 2, PASTEL, 2.8,
       B::Glazed, 2, 0.5, S::Band, C::Slab, 0.8, 0.5, false, false, 22.0, 3.0),
    // --- Western Europe -------------------------------------------------------------------------
    e!("haussmann", "Haussmannian apartment building", ["haussmann"], [R::France], (1853, 1914), (5, 8), (10.0, 22.0), Use::Housing, style::PLASTER, 1, LIMESTONE, 3.2,
       B::Railing, 1, 1.0, S::None, C::None, 1.0, 0.4, false, true, 20.0, 2.6),
    e!("terrace", "Victorian terraced houses", ["terrace"], [R::Uk], (1840, 1918), (2, 3), (7.0, 12.0), Use::House, style::BRICK, 0, RED_BRICK, 3.0,
       B::None, 1, 0.0, S::None, C::Gable, 0.5, 0.0, false, false, 5.5, 2.5),
    e!("eu-postwar", "post-war apartment block", [], [R::WesternEurope], (1950, 1985), (4, 12), (10.0, 14.0), Use::Housing, style::PLASTER, 2, PASTEL, 2.8,
       B::Open, 1, 0.5, S::Band, C::Slab, 0.8, 0.5, false, false, 20.0, 3.0),
    e!("eu-modern", "contemporary apartment block", [], [R::WesternEurope], (1990, 2035), (4, 12), (10.0, 18.0), Use::Housing, style::PLASTER, 0, MODERN, 3.0,
       B::Glazed, 1, 0.5, S::Band, C::Slab, 0.6, 0.9, false, true, 22.0, 3.0),
    // --- North America / Asia -------------------------------------------------------------------
    e!("us-house", "North American frame house", [], [R::NorthAmerica], (1900, 2035), (1, 3), (6.0, 16.0), Use::House, style::WOOD, 0, SIDING, 2.9,
       B::None, 1, 0.0, S::None, C::Gable, 0.5, 0.0, false, false, 999.0, 3.0),
    e!("us-walkup", "brick walk-up apartments", [], [R::NorthAmerica], (1880, 1960), (3, 6), (10.0, 25.0), Use::Housing, style::BRICK, 1, RED_BRICK, 3.1,
       B::None, 1, 0.0, S::None, C::Slab, 0.8, 0.8, false, true, 20.0, 3.0),
    e!("jp-mansion", "Japanese mansion block", [], [R::Japan], (1970, 2035), (4, 15), (8.0, 15.0), Use::Housing, style::PLASTER, 0, WHITE_TILE, 2.9,
       B::Open, 1, 0.5, S::None, C::Slab, 0.5, 1.0, false, false, 999.0, 2.8),
    e!("cn-tower", "Chinese residential tower", [], [R::China], (1990, 2035), (12, 34), (14.0, 26.0), Use::Housing, style::PLASTER, 2, MODERN, 3.0,
       B::Glazed, 1, 1.0, S::None, C::Slab, 1.0, 1.2, false, false, 999.0, 3.0),
    // --- anywhere -----------------------------------------------------------------------------
    e!("office-tower", "glass office tower", [], [R::Any], (1960, 2035), (8, 120), (15.0, 80.0), Use::Office, style::GLASS, 0, GLASSY, 3.8,
       B::None, 1, 0.0, S::None, C::Slab, 1.2, 1.2, false, true, 999.0, 1.5),
];

/// Normalise a `building:series` value: lowercase, Cyrillic look-alikes to Latin, no separators.
pub fn normalise(s: &str) -> String {
    s.to_lowercase()
        .chars()
        .map(|c| match c {
            'п' => 'p',
            'к' => 'k',
            'о' => 'o',
            'е' | 'э' => 'e',
            'м' => 'm',
            'т' => 't',
            'а' => 'a',
            'с' => 's',
            'д' => 'd',
            'и' => 'i',
            'в' => 'v',
            'у' => 'u',
            other => other,
        })
        .filter(|c| c.is_ascii_alphanumeric())
        .collect()
}

/// Match a building. `levels` from the data (None = unknown), `depth` = corpus depth (m, the outline's
/// smaller dimension), `year` from `start_date`.
pub fn match_design(series_tag: Option<&str>, class: &str, lat: f64, lon: f64, levels: Option<u32>, depth: f64, year: Option<i32>, seed: u64) -> Option<&'static Entry> {
    if let Some(t) = series_tag {
        let n = normalise(t);
        if let Some(e) = CATALOGUE.iter().find(|e| e.aliases.iter().any(|a| *a == n) || normalise(e.id) == n) {
            return Some(e);
        }
        // "1-464A-17" and similar: longest alias that the tag starts with
        if let Some(e) = CATALOGUE.iter().filter_map(|e| e.aliases.iter().filter(|a| a.len() >= 2 && n.starts_with(**a)).map(|a| (a.len(), e)).max_by_key(|x| x.0)).max_by_key(|x| x.0).map(|x| x.1) {
            return Some(e);
        }
    }
    let usage = match class {
        "house" | "detached" | "semidetached_house" | "terrace" | "bungalow" => Use::House,
        "office" | "commercial" => Use::Office,
        "apartments" | "residential" | "dormitory" | "yes" => Use::Housing,
        _ => return None,
    };
    let here = regions_at(lat, lon);
    let mut best: Vec<(f64, &'static Entry)> = Vec::new();
    for e in CATALOGUE.iter() {
        if e.usage != usage {
            continue;
        }
        let in_region = e.regions.iter().any(|r| *r == Region::Any || here.contains(r));
        if !in_region {
            continue;
        }
        let mut score = 0.0;
        // a more specific region (Belarus over the whole ex-USSR) wins ties
        if e.regions.iter().any(|r| here.first() == Some(r)) {
            score += 1.0;
        }
        match levels {
            Some(l) if l >= e.levels.0 && l <= e.levels.1 => score += 4.0,
            Some(l) if l + 1 >= e.levels.0 && l <= e.levels.1 + 1 => score += 1.5,
            Some(_) => continue,
            None => {}
        }
        if depth >= e.depth.0 - 0.8 && depth <= e.depth.1 + 0.8 {
            score += 2.0;
        } else if depth < e.depth.0 - 4.0 || depth > e.depth.1 + 8.0 {
            continue;
        }
        match year {
            Some(y) if y >= e.years.0 && y <= e.years.1 => score += 4.0,
            Some(y) if y + 8 < e.years.0 || y > e.years.1 + 8 => continue,
            Some(_) => score += 0.5,
            None => {}
        }
        // offices only for tall commercial buildings
        if e.usage == Use::Office && levels.unwrap_or(0) < 8 {
            continue;
        }
        best.push((score, e));
    }
    let top = best.iter().map(|b| b.0).fold(f64::MIN, f64::max);
    let tied: Vec<&'static Entry> = best.iter().filter(|b| b.0 >= top - 1e-9).map(|b| b.1).collect();
    if tied.is_empty() {
        return None;
    }
    Some(tied[(seed as usize >> 3) % tied.len()])
}

impl Entry {
    pub fn design(&self) -> Design {
        Design {
            id: self.id.to_string(),
            name: self.name.to_string(),
            balconies: self.balconies,
            balcony_every: self.balcony_every,
            balcony_sides: self.balcony_sides,
            stair: self.stair,
            canopy: self.canopy,
            plinth_h: self.plinth_h,
            parapet_h: self.parapet_h,
            tech_floor: self.tech_floor,
            shops: self.shops,
            section_len: self.section_len,
            bay: self.bay,
        }
    }
    /// Typical storeys when the data has none.
    pub fn typical_levels(&self, seed: u64) -> u32 {
        let (a, b) = self.levels;
        a + (seed % (b - a + 1) as u64) as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn series_tag_wins_and_cyrillic_is_folded() {
        assert_eq!(match_design(Some("П-44Т"), "apartments", 55.7, 37.6, None, 12.0, None, 1).unwrap().id, "P-44");
        assert_eq!(match_design(Some("1-464А-17"), "yes", 53.9, 27.5, None, 11.0, None, 1).unwrap().id, "1-464");
        assert_eq!(match_design(Some("II-18/12"), "yes", 55.7, 37.6, None, 20.0, None, 1).unwrap().id, "II-18");
    }

    #[test]
    fn regions_storeys_depth_and_year_pick_plausible_designs() {
        // Minsk, 5 storeys, 11 m deep, 1962: a khrushchyovka
        let e = match_design(None, "apartments", 53.9, 27.56, Some(5), 11.0, Some(1962), 7).unwrap();
        assert!(["1-464", "1-335", "1-447", "m464"].contains(&e.id), "{}", e.id);
        // Minsk, 9 storeys, 1985: a Belarusian panel series
        let e = match_design(None, "apartments", 53.9, 27.56, Some(9), 13.0, Some(1985), 7).unwrap();
        assert!(["90", "111-90", "m464", "1-515", "II-49"].contains(&e.id), "{}", e.id);
        // Paris, 6 storeys, 1880: Haussmann
        assert_eq!(match_design(None, "apartments", 48.86, 2.35, Some(6), 14.0, Some(1880), 1).unwrap().id, "haussmann");
        // a suburban US house
        assert_eq!(match_design(None, "house", 40.0, -90.0, Some(2), 10.0, None, 3).unwrap().id, "us-house");
        // nothing sensible for a garage
        assert!(match_design(None, "garage", 53.9, 27.56, Some(1), 6.0, None, 1).is_none());
    }
}
