//! Web-Mercator ("slippy map") tile scheme used as the streaming cell grid. Zoom 15 is about 720 m x 720 m at
//! Minsk's latitude (1.2 km at the equator): small enough to load quickly, big enough to keep request counts sane.

pub const ZOOM: u8 = 15;
pub const MAX_LAT: f64 = 85.05112878;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, PartialOrd, Ord)]
pub struct TileId {
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    pub south: f64,
    pub west: f64,
    pub north: f64,
    pub east: f64,
}

impl Bounds {
    pub fn contains(&self, lat: f64, lon: f64) -> bool {
        // half-open like `TileId::from_lat_lon`: north and west edges belong to the tile
        lat > self.south && lat <= self.north && lon >= self.west && lon < self.east
    }
    pub fn intersects_box(&self, s: f64, w: f64, n: f64, e: f64) -> bool {
        !(n < self.south || s > self.north || e < self.west || w > self.east)
    }
}

impl TileId {
    pub fn count(z: u8) -> u32 {
        1u32 << z
    }

    pub fn from_lat_lon(lat: f64, lon: f64, z: u8) -> TileId {
        let n = Self::count(z) as f64;
        let lat = lat.clamp(-MAX_LAT, MAX_LAT);
        let lon = ((lon + 180.0).rem_euclid(360.0)) - 180.0;
        let x = ((lon + 180.0) / 360.0 * n).floor();
        let y = ((1.0 - lat.to_radians().tan().asinh() / std::f64::consts::PI) / 2.0 * n).floor();
        TileId { z, x: (x as i64).clamp(0, n as i64 - 1) as u32, y: (y as i64).clamp(0, n as i64 - 1) as u32 }
    }

    fn lon_of(x: f64, n: f64) -> f64 {
        x / n * 360.0 - 180.0
    }
    fn lat_of(y: f64, n: f64) -> f64 {
        (std::f64::consts::PI * (1.0 - 2.0 * y / n)).sinh().atan().to_degrees()
    }

    pub fn bounds(&self) -> Bounds {
        let n = Self::count(self.z) as f64;
        Bounds {
            west: Self::lon_of(self.x as f64, n),
            east: Self::lon_of(self.x as f64 + 1.0, n),
            north: Self::lat_of(self.y as f64, n),
            south: Self::lat_of(self.y as f64 + 1.0, n),
        }
    }

    pub fn center(&self) -> (f64, f64) {
        let b = self.bounds();
        ((b.south + b.north) / 2.0, (b.west + b.east) / 2.0)
    }

    pub fn key(&self) -> u64 {
        ((self.z as u64) << 56) | ((self.x as u64) << 28) | self.y as u64
    }

    /// "z/x/y", also used as the cache path.
    pub fn path(&self) -> String {
        format!("{}/{}/{}", self.z, self.x, self.y)
    }

    /// All tiles that overlap the square of half-size `radius_m` around a point, nearest first.
    pub fn around(lat: f64, lon: f64, radius_m: f64, z: u8) -> Vec<TileId> {
        let dlat = radius_m / 111_320.0;
        let dlon = radius_m / (111_320.0 * lat.to_radians().cos().max(0.01));
        let a = Self::from_lat_lon(lat + dlat, lon - dlon, z); // north-west
        let b = Self::from_lat_lon(lat - dlat, lon + dlon, z); // south-east
        let n = Self::count(z);
        let span_x = if b.x >= a.x { b.x - a.x } else { b.x + n - a.x };
        let mut v = Vec::new();
        for dx in 0..=span_x.min(n - 1) {
            for y in a.y..=b.y {
                v.push(TileId { z, x: (a.x + dx) % n, y });
            }
        }
        let centre = Self::from_lat_lon(lat, lon, z);
        v.sort_by_key(|t| {
            let ddx = ((t.x as i64 - centre.x as i64).abs()).min(n as i64 - (t.x as i64 - centre.x as i64).abs());
            let ddy = (t.y as i64 - centre.y as i64).abs();
            ddx * ddx + ddy * ddy
        });
        v.dedup();
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn minsk_tile_matches_python_reference() {
        // Python: x = 18892.709, y = 10535.975 at z15
        let t = TileId::from_lat_lon(53.9045, 27.5615, 15);
        assert_eq!((t.x, t.y), (18892, 10535));
        assert_eq!(t.path(), "15/18892/10535");
    }

    #[test]
    fn bounds_contain_their_points_and_tile_back_to_themselves() {
        for &(la, lo) in &[(53.9045, 27.5615), (48.8584, 2.2945), (-33.8568, 151.2153), (35.6595, 139.7005), (0.0, 0.0), (-0.0001, -0.0001), (64.1466, -21.9426)] {
            let t = TileId::from_lat_lon(la, lo, 15);
            let b = t.bounds();
            assert!(b.contains(la, lo), "{la},{lo} not in {b:?}");
            let (cl, co) = t.center();
            assert_eq!(TileId::from_lat_lon(cl, co, 15), t);
            assert!(b.north > b.south && b.east > b.west);
        }
    }

    #[test]
    fn adjacent_tiles_share_edges_exactly() {
        let t = TileId { z: 15, x: 18892, y: 10535 };
        let r = TileId { z: 15, x: 18893, y: 10535 };
        let d = TileId { z: 15, x: 18892, y: 10536 };
        assert_eq!(t.bounds().east, r.bounds().west);
        assert_eq!(t.bounds().south, d.bounds().north);
    }

    #[test]
    fn tile_size_is_a_few_hundred_metres_at_minsk() {
        let b = TileId::from_lat_lon(53.9, 27.56, 15).bounds();
        let w = (b.east - b.west) * 111_320.0 * 53.9f64.to_radians().cos();
        let h = (b.north - b.south) * 111_320.0;
        assert!((w - 720.0).abs() < 30.0 && (h - 720.0).abs() < 40.0, "{w} x {h}");
    }

    #[test]
    fn poles_and_antimeridian_do_not_panic() {
        for &(la, lo) in &[(90.0, 0.0), (-90.0, 0.0), (0.0, 180.0), (0.0, -180.0), (0.0, 540.0), (85.0, 179.99)] {
            let t = TileId::from_lat_lon(la, lo, 15);
            assert!(t.x < 32768 && t.y < 32768);
            let _ = t.bounds();
        }
        let v = TileId::around(0.0, 179.999, 2000.0, 15);
        assert!(v.iter().any(|t| t.x == 32767) && v.iter().any(|t| t.x == 0), "should wrap over the antimeridian");
    }

    #[test]
    fn around_is_nearest_first_and_covers_the_radius() {
        let v = TileId::around(53.9045, 27.5615, 1500.0, 15);
        let c = TileId::from_lat_lon(53.9045, 27.5615, 15);
        assert_eq!(v[0], c);
        assert!(v.len() >= 9 && v.len() <= 36, "{}", v.len());
        let mut u = v.clone();
        u.sort();
        u.dedup();
        assert_eq!(u.len(), v.len());
        // every tile whose centre is within the radius must be present
        for dx in -3i32..=3 {
            for dy in -3i32..=3 {
                let t = TileId { z: 15, x: (c.x as i32 + dx) as u32, y: (c.y as i32 + dy) as u32 };
                let (la, lo) = t.center();
                let fr = crate::geo::LocalFrame::new(53.9045, 27.5615, 0.0);
                let p = fr.to_local(la, lo, 0.0);
                if p[0].abs() < 1000.0 && p[1].abs() < 1000.0 {
                    assert!(v.contains(&t), "missing {t:?}");
                }
            }
        }
    }

    #[test]
    fn keys_are_unique() {
        let a = TileId { z: 15, x: 1, y: 2 }.key();
        let b = TileId { z: 15, x: 2, y: 1 }.key();
        let c = TileId { z: 14, x: 1, y: 2 }.key();
        assert!(a != b && a != c && b != c);
    }
}
