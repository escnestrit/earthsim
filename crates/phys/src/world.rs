//! Ground and static obstacles.
use crate::math::*;
use std::collections::HashMap;

#[derive(Clone, Copy, Debug)]
pub struct GroundSample {
    pub height: f64,
    pub normal: V3,
    /// Grip multiplier of the surface (asphalt 1.0).
    pub grip: f64,
}

pub trait Ground {
    fn sample(&self, x: f64, y: f64) -> GroundSample;
}

pub struct FlatGround {
    pub grip: f64,
}
impl Ground for FlatGround {
    fn sample(&self, _x: f64, _y: f64) -> GroundSample {
        GroundSample { height: 0.0, normal: V3::Z, grip: self.grip }
    }
}

/// Inclined plane: rises along +X by `angle` radians.
pub struct SlopeGround {
    pub angle: f64,
    pub grip: f64,
}
impl Ground for SlopeGround {
    fn sample(&self, x: f64, _y: f64) -> GroundSample {
        let (s, c) = self.angle.sin_cos();
        GroundSample { height: x * self.angle.tan(), normal: v3(-s, 0.0, c), grip: self.grip }
    }
}

/// Wraps any closure `(x, y) -> GroundSample`.
pub struct FnGround<F: Fn(f64, f64) -> GroundSample>(pub F);
impl<F: Fn(f64, f64) -> GroundSample> Ground for FnGround<F> {
    fn sample(&self, x: f64, y: f64) -> GroundSample {
        (self.0)(x, y)
    }
}

#[derive(Clone, Debug)]
pub struct Poly {
    pub pts: Vec<(f64, f64)>,
}

#[derive(Clone, Copy, Debug)]
pub struct Contact {
    /// Unit normal in the XY plane pointing away from the obstacle.
    pub nx: f64,
    pub ny: f64,
    pub depth: f64,
}

/// Anything a car can run into: queried with a disc (centre, radius) in plan view.
pub trait Obstacles {
    fn query(&self, x: f64, y: f64, r: f64, out: &mut Vec<Contact>);
    /// Cheap test: could anything be within `r` of (x, y)? False only when certainly nothing is.
    fn any_near(&self, _x: f64, _y: f64, _r: f64) -> bool {
        true
    }
}

#[derive(Clone, Debug)]
enum Shape {
    Poly(Poly),
    Circle(f64, f64, f64),
}

#[derive(Clone, Debug)]
struct Item {
    bbox: (f64, f64, f64, f64),
    shape: Option<Shape>, // None = free slot
}

/// Static obstacles in plan view (building footprints, tree trunks), with a uniform grid for lookup.
/// Items carry a tag (for example the streaming tile they came from) so a whole group can be removed at once.
#[derive(Clone)]
pub struct StaticWorld {
    cell: f64,
    items: Vec<Item>,
    free: Vec<u32>,
    by_tag: HashMap<u64, Vec<u32>>,
    grid: HashMap<(i32, i32), Vec<u32>>,
    live_polys: usize,
    live_circles: usize,
}

impl StaticWorld {
    pub fn new(cell: f64) -> Self {
        StaticWorld { cell, items: Vec::new(), free: Vec::new(), by_tag: HashMap::new(), grid: HashMap::new(), live_polys: 0, live_circles: 0 }
    }
    pub fn poly_count(&self) -> usize {
        self.live_polys
    }
    pub fn circle_count(&self) -> usize {
        self.live_circles
    }
    fn cells(&self, b: (f64, f64, f64, f64)) -> Vec<(i32, i32)> {
        let c = self.cell;
        let (i0, i1) = ((b.0 / c).floor() as i32, (b.2 / c).floor() as i32);
        let (j0, j1) = ((b.1 / c).floor() as i32, (b.3 / c).floor() as i32);
        let mut v = Vec::with_capacity(((i1 - i0 + 1) * (j1 - j0 + 1)).max(0) as usize);
        for i in i0..=i1 {
            for j in j0..=j1 {
                v.push((i, j));
            }
        }
        v
    }
    fn insert(&mut self, tag: u64, bbox: (f64, f64, f64, f64), shape: Shape) {
        let item = Item { bbox, shape: Some(shape) };
        let id = if let Some(id) = self.free.pop() {
            self.items[id as usize] = item;
            id
        } else {
            self.items.push(item);
            (self.items.len() - 1) as u32
        };
        for k in self.cells(bbox) {
            self.grid.entry(k).or_default().push(id);
        }
        self.by_tag.entry(tag).or_default().push(id);
    }
    pub fn add_poly(&mut self, pts: Vec<(f64, f64)>) {
        self.add_poly_tagged(0, pts);
    }
    pub fn add_circle(&mut self, x: f64, y: f64, r: f64) {
        self.add_circle_tagged(0, x, y, r);
    }
    pub fn add_poly_tagged(&mut self, tag: u64, pts: Vec<(f64, f64)>) {
        if pts.len() < 3 {
            return;
        }
        let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for p in &pts {
            x0 = x0.min(p.0);
            y0 = y0.min(p.1);
            x1 = x1.max(p.0);
            y1 = y1.max(p.1);
        }
        self.live_polys += 1;
        self.insert(tag, (x0, y0, x1, y1), Shape::Poly(Poly { pts }));
    }
    pub fn add_circle_tagged(&mut self, tag: u64, x: f64, y: f64, r: f64) {
        self.live_circles += 1;
        self.insert(tag, (x - r, y - r, x + r, y + r), Shape::Circle(x, y, r));
    }
    /// Remove everything added with `tag`. Returns how many items were removed.
    pub fn remove_tag(&mut self, tag: u64) -> usize {
        let Some(ids) = self.by_tag.remove(&tag) else { return 0 };
        for &id in &ids {
            let bbox = self.items[id as usize].bbox;
            for k in self.cells(bbox) {
                if let Some(v) = self.grid.get_mut(&k) {
                    v.retain(|&x| x != id);
                    if v.is_empty() {
                        self.grid.remove(&k);
                    }
                }
            }
            match self.items[id as usize].shape.take() {
                Some(Shape::Poly(_)) => self.live_polys -= 1,
                Some(Shape::Circle(..)) => self.live_circles -= 1,
                None => {}
            }
            self.free.push(id);
        }
        ids.len()
    }
}

impl Obstacles for StaticWorld {
    fn any_near(&self, x: f64, y: f64, r: f64) -> bool {
        if self.grid.is_empty() {
            return false;
        }
        let c = self.cell;
        let (i0, i1) = (((x - r) / c).floor() as i32, ((x + r) / c).floor() as i32);
        let (j0, j1) = (((y - r) / c).floor() as i32, ((y + r) / c).floor() as i32);
        if (i1 - i0 + 1) as i64 * (j1 - j0 + 1) as i64 > 400 {
            return true;
        }
        (i0..=i1).any(|i| (j0..=j1).any(|j| self.grid.get(&(i, j)).is_some_and(|l| !l.is_empty())))
    }

    /// Contacts of a disc (centre `(x,y)`, radius `r`) with all obstacles.
    fn query(&self, x: f64, y: f64, r: f64, out: &mut Vec<Contact>) {
        out.clear();
        let c = self.cell;
        let (ci, cj) = ((x / c).floor() as i32, (y / c).floor() as i32);
        let reach = (r / c).ceil() as i32;
        let mut seen: Vec<u32> = Vec::new();
        for i in (ci - reach)..=(ci + reach) {
            for j in (cj - reach)..=(cj + reach) {
                let Some(list) = self.grid.get(&(i, j)) else { continue };
                for &id in list {
                    if seen.contains(&id) {
                        continue;
                    }
                    seen.push(id);
                    match &self.items[id as usize].shape {
                        Some(Shape::Circle(cx, cy, cr)) => {
                            let (dx, dy) = (x - cx, y - cy);
                            let d = (dx * dx + dy * dy).sqrt();
                            if d < cr + r {
                                let (nx, ny) = if d > 1e-9 { (dx / d, dy / d) } else { (1.0, 0.0) };
                                out.push(Contact { nx, ny, depth: cr + r - d });
                            }
                        }
                        Some(Shape::Poly(p)) => {
                            if let Some(ct) = poly_contact(p, x, y, r) {
                                out.push(ct);
                            }
                        }
                        None => {}
                    }
                }
            }
        }
    }
}

fn poly_contact(poly: &Poly, x: f64, y: f64, r: f64) -> Option<Contact> {
    let p = &poly.pts;
    let n = p.len();
    let mut inside = false;
    let mut best = f64::MAX;
    let (mut qx, mut qy) = (0.0, 0.0);
    let mut j = n - 1;
    for i in 0..n {
        let (ax, ay) = p[j];
        let (bx, by) = p[i];
        if (ay > y) != (by > y) && x < (bx - ax) * (y - ay) / (by - ay) + ax {
            inside = !inside;
        }
        let (dx, dy) = (bx - ax, by - ay);
        let l2 = dx * dx + dy * dy;
        let t = if l2 > 0.0 { (((x - ax) * dx + (y - ay) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
        let (ex, ey) = (ax + dx * t, ay + dy * t);
        let d = ((x - ex).powi(2) + (y - ey).powi(2)).sqrt();
        if d < best {
            best = d;
            qx = ex;
            qy = ey;
        }
        j = i;
    }
    if inside {
        // Push out through the nearest edge.
        let (nx, ny) = if best > 1e-9 { ((qx - x) / best, (qy - y) / best) } else { (1.0, 0.0) };
        Some(Contact { nx, ny, depth: best + r })
    } else if best < r {
        let (nx, ny) = if best > 1e-9 { ((x - qx) / best, (y - qy) / best) } else { (1.0, 0.0) };
        Some(Contact { nx, ny, depth: r - best })
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> Vec<(f64, f64)> {
        vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    }

    #[test]
    fn disc_outside_touching_and_inside() {
        let mut w = StaticWorld::new(8.0);
        w.add_poly(square());
        let mut out = Vec::new();
        w.query(-5.0, 5.0, 0.5, &mut out);
        assert!(out.is_empty());
        w.query(-0.3, 5.0, 0.5, &mut out);
        assert_eq!(out.len(), 1);
        assert!((out[0].nx + 1.0).abs() < 1e-9 && (out[0].depth - 0.2).abs() < 1e-9);
        w.query(9.0, 5.0, 0.5, &mut out); // inside, nearest edge is x=10
        assert_eq!(out.len(), 1);
        assert!((out[0].nx - 1.0).abs() < 1e-9 && (out[0].depth - 1.5).abs() < 1e-9);
    }

    #[test]
    fn circle_obstacle() {
        let mut w = StaticWorld::new(8.0);
        w.add_circle(0.0, 0.0, 0.4);
        let mut out = Vec::new();
        w.query(1.0, 0.0, 0.8, &mut out);
        assert_eq!(out.len(), 1);
        assert!((out[0].depth - 0.2).abs() < 1e-9);
    }

    #[test]
    fn tagged_items_can_be_removed_and_slots_reused() {
        let mut w = StaticWorld::new(8.0);
        w.add_poly_tagged(7, square());
        w.add_circle_tagged(7, 30.0, 30.0, 1.0);
        w.add_circle_tagged(9, -30.0, 0.0, 1.0);
        assert_eq!((w.poly_count(), w.circle_count()), (1, 2));
        let mut out = Vec::new();
        w.query(5.0, 5.0, 0.5, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(w.remove_tag(7), 2);
        w.query(5.0, 5.0, 0.5, &mut out);
        assert!(out.is_empty());
        assert_eq!((w.poly_count(), w.circle_count()), (0, 1));
        assert_eq!(w.remove_tag(7), 0);
        // slot reuse keeps queries correct
        w.add_poly_tagged(11, square());
        w.query(5.0, 5.0, 0.5, &mut out);
        assert_eq!(out.len(), 1);
        w.query(-30.0, 0.0, 0.5, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(w.items.len(), 3, "freed slot should have been reused");
    }

    #[test]
    fn slope_normal_is_unit_and_tilted() {
        let g = SlopeGround { angle: 0.2, grip: 1.0 };
        let s = g.sample(3.0, 0.0);
        assert!((s.normal.len() - 1.0).abs() < 1e-12 && s.normal.x < 0.0);
        assert!((s.height - 3.0 * 0.2f64.tan()).abs() < 1e-12);
    }
}
