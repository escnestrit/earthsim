//! CPU-side mesh generation. Vertex layout (9 floats): position xyz, normal xyz, uv, material id.
//! World is Z-up. Ported from the WebGL prototype so both viewers show the same thing.
use crate::model::Scene;
use std::collections::HashMap;

pub const FLOATS_PER_VERTEX: usize = 9;

pub mod mat {
    pub const GROUND: f32 = 0.0;
    pub const SIDEWALK: f32 = 1.0;
    pub const ASPHALT: f32 = 2.0;
    pub const MARKING: f32 = 3.0;
    pub const WALL: f32 = 4.0;
    pub const ROOF: f32 = 5.0;
    pub const PAINT: f32 = 6.0;
    pub const GLASS: f32 = 7.0;
    pub const TIRE: f32 = 8.0;
    pub const HEADLIGHT: f32 = 9.0;
    pub const FOLIAGE: f32 = 10.0;
    pub const TRUNK: f32 = 11.0;
    pub const TAILLIGHT: f32 = 12.0;
    pub const RIM: f32 = 13.0;
    /// ground cover
    pub const GRASS: f32 = 14.0;
    pub const WATER: f32 = 15.0;
    pub const SAND: f32 = 16.0;
    pub const FOREST_FLOOR: f32 = 17.0;
    pub const PARKING: f32 = 18.0;
    /// Facade styles (walls with window rows); the old OpenGL renderer draws them all as WALL.
    pub const WALL_PLASTER: f32 = 20.0;
    pub const WALL_BRICK: f32 = 21.0;
    pub const WALL_PANEL: f32 = 22.0;
    pub const WALL_GLASS: f32 = 23.0;
    pub const WALL_WOOD: f32 = 24.0;
    pub const WALL_METAL: f32 = 25.0;
    pub fn is_facade(m: f32) -> bool {
        (19.5..26.5).contains(&m)
    }
}

#[derive(Clone, Debug)]
pub struct MeshData {
    pub name: String,
    pub verts: Vec<f32>,
    /// glPolygonOffset factor/units used to stop coplanar layers z-fighting.
    pub polygon_offset: f32,
    pub casts_shadow: bool,
    /// Optional per-vertex colour (rgb, one triple per vertex) multiplied with the material colour.
    pub colors: Vec<f32>,
}

impl MeshData {
    pub fn vertex_count(&self) -> usize {
        self.verts.len() / FLOATS_PER_VERTEX
    }
}

type V = [f64; 3];

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm(a: V) -> V {
    let l = dot(a, a).sqrt();
    if l < 1e-12 { [0.0, 0.0, 1.0] } else { [a[0] / l, a[1] / l, a[2] / l] }
}

#[derive(Default)]
pub struct Mb {
    pub a: Vec<f32>,
}

impl Mb {
    pub fn v(&mut self, p: V, n: V, u: f64, v: f64, m: f32) {
        self.a.extend_from_slice(&[p[0] as f32, p[1] as f32, p[2] as f32, n[0] as f32, n[1] as f32, n[2] as f32, u as f32, v as f32, m]);
    }
    pub fn tri(&mut self, p: V, q: V, r: V, n: V, m: f32, u: f64) {
        self.v(p, n, u, 0.0, m);
        self.v(q, n, u, 0.0, m);
        self.v(r, n, u, 0.0, m);
    }
    pub fn quad(&mut self, p0: V, p1: V, p2: V, p3: V, n: V, m: f32) {
        self.tri(p0, p1, p2, n, m, 0.0);
        self.tri(p0, p2, p3, n, m, 0.0);
    }
    /// Convex planar polygon, wound so the normal points away from `c`.
    pub fn face(&mut self, pts: &[V], c: V, m: f32) {
        let n = norm(cross(sub(pts[1], pts[0]), sub(pts[2], pts[0])));
        let mut fc = [0.0; 3];
        for p in pts {
            for k in 0..3 {
                fc[k] += p[k] / pts.len() as f64;
            }
        }
        let flip = dot(n, sub(fc, c)) < 0.0;
        let nn = if flip { [-n[0], -n[1], -n[2]] } else { n };
        let order: Vec<V> = if flip { pts.iter().rev().copied().collect() } else { pts.to_vec() };
        for i in 1..order.len() - 1 {
            self.tri(order[0], order[i], order[i + 1], nn, m, 0.0);
        }
    }
    pub fn box_(&mut self, x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64, ms: &[f32]) {
        let c = [(x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0];
        let p = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0], [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]];
        let f = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]];
        for (i, fi) in f.iter().enumerate() {
            let pts: Vec<V> = fi.iter().map(|&k| p[k]).collect();
            self.face(&pts, c, ms[if ms.len() == 6 { i } else { 0 }]);
        }
    }
    #[allow(clippy::too_many_arguments)]
    pub fn sphere(&mut self, cx: f64, cy: f64, cz: f64, rx: f64, rz: f64, m: f32, u: f64, seg: usize, rings: usize) {
        let pt = |a: f64, b: f64| {
            let (x, y, z) = (a.cos() * b.cos(), a.sin() * b.cos(), b.sin());
            ([cx + x * rx, cy + y * rx, cz + z * rz], norm([x / rx, y / rx, z / rz]))
        };
        let pi = std::f64::consts::PI;
        for i in 0..seg {
            for j in 0..rings {
                let (a0, a1) = (i as f64 / seg as f64 * 2.0 * pi, (i + 1) as f64 / seg as f64 * 2.0 * pi);
                let (b0, b1) = (-pi / 2.0 + j as f64 / rings as f64 * pi, -pi / 2.0 + (j + 1) as f64 / rings as f64 * pi);
                let (a, b, c, d) = (pt(a0, b0), pt(a1, b0), pt(a1, b1), pt(a0, b1));
                for t in [[a, b, c], [a, c, d]] {
                    for q in t {
                        self.v(q.0, q.1, u, 0.0, m);
                    }
                }
            }
        }
    }
    /// Cylinder with its axis along Y.
    pub fn cyl(&mut self, cx: f64, cz: f64, r: f64, y0: f64, y1: f64, m: f32, seg: usize) {
        let pi = std::f64::consts::PI;
        let p = |a: f64, y: f64| [cx + a.cos() * r, y, cz + a.sin() * r];
        for i in 0..seg {
            let (a0, a1) = (i as f64 / seg as f64 * 2.0 * pi, (i + 1) as f64 / seg as f64 * 2.0 * pi);
            let am = (a0 + a1) / 2.0;
            let nm = [am.cos(), 0.0, am.sin()];
            self.tri(p(a0, y0), p(a1, y0), p(a1, y1), nm, m, 0.0);
            self.tri(p(a0, y0), p(a1, y1), p(a0, y1), nm, m, 0.0);
            self.tri([cx, y1, cz], p(a0, y1), p(a1, y1), [0.0, 1.0, 0.0], m, 0.0);
            self.tri([cx, y0, cz], p(a1, y0), p(a0, y0), [0.0, -1.0, 0.0], m, 0.0);
        }
    }
}

/// A flat square ground plane of half-size `e` metres centred on the origin.
pub fn ground_mesh(e: f64) -> MeshData {
    let mut g = Mb::default();
    g.quad([-e, -e, 0.0], [e, -e, 0.0], [e, e, 0.0], [-e, e, 0.0], [0.0, 0.0, 1.0], mat::GROUND);
    MeshData { name: "ground".into(), verts: g.a, polygon_offset: 0.0, casts_shadow: false, colors: Vec::new() }
}

/// Road-like strip along a polyline with mitred joins (mitre length clamped).
fn ribbon(mb: &mut Mb, pts: &[[f64; 2]], hw: f64, z: f64, m: f32) {
    let n = pts.len();
    if n < 2 {
        return;
    }
    let mut nm = Vec::with_capacity(n - 1);
    for i in 0..n - 1 {
        let (dx, dy) = (pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]);
        let l = dx.hypot(dy).max(1e-9);
        nm.push([-dy / l, dx / l]);
    }
    let (mut left, mut right) = (Vec::with_capacity(n), Vec::with_capacity(n));
    for i in 0..n {
        let a = if i > 0 { nm[i - 1] } else { nm[0] };
        let b = if i < n - 1 { nm[i] } else { nm[n - 2] };
        let (mut mx, mut my) = (a[0] + b[0], a[1] + b[1]);
        let ml = mx.hypot(my);
        if ml < 1e-9 {
            mx = a[0];
            my = a[1];
        } else {
            mx /= ml;
            my /= ml;
        }
        let sc = 1.0 / (mx * a[0] + my * a[1]).max(0.45);
        left.push([pts[i][0] + mx * hw * sc, pts[i][1] + my * hw * sc, z]);
        right.push([pts[i][0] - mx * hw * sc, pts[i][1] - my * hw * sc, z]);
    }
    let up = [0.0, 0.0, 1.0];
    let mut s = 0.0;
    for i in 0..n - 1 {
        let l = (pts[i + 1][0] - pts[i][0]).hypot(pts[i + 1][1] - pts[i][1]);
        mb.v(left[i], up, s, 0.0, m);
        mb.v(right[i], up, s, 1.0, m);
        mb.v(right[i + 1], up, s + l, 1.0, m);
        mb.v(left[i], up, s, 0.0, m);
        mb.v(right[i + 1], up, s + l, 1.0, m);
        mb.v(left[i + 1], up, s + l, 0.0, m);
        s += l;
    }
}

fn dedup(pts: &[[f64; 2]], junction: &[bool]) -> (Vec<[f64; 2]>, Vec<bool>) {
    let (mut p, mut j): (Vec<[f64; 2]>, Vec<bool>) = (Vec::new(), Vec::new());
    for (i, q) in pts.iter().enumerate() {
        if let Some(l) = p.last() {
            if (q[0] - l[0]).hypot(q[1] - l[1]) < 0.05 {
                continue;
            }
        }
        p.push(*q);
        j.push(junction.get(i).copied().unwrap_or(false));
    }
    (p, j)
}

/// Meshes of a whole scene including the ground plane (demo / single-scene mode).
pub fn scene_meshes(sc: &Scene) -> Vec<MeshData> {
    scene_meshes_opts(sc, true)
}

/// `ground = false` for streamed tiles (the ground is a single plane that follows the camera).
pub fn scene_meshes_opts(sc: &Scene, ground: bool) -> Vec<MeshData> {
    let mut out = Vec::new();
    let roads: Vec<_> = sc
        .roads
        .iter()
        .filter_map(|r| {
            let (pts, jn) = dedup(&r.pts, &r.junction);
            (pts.len() > 1).then(|| (r, pts, jn))
        })
        .collect();

    if ground {
        out.push(ground_mesh(sc.extent() + 3000.0));
    }

    // ground cover, under sidewalks and roads (water a little above grass where they overlap)
    let mut kinds: Vec<crate::model::AreaKind> = sc.areas.iter().map(|a| a.kind).collect();
    kinds.sort_by_key(|k| *k as u8);
    kinds.dedup();
    for k in kinds {
        let mut am = Mb::default();
        let (z, off) = match k {
            crate::model::AreaKind::Water => (0.006, -0.75),
            crate::model::AreaKind::Parking => (0.008, -0.9),
            _ => (0.004, -0.5),
        };
        for a in sc.areas.iter().filter(|a| a.kind == k) {
            for t in &a.tris {
                let p = t.map(|i| {
                    let q = a.ring[i as usize];
                    [q[0], q[1], z]
                });
                // counter-clockwise from above whatever the ring orientation
                let up = (p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0]) > 0.0;
                let (b, c) = if up { (p[1], p[2]) } else { (p[2], p[1]) };
                for q in [p[0], b, c] {
                    am.v(q, [0.0, 0.0, 1.0], q[0] / 4.0, q[1] / 4.0, k.material());
                }
            }
        }
        out.push(MeshData { name: format!("area_{}", k.name()), verts: am.a, polygon_offset: off, casts_shadow: false, colors: Vec::new() });
    }

    // sidewalks
    let mut sw = Mb::default();
    for (r, pts, _) in &roads {
        if r.cls.starts_with("motorway") || r.cls.starts_with("trunk") {
            continue;
        }
        ribbon(&mut sw, pts, r.width / 2.0 + if r.cls == "service" { 1.2 } else { 2.4 }, 0.01, mat::SIDEWALK);
    }
    out.push(MeshData { name: "sidewalk".into(), verts: sw.a, polygon_offset: -1.0, casts_shadow: false, colors: Vec::new() });

    // roads: one mesh per width so wider roads can be layered over narrower ones at junctions
    let mut widths: Vec<f64> = roads.iter().map(|(r, _, _)| r.width).collect();
    widths.sort_by(|a, b| a.partial_cmp(b).unwrap());
    widths.dedup();
    for w in widths.iter() {
        let mut rb = Mb::default();
        for (r, pts, _) in &roads {
            if r.width == *w {
                ribbon(&mut rb, pts, w / 2.0, 0.02, mat::ASPHALT);
            }
        }
        // Offset depends only on the width, so the same road drawn by two neighbouring streamed tiles
        // gets the same depth bias, and wider roads always win at junctions.
        out.push(MeshData { name: format!("road{w}"), verts: rb.a, polygon_offset: -2.0 - w.round() as f32, casts_shadow: false, colors: Vec::new() });
    }

    // lane markings: dashed centre line, skipped near junctions and road ends
    let mut jg: HashMap<(i64, i64), Vec<[f64; 2]>> = HashMap::new();
    for (_, pts, jn) in &roads {
        for (i, p) in pts.iter().enumerate() {
            if jn[i] || i == 0 || i == pts.len() - 1 {
                jg.entry(((p[0] / 30.0).floor() as i64, (p[1] / 30.0).floor() as i64)).or_default().push(*p);
            }
        }
    }
    let near_j = |x: f64, y: f64, r: f64| {
        let (ci, cj) = ((x / 30.0).floor() as i64, (y / 30.0).floor() as i64);
        (ci - 1..=ci + 1).any(|i| (cj - 1..=cj + 1).any(|j| jg.get(&(i, j)).is_some_and(|a| a.iter().any(|p| (p[0] - x).hypot(p[1] - y) < r))))
    };
    let mut mk = Mb::default();
    for (r, pts, _) in &roads {
        if r.width < 5.5 || r.cls.ends_with("link") || r.cls == "service" {
            continue;
        }
        let mut cum = 0.0_f64;
        for w in pts.windows(2) {
            let (a, b) = (w[0], w[1]);
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let len = dx.hypot(dy);
            let (ux, uy) = (dx / len, dy / len);
            let hw = 0.09;
            let mut k = (cum / 9.0).floor() as i64;
            while (k as f64) * 9.0 < cum + len {
                let s0 = (k as f64 * 9.0).max(cum) - cum;
                let s1 = (k as f64 * 9.0 + 3.0).min(cum + len) - cum;
                k += 1;
                if s1 - s0 < 0.4 {
                    continue;
                }
                let (mx, my) = (a[0] + ux * (s0 + s1) / 2.0, a[1] + uy * (s0 + s1) / 2.0);
                if near_j(mx, my, r.width / 2.0 + 4.0) {
                    continue;
                }
                let (p0, p1) = ([a[0] + ux * s0, a[1] + uy * s0], [a[0] + ux * s1, a[1] + uy * s1]);
                mk.quad(
                    [p0[0] - uy * hw, p0[1] + ux * hw, 0.03],
                    [p1[0] - uy * hw, p1[1] + ux * hw, 0.03],
                    [p1[0] + uy * hw, p1[1] - ux * hw, 0.03],
                    [p0[0] + uy * hw, p0[1] - ux * hw, 0.03],
                    [0.0, 0.0, 1.0],
                    mat::MARKING,
                );
            }
            cum += len;
        }
    }
    out.push(MeshData { name: "marks".into(), verts: mk.a, polygon_offset: -30.0, casts_shadow: false, colors: Vec::new() });

    // street lights (see `lamp_posts`): pole, arm over the carriageway, lamp head
    let mut lm = Mb::default();
    for (p, h, nrm) in lamp_posts(sc) {
        let (px, py, ax, ay) = (p[0], p[1], h[0], h[1]);
        lm.box_(px - 0.09, py - 0.09, 0.0, px + 0.09, py + 0.09, 7.0, &[mat::RIM]);
        let (tx, ty) = (-nrm[1] * 0.05, nrm[0] * 0.05);
        let (p0, p1, p2, p3) = ([px + tx, py + ty, 6.97], [ax + tx, ay + ty, 6.97], [ax - tx, ay - ty, 6.97], [px - tx, py - ty, 6.97]);
        lm.quad(p0, p1, p2, p3, [0.0, 0.0, 1.0], mat::RIM);
        lm.quad(p3, p2, p1, p0, [0.0, 0.0, -1.0], mat::RIM);
        lm.box_(ax - 0.28, ay - 0.16, 6.7, ax + 0.28, ay + 0.16, 6.88, &[mat::HEADLIGHT]);
    }
    out.push(MeshData { name: "streetlights".into(), verts: lm.a, polygon_offset: 0.0, casts_shadow: true, colors: Vec::new() });

    // buildings: the Stage A mesh. Walls get u = distance along the footprint (m) and v = height scaled so
    // that one storey is 3 units, so a facade texture repeating every 3 x 3 units puts one window per bay and
    // floor; the facade style picks the material and the colour is per vertex.
    let mut bm = Mb::default();
    let mut bcol: Vec<f32> = Vec::new();
    for b in sc.buildings.iter() {
        let n = if b.ring > 0 { b.ring as usize } else { b.verts.len() / 6 };
        let ring_ok = n >= 3 && b.verts.len() >= n * 6;
        let corner = |i: usize| [b.verts[i * 3] as f64, b.verts[i * 3 + 1] as f64];
        let mut cum = vec![0.0f64; n + 1];
        let mut dir = vec![[0.0f64; 2]; n];
        if ring_ok {
            for i in 0..n {
                let (a, c) = (corner(i), corner((i + 1) % n));
                let (dx, dy) = (c[0] - a[0], c[1] - a[1]);
                let l = (dx * dx + dy * dy).sqrt();
                cum[i + 1] = cum[i] + l;
                dir[i] = if l > 1e-9 { [dx / l, dy / l] } else { [1.0, 0.0] };
            }
        }
        let vscale = 3.0 / b.facade.floor_height.max(2.0) as f64;
        for t in b.indices.chunks_exact(3) {
            let p: Vec<V> = t.iter().map(|&k| {
                let k = k as usize * 3;
                [b.verts[k] as f64, b.verts[k + 1] as f64, b.verts[k + 2] as f64]
            }).collect();
            let nn = cross(sub(p[1], p[0]), sub(p[2], p[0]));
            let l = dot(nn, nn).sqrt();
            if l < 1e-6 {
                continue;
            }
            let nn = [nn[0] / l, nn[1] / l, nn[2] / l];
            let roof = nn[2].abs() > 0.5;
            let (m, col) = if roof { (mat::ROOF, b.facade.roof_colour) } else { (b.facade.style, b.facade.colour) };
            // wall triangles start with a ring vertex of the edge they stand on
            let edge = t[0] as usize % n.max(1);
            for q in &p {
                let (u, v) = if roof || !ring_ok {
                    (q[0], q[1])
                } else {
                    let a = corner(edge);
                    (cum[edge] + (q[0] - a[0]) * dir[edge][0] + (q[1] - a[1]) * dir[edge][1], q[2] * vscale)
                };
                bm.v(*q, nn, u, v, m);
                bcol.extend_from_slice(&col);
            }
        }
    }
    out.push(MeshData { name: "buildings".into(), verts: bm.a, polygon_offset: 0.0, casts_shadow: true, colors: bcol });

    // entrances: door in the wall, and for doors people use a porch slab and a canopy (the common Soviet
    // and modern block layout); sizes are typical, not measured
    let mut em = Mb::default();
    for e in &sc.entrances {
        let (o, t) = (e.out, [-e.out[1], e.out[0]]);
        let (w, canopy) = match e.kind.as_str() {
            "garage" => (2.6, false),
            "service" | "emergency" | "exit" => (1.0, false),
            "main" => (2.0, true),
            _ => (1.4, true),
        };
        let at = |a: f64, d: f64, z: f64| [e.p[0] + t[0] * a + o[0] * d, e.p[1] + t[1] * a + o[1] * d, z];
        let porch = if canopy { 0.3 } else { 0.0 };
        // door leaf just outside the wall
        em.quad(at(-w / 2.0, 0.04, porch), at(w / 2.0, 0.04, porch), at(w / 2.0, 0.04, porch + 2.2), at(-w / 2.0, 0.04, porch + 2.2), [o[0], o[1], 0.0], mat::GLASS);
        // frame
        for (a0, a1, z0, z1) in [(-w / 2.0 - 0.12, -w / 2.0, porch, porch + 2.32), (w / 2.0, w / 2.0 + 0.12, porch, porch + 2.32), (-w / 2.0 - 0.12, w / 2.0 + 0.12, porch + 2.2, porch + 2.32)] {
            em.quad(at(a0, 0.06, z0), at(a1, 0.06, z0), at(a1, 0.06, z1), at(a0, 0.06, z1), [o[0], o[1], 0.0], mat::RIM);
        }
        if canopy {
            let slab = |mb: &mut Mb, hw: f64, depth: f64, z0: f64, z1: f64, m: f32| {
                let c = at(0.0, depth / 2.0, (z0 + z1) / 2.0);
                let pts = |z: f64| [at(-hw, 0.0, z), at(hw, 0.0, z), at(hw, depth, z), at(-hw, depth, z)];
                let (b, tp) = (pts(z0), pts(z1));
                mb.face(&tp, c, m);
                mb.face(&b, c, m);
                for k in 0..4 {
                    let k1 = (k + 1) % 4;
                    mb.face(&[b[k], b[k1], tp[k1], tp[k]], c, m);
                }
            };
            slab(&mut em, w / 2.0 + 0.6, 1.6, 0.0, porch, mat::SIDEWALK);
            slab(&mut em, w / 2.0 + 0.5, 1.3, porch + 2.55, porch + 2.7, mat::ROOF);
        }
    }
    out.push(MeshData { name: "entrances".into(), verts: em.a, polygon_offset: 0.0, casts_shadow: true, colors: Vec::new() });

    // trees
    let mut tm = Mb::default();
    for (i, t) in sc.trees.iter().enumerate() {
        let s = (i as f64 * 0.7548776662).fract();
        let rr = 1.9 + s * 1.3;
        tm.box_(t[0] - 0.15, t[1] - 0.15, 0.0, t[0] + 0.15, t[1] + 0.15, 2.3, &[mat::TRUNK]);
        tm.sphere(t[0], t[1], 2.3 + rr * 0.8, rr, rr * 1.1, mat::FOLIAGE, s, 6, 3);
    }
    out.push(MeshData { name: "trees".into(), verts: tm.a, polygon_offset: 0.0, casts_shadow: true, colors: Vec::new() });
    out
}

/// Street lamp posts of a scene: (pole foot, lamp head, unit normal from the road to the pole). Town roads,
/// every ~32 m on alternating sides, none within 8 m of a junction or road end.
pub fn lamp_posts(sc: &Scene) -> Vec<([f64; 2], [f64; 2], [f64; 2])> {
    let roads: Vec<_> = sc.roads.iter().filter_map(|r| {
        let (pts, jn) = dedup(&r.pts, &r.junction);
        (pts.len() > 1).then(|| (r, pts, jn))
    }).collect();
    let mut jg: HashMap<(i64, i64), Vec<[f64; 2]>> = HashMap::new();
    for (_, pts, jn) in &roads {
        for (i, p) in pts.iter().enumerate() {
            if jn[i] || i == 0 || i == pts.len() - 1 {
                jg.entry(((p[0] / 30.0).floor() as i64, (p[1] / 30.0).floor() as i64)).or_default().push(*p);
            }
        }
    }
    let near_j = |x: f64, y: f64, r: f64| {
        let (ci, cj) = ((x / 30.0).floor() as i64, (y / 30.0).floor() as i64);
        (ci - 1..=ci + 1).any(|i| (cj - 1..=cj + 1).any(|j| jg.get(&(i, j)).is_some_and(|a| a.iter().any(|p| (p[0] - x).hypot(p[1] - y) < r))))
    };
    let mut out = Vec::new();
    for (ri, (r, pts, _)) in roads.iter().enumerate() {
        if r.width < 5.5 || r.cls.starts_with("motorway") || r.cls == "service" || r.cls.ends_with("link") {
            continue;
        }
        let spacing = 32.0;
        let mut cum = 0.0_f64;
        for w in pts.windows(2) {
            let (a, b) = (w[0], w[1]);
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let len = dx.hypot(dy);
            if len < 1e-6 {
                continue;
            }
            let (ux, uy) = (dx / len, dy / len);
            let mut k = (cum / spacing).ceil() as i64;
            while (k as f64) * spacing < cum + len {
                let s = k as f64 * spacing - cum;
                let side = if (k + ri as i64) % 2 == 0 { 1.0 } else { -1.0 };
                k += 1;
                let (cx, cy) = (a[0] + ux * s, a[1] + uy * s);
                if near_j(cx, cy, r.width / 2.0 + 8.0) {
                    continue;
                }
                let (nx, ny) = (-uy * side, ux * side);
                let off = r.width / 2.0 + 1.3;
                let (px, py) = (cx + nx * off, cy + ny * off);
                out.push(([px, py], [px - nx * 1.8, py - ny * 1.8], [nx, ny]));
            }
            cum += len;
        }
    }
    out
}

/// Lamp head positions (x, y, z) for placing lights at night.
pub fn street_lamps(sc: &Scene) -> Vec<[f64; 3]> {
    lamp_posts(sc).into_iter().map(|(_, h, _)| [h[0], h[1], 6.6]).collect()
}

/// Car body and one wheel, in the car frame (+X forward, +Y left, +Z up) with the ground at z = 0
/// under the body for the body mesh, and the wheel centred on its own origin.
pub fn car_meshes() -> (MeshData, MeshData) {
    use mat::*;
    let mut b = Mb::default();
    b.box_(-2.05, -0.88, 0.30, 2.3, 0.88, 0.92, &[PAINT]);
    let cb = [[-1.25, -0.80, 0.92], [0.75, -0.80, 0.92], [0.75, 0.80, 0.92], [-1.25, 0.80, 0.92]];
    let ct = [[-0.95, -0.68, 1.46], [0.35, -0.68, 1.46], [0.35, 0.68, 1.46], [-0.95, 0.68, 1.46]];
    let c = [-0.2, 0.0, 1.2];
    b.face(&[cb[1], cb[2], ct[2], ct[1]], c, GLASS);
    b.face(&[cb[3], cb[0], ct[0], ct[3]], c, GLASS);
    b.face(&[cb[0], cb[1], ct[1], ct[0]], c, GLASS);
    b.face(&[cb[2], cb[3], ct[3], ct[2]], c, GLASS);
    b.face(&[ct[0], ct[1], ct[2], ct[3]], c, PAINT);
    b.box_(2.28, -0.78, 0.55, 2.38, -0.38, 0.75, &[HEADLIGHT]);
    b.box_(2.28, 0.38, 0.55, 2.38, 0.78, 0.75, &[HEADLIGHT]);
    b.box_(-2.12, -0.78, 0.58, -2.04, -0.38, 0.78, &[TAILLIGHT]);
    b.box_(-2.12, 0.38, 0.58, -2.04, 0.78, 0.78, &[TAILLIGHT]);
    b.box_(-2.0, -0.8, 0.2, 2.25, 0.8, 0.31, &[TIRE]);
    let mut w = Mb::default();
    w.cyl(0.0, 0.0, 0.33, -0.11, 0.11, TIRE, 14);
    w.cyl(0.0, 0.0, 0.2, 0.105, 0.13, RIM, 10);
    w.cyl(0.0, 0.0, 0.2, -0.13, -0.105, RIM, 10);
    (
        MeshData { name: "car_body".into(), verts: b.a, polygon_offset: 0.0, casts_shadow: true, colors: Vec::new() },
        MeshData { name: "car_wheel".into(), verts: w.a, polygon_offset: 0.0, casts_shadow: true, colors: Vec::new() },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Scene;

    const SAMPLE: &str = r#"{"roads":[{"cls":"primary","w":11,"pts":[[-100,0],[0,0],[100,0]],"junction":[false,true,false]},
      {"cls":"residential","w":6.5,"pts":[[0,-80],[0,80]],"junction":[false,false]}],
      "buildings":[{"id":"b","status":"X","fp":[[20,10],[40,10],[40,30],[20,30]],
        "v":[20,10,0,40,10,0,40,30,0,20,30,0,20,10,9,40,10,9,40,30,9,20,30,9],
        "i":[0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7]}],
      "trees":[[5,8]]}"#;

    fn all_finite(m: &MeshData) -> bool {
        m.verts.iter().all(|v| v.is_finite())
    }

    #[test]
    fn scene_meshes_are_well_formed() {
        let sc = Scene::from_json_str(SAMPLE).unwrap();
        let ms = scene_meshes(&sc);
        for m in &ms {
            assert_eq!(m.verts.len() % (FLOATS_PER_VERTEX * 3), 0, "{} not whole triangles", m.name);
            assert!(all_finite(m), "{} has non-finite values", m.name);
        }
        let names: Vec<_> = ms.iter().map(|m| m.name.as_str()).collect();
        for n in ["ground", "sidewalk", "road11", "road6.5", "marks", "buildings", "trees"] {
            assert!(names.contains(&n), "missing mesh {n}: {names:?}");
        }
        assert!(ms.iter().find(|m| m.name == "buildings").unwrap().vertex_count() == 36);
    }

    #[test]
    fn road_ribbon_faces_up_and_has_the_right_width() {
        let mut mb = Mb::default();
        ribbon(&mut mb, &[[0.0, 0.0], [10.0, 0.0]], 3.0, 0.02, mat::ASPHALT);
        assert_eq!(mb.a.len() / 9, 6);
        let ys: Vec<f32> = mb.a.chunks(9).map(|v| v[1]).collect();
        let (lo, hi) = (ys.iter().cloned().fold(f32::MAX, f32::min), ys.iter().cloned().fold(f32::MIN, f32::max));
        assert!((hi - lo - 6.0).abs() < 1e-5);
        // wound counter-clockwise seen from above (geometric normal +Z)
        let p: Vec<V> = (0..3).map(|i| [mb.a[i * 9] as f64, mb.a[i * 9 + 1] as f64, mb.a[i * 9 + 2] as f64]).collect();
        assert!(cross(sub(p[1], p[0]), sub(p[2], p[0]))[2] > 0.0);
    }

    #[test]
    fn face_normals_point_away_from_the_centre() {
        let mut mb = Mb::default();
        mb.box_(0.0, 0.0, 0.0, 2.0, 2.0, 2.0, &[mat::PAINT]);
        for tri in mb.a.chunks(27) {
            let c = [1.0, 1.0, 1.0];
            let p0 = [tri[0] as f64, tri[1] as f64, tri[2] as f64];
            let n = [tri[3] as f64, tri[4] as f64, tri[5] as f64];
            assert!(dot(n, sub(p0, c)) > 0.0);
            // geometric winding agrees with the stored normal
            let p1 = [tri[9] as f64, tri[10] as f64, tri[11] as f64];
            let p2 = [tri[18] as f64, tri[19] as f64, tri[20] as f64];
            assert!(dot(cross(sub(p1, p0), sub(p2, p0)), n) > 0.0);
        }
    }

    #[test]
    fn mitred_corner_stays_finite_even_on_a_hairpin() {
        let mut mb = Mb::default();
        ribbon(&mut mb, &[[0.0, 0.0], [10.0, 0.0], [0.0, 0.001]], 3.0, 0.0, mat::ASPHALT);
        assert!(mb.a.iter().all(|v| v.is_finite()));
        let max = mb.a.chunks(9).map(|v| v[0].abs().max(v[1].abs())).fold(0.0f32, f32::max);
        assert!(max < 30.0, "mitre spike {max}");
    }

    #[test]
    fn car_meshes_exist() {
        let (b, w) = car_meshes();
        assert!(b.vertex_count() > 100 && w.vertex_count() > 100);
        assert!(all_finite(&b) && all_finite(&w));
    }
}
