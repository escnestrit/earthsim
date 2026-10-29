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
    /// Number of facade styles: plaster, brick, panel, glass, wood, metal, tiled panel, silicate brick,
    /// arched (churches, civic), school (wide glazing), shopfront/garage, office ribbon.
    pub const FACADE_STYLES: usize = 12;
    pub fn is_facade(m: f32) -> bool {
        (19.5..25.5).contains(&m) || (29.5..30.5 + (FACADE_STYLES * 3) as f32).contains(&m)
    }
    /// Facade style index 0..FACADE_STYLES and variant 0..3 -> material id.
    pub fn facade(style: usize, variant: usize) -> f32 {
        if variant == 0 && style < 6 {
            20.0 + style as f32
        } else {
            30.0 + (style * 3 + variant) as f32
        }
    }
    /// Inverse of `facade`.
    pub fn facade_parts(m: f32) -> (usize, usize) {
        if m < 29.5 {
            (((m - 20.0).round() as usize).min(5), 0)
        } else {
            let k = (m - 30.0).round() as usize;
            ((k / 3).min(FACADE_STYLES - 1), k % 3)
        }
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
    /// Box turned by `yaw` about Z: centre (cx, cy), length along the heading, width across, z0..z1.
    #[allow(clippy::too_many_arguments)]
    pub fn obox(&mut self, cx: f64, cy: f64, yaw: f64, l: f64, w: f64, z0: f64, z1: f64, m: f32) {
        let (c, s) = (yaw.cos(), yaw.sin());
        let pt = |a: f64, b: f64, z: f64| [cx + a * c - b * s, cy + a * s + b * c, z];
        let (hl, hw) = (l / 2.0, w / 2.0);
        let p = [pt(-hl, -hw, z0), pt(hl, -hw, z0), pt(hl, hw, z0), pt(-hl, hw, z0), pt(-hl, -hw, z1), pt(hl, -hw, z1), pt(hl, hw, z1), pt(-hl, hw, z1)];
        let ctr = [cx, cy, (z0 + z1) / 2.0];
        for f in [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]] {
            let pts: Vec<V> = f.iter().map(|&k| p[k]).collect();
            self.face(&pts, ctr, m);
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
        .filter(|r| !r.bridge)
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
        // each wall holds a whole number of 3 m window bays, stretched to fit, so windows never run round a
        // corner; walls under 1.5 m get none (u stays on the plain strip at the bay edge)
        let mut ustart = vec![0.0f64; n + 1];
        let mut uscale = vec![0.0f64; n];
        let mut dir = vec![[0.0f64; 2]; n];
        if ring_ok {
            for i in 0..n {
                let (a, c) = (corner(i), corner((i + 1) % n));
                let (dx, dy) = (c[0] - a[0], c[1] - a[1]);
                let l = (dx * dx + dy * dy).sqrt();
                let bays = (l / 3.0).round();
                uscale[i] = if bays >= 1.0 && l > 1e-9 { bays * 3.0 / l } else { 0.0 };
                ustart[i + 1] = ustart[i] + bays * 3.0;
                dir[i] = if l > 1e-9 { [dx / l, dy / l] } else { [1.0, 0.0] };
            }
        }
        let vscale = 3.0 / b.facade.floor_height.max(2.0) as f64;
        // start each building at a different bay of the texture (lit-window pattern differs between neighbours)
        let u_off = (b.seed % 4) as f64 * 3.0;
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
                    let along = (q[0] - a[0]) * dir[edge][0] + (q[1] - a[1]) * dir[edge][1];
                    let u = if uscale[edge] > 0.0 { ustart[edge] + along * uscale[edge] } else { ustart[edge] + 0.12 };
                    (u_off + u, q[2] * vscale)
                };
                bm.v(*q, nn, u, v, m);
                bcol.extend_from_slice(&col);
            }
        }
    }
    out.push(MeshData { name: "buildings".into(), verts: bm.a, polygon_offset: 0.0, casts_shadow: true, colors: bcol });
    out.push(building_details(sc));
    out.extend(crate::bridge::meshes(sc));

    // parked cars: body, glazed cabin with a painted roof, wheels; colours in typical proportions
    let mut pm = Mb::default();
    let mut pcol: Vec<f32> = Vec::new();
    let fill = |m: &Mb, col: &mut Vec<f32>, c: [f32; 3]| {
        while col.len() / 3 < m.a.len() / FLOATS_PER_VERTEX {
            col.extend_from_slice(&c);
        }
    };
    for c in &sc.parked {
        let (l, w) = c.size();
        let pal: [[f32; 3]; 10] = [[0.92, 0.92, 0.92], [0.92, 0.92, 0.92], [0.62, 0.64, 0.66], [0.62, 0.64, 0.66], [0.06, 0.06, 0.07], [0.06, 0.06, 0.07], [0.32, 0.34, 0.36], [0.55, 0.06, 0.06], [0.08, 0.16, 0.36], [0.55, 0.50, 0.42]];
        let paint = pal[(c.seed >> 20) as usize % pal.len()];
        let tall = (c.seed % 1000) > 700; // SUV/estate: taller cabin
        let (zb, zt) = (0.32, if tall { 1.0 } else { 0.85 });
        pm.obox(c.x, c.y, c.yaw, l, w, zb, zt, mat::MARKING);
        fill(&pm, &mut pcol, [paint[0] / 0.85, paint[1] / 0.85, paint[2] / 0.82]);
        let (cl, ch) = (l * if tall { 0.62 } else { 0.5 }, if tall { 1.75 } else { 1.42 });
        let back = -l * 0.06;
        let (cx, cy) = (c.x + c.yaw.cos() * back, c.y + c.yaw.sin() * back);
        pm.obox(cx, cy, c.yaw, cl, w * 0.9, zt, ch - 0.06, mat::GLASS);
        fill(&pm, &mut pcol, [1.0, 1.0, 1.0]);
        pm.obox(cx, cy, c.yaw, cl * 0.96, w * 0.86, ch - 0.06, ch, mat::MARKING);
        fill(&pm, &mut pcol, [paint[0] / 0.85, paint[1] / 0.85, paint[2] / 0.82]);
        for (a, b) in [(0.33, 0.5), (0.33, -0.5), (-0.33, 0.5), (-0.33, -0.5)] {
            let (da, db) = (a * l, b * (w - 0.18));
            let (wx, wy) = (c.x + c.yaw.cos() * da - c.yaw.sin() * db, c.y + c.yaw.sin() * da + c.yaw.cos() * db);
            pm.obox(wx, wy, c.yaw, 0.64, 0.2, 0.0, 0.64, mat::TIRE);
        }
        fill(&pm, &mut pcol, [1.0, 1.0, 1.0]);
    }
    out.push(MeshData { name: "parked_cars".into(), verts: pm.a, polygon_offset: 0.0, casts_shadow: true, colors: pcol });

    // entrances: door in the wall, and for doors people use a porch slab and a canopy (the common Soviet
    // and modern block layout); sizes are typical, not measured
    let mut em = Mb::default();
    for e in &sc.entrances {
        let (o, t) = (e.out, [-e.out[1], e.out[0]]);
        let (w, mut canopy) = match e.kind.as_str() {
            "garage" => (2.6, false),
            "service" | "emergency" | "exit" => (1.0, false),
            "main" => (2.0, true),
            _ => (1.4, true),
        };
        let design_canopy = e.building.and_then(|bi| sc.buildings.get(bi as usize)).and_then(|b| b.design.as_ref()).map(|d| d.canopy);
        if design_canopy == Some(crate::model::Canopy::None) {
            canopy = false;
        }
        let gable = design_canopy == Some(crate::model::Canopy::Gable);
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
            if gable {
                // a little pitched roof over the door
                let (hw, dd, z0, rise) = (w / 2.0 + 0.4, 1.0, porch + 2.5, 0.6);
                let ridge0 = at(0.0, 0.0, z0 + rise);
                let ridge1 = at(0.0, dd, z0 + rise);
                em.quad(at(-hw, 0.0, z0), at(-hw, dd, z0), ridge1, ridge0, [-t[0], -t[1], 1.0], mat::ROOF);
                em.quad(at(hw, dd, z0), at(hw, 0.0, z0), ridge0, ridge1, [t[0], t[1], 1.0], mat::ROOF);
                em.tri(at(-hw, dd, z0), at(hw, dd, z0), ridge1, [o[0], o[1], 0.0], mat::ROOF, 0.0);
            } else {
                slab(&mut em, w / 2.0 + 0.5, 1.3, porch + 2.55, porch + 2.7, mat::ROOF);
            }
        }
    }
    out.push(MeshData { name: "entrances".into(), verts: em.a, polygon_offset: 0.0, casts_shadow: true, colors: Vec::new() });

    // trees: broadleaf crowns of 1-3 overlapping clumps or conifers, each with its own size and shade
    let mut tm = Mb::default();
    let mut tcol: Vec<f32> = Vec::new();
    for t in sc.trees.iter() {
        let seed = ((t[0] * 7.31).floor() as i64 as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ ((t[1] * 3.17).floor() as i64 as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F);
        let r = |k: u64| hash01(seed, k);
        let conifer = r(1) < 0.22;
        let scale = 0.75 + r(2) * 0.6;
        let trunk_h = if conifer { 1.2 } else { 1.8 + r(3) * 1.2 } * scale;
        let tw = 0.12 + 0.08 * scale;
        tm.box_(t[0] - tw, t[1] - tw, 0.0, t[0] + tw, t[1] + tw, trunk_h + 0.4, &[mat::TRUNK]);
        let bark = 0.8 + r(4) as f32 * 0.4;
        let n0 = tm.a.len() / FLOATS_PER_VERTEX;
        while tcol.len() / 3 < n0 {
            tcol.extend_from_slice(&[1.0, 1.0, 1.0]);
        }
        let _ = bark;
        // leaf colour: from yellow-green to deep green, conifers darker and bluer
        let (lr, lg, lb) = if conifer { (0.55 + r(5) as f32 * 0.2, 0.75 + r(6) as f32 * 0.2, 0.75 + r(7) as f32 * 0.3) } else { (0.8 + r(5) as f32 * 0.5, 0.85 + r(6) as f32 * 0.35, 0.6 + r(7) as f32 * 0.4) };
        if conifer {
            let hgt = (6.0 + r(8) * 6.0) * scale;
            let rad = hgt * 0.28;
            // two stacked cones
            for (z0, rr, hh) in [(trunk_h, rad, hgt * 0.65), (trunk_h + hgt * 0.4, rad * 0.7, hgt * 0.6)] {
                let segs = 7;
                let apex = [t[0], t[1], z0 + hh];
                for k in 0..segs {
                    let (a0, a1) = (k as f64 / segs as f64 * std::f64::consts::TAU, (k + 1) as f64 / segs as f64 * std::f64::consts::TAU);
                    let p0 = [t[0] + a0.cos() * rr, t[1] + a0.sin() * rr, z0];
                    let p1 = [t[0] + a1.cos() * rr, t[1] + a1.sin() * rr, z0];
                    let am = (a0 + a1) / 2.0;
                    let nrm = norm([am.cos(), am.sin(), rr / hh]);
                    tm.v(p0, nrm, 0.0, 0.0, mat::FOLIAGE);
                    tm.v(p1, nrm, 0.0, 0.0, mat::FOLIAGE);
                    tm.v(apex, nrm, 0.0, 0.0, mat::FOLIAGE);
                    tm.v(p1, [0.0, 0.0, -1.0], 0.0, 0.0, mat::FOLIAGE);
                    tm.v(p0, [0.0, 0.0, -1.0], 0.0, 0.0, mat::FOLIAGE);
                    tm.v([t[0], t[1], z0], [0.0, 0.0, -1.0], 0.0, 0.0, mat::FOLIAGE);
                }
            }
        } else {
            let rr = (1.7 + r(9) * 1.4) * scale;
            let clumps = 1 + (r(10) * 2.99) as usize;
            for c in 0..clumps {
                let ang = r(11 + c as u64) * std::f64::consts::TAU;
                let off = if c == 0 { 0.0 } else { rr * 0.55 };
                let cr = if c == 0 { rr } else { rr * (0.6 + r(14 + c as u64) * 0.25) };
                let cz = trunk_h + rr * 0.85 + if c == 0 { 0.0 } else { rr * (r(17 + c as u64) - 0.3) * 0.6 };
                tm.sphere(t[0] + ang.cos() * off, t[1] + ang.sin() * off, cz, cr, cr * (0.9 + r(20) * 0.3), mat::FOLIAGE, 0.0, 7, 4);
            }
        }
        let n1 = tm.a.len() / FLOATS_PER_VERTEX;
        while tcol.len() / 3 < n1 {
            tcol.extend_from_slice(&[lr, lg, lb]);
        }
    }
    out.push(MeshData { name: "trees".into(), verts: tm.a, polygon_offset: 0.0, casts_shadow: true, colors: tcol });
    out
}

fn hash01(seed: u64, k: u64) -> f64 {
    let mut h = seed ^ k.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    h ^= h >> 31;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 27;
    (h % 10_000) as f64 / 10_000.0
}

/// Exterior elements of buildings. Buildings matched to a typical design (`Building::design`) take them from
/// it: plinth, parapet, balconies/loggias and their rhythm, stairwell windows above each entrance, a low
/// technical top floor, shop fronts. Others get plausible ones from their class, size and seed. Typical
/// shapes, not surveyed.
pub fn building_details(sc: &Scene) -> MeshData {
    use crate::model::{Balconies, StairWindows};
    let mut m = Mb::default();
    let mut col: Vec<f32> = Vec::new();
    let mut balconies = 0usize;
    const MAX_BALCONIES: usize = 3000;
    let push_col = |m: &Mb, col: &mut Vec<f32>, c: [f32; 3]| {
        let n = m.a.len() / FLOATS_PER_VERTEX;
        while col.len() / 3 < n {
            col.extend_from_slice(&c);
        }
    };
    // doors per building
    let mut doors: HashMap<u32, Vec<&crate::model::Entrance>> = HashMap::new();
    for e in &sc.entrances {
        if let Some(bi) = e.building {
            doors.entry(bi).or_default().push(e);
        }
    }
    for (bi, b) in sc.buildings.iter().enumerate() {
        let f = &b.footprint;
        let n = f.len();
        if n < 3 || b.height < 2.5 {
            continue;
        }
        let zs: Vec<f32> = b.verts.chunks(3).map(|v| v[2]).collect();
        let base = zs.iter().cloned().fold(f32::MAX, f32::min) as f64;
        let h = b.height as f64;
        let flat = b.verts.len() / 3 == 2 * n;
        let colour = b.facade.colour;
        let wallc = |k: f32| [colour[0] * k, colour[1] * k, colour[2] * k];
        let d = b.design.as_ref();
        let floor = b.facade.floor_height.max(2.4) as f64;
        let floors = ((h - base) / floor).floor() as i64;
        let my_doors: Vec<&crate::model::Entrance> = doors.get(&(bi as u32)).cloned().unwrap_or_default();
        let edge = |i: usize| {
            let (a, c) = (f[i], f[(i + 1) % n]);
            let (dx, dy) = (c[0] - a[0], c[1] - a[1]);
            let l = (dx * dx + dy * dy).sqrt();
            (a, c, l, if l > 1e-9 { [dy / l, -dx / l] } else { [0.0, 0.0] })
        };
        // positions along edge i of the doors on it
        let doors_on = |i: usize| -> Vec<f64> {
            let (a, c, l, o) = edge(i);
            if l < 1e-6 {
                return Vec::new();
            }
            let dir = [(c[0] - a[0]) / l, (c[1] - a[1]) / l];
            my_doors
                .iter()
                .filter(|e| e.out[0] * o[0] + e.out[1] * o[1] > 0.95)
                .filter_map(|e| {
                    let s = (e.p[0] - a[0]) * dir[0] + (e.p[1] - a[1]) * dir[1];
                    let off = (e.p[0] - a[0]) * o[0] + (e.p[1] - a[1]) * o[1];
                    (s > -0.5 && s < l + 0.5 && off.abs() < 0.5).then_some(s)
                })
                .collect()
        };
        // a wall-coloured panel just proud of the wall hides the texture's windows behind added elements
        let cover = |m: &mut Mb, a: [f64; 2], dir: [f64; 2], o: [f64; 2], s0: f64, s1: f64, z0: f64, z1: f64| {
            let p = |s: f64, z: f64| [a[0] + dir[0] * s + o[0] * 0.025, a[1] + dir[1] * s + o[1] * 0.025, z];
            m.quad(p(s0, z0), p(s1, z0), p(s1, z1), p(s0, z1), [o[0], o[1], 0.0], mat::SIDEWALK);
        };
        // plinth
        let plinth = d.map_or(0.7, |d| d.plinth_h as f64);
        if base < 0.1 && h > 4.0 && plinth > 0.05 {
            for i in 0..n {
                let (a, c, l, o) = edge(i);
                if l < 0.5 {
                    continue;
                }
                let dd = 0.05;
                m.quad([a[0] + o[0] * dd, a[1] + o[1] * dd, 0.0], [c[0] + o[0] * dd, c[1] + o[1] * dd, 0.0], [c[0] + o[0] * dd, c[1] + o[1] * dd, plinth], [a[0] + o[0] * dd, a[1] + o[1] * dd, plinth], [o[0], o[1], 0.0], mat::SIDEWALK);
            }
            push_col(&m, &mut col, wallc(0.55));
        }
        // parapet around flat roofs, with a cap
        let parapet = d.map_or(0.5 + hash01(b.seed, 1) * 0.6, |d| d.parapet_h as f64);
        if flat && parapet > 0.05 && (h > 5.0 && b.kind != "part" || b.kind == "part" && h > 8.0) {
            for i in 0..n {
                let (a, c, l, o) = edge(i);
                if l < 0.3 {
                    continue;
                }
                let t = 0.22;
                let (ai, ci) = ([a[0] - o[0] * t, a[1] - o[1] * t], [c[0] - o[0] * t, c[1] - o[1] * t]);
                m.quad([a[0], a[1], h], [c[0], c[1], h], [c[0], c[1], h + parapet], [a[0], a[1], h + parapet], [o[0], o[1], 0.0], mat::SIDEWALK);
                m.quad([ci[0], ci[1], h], [ai[0], ai[1], h], [ai[0], ai[1], h + parapet], [ci[0], ci[1], h + parapet], [-o[0], -o[1], 0.0], mat::SIDEWALK);
                m.quad([a[0], a[1], h + parapet], [c[0], c[1], h + parapet], [ci[0], ci[1], h + parapet], [ai[0], ai[1], h + parapet], [0.0, 0.0, 1.0], mat::SIDEWALK);
            }
            push_col(&m, &mut col, wallc(0.8));
        }
        crate::landmark::add(&mut m, &mut col, b);
        // machine rooms / stair heads on tall flat roofs (one per section of a designed block)
        if flat && h >= 12.0 && b.landmark == crate::model::Landmark::None {
            let area: f64 = (0..n).map(|i| f[i][0] * f[(i + 1) % n][1] - f[(i + 1) % n][0] * f[i][1]).sum::<f64>().abs() / 2.0;
            if area > 150.0 {
                let inside = |x: f64, y: f64| {
                    let mut ins = false;
                    for i in 0..n {
                        let (p, q) = (f[i], f[(i + 1) % n]);
                        if (p[1] > y) != (q[1] > y) && x < p[0] + (y - p[1]) / (q[1] - p[1]) * (q[0] - p[0]) {
                            ins = !ins;
                        }
                    }
                    ins
                };
                // above each staircase: its head house, 2.5 m in from the door
                let mut spots: Vec<[f64; 2]> = my_doors.iter().filter(|e| e.kind == "staircase").map(|e| [e.p[0] - e.out[0] * 3.5, e.p[1] - e.out[1] * 3.5]).collect();
                if spots.is_empty() {
                    let cx = f.iter().map(|p| p[0]).sum::<f64>() / n as f64;
                    let cy = f.iter().map(|p| p[1]).sum::<f64>() / n as f64;
                    spots.push([cx, cy]);
                }
                for (k, sp) in spots.iter().enumerate() {
                    let (w, dd, bh) = (2.6 + hash01(b.seed, 4 + k as u64) * 1.2, 3.0 + hash01(b.seed, 9) * 1.5, 2.4 + hash01(b.seed, 14) * 0.8);
                    let (x, y) = (sp[0], sp[1]);
                    if inside(x - w / 2.0, y - dd / 2.0) && inside(x + w / 2.0, y + dd / 2.0) && inside(x - w / 2.0, y + dd / 2.0) && inside(x + w / 2.0, y - dd / 2.0) {
                        m.box_(x - w / 2.0, y - dd / 2.0, h, x + w / 2.0, y + dd / 2.0, h + bh, &[mat::SIDEWALK]);
                    }
                }
                push_col(&m, &mut col, wallc(0.75));
            }
        }
        let style = mat::facade_parts(b.facade.style).0;
        let shops = d.map_or(matches!(b.kind.as_str(), "retail" | "commercial" | "supermarket" | "kiosk" | "office"), |d| d.shops);
        // shop fronts: on walls without staircase doors (those face the courtyard)
        if base < 0.1 && shops {
            for i in 0..n {
                let (a, c, l, o) = edge(i);
                if l < 6.0 || !doors_on(i).is_empty() && d.is_some() {
                    continue;
                }
                let dir = [(c[0] - a[0]) / l, (c[1] - a[1]) / l];
                let dd = 0.06;
                let p = |s: f64, z: f64| [a[0] + dir[0] * s + o[0] * dd, a[1] + dir[1] * s + o[1] * dd, z];
                let (s0, s1) = (1.0, l - 1.0);
                let top = floor.min(4.5);
                m.quad(p(s0, plinth.max(0.4)), p(s1, plinth.max(0.4)), p(s1, top - 0.6), p(s0, top - 0.6), [o[0], o[1], 0.0], mat::GLASS);
                push_col(&m, &mut col, [1.0, 1.0, 1.0]);
                let sign = [[0.75, 0.12, 0.10], [0.10, 0.30, 0.65], [0.15, 0.50, 0.25], [0.85, 0.65, 0.10], [0.20, 0.20, 0.22]][((b.seed >> 5) as usize + i) % 5];
                m.quad(p(s0, top - 0.55), p(s1, top - 0.55), p(s1, top - 0.1), p(s0, top - 0.1), [o[0], o[1], 0.0], mat::MARKING);
                push_col(&m, &mut col, [sign[0] / 0.85, sign[1] / 0.85, sign[2] / 0.82]);
            }
        }
        // stairwell windows above staircase doors: a landing window half a storey up, or a glazed band
        let stair = d.map_or(if floors >= 3 && matches!(b.kind.as_str(), "apartments" | "residential" | "dormitory" | "yes") { StairWindows::Landing } else { StairWindows::None }, |d| d.stair);
        if stair != StairWindows::None && floors >= 2 {
            for i in 0..n {
                let (a, c, l, o) = edge(i);
                if l < 3.0 {
                    continue;
                }
                let dir = [(c[0] - a[0]) / l, (c[1] - a[1]) / l];
                for s in doors_on(i) {
                    let bay = d.map_or(3.0, |d| d.bay as f64).max(2.4);
                    let (s0, s1) = ((s - bay / 2.0).max(0.1), (s + bay / 2.0).min(l - 0.1));
                    cover(&mut m, a, dir, o, s0, s1, 2.9, h - 0.15);
                    push_col(&m, &mut col, wallc(0.97));
                    let p = |ss: f64, z: f64| [a[0] + dir[0] * ss + o[0] * 0.05, a[1] + dir[1] * ss + o[1] * 0.05, z];
                    let hw = 0.6;
                    match stair {
                        StairWindows::Band => {
                            m.quad(p(s - hw, 3.2), p(s + hw, 3.2), p(s + hw, h - 0.6), p(s - hw, h - 0.6), [o[0], o[1], 0.0], mat::GLASS);
                            push_col(&m, &mut col, [1.0, 1.0, 1.0]);
                            // mullions every storey
                            for fl in 1..floors {
                                let z = fl as f64 * floor + floor * 0.5;
                                m.quad(p(s - hw, z), p(s + hw, z), p(s + hw, z + 0.08), p(s - hw, z + 0.08), [o[0], o[1], 0.0], mat::MARKING);
                            }
                            push_col(&m, &mut col, [0.95, 0.95, 0.95]);
                        }
                        _ => {
                            for fl in 1..floors.max(1) {
                                // landing between floor fl and fl+1
                                let z0 = fl as f64 * floor + floor * 0.5 - 0.15;
                                if z0 + 1.1 > h - 0.3 {
                                    break;
                                }
                                m.quad(p(s - hw, z0), p(s + hw, z0), p(s + hw, z0 + 1.1), p(s - hw, z0 + 1.1), [o[0], o[1], 0.0], mat::GLASS);
                            }
                            push_col(&m, &mut col, [1.0, 1.0, 1.0]);
                            for fl in 1..floors.max(1) {
                                let z0 = fl as f64 * floor + floor * 0.5 - 0.15;
                                if z0 + 1.1 > h - 0.3 {
                                    break;
                                }
                                // white frame: sill and head
                                m.quad(p(s - hw - 0.08, z0 - 0.08), p(s + hw + 0.08, z0 - 0.08), p(s + hw + 0.08, z0), p(s - hw - 0.08, z0), [o[0], o[1], 0.0], mat::MARKING);
                                m.quad(p(s - hw - 0.08, z0 + 1.1), p(s + hw + 0.08, z0 + 1.1), p(s + hw + 0.08, z0 + 1.18), p(s - hw - 0.08, z0 + 1.18), [o[0], o[1], 0.0], mat::MARKING);
                            }
                            push_col(&m, &mut col, [0.95, 0.95, 0.95]);
                        }
                    }
                }
            }
        }
        // low technical top floor: a plain band with small vents instead of windows
        if d.is_some_and(|d| d.tech_floor) && floors >= 6 && flat {
            let tz = h - 1.8;
            for i in 0..n {
                let (a, c, l, o) = edge(i);
                if l < 2.0 {
                    continue;
                }
                let dir = [(c[0] - a[0]) / l, (c[1] - a[1]) / l];
                cover(&mut m, a, dir, o, 0.0, l, tz, h);
                push_col(&m, &mut col, wallc(0.93));
                let p = |ss: f64, z: f64| [a[0] + dir[0] * ss + o[0] * 0.05, a[1] + dir[1] * ss + o[1] * 0.05, z];
                let mut s = 1.5;
                while s + 0.6 < l - 0.5 {
                    m.quad(p(s, tz + 0.6), p(s + 0.6, tz + 0.6), p(s + 0.6, tz + 1.0), p(s, tz + 1.0), [o[0], o[1], 0.0], mat::GLASS);
                    s += 3.0;
                }
                push_col(&m, &mut col, [1.0, 1.0, 1.0]);
            }
        }
        // balconies / loggias
        let kind = d.map_or(
            {
                let residential = matches!(b.kind.as_str(), "apartments" | "residential" | "dormitory" | "yes") && style != 3 && style != 5;
                if residential && hash01(b.seed, 20) < 0.75 {
                    if hash01(b.seed, 23) < 0.4 { Balconies::Glazed } else { Balconies::Open }
                } else {
                    Balconies::None
                }
            },
            |d| d.balconies,
        );
        let every = d.map_or(1 + (hash01(b.seed, 21) * 2.0) as i64, |d| d.balcony_every.max(1) as i64);
        let sides = d.map_or(0.65, |d| d.balcony_sides as f64);
        if kind != Balconies::None && floors >= 2 && base < 0.1 && balconies < MAX_BALCONIES {
            let bay = d.map_or(3.0, |d| d.bay as f64).max(2.4);
            let depth = if kind == Balconies::Railing { 0.25 } else { 0.9 + hash01(b.seed, 22) * 0.4 };
            for i in 0..n {
                let (a, c, l, o) = edge(i);
                if l < 9.0 || hash01(b.seed, 30 + i as u64) >= sides {
                    continue;
                }
                let dir = [(c[0] - a[0]) / l, (c[1] - a[1]) / l];
                let door_s = doors_on(i);
                let bays = (l / bay).floor() as i64;
                for k in 0..bays {
                    if k % every != 0 || k == 0 || k == bays - 1 {
                        continue;
                    }
                    let s0 = (l - bays as f64 * bay) / 2.0 + k as f64 * bay + 0.3;
                    let s1 = s0 + bay - 0.6;
                    // the stairwell column stays clear
                    if door_s.iter().any(|ds| *ds > s0 - 1.2 && *ds < s1 + 1.2) {
                        continue;
                    }
                    for fl in 1..floors {
                        if balconies >= MAX_BALCONIES {
                            break;
                        }
                        balconies += 1;
                        let z = fl as f64 * floor;
                        let p = |s: f64, dd: f64, zz: f64| [a[0] + dir[0] * s + o[0] * dd, a[1] + dir[1] * s + o[1] * dd, zz];
                        if kind == Balconies::Railing {
                            // French balcony: a dark railing across the window
                            m.quad(p(s0 + 0.3, depth, z + 0.1), p(s1 - 0.3, depth, z + 0.1), p(s1 - 0.3, depth, z + 1.0), p(s0 + 0.3, depth, z + 1.0), [o[0], o[1], 0.0], mat::TIRE);
                            push_col(&m, &mut col, [1.0, 1.0, 1.0]);
                            continue;
                        }
                        m.quad(p(s0, 0.0, z), p(s1, 0.0, z), p(s1, depth, z), p(s0, depth, z), [0.0, 0.0, 1.0], mat::SIDEWALK);
                        m.quad(p(s0, depth, z - 0.15), p(s1, depth, z - 0.15), p(s1, depth, z + 1.0), p(s0, depth, z + 1.0), [o[0], o[1], 0.0], mat::SIDEWALK);
                        m.quad(p(s0, 0.0, z - 0.15), p(s0, depth, z - 0.15), p(s0, depth, z + 1.0), p(s0, 0.0, z + 1.0), [-dir[0], -dir[1], 0.0], mat::SIDEWALK);
                        m.quad(p(s1, depth, z - 0.15), p(s1, 0.0, z - 0.15), p(s1, 0.0, z + 1.0), p(s1, depth, z + 1.0), [dir[0], dir[1], 0.0], mat::SIDEWALK);
                        push_col(&m, &mut col, wallc(0.92));
                        if kind == Balconies::Glazed {
                            m.quad(p(s0, depth, z + 1.0), p(s1, depth, z + 1.0), p(s1, depth, z + floor - 0.2), p(s0, depth, z + floor - 0.2), [o[0], o[1], 0.0], mat::GLASS);
                            push_col(&m, &mut col, [1.0, 1.0, 1.0]);
                        }
                    }
                }
            }
        }
    }
    MeshData { name: "building_details".into(), verts: m.a, polygon_offset: 0.0, casts_shadow: true, colors: col }
}

/// Street lamp posts of a scene: (pole foot, lamp head, unit normal from the road to the pole). Town roads,
/// every ~32 m on alternating sides, none within 8 m of a junction or road end.
pub fn lamp_posts(sc: &Scene) -> Vec<([f64; 2], [f64; 2], [f64; 2])> {
    let roads: Vec<_> = sc.roads.iter().filter(|r| !r.bridge).filter_map(|r| {
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
