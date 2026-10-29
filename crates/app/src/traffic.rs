//! Traffic: simple cars driving the loaded road network around the player.
//!
//! The network is built from the road centre lines of the loaded tiles (points shared by two roads are
//! junctions; the same road drawn by two neighbouring tiles joins up because its points coincide). Cars keep
//! to the right-hand lane, follow the car or player ahead, slow down for turns, take turns crossing a
//! junction, and pick a random onward road at each junction. They are not physical: when the player hits
//! one, it stops and becomes a fixed obstacle (a wreck) until it is far away. No traffic lights, signs,
//! lane changes or one-way streets (the data carries no `oneway` yet), so this is atmosphere, not a
//! traffic model.
use std::collections::HashMap;

use scene::model::Scene;

/// Obstacle tags of wrecked traffic cars (one per car: TAG_BASE + id).
pub const TAG_BASE: u64 = 0xFFFF_0000_0000_0000;

#[derive(Clone, Copy, Debug)]
struct Edge {
    to: u32,
    /// speed limit (m/s)
    vmax: f32,
    /// lateral offset of the right-hand lane from the centre line (m)
    lane: f32,
}

#[derive(Default)]
struct Graph {
    pos: Vec<[f64; 2]>,
    adj: Vec<Vec<Edge>>,
    index: HashMap<(i64, i64), u32>,
}

impl Graph {
    fn node(&mut self, p: [f64; 2]) -> u32 {
        let k = ((p[0] * 10.0).round() as i64, (p[1] * 10.0).round() as i64);
        if let Some(&i) = self.index.get(&k) {
            return i;
        }
        let i = self.pos.len() as u32;
        self.pos.push(p);
        self.adj.push(Vec::new());
        self.index.insert(k, i);
        i
    }
    fn link(&mut self, a: u32, b: u32, vmax: f32, lane: f32) {
        if a == b || self.adj[a as usize].iter().any(|e| e.to == b) {
            return;
        }
        self.adj[a as usize].push(Edge { to: b, vmax, lane });
    }
    fn edge(&self, a: u32, b: u32) -> Option<Edge> {
        self.adj[a as usize].iter().find(|e| e.to == b).copied()
    }
}

/// Speed limit by road class (m/s), or None for roads without through traffic.
fn vmax(cls: &str) -> Option<f32> {
    Some(match cls {
        "motorway" | "trunk" => 25.0,
        "motorway_link" | "trunk_link" => 15.0,
        "primary" | "secondary" => 15.0,
        "primary_link" | "secondary_link" | "tertiary" | "tertiary_link" => 13.0,
        "unclassified" | "residential" => 10.0,
        "living_street" => 5.0,
        _ => return None,
    })
}

#[derive(Clone, Debug)]
pub struct Agent {
    pub id: u64,
    pub seed: u64,
    from: u32,
    to: u32,
    next: Option<u32>,
    /// distance travelled along the current edge
    s: f64,
    pub speed: f64,
    /// drawn pose (smoothed): x, y, z, heading
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub yaw: f64,
    pub crashed: bool,
    wait: f64,
}

impl Agent {
    pub fn outline(&self) -> Vec<(f64, f64)> {
        scene::model::ParkedCar { x: self.x, y: self.y, yaw: self.yaw, seed: self.seed }.outline()
    }
    pub fn length(&self) -> f64 {
        scene::model::ParkedCar { x: 0.0, y: 0.0, yaw: 0.0, seed: self.seed }.size().0
    }
}

/// Result of one step for the game: wrecks to add to / remove from the obstacles.
#[derive(Default, Debug)]
pub struct Events {
    pub wrecked: Vec<(u64, Vec<(f64, f64)>)>,
    pub cleared: Vec<u64>,
}

pub struct Traffic {
    graph: Graph,
    pub agents: Vec<Agent>,
    /// how many cars to keep around the player
    pub target: usize,
    next_id: u64,
    rng: u64,
    /// junction node -> agent crossing it
    held: HashMap<u32, u64>,
}

/// Plan-view size assumed for the player's car (m).
const PLAYER_L: f64 = 4.6;
const PLAYER_W: f64 = 1.9;

/// Do two rotated rectangles (centre x, y, heading, length, width) overlap? Separating axis test.
fn boxes_overlap(a: (f64, f64, f64, f64, f64), b: (f64, f64, f64, f64, f64)) -> bool {
    let corners = |r: (f64, f64, f64, f64, f64)| {
        let (c, s) = (r.2.cos(), r.2.sin());
        [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].map(|(i, j)| (r.0 + i * r.3 / 2.0 * c - j * r.4 / 2.0 * s, r.1 + i * r.3 / 2.0 * s + j * r.4 / 2.0 * c))
    };
    let (ca, cb) = (corners(a), corners(b));
    for yaw in [a.2, a.2 + std::f64::consts::FRAC_PI_2, b.2, b.2 + std::f64::consts::FRAC_PI_2] {
        let (ax, ay) = (yaw.cos(), yaw.sin());
        let proj = |cs: &[(f64, f64); 4]| cs.iter().map(|p| p.0 * ax + p.1 * ay).fold((f64::MAX, f64::MIN), |m, v| (m.0.min(v), m.1.max(v)));
        let (pa, pb) = (proj(&ca), proj(&cb));
        if pa.1 < pb.0 || pb.1 < pa.0 {
            return false;
        }
    }
    true
}

const SPAWN_MIN: f64 = 110.0;
const SPAWN_MAX: f64 = 380.0;
const DESPAWN: f64 = 460.0;

impl Default for Traffic {
    fn default() -> Self {
        Traffic { graph: Graph::default(), agents: Vec::new(), target: 30, next_id: 1, rng: 0x9E37_79B9_7F4A_7C15, held: HashMap::new() }
    }
}

impl Traffic {
    fn rand(&mut self) -> f64 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        (self.rng >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Rebuild the road network from the loaded scenes. Cars whose edge vanished are removed.
    pub fn rebuild<'a>(&mut self, scenes: impl Iterator<Item = &'a Scene>, events: &mut Events) {
        let old_pos = std::mem::take(&mut self.graph.pos);
        let mut g = Graph::default();
        for sc in scenes {
            for r in &sc.roads {
                let Some(v) = vmax(&r.cls) else { continue };
                let lane = (r.width / 4.0).clamp(1.4, 3.6) as f32;
                let ids: Vec<u32> = r.pts.iter().map(|p| g.node(*p)).collect();
                // one-way streets: a narrow one is driven in the middle
                let lane = if r.oneway != 0 && r.width < 6.0 { 0.0 } else { lane };
                for w in ids.windows(2) {
                    if r.oneway >= 0 {
                        g.link(w[0], w[1], v, lane);
                    }
                    if r.oneway <= 0 {
                        g.link(w[1], w[0], v, lane);
                    }
                }
            }
        }
        // re-attach cars to the new node numbers
        let mut keep = Vec::new();
        for mut a in std::mem::take(&mut self.agents) {
            let (pf, pt) = (old_pos.get(a.from as usize).copied(), old_pos.get(a.to as usize).copied());
            let key = |p: [f64; 2]| ((p[0] * 10.0).round() as i64, (p[1] * 10.0).round() as i64);
            let ok = match (pf, pt) {
                (Some(f), Some(t)) => match (g.index.get(&key(f)), g.index.get(&key(t))) {
                    (Some(&nf), Some(&nt)) if g.edge(nf, nt).is_some() => {
                        a.from = nf;
                        a.to = nt;
                        a.next = None;
                        true
                    }
                    _ => false,
                },
                _ => false,
            };
            if ok {
                keep.push(a);
            } else if a.crashed {
                events.cleared.push(a.id);
            }
        }
        self.agents = keep;
        self.held.clear();
        self.graph = g;
    }

    /// Rebuild after the local frame moved by (dx, dy): positions shift, the network is rebuilt by the caller.
    pub fn shift(&mut self, dx: f64, dy: f64) {
        for p in self.graph.pos.iter_mut() {
            p[0] += dx;
            p[1] += dy;
        }
        for a in self.agents.iter_mut() {
            a.x += dx;
            a.y += dy;
        }
    }

    fn pick_next(&mut self, from: u32, at: u32) -> Option<u32> {
        let p = self.graph.pos[at as usize];
        let q = self.graph.pos[from as usize];
        let din = [p[0] - q[0], p[1] - q[1]];
        let ln = din[0].hypot(din[1]).max(1e-9);
        let opts: Vec<(u32, f64)> = self.graph.adj[at as usize]
            .iter()
            .filter(|e| e.to != from)
            .map(|e| {
                let r = self.graph.pos[e.to as usize];
                let d = [r[0] - p[0], r[1] - p[1]];
                let c = (d[0] * din[0] + d[1] * din[1]) / (ln * d[0].hypot(d[1]).max(1e-9));
                // straight on is likelier than a turn; sharp turns rare
                (e.to, if c > 0.7 { 3.0 } else if c > -0.3 { 1.0 } else { 0.15 })
            })
            .collect();
        if opts.is_empty() {
            return None;
        }
        let total: f64 = opts.iter().map(|o| o.1).sum();
        let mut x = self.rand() * total;
        for (n, w) in &opts {
            x -= w;
            if x <= 0.0 {
                return Some(*n);
            }
        }
        Some(opts[0].0)
    }

    fn spawn(&mut self, px: f64, py: f64) -> bool {
        let n = self.graph.pos.len();
        if n < 2 {
            return false;
        }
        for _ in 0..12 {
            let a = (self.rand() * n as f64) as u32 % n as u32;
            let pa = self.graph.pos[a as usize];
            let d = (pa[0] - px).hypot(pa[1] - py);
            if !(SPAWN_MIN..SPAWN_MAX).contains(&d) || self.graph.adj[a as usize].is_empty() {
                continue;
            }
            if self.agents.iter().any(|o| (o.x - pa[0]).hypot(o.y - pa[1]) < 15.0) {
                continue;
            }
            let k = (self.rand() * self.graph.adj[a as usize].len() as f64) as usize % self.graph.adj[a as usize].len();
            let e = self.graph.adj[a as usize][k];
            let id = self.next_id;
            self.next_id += 1;
            let seed = (id.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0x51_7CC1_B727_220A) >> 3;
            let mut ag = Agent { id, seed, from: a, to: e.to, next: None, s: 0.0, speed: e.vmax as f64 * 0.8, x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, crashed: false, wait: 0.0 };
            let (x, y, yaw) = self.pose(&ag);
            ag.x = x;
            ag.y = y;
            ag.yaw = yaw;
            self.agents.push(ag);
            return true;
        }
        false
    }

    /// Lane position and heading of an agent on its edge.
    fn pose(&self, a: &Agent) -> (f64, f64, f64) {
        let (p, q) = (self.graph.pos[a.from as usize], self.graph.pos[a.to as usize]);
        let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
        let l = dx.hypot(dy).max(1e-9);
        let lane = self.graph.edge(a.from, a.to).map_or(1.6, |e| e.lane as f64);
        let t = (a.s / l).clamp(0.0, 1.0);
        let (ux, uy) = (dx / l, dy / l);
        (p[0] + dx * t + uy * lane, p[1] + dy * t - ux * lane, dy.atan2(dx))
    }

    /// Advance by `dt`. `player` is the player's car (x, y, heading, speed); `height(x, y, z)` the surface.
    pub fn step(&mut self, dt: f64, player: (f64, f64, f64, f64), height: &dyn Fn(f64, f64, f64) -> f64) -> Events {
        let mut ev = Events::default();
        let (px, py, _pyaw, pspeed) = player;
        // despawn far cars, spawn new ones
        let mut gone = Vec::new();
        self.agents.retain(|a| {
            let far = (a.x - px).hypot(a.y - py) > DESPAWN;
            if far {
                gone.push((a.id, a.crashed));
            }
            !far
        });
        for (id, crashed) in gone {
            self.held.retain(|_, h| *h != id);
            if crashed {
                ev.cleared.push(id);
            }
        }
        let mut tries = 0;
        while self.agents.len() < self.target && tries < 3 {
            tries += 1;
            self.spawn(px, py);
        }
        let snapshot: Vec<(u64, f64, f64)> = self.agents.iter().map(|a| (a.id, a.x, a.y)).collect();
        let mut remove = Vec::new();
        for i in 0..self.agents.len() {
            let mut a = self.agents[i].clone();
            if a.crashed {
                continue;
            }
            let (p, q) = (self.graph.pos[a.from as usize], self.graph.pos[a.to as usize]);
            let el = (q[0] - p[0]).hypot(q[1] - p[1]);
            let e = self.graph.edge(a.from, a.to);
            let mut want = e.map_or(8.0, |e| e.vmax as f64);
            if a.next.is_none() {
                a.next = self.pick_next(a.from, a.to);
            }
            // slow for the turn at the end of this edge
            let to_end = el - a.s;
            if let Some(n) = a.next {
                let r = self.graph.pos[n as usize];
                let (d0, d1) = ([q[0] - p[0], q[1] - p[1]], [r[0] - q[0], r[1] - q[1]]);
                let c = (d0[0] * d1[0] + d0[1] * d1[1]) / (d0[0].hypot(d0[1]) * d1[0].hypot(d1[1])).max(1e-9);
                if c < 0.7 && to_end < 25.0 {
                    want = want.min(5.5 + 6.0 * (to_end / 25.0));
                }
            } else if to_end < 20.0 {
                want = want.min(to_end * 0.4);
            }
            // keep a gap to whatever is ahead in the lane, the player included
            let (hx, hy) = (a.yaw.cos(), a.yaw.sin());
            let mut gap = f64::MAX;
            let mut check = |x: f64, y: f64, len: f64| {
                let (rx, ry) = (x - a.x, y - a.y);
                let ahead = rx * hx + ry * hy;
                let lat = (rx * hy - ry * hx).abs();
                if ahead > 0.5 && ahead < 45.0 && lat < 2.4 {
                    gap = gap.min(ahead - len);
                }
            };
            for &(id, x, y) in &snapshot {
                if id != a.id {
                    check(x, y, 4.8);
                }
            }
            check(px, py, 3.0);
            if gap < f64::MAX {
                want = want.min(((gap - 3.0) * 0.6).max(0.0));
            }
            // junctions: one car at a time within 12 m of a node where three or more roads meet
            let busy_node = self.graph.adj[a.to as usize].len() >= 3;
            if busy_node && to_end < 12.0 {
                match self.held.get(&a.to) {
                    Some(&h) if h != a.id && a.wait < 5.0 => {
                        want = want.min(((to_end - 8.0) * 0.8).max(0.0));
                        a.wait += dt;
                    }
                    _ => {
                        self.held.insert(a.to, a.id);
                    }
                }
            }
            // accelerate gently, brake hard
            let dv = want - a.speed;
            a.speed = (a.speed + dv.clamp(-7.0 * dt, 2.2 * dt)).max(0.0);
            a.s += a.speed * dt;
            while a.s >= el.max(0.01) {
                a.s -= el.max(0.01);
                let Some(n) = a.next else {
                    remove.push(a.id);
                    break;
                };
                if self.held.get(&a.from) == Some(&a.id) {
                    self.held.remove(&a.from);
                }
                a.from = a.to;
                a.to = n;
                a.wait = 0.0;
                a.next = self.pick_next(a.from, a.to);
                break;
            }
            // release a junction once well past it
            if a.s > 10.0 && self.held.get(&a.from) == Some(&a.id) {
                self.held.remove(&a.from);
            }
            let (tx, ty, tyaw) = self.pose(&a);
            // smooth the drawn pose so lane offsets do not jump at corners
            let k = (dt * 8.0).min(1.0);
            a.x += (tx - a.x) * k;
            a.y += (ty - a.y) * k;
            let mut dyaw = tyaw - a.yaw;
            dyaw = dyaw.sin().atan2(dyaw.cos());
            a.yaw += dyaw * (dt * 6.0).min(1.0);
            a.z = height(a.x, a.y, a.z + 0.6);
            // the player ran into it: it stops where it is and becomes an obstacle
            if (a.x - px).hypot(a.y - py) < a.length() / 2.0 + 3.0 && pspeed > 0.5 {
                let o = a.outline();
                let size = scene::model::ParkedCar { x: 0.0, y: 0.0, yaw: 0.0, seed: a.seed }.size();
                if boxes_overlap((a.x, a.y, a.yaw, size.0, size.1), (px, py, player.2, PLAYER_L, PLAYER_W)) {
                    a.crashed = true;
                    a.speed = 0.0;
                    self.held.retain(|_, h| *h != a.id);
                    ev.wrecked.push((a.id, o));
                }
            }
            self.agents[i] = a;
        }
        if !remove.is_empty() {
            self.agents.retain(|a| !remove.contains(&a.id));
            self.held.retain(|_, h| !remove.contains(h));
        }
        ev
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scene::model::Road;

    fn grid() -> Scene {
        let mut sc = Scene::default();
        let road = |pts: Vec<[f64; 2]>| Road { cls: "residential".into(), width: 7.0, junction: vec![false; pts.len()], pts, bridge: false, oneway: 0 };
        for k in -3..=3 {
            let c = k as f64 * 100.0;
            sc.roads.push(road((-30..=30).map(|i| [i as f64 * 10.0, c]).collect()));
            sc.roads.push(road((-30..=30).map(|i| [c, i as f64 * 10.0]).collect()));
        }
        sc
    }

    #[test]
    fn cars_drive_the_network_keep_their_distance_and_stay_on_roads() {
        let sc = grid();
        let mut t = Traffic { target: 25, ..Default::default() };
        let mut ev = Events::default();
        t.rebuild(std::iter::once(&sc), &mut ev);
        let flat = |_: f64, _: f64, _: f64| 0.0;
        let mut moved = 0.0;
        for step in 0..(60 * 60) {
            let before: HashMap<u64, (f64, f64)> = t.agents.iter().map(|a| (a.id, (a.x, a.y))).collect();
            let e = t.step(1.0 / 60.0, (0.0, 0.0, 0.0, 0.0), &flat);
            assert!(e.wrecked.is_empty());
            for a in &t.agents {
                if let Some(b) = before.get(&a.id) {
                    moved += (a.x - b.0).hypot(a.y - b.1);
                }
                // on a road of the grid: near a multiple of 100 m in x or y (lane offset 1.75 m)
                let off = |v: f64| (v - (v / 100.0).round() * 100.0).abs();
                if step > 120 {
                    assert!(off(a.x) < 4.0 || off(a.y) < 4.0, "car {} off the road at ({:.1}, {:.1})", a.id, a.x, a.y);
                }
            }
            if step % 60 == 0 && step > 600 {
                // nobody drives through anybody on the same lane
                for a in &t.agents {
                    for b in &t.agents {
                        if a.id < b.id && (a.yaw - b.yaw).sin().abs() < 0.2 && (a.yaw - b.yaw).cos() > 0.0 {
                            assert!((a.x - b.x).hypot(a.y - b.y) > 2.0, "cars {} and {} overlap", a.id, b.id);
                        }
                    }
                }
            }
        }
        assert!(t.agents.len() >= 15, "{} cars", t.agents.len());
        let mean = moved / t.agents.len() as f64 / 60.0;
        assert!(mean > 3.0, "cars move: {mean:.1} m/s on average");
    }

    #[test]
    fn one_way_streets_are_driven_one_way() {
        let mut sc = Scene::default();
        let pts: Vec<[f64; 2]> = (-30..=30).map(|i| [i as f64 * 10.0, 0.0]).collect();
        sc.roads.push(Road { cls: "residential".into(), width: 7.0, junction: vec![false; pts.len()], pts, bridge: false, oneway: -1 });
        let mut t = Traffic { target: 6, ..Default::default() };
        t.rebuild(std::iter::once(&sc), &mut Events::default());
        for _ in 0..600 {
            t.step(1.0 / 60.0, (0.0, 0.0, 0.0, 0.0), &|_, _, _| 0.0);
        }
        assert!(!t.agents.is_empty());
        for a in &t.agents {
            // against the points: westwards
            assert!(a.yaw.cos() < -0.9, "car {} heading {}", a.id, a.yaw);
        }
    }

    #[test]
    fn hitting_a_car_wrecks_it() {
        let mut sc = Scene::default();
        sc.roads.push(Road { cls: "residential".into(), width: 7.0, junction: vec![false; 61], pts: (-30..=30).map(|i| [i as f64 * 10.0, 0.0]).collect(), bridge: false, oneway: 0 });
        let mut t = Traffic { target: 1, ..Default::default() };
        t.rebuild(std::iter::once(&sc), &mut Events::default());
        t.step(0.016, (0.0, 0.0, 0.0, 0.0), &|_, _, _| 0.0);
        let a = t.agents[0].clone();
        // the player is right on it, moving
        // passing in the other lane is not a hit
        let ev = t.step(0.016, (a.x, a.y + 3.6, a.yaw + std::f64::consts::PI, 10.0), &|_, _, _| 0.0);
        assert!(ev.wrecked.is_empty());
        let ev = t.step(0.016, (a.x, a.y, 0.0, 10.0), &|_, _, _| 0.0);
        assert_eq!(ev.wrecked.len(), 1);
        assert!(t.agents[0].crashed);
        // far away: cleared
        let ev = t.step(0.016, (a.x + 2000.0, a.y, 0.0, 0.0), &|_, _, _| 0.0);
        assert_eq!(ev.cleared, vec![a.id]);
    }
}
