//! Pedestrians: people walking the pavements of the loaded streets around the player.
//!
//! They walk along the pavement beside each road (2.4 m wide pavements are drawn by scene::mesh), keep to
//! the right, choose a random way at junctions (crossing the street there) and keep clear of the player's
//! car: when it comes close they stop and step back from the kerb, and one it would run into jumps out of
//! the way. Nobody can be hurt. Atmosphere only: no crossings, signals or groups.
use crate::traffic::Graph;
use scene::model::Scene;

#[derive(Clone, Debug)]
pub struct Person {
    pub id: u64,
    pub seed: u64,
    from: u32,
    to: u32,
    s: f64,
    pace: f64,
    /// extra distance away from the road (stepping back from a car), m
    aside: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub yaw: f64,
    /// walking phase (radians) for the drawn stride
    pub phase: f64,
    pub walking: bool,
}

pub struct People {
    graph: Graph,
    pub people: Vec<Person>,
    pub target: usize,
    next_id: u64,
    rng: u64,
}

impl Default for People {
    fn default() -> Self {
        People { graph: Graph::default(), people: Vec::new(), target: 40, next_id: 1, rng: 0xD1B5_4A32_D192_ED03 }
    }
}

const SPAWN_MIN: f64 = 35.0;
const SPAWN_MAX: f64 = 190.0;
const DESPAWN: f64 = 240.0;

impl People {
    fn rand(&mut self) -> f64 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        (self.rng >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Pavement network from the loaded scenes (streets that have pavements).
    pub fn rebuild<'a>(&mut self, scenes: impl Iterator<Item = &'a Scene>) {
        let mut g = Graph::default();
        for sc in scenes {
            for r in &sc.roads {
                if r.cls.starts_with("motorway") || r.cls.starts_with("trunk") || r.bridge && r.width > 12.0 {
                    continue;
                }
                let off = (r.width / 2.0 + if r.cls == "service" { 0.6 } else { 1.3 }) as f32;
                let ids: Vec<u32> = r.pts.iter().map(|p| g.node(*p)).collect();
                for w in ids.windows(2) {
                    g.link(w[0], w[1], 1.0, off);
                    g.link(w[1], w[0], 1.0, off);
                }
            }
        }
        // people keep walking if their pavement is still there
        let old = std::mem::take(&mut self.graph);
        let key = |p: [f64; 2]| ((p[0] * 10.0).round() as i64, (p[1] * 10.0).round() as i64);
        self.people.retain_mut(|h| {
            let (Some(f), Some(t)) = (old.pos.get(h.from as usize), old.pos.get(h.to as usize)) else { return false };
            match (g.index.get(&key(*f)), g.index.get(&key(*t))) {
                (Some(&nf), Some(&nt)) if g.edge(nf, nt).is_some() => {
                    h.from = nf;
                    h.to = nt;
                    true
                }
                _ => false,
            }
        });
        self.graph = g;
    }

    pub fn shift(&mut self, dx: f64, dy: f64) {
        for p in self.graph.pos.iter_mut() {
            p[0] += dx;
            p[1] += dy;
        }
        for h in self.people.iter_mut() {
            h.x += dx;
            h.y += dy;
        }
    }

    fn pose(&self, h: &Person) -> (f64, f64, f64, [f64; 2]) {
        let (p, q) = (self.graph.pos[h.from as usize], self.graph.pos[h.to as usize]);
        let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
        let l = dx.hypot(dy).max(1e-9);
        let off = self.graph.edge(h.from, h.to).map_or(5.0, |e| e.lane as f64) + h.aside;
        let t = (h.s / l).clamp(0.0, 1.0);
        let (ux, uy) = (dx / l, dy / l);
        // right of the walking direction: the pavement on that side
        (p[0] + dx * t + uy * off, p[1] + dy * t - ux * off, dy.atan2(dx), [uy, -ux])
    }

    fn spawn(&mut self, px: f64, py: f64) {
        let n = self.graph.pos.len();
        if n < 2 {
            return;
        }
        for _ in 0..10 {
            let a = (self.rand() * n as f64) as usize % n;
            let pa = self.graph.pos[a];
            let d = (pa[0] - px).hypot(pa[1] - py);
            if !(SPAWN_MIN..SPAWN_MAX).contains(&d) || self.graph.adj[a].is_empty() {
                continue;
            }
            let k = (self.rand() * self.graph.adj[a].len() as f64) as usize % self.graph.adj[a].len();
            let to = self.graph.adj[a][k].to;
            let id = self.next_id;
            self.next_id += 1;
            let seed = id.wrapping_mul(0xA24B_AED4_963E_E407) >> 5;
            let pace = 1.1 + self.rand() * 0.6;
            let s = self.rand() * 8.0;
            let mut h = Person { id, seed, from: a as u32, to, s, pace, aside: 0.0, x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, phase: self.rand() * 6.0, walking: true };
            let (x, y, yaw, _) = self.pose(&h);
            h.x = x;
            h.y = y;
            h.yaw = yaw;
            self.people.push(h);
            return;
        }
    }

    /// Advance by `dt`; `player` = (x, y, heading, speed) of the player's car; `height(x, y, z)` the surface.
    pub fn step(&mut self, dt: f64, player: (f64, f64, f64, f64), height: &dyn Fn(f64, f64, f64) -> f64) {
        let (px, py, pyaw, pspeed) = player;
        self.people.retain(|h| (h.x - px).hypot(h.y - py) < DESPAWN);
        for _ in 0..2 {
            if self.people.len() < self.target {
                self.spawn(px, py);
            }
        }
        let (fx, fy) = (pyaw.cos(), pyaw.sin());
        for i in 0..self.people.len() {
            let mut h = self.people[i].clone();
            // the car: close and coming this way, or just close
            let (rx, ry) = (h.x - px, h.y - py);
            let d = rx.hypot(ry);
            let ahead = rx * fx + ry * fy;
            let threat = d < 4.0 || d < 9.0 && pspeed > 1.0 && ahead > -2.0;
            h.walking = !threat;
            let (_, _, _, side) = self.pose(&h);
            if threat {
                // step back from the kerb, up to 3 m
                h.aside = (h.aside + 2.5 * dt).min(3.0);
            } else {
                h.aside = (h.aside - 0.8 * dt).max(0.0);
                h.s += h.pace * dt;
                h.phase += h.pace * dt * 4.2;
            }
            let (p, q) = (self.graph.pos[h.from as usize], self.graph.pos[h.to as usize]);
            let el = (q[0] - p[0]).hypot(q[1] - p[1]).max(0.01);
            if h.s >= el {
                h.s -= el;
                let at = h.to;
                let opts: Vec<u32> = self.graph.adj[at as usize].iter().map(|e| e.to).filter(|t| *t != h.from).collect();
                let back = h.from;
                h.from = at;
                h.to = if opts.is_empty() {
                    back
                } else {
                    let k = (self.rand() * opts.len() as f64) as usize % opts.len();
                    opts[k]
                };
            }
            let (tx, ty, tyaw, _) = self.pose(&h);
            let k = (dt * 5.0).min(1.0);
            h.x += (tx - h.x) * k;
            h.y += (ty - h.y) * k;
            let mut dyaw = tyaw - h.yaw;
            dyaw = dyaw.sin().atan2(dyaw.cos());
            h.yaw += dyaw * (dt * 5.0).min(1.0);
            // still in the way of the car: jump clear of it
            let (rx, ry) = (h.x - px, h.y - py);
            let d = rx.hypot(ry);
            if d < 2.6 {
                let (nx, ny) = if d > 1e-3 { (rx / d, ry / d) } else { (side[0], side[1]) };
                h.x = px + nx * 2.7;
                h.y = py + ny * 2.7;
            }
            h.z = height(h.x, h.y, h.z + 0.5);
            self.people[i] = h;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scene::model::Road;

    #[test]
    fn people_walk_the_pavements_and_keep_clear_of_the_car() {
        let mut sc = Scene::default();
        for k in -2..=2 {
            let c = k as f64 * 80.0;
            let road = |pts: Vec<[f64; 2]>| Road { cls: "residential".into(), width: 7.0, junction: vec![false; pts.len()], pts, bridge: false, oneway: 0 };
            sc.roads.push(road((-20..=20).map(|i| [i as f64 * 10.0, c]).collect()));
            sc.roads.push(road((-20..=20).map(|i| [c, i as f64 * 10.0]).collect()));
        }
        let mut p = People { target: 30, ..Default::default() };
        p.rebuild(std::iter::once(&sc));
        // the car drives along y = 0 at 10 m/s
        for step in 0..(60 * 40) {
            let t = step as f64 / 60.0;
            let car = (-150.0 + t * 10.0, 0.0, 0.0, 10.0);
            p.step(1.0 / 60.0, car, &|_, _, _| 0.0);
            for h in &p.people {
                assert!((h.x - car.0).hypot(h.y - car.1) > 2.5, "person {} under the car", h.id);
                if step > 120 {
                    // on a pavement: 4.8 m (+ up to 3 m aside) from a street centre line
                    let off = |v: f64| (v - (v / 80.0).round() * 80.0).abs();
                    // crossing at a junction
                    if off(h.x) < 10.0 && off(h.y) < 10.0 || h.x.abs() > 190.0 || h.y.abs() > 190.0 {
                        continue;
                    }
                    assert!(off(h.x) > 3.0 && off(h.x) < 8.5 || off(h.y) > 3.0 && off(h.y) < 8.5, "person {} at ({:.1},{:.1})", h.id, h.x, h.y);
                }
            }
        }
        assert!(p.people.len() >= 20, "{}", p.people.len());
        assert!(p.people.iter().filter(|h| h.walking).count() > 10);
    }
}
