//! Node-beam soft-body solver (the kind of model BeamNG.drive uses): point masses joined by spring-damper beams
//! that can yield (permanent deformation) and break, with penalty contact against the ground and static obstacles.
//!
//! Integration is semi-implicit Euler with small fixed steps (the caller picks `dt`, typically 0.25 ms).
use crate::math::*;
use crate::world::{Contact, Ground, Obstacles};

#[derive(Clone, Debug)]
pub struct Node {
    pub p: V3,
    pub v: V3,
    pub m: f64,
    pub f: V3,
    /// Contact radius against the ground and obstacles.
    pub radius: f64,
    pub hit_ground: bool,
    pub hit_world: bool,
    /// Position in the undeformed shape (reference for skinning and damage measurements).
    pub rest: V3,
}

#[derive(Clone, Copy, Debug)]
pub struct Stops {
    pub min_len: f64,
    pub max_len: f64,
    pub k: f64,
    pub c: f64,
}

/// Field order matters: the per-step loop reads the first cache line (a..break_force) of every beam.
#[derive(Clone, Debug)]
#[repr(C)]
pub struct Beam {
    pub a: usize,
    pub b: usize,
    pub rest: f64,
    pub k: f64,
    pub c: f64,
    /// Damping while extending (BeamNG `beamDampRebound`); equals `c` unless set.
    pub c_reb: f64,
    /// Largest tensile / compressive spring force before the beam yields (N). INFINITY = purely elastic.
    pub yield_t: f64,
    pub yield_c: f64,
    /// Elastic force at which the beam snaps (BeamNG `beamStrength`), N.
    pub break_force: f64,
    /// Tensile strain at which the beam snaps.
    pub break_strain: f64,
    pub broken: bool,
    /// Compression-only beam (BeamNG SUPPORT): no force while longer than its rest length.
    pub support: bool,
    pub rest0: f64,
    pub stops: Option<Stops>,
}

/// A node held on a rail (polyline of nodes) by a spring perpendicular to it, free to slide along it
/// (BeamNG `slidenodes`: MacPherson struts, steering racks).
#[derive(Clone, Debug)]
pub struct Slide {
    pub node: usize,
    pub rail: Vec<usize>,
    pub k: f64,
    pub c: f64,
    /// the node cannot slide past the rail ends
    pub capped: bool,
}

/// Two nodes held together by a zero-length spring that lets go above `strength` (door/hood latches,
/// BeamNG couplers). Unlike a beam it has no direction, so it works for coincident nodes.
#[derive(Clone, Debug)]
pub struct Pin {
    pub a: usize,
    pub b: usize,
    pub k: f64,
    pub c: f64,
    pub strength: f64,
    pub broken: bool,
}

#[derive(Clone, Debug, Default)]
pub struct ContactParams {
    pub k: f64,
    pub c: f64,
    pub mu: f64,
}

#[derive(Clone, Copy, Debug)]
struct Hot {
    p: V3,
    v: V3,
    m: f64,
}

#[derive(Clone, Debug)]
pub struct SoftBody {
    pub nodes: Vec<Node>,
    pub beams: Vec<Beam>,
    pub gravity: V3,
    pub ground_contact: ContactParams,
    pub world_contact: ContactParams,
    /// Obstacles stop nodes with an inelastic impulse (no bounce); the beams, not a contact spring, then carry and
    /// dissipate the crash energy. When false a penalty spring is used instead.
    pub world_inelastic: bool,
    /// Largest contact force on any node during the last step (N), for effects.
    pub peak_contact: f64,
    /// Largest obstacle (not ground) contact force during the last step (N).
    pub peak_world: f64,
    /// Work done by yielding beams so far (J).
    pub plastic_work: f64,
    pub diverged: bool,
    pub slides: Vec<Slide>,
    pub pins: Vec<Pin>,
    scratch: Vec<Contact>,
    hot: Vec<Hot>,
    hot_f: Vec<V3>,
}

impl SoftBody {
    pub fn new() -> SoftBody {
        SoftBody {
            nodes: Vec::new(),
            beams: Vec::new(),
            gravity: v3(0.0, 0.0, -9.81),
            ground_contact: ContactParams { k: 4.0e5, c: 3000.0, mu: 0.5 },
            world_contact: ContactParams { k: 2.0e6, c: 6000.0, mu: 0.3 },
            world_inelastic: true,
            peak_contact: 0.0,
            peak_world: 0.0,
            plastic_work: 0.0,
            diverged: false,
            slides: Vec::new(),
            pins: Vec::new(),
            scratch: Vec::new(),
            hot: Vec::new(),
            hot_f: Vec::new(),
        }
    }

    pub fn add_node(&mut self, p: V3, m: f64) -> usize {
        self.nodes.push(Node { p, v: V3::ZERO, m, f: V3::ZERO, radius: 0.0, hit_ground: false, hit_world: false, rest: p });
        self.nodes.len() - 1
    }

    pub fn add_beam(&mut self, a: usize, b: usize, k: f64, c: f64) -> usize {
        let len = (self.nodes[b].p - self.nodes[a].p).len();
        self.beams.push(Beam { a, b, rest: len, rest0: len, k, c, yield_t: f64::INFINITY, yield_c: f64::INFINITY, break_strain: f64::INFINITY, broken: false, stops: None, c_reb: c, support: false, break_force: f64::INFINITY });
        self.beams.len() - 1
    }

    pub fn total_mass(&self) -> f64 {
        self.nodes.iter().map(|n| n.m).sum()
    }

    pub fn centre_of_mass(&self) -> V3 {
        let mut c = V3::ZERO;
        for n in &self.nodes {
            c += n.p * n.m;
        }
        c / self.total_mass()
    }

    pub fn velocity_of_com(&self) -> V3 {
        let mut c = V3::ZERO;
        for n in &self.nodes {
            c += n.v * n.m;
        }
        c / self.total_mass()
    }

    pub fn kinetic_energy(&self) -> f64 {
        self.nodes.iter().map(|n| 0.5 * n.m * n.v.len2()).sum()
    }

    pub fn elastic_energy(&self) -> f64 {
        self.beams
            .iter()
            .filter(|b| !b.broken)
            .map(|b| {
                let d = (self.nodes[b.b].p - self.nodes[b.a].p).len() - b.rest;
                0.5 * b.k * d * d
            })
            .sum()
    }

    pub fn clear_forces(&mut self) {
        for n in self.nodes.iter_mut() {
            n.f = self.gravity * n.m;
        }
    }

    pub fn add_force(&mut self, i: usize, f: V3) {
        self.nodes[i].f += f;
    }

    /// Spring, damper, yield, break and stop forces of every beam.
    pub fn beam_forces(&mut self, dt: f64) {
        // Work on compact copies of the node state: the node structs are large, the beams touch them randomly.
        let mut hot = std::mem::take(&mut self.hot);
        hot.clear();
        hot.extend(self.nodes.iter().map(|n| Hot { p: n.p, v: n.v, m: n.m }));
        let mut f = std::mem::take(&mut self.hot_f);
        f.clear();
        f.resize(self.nodes.len(), V3::ZERO);
        let mut plastic = 0.0;
        for be in self.beams.iter_mut() {
            if be.broken {
                continue;
            }
            let (a, b) = (be.a, be.b);
            // SAFETY-free fast path: indices were validated when the beam was added
            let (na, nb) = (&hot[a], &hot[b]);
            let d = nb.p - na.p;
            let len2 = d.len2();
            if len2 < 1e-18 {
                continue;
            }
            let len = len2.sqrt();
            if be.support && len >= be.rest {
                if be.break_strain.is_finite() && (len - be.rest) / be.rest > be.break_strain {
                    be.broken = true;
                }
                continue;
            }
            let inv = 1.0 / len;
            let dir = d * inv;
            let vrel = (nb.v - na.v).dot(dir);
            let mut f_el = be.k * (len - be.rest);
            if f_el.abs() > be.break_force {
                be.broken = true;
                continue;
            }
            let mut yielding = false;
            // plastic flow: the rest length follows the node distance so that the force stays at the yield limit
            if f_el > be.yield_t {
                let new_rest = len - be.yield_t / be.k;
                plastic += be.yield_t * (new_rest - be.rest).abs();
                be.rest = new_rest;
                f_el = be.yield_t;
                yielding = true;
            } else if f_el < -be.yield_c {
                let new_rest = (len + be.yield_c / be.k).max(0.2 * be.rest0);
                plastic += be.yield_c * (new_rest - be.rest).abs();
                be.rest = new_rest;
                f_el = -be.yield_c;
                yielding = true;
            }
            if be.break_strain.is_finite() && (len - be.rest) > be.break_strain * be.rest {
                be.broken = true;
                continue;
            }
            let mut fsum = f_el;
            if !yielding {
                // damping force limited so one step cannot overshoot the relative velocity (no energy gain)
                let red = na.m * nb.m / (na.m + nb.m);
                let lim = red * vrel.abs() / dt;
                let c = if vrel > 0.0 { be.c_reb } else { be.c };
                fsum += (c * vrel).clamp(-lim, lim);
            }
            if let Some(st) = be.stops {
                if len < st.min_len {
                    fsum -= st.k * (st.min_len - len) + st.c * (-vrel).max(0.0);
                } else if len > st.max_len {
                    fsum += st.k * (len - st.max_len) + st.c * vrel.max(0.0);
                }
            }
            let fv = dir * fsum;
            f[a] += fv;
            f[b] -= fv;
        }
        for (n, fv) in self.nodes.iter_mut().zip(&f) {
            n.f += *fv;
        }
        self.plastic_work += plastic;
        self.hot = hot;
        self.hot_f = f;
    }

    /// Rail springs of the slide nodes and latch pins (call after `beam_forces`).
    pub fn slide_forces(&mut self) {
        for pn in self.pins.iter_mut() {
            if pn.broken {
                continue;
            }
            let (na, nb) = (&self.nodes[pn.a], &self.nodes[pn.b]);
            let f = (nb.p - na.p) * pn.k + (nb.v - na.v) * pn.c;
            if f.len() > pn.strength {
                pn.broken = true;
                continue;
            }
            self.nodes[pn.a].f += f;
            self.nodes[pn.b].f -= f;
        }
        for si in 0..self.slides.len() {
            let sl = &self.slides[si];
            let p = self.nodes[sl.node].p;
            // nearest point on the rail polyline
            let mut best: Option<(f64, usize, f64, V3)> = None;
            for w in 0..sl.rail.len().saturating_sub(1) {
                let (a, b) = (self.nodes[sl.rail[w]].p, self.nodes[sl.rail[w + 1]].p);
                let d = b - a;
                let dd = d.dot(d);
                if dd < 1e-12 {
                    continue;
                }
                let mut t = (p - a).dot(d) / dd;
                let last = w + 2 == sl.rail.len();
                if (sl.capped || w > 0) && t < 0.0 {
                    t = 0.0;
                }
                if (sl.capped || !last) && t > 1.0 {
                    t = 1.0;
                }
                let q = a + d * t;
                let dist = (p - q).len2();
                if best.map_or(true, |bb| dist < bb.0) {
                    best = Some((dist, w, t, q));
                }
            }
            let Some((_, w, t, q)) = best else { continue };
            let (ia, ib) = (sl.rail[w], sl.rail[w + 1]);
            let e = p - q;
            let vq = self.nodes[ia].v * (1.0 - t) + self.nodes[ib].v * t;
            let vrel = self.nodes[sl.node].v - vq;
            // damp only the motion along the error (do not resist sliding along the rail)
            let el = e.len();
            let vn = if el > 1e-9 { e * (vrel.dot(e) / (el * el)) } else { V3::ZERO };
            let f = e * -sl.k - vn * sl.c;
            self.nodes[sl.node].f += f;
            self.nodes[ia].f -= f * (1.0 - t);
            self.nodes[ib].f -= f * t;
        }
    }

    fn friction(vt: V3, fn_: f64, mu: f64, m: f64, dt: f64) -> V3 {
        let s = vt.len();
        if s < 1e-9 {
            return V3::ZERO;
        }
        // regularised Coulomb friction, never more than needed to stop the slip within one step
        let max_stop = m * s / dt;
        let mag = (mu * fn_).min(max_stop) * (s / (s + 0.02)).min(1.0).max(s.min(0.2) / 0.2);
        vt * (-mag / s)
    }

    /// Penalty contacts of the nodes with the ground and static obstacles.
    pub fn contacts(&mut self, ground: &dyn Ground, world: &dyn Obstacles, dt: f64) {
        let gc = self.ground_contact.clone();
        let wc = self.world_contact.clone();
        let mut peak: f64 = 0.0;
        let mut peak_w: f64 = 0.0;
        let mut scratch = std::mem::take(&mut self.scratch);
        for n in self.nodes.iter_mut() {
            if n.hit_ground {
                let s = ground.sample(n.p.x, n.p.y);
                let depth = n.radius - (n.p - v3(n.p.x, n.p.y, s.height)).dot(s.normal);
                if depth > 0.0 {
                    let vn = n.v.dot(s.normal);
                    let fnm = (gc.k * depth - gc.c * vn).max(0.0);
                    let vt = n.v - s.normal * vn;
                    n.f += s.normal * fnm + Self::friction(vt, fnm, gc.mu * s.grip, n.m, dt);
                    peak = peak.max(fnm);
                }
            }
            if n.hit_world && !self.world_inelastic {
                world.query(n.p.x, n.p.y, n.radius, &mut scratch);
                for c in scratch.iter() {
                    let nrm = v3(c.nx, c.ny, 0.0);
                    let vn = n.v.dot(nrm);
                    let fnm = (wc.k * c.depth - wc.c * vn).max(0.0);
                    let vt = n.v - nrm * vn;
                    n.f += nrm * fnm + Self::friction(vt, fnm, wc.mu, n.m, dt);
                    peak = peak.max(fnm);
                    peak_w = peak_w.max(fnm);
                }
            }
        }
        self.scratch = scratch;
        self.peak_contact = peak;
        self.peak_world = peak_w;
    }

    /// Inelastic obstacle response, applied after integration: push nodes out of obstacles, remove their approach
    /// velocity and a Coulomb-limited part of the sliding velocity.
    pub fn resolve_world(&mut self, world: &dyn Obstacles, dt: f64) {
        if !self.world_inelastic {
            return;
        }
        let mu = self.world_contact.mu;
        // nothing nearby: skip the per-node queries (the common case while driving)
        let (mut lo, mut hi) = (V3::splat(f64::MAX), V3::splat(f64::MIN));
        let mut rmax: f64 = 0.0;
        for n in self.nodes.iter().filter(|n| n.hit_world) {
            lo = lo.min(n.p);
            hi = hi.max(n.p);
            rmax = rmax.max(n.radius);
        }
        if lo.x > hi.x || !world.any_near((lo.x + hi.x) * 0.5, (lo.y + hi.y) * 0.5, ((hi.x - lo.x).hypot(hi.y - lo.y)) * 0.5 + rmax + 0.05) {
            return;
        }
        let mut scratch = std::mem::take(&mut self.scratch);
        let mut peak: f64 = 0.0;
        for n in self.nodes.iter_mut() {
            if !n.hit_world {
                continue;
            }
            for _ in 0..2 {
                world.query(n.p.x, n.p.y, n.radius, &mut scratch);
                if scratch.is_empty() {
                    break;
                }
                for c in scratch.iter() {
                    let nrm = v3(c.nx, c.ny, 0.0);
                    n.p += nrm * c.depth;
                    let vn = n.v.dot(nrm);
                    if vn < 0.0 {
                        n.v -= nrm * vn;
                        peak = peak.max(n.m * -vn / dt);
                        let vt = n.v - nrm * n.v.dot(nrm);
                        let s = vt.len();
                        if s > 1e-9 {
                            let dv = (mu * -vn).min(s);
                            n.v -= vt * (dv / s);
                        }
                    }
                }
            }
        }
        self.scratch = scratch;
        self.peak_world = self.peak_world.max(peak);
    }

    pub fn integrate(&mut self, dt: f64) {
        for n in self.nodes.iter_mut() {
            n.v += n.f * (dt / n.m);
            n.p += n.v * dt;
            if !(n.p.is_finite() && n.v.is_finite()) {
                self.diverged = true;
            }
        }
    }

    /// One full step for a body with no vehicle logic (tests and simple objects).
    pub fn step(&mut self, dt: f64, ground: &dyn Ground, world: &dyn Obstacles) {
        self.clear_forces();
        self.beam_forces(dt);
        self.slide_forces();
        self.contacts(ground, world, dt);
        self.integrate(dt);
        self.resolve_world(world, dt);
    }
}

impl Default for SoftBody {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod jbeam_feature_tests {
    use super::*;
    fn pair(len: f64) -> SoftBody {
        let mut b = SoftBody::new();
        b.gravity = V3::ZERO;
        b.add_node(v3(0.0, 0.0, 10.0), 1.0);
        b.add_node(v3(len, 0.0, 10.0), 1.0);
        b.add_beam(0, 1, 1.0e5, 0.0);
        b
    }

    #[test]
    fn support_beams_push_but_never_pull() {
        let mut b = pair(1.0);
        b.beams[0].support = true;
        b.nodes[1].p.x = 1.2; // stretched: no force
        b.clear_forces();
        b.beam_forces(1e-4);
        assert_eq!(b.nodes[1].f.x, 0.0);
        b.nodes[1].p.x = 0.9; // compressed: pushes apart
        b.clear_forces();
        b.beam_forces(1e-4);
        assert!(b.nodes[1].f.x > 9000.0);
    }

    #[test]
    fn beams_snap_at_their_strength() {
        let mut b = pair(1.0);
        b.beams[0].break_force = 5000.0;
        b.nodes[1].p.x = 1.04; // 4000 N
        b.clear_forces();
        b.beam_forces(1e-4);
        assert!(!b.beams[0].broken);
        b.nodes[1].p.x = 1.06; // 6000 N
        b.clear_forces();
        b.beam_forces(1e-4);
        assert!(b.beams[0].broken);
    }

    #[test]
    fn slide_node_stays_on_its_rail_but_slides_freely() {
        let mut b = SoftBody::new();
        b.gravity = V3::ZERO;
        let a = b.add_node(v3(0.0, 0.0, 0.0), 1000.0);
        let c = b.add_node(v3(0.0, 0.0, 1.0), 1000.0);
        let s = b.add_node(v3(0.0, 0.0, 0.5), 1.0);
        b.slides.push(Slide { node: s, rail: vec![a, c], k: 1.0e6, c: 200.0, capped: true });
        // push sideways: the rail spring brings it back; push along: it slides
        b.nodes[s].v = v3(0.5, 0.0, 0.3);
        for _ in 0..4000 {
            b.clear_forces();
            b.slide_forces();
            b.integrate(2.5e-4);
        }
        let p = b.nodes[s].p;
        assert!(p.x.abs() < 0.01, "off the rail by {}", p.x);
        assert!(p.z > 0.75 && p.z <= 1.0 + 0.01, "did not slide / not capped: {}", p.z);
    }

    #[test]
    fn latch_pin_holds_then_lets_go() {
        let mut b = SoftBody::new();
        b.gravity = V3::ZERO;
        let a = b.add_node(v3(0.0, 0.0, 0.0), 1.0);
        let c = b.add_node(v3(0.0, 0.0, 0.0), 1.0);
        b.pins.push(Pin { a, b: c, k: 1.0e5, c: 50.0, strength: 1000.0, broken: false });
        b.nodes[c].p.x = 0.005; // 500 N: holds and pulls back
        b.clear_forces();
        b.slide_forces();
        assert!(b.nodes[c].f.x < -400.0 && !b.pins[0].broken);
        b.nodes[c].p.x = 0.02; // 2000 N: opens
        b.clear_forces();
        b.slide_forces();
        assert!(b.pins[0].broken && b.nodes[c].f.x == 0.0);
    }

    #[test]
    fn rebound_damping_differs_from_bump() {
        let mut b = pair(1.0);
        b.beams[0].k = 0.0;
        b.beams[0].c = 100.0;
        b.beams[0].c_reb = 300.0;
        b.nodes[1].v.x = 0.1; // extending
        b.clear_forces();
        b.beam_forces(1e-4);
        assert!((b.nodes[1].f.x + 30.0).abs() < 1e-9);
        b.nodes[1].v.x = -0.1; // compressing
        b.clear_forces();
        b.beam_forces(1e-4);
        assert!((b.nodes[1].f.x - 10.0).abs() < 1e-9);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::{FlatGround, StaticWorld};
    fn pair(k: f64, c: f64) -> SoftBody {
        let mut b = SoftBody::new();
        b.gravity = V3::ZERO;
        let a = b.add_node(v3(0.0, 0.0, 10.0), 10.0);
        let c_ = b.add_node(v3(1.2, 0.0, 10.0), 10.0);
        b.add_beam(a, c_, k, c);
        b
    }

    #[test]
    fn spring_oscillates_at_the_analytic_frequency_and_conserves_energy() {
        let (k, m) = (2.0e5, 10.0);
        let mut b = pair(k, 0.0);
        b.beams[0].rest = 1.0; // stretched by 0.2
        let (g, w) = (FlatGround { grip: 1.0 }, StaticWorld::new(10.0));
        let e0 = b.elastic_energy();
        let dt = 0.00025;
        let mut crossings = Vec::new();
        let mut prev = b.nodes[1].p.x - b.nodes[0].p.x - 1.0;
        let mut emax: f64 = 0.0;
        for i in 0..40000 {
            b.step(dt, &g, &w);
            let x = b.nodes[1].p.x - b.nodes[0].p.x - 1.0;
            if prev > 0.0 && x <= 0.0 {
                crossings.push(i as f64 * dt);
            }
            prev = x;
            let e = b.kinetic_energy() + b.elastic_energy();
            emax = emax.max((e - e0).abs() / e0);
        }
        let w_an = (2.0 * k / m).sqrt(); // two equal masses: reduced mass m/2
        let period = (crossings[crossings.len() - 1] - crossings[0]) / (crossings.len() - 1) as f64;
        let expect = 2.0 * std::f64::consts::PI / w_an;
        assert!((period - expect).abs() / expect < 0.01, "period {period} vs {expect}");
        // symplectic Euler: energy oscillates by about omega*dt/2 (here ~8%) but must not drift
        assert!(emax < 0.05, "energy error {emax}");
        let e_end = b.kinetic_energy() + b.elastic_energy();
        assert!((e_end - e0).abs() / e0 < 0.06, "secular drift {}", (e_end - e0) / e0);
    }

    #[test]
    fn damping_removes_energy_and_never_adds_it() {
        let mut b = pair(2.0e5, 300.0);
        b.beams[0].rest = 1.0;
        let (g, w) = (FlatGround { grip: 1.0 }, StaticWorld::new(10.0));
        let mut last = b.kinetic_energy() + b.elastic_energy();
        for _ in 0..20000 {
            b.step(0.00025, &g, &w);
            let e = b.kinetic_energy() + b.elastic_energy();
            assert!(e <= last * 1.001 + 1e-9, "energy rose {last} -> {e}");
            last = e;
        }
        assert!(last < 1.0, "should have decayed, E={last}");
    }

    #[test]
    fn yielding_beam_deforms_permanently_and_caps_its_force() {
        let mut b = pair(1.0e6, 0.0);
        b.gravity = V3::ZERO;
        b.beams[0].rest = 1.2;
        b.beams[0].yield_c = 5000.0;
        b.beams[0].yield_t = 5000.0;
        // crush it: push node 1 toward node 0
        b.nodes[1].v = v3(-10.0, 0.0, 0.0);
        b.nodes[0].m = 1e9; // wall
        let (g, w) = (FlatGround { grip: 1.0 }, StaticWorld::new(10.0));
        let mut fmax: f64 = 0.0;
        for _ in 0..8000 {
            b.step(0.00005, &g, &w);
            let len = (b.nodes[1].p - b.nodes[0].p).len();
            fmax = fmax.max(1.0e6 * (b.beams[0].rest - len).abs());
        }
        assert!(b.beams[0].rest < 1.2 - 0.05, "rest length should have shrunk: {}", b.beams[0].rest);
        assert!(b.plastic_work > 300.0, "plastic work {}", b.plastic_work);
        // force at the yield limit (small overshoot allowed within one step)
        assert!(fmax < 5000.0 * 1.2, "force exceeded the limit: {fmax}");
    }

    #[test]
    fn beam_breaks_beyond_its_strain_and_then_carries_no_load() {
        let mut b = pair(1.0e5, 0.0);
        b.beams[0].break_strain = 0.1;
        b.nodes[1].v = v3(30.0, 0.0, 0.0);
        let (g, w) = (FlatGround { grip: 1.0 }, StaticWorld::new(10.0));
        for _ in 0..4000 {
            b.step(0.0001, &g, &w);
        }
        assert!(b.beams[0].broken);
        let v1 = b.nodes[1].v.x;
        let v0 = b.nodes[0].v.x;
        for _ in 0..1000 {
            b.step(0.0001, &g, &w);
        }
        assert!((b.nodes[1].v.x - v1).abs() < 1e-9 && (b.nodes[0].v.x - v0).abs() < 1e-9);
    }

    #[test]
    fn dropped_node_rests_on_the_ground_with_the_right_contact_force() {
        let mut b = SoftBody::new();
        let n = b.add_node(v3(0.0, 0.0, 0.5), 100.0);
        b.nodes[n].radius = 0.1;
        b.nodes[n].hit_ground = true;
        let (g, w) = (FlatGround { grip: 1.0 }, StaticWorld::new(10.0));
        for _ in 0..40000 {
            b.step(0.00025, &g, &w);
        }
        let depth = 0.1 - b.nodes[n].p.z;
        assert!((depth * b.ground_contact.k - 100.0 * 9.81).abs() < 20.0, "depth {depth}");
        assert!(b.nodes[n].v.len() < 1e-3);
    }

    #[test]
    fn sliding_node_decelerates_by_coulomb_friction_and_stops() {
        let mut b = SoftBody::new();
        let n = b.add_node(v3(0.0, 0.0, 0.1), 50.0);
        b.nodes[n].radius = 0.1;
        b.nodes[n].hit_ground = true;
        b.nodes[n].v = v3(4.0, 0.0, 0.0);
        let (g, w) = (FlatGround { grip: 1.0 }, StaticWorld::new(10.0));
        let mu = b.ground_contact.mu;
        let t_stop = 4.0 / (mu * 9.81);
        for i in 0..80000 {
            b.step(0.00025, &g, &w);
            if i as f64 * 0.00025 > t_stop * 0.5 && i as f64 * 0.00025 < t_stop * 0.5 + 0.0003 {
                let v = b.nodes[n].v.x;
                assert!((v - 2.0).abs() < 0.2, "half-time speed {v}");
            }
        }
        assert!(b.nodes[n].v.len() < 0.01 && b.nodes[n].p.x > 1.0 && b.nodes[n].p.x < 2.2, "x {}", b.nodes[n].p.x);
    }

    #[test]
    fn node_cannot_pass_through_a_wall_at_speed() {
        let mut b = SoftBody::new();
        b.gravity = V3::ZERO;
        let n = b.add_node(v3(0.0, 0.0, 1.0), 30.0);
        b.nodes[n].radius = 0.15;
        b.nodes[n].hit_world = true;
        b.nodes[n].v = v3(25.0, 0.0, 0.0);
        let mut w = StaticWorld::new(10.0);
        w.add_poly(vec![(10.0, -5.0), (12.0, -5.0), (12.0, 5.0), (10.0, 5.0)]);
        let g = FlatGround { grip: 1.0 };
        let mut xmax: f64 = 0.0;
        for _ in 0..20000 {
            b.step(0.00025, &g, &w);
            xmax = xmax.max(b.nodes[n].p.x);
        }
        assert!(xmax < 10.0, "node went {xmax}, into the wall");
        assert!(b.nodes[n].v.x.abs() < 0.5, "inelastic contact: it should stay at the wall, v={}", b.nodes[n].v.x);
    }
}
