//! Soft-body car: a deformable chassis lattice (node-beam), four hub nodes on beam suspension, and the same tire,
//! brake and drivetrain models as the rigid `Vehicle`.
//!
//! Hybrid design: tyre forces are computed per hub node (using the chassis frame for the wheel heading) and applied
//! at the hub; a force couple on the chassis restores the moment of applying them at the contact patch, so load
//! transfer matches the rigid model. Everything else (body shape, crash damage, suspension travel) comes from beams.
//!
//! Frame: +X forward, +Y left, +Z up. The undeformed shape is defined in "mesh space": x along the car, z measured
//! from the ground under the wheels, so it lines up with the body mesh used for rendering.
use crate::drivetrain::*;
use crate::math::*;
use crate::softbody::*;
use crate::tire::*;
use crate::vehicle::{Input, Wheel, HANDBRAKE_GRIP_LOSS, G, VehicleParams};
use crate::world::*;

pub const STATIONS: [f64; 6] = [-2.05, -1.45, -0.50, 0.45, 1.25, 2.30];
pub const YS: [f64; 3] = [-0.80, 0.0, 0.80];
pub const ZS: [f64; 2] = [0.30, 0.92];
const FRONT_STATION: usize = 4;
const REAR_STATION: usize = 1;
const TOWER_Z: f64 = 0.75;
const HUB_MASS: f64 = 28.0;
const TOWER_MASS: f64 = 5.0;
/// Axial stiffness EA of a chassis beam (N); the spring rate is EA / length.
pub const CHASSIS_EA: f64 = 2.0e6;
const LINK_EA: f64 = 6.0e6;
/// Strength of the crumple zones relative to the passenger cell.
pub const CRUMPLE: f64 = 0.35;
const TIRE_K: f64 = 2.0e5;
const TIRE_C: f64 = 1200.0;

pub fn cage_index(si: usize, yi: usize, zi: usize) -> usize {
    si * 6 + yi * 2 + zi
}

#[derive(Clone, Debug)]
pub struct SoftVehicle {
    pub p: VehicleParams,
    pub body: SoftBody,
    /// Chassis nodes (lattice + strut towers) — everything except the hubs.
    pub cage: Vec<usize>,
    pub hubs: [usize; 4],
    towers: [usize; 4],
    long_anchor: [usize; 4],
    lat_anchor: [usize; 4],
    pub wheels: [Wheel; 4],
    pub drive: Drivetrain,
    pub time: f64,
    pub impact: f64,
    pub accel: V3,
    /// Sprung centre of mass, orientation and velocity, refreshed every step.
    pub pos: V3,
    pub q: Quat,
    pub vel: V3,
    last_vel: V3,
    pub sub_dt: f64,
    pub substeps: usize,
    spring_rest: [f64; 2],
}

fn quat_from_axes(f: V3, l: V3, u: V3) -> Quat {
    // rotation matrix with columns f, l, u -> quaternion (Shepperd)
    let (m00, m10, m20) = (f.x, f.y, f.z);
    let (m01, m11, m21) = (l.x, l.y, l.z);
    let (m02, m12, m22) = (u.x, u.y, u.z);
    let tr = m00 + m11 + m22;
    let q = if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        Quat { w: 0.25 * s, x: (m21 - m12) / s, y: (m02 - m20) / s, z: (m10 - m01) / s }
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        Quat { w: (m21 - m12) / s, x: 0.25 * s, y: (m01 + m10) / s, z: (m02 + m20) / s }
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        Quat { w: (m02 - m20) / s, x: (m01 + m10) / s, y: 0.25 * s, z: (m12 + m21) / s }
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        Quat { w: (m10 - m01) / s, x: (m02 + m20) / s, y: (m12 + m21) / s, z: 0.25 * s }
    };
    q.normalize()
}

impl SoftVehicle {
    pub fn new(p: VehicleParams, x: f64, y: f64, yaw: f64, ground: &dyn Ground) -> SoftVehicle {
        let mut body = SoftBody::new();
        let r_w = p.wheel_radius;
        let cg_z = 0.581;
        let hub_x = [STATIONS[FRONT_STATION], STATIONS[FRONT_STATION], STATIONS[REAR_STATION], STATIONS[REAR_STATION]];
        let hw = p.track * 0.5;
        let hub_y = [hw, -hw, hw, -hw];

        // ---- masses: solve a linear x-gradient and a floor/roof split so the CG is where the rigid model has it ----
        let n_cage = STATIONS.len() * YS.len() * ZS.len();
        let m_cage = p.mass - 4.0 * HUB_MASS - 4.0 * TOWER_MASS;
        let mut pos_rest = Vec::new();
        for &sx in STATIONS.iter() {
            for &sy in YS.iter() {
                for &sz in ZS.iter() {
                    pos_rest.push(v3(sx, sy, sz));
                }
            }
        }
        let fixed_mx: f64 = (0..4).map(|i| HUB_MASS * hub_x[i] + TOWER_MASS * hub_x[i]).sum();
        let fixed_mz: f64 = 4.0 * HUB_MASS * r_w + 4.0 * TOWER_MASS * TOWER_Z;
        let weights = |b: f64, f: f64| -> Vec<f64> {
            pos_rest
                .iter()
                .enumerate()
                .map(|(i, q)| {
                    let zi = i % 2;
                    let yi = (i / 2) % 3;
                    (1.0 + b * q.x) * (0.12 + (-(q.x / 1.2).powi(2)).exp()) * if zi == 0 { f } else { 1.0 } * if yi == 1 { 2.0 } else { 1.0 }
                })
                .collect()
        };
        let moments = |b: f64, f: f64| -> (f64, f64) {
            let w = weights(b, f);
            let sw: f64 = w.iter().sum();
            let mx: f64 = w.iter().zip(&pos_rest).map(|(w, q)| w / sw * m_cage * q.x).sum();
            let mz: f64 = w.iter().zip(&pos_rest).map(|(w, q)| w / sw * m_cage * q.z).sum();
            (mx, mz)
        };
        let (mut b, mut f) = (0.0, 1.0);
        for _ in 0..40 {
            // x: bisection on b (moment increases with b)
            let (mut lo, mut hi) = (-0.3, 0.3);
            for _ in 0..50 {
                let mid = 0.5 * (lo + hi);
                if moments(mid, f).0 + fixed_mx > 0.0 { hi = mid } else { lo = mid }
            }
            b = 0.5 * (lo + hi);
            // z: bisection on f (moment decreases with f)
            let (mut lo, mut hi) = (0.2, 6.0);
            for _ in 0..50 {
                let mid = 0.5 * (lo + hi);
                if moments(b, mid).1 + fixed_mz > p.mass * cg_z { lo = mid } else { hi = mid }
            }
            f = 0.5 * (lo + hi);
        }
        let w = weights(b, f);
        let sw: f64 = w.iter().sum();
        let mut cage = Vec::new();
        for i in 0..n_cage {
            cage.push(body.add_node(pos_rest[i], w[i] / sw * m_cage));
        }
        for &i in &cage {
            body.nodes[i].radius = 0.10;
            body.nodes[i].hit_ground = true;
            body.nodes[i].hit_world = true;
        }

        // ---- lattice beams: every pair of nodes in neighbouring cells (edges, face and space diagonals) ----
        let idx = |si: usize, yi: usize, zi: usize| cage_index(si, yi, zi);
        for si in 0..STATIONS.len() {
            for yi in 0..3 {
                for zi in 0..2 {
                    let a = idx(si, yi, zi);
                    for dsi in -1i32..=1 {
                        for dyi in -1i32..=1 {
                            for dzi in -1i32..=1 {
                                let (s2, y2, z2) = (si as i32 + dsi, yi as i32 + dyi, zi as i32 + dzi);
                                if s2 < 0 || y2 < 0 || z2 < 0 || s2 >= STATIONS.len() as i32 || y2 >= 3 || z2 >= 2 {
                                    continue;
                                }
                                let bnode = idx(s2 as usize, y2 as usize, z2 as usize);
                                if bnode <= a {
                                    continue;
                                }
                                let len = (body.nodes[bnode].p - body.nodes[a].p).len();
                                let k = CHASSIS_EA / len;
                                let m_eff = body.nodes[a].m * body.nodes[bnode].m / (body.nodes[a].m + body.nodes[bnode].m);
                                let c = 2.0 * 0.08 * (k * m_eff).sqrt();
                                let bi = body.add_beam(a, bnode, k, c);
                                let dir = (body.nodes[bnode].p - body.nodes[a].p) / len;
                                // crush rails along the car are strongest; diagonals and cross members yield earlier
                                let along = dir.x.abs();
                                // crumple zones (front and rear of the lattice) yield at a fraction of the cabin's strength
                                let zone = |s: usize| if s >= 4 || s <= 1 { CRUMPLE } else { 1.0 };
                                let f = zone(si).min(zone(s2 as usize));
                                body.beams[bi].yield_c = (12_000.0 + 14_000.0 * along) * f;
                                body.beams[bi].yield_t = (18_000.0 + 14_000.0 * along) * f;
                                body.beams[bi].break_strain = 0.35;
                            }
                        }
                    }
                }
            }
        }

        // ---- hubs, strut towers, suspension ----
        let mut hubs = [0usize; 4];
        let mut towers = [0usize; 4];
        let mut long_anchor = [0usize; 4];
        let mut lat_anchor = [0usize; 4];
        let mut spring_rest = [0.0; 2];
        for i in 0..4 {
            let ax = i / 2;
            let front = ax == 0;
            let left = i % 2 == 0;
            let st = if front { FRONT_STATION } else { REAR_STATION };
            let yi = if left { 2 } else { 0 };
            let yi_opp = 2 - yi;
            let hub = body.add_node(v3(hub_x[i], hub_y[i], r_w - 0.017), HUB_MASS);
            body.nodes[hub].radius = r_w * 0.9;
            body.nodes[hub].hit_world = true;
            hubs[i] = hub;
            let tower = body.add_node(v3(hub_x[i], hub_y[i] * 0.9, TOWER_Z), TOWER_MASS);
            towers[i] = tower;
            cage.push(tower);
            // attach the tower to the six nearest cage nodes
            let mut near: Vec<(f64, usize)> = (0..n_cage).map(|c| ((body.nodes[c].p - body.nodes[tower].p).len(), c)).collect();
            near.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
            for &(len, c) in near.iter().take(6) {
                let k = CHASSIS_EA * 2.0 / len;
                let bi = body.add_beam(tower, c, k, 2.0 * 0.1 * (k * TOWER_MASS).sqrt());
                body.beams[bi].yield_t = 40_000.0;
                body.beams[bi].yield_c = 40_000.0;
                body.beams[bi].break_strain = 0.35;
            }
            // locating links (control arm and lateral link), nearly horizontal at hub height
            let la = idx(if front { FRONT_STATION - 1 } else { REAR_STATION + 1 }, yi, 0);
            let lt = idx(st, yi_opp, 0);
            long_anchor[i] = la;
            lat_anchor[i] = lt;
            for &anchor in &[la, lt] {
                let len = (body.nodes[anchor].p - body.nodes[hub].p).len();
                let k = LINK_EA / len;
                let m_eff = HUB_MASS * body.nodes[anchor].m / (HUB_MASS + body.nodes[anchor].m);
                let bi = body.add_beam(hub, anchor, k, 2.0 * 0.1 * (k * m_eff).sqrt());
                body.beams[bi].yield_t = 45_000.0;
                body.beams[bi].yield_c = 45_000.0;
                body.beams[bi].break_strain = 0.3;
            }
            // spring + damper
            let static_load = p.mass * G * if front { (p.wheelbase - 1.25) / p.wheelbase } else { 1.25 / p.wheelbase } / 2.0 - HUB_MASS * G;
            let len_now = TOWER_Z - (r_w - 0.017);
            let rest = len_now + static_load / p.spring[ax];
            spring_rest[ax] = rest;
            let bi = body.add_beam(hub, tower, p.spring[ax], p.damper[ax]);
            body.beams[bi].rest = rest;
            body.beams[bi].rest0 = rest;
            body.beams[bi].stops = Some(Stops { min_len: len_now - 0.09, max_len: rest + 0.01, k: p.bump_k, c: 2500.0 });
        }
        let drive = Drivetrain::new(&p.drive);
        let mut v = SoftVehicle {
            p,
            body,
            cage,
            hubs,
            towers,
            long_anchor,
            lat_anchor,
            wheels: Default::default(),
            drive,
            time: 0.0,
            impact: 0.0,
            accel: V3::ZERO,
            pos: V3::ZERO,
            q: Quat::IDENT,
            vel: V3::ZERO,
            last_vel: V3::ZERO,
            sub_dt: 0.00025,
            substeps: 4,
            spring_rest,
        };
        v.place(x, y, yaw, 0.0, ground);
        v
    }

    /// Put the (repaired) car on the ground at (x, y) with heading `yaw`, rolling at `speed`.
    pub fn place(&mut self, x: f64, y: f64, yaw: f64, speed: f64, ground: &dyn Ground) {
        let h = ground.sample(x, y).height;
        let q = Quat::from_yaw(yaw);
        let fwd = q.rotate(V3::X);
        for n in self.body.nodes.iter_mut() {
            n.p = q.rotate(n.rest) + v3(x, y, h);
            n.v = fwd * speed;
            n.f = V3::ZERO;
        }
        for b in self.body.beams.iter_mut() {
            b.broken = false;
            b.rest = b.rest0;
        }
        self.body.plastic_work = 0.0;
        self.body.diverged = false;
        for w in self.wheels.iter_mut() {
            *w = Wheel::default();
            w.w = speed / self.p.wheel_radius;
        }
        let dp = &self.p.drive;
        self.drive = Drivetrain::new(dp);
        let mut g = 1;
        while (g as usize) < dp.gears.len() && dp.gears[g as usize - 1] * dp.final_drive * speed / self.p.wheel_radius * 9.549 > 4200.0 {
            g += 1;
        }
        self.drive.gear = g;
        let rpm = (self.drive.ratio(dp).abs() * speed / self.p.wheel_radius * 9.549).max(dp.idle_rpm);
        self.drive.w_e = rpm / 9.549;
        self.time = 0.0;
        self.impact = 0.0;
        self.accel = V3::ZERO;
        self.refresh_frame();
        self.last_vel = self.vel;
    }

    /// Chassis axes from the lattice (robust to local crumpling): (forward, left, up).
    pub fn axes(&self) -> (V3, V3, V3) {
        let mean = |f: &dyn Fn(usize, usize, usize) -> bool| -> V3 {
            let mut s = V3::ZERO;
            let mut n = 0.0;
            for si in 0..STATIONS.len() {
                for yi in 0..3 {
                    for zi in 0..2 {
                        if f(si, yi, zi) {
                            s += self.body.nodes[cage_index(si, yi, zi)].p;
                            n += 1.0;
                        }
                    }
                }
            }
            s / n
        };
        let front = mean(&|si, _, _| si >= 4);
        let rear = mean(&|si, _, _| si <= 1);
        let left = mean(&|_, yi, _| yi == 2);
        let right = mean(&|_, yi, _| yi == 0);
        let f = (front - rear).norm();
        let up = f.cross((left - right).norm()).norm();
        let l = up.cross(f).norm();
        (f, l, up)
    }

    fn refresh_frame(&mut self) {
        let (f, l, u) = self.axes();
        self.q = quat_from_axes(f, l, u);
        let (mut c, mut v, mut m) = (V3::ZERO, V3::ZERO, 0.0);
        for &i in &self.cage {
            let n = &self.body.nodes[i];
            c += n.p * n.m;
            v += n.v * n.m;
            m += n.m;
        }
        self.pos = c / m;
        self.vel = v / m;
    }

    pub fn forward(&self) -> V3 {
        self.q.rotate(V3::X)
    }

    /// Largest distance of a chassis node from where it would be if the body were undamaged (m), and the count of
    /// beams that have snapped.
    pub fn damage(&self) -> (f64, usize) {
        let (mut c, mut rc, mut m) = (V3::ZERO, V3::ZERO, 0.0);
        for &i in &self.cage {
            let n = &self.body.nodes[i];
            c += n.p * n.m;
            rc += n.rest * n.m;
            m += n.m;
        }
        let (c, rc) = (c / m, rc / m);
        let mut worst: f64 = 0.0;
        for &i in &self.cage {
            let n = &self.body.nodes[i];
            let ideal = c + self.q.rotate(n.rest - rc);
            worst = worst.max((n.p - ideal).len());
        }
        (worst, self.body.beams.iter().filter(|b| b.broken).count())
    }

    /// Advance by `dt` seconds (a multiple of `sub_dt`; 1 ms = 4 substeps).
    pub fn step(&mut self, dt: f64, inp: &Input, ground: &dyn Ground, world: &dyn Obstacles) {
        let n = ((dt / self.sub_dt).round() as usize).max(1);
        let h = dt / n as f64;
        let mut impact: f64 = 0.0;
        for _ in 0..n {
            self.substep(h, inp, ground, world);
            impact = impact.max(self.body.peak_world);
        }
        self.impact = if impact > 3000.0 { 1000.0 } else { 0.0 };
        self.refresh_frame();
        // filtered specific force (gravity removed) for telemetry
        let a = (self.vel - self.last_vel) / dt - v3(0.0, 0.0, -G);
        self.last_vel = self.vel;
        let spec = self.q.inv_rotate(a);
        let k = (dt / 0.1).min(1.0);
        if spec.is_finite() {
            self.accel = self.accel * (1.0 - k) + spec * k;
        }
        self.time += dt;
    }

    fn substep(&mut self, dt: f64, inp: &Input, ground: &dyn Ground, world: &dyn Obstacles) {
        self.refresh_frame();
        let (fwd, left, up) = self.axes();
        let r_w = self.p.wheel_radius;
        self.body.clear_forces();
        self.body.beam_forces(dt);

        // aerodynamic drag, spread over the chassis
        let sp = self.vel.len();
        let drag = self.vel * (-0.5 * 1.2 * self.p.cd_a * sp);
        let m_cage: f64 = self.cage.iter().map(|&i| self.body.nodes[i].m).sum();
        for &i in &self.cage.clone() {
            let m = self.body.nodes[i].m;
            self.body.nodes[i].f += drag * (m / m_cage);
        }

        // ---- drivetrain ----
        let mut throttle = inp.throttle;
        let (da, db) = self.p.driven;
        if inp.tcs && inp.handbrake < 0.1 {
            let k = self.wheels[da].kappa.max(self.wheels[db].kappa);
            if self.wheels[da].contact || self.wheels[db].contact {
                throttle *= (1.0 - (k - 0.22) / 0.15).clamp(0.0, 1.0);
            }
        }
        let dout = self.drive.step(dt, &self.p.drive, throttle, inp.reverse, self.wheels[da].w, self.wheels[db].w);

        // ---- steering (Ackermann) ----
        let t = inp.steer_angle.tan();
        let half = self.p.track * 0.5;
        let l = self.p.wheelbase;
        let steer_l = (l * t).atan2(l - half * t);
        let steer_r = (l * t).atan2(l + half * t);

        // ---- tyres at the hubs ----
        for i in 0..4 {
            let front = i < 2;
            let delta = if front { if i == 0 { steer_l } else { steer_r } } else { 0.0 };
            self.wheels[i].steer = delta;
            let hub = self.hubs[i];
            let (hp, hv) = (self.body.nodes[hub].p, self.body.nodes[hub].v);
            let s = ground.sample(hp.x, hp.y);
            let n = s.normal;
            let pen = r_w - (hp - v3(hp.x, hp.y, s.height)).dot(n);
            let contact = pen > 0.0;

            let mut t_drive = 0.0;
            let mut stiff = 0.0;
            if i == da {
                t_drive = dout.torque_l;
                stiff = dout.stiffness;
            } else if i == db {
                t_drive = dout.torque_r;
                stiff = dout.stiffness;
            }
            let mut tb = inp.brake * if front { self.p.brake_front } else { self.p.brake_rear };
            if inp.abs && inp.brake > 0.05 && self.wheels[i].contact {
                let slip = -self.wheels[i].kappa;
                if slip > 0.12 {
                    tb *= (1.0 - (slip - 0.12) / 0.10).clamp(0.05, 1.0);
                }
            }
            if !front {
                tb += inp.handbrake * self.p.handbrake * (1.0 - 0.75 * throttle);
            }
            let w0 = self.wheels[i].w;
            let mut w_new;
            let mut fz = 0.0;
            let mut out = TireOut::default();
            let mut pc = hp;
            if contact {
                let vn = hv.dot(n);
                fz = (TIRE_K * pen - TIRE_C * vn).max(0.0);
                pc = hp - n * r_w;
                let head = fwd * delta.cos() + left * delta.sin();
                let f_p = (head - n * head.dot(n)).norm();
                let lat = n.cross(f_p);
                let (vl, vt) = (hv.dot(f_p), hv.dot(lat));
                let grip = s.grip * if front { 1.0 } else { 1.0 - HANDBRAKE_GRIP_LOSS * inp.handbrake };
                let mut tp = self.p.tire;
                if tb > 0.0 && w0.abs() < 0.5 && sp < 1.0 {
                    tp.v0 = 0.4;
                }
                let o1 = tire_force(&tp, fz, grip, vl, vt, w0, r_w);
                let o2 = tire_force(&tp, fz, grip, vl, vt, w0 + 1.0, r_w);
                let jx = (o2.fx - o1.fx).max(0.0);
                let denom = self.p.wheel_inertia + dt * (r_w * jx + stiff);
                let pre = w0 + dt * (t_drive - r_w * o1.fx) / denom;
                w_new = apply_brake(pre, tb, dt, self.p.wheel_inertia);
                out = tire_force(&tp, fz, grip, vl, vt, w_new, r_w);
                let rr = -self.p.crr * fz * (vl / 0.3).tanh();
                let f_t = f_p * (out.fx + rr) + lat * out.fy;
                self.body.nodes[hub].f += n * fz + f_t;
                // couple restoring the patch-level moment on the chassis (see module docs)
                let mc = (pc - hp).cross(f_t);
                let (ml, mf) = (mc.dot(left), mc.dot(fwd));
                let tw = self.towers[i];
                let (la, lt) = (self.long_anchor[i], self.lat_anchor[i]);
                let dx = (self.body.nodes[tw].p - self.body.nodes[la].p).dot(fwd);
                let dy = (self.body.nodes[tw].p - self.body.nodes[lt].p).dot(left);
                if dx.abs() > 0.2 {
                    let fv = up * (-ml / dx);
                    self.body.nodes[tw].f += fv;
                    self.body.nodes[la].f -= fv;
                }
                if dy.abs() > 0.2 {
                    let fv = up * (mf / dy);
                    self.body.nodes[tw].f += fv;
                    self.body.nodes[lt].f -= fv;
                }
            } else {
                let pre = w0 + dt * t_drive / (self.p.wheel_inertia + dt * stiff);
                w_new = apply_brake(pre, tb, dt, self.p.wheel_inertia);
            }
            if !w_new.is_finite() {
                w_new = 0.0;
            }
            let wh = &mut self.wheels[i];
            wh.w = w_new;
            wh.spin += w_new * dt;
            wh.fz = fz;
            wh.contact = contact;
            wh.kappa = out.kappa;
            wh.tan_alpha = out.tan_alpha;
            wh.fx = out.fx;
            wh.fy = out.fy;
            wh.locked = tb > 0.0 && w_new == 0.0;
            wh.pc = pc;
            // suspension compression as the rigid model reports it (wheel centre below its mount, along body up)
            let mount_b = self.p.mounts[i];
            let hub_b = self.q.inv_rotate(hp - self.pos);
            wh.comp = (hub_b.z - mount_b.z + self.p.susp_rest).max(0.0);
        }

        // anti-roll bars: force proportional to the left/right compression difference, acting between hub and chassis
        for ax in 0..2 {
            let (l, r) = (ax * 2, ax * 2 + 1);
            let f = self.p.arb[ax] * (self.wheels[l].comp - self.wheels[r].comp);
            let fu = up * f;
            self.body.nodes[self.towers[l]].f += fu;
            self.body.nodes[self.hubs[l]].f -= fu;
            self.body.nodes[self.towers[r]].f -= fu;
            self.body.nodes[self.hubs[r]].f += fu;
        }

        self.body.contacts(ground, world, dt);
        self.body.integrate(dt);
        self.body.resolve_world(world, dt);
    }

    /// Where the wheel centre is (for rendering).
    pub fn wheel_center(&self, i: usize) -> V3 {
        self.body.nodes[self.hubs[i]].p
    }

    pub fn spring_rest(&self) -> [f64; 2] {
        self.spring_rest
    }
}

fn apply_brake(w: f64, tb: f64, dt: f64, i: f64) -> f64 {
    if tb <= 0.0 {
        return w;
    }
    let dw = dt * tb / i;
    if w.abs() <= dw {
        0.0
    } else {
        w - w.signum() * dw
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::Vehicle;

    fn flat() -> FlatGround {
        FlatGround { grip: 1.0 }
    }

    fn sim(c: &mut SoftVehicle, secs: f64, inp: &Input, w: &dyn Obstacles) {
        for _ in 0..(secs * 1000.0) as usize {
            c.step(0.001, inp, &flat(), w);
        }
    }

    fn car() -> SoftVehicle {
        SoftVehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &flat())
    }

    #[test]
    fn mass_properties_match_the_rigid_model() {
        let c = car();
        assert!((c.body.total_mass() - 1450.0).abs() < 1e-6);
        let com = c.body.centre_of_mass();
        assert!(com.x.abs() < 0.01, "cg x {}", com.x);
        assert!((com.z - 0.581).abs() < 0.005, "cg z {}", com.z);
        // inertia from the point masses
        let (mut ix, mut iy, mut iz) = (0.0, 0.0, 0.0);
        for n in &c.body.nodes {
            let r = n.p - com;
            ix += n.m * (r.y * r.y + r.z * r.z);
            iy += n.m * (r.x * r.x + r.z * r.z);
            iz += n.m * (r.x * r.x + r.y * r.y);
        }
        let p = VehicleParams::sedan().inertia;
        assert!((ix / p.x - 1.0).abs() < 0.5 && (iy / p.y - 1.0).abs() < 0.25 && (iz / p.z - 1.0).abs() < 0.25, "inertia {ix:.0} {iy:.0} {iz:.0} vs {p:?}");
    }

    #[test]
    fn settles_on_its_springs_without_damage_or_drift() {
        let mut c = car();
        sim(&mut c, 4.0, &Input::default(), &StaticWorld::new(10.0));
        let load: f64 = c.wheels.iter().map(|w| w.fz).sum();
        assert!((load - 1450.0 * G).abs() / (1450.0 * G) < 0.01, "load {load}");
        assert!(c.vel.len() < 0.01, "moving {}", c.vel.len());
        assert!(c.pos.z > 0.55 && c.pos.z < 0.64, "cg z {}", c.pos.z);
        let front = c.wheels[0].fz + c.wheels[1].fz;
        let rear = c.wheels[2].fz + c.wheels[3].fz;
        assert!(front > rear);
        let (dmg, broken) = c.damage();
        assert!(dmg < 0.005 && broken == 0, "dmg {dmg} broken {broken}");
        assert!(c.body.beams.iter().all(|b| (b.rest - b.rest0).abs() < 1e-9 || b.stops.is_some()), "nothing may yield under its own weight");
    }

    #[test]
    fn accelerates_like_the_rigid_car() {
        let inp = Input { throttle: 1.0, tcs: true, abs: true, ..Default::default() };
        let g = flat();
        let w = StaticWorld::new(10.0);
        let mut s = car();
        let mut t: f64 = 0.0;
        while s.vel.dot(s.forward()) < 27.78 && t < 30.0 {
            s.step(0.001, &inp, &g, &w);
            t += 0.001;
        }
        let mut r = Vehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
        let mut tr: f64 = 0.0;
        while r.telemetry().forward_speed < 27.78 && tr < 30.0 {
            r.step(0.001, &inp, &g, &w);
            tr += 0.001;
        }
        assert!((t / tr - 1.0).abs() < 0.08, "soft {t:.2} s rigid {tr:.2} s");
        assert!(s.body.beams.iter().all(|b| !b.broken));
    }

    #[test]
    fn braking_distance_and_pitch_are_sane() {
        let g = flat();
        let w = StaticWorld::new(10.0);
        let mut s = car();
        sim(&mut s, 1.5, &Input::default(), &w);
        s.place(0.0, 0.0, 0.0, 27.78, &g);
        let inp = Input { brake: 1.0, tcs: true, abs: true, ..Default::default() };
        let mut pitch: f64 = 0.0;
        for _ in 0..8000 {
            s.step(0.001, &inp, &g, &w);
            pitch = pitch.max(s.forward().z.abs().asin());
            if s.vel.len() < 0.1 {
                break;
            }
        }
        assert!(s.pos.x > 30.0 && s.pos.x < 46.0, "stopping distance {}", s.pos.x);
        assert!(pitch.to_degrees() < 5.0, "pitch {}", pitch.to_degrees());
        assert!(s.vel.len() < 0.2);
        assert!(s.damage().0 < 0.01);
    }

    #[test]
    fn cornering_matches_the_rigid_car() {
        let g = flat();
        let w = StaticWorld::new(10.0);
        let mut s = car();
        let mut r = Vehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
        s.place(0.0, 0.0, 0.0, 15.0, &g);
        r.place(0.0, 0.0, 0.0, 15.0, &g);
        let inp = Input { throttle: 0.15, steer_angle: 0.08, tcs: true, abs: true, ..Default::default() };
        for _ in 0..4000 {
            s.step(0.001, &inp, &g, &w);
            r.step(0.001, &inp, &g, &w);
        }
        let (gs, gr) = (s.accel.y / G, r.accel.y / G);
        assert!((gs / gr - 1.0).abs() < 0.2, "lat g soft {gs:.2} rigid {gr:.2}");
        let roll = s.q.rotate(V3::Y).z.asin().to_degrees().abs();
        assert!(roll < 4.0, "roll {roll}");
    }

    fn crash(speed: f64, wall_x: f64) -> (SoftVehicle, f64, f64, f64) {
        let g = flat();
        let mut wall = StaticWorld::new(10.0);
        wall.add_poly(vec![(wall_x, -10.0), (wall_x + 2.0, -10.0), (wall_x + 2.0, 10.0), (wall_x, 10.0)]);
        let mut c = car();
        c.place(0.0, 0.0, 0.0, speed, &g);
        let none = Input { tcs: true, abs: true, ..Default::default() };
        let mut peak_g: f64 = 0.0;
        let mut last = c.vel.x;
        let mut front: f64 = 0.0;
        let mut ke_hit = 0.0;
        for i in 0..3500 {
            c.step(0.001, &none, &g, &wall);
            if i % 5 == 4 {
                peak_g = peak_g.max(((c.vel.x - last) / 0.005).abs() / G);
                last = c.vel.x;
            }
            for &k in &c.cage {
                front = front.max(c.body.nodes[k].p.x);
            }
            if ke_hit == 0.0 && c.body.plastic_work > 0.0 {
                ke_hit = 0.5 * 1450.0 * c.vel.x * c.vel.x;
            }
        }
        (c, peak_g, front, ke_hit)
    }

    #[test]
    fn frontal_crash_at_50_kmh_crumples_absorbs_energy_and_stays_outside_the_wall() {
        let (c, peak_g, front, ke_hit) = crash(13.9, 30.0);
        assert!(!c.body.diverged);
        assert!(front < 30.0, "a node got into the wall: {front}");
        assert!(c.vel.len() < 1.5, "should have stopped, v = {}", c.vel.len());
        let (dmg, _) = c.damage();
        assert!(dmg > 0.25 && dmg < 1.0, "crush {dmg}");
        assert!(c.body.plastic_work > 0.6 * ke_hit, "plastic work {} J of {} J", c.body.plastic_work, ke_hit);
        assert!(peak_g < 60.0, "peak decel {peak_g} g");
        // the passenger cell is much less damaged than the nose: rear cell nodes stay close to their ideal place
        let rear_dev: f64 = (0..3)
            .flat_map(|yi| (0..2).map(move |zi| (yi, zi)))
            .map(|(yi, zi)| {
                let i = cage_index(2, yi, zi);
                let n = &c.body.nodes[i];
                let (mut cc, mut rc, mut m) = (V3::ZERO, V3::ZERO, 0.0);
                for &k in &c.cage {
                    cc += c.body.nodes[k].p * c.body.nodes[k].m;
                    rc += c.body.nodes[k].rest * c.body.nodes[k].m;
                    m += c.body.nodes[k].m;
                }
                (n.p - (cc / m + c.q.rotate(n.rest - rc / m))).len()
            })
            .fold(0.0, f64::max);
        assert!(rear_dev < dmg * 0.9, "cabin {rear_dev} vs nose {dmg}");
    }

    #[test]
    fn harder_crashes_do_more_damage_and_nothing_explodes() {
        let slow = crash(8.0, 30.0).0;
        let fast = crash(25.0, 60.0).0;
        assert!(!fast.body.diverged && !slow.body.diverged);
        assert!(fast.damage().0 > slow.damage().0 + 0.1, "slow {} fast {}", slow.damage().0, fast.damage().0);
        assert!(fast.body.nodes.iter().all(|n| n.p.is_finite() && n.v.len() < 100.0));
        assert!(slow.damage().0 < 0.3, "a 29 km/h bump should not wreck the car: {}", slow.damage().0);
    }

    #[test]
    fn a_tree_hits_the_nose_between_the_corner_nodes() {
        let g = flat();
        let mut w = StaticWorld::new(10.0);
        w.add_circle(25.0, 0.0, 0.35);
        let mut c = car();
        c.place(0.0, 0.0, 0.0, 12.0, &g);
        let none = Input { tcs: true, abs: true, ..Default::default() };
        let mut front: f64 = 0.0;
        for _ in 0..3000 {
            c.step(0.001, &none, &g, &w);
            // the centre-line nodes of the nose are the ones the trunk meets
            for si in 0..STATIONS.len() {
                for zi in 0..2 {
                    front = front.max(c.body.nodes[cage_index(si, 1, zi)].p.x);
                }
            }
        }
        assert!(front < 25.0, "tree was ignored, nose reached {front}");
        assert!(c.damage().0 > 0.1);
    }

    #[test]
    fn long_noisy_drive_stays_stable_and_undamaged() {
        let g = flat();
        let w = StaticWorld::new(10.0);
        let mut c = car();
        let mut seed = 12345u64;
        let mut rnd = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 33) as f64 / (1u64 << 31) as f64) * 2.0 - 1.0
        };
        let mut inp = Input { tcs: true, abs: true, ..Default::default() };
        let mut maxv: f64 = 0.0;
        for i in 0..60_000 {
            if i % 400 == 0 {
                inp.throttle = (rnd() * 0.8 + 0.4).clamp(0.0, 1.0);
                inp.brake = if rnd() > 0.6 { rnd().abs() } else { 0.0 };
                inp.steer_angle = rnd() * 0.3;
                inp.handbrake = if rnd() > 0.9 { 1.0 } else { 0.0 };
            }
            c.step(0.001, &inp, &g, &w);
            maxv = maxv.max(c.vel.len());
            assert!(!c.body.diverged, "diverged at step {i}");
        }
        let (dmg, broken) = c.damage();
        assert!(dmg < 0.02 && broken == 0, "dmg {dmg} broken {broken}");
        assert!(maxv > 5.0, "never really drove ({maxv})");
        assert!(c.q.rotate(V3::Z).z > 0.9);
    }

    #[test]
    fn dropping_the_car_half_a_metre_lands_without_breaking_it() {
        let g = flat();
        let mut c = car();
        for n in c.body.nodes.iter_mut() {
            n.p.z += 0.5;
        }
        sim(&mut c, 4.0, &Input::default(), &StaticWorld::new(10.0));
        assert!(!c.body.diverged);
        assert!(c.vel.len() < 0.05 && c.q.rotate(V3::Z).z > 0.999);
        let (dmg, broken) = c.damage();
        assert!(dmg < 0.03 && broken == 0, "dmg {dmg} broken {broken}");
        let _ = g;
    }

    #[test]
    fn place_repairs_the_car() {
        let g = flat();
        let (mut c, ..) = crash(13.9, 30.0);
        assert!(c.damage().0 > 0.2);
        c.place(0.0, 0.0, 0.0, 0.0, &g);
        assert!(c.damage().0 < 1e-9 && c.body.beams.iter().all(|b| !b.broken));
    }
}
