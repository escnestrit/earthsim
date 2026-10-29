//! Rigid-body car: 6-DOF body, four spring/damper suspension corners with ground probing,
//! combined-slip tires, per-wheel spin dynamics (brakes and handbrake lock wheels), drivetrain.
//!
//! Frame: +X forward, +Y left, +Z up (body); world is +Z up. Units: SI.
use crate::drivetrain::*;
use crate::math::*;
use crate::tire::*;
use crate::world::*;

pub const G: f64 = 9.81;
/// Fraction of rear tyre grip lost while the handbrake is fully applied.
pub const HANDBRAKE_GRIP_LOSS: f64 = 0.45;

#[derive(Clone, Debug)]
pub struct VehicleParams {
    pub mass: f64,
    /// Principal moments of inertia about the CG (Ixx roll, Iyy pitch, Izz yaw).
    pub inertia: V3,
    /// Suspension top mounts in the body frame, order FL, FR, RL, RR.
    pub mounts: [V3; 4],
    pub wheel_radius: f64,
    pub wheel_inertia: f64,
    /// Strut length from mount to wheel centre at full droop.
    pub susp_rest: f64,
    /// Front / rear spring rate (N/m), damping (N·s/m), anti-roll bar (N/m of left-right compression difference).
    pub spring: [f64; 2],
    pub damper: [f64; 2],
    pub arb: [f64; 2],
    pub bump_start: f64,
    pub bump_k: f64,
    pub tire: TireParams,
    /// Max brake torque per wheel (N·m).
    pub brake_front: f64,
    pub brake_rear: f64,
    /// Handbrake torque per rear wheel (N·m).
    pub handbrake: f64,
    /// Drag area Cd·A (m²) and rolling resistance coefficient.
    pub cd_a: f64,
    pub crr: f64,
    pub wheelbase: f64,
    pub track: f64,
    pub steer_max: f64,
    pub drive: DrivetrainParams,
    /// Indices of the two driven wheels.
    pub driven: (usize, usize),
    /// Collision discs: body-frame centre and radius.
    pub collision: Vec<(V3, f64)>,
}

impl VehicleParams {
    /// A 1450 kg rear-wheel-drive sedan, ~105 kW.
    pub fn sedan() -> Self {
        let (a, b, hw, mz) = (1.25, 1.45, 0.78, -0.05);
        let mut col = Vec::new();
        for &x in &[2.0, 1.0, 0.0, -1.0, -2.0] {
            for &y in &[0.62, -0.62] {
                col.push((v3(x, y, 0.1), 0.38));
            }
        }
        VehicleParams {
            mass: 1450.0,
            inertia: v3(520.0, 2250.0, 2400.0),
            mounts: [v3(a, hw, mz), v3(a, -hw, mz), v3(-b, hw, mz), v3(-b, -hw, mz)],
            wheel_radius: 0.33,
            wheel_inertia: 1.4,
            susp_rest: 0.30,
            spring: [35000.0, 38000.0],
            damper: [3200.0, 3000.0],
            arb: [16000.0, 9000.0],
            bump_start: 0.20,
            bump_k: 250000.0,
            tire: TireParams::default(),
            brake_front: 1700.0,
            brake_rear: 1000.0,
            handbrake: 1500.0,
            cd_a: 0.75,
            crr: 0.014,
            wheelbase: a + b,
            track: 2.0 * hw,
            steer_max: 0.55,
            drive: DrivetrainParams::default(),
            driven: (2, 3),
            collision: col,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Input {
    pub throttle: f64,
    pub brake: f64,
    /// Road-wheel steering angle, positive = left (rad).
    pub steer_angle: f64,
    pub handbrake: f64,
    pub reverse: bool,
    pub tcs: bool,
    pub abs: bool,
}

#[derive(Clone, Debug, Default)]
pub struct Wheel {
    /// Spin, rad/s (positive = rolling forward).
    pub w: f64,
    /// Accumulated spin angle (rad), for rendering.
    pub spin: f64,
    /// Steering angle actually applied (rad).
    pub steer: f64,
    /// Suspension compression (m).
    pub comp: f64,
    pub fz: f64,
    pub contact: bool,
    pub kappa: f64,
    pub tan_alpha: f64,
    pub fx: f64,
    pub fy: f64,
    pub locked: bool,
    pub pc: V3,
}

#[derive(Clone, Debug)]
pub struct Vehicle {
    pub p: VehicleParams,
    pub pos: V3,
    pub q: Quat,
    pub vel: V3,
    pub omega: V3,
    pub wheels: [Wheel; 4],
    pub drive: Drivetrain,
    pub time: f64,
    /// Peak collision impulse of the last step (N·s), for effects.
    pub impact: f64,
    /// Specific force (what an accelerometer would read) in the body frame, low-pass filtered over ~0.1 s (m/s²).
    pub accel: V3,
    scratch: Vec<Contact>,
}

#[derive(Clone, Copy, Debug)]
pub struct Telemetry {
    pub speed: f64,
    pub forward_speed: f64,
    pub side_slip: f64,
    pub rpm: f64,
    pub gear: i32,
    pub lat_g: f64,
}

impl Vehicle {
    pub fn new(p: VehicleParams, x: f64, y: f64, yaw: f64, ground: &dyn Ground) -> Self {
        let drive = Drivetrain::new(&p.drive);
        let mut v = Vehicle {
            pos: v3(x, y, ground.sample(x, y).height + 0.60),
            q: Quat::from_yaw(yaw),
            vel: V3::ZERO,
            omega: V3::ZERO,
            wheels: Default::default(),
            drive,
            time: 0.0,
            impact: 0.0,
            accel: V3::ZERO,
            scratch: Vec::new(),
            p,
        };
        v.place(x, y, yaw, 0.0, ground);
        v
    }

    /// Teleport and optionally give an initial forward speed (wheels and engine are spun up to match).
    pub fn place(&mut self, x: f64, y: f64, yaw: f64, speed: f64, ground: &dyn Ground) {
        let s = ground.sample(x, y);
        self.q = Quat::from_yaw(yaw);
        self.pos = v3(x, y, s.height + 0.60);
        self.vel = self.q.rotate(V3::X) * speed;
        self.omega = V3::ZERO;
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
    }

    /// World position of a wheel centre (for rendering).
    pub fn wheel_center(&self, i: usize) -> V3 {
        let up = self.q.rotate(V3::Z);
        self.pos + self.q.rotate(self.p.mounts[i]) - up * (self.p.susp_rest - self.wheels[i].comp)
    }

    pub fn forward(&self) -> V3 {
        self.q.rotate(V3::X)
    }

    pub fn telemetry(&self) -> Telemetry {
        let f = self.forward();
        let left = self.q.rotate(V3::Y);
        let vf = self.vel.dot(f);
        let vl = self.vel.dot(left);
        Telemetry {
            speed: self.vel.len(),
            forward_speed: vf,
            side_slip: vl.atan2(vf.abs().max(2.0)),
            rpm: self.drive.rpm(),
            gear: self.drive.gear,
            lat_g: self.accel.y / G,
        }
    }

    fn inv_inertia_world(&self, v: V3) -> V3 {
        let b = self.q.inv_rotate(v);
        self.q.rotate(v3(b.x / self.p.inertia.x, b.y / self.p.inertia.y, b.z / self.p.inertia.z))
    }

    /// Advance by `dt` seconds (use ~1 ms; the tire model is stiff at low speed).
    pub fn step(&mut self, dt: f64, inp: &Input, ground: &dyn Ground, world: &dyn Obstacles) {
        let q = self.q;
        let up = q.rotate(V3::Z);
        let down = -up;
        let reach = self.p.susp_rest + self.p.wheel_radius;
        let r_w = self.p.wheel_radius;

        // ---- 1. probe the ground under each wheel ----
        struct Probe {
            contact: bool,
            comp: f64,
            t: f64,
            n: V3,
            grip: f64,
            mount: V3,
        }
        let pr: [Probe; 4] = std::array::from_fn(|i| {
            let mount = self.pos + q.rotate(self.p.mounts[i]);
            let mut t = reach;
            let mut n = V3::Z;
            let mut grip = 1.0;
            let mut ok = true;
            for _ in 0..3 {
                let pt = mount + down * t;
                let s = ground.sample_at(pt.x, pt.y, pt.z);
                let denom = down.dot(s.normal);
                if denom > -0.2 {
                    ok = false;
                    break;
                }
                t = (v3(pt.x, pt.y, s.height) - mount).dot(s.normal) / denom;
                n = s.normal;
                grip = s.grip;
            }
            let comp = if ok { reach - t } else { 0.0 };
            Probe { contact: ok && comp > 0.0, comp: comp.max(0.0), t, n, grip, mount }
        });

        // ---- 2. suspension forces ----
        let mut fs = [0.0f64; 4];
        for i in 0..4 {
            let ax = i / 2;
            if pr[i].contact {
                let vm = self.vel + self.omega.cross(pr[i].mount - self.pos);
                let rate = vm.dot(down);
                let mut f = self.p.spring[ax] * pr[i].comp + self.p.damper[ax] * rate;
                if pr[i].comp > self.p.bump_start {
                    f += self.p.bump_k * (pr[i].comp - self.p.bump_start);
                }
                fs[i] = f;
            }
        }
        for ax in 0..2 {
            let (l, r) = (ax * 2, ax * 2 + 1);
            let d = pr[l].comp - pr[r].comp;
            let f = self.p.arb[ax] * d;
            if pr[l].contact {
                fs[l] += f;
            }
            if pr[r].contact {
                fs[r] -= f;
            }
        }
        for f in fs.iter_mut() {
            *f = f.max(0.0);
        }

        let mut force = v3(0.0, 0.0, -self.p.mass * G);
        let mut torque = V3::ZERO;
        let add = |f: V3, at: V3, pos: V3, force: &mut V3, torque: &mut V3| {
            *force += f;
            *torque += (at - pos).cross(f);
        };
        for i in 0..4 {
            if fs[i] > 0.0 {
                // Force goes along the ground normal (the knuckle/arms react the rest), so a tilted body does not self-propel.
                add(pr[i].n * fs[i], pr[i].mount, self.pos, &mut force, &mut torque);
            }
        }

        // ---- 3. drivetrain ----
        let mut throttle = inp.throttle;
        if inp.tcs && inp.handbrake < 0.1 {
            let (a, b) = self.p.driven;
            let k = self.wheels[a].kappa.max(self.wheels[b].kappa);
            if self.wheels[a].contact || self.wheels[b].contact {
                throttle *= (1.0 - (k - 0.22) / 0.15).clamp(0.0, 1.0);
            }
        }
        let (da, db) = self.p.driven;
        let dout = self.drive.step(dt, &self.p.drive, throttle, inp.reverse, self.wheels[da].w, self.wheels[db].w);

        // ---- 4. steering geometry (Ackermann) ----
        let t = inp.steer_angle.tan();
        let half = self.p.track * 0.5;
        let l = self.p.wheelbase;
        let steer_l = (l * t).atan2(l - half * t);
        let steer_r = (l * t).atan2(l + half * t);

        // ---- 5. tires and wheel dynamics ----
        let mut max_impulse: f64 = 0.0;
        for i in 0..4 {
            let front = i < 2;
            let ax = i / 2;
            let delta = if front { if i == 0 { steer_l } else { steer_r } } else { 0.0 };
            self.wheels[i].steer = delta;
            let mut t_drive = 0.0;
            let mut stiff = 0.0;
            if i == da {
                t_drive = dout.torque_l;
                stiff = dout.stiffness;
            } else if i == db {
                t_drive = dout.torque_r;
                stiff = dout.stiffness;
            }
            // brake torque with simple ABS
            let mut tb = inp.brake * if front { self.p.brake_front } else { self.p.brake_rear };
            if inp.abs && inp.brake > 0.05 && self.wheels[i].contact {
                let slip = -self.wheels[i].kappa;
                if slip > 0.12 {
                    tb *= (1.0 - (slip - 0.12) / 0.10).clamp(0.05, 1.0);
                }
            }
            if !front {
                // Handbrake = rear brake torque that yields to throttle (so holding it with power on keeps the
                // car moving instead of dragging it to a halt) ...
                tb += inp.handbrake * self.p.handbrake * (1.0 - 0.75 * throttle);
            }

            let w0 = self.wheels[i].w;
            let mut w_new;
            let mut fz = 0.0;
            let mut out = TireOut::default();
            let f_p;
            let lat;
            let mut pc = pr[i].mount;
            if pr[i].contact {
                fz = fs[i];
                let n = pr[i].n;
                pc = pr[i].mount + down * pr[i].t;
                let rc = pc - self.pos;
                let vc = self.vel + self.omega.cross(rc);
                let head = q.rotate(v3(delta.cos(), delta.sin(), 0.0));
                f_p = (head - n * head.dot(n)).norm();
                lat = n.cross(f_p);
                let (vl, vt) = (vc.dot(f_p), vc.dot(lat));
                // ... plus a drift assist: rear tyres lose part of their grip while the handbrake is held (a locked or
                // spinning-against-the-brake tyre slides at below peak friction), so a slide starts at speed even
                // with power on and persists while the handbrake is held.
                let grip = pr[i].grip * if front { 1.0 } else { 1.0 - HANDBRAKE_GRIP_LOSS * inp.handbrake };
                // A braked, (nearly) stopped wheel on a stopped car must hold: shrink the low-speed regularisation
                // so the tyre acts like a much stiffer bristle instead of creeping at ~7 cm/s on a slope.
                let mut tp = self.p.tire;
                if tb > 0.0 && w0.abs() < 0.5 && self.vel.len() < 1.0 {
                    tp.v0 = 0.4;
                }
                let o1 = tire_force(&tp, fz, grip, vl, vt, w0, r_w);
                let o2 = tire_force(&tp, fz, grip, vl, vt, w0 + 1.0, r_w);
                let jx = ((o2.fx - o1.fx)).max(0.0);
                let denom = self.p.wheel_inertia + dt * (r_w * jx + stiff);
                let pre = w0 + dt * (t_drive - r_w * o1.fx) / denom;
                w_new = apply_brake(pre, tb, dt, self.p.wheel_inertia);
                out = tire_force(&tp, fz, grip, vl, vt, w_new, r_w);
                // rolling resistance
                let rr = -self.p.crr * fz * (vl / 0.3).tanh();
                let f = f_p * (out.fx + rr) + lat * out.fy;
                add(f, pc, self.pos, &mut force, &mut torque);
            } else {
                // airborne: free-spinning wheel
                let pre = w0 + dt * t_drive / (self.p.wheel_inertia + dt * stiff);
                w_new = apply_brake(pre, tb, dt, self.p.wheel_inertia);
            }
            if !w_new.is_finite() {
                w_new = 0.0;
            }
            let wh = &mut self.wheels[i];
            wh.w = w_new;
            wh.spin += w_new * dt;
            wh.comp = pr[i].comp;
            wh.fz = fz;
            wh.contact = pr[i].contact;
            wh.kappa = out.kappa;
            wh.tan_alpha = out.tan_alpha;
            wh.fx = out.fx;
            wh.fy = out.fy;
            wh.locked = tb > 0.0 && w_new == 0.0;
            wh.pc = pc;
            let _ = ax;
        }

        // ---- 6. aerodynamic drag ----
        let sp = self.vel.len();
        force += self.vel * (-0.5 * 1.2 * self.p.cd_a * sp);

        // filtered specific force for telemetry (gravity removed)
        let spec = q.inv_rotate((force + v3(0.0, 0.0, self.p.mass * G)) * (1.0 / self.p.mass));
        let k = (dt / 0.1).min(1.0);
        self.accel = self.accel * (1.0 - k) + spec * k;

        // ---- 7. integrate the body ----
        self.vel += force * (dt / self.p.mass);
        let wb = q.inv_rotate(self.omega);
        let tb_ = q.inv_rotate(torque);
        let iw = self.p.inertia.mulv(wb);
        let gyro = wb.cross(iw);
        let i = self.p.inertia;
        let rhs = tb_ - gyro;
        let alpha = v3(rhs.x / i.x, rhs.y / i.y, rhs.z / i.z);
        self.omega = q.rotate(wb + alpha * dt);
        self.pos += self.vel * dt;
        self.q = q.integrate(self.omega, dt);

        // ---- 8. collisions with static obstacles ----
        let mut scratch = std::mem::take(&mut self.scratch);
        for k in 0..self.p.collision.len() {
            let (local, r) = self.p.collision[k];
            let c = self.pos + self.q.rotate(local);
            world.query(c.x, c.y, r, &mut scratch);
            for ct in scratch.iter() {
                let n = v3(ct.nx, ct.ny, 0.0);
                let rv = c - self.pos;
                // position correction
                self.pos += n * (ct.depth * 0.8);
                let vc = self.vel + self.omega.cross(rv);
                let vn = vc.dot(n);
                if vn < 0.0 {
                    let e = 0.15;
                    let ang = self.inv_inertia_world(rv.cross(n)).cross(rv).dot(n);
                    let j = -(1.0 + e) * vn / (1.0 / self.p.mass + ang);
                    max_impulse = max_impulse.max(j);
                    self.vel += n * (j / self.p.mass);
                    self.omega += self.inv_inertia_world(rv.cross(n * j));
                    // tangential friction
                    let vt = vc - n * vn;
                    let vt_len = vt.len();
                    if vt_len > 1e-6 {
                        let td = vt / vt_len;
                        let angt = self.inv_inertia_world(rv.cross(td)).cross(rv).dot(td);
                        let jt = (vt_len / (1.0 / self.p.mass + angt)).min(0.4 * j);
                        self.vel -= td * (jt / self.p.mass);
                        self.omega -= self.inv_inertia_world(rv.cross(td * jt));
                    }
                }
            }
        }
        self.scratch = scratch;
        self.impact = max_impulse;
        self.time += dt;
    }
}

/// Apply a dissipative brake torque `tb` (>= 0) to a wheel of inertia `i` moving at `w`, sticking at zero.
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

    pub fn sim(v: &mut Vehicle, secs: f64, inp: &Input, g: &dyn Ground, w: &StaticWorld) {
        let n = (secs * 1000.0).round() as usize;
        for _ in 0..n {
            v.step(0.001, inp, g, w);
        }
    }

    fn setup() -> (Vehicle, FlatGround, StaticWorld) {
        let g = FlatGround { grip: 1.0 };
        let v = Vehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
        (v, g, StaticWorld::new(8.0))
    }

    #[test]
    fn settles_on_its_springs_with_correct_load() {
        let (mut v, g, w) = setup();
        sim(&mut v, 4.0, &Input::default(), &g, &w);
        let load: f64 = v.wheels.iter().map(|w| w.fz).sum();
        assert!((load - 1450.0 * G).abs() / (1450.0 * G) < 0.01, "load {load}");
        assert!(v.pos.z > 0.54 && v.pos.z < 0.64, "cg height {}", v.pos.z);
        assert!(v.vel.len() < 0.02, "still moving {}", v.vel.len());
        // front axle carries slightly more than rear (CG nearer the front: a=1.25 < b=1.45)
        let front = v.wheels[0].fz + v.wheels[1].fz;
        let rear = v.wheels[2].fz + v.wheels[3].fz;
        assert!(front > rear, "front {front} rear {rear}");
    }

    #[test]
    fn accelerates_0_100_in_a_sane_time() {
        let (mut v, g, w) = setup();
        let inp = Input { throttle: 1.0, tcs: true, abs: true, ..Default::default() };
        let mut t = 0.0;
        while v.telemetry().forward_speed < 27.78 && t < 30.0 {
            v.step(0.001, &inp, &g, &w);
            t += 0.001;
        }
        assert!(t > 5.0 && t < 11.0, "0-100 took {t}");
        assert!(v.wheels.iter().all(|w| w.w.is_finite()));
    }

    #[test]
    fn top_speed_is_drag_limited() {
        let (mut v, g, w) = setup();
        v.place(0.0, 0.0, 0.0, 30.0, &g);
        let inp = Input { throttle: 1.0, tcs: true, abs: true, ..Default::default() };
        sim(&mut v, 90.0, &inp, &g, &w);
        let kmh = v.telemetry().forward_speed * 3.6;
        assert!(kmh > 150.0 && kmh < 235.0, "top speed {kmh}");
        assert!(v.telemetry().rpm < 7000.0);
    }

    #[test]
    fn abs_brakes_from_100_in_a_realistic_distance() {
        let (mut v, g, w) = setup();
        v.place(0.0, 0.0, 0.0, 27.78, &g);
        sim(&mut v, 0.3, &Input::default(), &g, &w); // settle
        let x0 = v.pos.x;
        let inp = Input { brake: 1.0, abs: true, ..Default::default() };
        let mut t = 0.0;
        while v.telemetry().forward_speed > 0.2 && t < 10.0 {
            v.step(0.001, &inp, &g, &w);
            t += 0.001;
        }
        let d = v.pos.x - x0;
        assert!(d > 28.0 && d < 55.0, "braking distance {d}");
        assert!(v.pos.y.abs() < 1.0, "should brake straight, drifted {}", v.pos.y);
    }

    #[test]
    fn steady_turn_reaches_about_one_g_and_no_more() {
        let (mut v, g, w) = setup();
        v.place(0.0, 0.0, 0.0, 15.0, &g);
        // hold speed roughly with throttle while steering hard enough to saturate the tires
        let mut peak: f64 = 0.0;
        for k in 0..8000 {
            let thr = if v.telemetry().forward_speed < 15.0 { 0.35 } else { 0.0 };
            let inp = Input { throttle: thr, steer_angle: 0.12, tcs: true, abs: true, ..Default::default() };
            v.step(0.001, &inp, &g, &w);
            if k > 3000 {
                peak = peak.max(v.telemetry().lat_g.abs());
            }
        }
        assert!(peak > 0.7 && peak < 1.25, "peak lateral g {peak}");
    }

    #[test]
    fn handbrake_locks_the_rear_wheels_and_swings_the_tail() {
        let (mut v, g, w) = setup();
        v.place(0.0, 0.0, 0.0, 22.0, &g);
        let inp = Input { steer_angle: 0.08, handbrake: 1.0, ..Default::default() };
        sim(&mut v, 1.5, &inp, &g, &w);
        assert!(v.wheels[2].locked && v.wheels[3].locked, "rear wheels should be locked");
        assert!(!v.wheels[0].locked, "front wheels must keep rolling");
        let beta = v.telemetry().side_slip.abs();
        assert!(beta > 0.12, "tail should step out, side slip {beta}");
        assert!(v.vel.is_finite() && v.omega.is_finite());
    }

    #[test]
    fn handbrake_with_power_keeps_the_car_moving_and_sliding() {
        // Hold the handbrake and the throttle in a low gear: the rear wheels break free of the brake and spin,
        // so the car keeps moving instead of grinding to a halt.
        let (mut v, g, w) = setup();
        v.place(0.0, 0.0, 0.0, 14.0, &g);
        let inp = Input { throttle: 1.0, steer_angle: 0.10, handbrake: 1.0, ..Default::default() };
        sim(&mut v, 20.0, &inp, &g, &w);
        let sp = v.telemetry().speed;
        assert!(sp > 6.0, "car stalled to {sp} m/s with throttle + handbrake");
        assert!(v.telemetry().rpm > 1500.0);
    }

    #[test]
    fn handbrake_alone_stops_the_car_without_flipping() {
        let (mut v, g, w) = setup();
        v.place(0.0, 0.0, 0.0, 22.0, &g);
        let inp = Input { handbrake: 1.0, ..Default::default() };
        sim(&mut v, 12.0, &inp, &g, &w);
        assert!(v.telemetry().speed < 0.5, "should have stopped, speed {}", v.telemetry().speed);
        let up = v.q.rotate(V3::Z);
        assert!(up.z > 0.95, "car tipped over, up.z={}", up.z);
    }

    #[test]
    fn handbrake_holds_the_car_on_a_slope() {
        let g = SlopeGround { angle: 0.17, grip: 1.0 }; // ~10 degrees
        let mut v = Vehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g); // nose uphill
        let w = StaticWorld::new(8.0);
        let inp = Input { handbrake: 1.0, brake: 0.0, ..Default::default() };
        sim(&mut v, 3.0, &inp, &g, &w);
        let (x0, y0) = (v.pos.x, v.pos.y);
        sim(&mut v, 10.0, &inp, &g, &w);
        let moved = ((v.pos.x - x0).powi(2) + (v.pos.y - y0).powi(2)).sqrt();
        assert!(moved < 0.5, "rolled {moved} m on a 10 degree slope with the handbrake on");
    }

    #[test]
    fn rolls_back_down_a_slope_without_brakes() {
        let g = SlopeGround { angle: 0.17, grip: 1.0 };
        let mut v = Vehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
        let w = StaticWorld::new(8.0);
        sim(&mut v, 3.0, &Input::default(), &g, &w);
        // Engine braking holds it in gear only through the clutch; with idle clutch it creeps. It must not
        // fall through the ground or explode.
        assert!(v.pos.is_finite());
        let h = g.sample(v.pos.x, v.pos.y).height;
        assert!(v.pos.z - h > 0.3 && v.pos.z - h < 1.0);
    }

    #[test]
    fn body_rolls_outward_in_a_corner_and_dives_under_braking() {
        let (mut v, g, w) = setup();
        v.place(0.0, 0.0, 0.0, 20.0, &g);
        sim(&mut v, 0.5, &Input::default(), &g, &w);
        let inp = Input { steer_angle: 0.06, ..Default::default() }; // left turn
        sim(&mut v, 1.5, &inp, &g, &w);
        // left turn -> body leans right: right side lower, left side higher -> left compression < right compression
        assert!(v.wheels[1].comp > v.wheels[0].comp + 0.005, "no body roll: L {} R {}", v.wheels[0].comp, v.wheels[1].comp);
        let (mut v, g, w) = setup();
        v.place(0.0, 0.0, 0.0, 20.0, &g);
        sim(&mut v, 0.5, &Input::default(), &g, &w);
        let front0 = v.wheels[0].comp;
        sim(&mut v, 0.6, &Input { brake: 0.6, abs: true, ..Default::default() }, &g, &w);
        assert!(v.wheels[0].comp > front0 + 0.01, "no brake dive");
    }

    #[test]
    fn hits_a_wall_and_stops_without_tunnelling() {
        let (mut v, g, mut w) = setup();
        w.add_poly(vec![(30.0, -20.0), (40.0, -20.0), (40.0, 20.0), (30.0, 20.0)]);
        v.place(0.0, 0.0, 0.0, 14.0, &g); // 50 km/h
        let inp = Input { throttle: 0.0, ..Default::default() };
        sim(&mut v, 6.0, &inp, &g, &w);
        assert!(v.pos.x < 30.0, "went through the wall: x={}", v.pos.x);
        assert!(v.pos.is_finite() && v.vel.is_finite() && v.omega.is_finite());
        assert!(v.vel.len() < 3.0, "should have lost most speed, {}", v.vel.len());
    }

    #[test]
    fn glancing_hit_makes_the_car_rotate() {
        let (mut v, g, mut w) = setup();
        w.add_poly(vec![(20.0, 1.2), (60.0, 1.2), (60.0, 5.0), (20.0, 5.0)]); // wall just beside the path, left side
        v.place(0.0, 0.0, 0.05, 15.0, &g); // veering slightly left into it
        let mut max_yaw_rate: f64 = 0.0;
        let inp = Input::default();
        for _ in 0..4000 {
            v.step(0.001, &inp, &g, &w);
            max_yaw_rate = max_yaw_rate.max(v.omega.z.abs());
        }
        assert!(max_yaw_rate > 0.1, "an off-centre impact must induce yaw ({max_yaw_rate})");
        assert!(v.pos.is_finite());
    }

    #[test]
    fn deterministic_for_identical_inputs() {
        let run = || {
            let (mut v, g, w) = setup();
            let mut x = 0.0;
            for k in 0..6000 {
                let inp = Input {
                    throttle: if k % 1500 < 1000 { 1.0 } else { 0.0 },
                    steer_angle: 0.1 * ((k as f64) * 0.002).sin(),
                    brake: if k % 1500 >= 1200 { 0.5 } else { 0.0 },
                    tcs: true,
                    abs: true,
                    ..Default::default()
                };
                v.step(0.001, &inp, &g, &w);
                x += v.pos.x + v.pos.y;
            }
            x
        };
        assert_eq!(run().to_bits(), run().to_bits());
    }

    #[test]
    fn survives_random_abuse_without_nan_or_blowup() {
        // xorshift, fixed seed
        let mut s: u64 = 0x2545F4914F6CDD1D;
        let mut rnd = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 11) as f64 / (1u64 << 53) as f64
        };
        let (mut v, g, mut w) = setup();
        for i in 0..40 {
            w.add_poly(vec![(100.0 + 30.0 * i as f64, 20.0), (110.0 + 30.0 * i as f64, 20.0), (110.0 + 30.0 * i as f64, 30.0), (100.0 + 30.0 * i as f64, 30.0)]);
        }
        let mut inp = Input::default();
        for k in 0..180_000 {
            if k % 250 == 0 {
                inp = Input {
                    throttle: if rnd() < 0.7 { rnd() } else { 0.0 },
                    brake: if rnd() < 0.2 { rnd() } else { 0.0 },
                    steer_angle: (rnd() - 0.5) * 1.1,
                    handbrake: if rnd() < 0.15 { 1.0 } else { 0.0 },
                    reverse: rnd() < 0.1,
                    tcs: rnd() < 0.5,
                    abs: rnd() < 0.5,
                };
            }
            v.step(0.001, &inp, &g, &w);
            if k % 1000 == 0 {
                assert!(v.pos.is_finite() && v.vel.is_finite() && v.omega.is_finite(), "non-finite state at step {k}");
                assert!(v.vel.len() < 120.0 && v.omega.len() < 40.0, "blow-up at step {k}: v={} w={}", v.vel.len(), v.omega.len());
                assert!(v.pos.z > -2.0 && v.pos.z < 20.0, "left the world at step {k}: z={}", v.pos.z);
            }
        }
    }
}
