//! A drivable vehicle built from an assembled JBeam model: every node and beam of the mod goes into the
//! soft-body solver; tyres act at the wheel axle nodes (`pressureWheels` node1/node2) through the engine's tyre
//! model instead of BeamNG's generated tyre nodes; steering moves the hubs through the mod's `hydros`;
//! engine, gearbox and brakes come from the mod's `mainEngine`, `gearbox` and wheel data where present.
//!
//! What is NOT like BeamNG: tyres are not node/beam meshes, no Lua controllers (the gearbox shifts
//! automatically, ABS/TCS are ours), torsion bars, triangles' aerodynamics, self-collision, thermals and
//! damage of powertrain parts are ignored.

use crate::model::{to_ours, Model};
use phys::drivetrain::{Drivetrain, DrivetrainParams};
use phys::math::{v3, Quat, V3};
use phys::softbody::{Pin, Slide, SoftBody, Stops};
use phys::softcar::quat_from_axes;
use phys::tire::{tire_force, TireOut, TireParams};
use phys::vehicle::{Input, Wheel};
use phys::world::{Ground, Obstacles};

const TIRE_K: f64 = 2.2e5;
const TIRE_C: f64 = 1500.0;
/// tyre + rim mass added to each wheel's axle nodes (BeamNG builds these as extra tyre nodes)
const WHEEL_EXTRA_MASS: f64 = 16.0;
const SUB_DT: f64 = 0.000_25;

#[derive(Clone, Debug)]
pub struct JWheel {
    pub name: String,
    /// outer and inner axle node
    pub n1: usize,
    pub n2: usize,
    pub arm: Option<usize>,
    pub radius: f64,
    pub width: f64,
    pub front: bool,
    pub driven: bool,
    pub brake_torque: f64,
    pub parking_torque: f64,
    pub inertia: f64,
    pub state: Wheel,
}

pub struct Hydro {
    pub beam: usize,
    pub base: f64,
    pub factor: f64,
    /// how fast the hydro follows its input (input units per second), BeamNG `inRate` / `outRate`
    pub in_rate: f64,
    pub out_rate: f64,
    /// current (rate-limited) input -1..1
    pub state: f64,
}

pub struct JbeamVehicle {
    pub body: SoftBody,
    pub names: Vec<String>,
    pub wheels: Vec<JWheel>,
    pub hydros: Vec<Hydro>,
    pub drive: Drivetrain,
    pub dparams: DrivetrainParams,
    pub tire: TireParams,
    /// render triangles (node indices) from the mod's `triangles` section
    pub triangles: Vec<[usize; 3]>,
    pub pos: V3,
    pub q: Quat,
    pub vel: V3,
    pub accel: V3,
    pub impact: f64,
    pub cd_a: f64,
    /// (ref, back, left, up) nodes used for the body frame
    frame_nodes: [usize; 4],
    last_vel: V3,
    pub notes: Vec<String>,
    /// driven wheel indices (left, right) for the 2-output drivetrain model
    driven_pair: (usize, usize),
    pub steer_cmd: f64,
    /// nodes held by fewer than three beams (left dangling by parts missing from the mod); not counted as damage
    pub weak: Vec<bool>,
    /// body-frame orientation of the undamaged rest shape (refNodes need not be exactly axis-aligned)
    q_rest: Quat,
}

fn idx(m: &Model, name: Option<&str>) -> Option<usize> {
    name.and_then(|n| m.node_index.get(n).copied())
}

/// Engine/gearbox from the mod's `mainEngine` and `gearbox` sections; final drive defaults to 4.1 when the mod
/// does not give one (BeamNG keeps it in a differential part).
pub fn drivetrain_params(m: &Model, notes: &mut Vec<String>) -> DrivetrainParams {
    let mut p = DrivetrainParams::default();
    let section = |name: &str| m.asm.parts.iter().find_map(|pt| pt.body.get(name).filter(|v| v.get("torque").is_some() || v.get("gearRatios").is_some() || name != "mainEngine"));
    if let Some(e) = m.asm.parts.iter().find_map(|pt| pt.body.get("mainEngine").filter(|v| v.get("torque").is_some())) {
        if let Some(rows) = e.get("torque").and_then(|t| t.as_arr()) {
            let curve: Vec<(f64, f64)> = rows.iter().filter_map(|r| r.as_arr()).filter_map(|r| Some((r.first()?.as_f64()?, r.get(1)?.as_f64()?))).filter(|(rpm, _)| *rpm > 0.0).collect();
            if curve.len() >= 2 {
                p.torque_curve = curve;
            }
        }
        if let Some(v) = e.get("idleRPM").and_then(|v| v.as_f64()) {
            p.idle_rpm = v;
        }
        if let Some(v) = e.get("maxRPM").and_then(|v| v.as_f64()) {
            p.max_rpm = v;
        }
        if let Some(v) = e.get("inertia").and_then(|v| v.as_f64()) {
            // our clutch model is tuned for a heavier engine+flywheel; keep a floor for stability
            p.engine_inertia = v.max(0.15);
        }
        notes.push(format!("engine from mod: peak {:.0} Nm, idle {:.0}, max {:.0} rpm", p.torque_curve.iter().map(|c| c.1).fold(0.0, f64::max), p.idle_rpm, p.max_rpm));
    } else {
        notes.push("engine: none in mod, using default".into());
    }
    if let Some(g) = section("gearbox").and_then(|g| g.get("gearRatios")).and_then(|r| r.as_arr()) {
        let r: Vec<f64> = g.iter().filter_map(|v| v.as_f64()).collect();
        let fwd: Vec<f64> = r.iter().copied().filter(|v| *v > 0.0).collect();
        if let Some(rev) = r.iter().copied().find(|v| *v < 0.0) {
            p.reverse = -rev;
        }
        if !fwd.is_empty() {
            p.gears = fwd;
        }
        notes.push(format!("gearbox from mod: {} forward gears (shifted automatically)", p.gears.len()));
    }
    let fd = m.table("powertrain").iter().find_map(|(_, r)| (r.str("type") == Some("differential")).then(|| r.num("gearRatio")).flatten());
    p.final_drive = fd.unwrap_or(4.1);
    if fd.is_none() {
        notes.push("final drive not in mod: 4.1 assumed".into());
    }
    p
}

impl JbeamVehicle {
    pub fn from_model(m: &Model) -> Result<JbeamVehicle, String> {
        let mut notes = Vec::new();
        let mut body = SoftBody::new();
        body.world_inelastic = true;
        // positions in the engine frame, ground under the lowest wheel set later by `place`
        for n in &m.nodes {
            let i = body.add_node({ let p = to_ours(n.pos); v3(p[0], p[1], p[2]) }, n.mass.max(0.05));
            body.nodes[i].radius = 0.03;
            body.nodes[i].hit_ground = n.collision;
            body.nodes[i].hit_world = n.collision;
        }
        // nodes that BeamNG excludes from static collision (wheel hubs)
        for (pi, r) in m.table("nodes") {
            let _ = pi;
            if matches!(r.get("staticCollision"), Some(scene::json::Json::Bool(false))) {
                if let Some(i) = idx(m, r.str("id")) {
                    body.nodes[i].hit_ground = false;
                    body.nodes[i].hit_world = false;
                }
            }
        }
        for b in &m.beams {
            let k = b.spring();
            let c = b.damp();
            let bi = body.add_beam(b.a, b.b, k, c);
            let be = &mut body.beams[bi];
            let pre = b.row.num_or("beamPrecompression", 1.0);
            if pre > 0.0 && (pre - 1.0).abs() > 1e-9 {
                be.rest = be.rest0 * pre;
                be.rest0 = be.rest;
            }
            // coil springs: free length = installed length + precompressionRange (m), i.e. preloaded
            if let Some(r) = b.row.num("precompressionRange") {
                be.rest += r;
                be.rest0 = be.rest;
            }
            if let Some(d) = b.deform() {
                be.yield_t = d;
                be.yield_c = d;
            }
            if let Some(s) = b.strength() {
                be.break_force = s;
            }
            if let Some(cr) = b.row.num("beamDampRebound") {
                be.c_reb = cr;
            }
            // beams torn far beyond their length are gone even if the force limit was never reached
            be.break_strain = 1.5;
            match b.kind.as_str() {
                "SUPPORT" => {
                    be.support = true;
                    be.break_strain = b.row.num_or("beamLongBound", 1.0).max(0.05);
                }
                "BOUNDED" => {
                    let long = b.row.num_or("beamLongBound", 1.0);
                    let short = b.row.num_or("beamShortBound", 1.0);
                    let lk = b.row.num_or("beamLimitSpring", 0.0);
                    let lc = b.row.num_or("beamLimitDamp", 0.0);
                    if lk > 0.0 || lc > 0.0 {
                        be.stops = Some(Stops { min_len: be.rest * (1.0 - short).max(0.0), max_len: be.rest * (1.0 + long), k: lk, c: lc });
                    }
                    be.break_strain = f64::INFINITY;
                }
                _ => {}
            }
        }
        // slide nodes on rails (MacPherson struts, steering rack)
        let mut rails: std::collections::HashMap<String, (Vec<usize>, bool)> = std::collections::HashMap::new();
        for p in &m.asm.parts {
            if let Some(scene::json::Json::Obj(o)) = p.body.get("rails") {
                for (name, r) in o {
                    let links: Vec<usize> = r.get("links:").or(r.get("links")).and_then(|l| l.as_arr()).map(|a| a.iter().filter_map(|n| idx(m, n.as_str())).collect()).unwrap_or_default();
                    let capped = r.get("capped").and_then(|c| c.as_bool()).unwrap_or(true);
                    if links.len() >= 2 {
                        rails.insert(name.clone(), (links, capped));
                    }
                }
            }
        }
        for (_, r) in m.table("slidenodes") {
            let (Some(n), Some(rail)) = (idx(m, r.str("id")), r.str("railName").and_then(|n| rails.get(n))) else { continue };
            let k = r.num_or("spring", 1.0e7);
            let c = 2.0 * 0.2 * (k * body.nodes[n].m).sqrt();
            body.slides.push(Slide { node: n, rail: rail.0.clone(), k, c, capped: rail.1 });
        }
        if !body.slides.is_empty() {
            notes.push(format!("{} slide nodes on rails", body.slides.len()));
        }
        // couplers (door, hood, tailgate latches): closed at spawn, open above their strength
        for p in &m.asm.parts {
            if let scene::json::Json::Obj(o) = &p.body {
                for (_, sec) in o {
                    let Some(cn) = sec.get("couplerNodes") else { continue };
                    for r in crate::table::rows(cn) {
                        let (Some(a), Some(b)) = (idx(m, r.str("cid1")), idx(m, r.str("cid2"))) else { continue };
                        let strength = r.num_or("autoCouplingStrength", 30000.0);
                        let mm = body.nodes[a].m.min(body.nodes[b].m);
                        // stiff enough to hold a door shut, soft enough for the 0.25 ms step
                        let k = (0.5 * mm / (SUB_DT * SUB_DT) * 0.25).min(2.0e6);
                        body.pins.push(Pin { a, b, k, c: 2.0 * (k * mm).sqrt() * 0.3, strength, broken: false });
                    }
                }
            }
        }
        if !body.pins.is_empty() {
            notes.push(format!("{} latches (doors/hood) closed", body.pins.len()));
        }
        // steering hydros: rest length follows the steering input
        let mut hydros = Vec::new();
        for (_, r) in m.table("hydros") {
            let (Some(a), Some(b)) = (idx(m, r.str("id1")), idx(m, r.str("id2"))) else { continue };
            let len = (body.nodes[b].p - body.nodes[a].p).len();
            let bi = body.add_beam(a, b, r.num_or("beamSpring", 4.3e6), r.num_or("beamDamp", 580.0));
            body.beams[bi].break_force = r.num("beamStrength").unwrap_or(f64::INFINITY);
            hydros.push(Hydro { beam: bi, base: len, factor: r.num_or("factor", 0.0), in_rate: r.num_or("inRate", 2.0), out_rate: r.num_or("outRate", 2.0), state: 0.0 });
        }
        if hydros.is_empty() {
            notes.push("no steering hydros found: the front wheels will not steer".into());
        }
        // wheels
        let mut wheels = Vec::new();
        for (_, r) in m.table("pressureWheels").into_iter().chain(m.table("hubWheels")).chain(m.table("wheels")) {
            let (Some(n1), Some(n2)) = (idx(m, r.str("node1")), idx(m, r.str("node2"))) else {
                notes.push(format!("wheel {} skipped: axle nodes missing", r.str("name").unwrap_or("?")));
                continue;
            };
            let name = r.str("name").unwrap_or("?").to_string();
            let radius = r.num("radius").or(r.num("hubRadius").map(|h| h + 0.12)).unwrap_or(0.33);
            let width = r.num("tireWidth").unwrap_or(0.2);
            for n in [n1, n2] {
                body.nodes[n].m += WHEEL_EXTRA_MASS / 2.0;
            }
            let front = to_ours(m.nodes[n1].pos)[0] > 0.0;
            wheels.push(JWheel {
                name,
                n1,
                n2,
                arm: idx(m, r.str("nodeArm")),
                radius,
                width,
                front,
                driven: false,
                brake_torque: r.num_or("brakeTorque", 1500.0),
                parking_torque: r.num_or("parkingTorque", 0.0),
                inertia: 1.1,
                state: Wheel::default(),
            });
        }
        if wheels.len() < 3 {
            return Err(format!("only {} usable wheels found", wheels.len()));
        }
        // driven wheels: spindle shafts whose input chain reaches the engine
        let pt = m.table("powertrain");
        let input_of = |name: &str| pt.iter().find(|(_, r)| r.str("name") == Some(name)).and_then(|(_, r)| r.str("inputName").map(String::from));
        for w in wheels.iter_mut() {
            let shaft = pt.iter().find(|(_, r)| r.str("connectedWheel") == Some(w.name.as_str())).and_then(|(_, r)| r.str("name").map(String::from));
            let mut cur = shaft;
            for _ in 0..12 {
                match cur {
                    Some(ref n) if n == "mainEngine" => {
                        w.driven = true;
                        break;
                    }
                    Some(n) => cur = input_of(&n),
                    None => break,
                }
            }
        }
        if !wheels.iter().any(|w| w.driven) {
            notes.push("no driven wheels found in the powertrain: front wheels driven".into());
            for w in wheels.iter_mut().filter(|w| w.front) {
                w.driven = true;
            }
        }
        let driven: Vec<usize> = (0..wheels.len()).filter(|&i| wheels[i].driven).collect();
        let leftmost = |v: &[usize]| v.iter().copied().max_by(|&a, &b| to_ours(m.nodes[wheels[a].n1].pos)[1].total_cmp(&to_ours(m.nodes[wheels[b].n1].pos)[1]));
        let rightmost = |v: &[usize]| v.iter().copied().min_by(|&a, &b| to_ours(m.nodes[wheels[a].n1].pos)[1].total_cmp(&to_ours(m.nodes[wheels[b].n1].pos)[1]));
        let driven_pair = (leftmost(&driven).unwrap(), rightmost(&driven).unwrap());
        notes.push(format!("{} wheels, driven: {}", wheels.len(), wheels.iter().filter(|w| w.driven).map(|w| w.name.as_str()).collect::<Vec<_>>().join(" ")));

        // body frame: refNodes when present, else geometric
        let rn = m.table("refNodes");
        let fr = rn.first().map(|(_, r)| [idx(m, r.str("ref")), idx(m, r.str("back")), idx(m, r.str("left")), idx(m, r.str("up"))]);
        let frame_nodes = match fr {
            Some([Some(a), Some(b), Some(c), Some(d)]) => [a, b, c, d],
            _ => return Err("refNodes (ref/back/left/up) missing".into()),
        };
        let dparams = drivetrain_params(m, &mut notes);
        let triangles = m
            .table("triangles")
            .iter()
            .filter_map(|(_, r)| Some([idx(m, r.str("id1"))?, idx(m, r.str("id2"))?, idx(m, r.str("id3"))?]))
            .collect();
        let mut v = JbeamVehicle {
            body,
            names: m.nodes.iter().map(|n| n.name.clone()).collect(),
            wheels,
            hydros,
            drive: Drivetrain::new(&dparams),
            dparams,
            tire: TireParams::default(),
            triangles,
            pos: V3::ZERO,
            q: Quat::from_yaw(0.0),
            vel: V3::ZERO,
            accel: V3::ZERO,
            impact: 0.0,
            cd_a: 0.85,
            frame_nodes,
            last_vel: V3::ZERO,
            notes,
            driven_pair,
            steer_cmd: 0.0,
            q_rest: Quat::from_yaw(0.0),
            weak: Vec::new(),
        };
        let mut deg = vec![0usize; v.body.nodes.len()];
        for b in &v.body.beams {
            deg[b.a] += 1;
            deg[b.b] += 1;
        }
        v.weak = deg.iter().map(|&d| d < 3).collect();
        let nweak = v.weak.iter().filter(|w| **w).count();
        if nweak > 0 {
            v.notes.push(format!("{nweak} loosely attached nodes (their parts are missing)"));
        }
        // rest shape relative to the ground: lowest wheel bottom at z = 0
        let bottom = v.wheels.iter().map(|w| (v.body.nodes[w.n1].p.z + v.body.nodes[w.n2].p.z) / 2.0 - w.radius).fold(f64::MAX, f64::min);
        for n in v.body.nodes.iter_mut() {
            n.p.z -= bottom;
            n.rest = n.p;
        }
        v.refresh_frame();
        v.q_rest = v.q;
        Ok(v)
    }

    pub fn total_mass(&self) -> f64 {
        self.body.total_mass()
    }

    pub fn axes(&self) -> (V3, V3, V3) {
        let [r, b, l, u] = self.frame_nodes.map(|i| self.body.nodes[i].p);
        let f = (r - b).norm();
        let up0 = (u - r).norm();
        let left = up0.cross(f).norm();
        let _ = l;
        let up = f.cross(left).norm();
        (f, left, up)
    }

    fn refresh_frame(&mut self) {
        let (f, l, u) = self.axes();
        self.q = quat_from_axes(f, l, u);
        self.pos = self.body.centre_of_mass();
        self.vel = self.body.velocity_of_com();
    }

    pub fn forward(&self) -> V3 {
        self.q.rotate(V3::X)
    }

    /// Rotation from the rest shape to the current body orientation.
    pub fn body_rotation(&self) -> Quat {
        self.q.mul(self.q_rest.conj())
    }

    /// Mass centre of the rest shape (rest-frame origin of `body_rotation`).
    pub fn rest_com(&self) -> V3 {
        let (mut s, mut m) = (V3::ZERO, 0.0);
        for n in &self.body.nodes {
            s += n.rest * n.m;
            m += n.m;
        }
        s / m
    }

    /// Put the car at (x, y) on the ground facing `yaw`, undamaged, rolling at `speed`.
    pub fn place(&mut self, x: f64, y: f64, yaw: f64, speed: f64, ground: &dyn Ground) {
        let h = ground.sample(x, y).height;
        let q = Quat::from_yaw(yaw);
        let fwd = q.rotate(V3::X);
        let c = {
            let mut s = V3::ZERO;
            let mut mt = 0.0;
            for n in &self.body.nodes {
                s += n.rest * n.m;
                mt += n.m;
            }
            s / mt
        };
        let c = v3(c.x, c.y, 0.0);
        for n in self.body.nodes.iter_mut() {
            n.p = q.rotate(n.rest - c) + v3(x, y, h + 0.003);
            n.v = fwd * speed;
            n.f = V3::ZERO;
        }
        for b in self.body.beams.iter_mut() {
            b.broken = false;
            b.rest = b.rest0;
        }
        for p in self.body.pins.iter_mut() {
            p.broken = false;
        }
        for hd in self.hydros.iter_mut() {
            hd.state = 0.0;
            self.body.beams[hd.beam].rest = hd.base;
        }
        self.body.plastic_work = 0.0;
        self.body.diverged = false;
        for w in self.wheels.iter_mut() {
            w.state = Wheel::default();
            w.state.w = speed / w.radius;
        }
        self.drive = Drivetrain::new(&self.dparams);
        self.impact = 0.0;
        self.accel = V3::ZERO;
        self.refresh_frame();
        self.last_vel = self.vel;
    }

    /// Shift everything (floating-origin re-base).
    pub fn shift(&mut self, d: V3) {
        for n in self.body.nodes.iter_mut() {
            n.p += d;
        }
        for w in self.wheels.iter_mut() {
            w.state.pc += d;
        }
        self.pos += d;
    }

    pub fn wheel_center(&self, i: usize) -> V3 {
        let w = &self.wheels[i];
        (self.body.nodes[w.n1].p + self.body.nodes[w.n2].p) * 0.5
    }

    /// Axle direction pointing to the car's left.
    pub fn wheel_axis(&self, i: usize) -> V3 {
        let w = &self.wheels[i];
        let a = (self.body.nodes[w.n1].p - self.body.nodes[w.n2].p).norm();
        let (_, left, _) = self.axes();
        if a.dot(left) < 0.0 { a * -1.0 } else { a }
    }

    /// Largest node displacement from the undamaged shape after removing rigid motion (m), and broken beams.
    /// Per-node deviation from the undamaged shape (rigid motion removed), m.
    pub fn deviations(&self) -> Vec<f64> {
        let mut rc = V3::ZERO;
        let mut mt = 0.0;
        for n in &self.body.nodes {
            rc += n.rest * n.m;
            mt += n.m;
        }
        let rc = rc / mt;
        let r = self.q.mul(self.q_rest.conj());
        self.body.nodes.iter().map(|n| (n.p - (r.rotate(n.rest - rc) + self.pos)).len()).collect()
    }

    pub fn damage(&self) -> (f64, usize) {
        let dev = self.deviations();
        let worst = dev.iter().zip(&self.weak).filter(|(_, w)| !**w).map(|(d, _)| *d).fold(0.0, f64::max);
        (worst, self.body.beams.iter().filter(|b| b.broken).count())
    }

    pub fn step(&mut self, dt: f64, inp: &Input, ground: &dyn Ground, world: &dyn Obstacles) {
        let n = ((dt / SUB_DT).round() as usize).max(1);
        let h = dt / n as f64;
        let mut impact: f64 = 0.0;
        for _ in 0..n {
            self.substep(h, inp, ground, world);
            impact = impact.max(self.body.peak_world);
        }
        self.impact = if impact > 3000.0 { 1000.0 } else { 0.0 };
        self.refresh_frame();
        let a = (self.vel - self.last_vel) / dt - v3(0.0, 0.0, -9.81);
        self.last_vel = self.vel;
        let spec = self.q.inv_rotate(a);
        let k = (dt / 0.1).min(1.0);
        if spec.is_finite() {
            self.accel = self.accel * (1.0 - k) + spec * k;
        }
    }

    fn substep(&mut self, dt: f64, inp: &Input, ground: &dyn Ground, world: &dyn Obstacles) {
        let (fwd, left, up) = self.axes();
        let vel = self.body.velocity_of_com();
        // steering: the driver model gives a road-wheel angle (positive = left); BeamNG's steering input is
        // positive to the right, so the hydro input is the negated, normalised angle
        let steer = -(inp.steer_angle / 0.55).clamp(-1.0, 1.0);
        self.steer_cmd = steer;
        for hd in self.hydros.iter_mut() {
            // rate-limited like BeamNG: a sudden full lock would otherwise tear the steering apart
            let away = steer.abs() > hd.state.abs() && steer.signum() == hd.state.signum() || hd.state == 0.0;
            let rate = if away { hd.in_rate } else { hd.out_rate } * dt;
            hd.state += (steer - hd.state).clamp(-rate, rate);
            self.body.beams[hd.beam].rest = hd.base * (1.0 + hd.factor * hd.state);
        }
        self.body.clear_forces();
        self.body.beam_forces(dt);
        self.body.slide_forces();
        // aerodynamic drag spread by mass
        let sp = vel.len();
        let mt = self.body.total_mass();
        let drag = vel * (-0.5 * 1.2 * self.cd_a * sp);
        for n in self.body.nodes.iter_mut() {
            n.f += drag * (n.m / mt);
        }
        // drivetrain
        let (dl, dr) = self.driven_pair;
        let mut throttle = inp.throttle;
        if inp.tcs && inp.handbrake < 0.1 {
            let k = self.wheels[dl].state.kappa.max(self.wheels[dr].state.kappa);
            if self.wheels[dl].state.contact || self.wheels[dr].state.contact {
                throttle *= (1.0 - (k - 0.22) / 0.15).clamp(0.0, 1.0);
            }
        }
        let dout = self.drive.step(dt, &self.dparams, throttle, inp.reverse, self.wheels[dl].state.w, self.wheels[dr].state.w);
        let n_driven = self.wheels.iter().filter(|w| w.driven).count().max(2) as f64;
        for i in 0..self.wheels.len() {
            let (n1, n2, arm) = (self.wheels[i].n1, self.wheels[i].n2, self.wheels[i].arm);
            let (r_w, front, inertia) = (self.wheels[i].radius, self.wheels[i].front, self.wheels[i].inertia);
            let c = (self.body.nodes[n1].p + self.body.nodes[n2].p) * 0.5;
            let cv = (self.body.nodes[n1].v + self.body.nodes[n2].v) * 0.5;
            let mut lat = (self.body.nodes[n1].p - self.body.nodes[n2].p).norm();
            if lat.dot(left) < 0.0 {
                lat = lat * -1.0;
            }
            let (mut t_drive, mut stiff) = (0.0, 0.0);
            if i == dl {
                t_drive = dout.torque_l;
                stiff = dout.stiffness;
            } else if i == dr {
                t_drive = dout.torque_r;
                stiff = dout.stiffness;
            } else if self.wheels[i].driven {
                // more than two driven wheels: share the axle torque evenly (no centre differential model)
                t_drive = (dout.torque_l + dout.torque_r) / n_driven;
            }
            let st = self.wheels[i].state.clone();
            let mut tb = inp.brake * self.wheels[i].brake_torque;
            if inp.abs && inp.brake > 0.05 && st.contact && -st.kappa > 0.12 {
                tb *= (1.0 - (-st.kappa - 0.12) / 0.10).clamp(0.05, 1.0);
            }
            tb += inp.handbrake * self.wheels[i].parking_torque * (1.0 - 0.75 * throttle);
            let s = ground.sample(c.x, c.y);
            let nrm = s.normal;
            let pen = r_w - (c - v3(c.x, c.y, s.height)).dot(nrm);
            let w0 = st.w;
            let mut w_new;
            let mut out = TireOut::default();
            let mut fz = 0.0;
            let mut pc = c - nrm * r_w;
            if pen > 0.0 {
                let vn = cv.dot(nrm);
                fz = (TIRE_K * pen - TIRE_C * vn).max(0.0);
                pc = c - nrm * r_w;
                let head = lat.cross(nrm).norm();
                let side = nrm.cross(head);
                let (vl, vt) = (cv.dot(head), cv.dot(side));
                let grip = s.grip * if !front { 1.0 - 0.45 * inp.handbrake } else { 1.0 };
                let mut tp = self.tire;
                if tb > 0.0 && w0.abs() < 0.5 && sp < 1.0 {
                    tp.v0 = 0.4;
                }
                let o1 = tire_force(&tp, fz, grip, vl, vt, w0, r_w);
                let o2 = tire_force(&tp, fz, grip, vl, vt, w0 + 1.0, r_w);
                let jx = (o2.fx - o1.fx).max(0.0);
                let denom = inertia + dt * (r_w * jx + stiff);
                let pre = w0 + dt * (t_drive - r_w * o1.fx) / denom;
                w_new = apply_brake(pre, tb, dt, inertia);
                out = tire_force(&tp, fz, grip, vl, vt, w_new, r_w);
                let rr = -0.012 * fz * (vl / 0.05).tanh();
                let f_t = head * (out.fx + rr) + side * out.fy;
                let f = nrm * fz + f_t;
                self.body.nodes[n1].f += f * 0.5;
                self.body.nodes[n2].f += f * 0.5;
                // the contact force acts at the patch, not at the axle: add the missing moment
                // The contact force acts at the patch, not at the axle. Its moment about the axle (from the
                // longitudinal force) is passed to the knuckle through the arm node, which gives brake dive and
                // drive squat. The moment about the wheel's heading (from the lateral force) is left out: applied
                // to the two light axle nodes it excited a wheel-hop oscillation in tests.
                let mom = (pc - c).cross(f);
                let m_lat = lat * mom.dot(lat);
                if let Some(a) = arm {
                    let ra = self.body.nodes[a].p - c;
                    let ra = ra - lat * ra.dot(lat);
                    let rr2 = ra.dot(ra);
                    if rr2 > 1e-4 {
                        let fa = m_lat.cross(ra) / rr2;
                        self.body.nodes[a].f += fa;
                        self.body.nodes[n1].f -= fa * 0.5;
                        self.body.nodes[n2].f -= fa * 0.5;
                    }
                }
            } else {
                let pre = w0 + dt * t_drive / (inertia + dt * stiff);
                w_new = apply_brake(pre, tb, dt, inertia);
            }
            if !w_new.is_finite() {
                w_new = 0.0;
            }
            let ws = &mut self.wheels[i].state;
            ws.w = w_new;
            ws.spin += w_new * dt;
            ws.fz = fz;
            ws.contact = pen > 0.0;
            ws.kappa = out.kappa;
            ws.tan_alpha = out.tan_alpha;
            ws.fx = out.fx;
            ws.fy = out.fy;
            ws.locked = tb > 0.0 && w_new == 0.0;
            ws.pc = pc;
            // steering angle of this wheel relative to the body (positive = left), for rendering and HUD
            let head = lat.cross(up);
            ws.steer = head.dot(left).atan2(head.dot(fwd));
        }
        self.body.contacts(ground, world, dt);
        self.body.integrate(dt);
        self.body.resolve_world(world, dt);
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

/// A small synthetic JBeam car (for tests and as a fallback demo): stiff box chassis, axle node pairs bolted on, front-wheel drive through a
/// BeamNG-style powertrain chain, a preloaded spring (precompressionRange) and a slide node on a rail.
pub fn synthetic_jbeam() -> String {
    let mut nodes = vec![r#"["id","posX","posY","posZ"]"#.to_string(), r#"{"nodeWeight":100}"#.to_string()];
    let corners = [("fl", 0.7, -1.2), ("fr", -0.7, -1.2), ("rl", 0.7, 1.2), ("rr", -0.7, 1.2)];
    let mut chassis = Vec::new();
    for (n, x, y) in corners {
        for (h, z) in [("lo", 0.3), ("hi", 0.8)] {
            nodes.push(format!(r#"["c{n}{h}",{x},{y},{z}]"#));
            chassis.push(format!("c{n}{h}"));
        }
    }
    nodes.push(r#"{"nodeWeight":15}"#.into());
    for (n, x, y) in corners {
        let s = if x > 0.0 { 1.0 } else { -1.0 };
        nodes.push(format!(r#"["w{n}o",{},{y},0.3]"#, s * 0.85));
        nodes.push(format!(r#"["w{n}i",{},{y},0.3]"#, s * 0.6));
    }
    nodes.push(r#"{"nodeWeight":5}"#.into());
    nodes.push(r#"["sl",0.0,0.0,0.55]"#.into());
    let mut beams = vec![r#"["id1:","id2:"]"#.to_string(), r#"{"beamSpring":3000000,"beamDamp":3000}"#.to_string()];
    for i in 0..chassis.len() {
        for j in i + 1..chassis.len() {
            beams.push(format!(r#"["{}","{}"]"#, chassis[i], chassis[j]));
        }
    }
    for (n, _, _) in corners {
        for w in ["o", "i"] {
            for h in ["lo", "hi"] {
                beams.push(format!(r#"["w{n}{w}","c{n}{h}"]"#));
            }
            let other = if n.ends_with('l') { n.replace('l', "r") } else { n.replace('r', "l") };
            beams.push(format!(r#"["w{n}{w}","c{other}lo"]"#));
            let fb = if n.starts_with('f') { n.replacen('f', "r", 1) } else { n.replacen('r', "f", 1) };
            beams.push(format!(r#"["w{n}{w}","c{fb}lo"]"#));
        }
        beams.push(format!(r#"["w{n}o","w{n}i"]"#));
    }
    beams.push(r#"{"beamSpring":20000,"beamDamp":0,"precompressionRange":0.1}"#.into());
    beams.push(r#"["sl","cfllo"]"#.into());
    format!(
        r#"{{"car":{{"slotType":"main",
          "refNodes":[["ref:","back:","left:","up:"],["cfllo","crllo","cfllo","cflhi"]],
          "nodes":[{}],
          "beams":[{}],
          "rails":{{"r1":{{"links:":["cfllo","cfrlo"],"capped":true}}}},
          "triangles":[["id1:","id2:","id3:"],
             ["cfllo","cfrlo","crrlo"],["cfllo","crrlo","crllo"],["cflhi","cfrhi","crrhi"],["cflhi","crrhi","crlhi"],
             ["cfllo","cfrlo","cfrhi"],["cfllo","cfrhi","cflhi"],["crllo","crrlo","crrhi"],["crllo","crrhi","crlhi"],
             ["cfllo","crllo","crlhi"],["cfllo","crlhi","cflhi"],["cfrlo","crrlo","crrhi"],["cfrlo","crrhi","cfrhi"]],
          "slidenodes":[["id:","railName","attached","fixToRail","tolerance","spring"],["sl","r1",true,true,0,1000000]],
          "pressureWheels":[["name","node1:","node2:"],{{"radius":0.33,"tireWidth":0.2,"brakeTorque":1500}},
             ["FL","wflo","wfli"],["FR","wfro","wfri"],["RL","wrlo","wrli"],{{"parkingTorque":1000}},["RR","wrro","wrri"]],
          "powertrain":[["type","name","inputName","inputIndex"],["combustionEngine","mainEngine","dummy",0],
             ["differential","diff","mainEngine",1,{{"gearRatio":4.0}}],
             ["shaft","axleFL","diff",1,{{"connectedWheel":"FL"}}],["shaft","axleFR","diff",2,{{"connectedWheel":"FR"}}]],
          "mainEngine":{{"torque":[["rpm","torque"],[0,0],[1000,150],[4000,200],[6500,170]],"idleRPM":800,"maxRPM":6500,"inertia":0.2}},
          "gearbox":{{"gearRatios":[-3.5,0,3.6,2.1,1.4,1.0,0.8]}}
        }}}}"#,
        nodes.join(","),
        beams.join(",")
    )
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::assemble::{assemble, Library};
    use crate::model::build;
    use phys::world::{FlatGround, StaticWorld};
    use std::collections::HashMap;

    fn vehicle() -> JbeamVehicle {
        let mut lib = Library::default();
        lib.add_file("car.jbeam", &synthetic_jbeam());
        assert!(lib.warnings.is_empty(), "{:?}", lib.warnings);
        let m = build(assemble(&lib, "car", &HashMap::new()));
        JbeamVehicle::from_model(&m).unwrap()
    }

    #[test]
    fn loads_wheels_powertrain_preload_and_rails() {
        let v = vehicle();
        assert_eq!(v.wheels.len(), 4);
        let driven: Vec<&str> = v.wheels.iter().filter(|w| w.driven).map(|w| w.name.as_str()).collect();
        assert_eq!(driven, ["FL", "FR"]);
        assert_eq!(v.dparams.final_drive, 4.0);
        assert_eq!(v.dparams.gears.len(), 5);
        assert!((v.dparams.reverse - 3.5).abs() < 1e-12);
        assert_eq!(v.body.slides.len(), 1);
        assert_eq!(v.triangles.len(), 12);
        assert_eq!(v.wheels[3].parking_torque, 1000.0);
        // preloaded spring: rest = installed length + 0.1 m
        let sb = v.body.beams.iter().find(|b| b.k == 20000.0).unwrap();
        let installed = (v.body.nodes[sb.b].rest - v.body.nodes[sb.a].rest).len();
        assert!((sb.rest - installed - 0.1).abs() < 1e-9);
        // the lowest wheel touches z = 0 in the rest shape
        let low = (0..4).map(|i| v.wheel_center(i).z - v.wheels[i].radius).fold(f64::MAX, f64::min);
        assert!(low.abs() < 1e-9);
    }

    #[test]
    fn stands_drives_forward_and_brakes() {
        let mut v = vehicle();
        let g = FlatGround { grip: 1.0 };
        let w = StaticWorld::new(8.0);
        v.place(0.0, 0.0, 0.0, 0.0, &g);
        let idle = Input { throttle: 0.0, brake: 0.0, steer_angle: 0.0, handbrake: 0.0, reverse: false, tcs: true, abs: true };
        for k in 0..2000 {
            v.step(0.001, &idle, &g, &w);
            if std::env::var("JB_TRACE").is_ok() && k % 100 == 0 {
                eprintln!("t={:.1} vel=({:.4},{:.4},{:.4}) z={:.4}", k as f64 / 1000.0, v.vel.x, v.vel.y, v.vel.z, v.pos.z);
            }
        }
        let weight = v.total_mass() * 9.81;
        let fz: f64 = v.wheels.iter().map(|w| w.state.fz).sum();
        assert!((fz - weight).abs() < 0.05 * weight, "tyres carry {fz:.0} N of {weight:.0} N");
        assert!(v.vel.len() < 0.05 && !v.body.diverged, "vel {:?} w {:?}", v.vel, v.wheels.iter().map(|w| w.state.w).collect::<Vec<_>>());
        let thr = Input { throttle: 1.0, ..idle };
        for _ in 0..5000 {
            v.step(0.001, &thr, &g, &w);
        }
        let fwd_speed = v.vel.dot(v.forward());
        assert!(fwd_speed > 12.0, "only {fwd_speed:.1} m/s after 5 s");
        assert!(v.pos.x > 20.0 && v.pos.y.abs() < 2.0, "pos {:?}", v.pos);
        let brk = Input { brake: 1.0, ..idle };
        for _ in 0..6000 {
            v.step(0.001, &brk, &g, &w);
        }
        assert!(v.vel.len() < 0.3, "still moving at {:.2} m/s", v.vel.len());
        let (_, broken) = v.damage();
        assert_eq!(broken, 0);
    }
}
