//! Prints headline numbers of the default sedan so they can be sanity-checked against a real car.
use phys::*;
fn main() {
    let g = FlatGround { grip: 1.0 };
    let w = StaticWorld::new(8.0);
    // 0-100 km/h
    let mut v = Vehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
    let inp = Input { throttle: 1.0, tcs: true, abs: true, ..Default::default() };
    let mut t = 0.0;
    while v.telemetry().forward_speed < 27.78 && t < 30.0 { v.step(0.001, &inp, &g, &w); t += 0.001; }
    println!("0-100 km/h: {t:.1} s");
    // top speed
    let mut v = Vehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
    v.place(0.0, 0.0, 0.0, 30.0, &g);
    for _ in 0..120_000 { v.step(0.001, &inp, &g, &w); }
    println!("top speed: {:.0} km/h at {:.0} rpm, gear {}", v.telemetry().forward_speed * 3.6, v.telemetry().rpm, v.telemetry().gear);
    // 100-0 braking with ABS
    let mut v = Vehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
    v.place(0.0, 0.0, 0.0, 27.78, &g);
    for _ in 0..300 { v.step(0.001, &Input::default(), &g, &w); }
    let x0 = v.pos.x;
    let b = Input { brake: 1.0, abs: true, ..Default::default() };
    while v.telemetry().forward_speed > 0.2 { v.step(0.001, &b, &g, &w); }
    println!("100-0 km/h: {:.1} m", v.pos.x - x0);
    // skidpad
    let mut v = Vehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
    v.place(0.0, 0.0, 0.0, 15.0, &g);
    let mut peak: f64 = 0.0;
    for k in 0..8000 {
        let thr = if v.telemetry().forward_speed < 15.0 { 0.35 } else { 0.0 };
        v.step(0.001, &Input { throttle: thr, steer_angle: 0.12, tcs: true, abs: true, ..Default::default() }, &g, &w);
        if k > 3000 { peak = peak.max(v.telemetry().lat_g.abs()); }
    }
    println!("peak lateral acceleration: {peak:.2} g");
}
