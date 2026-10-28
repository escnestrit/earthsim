//! Keyboard -> vehicle input: smoothed pedals, speed-sensitive steering, automatic reverse.
use crate::vehicle::Input;

#[derive(Clone, Copy, Debug, Default)]
pub struct Keys {
    pub up: bool,
    pub down: bool,
    pub left: bool,
    pub right: bool,
    pub handbrake: bool,
    /// Steering wheel / gamepad. When present it replaces the keyboard pedals and steering.
    pub analog: Option<Analog>,
}

/// Analog controls: steer -1 (full right) .. +1 (full left), pedals 0..1.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Analog {
    pub steer: f64,
    pub throttle: f64,
    pub brake: f64,
}

#[derive(Clone, Debug)]
pub struct Driver {
    pub throttle: f64,
    pub brake: f64,
    pub steer: f64,
    pub handbrake: f64,
    pub reverse: bool,
    pub tcs: bool,
    pub abs: bool,
    pub steer_max: f64,
}

impl Driver {
    pub fn new(steer_max: f64) -> Self {
        Driver { throttle: 0.0, brake: 0.0, steer: 0.0, handbrake: 0.0, reverse: false, tcs: true, abs: true, steer_max }
    }

    /// `fwd_speed`: signed speed along the car's heading (m/s).
    pub fn update(&mut self, dt: f64, k: &Keys, fwd_speed: f64) -> Input {
        if let Some(a) = k.analog {
            return self.update_analog(&a, k.handbrake, fwd_speed);
        }
        // W/S: throttle and brake, with S becoming reverse throttle once the car is (nearly) stopped.
        let (mut thr, mut brk) = (0.0, 0.0);
        if k.up && !k.down {
            if fwd_speed < -1.0 {
                brk = 1.0;
            } else {
                thr = 1.0;
                self.reverse = false;
            }
        } else if k.down && !k.up {
            if fwd_speed > 1.0 {
                brk = 1.0;
            } else {
                thr = 1.0;
                self.reverse = true;
            }
        }
        let rise = |cur: f64, tgt: f64, up: f64, down: f64| {
            if tgt > cur { (cur + up * dt).min(tgt) } else { (cur - down * dt).max(tgt) }
        };
        self.throttle = rise(self.throttle, thr, 4.0, 8.0);
        self.brake = rise(self.brake, brk, 6.0, 10.0);
        self.handbrake = if k.handbrake { 1.0 } else { 0.0 };

        // Steering: lock shrinks with speed; returns to centre faster than it is applied.
        let v = fwd_speed.abs();
        let lock = self.steer_max / (1.0 + (v / 14.0).powi(2));
        let dir = (k.left as i32 - k.right as i32) as f64;
        let target = dir * lock;
        let rate = if dir == 0.0 { 3.5 } else { 2.2 };
        let step = rate * dt * self.steer_max;
        let d = target - self.steer;
        self.steer += d.clamp(-step, step);
        self.steer = self.steer.clamp(-self.steer_max, self.steer_max);
        // If speed rose while turned, pull the angle in.
        if self.steer.abs() > lock && dir != 0.0 {
            self.steer = self.steer.signum() * lock;
        }

        Input {
            throttle: self.throttle,
            brake: self.brake,
            steer_angle: self.steer,
            handbrake: self.handbrake,
            reverse: self.reverse,
            tcs: self.tcs,
            abs: self.abs,
        }
    }
}

impl Driver {
    /// Wheel/pedals: no smoothing (the driver's hands and feet do that). Holding the brake pedal while
    /// stopped engages reverse, then the brake pedal works as reverse throttle (same rule as the S key).
    fn update_analog(&mut self, a: &Analog, handbrake: bool, fwd_speed: f64) -> Input {
        let thr = a.throttle.clamp(0.0, 1.0);
        let brk = a.brake.clamp(0.0, 1.0);
        if thr > 0.05 && fwd_speed > -1.0 {
            self.reverse = false;
        }
        if brk > 0.5 && thr < 0.05 && fwd_speed.abs() < 0.5 {
            self.reverse = true;
        }
        if self.reverse {
            // brake pedal = reverse throttle; throttle pedal = brake
            self.throttle = if fwd_speed < 1.0 { brk } else { 0.0 };
            self.brake = thr.max(if fwd_speed > 1.0 { brk } else { 0.0 });
        } else {
            self.throttle = thr;
            self.brake = brk;
        }
        self.steer = (a.steer.clamp(-1.0, 1.0)) * self.steer_max;
        self.handbrake = if handbrake { 1.0 } else { 0.0 };
        Input { throttle: self.throttle, brake: self.brake, steer_angle: self.steer, handbrake: self.handbrake, reverse: self.reverse, tcs: self.tcs, abs: self.abs }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steering_lock_shrinks_with_speed() {
        let mut d = Driver::new(0.55);
        let k = Keys { left: true, ..Default::default() };
        for _ in 0..2000 {
            d.update(0.001, &k, 0.0);
        }
        let slow = d.steer;
        let mut d = Driver::new(0.55);
        for _ in 0..2000 {
            d.update(0.001, &k, 40.0);
        }
        assert!(slow > 0.5 && d.steer < 0.12 && d.steer > 0.0, "{} {}", slow, d.steer);
    }

    #[test]
    fn analog_pedals_and_wheel() {
        let mut d = Driver::new(0.55);
        let k = Keys { analog: Some(Analog { steer: 0.5, throttle: 0.7, brake: 0.0 }), ..Default::default() };
        let i = d.update(0.001, &k, 20.0);
        assert_eq!((i.throttle, i.brake, i.reverse), (0.7, 0.0, false));
        assert!((i.steer_angle - 0.275).abs() < 1e-12); // no speed-dependent lock with a real wheel
        // brake while moving brakes, never reverses
        let k = Keys { analog: Some(Analog { steer: 0.0, throttle: 0.0, brake: 1.0 }), ..Default::default() };
        let i = d.update(0.001, &k, 10.0);
        assert!(i.brake == 1.0 && !i.reverse);
        // stopped + brake held -> reverse, brake pedal drives backwards
        let i = d.update(0.001, &k, 0.0);
        assert!(i.reverse && i.throttle == 1.0);
        // throttle pedal returns to forward
        let k = Keys { analog: Some(Analog { steer: 0.0, throttle: 0.6, brake: 0.0 }), ..Default::default() };
        let i = d.update(0.001, &k, -0.2);
        assert!(!i.reverse && i.throttle == 0.6);
    }

    #[test]
    fn down_key_brakes_then_reverses() {
        let mut d = Driver::new(0.55);
        let k = Keys { down: true, ..Default::default() };
        let i = d.update(0.01, &k, 10.0);
        assert!(!i.reverse && i.brake > 0.0 && i.throttle == 0.0);
        let mut i = i;
        for _ in 0..100 {
            i = d.update(0.01, &k, 0.2);
        }
        assert!(i.reverse && i.throttle > 0.5);
    }
}
