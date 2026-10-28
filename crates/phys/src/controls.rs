//! Keyboard -> vehicle input: smoothed pedals, speed-sensitive steering, automatic reverse.
use crate::vehicle::Input;

#[derive(Clone, Copy, Debug, Default)]
pub struct Keys {
    pub up: bool,
    pub down: bool,
    pub left: bool,
    pub right: bool,
    pub handbrake: bool,
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
