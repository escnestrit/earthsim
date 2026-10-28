use earthsim::game::*;
use scene::Scene;
fn main() {
    let s = std::fs::read_to_string("data/demo_scene.json").unwrap();
    let mut g = Game::new(Scene::from_json_str(&s).unwrap());
    g.key_state(Key::Up, true);
    let t = std::time::Instant::now();
    let frames = 60 * 60;
    for i in 0..frames {
        if i % 300 == 0 { g.key_state(Key::Left, i % 600 == 0); }
        g.update(1.0 / 60.0);
        if g.car.impact > 1000.0 { g.respawn(); }
    }
    let el = t.elapsed().as_secs_f64();
    println!("{} s of game time simulated in {:.2} s => {:.1} us per 1 ms physics step", frames as f64 / 60.0, el, el * 1e6 / (frames as f64 * 1000.0 / 60.0 * 1.0));
}
