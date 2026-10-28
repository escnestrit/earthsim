#[cfg(windows)]
fn main() {
    earthsim::win::run();
}

#[cfg(not(windows))]
fn main() {
    eprintln!("earthsim is a Windows application (Win32 + OpenGL 3.3). On other systems only the library and tests build.");
    eprintln!("Run `cargo test --release` to run the physics, scene, game-logic and headless render tests.");
}
