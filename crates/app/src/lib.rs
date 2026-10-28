//! earthsim — native driving app (own physics + OpenGL 3.3 renderer). Only `win` is Windows-specific;
//! everything else builds and is tested on any OS (rendering is verified headless through EGL on Linux).
pub mod cli;
pub mod env;
pub mod font;
pub mod game;
pub mod gl;
pub mod m4;
pub mod render;
pub mod shaders;
pub mod softmesh;
#[cfg(any(windows, earthsim_check_win))]
pub mod win;
