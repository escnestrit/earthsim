//! Headless render check (Linux): creates an OpenGL 3.3 core context through Mesa EGL (no window),
//! runs the real renderer + game, and writes PPM screenshots. Usage: headless <scene.json|-> <outdir>
#[cfg(target_os = "linux")]
mod run {
    use earthsim::game::{Game, Key};
    use earthsim::gl::*;
    use earthsim::render::Renderer;
    use scene::Scene;
    use std::ffi::{c_void, CString};
    use std::os::raw::{c_char, c_int};

    #[link(name = "dl")]
    extern "C" {
        fn dlopen(name: *const c_char, flags: c_int) -> *mut c_void;
        fn dlsym(h: *mut c_void, s: *const c_char) -> *mut c_void;
    }

    fn sym(h: *mut c_void, n: &str) -> *const c_void {
        let c = CString::new(n).unwrap();
        unsafe { dlsym(h, c.as_ptr()) as *const c_void }
    }

    pub fn main() {
        let args: Vec<String> = std::env::args().collect();
        let scene_path = args.get(1).map(|s| s.as_str()).unwrap_or("-");
        let out = args.get(2).cloned().unwrap_or_else(|| ".".into());
        std::fs::create_dir_all(&out).unwrap();
        let (w, h) = (1280, 720);
        unsafe {
            let egl = dlopen(c"libEGL.so.1".as_ptr(), 2);
            assert!(!egl.is_null(), "libEGL.so.1 not found");
            let get_proc: unsafe extern "C" fn(*const c_char) -> *const c_void = std::mem::transmute(sym(egl, "eglGetProcAddress"));
            let f_get_platform: unsafe extern "C" fn(u32, *mut c_void, *const isize) -> *mut c_void = std::mem::transmute(sym(egl, "eglGetPlatformDisplay"));
            let f_init: unsafe extern "C" fn(*mut c_void, *mut i32, *mut i32) -> u32 = std::mem::transmute(sym(egl, "eglInitialize"));
            let f_bind: unsafe extern "C" fn(u32) -> u32 = std::mem::transmute(sym(egl, "eglBindAPI"));
            let f_choose: unsafe extern "C" fn(*mut c_void, *const i32, *mut *mut c_void, i32, *mut i32) -> u32 = std::mem::transmute(sym(egl, "eglChooseConfig"));
            let f_ctx: unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *const i32) -> *mut c_void = std::mem::transmute(sym(egl, "eglCreateContext"));
            let f_pbuf: unsafe extern "C" fn(*mut c_void, *mut c_void, *const i32) -> *mut c_void = std::mem::transmute(sym(egl, "eglCreatePbufferSurface"));
            let f_make: unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *mut c_void) -> u32 = std::mem::transmute(sym(egl, "eglMakeCurrent"));
            let f_err: unsafe extern "C" fn() -> i32 = std::mem::transmute(sym(egl, "eglGetError"));

            let dpy = f_get_platform(0x31DD, std::ptr::null_mut(), std::ptr::null()); // EGL_PLATFORM_SURFACELESS_MESA
            assert!(!dpy.is_null(), "no EGL display");
            let (mut maj, mut min) = (0, 0);
            assert!(f_init(dpy, &mut maj, &mut min) == 1, "eglInitialize failed 0x{:X}", f_err());
            assert!(f_bind(0x30A2) == 1);
            let cfg_attr = [0x3033, 1, 0x3040, 8, 0x3038];
            let mut cfg: *mut c_void = std::ptr::null_mut();
            let mut n = 0;
            assert!(f_choose(dpy, cfg_attr.as_ptr(), &mut cfg, 1, &mut n) == 1 && n > 0, "no EGL config");
            let ctx_attr = [0x3098, 3, 0x30FB, 3, 0x30FD, 1, 0x3038];
            let ctx = f_ctx(dpy, cfg, std::ptr::null_mut(), ctx_attr.as_ptr());
            assert!(!ctx.is_null(), "context creation failed 0x{:X}", f_err());
            let pb_attr = [0x3057, 16, 0x3056, 16, 0x3038];
            let surf = f_pbuf(dpy, cfg, pb_attr.as_ptr());
            assert!(f_make(dpy, surf, surf, ctx) == 1, "eglMakeCurrent failed 0x{:X}", f_err());

            let libgl = dlopen(c"libGL.so.1".as_ptr(), 2);
            let loader = |name: &str| -> *const c_void {
                let c = CString::new(name).unwrap();
                let p = get_proc(c.as_ptr());
                if p.is_null() && !libgl.is_null() { sym(libgl, name) } else { p }
            };
            let gl = Gl::load(&loader).expect("GL load");

            // offscreen target
            let (mut fbo, mut cb, mut db) = (0u32, 0u32, 0u32);
            (gl.GenFramebuffers)(1, &mut fbo);
            (gl.BindFramebuffer)(FRAMEBUFFER, fbo);
            (gl.GenRenderbuffers)(1, &mut cb);
            (gl.BindRenderbuffer)(RENDERBUFFER, cb);
            (gl.RenderbufferStorage)(RENDERBUFFER, RGBA8 as u32, w, h);
            (gl.FramebufferRenderbuffer)(FRAMEBUFFER, COLOR_ATTACHMENT0, RENDERBUFFER, cb);
            (gl.GenRenderbuffers)(1, &mut db);
            (gl.BindRenderbuffer)(RENDERBUFFER, db);
            (gl.RenderbufferStorage)(RENDERBUFFER, DEPTH_COMPONENT24 as u32, w, h);
            (gl.FramebufferRenderbuffer)(FRAMEBUFFER, DEPTH_ATTACHMENT, RENDERBUFFER, db);
            assert_eq!((gl.CheckFramebufferStatus)(FRAMEBUFFER), FRAMEBUFFER_COMPLETE);

            let json = if scene_path == "-" || scene_path == "stream" || scene_path == "crash" {
                include_str!("../../../data/demo_scene.json").to_string()
            } else {
                std::fs::read_to_string(scene_path).expect("read scene")
            };
            let streaming = scene_path == "stream";
            let mut game = if streaming {
                let o = earthsim::cli::Options { mode: earthsim::cli::Mode::Offline, lat: earthsim::cli::MINSK.0, lon: earthsim::cli::MINSK.1, cache: None, car: None, car_part: None };
                earthsim::game::build_game(&o, None, std::env::temp_dir()).expect("offline game")
            } else if scene_path == "crash" {
                Game::new(Scene::default())
            } else {
                Game::new(Scene::from_json_str(&json).expect("scene"))
            };
            let mut r = match Renderer::new(gl, &game.car_meshes.0, &game.car_meshes.1) {
                Ok(r) => r,
                Err(e) => {
                    eprintln!("RENDERER INIT FAILED:\n{e}");
                    std::process::exit(2);
                }
            };
            println!("GL: {}", r.info);
            r.set_target(fbo, w, h);
            r.set_scene(&game.meshes);
            if streaming {
                let t0 = std::time::Instant::now();
                while t0.elapsed().as_secs() < 20 {
                    game.update(0.016);
                    r.apply_ops(game.take_render_ops());
                    if game.hud().lines.iter().any(|l| l.starts_with("TILES LOADED 9") || l.starts_with("TILES LOADED 1")) && !game.hud().lines.iter().any(|l| l.contains("LOADING")) && game.hud().lines.iter().any(|l| l.contains("FETCHING 0  BUILDING 0")) {
                        break;
                    }
                    std::thread::sleep(std::time::Duration::from_millis(5));
                }
                println!("streaming: {} tiles on the GPU, {} collision polygons", r.tile_count(), game.world.poly_count());
            }
            game.renderer_info = r.info.clone();

            let shot = |game: &mut Game, r: &mut Renderer, name: &str| {
                r.apply_ops(game.take_render_ops());
                r.set_deformed_body(game.deformed_body());
                let f = game.frame_params();
                let hud = game.hud();
                let st = r.render(&f, &hud);
                game.draws = st.draws;
                game.triangles = st.triangles;
                let px = r.read_pixels();
                if let Some(e) = r.check_error() {
                    eprintln!("GL error 0x{e:X} after {name}");
                }
                let mut ppm = format!("P6\n{} {}\n255\n", w, h).into_bytes();
                for y in (0..h as usize).rev() {
                    for x in 0..w as usize {
                        let i = (y * w as usize + x) * 4;
                        ppm.extend_from_slice(&px[i..i + 3]);
                    }
                }
                std::fs::write(format!("{out}/{name}.ppm"), ppm).unwrap();
                let lit = px.chunks(4).filter(|p| p[0] as u32 + p[1] as u32 + p[2] as u32 > 30).count();
                println!("{name}: draws {} tris {} lit-pixels {:.0}%", st.draws, st.triangles, 100.0 * lit as f64 / (w * h) as f64);
            };

            if scene_path == "crash" {
                game.world.add_poly(vec![(70.0, -15.0), (80.0, -15.0), (80.0, 15.0), (70.0, 15.0)]);
                game.key_state(Key::Up, true);
                for _ in 0..(60 * 30) {
                    game.update(1.0 / 60.0);
                    if game.car.impact > 500.0 {
                        break;
                    }
                }
                game.key_state(Key::Up, false);
                for _ in 0..90 {
                    game.update(1.0 / 60.0);
                }
                shot(&mut game, &mut r, "c1_after_crash_chase");
                let cp = game.car.pos;
                let (cx, cy) = (cp.x as f32, cp.y as f32);
                game.cam_override = Some(([cx + 1.5, cy - 7.0, 2.2], [cx + 1.0, cy, 0.7], 50.0));
                shot(&mut game, &mut r, "c2_after_crash_side");
                game.cam_override = Some(([cx + 8.0, cy - 5.0, 3.0], [cx + 1.0, cy, 0.7], 50.0));
                shot(&mut game, &mut r, "c3_after_crash_front34");
                println!("damage: {:?}", game.soft.damage());
                return;
            }
            game.update(0.016);
            shot(&mut game, &mut r, "01_start_chase_15h");
            game.key_state(Key::Up, true);
            for _ in 0..(60 * 5) {
                game.update(1.0 / 60.0);
            }
            shot(&mut game, &mut r, "02_driving_chase_15h");
            game.key_state(Key::Camera, true);
            shot(&mut game, &mut r, "03_hood_15h");
            game.key_state(Key::Camera, true);
            shot(&mut game, &mut r, "04_top_15h");
            game.key_state(Key::Camera, true);
            game.hour = 21.0;
            shot(&mut game, &mut r, "05_night_chase_21h");
            game.hour = 18.0;
            shot(&mut game, &mut r, "06_sunset_chase_18h");
            game.hour = 8.0;
            game.key_state(Key::Handbrake, true);
            game.key_state(Key::Left, true);
            for _ in 0..40 {
                game.update(1.0 / 60.0);
            }
            shot(&mut game, &mut r, "07_handbrake_slide_8h");
        }
    }
}

fn main() {
    #[cfg(target_os = "linux")]
    run::main();
    #[cfg(not(target_os = "linux"))]
    eprintln!("headless example is Linux-only");
}
