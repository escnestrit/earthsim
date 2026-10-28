//! Win32 + WGL front end: window, OpenGL 3.3 core context, message loop, keyboard mapping.
//! NOTE: this file is only compiled on Windows and could not be compiled in the development sandbox
//! (no Windows target available there). Everything it drives (renderer, game, physics) is tested elsewhere.
#![allow(non_snake_case, non_camel_case_types, clippy::upper_case_acronyms)]

use crate::game::Key;
use crate::gl::Gl;
use crate::render::Renderer;
use std::cell::RefCell;
use std::ffi::{c_void, CString};
use std::os::raw::{c_char, c_int};

type HWND = *mut c_void;
type HINSTANCE = *mut c_void;
type HDC = *mut c_void;
type HGLRC = *mut c_void;
type HMENU = *mut c_void;
type HICON = *mut c_void;
type HCURSOR = *mut c_void;
type HBRUSH = *mut c_void;
type WPARAM = usize;
type LPARAM = isize;
type LRESULT = isize;

type WndProc = unsafe extern "system" fn(HWND, u32, WPARAM, LPARAM) -> LRESULT;

#[repr(C)]
struct WNDCLASSEXW {
    cb_size: u32,
    style: u32,
    wnd_proc: Option<WndProc>,
    cls_extra: c_int,
    wnd_extra: c_int,
    instance: HINSTANCE,
    icon: HICON,
    cursor: HCURSOR,
    background: HBRUSH,
    menu_name: *const u16,
    class_name: *const u16,
    icon_sm: HICON,
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct POINT {
    x: i32,
    y: i32,
}

#[repr(C)]
struct MSG {
    hwnd: HWND,
    message: u32,
    wparam: WPARAM,
    lparam: LPARAM,
    time: u32,
    pt: POINT,
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct RECT {
    left: i32,
    top: i32,
    right: i32,
    bottom: i32,
}

#[repr(C)]
struct PIXELFORMATDESCRIPTOR {
    n_size: u16,
    n_version: u16,
    flags: u32,
    pixel_type: u8,
    color_bits: u8,
    red_bits: u8,
    red_shift: u8,
    green_bits: u8,
    green_shift: u8,
    blue_bits: u8,
    blue_shift: u8,
    alpha_bits: u8,
    alpha_shift: u8,
    accum_bits: u8,
    accum_red_bits: u8,
    accum_green_bits: u8,
    accum_blue_bits: u8,
    accum_alpha_bits: u8,
    depth_bits: u8,
    stencil_bits: u8,
    aux_buffers: u8,
    layer_type: u8,
    reserved: u8,
    layer_mask: u32,
    visible_mask: u32,
    damage_mask: u32,
}

#[link(name = "kernel32")]
extern "system" {
    fn GetModuleHandleW(name: *const u16) -> HINSTANCE;
    fn LoadLibraryW(name: *const u16) -> *mut c_void;
    fn GetProcAddress(module: *mut c_void, name: *const c_char) -> *const c_void;
    fn QueryPerformanceCounter(v: *mut i64) -> i32;
    fn QueryPerformanceFrequency(v: *mut i64) -> i32;
    fn Sleep(ms: u32);
}

#[link(name = "user32")]
extern "system" {
    fn RegisterClassExW(c: *const WNDCLASSEXW) -> u16;
    fn CreateWindowExW(ex: u32, class: *const u16, title: *const u16, style: u32, x: c_int, y: c_int, w: c_int, h: c_int, parent: HWND, menu: HMENU, inst: HINSTANCE, param: *mut c_void) -> HWND;
    fn DefWindowProcW(h: HWND, m: u32, w: WPARAM, l: LPARAM) -> LRESULT;
    fn ShowWindow(h: HWND, cmd: c_int) -> i32;
    fn PeekMessageW(msg: *mut MSG, h: HWND, min: u32, max: u32, remove: u32) -> i32;
    fn TranslateMessage(msg: *const MSG) -> i32;
    fn DispatchMessageW(msg: *const MSG) -> LRESULT;
    fn GetDC(h: HWND) -> HDC;
    fn DestroyWindow(h: HWND) -> i32;
    fn PostQuitMessage(code: c_int);
    fn LoadCursorW(inst: HINSTANCE, name: *const u16) -> HCURSOR;
    fn AdjustWindowRect(r: *mut RECT, style: u32, menu: i32) -> i32;
    fn GetClientRect(h: HWND, r: *mut RECT) -> i32;
    fn GetWindowRect(h: HWND, r: *mut RECT) -> i32;
    fn SetWindowTextW(h: HWND, t: *const u16) -> i32;
    fn SetWindowLongPtrW(h: HWND, idx: c_int, v: isize) -> isize;
    fn SetWindowPos(h: HWND, after: HWND, x: c_int, y: c_int, w: c_int, hh: c_int, flags: u32) -> i32;
    fn GetSystemMetrics(i: c_int) -> c_int;
    fn MessageBoxW(h: HWND, text: *const u16, caption: *const u16, kind: u32) -> c_int;
    fn SetProcessDPIAware() -> i32;
}

#[link(name = "gdi32")]
extern "system" {
    fn ChoosePixelFormat(dc: HDC, pfd: *const PIXELFORMATDESCRIPTOR) -> c_int;
    fn SetPixelFormat(dc: HDC, fmt: c_int, pfd: *const PIXELFORMATDESCRIPTOR) -> i32;
    fn SwapBuffers(dc: HDC) -> i32;
}

#[link(name = "opengl32")]
extern "system" {
    fn wglCreateContext(dc: HDC) -> HGLRC;
    fn wglMakeCurrent(dc: HDC, rc: HGLRC) -> i32;
    fn wglDeleteContext(rc: HGLRC) -> i32;
    fn wglGetProcAddress(name: *const c_char) -> *const c_void;
}

const WM_DESTROY: u32 = 0x0002;
const WM_SIZE: u32 = 0x0005;
const WM_KILLFOCUS: u32 = 0x0008;
const WM_CLOSE: u32 = 0x0010;
const WM_ERASEBKGND: u32 = 0x0014;
const WM_KEYDOWN: u32 = 0x0100;
const WM_KEYUP: u32 = 0x0101;
const WM_SYSKEYDOWN: u32 = 0x0104;
const WM_SYSKEYUP: u32 = 0x0105;
const WM_QUIT: u32 = 0x0012;
const PM_REMOVE: u32 = 1;
const CS_OWNDC: u32 = 0x0020;
const CS_HREDRAW: u32 = 0x0002;
const CS_VREDRAW: u32 = 0x0001;
const WS_OVERLAPPEDWINDOW: u32 = 0x00CF0000;
const WS_POPUP: u32 = 0x80000000;
const WS_VISIBLE: u32 = 0x10000000;
const CW_USEDEFAULT: c_int = 0x80000000u32 as c_int;
const SW_SHOW: c_int = 5;
const IDC_ARROW: usize = 32512;
const PFD_DRAW_TO_WINDOW: u32 = 0x4;
const PFD_SUPPORT_OPENGL: u32 = 0x20;
const PFD_DOUBLEBUFFER: u32 = 0x1;
const PFD_TYPE_RGBA: u8 = 0;
const GWL_STYLE: c_int = -16;
const SWP_FRAMECHANGED: u32 = 0x20;
const SWP_SHOWWINDOW: u32 = 0x40;
const MB_ICONERROR: u32 = 0x10;

#[derive(Clone, Copy, Debug)]
enum Event {
    Key { vk: u32, down: bool, repeat: bool },
    Resize(i32, i32),
    Close,
    LostFocus,
}

thread_local! {
    static QUEUE: RefCell<Vec<Event>> = const { RefCell::new(Vec::new()) };
}

fn push(e: Event) {
    QUEUE.with(|q| q.borrow_mut().push(e));
}

fn wide(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(std::iter::once(0)).collect()
}

fn error_box(msg: &str) {
    eprintln!("ERROR: {msg}");
    unsafe {
        MessageBoxW(std::ptr::null_mut(), wide(msg).as_ptr(), wide("earthsim").as_ptr(), MB_ICONERROR);
    }
}

unsafe extern "system" fn wnd_proc(h: HWND, m: u32, w: WPARAM, l: LPARAM) -> LRESULT {
    match m {
        WM_CLOSE => {
            push(Event::Close);
            0
        }
        WM_DESTROY => {
            PostQuitMessage(0);
            0
        }
        WM_SIZE => {
            push(Event::Resize((l & 0xFFFF) as i32, ((l >> 16) & 0xFFFF) as i32));
            0
        }
        WM_KILLFOCUS => {
            push(Event::LostFocus);
            0
        }
        WM_ERASEBKGND => 1,
        WM_KEYDOWN | WM_SYSKEYDOWN => {
            push(Event::Key { vk: w as u32, down: true, repeat: (l >> 30) & 1 == 1 });
            if m == WM_SYSKEYDOWN && (w as u32 == 0x73) {
                return DefWindowProcW(h, m, w, l); // let Alt+F4 through
            }
            0
        }
        WM_KEYUP | WM_SYSKEYUP => {
            push(Event::Key { vk: w as u32, down: false, repeat: false });
            0
        }
        _ => DefWindowProcW(h, m, w, l),
    }
}

fn map_key(vk: u32) -> Option<Key> {
    Some(match vk {
        0x57 | 0x26 => Key::Up,     // W, Up
        0x53 | 0x28 => Key::Down,   // S, Down
        0x41 | 0x25 => Key::Left,   // A, Left
        0x44 | 0x27 => Key::Right,  // D, Right
        0x20 => Key::Handbrake,     // Space
        0x43 => Key::Camera,        // C
        0xDD => Key::HourUp,        // ]
        0xDB => Key::HourDown,      // [
        0x4C => Key::Lights,        // L
        0x52 => Key::Respawn,       // R
        0x55 => Key::Recover,       // U
        0x48 => Key::Hud,           // H
        0x54 => Key::Tcs,           // T
        0x42 => Key::Abs,           // B
        0x70 => Key::Help,          // F1
        0x71 => Key::Teleport,      // F2
        0x72 => Key::SoftBody,      // F3
        _ => return None,
    })
}

fn create_gl_context(dc: HDC) -> Result<HGLRC, String> {
    unsafe {
        let pfd = PIXELFORMATDESCRIPTOR {
            n_size: std::mem::size_of::<PIXELFORMATDESCRIPTOR>() as u16,
            n_version: 1,
            flags: PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
            pixel_type: PFD_TYPE_RGBA,
            color_bits: 32,
            red_bits: 0, red_shift: 0, green_bits: 0, green_shift: 0, blue_bits: 0, blue_shift: 0,
            alpha_bits: 8, alpha_shift: 0, accum_bits: 0, accum_red_bits: 0, accum_green_bits: 0, accum_blue_bits: 0, accum_alpha_bits: 0,
            depth_bits: 24,
            stencil_bits: 8,
            aux_buffers: 0, layer_type: 0, reserved: 0, layer_mask: 0, visible_mask: 0, damage_mask: 0,
        };
        let fmt = ChoosePixelFormat(dc, &pfd);
        if fmt == 0 {
            return Err("ChoosePixelFormat failed".into());
        }
        if SetPixelFormat(dc, fmt, &pfd) == 0 {
            return Err("SetPixelFormat failed".into());
        }
        let legacy = wglCreateContext(dc);
        if legacy.is_null() || wglMakeCurrent(dc, legacy) == 0 {
            return Err("Could not create a legacy OpenGL context (is a graphics driver installed?)".into());
        }
        let name = CString::new("wglCreateContextAttribsARB").unwrap();
        let p = wglGetProcAddress(name.as_ptr());
        if p.is_null() {
            return Err("wglCreateContextAttribsARB is not available: the driver does not support OpenGL 3.3".into());
        }
        let create: unsafe extern "system" fn(HDC, HGLRC, *const c_int) -> HGLRC = std::mem::transmute(p);
        let attribs: [c_int; 7] = [0x2091, 3, 0x2092, 3, 0x9126, 1, 0];
        let ctx = create(dc, std::ptr::null_mut(), attribs.as_ptr());
        wglMakeCurrent(std::ptr::null_mut(), std::ptr::null_mut());
        wglDeleteContext(legacy);
        if ctx.is_null() {
            return Err("Could not create an OpenGL 3.3 core context (update your graphics driver)".into());
        }
        if wglMakeCurrent(dc, ctx) == 0 {
            return Err("wglMakeCurrent failed".into());
        }
        // vsync on (if the extension exists)
        let sw = CString::new("wglSwapIntervalEXT").unwrap();
        let p = wglGetProcAddress(sw.as_ptr());
        if !p.is_null() {
            let f: unsafe extern "system" fn(c_int) -> i32 = std::mem::transmute(p);
            f(1);
        }
        Ok(ctx)
    }
}

pub fn run() {
    unsafe {
        SetProcessDPIAware();
        let inst = GetModuleHandleW(std::ptr::null());
        let class = wide("EarthSimWindow");
        let wc = WNDCLASSEXW {
            cb_size: std::mem::size_of::<WNDCLASSEXW>() as u32,
            style: CS_OWNDC | CS_HREDRAW | CS_VREDRAW,
            wnd_proc: Some(wnd_proc),
            cls_extra: 0,
            wnd_extra: 0,
            instance: inst,
            icon: std::ptr::null_mut(),
            cursor: LoadCursorW(std::ptr::null_mut(), IDC_ARROW as *const u16),
            background: std::ptr::null_mut(),
            menu_name: std::ptr::null(),
            class_name: class.as_ptr(),
            icon_sm: std::ptr::null_mut(),
        };
        if RegisterClassExW(&wc) == 0 {
            return error_box("RegisterClassExW failed");
        }
        let mut r = RECT { left: 0, top: 0, right: 1280, bottom: 720 };
        AdjustWindowRect(&mut r, WS_OVERLAPPEDWINDOW, 0);
        let title = wide("BeamNG.Earth prototype - earthsim");
        let hwnd = CreateWindowExW(0, class.as_ptr(), title.as_ptr(), WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, r.right - r.left, r.bottom - r.top, std::ptr::null_mut(), std::ptr::null_mut(), inst, std::ptr::null_mut());
        if hwnd.is_null() {
            return error_box("CreateWindowExW failed");
        }
        let dc = GetDC(hwnd);
        let ctx = match create_gl_context(dc) {
            Ok(c) => c,
            Err(e) => {
                DestroyWindow(hwnd);
                return error_box(&e);
            }
        };

        // GL function loader: wglGetProcAddress for 1.2+, opengl32.dll exports for 1.0/1.1.
        let gl_dll = LoadLibraryW(wide("opengl32.dll").as_ptr());
        let loader = |name: &str| -> *const c_void {
            let c = CString::new(name).unwrap();
            let p = wglGetProcAddress(c.as_ptr());
            let v = p as usize;
            if v == 0 || v == 1 || v == 2 || v == 3 || v == usize::MAX { GetProcAddress(gl_dll, c.as_ptr()) } else { p }
        };
        let gl = match Gl::load(&loader) {
            Ok(g) => g,
            Err(e) => return error_box(&e),
        };

        let opts = match crate::cli::parse(std::env::args().skip(1)) {
            Ok(o) => o,
            Err(e) => return error_box(&e),
        };
        let cache_dir = std::env::var("EARTHSIM_CACHE").ok().map(std::path::PathBuf::from).or_else(|| opts.cache.clone().map(Into::into)).unwrap_or_else(|| {
            std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.join("cache"))).unwrap_or_else(|| "cache".into())
        });
        let log_path = std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.join("earthsim.log"))).unwrap_or_else(|| "earthsim.log".into());
        world::log::init(&log_path);
        let live: Option<std::sync::Arc<dyn world::fetch::Fetcher>> = Some(std::sync::Arc::new(world::fetch::WinHttpFetcher::from_env()));
        let mut game = match crate::game::build_game(&opts, live, cache_dir.clone()) {
            Ok(g) => g,
            Err(e) => return error_box(&e),
        };
        let mut renderer = match Renderer::new(gl, &game.car_meshes.0, &game.car_meshes.1) {
            Ok(r) => r,
            Err(e) => return error_box(&e),
        };
        renderer.set_scene(&game.meshes);
        renderer.apply_ops(game.take_render_ops());
        game.renderer_info = renderer.info.clone();
        println!("{}", renderer.info);
        println!("mode: {:?}   cache: {}", opts.mode, cache_dir.display());

        let mut crect = RECT::default();
        GetClientRect(hwnd, &mut crect);
        renderer.set_target(0, crect.right - crect.left, crect.bottom - crect.top);
        ShowWindow(hwnd, SW_SHOW);

        let (mut freq, mut last) = (0i64, 0i64);
        QueryPerformanceFrequency(&mut freq);
        QueryPerformanceCounter(&mut last);
        let mut fullscreen = false;
        let mut saved = RECT::default();
        let mut minimized = false;
        let mut running = true;
        while running {
            let mut msg: MSG = std::mem::zeroed();
            while PeekMessageW(&mut msg, std::ptr::null_mut(), 0, 0, PM_REMOVE) != 0 {
                if msg.message == WM_QUIT {
                    running = false;
                }
                TranslateMessage(&msg);
                DispatchMessageW(&msg);
            }
            let events: Vec<Event> = QUEUE.with(|q| std::mem::take(&mut *q.borrow_mut()));
            for e in events {
                match e {
                    Event::Close => running = false,
                    Event::LostFocus => game.release_all(),
                    Event::Resize(w, h) => {
                        minimized = w == 0 || h == 0;
                        if !minimized {
                            renderer.set_target(0, w, h);
                        }
                    }
                    Event::Key { vk, down, repeat } => {
                        if vk == 0x1B && down {
                            running = false;
                        } else if vk == 0x7A && down && !repeat {
                            // F11: borderless fullscreen toggle
                            fullscreen = !fullscreen;
                            if fullscreen {
                                GetWindowRect(hwnd, &mut saved);
                                SetWindowLongPtrW(hwnd, GWL_STYLE, (WS_POPUP | WS_VISIBLE) as isize);
                                SetWindowPos(hwnd, std::ptr::null_mut(), 0, 0, GetSystemMetrics(0), GetSystemMetrics(1), SWP_FRAMECHANGED | SWP_SHOWWINDOW);
                            } else {
                                SetWindowLongPtrW(hwnd, GWL_STYLE, (WS_OVERLAPPEDWINDOW | WS_VISIBLE) as isize);
                                SetWindowPos(hwnd, std::ptr::null_mut(), saved.left, saved.top, saved.right - saved.left, saved.bottom - saved.top, SWP_FRAMECHANGED | SWP_SHOWWINDOW);
                            }
                        } else if let Some(k) = map_key(vk) {
                            if !(repeat && down) {
                                game.key_state(k, down);
                            }
                        }
                    }
                }
            }
            let mut now = 0i64;
            QueryPerformanceCounter(&mut now);
            let dt = (now - last) as f64 / freq as f64;
            last = now;
            if minimized {
                Sleep(50);
                continue;
            }
            game.update(dt);
            renderer.apply_ops(game.take_render_ops());
            renderer.set_deformed_body(game.deformed_body());
            let f = game.frame_params();
            let hud = game.hud();
            let st = renderer.render(&f, &hud);
            game.draws = st.draws;
            game.triangles = st.triangles;
            SwapBuffers(dc);
        }
        wglMakeCurrent(std::ptr::null_mut(), std::ptr::null_mut());
        wglDeleteContext(ctx);
        DestroyWindow(hwnd);
        let _ = SetWindowTextW;
    }
}
