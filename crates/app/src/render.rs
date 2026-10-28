//! OpenGL renderer: shadow pass, sky, lit scene with procedural materials, HUD. Platform independent —
//! it only needs a current GL 3.3 core context and a loaded `Gl`.
use crate::env::{lighting, Lighting};
use crate::font;
use crate::game::RenderOp;
use crate::gl::*;
use crate::m4::{self, M4, V3f};
use crate::shaders;
use scene::mesh::FLOATS_PER_VERTEX;
use scene::MeshData;
use std::cell::RefCell;
use std::collections::HashMap;
use std::ffi::{c_void, CString};

const SHADOW_SIZE: i32 = 2048;

struct Prog {
    id: GLuint,
    locs: RefCell<HashMap<String, GLint>>,
}

struct GpuMesh {
    vao: GLuint,
    vbo: GLuint,
    count: i32,
    polygon_offset: f32,
    casts_shadow: bool,
    name: String,
}

#[derive(Clone, Debug)]
pub struct FrameParams {
    pub eye: V3f,
    pub target: V3f,
    pub fov_deg: f32,
    pub body: M4,
    pub wheels: [M4; 4],
    pub draw_car: bool,
    pub car_pos: V3f,
    pub head_pos: V3f,
    pub head_dir: V3f,
    pub headlights: bool,
    pub braking: bool,
    pub hour: f32,
    pub paint: V3f,
}

#[derive(Clone, Debug, Default)]
pub struct HudData {
    pub speed_kmh: f32,
    pub gear: i32,
    pub rpm: f32,
    pub rpm_max: f32,
    pub lines: Vec<String>,
    pub warn: String,
    pub warn_is_demo: bool,
    pub help: String,
    pub show: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DrawStats {
    pub draws: u32,
    pub triangles: u64,
}

pub struct Renderer {
    gl: Gl,
    p_main: Prog,
    p_shadow: Prog,
    p_sky: Prog,
    p_hud: Prog,
    statics: Vec<GpuMesh>,
    tiles: HashMap<u64, Vec<GpuMesh>>,
    follow_ground: Option<GpuMesh>,
    car_dyn: Option<GpuMesh>,
    car_body: GpuMesh,
    car_wheel: GpuMesh,
    shadow_tex: GLuint,
    shadow_fbo: GLuint,
    font_tex: GLuint,
    hud_vao: GLuint,
    hud_vbo: GLuint,
    empty_vao: GLuint,
    target_fbo: GLuint,
    pub width: i32,
    pub height: i32,
    pub info: String,
}

fn compile(gl: &Gl, kind: GLenum, src: &str, what: &str) -> Result<GLuint, String> {
    unsafe {
        let s = (gl.CreateShader)(kind);
        let ptr = src.as_ptr() as *const GLchar;
        let len = src.len() as GLint;
        (gl.ShaderSource)(s, 1, &ptr, &len);
        (gl.CompileShader)(s);
        let mut ok = 0;
        (gl.GetShaderiv)(s, COMPILE_STATUS, &mut ok);
        if ok == 0 {
            let mut n = 0;
            (gl.GetShaderiv)(s, INFO_LOG_LENGTH, &mut n);
            let mut buf = vec![0u8; n.max(1) as usize];
            let mut w = 0;
            (gl.GetShaderInfoLog)(s, n, &mut w, buf.as_mut_ptr() as *mut GLchar);
            return Err(format!("{what} shader failed to compile:\n{}", String::from_utf8_lossy(&buf[..w.max(0) as usize])));
        }
        Ok(s)
    }
}

fn link(gl: &Gl, vs: &str, fs: &str, what: &str) -> Result<Prog, String> {
    unsafe {
        let v = compile(gl, VERTEX_SHADER, vs, &format!("{what} vertex"))?;
        let f = compile(gl, FRAGMENT_SHADER, fs, &format!("{what} fragment"))?;
        let p = (gl.CreateProgram)();
        (gl.AttachShader)(p, v);
        (gl.AttachShader)(p, f);
        (gl.LinkProgram)(p);
        let mut ok = 0;
        (gl.GetProgramiv)(p, LINK_STATUS, &mut ok);
        if ok == 0 {
            let mut n = 0;
            (gl.GetProgramiv)(p, INFO_LOG_LENGTH, &mut n);
            let mut buf = vec![0u8; n.max(1) as usize];
            let mut w = 0;
            (gl.GetProgramInfoLog)(p, n, &mut w, buf.as_mut_ptr() as *mut GLchar);
            return Err(format!("{what} program failed to link:\n{}", String::from_utf8_lossy(&buf[..w.max(0) as usize])));
        }
        (gl.DeleteShader)(v);
        (gl.DeleteShader)(f);
        Ok(Prog { id: p, locs: RefCell::new(HashMap::new()) })
    }
}

fn upload(gl: &Gl, m: &MeshData) -> GpuMesh {
    // facade styles are newer than this renderer's shader: draw them as plain walls
    let remapped;
    let m = if m.verts.chunks_exact(FLOATS_PER_VERTEX).any(|v| scene::mesh::mat::is_facade(v[8])) {
        let mut verts = m.verts.clone();
        for v in verts.chunks_exact_mut(FLOATS_PER_VERTEX) {
            if scene::mesh::mat::is_facade(v[8]) {
                v[8] = scene::mesh::mat::WALL;
                v[6] = 0.5;
            }
        }
        remapped = MeshData { verts, ..m.clone() };
        &remapped
    } else {
        m
    };
    unsafe {
        let (mut vao, mut vbo) = (0, 0);
        (gl.GenVertexArrays)(1, &mut vao);
        (gl.BindVertexArray)(vao);
        (gl.GenBuffers)(1, &mut vbo);
        (gl.BindBuffer)(ARRAY_BUFFER, vbo);
        (gl.BufferData)(ARRAY_BUFFER, (m.verts.len() * 4) as GLsizeiptr, if m.verts.is_empty() { std::ptr::null() } else { m.verts.as_ptr() as *const c_void }, STATIC_DRAW);
        let stride = (FLOATS_PER_VERTEX * 4) as GLsizei;
        for (loc, size, off) in [(0u32, 3, 0usize), (1, 3, 12), (2, 2, 24), (3, 1, 32)] {
            (gl.EnableVertexAttribArray)(loc);
            (gl.VertexAttribPointer)(loc, size, FLOAT, FALSE, stride, off as *const c_void);
        }
        (gl.BindVertexArray)(0);
        GpuMesh { vao, vbo, count: m.vertex_count() as i32, polygon_offset: m.polygon_offset, casts_shadow: m.casts_shadow, name: m.name.clone() }
    }
}

impl Renderer {
    pub fn new(gl: Gl, car_body: &MeshData, car_wheel: &MeshData) -> Result<Renderer, String> {
        let p_main = link(&gl, shaders::MAIN_VS, shaders::MAIN_FS, "scene")?;
        let p_shadow = link(&gl, shaders::SHADOW_VS, shaders::SHADOW_FS, "shadow")?;
        let p_sky = link(&gl, shaders::SKY_VS, shaders::SKY_FS, "sky")?;
        let p_hud = link(&gl, shaders::HUD_VS, shaders::HUD_FS, "hud")?;
        let cstr = |e: GLenum| unsafe {
            let p = (gl.GetString)(e);
            if p.is_null() { String::new() } else { std::ffi::CStr::from_ptr(p as *const _).to_string_lossy().into_owned() }
        };
        let info = format!("{} / OpenGL {}", cstr(RENDERER), cstr(VERSION));
        let (mut shadow_tex, mut shadow_fbo, mut font_tex, mut hud_vao, mut hud_vbo, mut empty_vao) = (0, 0, 0, 0, 0, 0);
        unsafe {
            // shadow map
            (gl.GenTextures)(1, &mut shadow_tex);
            (gl.BindTexture)(TEXTURE_2D, shadow_tex);
            (gl.TexImage2D)(TEXTURE_2D, 0, DEPTH_COMPONENT24, SHADOW_SIZE, SHADOW_SIZE, 0, DEPTH_COMPONENT, UNSIGNED_INT, std::ptr::null());
            (gl.TexParameteri)(TEXTURE_2D, TEXTURE_MIN_FILTER, LINEAR);
            (gl.TexParameteri)(TEXTURE_2D, TEXTURE_MAG_FILTER, LINEAR);
            (gl.TexParameteri)(TEXTURE_2D, TEXTURE_WRAP_S, CLAMP_TO_EDGE);
            (gl.TexParameteri)(TEXTURE_2D, TEXTURE_WRAP_T, CLAMP_TO_EDGE);
            (gl.TexParameteri)(TEXTURE_2D, TEXTURE_COMPARE_MODE, COMPARE_REF_TO_TEXTURE as GLint);
            (gl.TexParameteri)(TEXTURE_2D, TEXTURE_COMPARE_FUNC, LEQUAL as GLint);
            (gl.GenFramebuffers)(1, &mut shadow_fbo);
            (gl.BindFramebuffer)(FRAMEBUFFER, shadow_fbo);
            (gl.FramebufferTexture2D)(FRAMEBUFFER, DEPTH_ATTACHMENT, TEXTURE_2D, shadow_tex, 0);
            (gl.DrawBuffer)(NONE);
            (gl.ReadBuffer)(NONE);
            let st = (gl.CheckFramebufferStatus)(FRAMEBUFFER);
            (gl.BindFramebuffer)(FRAMEBUFFER, 0);
            if st != FRAMEBUFFER_COMPLETE {
                return Err(format!("shadow framebuffer incomplete (0x{st:X})"));
            }
            // font atlas
            let (px, w, h) = font::atlas();
            (gl.PixelStorei)(UNPACK_ALIGNMENT, 1);
            (gl.GenTextures)(1, &mut font_tex);
            (gl.BindTexture)(TEXTURE_2D, font_tex);
            (gl.TexImage2D)(TEXTURE_2D, 0, R8, w as GLsizei, h as GLsizei, 0, RED, UNSIGNED_BYTE, px.as_ptr() as *const c_void);
            (gl.TexParameteri)(TEXTURE_2D, TEXTURE_MIN_FILTER, NEAREST);
            (gl.TexParameteri)(TEXTURE_2D, TEXTURE_MAG_FILTER, NEAREST);
            (gl.TexParameteri)(TEXTURE_2D, TEXTURE_WRAP_S, CLAMP_TO_EDGE);
            (gl.TexParameteri)(TEXTURE_2D, TEXTURE_WRAP_T, CLAMP_TO_EDGE);
            // hud buffers
            (gl.GenVertexArrays)(1, &mut hud_vao);
            (gl.BindVertexArray)(hud_vao);
            (gl.GenBuffers)(1, &mut hud_vbo);
            (gl.BindBuffer)(ARRAY_BUFFER, hud_vbo);
            (gl.EnableVertexAttribArray)(0);
            (gl.VertexAttribPointer)(0, 2, FLOAT, FALSE, 16, std::ptr::null());
            (gl.EnableVertexAttribArray)(1);
            (gl.VertexAttribPointer)(1, 2, FLOAT, FALSE, 16, 8 as *const c_void);
            (gl.BindVertexArray)(0);
            (gl.GenVertexArrays)(1, &mut empty_vao);
        }
        let car_body = upload(&gl, car_body);
        let car_wheel = upload(&gl, car_wheel);
        Ok(Renderer {
            gl, p_main, p_shadow, p_sky, p_hud, statics: Vec::new(), tiles: HashMap::new(), follow_ground: None, car_dyn: None, car_body, car_wheel, shadow_tex, shadow_fbo, font_tex, hud_vao, hud_vbo, empty_vao,
            target_fbo: 0, width: 1280, height: 720, info,
        })
    }

    /// Replace the static scene geometry.
    pub fn set_scene(&mut self, meshes: &[MeshData]) {
        unsafe {
            for m in &self.statics {
                (self.gl.DeleteVertexArrays)(1, &m.vao);
                (self.gl.DeleteBuffers)(1, &m.vbo);
            }
        }
        self.statics = meshes.iter().filter(|m| !m.verts.is_empty()).map(|m| upload(&self.gl, m)).collect();
    }

    fn free(&self, m: &GpuMesh) {
        unsafe {
            (self.gl.DeleteVertexArrays)(1, &m.vao);
            (self.gl.DeleteBuffers)(1, &m.vbo);
        }
    }

    /// One big ground plane that follows the camera (streamed tiles carry no ground of their own).
    pub fn set_follow_ground(&mut self, m: Option<&MeshData>) {
        if let Some(old) = self.follow_ground.take() {
            self.free(&old);
        }
        self.follow_ground = m.map(|m| upload(&self.gl, m));
    }

    /// Replace the car body by a deformed copy (world-space vertices, same layout); `None` goes back to the rigid mesh.
    pub fn set_deformed_body(&mut self, verts: Option<&[f32]>) {
        let Some(v) = verts else {
            if let Some(m) = self.car_dyn.take() {
                self.free(&m);
            }
            return;
        };
        if self.car_dyn.is_none() {
            self.car_dyn = Some(upload(&self.gl, &MeshData { name: "car_body_deformed".into(), verts: Vec::new(), polygon_offset: 0.0, casts_shadow: true, colors: Vec::new() }));
        }
        let m = self.car_dyn.as_mut().unwrap();
        unsafe {
            (self.gl.BindBuffer)(ARRAY_BUFFER, m.vbo);
            (self.gl.BufferData)(ARRAY_BUFFER, (v.len() * 4) as GLsizeiptr, v.as_ptr() as *const c_void, DYNAMIC_DRAW);
        }
        m.count = (v.len() / FLOATS_PER_VERTEX) as i32;
    }

    /// Add (or replace) the meshes of a streamed tile.
    pub fn add_tile(&mut self, key: u64, meshes: &[MeshData]) {
        self.remove_tile(key);
        let g: Vec<GpuMesh> = meshes.iter().filter(|m| !m.verts.is_empty()).map(|m| upload(&self.gl, m)).collect();
        self.tiles.insert(key, g);
    }

    pub fn remove_tile(&mut self, key: u64) {
        if let Some(v) = self.tiles.remove(&key) {
            for m in &v {
                self.free(m);
            }
        }
    }

    pub fn tile_count(&self) -> usize {
        self.tiles.len()
    }

    /// Apply the changes the game queued since the last frame.
    pub fn apply_ops(&mut self, ops: Vec<RenderOp>) {
        for op in ops {
            match op {
                RenderOp::AddTile(k, m) => self.add_tile(k, &m),
                RenderOp::RemoveTile(k) => self.remove_tile(k),
                RenderOp::FollowGround(on) => {
                    let g = on.then(|| scene::mesh::ground_mesh(7000.0));
                    self.set_follow_ground(g.as_ref());
                }
            }
        }
    }

    pub fn mesh_names(&self) -> Vec<&str> {
        self.statics.iter().map(|m| m.name.as_str()).collect()
    }

    /// Render into `fbo` (0 = window back buffer) of the given size.
    pub fn set_target(&mut self, fbo: GLuint, w: i32, h: i32) {
        self.target_fbo = fbo;
        self.width = w.max(1);
        self.height = h.max(1);
    }

    pub fn check_error(&self) -> Option<u32> {
        let e = unsafe { (self.gl.GetError)() };
        (e != 0).then_some(e)
    }

    /// RGBA8 pixels of the current target, bottom-up as OpenGL returns them.
    pub fn read_pixels(&self) -> Vec<u8> {
        let mut buf = vec![0u8; (self.width * self.height * 4) as usize];
        unsafe {
            (self.gl.BindFramebuffer)(FRAMEBUFFER, self.target_fbo);
            (self.gl.PixelStorei)(PACK_ALIGNMENT, 1);
            (self.gl.ReadPixels)(0, 0, self.width, self.height, RGBA, UNSIGNED_BYTE, buf.as_mut_ptr() as *mut c_void);
        }
        buf
    }

    fn loc(&self, p: &Prog, name: &str) -> GLint {
        if let Some(&l) = p.locs.borrow().get(name) {
            return l;
        }
        let c = CString::new(name).unwrap();
        let l = unsafe { (self.gl.GetUniformLocation)(p.id, c.as_ptr()) };
        p.locs.borrow_mut().insert(name.to_string(), l);
        l
    }
    fn u1(&self, p: &Prog, n: &str, v: f32) {
        unsafe { (self.gl.Uniform1f)(self.loc(p, n), v) }
    }
    fn u1i(&self, p: &Prog, n: &str, v: i32) {
        unsafe { (self.gl.Uniform1i)(self.loc(p, n), v) }
    }
    fn u3(&self, p: &Prog, n: &str, v: V3f) {
        unsafe { (self.gl.Uniform3f)(self.loc(p, n), v[0], v[1], v[2]) }
    }
    fn um(&self, p: &Prog, n: &str, m: &M4) {
        unsafe { (self.gl.UniformMatrix4fv)(self.loc(p, n), 1, FALSE, m.as_ptr()) }
    }

    fn draw_mesh(&self, m: &GpuMesh) {
        unsafe {
            (self.gl.BindVertexArray)(m.vao);
            (self.gl.DrawArrays)(TRIANGLES, 0, m.count);
        }
    }

    pub fn render(&mut self, f: &FrameParams, hud: &HudData) -> DrawStats {
        let gl = &self.gl;
        let lg: Lighting = lighting(f.hour);
        let (w, h) = (self.width, self.height);
        let aspect = w as f32 / h as f32;
        let mut stats = DrawStats::default();
        let fov = f.fov_deg.to_radians();
        let proj = m4::perspective(fov, aspect, 0.5, 6000.0);
        let view = m4::look_at(f.eye, f.target, [0.0, 0.0, 1.0]);
        let vp = m4::mul(&proj, &view);
        let fwd = m4::norm(m4::sub(f.target, f.eye));
        let right = m4::norm(m4::cross(fwd, [0.0, 0.0, 1.0]));
        let upv = m4::cross(right, fwd);
        let ident = m4::id();

        // ---- shadow pass (sun only, orthographic box following the car, snapped to texels) ----
        let shadow_on = lg.light_dir[2] > 0.12;
        let mut light_vp = ident;
        if shadow_on {
            let lf = m4::scale(lg.light_dir, -1.0);
            let s = m4::norm(m4::cross(lf, [0.0, 0.0, 1.0]));
            let u = m4::cross(s, lf);
            let half = 110.0f32;
            let tx = 2.0 * half / SHADOW_SIZE as f32;
            let cp = [f.car_pos[0], f.car_pos[1], 0.0];
            let (cx, cy, cz) = (m4::dot(s, cp), m4::dot(u, cp), m4::dot(lf, cp));
            let (rx, ry) = ((cx / tx).round() * tx, (cy / tx).round() * tx);
            let c = m4::add(m4::add(m4::scale(s, rx), m4::scale(u, ry)), m4::scale(lf, cz));
            let eye_l = m4::sub(c, m4::scale(lf, 500.0));
            light_vp = m4::mul(&m4::ortho(-half, half, -half, half, 1.0, 1100.0), &m4::look_at(eye_l, c, [0.0, 0.0, 1.0]));
            unsafe {
                (gl.BindFramebuffer)(FRAMEBUFFER, self.shadow_fbo);
                (gl.Viewport)(0, 0, SHADOW_SIZE, SHADOW_SIZE);
                (gl.Clear)(DEPTH_BUFFER_BIT);
                (gl.Enable)(DEPTH_TEST);
                (gl.DepthFunc)(LEQUAL);
                (gl.DepthMask)(TRUE);
                (gl.Disable)(CULL_FACE);
                (gl.UseProgram)(self.p_shadow.id);
                self.um(&self.p_shadow, "uLightVP", &light_vp);
                (gl.Enable)(POLYGON_OFFSET_FILL);
                (gl.PolygonOffset)(2.0, 4.0);
                self.um(&self.p_shadow, "uModel", &ident);
                for m in self.statics.iter().chain(self.tiles.values().flatten()).filter(|m| m.casts_shadow) {
                    self.draw_mesh(m);
                }
                if f.draw_car {
                    if let Some(d) = &self.car_dyn {
                        self.um(&self.p_shadow, "uModel", &ident);
                        self.draw_mesh(d);
                    } else {
                        self.um(&self.p_shadow, "uModel", &f.body);
                        self.draw_mesh(&self.car_body);
                    }
                    for wm in &f.wheels {
                        self.um(&self.p_shadow, "uModel", wm);
                        self.draw_mesh(&self.car_wheel);
                    }
                }
                (gl.Disable)(POLYGON_OFFSET_FILL);
            }
        }

        // ---- main pass ----
        unsafe {
            (gl.BindFramebuffer)(FRAMEBUFFER, self.target_fbo);
            (gl.Viewport)(0, 0, w, h);
            (gl.DepthMask)(TRUE);
            (gl.ClearColor)(0.0, 0.0, 0.0, 1.0);
            (gl.Clear)(COLOR_BUFFER_BIT | DEPTH_BUFFER_BIT);
            (gl.Disable)(CULL_FACE);
            (gl.Disable)(BLEND);
            // sky
            (gl.BindVertexArray)(self.empty_vao);
            (gl.DepthMask)(FALSE);
            (gl.Disable)(DEPTH_TEST);
            (gl.UseProgram)(self.p_sky.id);
        }
        let pk = &self.p_sky;
        self.u1(pk, "uExposure", 1.0);
        self.u3(pk, "uFwd", fwd);
        self.u3(pk, "uRight", right);
        self.u3(pk, "uUp", upv);
        self.u3(pk, "uZenith", lg.zenith);
        self.u3(pk, "uHorizon", lg.horizon);
        self.u3(pk, "uSunDirS", lg.sun_dir);
        self.u3(pk, "uSunColS", lg.sun_col);
        let th = (fov / 2.0).tan();
        unsafe {
            (gl.Uniform2f)(self.loc(pk, "uTan"), th * aspect, th);
        }
        self.u1(pk, "uDay", lg.day);
        unsafe {
            (gl.DrawArrays)(TRIANGLES, 0, 3);
            (gl.Enable)(DEPTH_TEST);
            (gl.DepthMask)(TRUE);
            (gl.DepthFunc)(LEQUAL);
            (gl.UseProgram)(self.p_main.id);
        }
        stats.draws += 1;
        let pm = &self.p_main;
        self.um(pm, "uVP", &vp);
        self.um(pm, "uLightVP", &light_vp);
        self.u3(pm, "uCam", f.eye);
        self.u3(pm, "uLightDir", lg.light_dir);
        self.u3(pm, "uSunCol", lg.sun_col);
        self.u3(pm, "uSkyAmb", lg.sky_amb);
        self.u3(pm, "uGroundAmb", lg.ground_amb);
        self.u3(pm, "uHorizon", lg.horizon);
        self.u3(pm, "uZenith", lg.zenith);
        self.u3(pm, "uPaint", f.paint);
        self.u3(pm, "uHeadPos", f.head_pos);
        self.u3(pm, "uHeadDir", f.head_dir);
        self.u1(pm, "uHeadOn", if f.headlights { 1.0 } else { 0.0 });
        self.u1(pm, "uSunI", lg.sun_i);
        self.u1(pm, "uDay", lg.day);
        self.u1(pm, "uShadowOn", if shadow_on { 1.0 } else { 0.0 });
        self.u1(pm, "uFogD", 1.0 / (1400.0 + 2200.0 * lg.day));
        self.u1(pm, "uExposure", 1.0);
        self.u1(pm, "uBrake", if f.braking { 1.0 } else { 0.0 });
        unsafe {
            (gl.ActiveTexture)(TEXTURE0);
            (gl.BindTexture)(TEXTURE_2D, self.shadow_tex);
        }
        self.u1i(pm, "uShadow", 0);
        self.um(pm, "uModel", &ident);
        unsafe {
            (gl.Enable)(POLYGON_OFFSET_FILL);
        }
        if let Some(g) = &self.follow_ground {
            let snap = |v: f32| (v / 50.0).round() * 50.0;
            self.um(pm, "uModel", &m4::translate(snap(f.eye[0]), snap(f.eye[1]), 0.0));
            unsafe {
                (gl.PolygonOffset)(g.polygon_offset, g.polygon_offset);
            }
            self.draw_mesh(g);
            stats.draws += 1;
            stats.triangles += g.count as u64 / 3;
            self.um(pm, "uModel", &ident);
        }
        for m in self.statics.iter().chain(self.tiles.values().flatten()) {
            unsafe {
                (gl.PolygonOffset)(m.polygon_offset, m.polygon_offset);
            }
            self.draw_mesh(m);
            stats.draws += 1;
            stats.triangles += m.count as u64 / 3;
        }
        unsafe {
            (gl.Disable)(POLYGON_OFFSET_FILL);
        }
        if f.draw_car {
            let body = self.car_dyn.as_ref().unwrap_or(&self.car_body);
            self.um(pm, "uModel", if self.car_dyn.is_some() { &ident } else { &f.body });
            self.draw_mesh(body);
            stats.draws += 1;
            stats.triangles += body.count as u64 / 3;
            for wm in &f.wheels {
                self.um(pm, "uModel", wm);
                self.draw_mesh(&self.car_wheel);
                stats.draws += 1;
                stats.triangles += self.car_wheel.count as u64 / 3;
            }
        }
        unsafe {
            (gl.BindVertexArray)(0);
        }
        if hud.show {
            self.draw_hud(hud);
        }
        stats
    }

    // ---- HUD ----
    fn hud_quads(&self, verts: &[f32], color: [f32; 4], solid: bool) {
        if verts.is_empty() {
            return;
        }
        let gl = &self.gl;
        let p = &self.p_hud;
        unsafe {
            (gl.UseProgram)(p.id);
            (gl.Uniform4f)(self.loc(p, "uColor"), color[0], color[1], color[2], color[3]);
            (gl.Uniform1f)(self.loc(p, "uSolid"), if solid { 1.0 } else { 0.0 });
            (gl.BindVertexArray)(self.hud_vao);
            (gl.BindBuffer)(ARRAY_BUFFER, self.hud_vbo);
            (gl.BufferData)(ARRAY_BUFFER, (verts.len() * 4) as GLsizeiptr, verts.as_ptr() as *const c_void, DYNAMIC_DRAW);
            (gl.DrawArrays)(TRIANGLES, 0, (verts.len() / 4) as GLsizei);
        }
    }

    fn rect(&self, x: f32, y: f32, w: f32, h: f32, color: [f32; 4]) {
        let v = [x, y, 0.0, 0.0, x + w, y, 0.0, 0.0, x + w, y + h, 0.0, 0.0, x, y, 0.0, 0.0, x + w, y + h, 0.0, 0.0, x, y + h, 0.0, 0.0];
        self.hud_quads(&v, color, true);
    }

    pub fn text_width(s: &str, scale: f32) -> f32 {
        s.chars().count() as f32 * font::CELL_W as f32 * scale
    }

    fn text(&self, x: f32, y: f32, scale: f32, color: [f32; 4], s: &str) {
        let mut v: Vec<f32> = Vec::with_capacity(s.len() * 24);
        let aw = (font::COUNT * font::CELL_W) as f32;
        let (cw, ch) = (font::CELL_W as f32 * scale, font::CELL_H as f32 * scale);
        for (i, c) in s.chars().enumerate() {
            let g = font::index(c);
            let (u0, u1) = ((g * font::CELL_W) as f32 / aw, ((g + 1) * font::CELL_W) as f32 / aw);
            let (x0, x1, y0, y1) = (x + i as f32 * cw, x + (i + 1) as f32 * cw, y, y + ch);
            v.extend_from_slice(&[x0, y0, u0, 0.0, x1, y0, u1, 0.0, x1, y1, u1, 1.0, x0, y0, u0, 0.0, x1, y1, u1, 1.0, x0, y1, u0, 1.0]);
        }
        self.hud_quads(&v, color, false);
    }

    fn draw_hud(&self, hud: &HudData) {
        let gl = &self.gl;
        let (w, h) = (self.width as f32, self.height as f32);
        unsafe {
            (gl.Disable)(DEPTH_TEST);
            (gl.DepthMask)(FALSE);
            (gl.Enable)(BLEND);
            (gl.BlendFunc)(SRC_ALPHA, ONE_MINUS_SRC_ALPHA);
            (gl.UseProgram)(self.p_hud.id);
            (gl.Uniform2f)(self.loc(&self.p_hud, "uScreen"), w, h);
            (gl.ActiveTexture)(TEXTURE0);
            (gl.BindTexture)(TEXTURE_2D, self.font_tex);
            (gl.Uniform1i)(self.loc(&self.p_hud, "uTex"), 0);
        }
        let k = (h / 720.0).max(1.0).round().max(1.0);
        let sc = 2.0 * k;
        let white = [1.0, 1.0, 1.0, 1.0];
        let shadow = [0.0, 0.0, 0.0, 0.7];
        let shadow_text = |x: f32, y: f32, s: f32, c: [f32; 4], t: &str| {
            self.text(x + s * 0.5, y + s * 0.5, s, shadow, t);
            self.text(x, y, s, c, t);
        };
        // top-left info (below the banner)
        let line_h = 8.0 * sc * 1.25;
        let top = 8.0 * sc + 24.0 * k;
        for (i, l) in hud.lines.iter().enumerate() {
            shadow_text(12.0 * k, top + i as f32 * line_h, sc, white, l);
        }
        // banner
        if !hud.warn.is_empty() {
            let tw = Self::text_width(&hud.warn, sc);
            let (bx, by) = ((w - tw) / 2.0 - 8.0 * k, 8.0 * k);
            self.rect(bx, by, tw + 16.0 * k, 8.0 * sc + 8.0 * k, if hud.warn_is_demo { [0.65, 0.2, 0.2, 0.75] } else { [0.15, 0.3, 0.55, 0.75] });
            self.text(bx + 8.0 * k, by + 4.0 * k, sc, white, &hud.warn);
        }
        // speed
        let big = 7.0 * k;
        let sp = format!("{}", hud.speed_kmh.round() as i32);
        let sx = w - 24.0 * k - Self::text_width(&sp, big) - Self::text_width(" KM/H", sc);
        shadow_text(sx, h - 24.0 * k - 8.0 * big, big, white, &sp);
        shadow_text(sx + Self::text_width(&sp, big), h - 24.0 * k - 8.0 * sc, sc, white, " KM/H");
        // gear + rpm bar
        let gear = match hud.gear {
            g if g < 0 => "R".to_string(),
            0 => "N".to_string(),
            g => g.to_string(),
        };
        let bar_w = 260.0 * k;
        let bx = w - 24.0 * k - bar_w;
        let by = h - 24.0 * k - 8.0 * big - 18.0 * k;
        self.rect(bx, by, bar_w, 10.0 * k, [0.0, 0.0, 0.0, 0.5]);
        let frac = (hud.rpm / hud.rpm_max.max(1.0)).clamp(0.0, 1.0);
        let col = if frac > 0.88 { [0.95, 0.25, 0.2, 0.95] } else { [0.95, 0.85, 0.3, 0.95] };
        self.rect(bx, by, bar_w * frac, 10.0 * k, col);
        shadow_text(bx, by - 8.0 * sc - 4.0 * k, sc, white, &format!("GEAR {}   {} RPM", gear, hud.rpm.round() as i32));
        // help
        if !hud.help.is_empty() {
            let hs = 1.5 * k;
            let lines: Vec<&str> = hud.help.lines().collect();
            for (i, l) in lines.iter().enumerate() {
                let y = h - 10.0 * k - 8.0 * hs * 1.3 * (lines.len() - i) as f32;
                shadow_text(12.0 * k, y, hs, [1.0, 1.0, 1.0, 0.85], l);
            }
        }
        unsafe {
            (gl.Disable)(BLEND);
            (gl.Enable)(DEPTH_TEST);
            (gl.DepthMask)(TRUE);
            (gl.BindVertexArray)(0);
        }
    }
}
