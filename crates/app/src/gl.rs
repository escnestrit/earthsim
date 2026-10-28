//! Minimal OpenGL 3.3 core bindings loaded at run time through a caller-supplied loader
//! (wglGetProcAddress + opengl32.dll on Windows, eglGetProcAddress in the headless test on Linux).
#![allow(non_snake_case, non_upper_case_globals, dead_code)]
use std::ffi::c_void;
use std::os::raw::c_char;

pub type GLenum = u32;
pub type GLuint = u32;
pub type GLint = i32;
pub type GLsizei = i32;
pub type GLfloat = f32;
pub type GLbitfield = u32;
pub type GLboolean = u8;
pub type GLsizeiptr = isize;
pub type GLchar = c_char;

pub const FALSE: GLboolean = 0;
pub const TRUE: GLboolean = 1;
pub const COLOR_BUFFER_BIT: GLbitfield = 0x4000;
pub const DEPTH_BUFFER_BIT: GLbitfield = 0x100;
pub const TRIANGLES: GLenum = 4;
pub const FLOAT: GLenum = 0x1406;
pub const UNSIGNED_BYTE: GLenum = 0x1401;
pub const UNSIGNED_INT: GLenum = 0x1405;
pub const ARRAY_BUFFER: GLenum = 0x8892;
pub const STATIC_DRAW: GLenum = 0x88E4;
pub const DYNAMIC_DRAW: GLenum = 0x88E8;
pub const VERTEX_SHADER: GLenum = 0x8B31;
pub const FRAGMENT_SHADER: GLenum = 0x8B30;
pub const COMPILE_STATUS: GLenum = 0x8B81;
pub const LINK_STATUS: GLenum = 0x8B82;
pub const INFO_LOG_LENGTH: GLenum = 0x8B84;
pub const DEPTH_TEST: GLenum = 0x0B71;
pub const CULL_FACE: GLenum = 0x0B44;
pub const BLEND: GLenum = 0x0BE2;
pub const POLYGON_OFFSET_FILL: GLenum = 0x8037;
pub const LEQUAL: GLenum = 0x0203;
pub const SRC_ALPHA: GLenum = 0x0302;
pub const ONE_MINUS_SRC_ALPHA: GLenum = 0x0303;
pub const TEXTURE_2D: GLenum = 0x0DE1;
pub const TEXTURE0: GLenum = 0x84C0;
pub const TEXTURE_MIN_FILTER: GLenum = 0x2801;
pub const TEXTURE_MAG_FILTER: GLenum = 0x2800;
pub const TEXTURE_WRAP_S: GLenum = 0x2802;
pub const TEXTURE_WRAP_T: GLenum = 0x2803;
pub const TEXTURE_COMPARE_MODE: GLenum = 0x884C;
pub const TEXTURE_COMPARE_FUNC: GLenum = 0x884D;
pub const COMPARE_REF_TO_TEXTURE: GLenum = 0x884E;
pub const NEAREST: GLint = 0x2600;
pub const LINEAR: GLint = 0x2601;
pub const CLAMP_TO_EDGE: GLint = 0x812F;
pub const DEPTH_COMPONENT: GLenum = 0x1902;
pub const DEPTH_COMPONENT24: GLint = 0x81A6;
pub const RED: GLenum = 0x1903;
pub const R8: GLint = 0x8229;
pub const RGBA: GLenum = 0x1908;
pub const RGBA8: GLint = 0x8058;
pub const FRAMEBUFFER: GLenum = 0x8D40;
pub const DEPTH_ATTACHMENT: GLenum = 0x8D00;
pub const COLOR_ATTACHMENT0: GLenum = 0x8CE0;
pub const FRAMEBUFFER_COMPLETE: GLenum = 0x8CD5;
pub const NONE: GLenum = 0;
pub const UNPACK_ALIGNMENT: GLenum = 0x0CF5;
pub const PACK_ALIGNMENT: GLenum = 0x0D05;
pub const VERSION: GLenum = 0x1F02;
pub const RENDERER: GLenum = 0x1F01;
pub const RENDERBUFFER: GLenum = 0x8D41;

macro_rules! gl_funcs {
    ($( $name:ident ( $($an:ident : $at:ty),* ) $(-> $ret:ty)? = $sym:literal ;)*) => {
        pub struct Gl { $( pub $name: unsafe extern "system" fn($($at),*) $(-> $ret)? ),* }
        impl Gl {
            /// `get` returns the address of a GL entry point or null.
            pub fn load(get: &dyn Fn(&str) -> *const c_void) -> Result<Gl, String> {
                Ok(Gl { $( $name: {
                    let p = get($sym);
                    if p.is_null() { return Err(format!("OpenGL function {} is not available (need OpenGL 3.3)", $sym)); }
                    unsafe { std::mem::transmute::<*const c_void, unsafe extern "system" fn($($at),*) $(-> $ret)?>(p) }
                } ),* })
            }
        }
    };
}

gl_funcs! {
    GetError() -> GLenum = "glGetError";
    GetString(name: GLenum) -> *const u8 = "glGetString";
    Viewport(x: GLint, y: GLint, w: GLsizei, h: GLsizei) = "glViewport";
    Clear(mask: GLbitfield) = "glClear";
    ClearColor(r: GLfloat, g: GLfloat, b: GLfloat, a: GLfloat) = "glClearColor";
    Enable(cap: GLenum) = "glEnable";
    Disable(cap: GLenum) = "glDisable";
    DepthFunc(f: GLenum) = "glDepthFunc";
    DepthMask(m: GLboolean) = "glDepthMask";
    BlendFunc(s: GLenum, d: GLenum) = "glBlendFunc";
    PolygonOffset(f: GLfloat, u: GLfloat) = "glPolygonOffset";
    PixelStorei(p: GLenum, v: GLint) = "glPixelStorei";
    ReadPixels(x: GLint, y: GLint, w: GLsizei, h: GLsizei, f: GLenum, t: GLenum, d: *mut c_void) = "glReadPixels";
    DrawBuffer(m: GLenum) = "glDrawBuffer";
    ReadBuffer(m: GLenum) = "glReadBuffer";
    GenTextures(n: GLsizei, t: *mut GLuint) = "glGenTextures";
    BindTexture(target: GLenum, t: GLuint) = "glBindTexture";
    TexImage2D(target: GLenum, level: GLint, internal: GLint, w: GLsizei, h: GLsizei, border: GLint, f: GLenum, t: GLenum, d: *const c_void) = "glTexImage2D";
    TexParameteri(target: GLenum, name: GLenum, v: GLint) = "glTexParameteri";
    ActiveTexture(t: GLenum) = "glActiveTexture";
    CreateShader(t: GLenum) -> GLuint = "glCreateShader";
    ShaderSource(s: GLuint, n: GLsizei, src: *const *const GLchar, len: *const GLint) = "glShaderSource";
    CompileShader(s: GLuint) = "glCompileShader";
    GetShaderiv(s: GLuint, p: GLenum, v: *mut GLint) = "glGetShaderiv";
    GetShaderInfoLog(s: GLuint, max: GLsizei, len: *mut GLsizei, log: *mut GLchar) = "glGetShaderInfoLog";
    DeleteShader(s: GLuint) = "glDeleteShader";
    CreateProgram() -> GLuint = "glCreateProgram";
    AttachShader(p: GLuint, s: GLuint) = "glAttachShader";
    LinkProgram(p: GLuint) = "glLinkProgram";
    GetProgramiv(p: GLuint, n: GLenum, v: *mut GLint) = "glGetProgramiv";
    GetProgramInfoLog(p: GLuint, max: GLsizei, len: *mut GLsizei, log: *mut GLchar) = "glGetProgramInfoLog";
    UseProgram(p: GLuint) = "glUseProgram";
    GetUniformLocation(p: GLuint, name: *const GLchar) -> GLint = "glGetUniformLocation";
    Uniform1f(l: GLint, a: GLfloat) = "glUniform1f";
    Uniform1i(l: GLint, a: GLint) = "glUniform1i";
    Uniform2f(l: GLint, a: GLfloat, b: GLfloat) = "glUniform2f";
    Uniform3f(l: GLint, a: GLfloat, b: GLfloat, c: GLfloat) = "glUniform3f";
    Uniform4f(l: GLint, a: GLfloat, b: GLfloat, c: GLfloat, d: GLfloat) = "glUniform4f";
    UniformMatrix4fv(l: GLint, n: GLsizei, transpose: GLboolean, v: *const GLfloat) = "glUniformMatrix4fv";
    GenVertexArrays(n: GLsizei, a: *mut GLuint) = "glGenVertexArrays";
    BindVertexArray(a: GLuint) = "glBindVertexArray";
    DeleteVertexArrays(n: GLsizei, a: *const GLuint) = "glDeleteVertexArrays";
    GenBuffers(n: GLsizei, b: *mut GLuint) = "glGenBuffers";
    BindBuffer(target: GLenum, b: GLuint) = "glBindBuffer";
    BufferData(target: GLenum, size: GLsizeiptr, data: *const c_void, usage: GLenum) = "glBufferData";
    DeleteBuffers(n: GLsizei, b: *const GLuint) = "glDeleteBuffers";
    EnableVertexAttribArray(i: GLuint) = "glEnableVertexAttribArray";
    VertexAttribPointer(i: GLuint, size: GLint, t: GLenum, norm: GLboolean, stride: GLsizei, ptr: *const c_void) = "glVertexAttribPointer";
    DrawArrays(mode: GLenum, first: GLint, count: GLsizei) = "glDrawArrays";
    GenFramebuffers(n: GLsizei, f: *mut GLuint) = "glGenFramebuffers";
    BindFramebuffer(target: GLenum, f: GLuint) = "glBindFramebuffer";
    FramebufferTexture2D(target: GLenum, att: GLenum, textarget: GLenum, tex: GLuint, level: GLint) = "glFramebufferTexture2D";
    CheckFramebufferStatus(target: GLenum) -> GLenum = "glCheckFramebufferStatus";
    GenRenderbuffers(n: GLsizei, r: *mut GLuint) = "glGenRenderbuffers";
    BindRenderbuffer(target: GLenum, r: GLuint) = "glBindRenderbuffer";
    RenderbufferStorage(target: GLenum, internal: GLenum, w: GLsizei, h: GLsizei) = "glRenderbufferStorage";
    FramebufferRenderbuffer(target: GLenum, att: GLenum, rbtarget: GLenum, rb: GLuint) = "glFramebufferRenderbuffer";
}
