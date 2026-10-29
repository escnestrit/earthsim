//! Getting OpenStreetMap data: a `Fetcher` trait, an offline "virtual Overpass" (serves a fixed dataset by tile,
//! used for tests and the offline demo) and, on Windows, a WinHTTP client.
//!
//! Etiquette: the public Overpass servers are a shared resource with a fair-use policy. The streamer sends at most
//! one request at a time with a minimum gap, identifies itself, backs off on errors and caches every response on
//! disk. For heavy use point `EARTHSIM_OVERPASS` at your own Overpass instance.
use crate::tiles::{Bounds, TileId};
use scene::json::Json;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

#[derive(Debug, Clone, PartialEq)]
pub enum FetchError {
    /// Worth retrying later (timeouts, 429, 5xx, no network).
    Transient(String),
    /// Retrying will not help soon (400, 403, 404...).
    Permanent(String),
}

impl std::fmt::Display for FetchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FetchError::Transient(m) => write!(f, "transient: {m}"),
            FetchError::Permanent(m) => write!(f, "permanent: {m}"),
        }
    }
}

pub trait Fetcher: Send + Sync {
    /// `query` is the Overpass QL text for the tile.
    fn fetch(&self, tile: TileId, query: &str) -> Result<String, FetchError>;
    fn describe(&self) -> String;
}

pub const USER_AGENT: &str = "earthsim/0.1 (BeamNG.Earth prototype; research/driving sim; respects Overpass fair use)";
pub const DEFAULT_ENDPOINTS: &[&str] = &[
    "https://overpass-api.de/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
    "https://overpass.openstreetmap.fr/api/interpreter",
    "https://overpass.osm.ch/api/interpreter",
    "https://z.overpass-api.de/api/interpreter",
];

pub fn percent_encode(s: &str) -> String {
    let mut o = String::with_capacity(s.len() * 3);
    for b in s.bytes() {
        if b.is_ascii_alphanumeric() || matches!(b, b'-' | b'_' | b'.' | b'~') {
            o.push(b as char);
        } else {
            o.push_str(&format!("%{b:02X}"));
        }
    }
    o
}

/// (host, port, path+query, tls) of an http(s) URL.
pub fn split_url(url: &str) -> Option<(String, u16, String, bool)> {
    let (tls, rest) = if let Some(r) = url.strip_prefix("https://") {
        (true, r)
    } else if let Some(r) = url.strip_prefix("http://") {
        (false, r)
    } else {
        return None;
    };
    let (hostport, path) = match rest.find('/') {
        Some(i) => (&rest[..i], &rest[i..]),
        None => (rest, "/"),
    };
    let (host, port) = match hostport.rsplit_once(':') {
        Some((h, p)) => (h, p.parse::<u16>().ok()?),
        None => (hostport, if tls { 443 } else { 80 }),
    };
    if host.is_empty() {
        return None;
    }
    Some((host.to_string(), port, path.to_string(), tls))
}

pub fn status_error(code: u32) -> FetchError {
    match code {
        429 | 500 | 502 | 503 | 504 | 408 => FetchError::Transient(format!("HTTP {code}")),
        c => FetchError::Permanent(format!("HTTP {c}")),
    }
}

// ------------------------------------------------------------------------------------------------------
/// An offline stand-in for Overpass that serves a fixed dataset by tile. Elements outside the requested tile are
/// omitted; ways are returned if their bounding box touches the tile (like the real server).
pub struct VirtualOsm {
    elements: Vec<(Json, (f64, f64, f64, f64))>, // element, (south, west, north, east)
    pub requests: Arc<AtomicUsize>,
    /// Number of initial requests that fail transiently (for tests).
    fail_first: AtomicUsize,
    permanent: bool,
    delay_ms: u64,
}

impl VirtualOsm {
    pub fn from_overpass_json(text: &str) -> Result<VirtualOsm, String> {
        let j = Json::parse(text).map_err(|e| e.to_string())?;
        let mut elements = Vec::new();
        for e in j.get("elements").and_then(|e| e.as_arr()).ok_or("no elements")? {
            let (mut s, mut w, mut n, mut ea) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
            let mut add = |la: f64, lo: f64| {
                s = s.min(la);
                n = n.max(la);
                w = w.min(lo);
                ea = ea.max(lo);
            };
            if let (Some(la), Some(lo)) = (e.get("lat").and_then(|v| v.as_f64()), e.get("lon").and_then(|v| v.as_f64())) {
                add(la, lo);
            }
            for p in e.get("geometry").and_then(|g| g.as_arr()).unwrap_or(&[]) {
                if let (Some(la), Some(lo)) = (p.get("lat").and_then(|v| v.as_f64()), p.get("lon").and_then(|v| v.as_f64())) {
                    add(la, lo);
                }
            }
            if s <= n {
                elements.push((e.clone(), (s, w, n, ea)));
            }
        }
        Ok(VirtualOsm { elements, requests: Arc::new(AtomicUsize::new(0)), fail_first: AtomicUsize::new(0), permanent: false, delay_ms: 0 })
    }
    pub fn failing_first(mut self, n: usize) -> Self {
        self.fail_first = AtomicUsize::new(n);
        self
    }
    pub fn always_permanent_error(mut self) -> Self {
        self.permanent = true;
        self
    }
    pub fn with_delay_ms(mut self, ms: u64) -> Self {
        self.delay_ms = ms;
        self
    }
    pub fn request_count(&self) -> usize {
        self.requests.load(Ordering::SeqCst)
    }
    fn select(&self, b: &Bounds) -> String {
        let els: Vec<Json> = self.elements.iter().filter(|(_, bb)| b.intersects_box(bb.0, bb.1, bb.2, bb.3)).map(|(e, _)| e.clone()).collect();
        Json::Obj(vec![("version".into(), Json::Num(0.6)), ("elements".into(), Json::Arr(els))]).dump()
    }
}

impl Fetcher for VirtualOsm {
    fn fetch(&self, tile: TileId, _query: &str) -> Result<String, FetchError> {
        self.requests.fetch_add(1, Ordering::SeqCst);
        if self.delay_ms > 0 {
            std::thread::sleep(std::time::Duration::from_millis(self.delay_ms));
        }
        if self.permanent {
            return Err(FetchError::Permanent("HTTP 403".into()));
        }
        if self.fail_first.load(Ordering::SeqCst) > 0 {
            self.fail_first.fetch_sub(1, Ordering::SeqCst);
            return Err(FetchError::Transient("HTTP 504".into()));
        }
        Ok(self.select(&tile.bounds()))
    }
    fn describe(&self) -> String {
        format!("virtual OSM ({} elements, offline demo data)", self.elements.len())
    }
}

/// Tries each source in turn (e.g. Overture first, Overpass as a fallback); keeps every error for the log.
pub struct ChainFetcher(pub Vec<Arc<dyn Fetcher>>);

impl Fetcher for ChainFetcher {
    fn fetch(&self, tile: TileId, query: &str) -> Result<String, FetchError> {
        let mut errs = Vec::new();
        for f in &self.0 {
            match f.fetch(tile, query) {
                Ok(t) => return Ok(t),
                Err(e) => {
                    crate::log::log(&format!("  source {} failed: {e}", f.describe()));
                    errs.push(e.to_string());
                }
            }
        }
        Err(FetchError::Transient(errs.join(" || ")))
    }
    fn describe(&self) -> String {
        self.0.iter().map(|f| f.describe()).collect::<Vec<_>>().join(" -> ")
    }
}

// ------------------------------------------------------------------------------------------------------
#[cfg(any(windows, earthsim_check_win))]
pub use winhttp::{default_live_fetcher, http_get, live_dem, live_map_source, overture_release, HttpRange, WinHttpFetcher};

#[cfg(any(windows, earthsim_check_win))]
mod winhttp {
    //! WinHTTP client (winhttp.dll is part of Windows; system proxy settings and certificates are honoured).
    //! Written without being able to run it on Windows in the dev sandbox; type-checked only.
    use super::*;
    use std::ffi::c_void;
    use std::sync::atomic::AtomicUsize;

    type HINTERNET = *mut c_void;
    type BOOL = i32;

    #[link(name = "winhttp")]
    extern "system" {
        fn WinHttpOpen(agent: *const u16, access: u32, proxy: *const u16, bypass: *const u16, flags: u32) -> HINTERNET;
        fn WinHttpSetTimeouts(h: HINTERNET, resolve: i32, connect: i32, send: i32, receive: i32) -> BOOL;
        fn WinHttpConnect(session: HINTERNET, host: *const u16, port: u16, reserved: u32) -> HINTERNET;
        fn WinHttpOpenRequest(conn: HINTERNET, verb: *const u16, path: *const u16, version: *const u16, referrer: *const u16, accept: *const *const u16, flags: u32) -> HINTERNET;
        fn WinHttpSendRequest(req: HINTERNET, headers: *const u16, headers_len: u32, optional: *const c_void, optional_len: u32, total_len: u32, context: usize) -> BOOL;
        fn WinHttpReceiveResponse(req: HINTERNET, reserved: *mut c_void) -> BOOL;
        fn WinHttpQueryHeaders(req: HINTERNET, level: u32, name: *const u16, buffer: *mut c_void, len: *mut u32, index: *mut u32) -> BOOL;
        fn WinHttpQueryDataAvailable(req: HINTERNET, available: *mut u32) -> BOOL;
        fn WinHttpReadData(req: HINTERNET, buffer: *mut c_void, to_read: u32, read: *mut u32) -> BOOL;
        fn WinHttpCloseHandle(h: HINTERNET) -> BOOL;
        fn WinHttpSetOption(h: HINTERNET, option: u32, buffer: *const c_void, len: u32) -> BOOL;
    }
    #[link(name = "kernel32")]
    extern "system" {
        fn GetLastError() -> u32;
    }

    const WINHTTP_ACCESS_TYPE_DEFAULT_PROXY: u32 = 0;
    const WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY: u32 = 4;
    const WINHTTP_OPTION_SECURE_PROTOCOLS: u32 = 84;
    const WINHTTP_FLAG_SECURE: u32 = 0x0080_0000;
    const WINHTTP_QUERY_STATUS_CODE: u32 = 19;
    const WINHTTP_QUERY_FLAG_NUMBER: u32 = 0x2000_0000;
    const MAX_RESPONSE_BYTES: usize = 64 * 1024 * 1024;

    /// Readable text for the WinHTTP / Win32 error codes that matter for a failed download.
    fn win_err(code: u32) -> String {
        let what = match code {
            12002 => "timed out",
            12007 => "server name could not be resolved (no internet / DNS)",
            12029 => "could not connect (firewall, proxy or no internet)",
            12030 => "connection was terminated",
            12038 => "certificate name does not match",
            12044 | 12045 => "certificate not trusted",
            12157 | 12175 => "TLS handshake failed",
            12152 => "invalid server response",
            12178 | 12180 => "automatic proxy detection failed",
            12017 | 12019 => "request was cancelled",
            _ => "",
        };
        if what.is_empty() { format!("error {code}") } else { format!("error {code}: {what}") }
    }

    fn wide(s: &str) -> Vec<u16> {
        s.encode_utf16().chain(std::iter::once(0)).collect()
    }

    struct Handle(HINTERNET);
    impl Drop for Handle {
        fn drop(&mut self) {
            if !self.0.is_null() {
                unsafe { WinHttpCloseHandle(self.0) };
            }
        }
    }

    pub struct WinHttpFetcher {
        endpoints: Vec<String>,
        current: AtomicUsize,
    }

    // HINTERNET handles are created per request, nothing shared is mutated except an atomic counter.
    unsafe impl Send for WinHttpFetcher {}
    unsafe impl Sync for WinHttpFetcher {}

    impl WinHttpFetcher {
        /// `EARTHSIM_OVERPASS` overrides the public endpoints.
        pub fn from_env() -> WinHttpFetcher {
            // `overpass.txt` next to the exe: one URL per line, tried before the built-in mirrors
            let mut user: Vec<String> = std::env::current_exe()
                .ok()
                .and_then(|p| std::fs::read_to_string(p.with_file_name("overpass.txt")).ok())
                .map(|t| t.lines().map(|l| l.trim().to_string()).filter(|l| l.starts_with("http")).collect())
                .unwrap_or_default();
            let endpoints = match std::env::var("EARTHSIM_OVERPASS") {
                Ok(u) if !u.trim().is_empty() => vec![u.trim().to_string()],
                _ => {
                    user.extend(DEFAULT_ENDPOINTS.iter().map(|s| s.to_string()));
                    user
                }
            };
            WinHttpFetcher { endpoints, current: AtomicUsize::new(0) }
        }

        fn get(&self, url: &str) -> Result<String, FetchError> {
            let body = http_get(url, None)?;
            String::from_utf8(body).map_err(|_| FetchError::Transient("response is not valid UTF-8".into()))
        }
    }

    /// GET `url`; with `range = Some((offset, len))` sends a Range header and accepts 206.
    pub fn http_get(url: &str, range: Option<(u64, u64)>) -> Result<Vec<u8>, FetchError> {
        {
            let (host, port, path, tls) = split_url(url).ok_or_else(|| FetchError::Permanent(format!("bad url {url}")))?;
            unsafe {
                // Automatic proxy (system / WPAD settings, Windows 8.1+); fall back to the plain default on older systems.
                let mut session = Handle(WinHttpOpen(wide(USER_AGENT).as_ptr(), WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, std::ptr::null(), std::ptr::null(), 0));
                if session.0.is_null() {
                    session = Handle(WinHttpOpen(wide(USER_AGENT).as_ptr(), WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, std::ptr::null(), std::ptr::null(), 0));
                }
                if session.0.is_null() {
                    return Err(FetchError::Transient(format!("WinHttpOpen failed ({})", win_err(GetLastError()))));
                }
                WinHttpSetTimeouts(session.0, 6_000, 6_000, 30_000, 90_000);
                // allow TLS 1.2 (and 1.3 where the system knows it); the result is ignored on systems that reject the flags
                let protocols: u32 = 0x0800 | 0x2000;
                WinHttpSetOption(session.0, WINHTTP_OPTION_SECURE_PROTOCOLS, &protocols as *const u32 as *const c_void, 4);
                let conn = Handle(WinHttpConnect(session.0, wide(&host).as_ptr(), port, 0));
                if conn.0.is_null() {
                    return Err(FetchError::Transient(format!("cannot connect to {host}: {}", win_err(GetLastError()))));
                }
                let req = Handle(WinHttpOpenRequest(conn.0, wide("GET").as_ptr(), wide(&path).as_ptr(), std::ptr::null(), std::ptr::null(), std::ptr::null(), if tls { WINHTTP_FLAG_SECURE } else { 0 }));
                if req.0.is_null() {
                    return Err(FetchError::Transient(format!("WinHttpOpenRequest failed ({})", win_err(GetLastError()))));
                }
                let hdr: Vec<u16> = match range {
                    Some((off, len)) => wide(&format!("Range: bytes={}-{}", off, off + len.max(1) - 1)),
                    None => wide(""),
                };
                let hdr_len = if range.is_some() { (hdr.len() - 1) as u32 } else { 0 };
                let hdr_ptr = if range.is_some() { hdr.as_ptr() } else { std::ptr::null() };
                if WinHttpSendRequest(req.0, hdr_ptr, hdr_len, std::ptr::null(), 0, 0, 0) == 0 {
                    return Err(FetchError::Transient(format!("send to {host} failed: {}", win_err(GetLastError()))));
                }
                if WinHttpReceiveResponse(req.0, std::ptr::null_mut()) == 0 {
                    return Err(FetchError::Transient(format!("no response from {host}: {}", win_err(GetLastError()))));
                }
                let mut code: u32 = 0;
                let mut len: u32 = 4;
                if WinHttpQueryHeaders(req.0, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, std::ptr::null(), &mut code as *mut u32 as *mut c_void, &mut len, std::ptr::null_mut()) == 0 {
                    return Err(FetchError::Transient(format!("no status ({})", win_err(GetLastError()))));
                }
                if code != 200 && !(code == 206 && range.is_some()) {
                    return Err(status_error(code));
                }
                let mut body: Vec<u8> = Vec::new();
                loop {
                    let mut avail: u32 = 0;
                    if WinHttpQueryDataAvailable(req.0, &mut avail) == 0 {
                        return Err(FetchError::Transient(format!("read failed ({})", win_err(GetLastError()))));
                    }
                    if avail == 0 {
                        break;
                    }
                    let start = body.len();
                    if start + avail as usize > MAX_RESPONSE_BYTES {
                        return Err(FetchError::Permanent("response too large".into()));
                    }
                    body.resize(start + avail as usize, 0);
                    let mut read: u32 = 0;
                    if WinHttpReadData(req.0, body[start..].as_mut_ptr() as *mut c_void, avail, &mut read) == 0 {
                        return Err(FetchError::Transient(format!("read failed ({})", win_err(GetLastError()))));
                    }
                    body.truncate(start + read as usize);
                }
                if let Some((_, len)) = range {
                    // a server that ignores Range answers 200 with the whole file: refuse rather than misread
                    if code == 200 && body.len() as u64 != len {
                        return Err(FetchError::Permanent("server ignored the Range header".into()));
                    }
                }
                Ok(body)
            }
        }
    }

    /// A remote file read with HTTP range requests (PMTiles archives).
    pub struct HttpRange(pub String);

    impl crate::pmtiles::RangeSource for HttpRange {
        fn read(&self, offset: u64, len: u64) -> Result<Vec<u8>, String> {
            let mut last = String::new();
            for _attempt in 0..3 {
                match http_get(&self.0, Some((offset, len))) {
                    Ok(b) => return Ok(b),
                    Err(e @ FetchError::Permanent(_)) => return Err(e.to_string()),
                    Err(e) => last = e.to_string(),
                }
            }
            Err(last)
        }
        fn describe(&self) -> String {
            self.0.clone()
        }
    }

    /// Newest Overture release on S3 (`EARTHSIM_OVERTURE_RELEASE` overrides; looked up once per run).
    pub fn overture_release() -> String {
        use crate::overture::*;
        static R: std::sync::OnceLock<String> = std::sync::OnceLock::new();
        R.get_or_init(|| {
            std::env::var("EARTHSIM_OVERTURE_RELEASE").ok().filter(|r| !r.is_empty()).or_else(|| {
                let url = format!("{BUCKET}/?list-type=2&prefix=tiles/&delimiter=/");
                match http_get(&url, None) {
                    Ok(b) => newest_release(&String::from_utf8_lossy(&b)),
                    Err(e) => {
                        crate::log::log(&format!("Overture: release list failed: {e}"));
                        None
                    }
                }
            }).unwrap_or_else(|| FALLBACK_RELEASE.to_string())
        })
        .clone()
    }

    /// Elevation tiles (terrarium PNG) from AWS Terrain Tiles, kept in `cache` so a place seen once works offline.
    pub struct HttpDem;
    impl crate::dem::DemSource for HttpDem {
        fn tile(&self, z: u8, x: u32, y: u32) -> Result<Vec<u8>, String> {
            http_get(&format!("{}/{z}/{x}/{y}.png", crate::dem::TERRARIUM_URL), None).map_err(|e| e.to_string())
        }
    }

    pub fn live_dem(cache: &std::path::Path) -> std::sync::Arc<dyn crate::dem::DemSource> {
        std::sync::Arc::new(crate::dem::DirDem { dir: cache.to_path_buf(), inner: Some(HttpDem) })
    }

    /// Map tiles straight from Overture's archives (base, transportation, buildings), byte ranges cached in `cache`.
    pub fn live_map_source(cache: &std::path::Path) -> crate::basemap::MapSource<Box<dyn crate::pmtiles::RangeSource>> {
        use crate::pmtiles::{CachedRange, PmTiles, RangeSource};
        let release = overture_release();
        let open = |theme: &str| -> Option<PmTiles<Box<dyn RangeSource>>> {
            let src: Box<dyn RangeSource> = Box::new(CachedRange { inner: HttpRange(crate::overture::theme_url(&release, theme)), dir: cache.join(theme) });
            match PmTiles::open(src) {
                Ok(p) => Some(p),
                Err(e) => {
                    crate::log::log(&format!("map: Overture {theme} not available: {e}"));
                    None
                }
            }
        };
        crate::basemap::MapSource { base: open("base"), transport: open("transportation"), buildings: open("buildings") }
    }

    /// Overture over HTTP range requests, opened on first use (release list + archive headers need the network).
    pub struct LazyOverture {
        inner: std::sync::Mutex<Option<Arc<crate::overture::OvertureFetcher<HttpRange>>>>,
        last_try: std::sync::Mutex<Option<std::time::Instant>>,
    }

    impl LazyOverture {
        pub fn new() -> Self {
            LazyOverture { inner: std::sync::Mutex::new(None), last_try: std::sync::Mutex::new(None) }
        }
        fn get(&self) -> Result<Arc<crate::overture::OvertureFetcher<HttpRange>>, FetchError> {
            if let Some(f) = self.inner.lock().unwrap().as_ref() {
                return Ok(f.clone());
            }
            {
                let mut lt = self.last_try.lock().unwrap();
                if lt.is_some_and(|t| t.elapsed() < std::time::Duration::from_secs(20)) {
                    return Err(FetchError::Transient("Overture not reachable (retrying shortly)".into()));
                }
                *lt = Some(std::time::Instant::now());
            }
            use crate::overture::*;
            let release = overture_release();
            crate::log::log(&format!("Overture: using release {release}"));
            let f = OvertureFetcher::new(HttpRange(theme_url(&release, "buildings")), HttpRange(theme_url(&release, "transportation")), &release)
                .map_err(|e| FetchError::Transient(format!("Overture {release}: {e}")))?
                .with_base(HttpRange(theme_url(&release, "base")));
            let f = Arc::new(f);
            *self.inner.lock().unwrap() = Some(f.clone());
            Ok(f)
        }
    }

    impl Fetcher for LazyOverture {
        fn fetch(&self, tile: TileId, query: &str) -> Result<String, FetchError> {
            self.get()?.fetch(tile, query)
        }
        fn describe(&self) -> String {
            match self.inner.lock().unwrap().as_ref() {
                Some(f) => f.describe(),
                None => "Overture (S3 PMTiles)".into(),
            }
        }
    }

    /// Live map source: `EARTHSIM_SOURCE=overpass` uses only Overpass; default is Overture, then Overpass.
    pub fn default_live_fetcher() -> Arc<dyn Fetcher> {
        let overpass: Arc<dyn Fetcher> = Arc::new(WinHttpFetcher::from_env());
        match std::env::var("EARTHSIM_SOURCE").unwrap_or_default().as_str() {
            "overpass" => overpass,
            "overture" => Arc::new(LazyOverture::new()),
            // Overture shapes, OSM storey counts on top (see enrich.rs); Overpass alone if Overture is down
            _ => Arc::new(crate::enrich::EnrichFetcher { base: Arc::new(ChainFetcher(vec![Arc::new(LazyOverture::new()), overpass.clone()])), osm: overpass }),
        }
    }

    impl Fetcher for WinHttpFetcher {
        fn fetch(&self, _tile: TileId, query: &str) -> Result<String, FetchError> {
            // Try the mirror that worked last first, then the others; remember whichever succeeds.
            let n = self.endpoints.len();
            let start = self.current.load(Ordering::Relaxed) % n;
            let mut errors: Vec<String> = Vec::new();
            for k in 0..n {
                let idx = (start + k) % n;
                let url = format!("{}?data={}", self.endpoints[idx], percent_encode(query));
                match self.get(&url) {
                    Ok(t) => {
                        self.current.store(idx, Ordering::Relaxed);
                        return Ok(t);
                    }
                    Err(FetchError::Permanent(m)) if n == 1 => return Err(FetchError::Permanent(m)),
                    Err(e) => {
                        crate::log::log(&format!("  endpoint {} failed: {}", self.endpoints[idx], e));
                        errors.push(format!("{}: {}", self.endpoints[idx].split('/').nth(2).unwrap_or("?"), e));
                    }
                }
            }
            // every mirror failed: transient (retry later with backoff), keeping all reasons for the log and HUD
            Err(FetchError::Transient(errors.join(" | ")))
        }
        fn describe(&self) -> String {
            format!("Overpass via WinHTTP ({})", self.endpoints.join(", "))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percent_encoding_and_url_splitting() {
        assert_eq!(percent_encode("a b&c=d[\"]~-_.é"), "a%20b%26c%3Dd%5B%22%5D~-_.%C3%A9");
        assert_eq!(split_url("https://overpass-api.de/api/interpreter"), Some(("overpass-api.de".into(), 443, "/api/interpreter".into(), true)));
        assert_eq!(split_url("http://localhost:12345/x?y=1"), Some(("localhost".into(), 12345, "/x?y=1".into(), false)));
        assert_eq!(split_url("https://host"), Some(("host".into(), 443, "/".into(), true)));
        assert_eq!(split_url("ftp://x"), None);
        assert_eq!(split_url("https://"), None);
        assert_eq!(split_url("https://h:abc/"), None);
    }

    #[test]
    fn http_status_classification() {
        assert!(matches!(status_error(429), FetchError::Transient(_)));
        assert!(matches!(status_error(504), FetchError::Transient(_)));
        assert!(matches!(status_error(400), FetchError::Permanent(_)));
        assert!(matches!(status_error(403), FetchError::Permanent(_)));
    }

    #[test]
    fn virtual_osm_serves_only_the_requested_tile() {
        let v = VirtualOsm::from_overpass_json(include_str!("../../../data/demo_overpass.json")).unwrap();
        let centre = TileId::from_lat_lon(53.9045, 27.5615, 15);
        let far = TileId::from_lat_lon(48.8584, 2.2945, 15);
        let a = v.fetch(centre, "").unwrap();
        let b = v.fetch(far, "").unwrap();
        assert!(a.len() > 10_000, "centre tile should have data");
        assert!(Json::parse(&b).unwrap().get("elements").unwrap().as_arr().unwrap().is_empty());
        assert_eq!(v.request_count(), 2);
    }

    #[test]
    fn failure_injection_works() {
        let v = VirtualOsm::from_overpass_json(r#"{"elements":[]}"#).unwrap().failing_first(2);
        let t = TileId::from_lat_lon(0.0, 0.0, 15);
        assert!(matches!(v.fetch(t, ""), Err(FetchError::Transient(_))));
        assert!(matches!(v.fetch(t, ""), Err(FetchError::Transient(_))));
        assert!(v.fetch(t, "").is_ok());
        let p = VirtualOsm::from_overpass_json(r#"{"elements":[]}"#).unwrap().always_permanent_error();
        assert!(matches!(p.fetch(t, ""), Err(FetchError::Permanent(_))));
    }
}
