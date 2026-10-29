//! Tiny append-only log file (`earthsim.log`), so a failed download is never lost with a closing console.
use std::io::Write;
use std::sync::{Mutex, OnceLock};
use std::time::Instant;

struct Log {
    file: Option<std::fs::File>,
    t0: Instant,
}

static LOG: OnceLock<Mutex<Log>> = OnceLock::new();

/// Start logging to `path` (truncated). Safe to call once; later calls are ignored.
pub fn init(path: impl AsRef<std::path::Path>) {
    // keep the previous run's log next to it (earthsim.prev.log), so a demo start does not wipe a drive
    let p = path.as_ref();
    if p.exists() {
        let _ = std::fs::rename(p, p.with_extension("prev.log"));
    }
    let file = std::fs::File::create(p).ok();
    let _ = LOG.set(Mutex::new(Log { file, t0: Instant::now() }));
    log(&format!("earthsim log started, file {}", path.as_ref().display()));
}

pub fn log(msg: &str) {
    let Some(m) = LOG.get() else { return };
    let Ok(mut l) = m.lock() else { return };
    let line = format!("[{:8.2}s] {}\n", l.t0.elapsed().as_secs_f64(), msg);
    if let Some(f) = l.file.as_mut() {
        let _ = f.write_all(line.as_bytes());
        let _ = f.flush();
    }
    eprint!("{line}");
}

#[cfg(test)]
mod tests {
    #[test]
    fn logging_without_init_is_harmless() {
        super::log("nothing happens");
    }
}
