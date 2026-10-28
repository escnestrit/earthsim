//! Disk cache of raw Overpass responses, one file per tile: `<root>/<z>/<x>/<y>.json`.
//! Writes are atomic (temp file + rename) so a crash cannot leave a half-written tile.
use crate::tiles::TileId;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

pub struct DiskCache {
    root: PathBuf,
    pub max_age: Duration,
}

impl DiskCache {
    pub fn new(root: impl Into<PathBuf>, max_age: Duration) -> DiskCache {
        DiskCache { root: root.into(), max_age }
    }

    pub fn root(&self) -> &Path {
        &self.root
    }

    pub fn path_of(&self, t: TileId) -> PathBuf {
        self.root.join(t.z.to_string()).join(t.x.to_string()).join(format!("{}.json", t.y))
    }

    /// (text, stale). `stale` means older than `max_age`: usable as a fallback when the network fails.
    pub fn get(&self, t: TileId) -> Option<(String, bool)> {
        let p = self.path_of(t);
        let text = fs::read_to_string(&p).ok()?;
        let age = fs::metadata(&p).and_then(|m| m.modified()).ok().and_then(|m| SystemTime::now().duration_since(m).ok()).unwrap_or(Duration::ZERO);
        Some((text, age > self.max_age))
    }

    pub fn put(&self, t: TileId, text: &str) -> io::Result<()> {
        let p = self.path_of(t);
        if let Some(d) = p.parent() {
            fs::create_dir_all(d)?;
        }
        let tmp = p.with_extension("json.tmp");
        fs::write(&tmp, text)?;
        fs::rename(&tmp, &p)
    }

    pub fn remove(&self, t: TileId) {
        let _ = fs::remove_file(self.path_of(t));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmpdir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("earthsim-test-{}-{}-{}", name, std::process::id(), std::time::SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap().as_nanos()));
        let _ = fs::remove_dir_all(&d);
        d
    }

    #[test]
    fn put_get_roundtrip_and_layout() {
        let d = tmpdir("cache");
        let c = DiskCache::new(&d, Duration::from_secs(3600));
        let t = TileId { z: 15, x: 18892, y: 10535 };
        assert!(c.get(t).is_none());
        c.put(t, "{\"elements\":[]}").unwrap();
        assert!(d.join("15/18892/10535.json").exists());
        let (text, stale) = c.get(t).unwrap();
        assert_eq!(text, "{\"elements\":[]}");
        assert!(!stale);
        assert!(!d.join("15/18892/10535.json.tmp").exists());
        c.remove(t);
        assert!(c.get(t).is_none());
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn zero_max_age_marks_everything_stale_and_overwrite_works() {
        let d = tmpdir("stale");
        let c = DiskCache::new(&d, Duration::ZERO);
        let t = TileId { z: 15, x: 1, y: 2 };
        c.put(t, "a").unwrap();
        std::thread::sleep(Duration::from_millis(20));
        assert!(c.get(t).unwrap().1, "should be stale");
        c.put(t, "b").unwrap();
        assert_eq!(c.get(t).unwrap().0, "b");
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn unwritable_root_reports_error_instead_of_panicking() {
        let c = DiskCache::new("/proc/definitely/not/writable", Duration::from_secs(1));
        assert!(c.put(TileId { z: 15, x: 1, y: 1 }, "x").is_err());
    }
}
