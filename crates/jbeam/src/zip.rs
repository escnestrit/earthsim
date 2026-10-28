//! Minimal ZIP reader (stored + deflate). Reads the central directory only; entries are read on demand
//! from the file, so a 100+ MB mod is not loaded into memory up front.

use crate::inflate::inflate;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::Path;

#[derive(Debug, Clone)]
pub struct Entry {
    pub name: String,
    pub method: u16,
    pub crc: u32,
    pub csize: u64,
    pub size: u64,
    offset: u64,
}

pub struct Zip<R: Read + Seek> {
    r: R,
    pub entries: Vec<Entry>,
}

pub fn crc32(data: &[u8]) -> u32 {
    static TABLE: std::sync::OnceLock<[u32; 256]> = std::sync::OnceLock::new();
    let t = TABLE.get_or_init(|| {
        let mut t = [0u32; 256];
        for (i, v) in t.iter_mut().enumerate() {
            let mut c = i as u32;
            for _ in 0..8 {
                c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
            }
            *v = c;
        }
        t
    });
    let mut c = !0u32;
    for &b in data {
        c = t[((c ^ b as u32) & 0xFF) as usize] ^ (c >> 8);
    }
    !c
}

fn u16le(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([b[o], b[o + 1]])
}
fn u32le(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}
fn u64le(b: &[u8], o: usize) -> u64 {
    let mut a = [0u8; 8];
    a.copy_from_slice(&b[o..o + 8]);
    u64::from_le_bytes(a)
}

impl Zip<File> {
    pub fn open(path: &Path) -> Result<Self, String> {
        let f = File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Zip::new(f)
    }
}

impl<R: Read + Seek> Zip<R> {
    pub fn new(mut r: R) -> Result<Self, String> {
        let len = r.seek(SeekFrom::End(0)).map_err(|e| e.to_string())?;
        let tail_len = len.min(65_557 + 22);
        r.seek(SeekFrom::Start(len - tail_len)).map_err(|e| e.to_string())?;
        let mut tail = vec![0u8; tail_len as usize];
        r.read_exact(&mut tail).map_err(|e| e.to_string())?;
        let eocd = (0..tail.len().saturating_sub(21))
            .rev()
            .find(|&i| tail[i..i + 4] == [0x50, 0x4b, 0x05, 0x06])
            .ok_or("zip: end of central directory not found")?;
        let mut count = u16le(&tail, eocd + 10) as u64;
        let mut cd_size = u32le(&tail, eocd + 12) as u64;
        let mut cd_off = u32le(&tail, eocd + 16) as u64;
        // zip64 locator right before EOCD
        if (cd_off == 0xFFFF_FFFF || count == 0xFFFF || cd_size == 0xFFFF_FFFF) && eocd >= 20 && tail[eocd - 20..eocd - 16] == [0x50, 0x4b, 0x06, 0x07] {
            let rec_off = u64le(&tail, eocd - 20 + 8);
            r.seek(SeekFrom::Start(rec_off)).map_err(|e| e.to_string())?;
            let mut rec = [0u8; 56];
            r.read_exact(&mut rec).map_err(|e| e.to_string())?;
            if rec[0..4] != [0x50, 0x4b, 0x06, 0x06] {
                return Err("zip64: bad end record".into());
            }
            count = u64le(&rec, 32);
            cd_size = u64le(&rec, 40);
            cd_off = u64le(&rec, 48);
        }
        if cd_off + cd_size > len {
            return Err("zip: central directory out of range".into());
        }
        r.seek(SeekFrom::Start(cd_off)).map_err(|e| e.to_string())?;
        let mut cd = vec![0u8; cd_size as usize];
        r.read_exact(&mut cd).map_err(|e| e.to_string())?;
        let mut entries = Vec::with_capacity(count as usize);
        let mut p = 0usize;
        while p + 46 <= cd.len() && cd[p..p + 4] == [0x50, 0x4b, 0x01, 0x02] {
            let method = u16le(&cd, p + 10);
            let crc = u32le(&cd, p + 16);
            let mut csize = u32le(&cd, p + 20) as u64;
            let mut size = u32le(&cd, p + 24) as u64;
            let nlen = u16le(&cd, p + 28) as usize;
            let elen = u16le(&cd, p + 30) as usize;
            let clen = u16le(&cd, p + 32) as usize;
            let mut offset = u32le(&cd, p + 42) as u64;
            if p + 46 + nlen + elen > cd.len() {
                return Err("zip: truncated central directory".into());
            }
            let name = String::from_utf8_lossy(&cd[p + 46..p + 46 + nlen]).replace('\\', "/");
            // zip64 extra field (id 1): fields appear only for those that were 0xFFFFFFFF
            let ex = &cd[p + 46 + nlen..p + 46 + nlen + elen];
            let mut q = 0;
            while q + 4 <= ex.len() {
                let id = u16le(ex, q);
                let sz = u16le(ex, q + 2) as usize;
                if id == 1 {
                    let mut o = q + 4;
                    if size == 0xFFFF_FFFF && o + 8 <= ex.len() {
                        size = u64le(ex, o);
                        o += 8;
                    }
                    if csize == 0xFFFF_FFFF && o + 8 <= ex.len() {
                        csize = u64le(ex, o);
                        o += 8;
                    }
                    if offset == 0xFFFF_FFFF && o + 8 <= ex.len() {
                        offset = u64le(ex, o);
                    }
                }
                q += 4 + sz;
            }
            entries.push(Entry { name, method, crc, csize, size, offset });
            p += 46 + nlen + elen + clen;
        }
        Ok(Zip { r, entries })
    }

    pub fn find(&self, name: &str) -> Option<&Entry> {
        self.entries.iter().find(|e| e.name == name)
    }

    pub fn read(&mut self, e: &Entry) -> Result<Vec<u8>, String> {
        self.r.seek(SeekFrom::Start(e.offset)).map_err(|e| e.to_string())?;
        let mut h = [0u8; 30];
        self.r.read_exact(&mut h).map_err(|e| e.to_string())?;
        if h[0..4] != [0x50, 0x4b, 0x03, 0x04] {
            return Err(format!("zip: bad local header for {}", e.name));
        }
        let skip = u16le(&h, 26) as i64 + u16le(&h, 28) as i64;
        self.r.seek(SeekFrom::Current(skip)).map_err(|e| e.to_string())?;
        let mut comp = vec![0u8; e.csize as usize];
        self.r.read_exact(&mut comp).map_err(|e| e.to_string())?;
        let out = match e.method {
            0 => comp,
            8 => inflate(&comp, e.size as usize).map_err(|x| format!("{}: {x}", e.name))?,
            m => return Err(format!("{}: unsupported compression method {m}", e.name)),
        };
        if out.len() as u64 != e.size {
            return Err(format!("{}: size mismatch ({} != {})", e.name, out.len(), e.size));
        }
        if crc32(&out) != e.crc {
            return Err(format!("{}: CRC mismatch", e.name));
        }
        Ok(out)
    }

    pub fn read_name(&mut self, name: &str) -> Result<Vec<u8>, String> {
        let e = self.find(name).ok_or_else(|| format!("zip: no entry {name}"))?.clone();
        self.read(&e)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::io::Cursor;

    /// Build a stored-only zip in memory (for tests).
    pub fn make_zip(files: &[(&str, &[u8])]) -> Vec<u8> {
        let mut out = Vec::new();
        let mut cd = Vec::new();
        for (name, data) in files {
            let off = out.len() as u32;
            let crc = crc32(data);
            out.extend_from_slice(&[0x50, 0x4b, 3, 4, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
            out.extend_from_slice(&crc.to_le_bytes());
            out.extend_from_slice(&(data.len() as u32).to_le_bytes());
            out.extend_from_slice(&(data.len() as u32).to_le_bytes());
            out.extend_from_slice(&(name.len() as u16).to_le_bytes());
            out.extend_from_slice(&[0, 0]);
            out.extend_from_slice(name.as_bytes());
            out.extend_from_slice(data);
            cd.extend_from_slice(&[0x50, 0x4b, 1, 2, 20, 0, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
            cd.extend_from_slice(&crc.to_le_bytes());
            cd.extend_from_slice(&(data.len() as u32).to_le_bytes());
            cd.extend_from_slice(&(data.len() as u32).to_le_bytes());
            cd.extend_from_slice(&(name.len() as u16).to_le_bytes());
            cd.extend_from_slice(&[0; 12]);
            cd.extend_from_slice(&off.to_le_bytes());
            cd.extend_from_slice(name.as_bytes());
        }
        let cd_off = out.len() as u32;
        out.extend_from_slice(&cd);
        out.extend_from_slice(&[0x50, 0x4b, 5, 6, 0, 0, 0, 0]);
        out.extend_from_slice(&(files.len() as u16).to_le_bytes());
        out.extend_from_slice(&(files.len() as u16).to_le_bytes());
        out.extend_from_slice(&(cd.len() as u32).to_le_bytes());
        out.extend_from_slice(&cd_off.to_le_bytes());
        out.extend_from_slice(&[0, 0]);
        out
    }

    #[test]
    fn crc_known_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn roundtrip_stored() {
        let z = make_zip(&[("a/b.txt", b"hello"), ("c.json", b"{}"), ("empty", b"")]);
        let mut zip = Zip::new(Cursor::new(z)).unwrap();
        assert_eq!(zip.entries.len(), 3);
        assert_eq!(zip.read_name("a/b.txt").unwrap(), b"hello");
        assert_eq!(zip.read_name("empty").unwrap(), b"");
        assert!(zip.read_name("nope").is_err());
    }

    #[test]
    fn detects_corruption() {
        let mut z = make_zip(&[("a.txt", b"hello world")]);
        z[40] ^= 1; // flip a data bit
        let mut zip = Zip::new(Cursor::new(z)).unwrap();
        assert!(zip.read_name("a.txt").unwrap_err().contains("CRC"));
    }

    #[test]
    fn rejects_non_zip() {
        assert!(Zip::new(Cursor::new(vec![0u8; 100])).is_err());
    }
}
