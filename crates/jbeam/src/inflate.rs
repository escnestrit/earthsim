//! DEFLATE (RFC 1951) decoder, std only. Canonical-Huffman decoding in the style of zlib's `puff.c`.

#[derive(Debug, PartialEq)]
pub enum InflateError {
    Truncated,
    BadBlockType,
    BadStoredLength,
    BadHuffman(&'static str),
    BadSymbol,
    BadDistance,
    TooLarge,
}

impl std::fmt::Display for InflateError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "inflate: {self:?}")
    }
}

struct Bits<'a> {
    data: &'a [u8],
    pos: usize,
    buf: u32,
    cnt: u32,
}

impl<'a> Bits<'a> {
    #[inline]
    fn need(&mut self, n: u32) -> Result<(), InflateError> {
        while self.cnt < n {
            let b = *self.data.get(self.pos).ok_or(InflateError::Truncated)?;
            self.pos += 1;
            self.buf |= (b as u32) << self.cnt;
            self.cnt += 8;
        }
        Ok(())
    }
    #[inline]
    fn bits(&mut self, n: u32) -> Result<u32, InflateError> {
        if n == 0 {
            return Ok(0);
        }
        self.need(n)?;
        let v = self.buf & ((1u32 << n) - 1);
        self.buf >>= n;
        self.cnt -= n;
        Ok(v)
    }
}

const MAXBITS: usize = 15;

struct Huff {
    count: [u16; MAXBITS + 1],
    symbol: Vec<u16>,
}

impl Huff {
    fn new(lengths: &[u8]) -> Result<Huff, InflateError> {
        let mut count = [0u16; MAXBITS + 1];
        for &l in lengths {
            count[l as usize] += 1;
        }
        let mut left: i32 = 1;
        for len in 1..=MAXBITS {
            left <<= 1;
            left -= count[len] as i32;
            if left < 0 {
                return Err(InflateError::BadHuffman("over-subscribed code"));
            }
        }
        let mut offs = [0u16; MAXBITS + 2];
        for len in 1..=MAXBITS {
            offs[len + 1] = offs[len] + count[len];
        }
        let mut symbol = vec![0u16; lengths.len()];
        for (s, &l) in lengths.iter().enumerate() {
            if l != 0 {
                symbol[offs[l as usize] as usize] = s as u16;
                offs[l as usize] += 1;
            }
        }
        Ok(Huff { count, symbol })
    }

    #[inline]
    fn decode(&self, b: &mut Bits) -> Result<u16, InflateError> {
        let (mut code, mut first, mut index) = (0i32, 0i32, 0i32);
        for len in 1..=MAXBITS {
            code |= b.bits(1)? as i32;
            let count = self.count[len] as i32;
            if code - count < first {
                return Ok(self.symbol[(index + (code - first)) as usize]);
            }
            index += count;
            first += count;
            first <<= 1;
            code <<= 1;
        }
        Err(InflateError::BadSymbol)
    }
}

const LBASE: [u16; 29] = [3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258];
const LEXT: [u8; 29] = [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0];
const DBASE: [u16; 30] = [1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577];
const DEXT: [u8; 30] = [0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13];

fn codes(b: &mut Bits, out: &mut Vec<u8>, lit: &Huff, dist: &Huff, limit: usize) -> Result<(), InflateError> {
    loop {
        let sym = lit.decode(b)? as usize;
        if sym < 256 {
            out.push(sym as u8);
        } else if sym == 256 {
            return Ok(());
        } else {
            let s = sym - 257;
            if s >= 29 {
                return Err(InflateError::BadSymbol);
            }
            let len = LBASE[s] as usize + b.bits(LEXT[s] as u32)? as usize;
            let ds = dist.decode(b)? as usize;
            if ds >= 30 {
                return Err(InflateError::BadSymbol);
            }
            let d = DBASE[ds] as usize + b.bits(DEXT[ds] as u32)? as usize;
            if d > out.len() {
                return Err(InflateError::BadDistance);
            }
            let start = out.len() - d;
            for i in 0..len {
                let v = out[start + i];
                out.push(v);
            }
        }
        if out.len() > limit {
            return Err(InflateError::TooLarge);
        }
    }
}

/// Decompress a raw DEFLATE stream. `size_hint` is the expected output size (also used as a hard limit + 1 MiB).
pub fn inflate(data: &[u8], size_hint: usize) -> Result<Vec<u8>, InflateError> {
    let limit = size_hint.saturating_add(1 << 20);
    let mut out = Vec::with_capacity(size_hint);
    let mut b = Bits { data, pos: 0, buf: 0, cnt: 0 };
    loop {
        let last = b.bits(1)?;
        match b.bits(2)? {
            0 => {
                b.buf = 0;
                b.cnt = 0;
                if b.pos + 4 > data.len() {
                    return Err(InflateError::Truncated);
                }
                let len = data[b.pos] as usize | (data[b.pos + 1] as usize) << 8;
                let nlen = data[b.pos + 2] as usize | (data[b.pos + 3] as usize) << 8;
                if len != (!nlen & 0xFFFF) {
                    return Err(InflateError::BadStoredLength);
                }
                b.pos += 4;
                if b.pos + len > data.len() {
                    return Err(InflateError::Truncated);
                }
                out.extend_from_slice(&data[b.pos..b.pos + len]);
                b.pos += len;
            }
            1 => {
                let mut l = [0u8; 288];
                for (i, v) in l.iter_mut().enumerate() {
                    *v = if i < 144 { 8 } else if i < 256 { 9 } else if i < 280 { 7 } else { 8 };
                }
                let lit = Huff::new(&l)?;
                let dist = Huff::new(&[5u8; 30])?;
                codes(&mut b, &mut out, &lit, &dist, limit)?;
            }
            2 => {
                let nlen = b.bits(5)? as usize + 257;
                let ndist = b.bits(5)? as usize + 1;
                let ncode = b.bits(4)? as usize + 4;
                if nlen > 286 || ndist > 30 {
                    return Err(InflateError::BadHuffman("too many codes"));
                }
                const ORDER: [usize; 19] = [16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15];
                let mut lengths = [0u8; 320];
                for &o in ORDER.iter().take(ncode) {
                    lengths[o] = b.bits(3)? as u8;
                }
                let lencode = Huff::new(&lengths[..19])?;
                let mut idx = 0;
                let mut lens = [0u8; 320];
                while idx < nlen + ndist {
                    let sym = lencode.decode(&mut b)?;
                    if sym < 16 {
                        lens[idx] = sym as u8;
                        idx += 1;
                    } else {
                        let (prev, rep) = match sym {
                            16 => {
                                if idx == 0 {
                                    return Err(InflateError::BadHuffman("repeat with no previous length"));
                                }
                                (lens[idx - 1], 3 + b.bits(2)? as usize)
                            }
                            17 => (0, 3 + b.bits(3)? as usize),
                            _ => (0, 11 + b.bits(7)? as usize),
                        };
                        if idx + rep > nlen + ndist {
                            return Err(InflateError::BadHuffman("too many lengths"));
                        }
                        for _ in 0..rep {
                            lens[idx] = prev;
                            idx += 1;
                        }
                    }
                }
                if lens[256] == 0 {
                    return Err(InflateError::BadHuffman("no end-of-block code"));
                }
                let lit = Huff::new(&lens[..nlen])?;
                let dist = Huff::new(&lens[nlen..nlen + ndist])?;
                codes(&mut b, &mut out, &lit, &dist, limit)?;
            }
            _ => return Err(InflateError::BadBlockType),
        }
        if last == 1 {
            return Ok(out);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stored_block() {
        let d = [0x01, 0x05, 0x00, 0xFA, 0xFF, b'h', b'e', b'l', b'l', b'o'];
        assert_eq!(inflate(&d, 5).unwrap(), b"hello");
    }

    #[test]
    fn fixed_huffman_block_from_zlib() {
        // zlib.compress(b"hello hello hello hello", wbits=-15, level 9) (fixed Huffman with a back-reference)
        let d = [0xcb, 0x48, 0xcd, 0xc9, 0xc9, 0x57, 0xc8, 0x40, 0x27, 0x01];
        assert_eq!(inflate(&d, 23).unwrap(), b"hello hello hello hello");
    }

    #[test]
    fn rejects_garbage() {
        assert!(inflate(&[0xff, 0xff, 0xff], 10).is_err());
        assert!(inflate(&[], 0).is_err());
        assert_eq!(inflate(&[0x01, 0x05, 0x00, 0x00, 0x00], 5), Err(InflateError::BadStoredLength));
    }
}
