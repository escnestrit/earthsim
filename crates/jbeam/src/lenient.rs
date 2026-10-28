//! BeamNG "JBeam" is JSON with comments, missing commas between values, trailing commas and sloppy numbers
//! (`1.`, `.5`, `+1`). `normalize` rewrites such text into strict JSON; `parse` feeds it to `scene::json`.

use scene::json::Json;

#[derive(Clone, Copy, PartialEq)]
enum Prev {
    None,
    Open,
    Val,
    Comma,
    Colon,
}

fn fix_number(tok: &str) -> Option<String> {
    let t = tok.strip_prefix('+').unwrap_or(tok);
    let (neg, t) = match t.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, t),
    };
    let (mant, exp) = match t.find(['e', 'E']) {
        Some(i) => (&t[..i], &t[i..]),
        None => (t, ""),
    };
    let (ip, fp) = match mant.find('.') {
        Some(i) => (&mant[..i], &mant[i + 1..]),
        None => (mant, ""),
    };
    if ip.is_empty() && fp.is_empty() {
        return None;
    }
    if !ip.chars().all(|c| c.is_ascii_digit()) || !fp.chars().all(|c| c.is_ascii_digit()) {
        return None;
    }
    if !exp.is_empty() {
        let e = &exp[1..];
        let d = e.strip_prefix(['+', '-']).unwrap_or(e);
        if d.is_empty() || !d.chars().all(|c| c.is_ascii_digit()) {
            return None;
        }
    }
    let ip = if ip.is_empty() { "0" } else { ip };
    let fp = if fp.is_empty() { "0" } else { fp };
    // strict JSON forbids leading zeros ("007"); parse the integer part numerically
    let ip = ip.trim_start_matches('0');
    let ip = if ip.is_empty() { "0" } else { ip };
    Some(format!("{}{}.{}{}", if neg { "-" } else { "" }, ip, fp, exp))
}

pub fn normalize(text: &str) -> String {
    let b = text.as_bytes();
    let n = b.len();
    let mut out = String::with_capacity(n);
    let mut prev = Prev::None;
    let mut i = 0;
    macro_rules! value_start {
        () => {{
            if prev == Prev::Val || (prev == Prev::Comma) {
                out.push(',');
            }
        }};
    }
    while i < n {
        let c = b[i];
        match c {
            b' ' | b'\t' | b'\r' | b'\n' | 0xEF | 0xBB | 0xBF => i += 1, // whitespace + UTF-8 BOM bytes
            b'/' if i + 1 < n && b[i + 1] == b'/' => {
                while i < n && b[i] != b'\n' {
                    i += 1;
                }
            }
            b'/' if i + 1 < n && b[i + 1] == b'*' => {
                i += 2;
                while i + 1 < n && !(b[i] == b'*' && b[i + 1] == b'/') {
                    i += 1;
                }
                i = (i + 2).min(n);
            }
            b'"' => {
                let s = i;
                i += 1;
                while i < n && b[i] != b'"' {
                    i += if b[i] == b'\\' { 2 } else { 1 };
                }
                let e = (i + 1).min(n);
                value_start!();
                // raw control characters inside strings are invalid JSON
                let raw = &text[s..e];
                if raw.bytes().any(|x| x < 0x20) {
                    for ch in raw.chars() {
                        match ch {
                            '\n' => out.push_str("\\n"),
                            '\r' => out.push_str("\\r"),
                            '\t' => out.push_str("\\t"),
                            c if (c as u32) < 0x20 => {}
                            c => out.push(c),
                        }
                    }
                } else {
                    out.push_str(raw);
                }
                prev = Prev::Val;
                i = e;
            }
            b'{' | b'[' => {
                value_start!();
                out.push(c as char);
                prev = Prev::Open;
                i += 1;
            }
            b'}' | b']' => {
                out.push(c as char);
                prev = Prev::Val;
                i += 1;
            }
            b',' => {
                // collapse duplicates / leading commas; trailing ones are dropped because the comma is
                // only emitted when a value follows
                if prev == Prev::Val {
                    prev = Prev::Comma;
                }
                i += 1;
            }
            b':' => {
                out.push(':');
                prev = Prev::Colon;
                i += 1;
            }
            b'-' | b'+' | b'.' | b'0'..=b'9' => {
                let s = i;
                i += 1;
                while i < n && (b[i].is_ascii_digit() || matches!(b[i], b'.' | b'e' | b'E') || ((b[i] == b'-' || b[i] == b'+') && matches!(b[i - 1], b'e' | b'E'))) {
                    i += 1;
                }
                if let Some(num) = fix_number(&text[s..i]) {
                    value_start!();
                    out.push_str(&num);
                    prev = Prev::Val;
                }
            }
            b'a'..=b'z' | b'A'..=b'Z' => {
                let s = i;
                while i < n && b[i].is_ascii_alphanumeric() {
                    i += 1;
                }
                match &text[s..i] {
                    w @ ("true" | "false" | "null") => {
                        value_start!();
                        out.push_str(w);
                        prev = Prev::Val;
                    }
                    "NaN" | "nan" | "inf" | "Infinity" => {
                        value_start!();
                        out.push('0');
                        prev = Prev::Val;
                    }
                    _ => {}
                }
            }
            _ => i += 1,
        }
    }
    out
}

pub fn parse(text: &str) -> Result<Json, String> {
    let s = normalize(text);
    Json::parse(&s).map_err(|e| format!("{e:?}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(s: &str) -> Json {
        parse(s).unwrap_or_else(|e| panic!("{e}: {}", normalize(s)))
    }

    #[test]
    fn comments_and_trailing_commas() {
        let j = p("{ // c\n \"a\": 1, /* x */ \"b\": [1,2,],\n}");
        assert_eq!(j.get("a").unwrap().as_f64(), Some(1.0));
        assert_eq!(j.get("b").unwrap().as_arr().unwrap().len(), 2);
    }

    #[test]
    fn missing_commas() {
        let j = p("{\"a\":1 \"b\":\"x\"\n\"c\":[[1 2][3 4]] \"d\":{\"e\":true}\"f\":null}");
        assert_eq!(j.get("b").unwrap().as_str(), Some("x"));
        assert_eq!(j.get("c").unwrap().as_arr().unwrap().len(), 2);
        assert_eq!(j.get("d").unwrap().get("e").unwrap().as_bool(), Some(true));
        let row = j.get("c").unwrap().as_arr().unwrap()[1].as_arr().unwrap();
        assert_eq!(row[1].as_f64(), Some(4.0));
    }

    #[test]
    fn sloppy_numbers() {
        let j = p("[1., .5, +2, -.25, 3.e2, 007, -0, 1e-3, 2.5E+2]");
        let v: Vec<f64> = j.as_arr().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
        assert_eq!(v, vec![1.0, 0.5, 2.0, -0.25, 300.0, 7.0, 0.0, 0.001, 250.0]);
    }

    #[test]
    fn negative_minus_not_confused_with_row_separator() {
        let j = p("[[\"a\", -1.5, 2], [\"b\" -3 4]]");
        let r = j.as_arr().unwrap()[1].as_arr().unwrap();
        assert_eq!(r[1].as_f64(), Some(-3.0));
        assert_eq!(r[2].as_f64(), Some(4.0));
    }

    #[test]
    fn strings_keep_slashes_and_raw_newlines() {
        let j = p("{\"p\":\"a//b/*c*/\", \"q\":\"line1\nline2\"}");
        assert_eq!(j.get("p").unwrap().as_str(), Some("a//b/*c*/"));
        assert_eq!(j.get("q").unwrap().as_str(), Some("line1\nline2"));
    }

    #[test]
    fn bom_and_empty() {
        assert!(parse("\u{feff}{\"a\":1}").is_ok());
        assert!(parse("").is_err());
    }
}
