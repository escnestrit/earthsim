//! Tiny evaluator for the Lua-like expressions found in `"$=..."` JBeam strings:
//! numbers, `$vars`, `nil`/`true`/`false`, `+ - * / % ^`, comparisons, `and`/`or`/`not`, parentheses.

use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Val {
    Nil,
    Bool(bool),
    Num(f64),
}

impl Val {
    fn truthy(self) -> bool {
        !matches!(self, Val::Nil | Val::Bool(false))
    }
}

struct P<'a> {
    s: &'a [u8],
    i: usize,
    vars: &'a HashMap<String, f64>,
}

type R = Result<Val, String>;

impl<'a> P<'a> {
    fn ws(&mut self) {
        while self.i < self.s.len() && self.s[self.i].is_ascii_whitespace() {
            self.i += 1;
        }
    }
    fn eat(&mut self, t: &str) -> bool {
        self.ws();
        if self.s[self.i..].starts_with(t.as_bytes()) {
            self.i += t.len();
            true
        } else {
            false
        }
    }
    fn word(&mut self, w: &str) -> bool {
        self.ws();
        let e = self.i + w.len();
        if self.s[self.i..].starts_with(w.as_bytes()) && !self.s.get(e).is_some_and(|c| c.is_ascii_alphanumeric() || *c == b'_') {
            self.i = e;
            true
        } else {
            false
        }
    }
    fn or(&mut self) -> R {
        let mut l = self.and()?;
        while self.word("or") {
            let r = self.and()?;
            l = if l.truthy() { l } else { r };
        }
        Ok(l)
    }
    fn and(&mut self) -> R {
        let mut l = self.cmp()?;
        while self.word("and") {
            let r = self.cmp()?;
            l = if l.truthy() { r } else { l };
        }
        Ok(l)
    }
    fn cmp(&mut self) -> R {
        let l = self.add()?;
        for op in ["==", "~=", "!=", "<=", ">=", "<", ">"] {
            if self.eat(op) {
                let r = self.add()?;
                let res = match (op, l, r) {
                    ("==", a, b) => a == b,
                    ("~=" | "!=", a, b) => a != b,
                    (_, Val::Num(a), Val::Num(b)) => match op {
                        "<=" => a <= b,
                        ">=" => a >= b,
                        "<" => a < b,
                        _ => a > b,
                    },
                    _ => return Err("comparison of non-numbers".into()),
                };
                return Ok(Val::Bool(res));
            }
        }
        Ok(l)
    }
    fn num(v: Val) -> Result<f64, String> {
        match v {
            Val::Num(n) => Ok(n),
            _ => Err("arithmetic on nil/bool".into()),
        }
    }
    fn add(&mut self) -> R {
        let mut l = self.mul()?;
        loop {
            self.ws();
            let op = match self.s.get(self.i) {
                Some(b'+') => '+',
                Some(b'-') => '-',
                _ => return Ok(l),
            };
            self.i += 1;
            let r = self.mul()?;
            let (a, b) = (Self::num(l)?, Self::num(r)?);
            l = Val::Num(if op == '+' { a + b } else { a - b });
        }
    }
    fn mul(&mut self) -> R {
        let mut l = self.unary()?;
        loop {
            self.ws();
            let op = match self.s.get(self.i) {
                Some(b'*') => '*',
                Some(b'/') => '/',
                Some(b'%') => '%',
                _ => return Ok(l),
            };
            self.i += 1;
            let r = self.unary()?;
            let (a, b) = (Self::num(l)?, Self::num(r)?);
            l = Val::Num(match op {
                '*' => a * b,
                '/' => a / b,
                _ => a - b * (a / b).floor(),
            });
        }
    }
    fn unary(&mut self) -> R {
        if self.eat("-") {
            return Ok(Val::Num(-Self::num(self.unary()?)?));
        }
        if self.eat("+") {
            return self.unary();
        }
        if self.word("not") {
            return Ok(Val::Bool(!self.unary()?.truthy()));
        }
        self.pow()
    }
    fn pow(&mut self) -> R {
        let b = self.atom()?;
        if self.eat("^") {
            let e = self.unary()?; // right associative
            return Ok(Val::Num(Self::num(b)?.powf(Self::num(e)?)));
        }
        Ok(b)
    }
    fn atom(&mut self) -> R {
        self.ws();
        let Some(&c) = self.s.get(self.i) else { return Err("unexpected end".into()) };
        if c == b'(' {
            self.i += 1;
            let v = self.or()?;
            if !self.eat(")") {
                return Err("missing )".into());
            }
            return Ok(v);
        }
        if c == b'$' {
            let s = self.i + 1;
            self.i = s;
            while self.i < self.s.len() && (self.s[self.i].is_ascii_alphanumeric() || self.s[self.i] == b'_') {
                self.i += 1;
            }
            let name = std::str::from_utf8(&self.s[s..self.i]).unwrap_or("");
            return Ok(self.vars.get(&format!("${name}")).map_or(Val::Nil, |v| Val::Num(*v)));
        }
        if self.word("nil") {
            return Ok(Val::Nil);
        }
        if self.word("true") {
            return Ok(Val::Bool(true));
        }
        if self.word("false") {
            return Ok(Val::Bool(false));
        }
        if c.is_ascii_digit() || c == b'.' {
            let s = self.i;
            while self.i < self.s.len() && (self.s[self.i].is_ascii_digit() || self.s[self.i] == b'.' || (matches!(self.s[self.i], b'e' | b'E')) || (matches!(self.s[self.i], b'+' | b'-') && matches!(self.s[self.i - 1], b'e' | b'E'))) {
                self.i += 1;
            }
            let t = std::str::from_utf8(&self.s[s..self.i]).unwrap_or("");
            return t.parse::<f64>().map(Val::Num).map_err(|_| format!("bad number '{t}'"));
        }
        Err(format!("unexpected '{}'", c as char))
    }
}

pub fn eval(src: &str, vars: &HashMap<String, f64>) -> Result<Val, String> {
    let mut p = P { s: src.as_bytes(), i: 0, vars };
    let v = p.or()?;
    p.ws();
    if p.i != p.s.len() {
        return Err(format!("trailing input at {}", p.i));
    }
    Ok(v)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vars() -> HashMap<String, f64> {
        [("$a".to_string(), 2.0), ("$bias".to_string(), 0.76)].into_iter().collect()
    }
    fn n(s: &str) -> f64 {
        match eval(s, &vars()).unwrap() {
            Val::Num(x) => x,
            v => panic!("{s} -> {v:?}"),
        }
    }

    #[test]
    fn arithmetic_and_precedence() {
        assert_eq!(n("1+2*3"), 7.0);
        assert_eq!(n("($a + 0.152) * 0.6"), 2.152 * 0.6);
        assert_eq!(n("2-$a"), 0.0);
        assert_eq!(n("2^3^2"), 512.0);
        assert_eq!(n("-$a*3"), -6.0);
        assert_eq!(n("7 % 4"), 3.0);
    }

    #[test]
    fn lua_and_or_nil() {
        let src = "$bias == nil and $s*1000 or $s*4600*(1-$bias)";
        let mut v = vars();
        v.insert("$s".into(), 2.0);
        assert_eq!(eval(src, &v).unwrap(), Val::Num(2.0 * 4600.0 * (1.0 - 0.76)));
        v.remove("$bias");
        let src2 = "$bias == nil and $s*1000 or $s*4600";
        assert_eq!(eval(src2, &v).unwrap(), Val::Num(2000.0));
    }

    #[test]
    fn errors() {
        assert!(eval("1 +", &vars()).is_err());
        assert!(eval("(1", &vars()).is_err());
        assert!(eval("nil + 1", &vars()).is_err());
        assert!(eval("1 2", &vars()).is_err());
    }
}
