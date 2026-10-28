//! JBeam tables: a header row, data rows, and "modifier" objects that change the defaults of all following rows.
//! A trailing object inside a row applies to that row only.

use scene::json::Json;
use std::collections::HashMap;

use crate::expr::{eval, Val};

#[derive(Clone, Debug, Default)]
pub struct Row {
    /// Header cells and active modifiers; later entries override earlier ones.
    pub vals: Vec<(String, Json)>,
}

impl Row {
    pub fn get(&self, key: &str) -> Option<&Json> {
        self.vals.iter().rev().find(|(k, _)| k == key).map(|(_, v)| v)
    }
    pub fn num(&self, key: &str) -> Option<f64> {
        self.get(key).and_then(|v| v.as_f64())
    }
    pub fn num_or(&self, key: &str, d: f64) -> f64 {
        self.num(key).unwrap_or(d)
    }
    pub fn str(&self, key: &str) -> Option<&str> {
        self.get(key).and_then(|v| v.as_str())
    }
    pub fn flag(&self, key: &str) -> bool {
        match self.get(key) {
            Some(Json::Bool(b)) => *b,
            Some(Json::Num(n)) => *n != 0.0,
            _ => false,
        }
    }
    /// A string-or-list field as a list of strings (`group` can be either).
    pub fn strings(&self, key: &str) -> Vec<String> {
        match self.get(key) {
            Some(Json::Str(s)) if !s.is_empty() => vec![s.clone()],
            Some(Json::Arr(a)) => a.iter().filter_map(|x| x.as_str().map(String::from)).filter(|s| !s.is_empty()).collect(),
            _ => Vec::new(),
        }
    }
}

fn set(state: &mut Vec<(String, Json)>, k: &str, v: Json) {
    let k = k.trim_end_matches(':'); // "torqueCoupling:" marks a node reference; the name is what matters
    if let Some(e) = state.iter_mut().find(|(kk, _)| kk == k) {
        e.1 = v;
    } else {
        state.push((k.to_string(), v));
    }
}

/// Expand a table into rows. Non-table sections (objects) yield no rows.
pub fn rows(section: &Json) -> Vec<Row> {
    let Some(items) = section.as_arr() else { return Vec::new() };
    let mut header: Option<Vec<String>> = None;
    let mut state: Vec<(String, Json)> = Vec::new();
    let mut out = Vec::new();
    for it in items {
        match it {
            Json::Obj(kv) => {
                for (k, v) in kv {
                    set(&mut state, k, v.clone());
                }
            }
            Json::Arr(cells) => {
                if header.is_none() {
                    header = Some(cells.iter().map(|c| c.as_str().unwrap_or("").trim_end_matches(':').to_string()).collect());
                    continue;
                }
                let h = header.as_ref().unwrap();
                let mut vals = state.clone();
                for (i, c) in cells.iter().enumerate() {
                    if let (Json::Obj(kv), true) = (c, i >= h.len()) {
                        for (k, v) in kv {
                            set(&mut vals, k, v.clone());
                        }
                    } else if let Some(name) = h.get(i) {
                        set(&mut vals, name, c.clone());
                    }
                }
                out.push(Row { vals });
            }
            _ => {}
        }
    }
    out
}

/// Replace `"$name"` / `"$=expr"` strings with values. The `variables` section itself is left alone.
pub fn resolve(j: &Json, vars: &HashMap<String, Json>, warn: &mut Vec<String>) -> Json {
    match j {
        Json::Str(s) if s.starts_with('$') => {
            if let Some(expr) = s.strip_prefix("$=") {
                let nums: HashMap<String, f64> = vars.iter().filter_map(|(k, v)| v.as_f64().map(|n| (k.clone(), n))).collect();
                return match eval(expr, &nums) {
                    Ok(Val::Num(n)) => Json::Num(n),
                    Ok(Val::Bool(b)) => Json::Bool(b),
                    Ok(Val::Nil) => Json::Null,
                    Err(e) => {
                        warn.push(format!("expression '{s}': {e}"));
                        Json::Null
                    }
                };
            }
            match vars.get(s) {
                Some(v) => v.clone(),
                None => j.clone(), // unknown variable names stay as text (they may be literal labels)
            }
        }
        Json::Arr(a) => Json::Arr(a.iter().map(|x| resolve(x, vars, warn)).collect()),
        Json::Obj(o) => Json::Obj(o.iter().map(|(k, v)| (k.clone(), if k == "variables" { v.clone() } else { resolve(v, vars, warn) })).collect()),
        _ => j.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lenient::parse;

    #[test]
    fn modifiers_persist_and_row_options_do_not() {
        let j = parse(r#"[["id","x"],{"w":5},["a",1],["b",2,{"w":9}],["c",3],{"w":7,"g":["p","q"]},["d",4]]"#).unwrap();
        let r = rows(&j);
        assert_eq!(r.len(), 4);
        assert_eq!(r[0].num("w"), Some(5.0));
        assert_eq!(r[1].num("w"), Some(9.0));
        assert_eq!(r[2].num("w"), Some(5.0));
        assert_eq!(r[3].num("w"), Some(7.0));
        assert_eq!(r[3].strings("g"), vec!["p", "q"]);
        assert_eq!(r[1].str("id"), Some("b"));
    }

    #[test]
    fn header_colons_stripped() {
        let j = parse(r#"[["id1:","id2:"],["a","b"]]"#).unwrap();
        assert_eq!(rows(&j)[0].str("id2"), Some("b"));
    }

    #[test]
    fn variables_resolve() {
        let mut vars = HashMap::new();
        vars.insert("$k".to_string(), Json::Num(3.0));
        let j = parse(r#"{"a":"$k","b":"$=$k*2+1","c":"plain","d":["$k"],"variables":[["$k"]]}"#).unwrap();
        let mut w = Vec::new();
        let r = resolve(&j, &vars, &mut w);
        assert_eq!(r.get("a").unwrap().as_f64(), Some(3.0));
        assert_eq!(r.get("b").unwrap().as_f64(), Some(7.0));
        assert_eq!(r.get("d").unwrap().as_arr().unwrap()[0].as_f64(), Some(3.0));
        assert_eq!(r.get("c").unwrap().as_str(), Some("plain"));
        assert!(w.is_empty());
    }
}
