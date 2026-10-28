use jbeam::{lenient, zip::Zip};
fn main() {
    let p = std::env::args().nth(1).expect("zip path");
    let mut z = Zip::open(std::path::Path::new(&p)).unwrap();
    let list = z.entries.clone();
    let (mut ok, mut bad, mut empty) = (0, 0, 0);
    for e in list.iter().filter(|e| e.name.ends_with(".jbeam") || e.name.ends_with(".json") || e.name.ends_with(".pc")) {
        let data = z.read(e).unwrap();
        if data.iter().all(|b| b.is_ascii_whitespace()) {
            empty += 1;
            println!("EMPTY {}", e.name);
            continue;
        }
        match lenient::parse(&String::from_utf8_lossy(&data)) {
            Ok(_) => ok += 1,
            Err(err) => {
                bad += 1;
                println!("FAIL {} {err}", e.name);
            }
        }
    }
    println!("ok {ok}, failed {bad}, empty {empty}");
}
