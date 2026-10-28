use jbeam::zip::Zip;
fn main() {
    let p = std::env::args().nth(1).expect("zip path");
    let mut z = Zip::open(std::path::Path::new(&p)).unwrap();
    let list = z.entries.clone();
    let (mut ok, mut bad) = (0, 0);
    for e in &list {
        match z.read(e) {
            Ok(_) => ok += 1,
            Err(err) => {
                bad += 1;
                println!("FAIL {err}");
            }
        }
    }
    println!("{} entries, {ok} ok, {bad} failed", list.len());
}
