@echo off
rem Only if you build yourself (needs Rust: https://rustup.rs, MSVC toolchain). Otherwise use the exe from GitHub Actions.
cargo run --release -p earthsim -- %*
pause
