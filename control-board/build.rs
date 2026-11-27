//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::Command;

include!("../util/fw_image_build.rs");
include!("../util/fw_image_elf_build.rs");

fn main() {
    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // Peripheral firmware images flashed by the control board (see
    // util/fw_image_build.rs and the include_* macros in src/lib.rs).
    println!("cargo:rerun-if-changed=../util/fw_image_build.rs");
    println!("cargo:rerun-if-changed=../util/fw_image_elf_build.rs");
    let wheel_hash = prepare_fw_image(
        Path::new("../motor-controller/build/bin/wheel-torque.bin"),
        "wheel-torque.bin",
        b"WheelImgHashWeel",
    );
    let kicker_bin_dir = Path::new("../kicker-board/target/thumbv7em-none-eabihf/release");
    let kicker_hash =
        prepare_fw_image_from_elf(&kicker_bin_dir.join("kicker"), "kicker.bin", b"KickrImgHashKick");
    prepare_fw_image_from_elf(&kicker_bin_dir.join("hwtest-coms"), "hwtest-coms.bin", b"KickrImgHashKick");
    write_fw_image_hash_consts(
        "fw_image_hashes.rs",
        &[("WHEEL_IMG_HASH", wheel_hash), ("KICKER_IMG_HASH", kicker_hash)],
    );

    // Embed git hashes and dirty flags for all three repos so the firmware
    // can report its exact provenance in the HelloRequest packet.
    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());

    let firmware_root = manifest_dir
        .join("..")
        .canonicalize()
        .expect("firmware root must exist");

    let work_dirs = [
        firmware_root.clone(),
        firmware_root.join("controls"),
        firmware_root.join("software-communication"),
    ];

    // Register precise git file watches so the build script reruns only when
    // the git state of any repo actually changes, not on every source edit.
    for work_dir in &work_dirs {
        register_git_rerun(work_dir);
    }

    let env_prefixes = ["GIT_FIRMWARE", "GIT_CONTROLS", "GIT_COMS"];

    for (work_dir, prefix) in work_dirs.iter().zip(env_prefixes.iter()) {
        let (hash, dirty) = get_git_info(work_dir);
        println!("cargo:rustc-env={}_HASH={}", prefix, hash);
        println!("cargo:rustc-env={}_DIRTY={}", prefix, dirty as u8);
    }
}

// Emit rerun-if-changed for the git files that change when commits are made,
// the checkout moves or files are staged. Paths are resolved with
// `git rev-parse --git-path` so worktrees (where `.git` is a file) and
// submodules (git dir under the superproject's modules/) work. Only existing
// files are watched: cargo treats a missing rerun-if-changed path as always
// changed, which would rebuild the crate on every build.
fn register_git_rerun(work_dir: &Path) {
    let git = |args: &[&str]| -> Option<String> {
        let out = Command::new("git")
            .arg("-C")
            .arg(work_dir)
            .args(args)
            .output()
            .ok()
            .filter(|o| o.status.success())?;
        String::from_utf8(out.stdout).ok().map(|s| s.trim().to_string())
    };

    let mut watched = vec![
        "HEAD".to_string(),
        // Updated on every commit/checkout/reset, even when the branch ref
        // itself lives in packed-refs and has no loose ref file to watch.
        "logs/HEAD".to_string(),
        "packed-refs".to_string(),
        // Changes on `git add` / `git reset`, for the dirty flag.
        "index".to_string(),
    ];
    if let Some(branch_ref) = git(&["symbolic-ref", "-q", "HEAD"]) {
        watched.push(branch_ref);
    }

    for name in &watched {
        let Some(rel) = git(&["rev-parse", "--git-path", name]) else {
            continue;
        };
        // Relative results are relative to work_dir; join keeps absolute ones.
        let path = work_dir.join(rel);
        if path.exists() {
            println!("cargo:rerun-if-changed={}", path.display());
        }
    }
}

// Returns (40-char hex hash, is_dirty).  Falls back to an all-zero hash and
// clean status if git is unavailable, so the build always succeeds.
fn get_git_info(work_dir: &PathBuf) -> (String, bool) {
    let work_str = work_dir.to_str().unwrap_or(".");

    let hash = Command::new("git")
        .args(["-C", work_str, "rev-parse", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|| "0000000000000000000000000000000000000000".to_string());

    // Exit code 1 means dirty; exit code 0 means clean; other values are errors.
    let dirty = Command::new("git")
        .args(["-C", work_str, "diff", "--quiet", "HEAD", "--"])
        .status()
        .map(|s| s.code() == Some(1))
        .unwrap_or(false);

    (hash, dirty)
}
