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
use std::path::PathBuf;
use std::process::Command;

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

    // Embed git hashes and dirty flags for all three repos so the firmware
    // can report its exact provenance in the HelloRequest packet.
    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());

    let firmware_root = manifest_dir
        .join("..")
        .canonicalize()
        .expect("firmware root must exist");

    // Register precise git file watches so the build script reruns only when
    // the git state of any repo actually changes, not on every source edit.
    let git_dirs = [
        firmware_root.join(".git"),
        firmware_root.join(".git/modules/controls"),
        firmware_root.join(".git/modules/software-communication"),
    ];

    for git_dir in &git_dirs {
        register_git_rerun(git_dir);
    }

    let work_dirs = [
        firmware_root.clone(),
        firmware_root.join("controls"),
        firmware_root.join("software-communication"),
    ];

    let env_prefixes = ["GIT_FIRMWARE", "GIT_CONTROLS", "GIT_COMS"];

    for (work_dir, prefix) in work_dirs.iter().zip(env_prefixes.iter()) {
        let (hash, dirty) = get_git_info(work_dir);
        println!("cargo:rustc-env={}_HASH={}", prefix, hash);
        println!("cargo:rustc-env={}_DIRTY={}", prefix, dirty as u8);
    }
}

// Emit rerun-if-changed for the files inside a git directory that change when
// commits are made or the working tree is modified.  Watching only these files
// (rather than the whole source tree) keeps incremental builds fast.
fn register_git_rerun(git_dir: &PathBuf) {
    let head = git_dir.join("HEAD");
    println!("cargo:rerun-if-changed={}", head.display());

    // HEAD is a symbolic ref in normal operation; also watch the branch file
    // so a new commit on the current branch triggers a rerun.
    if let Ok(content) = std::fs::read_to_string(&head) {
        if let Some(ref_path) = content.trim().strip_prefix("ref: ") {
            let ref_file = git_dir.join(ref_path);
            println!("cargo:rerun-if-changed={}", ref_file.display());
        }
    }

    // packed-refs is updated by `git gc` and can hold the tip of any branch.
    println!("cargo:rerun-if-changed={}", git_dir.join("packed-refs").display());

    // The index changes on `git add` / `git reset`; watching it lets us detect
    // whether the working tree has staged changes at build time.
    println!("cargo:rerun-if-changed={}", git_dir.join("index").display());
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
