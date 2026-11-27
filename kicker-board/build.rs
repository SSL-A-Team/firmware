use std::path::Path;

include!("../util/fw_image_build.rs");

fn main() {
    // TODO: flip-link?
    println!("cargo:rustc-link-arg=--nmagic");
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    // Motor controller images flashed by the kicker board (see
    // util/fw_image_build.rs and include_external_cpp_bin! in src/lib.rs).
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=../util/fw_image_build.rs");
    let motor_bin_dir = Path::new("../motor-controller/build/bin");
    let dribbler_hash = prepare_fw_image(
        &motor_bin_dir.join("dribbler-torque.bin"),
        "dribbler-torque.bin",
        b"DrbTrqImgHashDrb",
    );
    prepare_fw_image(&motor_bin_dir.join("wheel.bin"), "wheel.bin", b"WheelImgHashWeel");
    write_fw_image_hash_consts("fw_image_hashes.rs", &[("DRIBBLER_IMG_HASH", dribbler_hash)]);
}
