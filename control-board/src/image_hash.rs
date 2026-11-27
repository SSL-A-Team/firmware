//! Hashes of the peripheral firmware images embedded in this image, generated
//! at compile time by build.rs (see util/fw_image_build.rs). Each device reports
//! the hash stored in its own image; a mismatch means it needs reflashing.

include!(concat!(env!("OUT_DIR"), "/fw_image_hashes.rs"));

pub fn get_wheel_img_hash() -> [u8; 16] {
    WHEEL_IMG_HASH
}

pub fn get_kicker_img_hash() -> [u8; 16] {
    KICKER_IMG_HASH
}
