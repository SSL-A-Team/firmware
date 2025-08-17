use core::ptr::read_volatile;

struct ImgHash {
    #[allow(dead_code)]
    img_hash_magic: [u8; 16],  // Used to find this struct in binary
    img_hash: [u8; 16]  // Hash is injected after compilation
}

/// Wheel image hash, saved in the control image
static mut WHEEL_IMG_HASH: ImgHash = ImgHash {
    img_hash_magic: *b"WheelImgHashCtrl",
    img_hash: [0; 16]
};

/// Kicker image hash, saved in the control image
static mut KICKER_IMG_HASH: ImgHash = ImgHash {
    img_hash_magic: *b"KickrImgHashCtrl",
    img_hash: [0; 16]
};

pub fn get_wheel_img_hash() -> [u8; 16] {
    // Enforce a read from memory because the hash is injected after compilation
    unsafe {
        return read_volatile(&raw const WHEEL_IMG_HASH.img_hash as *const [u8; 16]);
    }
}

pub fn get_kicker_img_hash() -> [u8; 16] {
    // Enforce a read from memory because the hash is injected after compilation
    unsafe {
        return read_volatile(&raw const KICKER_IMG_HASH.img_hash as *const [u8; 16]);
    }
}