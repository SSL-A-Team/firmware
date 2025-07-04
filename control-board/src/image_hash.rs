use core::ptr::read_volatile;

struct WheelImgHash {
    wheel_img_hash_magic: [u8; 16],
    wheel_img_hash: [u8; 16]
}
static mut WHEEL_IMG_HASH: WheelImgHash = WheelImgHash {
    wheel_img_hash_magic: *b"WheelImgHashCtrl",
    wheel_img_hash: [0; 16]
};

pub fn get_wheel_img_hash() -> [u8; 16] {
    // Enforce a read from memory because the hash is injected at build time
    unsafe {
        return read_volatile(&WHEEL_IMG_HASH.wheel_img_hash as *const [u8; 16]);
    }
}