/// Standard CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320; same as zlib and
/// boost::crc_32_type). Bitwise; fast enough for radio packets and calibration data.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}
