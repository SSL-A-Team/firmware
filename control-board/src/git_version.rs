const fn hex_nibble(c: u8) -> u8 {
    match c {
        b'0'..=b'9' => c - b'0',
        b'a'..=b'f' => c - b'a' + 10,
        b'A'..=b'F' => c - b'A' + 10,
        _ => 0,
    }
}

// Takes the first 4 bytes (8 hex chars) of a 40-char git hash string.
const fn parse_git_hash(hex: &str) -> [u8; 4] {
    let b = hex.as_bytes();
    [
        (hex_nibble(b[0]) << 4) | hex_nibble(b[1]),
        (hex_nibble(b[2]) << 4) | hex_nibble(b[3]),
        (hex_nibble(b[4]) << 4) | hex_nibble(b[5]),
        (hex_nibble(b[6]) << 4) | hex_nibble(b[7]),
    ]
}

const fn parse_dirty(s: &str) -> bool {
    let b = s.as_bytes();
    !b.is_empty() && b[0] == b'1'
}

pub const FIRMWARE_HASH: [u8; 4] = parse_git_hash(env!("GIT_FIRMWARE_HASH"));
pub const CONTROLS_HASH: [u8; 4] = parse_git_hash(env!("GIT_CONTROLS_HASH"));
pub const COMS_HASH: [u8; 4] = parse_git_hash(env!("GIT_COMS_HASH"));

pub const FIRMWARE_DIRTY: bool = parse_dirty(env!("GIT_FIRMWARE_DIRTY"));
pub const CONTROLS_DIRTY: bool = parse_dirty(env!("GIT_CONTROLS_DIRTY"));
pub const COMS_DIRTY: bool = parse_dirty(env!("GIT_COMS_DIRTY"));
