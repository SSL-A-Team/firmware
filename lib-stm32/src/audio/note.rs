enum Note {
    Pitch {pitch: u16, duration: u32},
    Rest(u32),
}