enum Beat {
    Note {tone: u16, duration: u32},
    Rest(u32),
}