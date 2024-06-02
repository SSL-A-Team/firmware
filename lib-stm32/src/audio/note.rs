pub enum Beat {
    Note {tone: u16, duration: u32},
    Rest(u32),
}

pub type Song = [Beat];