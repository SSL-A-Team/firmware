use songs::SongId;

pub mod note;
pub mod songs;
pub mod tone_player;
pub mod pitches;

#[derive(Debug)]
pub enum AudioError {
    UnplayablePitch
}

#[derive(Clone, Copy, Debug)]
pub enum AudioCommand {
    PlaySong(SongId),
    Stop,
}