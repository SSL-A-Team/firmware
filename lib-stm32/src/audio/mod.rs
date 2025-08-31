use songs::SongId;

pub mod note;
pub mod pitches;
pub mod songs;
pub mod tone_player;

#[derive(Debug)]
pub enum AudioError {
    UnplayablePitch,
}

#[derive(Clone, Copy, Debug)]
pub enum AudioCommand {
    PlaySong(SongId),
    Stop,
}
