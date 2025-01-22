pub mod note;
pub mod tone_player;
pub mod pitches;

#[derive(Debug)]
pub enum AudioError {
    UnplayablePitch
}