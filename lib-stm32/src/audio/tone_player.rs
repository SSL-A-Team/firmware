
use embassy_time::Timer;

use crate::drivers::audio::PlayTone;
use super::note::{Beat, Song};


pub struct TonePlayer<'a, D: PlayTone> {
    audio_driver: D,
    song: Option<&'a Song>,
}

impl<'a, D: PlayTone> TonePlayer<'a, D> {
    pub fn new(audio_driver: D) -> Self {
        TonePlayer {
            audio_driver,
            song: None,
        }
    }

    pub fn load_song(&mut self, song: &'a Song) -> Result<(), ()> {
        let mut can_play = true;

        'note_loop:
        for beat in song.iter() {
            match beat {
                Beat::Note { tone, duration: _ } => {
                    if !self.audio_driver.can_play_tone(*tone) {
                        can_play = false;
                        break 'note_loop;
                    }
                },
                Beat::Rest(_) => { },
            }
        }

        if !can_play {
            return Err(());
        }

        self.song = Some(song);

        Ok(())
    }

    pub async fn play_song(&mut self) {
        if let Some(song) = self.song {
            for beat in song.iter() {
                match beat {
                    Beat::Note { tone, duration } => {
                        self.audio_driver.play_tone(*tone);
                        Timer::after_micros(*duration as u64).await;
                    },
                    Beat::Rest(duration) => {
                        self.audio_driver.play_tone(0);
                        Timer::after_micros(*duration as u64).await;
                    },
                }
            }

            self.audio_driver.play_tone(0);
        }
    }
}