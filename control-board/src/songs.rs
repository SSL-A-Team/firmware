use ateam_lib_stm32::audio::note::Beat;


pub const TEST_SONG: [Beat; 17] = [
    Beat::Note { tone: 392, duration: 250_000 },
    Beat::Note { tone: 392, duration: 250_000 },
    Beat::Note { tone: 440, duration: 250_000 },
    Beat::Note { tone: 523, duration: 250_000 },
    Beat::Note { tone: 392, duration: 250_000 },
    Beat::Note { tone: 392, duration: 250_000 },
    Beat::Note { tone: 440, duration: 250_000 },
    Beat::Note { tone: 523, duration: 250_000 },

    Beat::Note { tone: 392, duration: 125_000 },
    Beat::Note { tone: 329, duration: 125_000 },
    Beat::Note { tone: 392, duration: 125_000 },
    Beat::Note { tone: 329, duration: 125_000 },
    Beat::Note { tone: 440, duration: 125_000 },
    Beat::Note { tone: 329, duration: 125_000 },
    Beat::Note { tone: 523, duration: 125_000 },
    Beat::Note { tone: 329, duration: 125_000 },
    
    Beat::Note { tone: 392, duration: 1_000_000 },
    ];