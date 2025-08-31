use ateam_lib_stm32::audio::note::Beat;

pub const TIPPED_WARNING_SONG: [Beat; 18] = [
    Beat::Note {
        tone: 392,
        duration: 125_000,
    },
    Beat::Rest(125_000),
    Beat::Note {
        tone: 392,
        duration: 125_000,
    },
    Beat::Rest(125_000),
    Beat::Note {
        tone: 392,
        duration: 125_000,
    },
    Beat::Rest(125_000),
    Beat::Note {
        tone: 392,
        duration: 300_000,
    },
    Beat::Rest(125_000),
    Beat::Note {
        tone: 392,
        duration: 300_000,
    },
    Beat::Rest(125_000),
    Beat::Note {
        tone: 392,
        duration: 300_000,
    },
    Beat::Rest(125_000),
    Beat::Note {
        tone: 392,
        duration: 125_000,
    },
    Beat::Rest(125_000),
    Beat::Note {
        tone: 392,
        duration: 125_000,
    },
    Beat::Rest(125_000),
    Beat::Note {
        tone: 392,
        duration: 125_000,
    },
    Beat::Rest(1_000_000),
];

pub const BATTERY_WARNING_SONG: [Beat; 4] = [
    Beat::Note {
        tone: 392,
        duration: 250_000,
    },
    Beat::Rest(250_000),
    Beat::Note {
        tone: 392,
        duration: 250_000,
    },
    Beat::Rest(250_000),
];

pub const BATTERY_CRITICAL_SONG: [Beat; 2] = [
    Beat::Note {
        tone: 440,
        duration: 250_000,
    },
    Beat::Note {
        tone: 466,
        duration: 250_000,
    },
];

pub const TEST_SONG: [Beat; 17] = [
    Beat::Note {
        tone: 392,
        duration: 250_000,
    },
    Beat::Note {
        tone: 392,
        duration: 250_000,
    },
    Beat::Note {
        tone: 440,
        duration: 250_000,
    },
    Beat::Note {
        tone: 523,
        duration: 250_000,
    },
    Beat::Note {
        tone: 392,
        duration: 250_000,
    },
    Beat::Note {
        tone: 392,
        duration: 250_000,
    },
    Beat::Note {
        tone: 440,
        duration: 250_000,
    },
    Beat::Note {
        tone: 523,
        duration: 250_000,
    },
    Beat::Note {
        tone: 392,
        duration: 125_000,
    },
    Beat::Note {
        tone: 329,
        duration: 125_000,
    },
    Beat::Note {
        tone: 392,
        duration: 125_000,
    },
    Beat::Note {
        tone: 329,
        duration: 125_000,
    },
    Beat::Note {
        tone: 440,
        duration: 125_000,
    },
    Beat::Note {
        tone: 329,
        duration: 125_000,
    },
    Beat::Note {
        tone: 523,
        duration: 125_000,
    },
    Beat::Note {
        tone: 329,
        duration: 125_000,
    },
    Beat::Note {
        tone: 392,
        duration: 1_000_000,
    },
];
