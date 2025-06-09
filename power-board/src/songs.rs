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

pub const POWER_ON_SONG: [Beat; 3] = [
    Beat::Note { tone: 330, duration: 250_000},
    Beat::Note { tone: 392, duration: 250_000},
    Beat::Note { tone: 523, duration: 500_000},
];

pub const BALANCE_CONNECTED_SONG: [Beat; 2] = [
    Beat::Note { tone: 294, duration: 125_000 },
    Beat::Note { tone: 349, duration: 125_000 }
];

pub const BALANCE_DISCONNECTED_SONG: [Beat; 2] = [
    Beat::Note { tone: 349, duration: 125_000 },
    Beat::Note { tone: 294, duration: 125_000 }
];

pub const POWER_OFF_SONG: [Beat; 3] = [
    Beat::Note { tone: 523, duration: 250_000},
    Beat::Note { tone: 392, duration: 250_000},
    Beat::Note { tone: 330, duration: 500_000},
];

pub const SHUTDOWN_REQUESTED_SONG: [Beat; 4] = [
    Beat::Note { tone: 330, duration: 125_000},
    Beat::Rest(125_000),
    Beat::Note { tone: 330, duration: 125_000},
    Beat::Rest(125_000)
];

pub const BATTERY_LOW_SONG: [Beat; 4] = [
    Beat::Note { tone: 392, duration: 250_000 },
    Beat::Rest(250_000),
    Beat::Note { tone: 392, duration: 250_000 },
    Beat::Rest(250_000),
];

pub const BATTERY_CRITICAL_SONG: [Beat; 2] = [
    Beat::Note { tone: 440, duration: 250_000 },
    Beat::Note { tone: 466, duration: 250_000 },
];

pub const BATTERY_UH_OH_SONG: [Beat; 8] = [
    Beat::Note { tone: 440, duration: 250_000 },
    Beat::Note { tone: 311, duration: 250_000 },
    Beat::Note { tone: 440, duration: 250_000 },
    Beat::Note { tone: 311, duration: 250_000 },
    Beat::Note { tone: 440, duration: 250_000 },
    Beat::Note { tone: 311, duration: 250_000 },
    Beat::Note { tone: 440, duration: 250_000 },
    Beat::Note { tone: 311, duration: 250_000 },
];
