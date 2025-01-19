use std::fs;
use std::io::{BufWriter, Write};

fn generate_pitch_set(ref_freq: u16, octaves_below: i8, octaves_above: i8) -> Vec<(String, u16)>{
    // (Ref pitch is assumed to be A4)
    let mut pitches = Vec::<(String, u16)>::new();
    let scale : [&str; 12] = ["a", "as", "b", "c", "cs", "d", "ds", "e", "f", "fs", "g", "gs"];
    // Calculate ALL the pitches!
    for o in 1..octaves_below {
        for semitone in 0..11 {
            // vector of (pitch name, freq in hertz)
            // -12 semitones for each octave + number of semitones above A in that octave
            let freq = generate_pitch_from_reference(ref_freq, (-12 * o) + semitone);
            let pitch = (format!("{}{}", scale[semitone as usize], 4 - o), freq);
            pitches.push(pitch);
        }
    }
    // Start at 0 since we also have to include a4-a5
    // (starting with the reference pitch)
    for o in 0..octaves_above - 1 {
        for semitone in 0..11 {
            // vector of (pitch name, freq in hertz)
            // +12 semitones for each octave + number of semitones above A in that octave
            let freq = generate_pitch_from_reference(ref_freq, (12 * o) + semitone);
            let pitch = (format!("{}{}", scale[semitone as usize], 4 + o), freq);
            pitches.push(pitch);
        }
    }
    pitches
}

fn write_pitches_to_file(pitch_set: Vec<(String, u16)>){
    // Write each pitch name as a constant
    // in the format PITCH_NAME_OCTAVE, ex. 
    // pub const A4: u16 = 440
    let f = fs::File::create("./src/audio/pitches.rs").expect("Unable to create pitches.rs");
    let mut f = BufWriter::new(f);
    for pitch in pitch_set {
        let _ = writeln!(f, "pub const {}: u16 = {};", pitch.0.to_uppercase(), pitch.1);
    }
    let _ = f.flush();
}

fn generate_pitch_from_reference(ref_pitch: u16, semitones: i8) -> u16{
    let diff = semitones as f32 / 12_f32;
    (ref_pitch as f32/ 2_f32.powf(diff)) as u16
}

fn main() {
    let a4 = 440;
    let pitch_set = generate_pitch_set(a4, 2, 2);
    // TODO compare contents, only write if different
    // invalidates build caches and makes build times high
    // write_pitches_to_file(pitch_set);
}