use std::fs;
use std::io::{BufWriter, Write};

fn generate_pitch_set(ref_freq: u16, octaves_below: i8, octaves_above: i8) {
    // (Ref pitch is assumed to be A4)
    let mut pitches = Vec::<(&str, u16)>::new();
    let scale : [&str; 12] = ["a", "as", "b", "c", "cs", "d", "ds", "e", "f", "fs", "g", "gs"];
    // Calculate ALL the pitches!
    for o in 1..octaves_below {
        for semitone in 0..11 {
            // vector of (pitch name, freq in hertz)
            // -12 semitones for each octave + number of semitones above A in that octave
            let freq = generate_pitch_from_reference(ref_freq, (-12 * o) + semitone);
            let pitch = (scale[semitone as usize], freq);
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
            let pitch = (scale[semitone as usize] + String::from(4 + o), freq);
            pitches.push(pitch);
        }
}

fn write_pitches_to_file(mut pitch_set: Vec<(&str, u16)>){
    // Open the file (audio/pitches.rs)
    let f = fs::File::create("./audio/pitches.rs").expect("Unable to create pitches.rs");
    let mut f = BufWriter::new(f);
    // Write to it
    for pitch in pitch_set {
        writeln!(f, "let {}: u16 = {}", pitch[0], pitch[1]);
    }
    f.write_all(data.as_bytes()).expect("Unable to write pitch data");
}

fn generate_pitch_from_reference(ref_pitch: u16, semitones: i8) -> u16{
    let diff = semitones as f32 / 12_f32;
    return (ref_pitch as f32/ 2_f32.powf(diff)) as u16;
}

fn main() {
    let a4 = 440;
    let mut pitch_set = generate_pitch_set(a4, 2, 2);
    write_pitches_to_file(pitch_set);
}