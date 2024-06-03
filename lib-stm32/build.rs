use std::fs;
use std::io;
use std::io::{Write, BufRead};
use std::path::Path;
// Have to move this to another package so I can include it.
// use audio::note::Beat;
use ateam_lib_stm32::audio::note::Beat;

const SCALE : [&str; 12] = ["a", "as", "b", "c", "cs", "d", "ds", "e", "f", "fs", "g", "gs"];
const MICROSEC_PER_MIN: f32 = 600_000.0;

fn generate_pitch_set(ref_freq: u16, octaves_below: i8, octaves_above: i8) -> Vec<(String, u16)>{
    // (Ref pitch is assumed to be A4)
    let mut pitches = Vec::<(String, u16)>::new();
    // Calculate ALL the pitches!
    for o in 1..octaves_below + 1 {
        for semitone in 0..12 {
            // vector of (pitch name, freq in hertz)
            // -12 semitones for each octave + number of semitones above A in that octave
            let freq = generate_pitch_from_reference(ref_freq, (-12 * o) + semitone);
            let pitch = (format!("{}{}", SCALE[semitone as usize], 4 - o), freq);
            pitches.push(pitch);
        }
    }
    // Start at 0 since we also have to include a4-a5
    // (starting with the reference pitch)
    for o in 0..octaves_above {
        for semitone in 0..12 {
            // vector of (pitch name, freq in hertz)
            // +12 semitones for each octave + number of semitones above A in that octave
            let freq = generate_pitch_from_reference(ref_freq, (12 * o) + semitone);
            let pitch = (format!("{}{}", SCALE[semitone as usize], 4 + o), freq);
            pitches.push(pitch);
        }
    }
    return pitches;
}

fn write_pitches_to_file(pitch_set: Vec<(String, u16)>){
    /// Write each pitch name as a constant
    /// in the format PITCH_NAME_OCTAVE, ex. 
    /// pub const A4: u16 = 440
    let f = fs::File::create("./src/audio/pitches.rs").expect("Unable to create pitches.rs");
    let mut f = io::BufWriter::new(f);
    for pitch in pitch_set {
        let _ = writeln!(f, "pub const {}: u16 = {};", pitch.0.to_uppercase(), pitch.1);
    }
    let _ = f.flush();
}

fn generate_pitch_from_reference(ref_pitch: u16, semitones: i8) -> u16 {
    let diff = semitones as f32 / 12_f32;
    return (ref_pitch as f32 * 2_f32.powf(diff)) as u16;
}


fn read_lines<P>(filename: P) -> io::Result<io::Lines<io::BufReader<fs::File>>>
where P: AsRef<Path>, {
    let file = fs::File::open(filename).expect(
        "Could not open file!");
    Ok(io::BufReader::new(file).lines())
}

fn remove_comments(line: String) -> String { 
    let mut commented_line: Vec<String> = line.split('#').map(String::from).collect();
    if commented_line.len() > 1 {
        let _ = commented_line.pop();
    }
    return commented_line.join("");
}

fn translate_chars_to_note(input: String, ref_duration: f32) -> Note {
    let (duration_part, pitch_or_rest) = input.split_at(input.find(|c: char| !c.is_digit(10) && c != '.').unwrap_or(input.len()));
    
    let duration = match duration_part.parse::<f32>() {
        Ok(d) => d,
        Err(_) => return Err("Failed to parse duration".to_string()),
    };
    
    if pitch_or_rest.starts_with('r') {
        return Ok(Beat::Rest(duration * ref_duration));
    } else {
        return Ok(Beat::Note(
            pitch_or_rest,
            duration * ref_duration  
        ));
    }
}

fn generate_song(source_file: &Path) -> Vec<Beat> {
    let lines = read_lines(source_file).expect(
        "Unable to read song source file!");
    let mut song = Vec::new();
    let mut ref_duration: Option<f32> = None; 
    for line in lines {
        // Split at # for comments
        let maybe_line = line.expect("Unable to read song file lines!"); 
        let mut stripped_line = remove_comments(maybe_line);
        // We expect the tempo to be set in the first line
        // before any notes are given.
        if ref_duration.is_none(){
            let tempo = stripped_line.parse::<f32>().expect(
                "Unable to obtain song tempo!");
            ref_duration = Some(MICROSEC_PER_MIN / tempo); 
        } else {
            let notes = stripped_line.split(" ");
            for note in notes {
                let next_note = translate_chars_to_note(
                    String::from(note), ref_duration?);
                song.push(next_note);
            } 
        }
    }
    return song; 
}

// fn write_songs_to_file(){}
// We'll create an array of Notes from each song vector
// and write it to the songs.rs file.

fn main() {
    let a4 = 440;
    let pitch_set = generate_pitch_set(a4, 2, 2);
    write_pitches_to_file(pitch_set);
    let songs_path = Path::new("./src/audio/songs");
    let song_list = ["test_song.txt"];
    // Figure out the syntax for the line below
    let mut all_songs: Vec<Vec<Beat>> = Vec::new();
    for song in &song_list {
        let song_path = songs_path.join(song);
        all_songs.push(generate_song(&song_path));
    }
}