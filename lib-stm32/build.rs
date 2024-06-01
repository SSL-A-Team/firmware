fn generate_pitch_set(ref_pitch: u16) {
    // (Ref pitch is assumed to be A4)
    // Calculate all (reasonable) pitches
    // Open the file (audio/pitches.rs)
    // Write to it
    // Close the file
}

fn generate_pitch_from_reference(ref_pitch: u16, semitones: i8) -> u16{
   if semitones < 0 {
    let diff = (semitones * -1) as u16;
    return ref_pitch / 2.pow(diff/12);
   } else {
    return 2.pow(semitones/12) * ref_pitch;
   }
}

fn main() {

}