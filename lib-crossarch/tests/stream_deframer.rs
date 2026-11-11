use ateam_lib_crossarch::stream_deframer::{DeframerError, FrameCheck, StreamDeframer};

// Test protocol: [0xAA, len, payload[len]]
fn check(buf: &[u8]) -> FrameCheck {
    if buf[0] != 0xAA {
        return FrameCheck::Invalid;
    }
    if buf.len() < 2 {
        return FrameCheck::NeedMore;
    }
    let total = 2 + buf[1] as usize;
    if buf.len() < total {
        FrameCheck::NeedMore
    } else {
        FrameCheck::Complete(total)
    }
}

fn next(d: &mut StreamDeframer<64>) -> Option<Vec<u8>> {
    d.next_frame(check, |f| f.to_vec())
}

#[test]
fn single_frame() {
    let mut d = StreamDeframer::<64>::new();
    d.push(&[0xAA, 2, 1, 2]).unwrap();
    assert_eq!(next(&mut d), Some(vec![0xAA, 2, 1, 2]));
    assert_eq!(next(&mut d), None);
    assert!(d.is_empty());
}

#[test]
fn split_across_chunks() {
    let mut d = StreamDeframer::<64>::new();
    d.push(&[0xAA]).unwrap();
    assert_eq!(next(&mut d), None);
    d.push(&[3, 1]).unwrap();
    assert_eq!(next(&mut d), None);
    d.push(&[2, 3]).unwrap();
    assert_eq!(next(&mut d), Some(vec![0xAA, 3, 1, 2, 3]));
}

#[test]
fn merged_frames() {
    let mut d = StreamDeframer::<64>::new();
    d.push(&[0xAA, 1, 9, 0xAA, 0, 0xAA, 2, 7]).unwrap();
    assert_eq!(next(&mut d), Some(vec![0xAA, 1, 9]));
    assert_eq!(next(&mut d), Some(vec![0xAA, 0]));
    assert_eq!(next(&mut d), None);
    d.push(&[8]).unwrap();
    assert_eq!(next(&mut d), Some(vec![0xAA, 2, 7, 8]));
}

#[test]
fn resync_on_garbage() {
    let mut d = StreamDeframer::<64>::new();
    d.push(&[0x00, 0x13, 0xAA, 1, 5]).unwrap();
    assert_eq!(next(&mut d), Some(vec![0xAA, 1, 5]));
    assert_eq!(d.resync_count(), 2);
}

#[test]
fn all_garbage_drains() {
    let mut d = StreamDeframer::<64>::new();
    d.push(&[1, 2, 3]).unwrap();
    assert_eq!(next(&mut d), None);
    assert!(d.is_empty());
}

#[test]
fn zero_length_complete_is_treated_as_invalid() {
    let mut d = StreamDeframer::<64>::new();
    d.push(&[1, 2]).unwrap();
    assert_eq!(d.next_frame(|_| FrameCheck::Complete(0), |f| f.to_vec()), None);
    assert!(d.is_empty());
}

#[test]
fn overflow_discards_buffer() {
    let mut d = StreamDeframer::<64>::new();
    d.push(&[0xAA, 200]).unwrap();
    d.push(&[0u8; 60]).unwrap();
    assert_eq!(d.push(&[0xAA, 0, 0, 0]), Err(DeframerError::Overflow));
    assert_eq!(d.overflow_count(), 1);
    assert_eq!(d.len(), 4);
    assert_eq!(next(&mut d), Some(vec![0xAA, 0]));
}

#[test]
fn overflow_oversized_chunk() {
    let mut d = StreamDeframer::<64>::new();
    d.push(&[0xAA]).unwrap();
    assert_eq!(d.push(&[0u8; 65]), Err(DeframerError::Overflow));
    assert!(d.is_empty());
}
