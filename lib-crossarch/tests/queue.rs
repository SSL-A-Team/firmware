//! Host-side tests for Queue / Buffer.
//!
//! These mirror the embedded defmt_test suite in lib-stm32/tests/queue.rs so
//! the same logic can be verified quickly with `cargo test` on the development
//! machine without flashing hardware.
//!
//! Each test gets its own static buffers — Queue has no reset method and
//! static state persists across the lifetime of the test binary.
//!
//! DEPTH=4, LENGTH=8 for all tests.
#![feature(sync_unsafe_cell)]

use core::cell::SyncUnsafeCell;
use ateam_lib_crossarch::queue::{Buffer, Queue};

static T1_BUFS: [SyncUnsafeCell<Buffer<8>>; 4] =
    [const { SyncUnsafeCell::new(Buffer::<8>::new()) }; 4];
static T1_Q: Queue<8, 4> = Queue::new(&T1_BUFS);

static T2_BUFS: [SyncUnsafeCell<Buffer<8>>; 4] =
    [const { SyncUnsafeCell::new(Buffer::<8>::new()) }; 4];
static T2_Q: Queue<8, 4> = Queue::new(&T2_BUFS);

static T3_BUFS: [SyncUnsafeCell<Buffer<8>>; 4] =
    [const { SyncUnsafeCell::new(Buffer::<8>::new()) }; 4];
static T3_Q: Queue<8, 4> = Queue::new(&T3_BUFS);

static T4_BUFS: [SyncUnsafeCell<Buffer<8>>; 4] =
    [const { SyncUnsafeCell::new(Buffer::<8>::new()) }; 4];
static T4_Q: Queue<8, 4> = Queue::new(&T4_BUFS);

static T5_BUFS: [SyncUnsafeCell<Buffer<8>>; 4] =
    [const { SyncUnsafeCell::new(Buffer::<8>::new()) }; 4];
static T5_Q: Queue<8, 4> = Queue::new(&T5_BUFS);

static T6_BUFS: [SyncUnsafeCell<Buffer<8>>; 4] =
    [const { SyncUnsafeCell::new(Buffer::<8>::new()) }; 4];
static T6_Q: Queue<8, 4> = Queue::new(&T6_BUFS);

// -----------------------------------------------------------------------
// Test 1: basic enqueue / dequeue round-trip
// -----------------------------------------------------------------------
#[test]
fn normal_enqueue_dequeue() {
    for i in 0u8..3 {
        let mut buf = T1_Q.try_enqueue().unwrap();
        buf.data()[0] = i;
        *buf.len() = 1;
    }
    assert!(!T1_Q.is_full(), "queue should not be full with 3/4 entries");

    {
        let mut buf = T1_Q.try_enqueue().unwrap();
        buf.data()[0] = 3;
        *buf.len() = 1;
    }
    assert!(T1_Q.is_full(), "queue should be full with 4/4 entries");

    for expected in 0u8..4 {
        let buf = T1_Q.try_dequeue().unwrap();
        assert_eq!(buf.data()[0], expected, "dequeue order mismatch");
    }
    assert!(!T1_Q.can_dequeue(), "queue should be empty after draining");
}

// -----------------------------------------------------------------------
// Test 2: try_enqueue_override cancel on a non-full queue
// -----------------------------------------------------------------------
#[test]
fn override_no_eviction_cancel_preserves_depth() {
    for i in 0u8..2 {
        let mut buf = T2_Q.try_enqueue().unwrap();
        buf.data()[0] = i;
        *buf.len() = 1;
    }

    // No eviction — queue has room
    let buf = T2_Q.try_enqueue_override().unwrap();
    buf.cancel();

    for expected in 0u8..2 {
        let buf = T2_Q.try_dequeue().unwrap();
        assert_eq!(buf.data()[0], expected);
    }
    assert!(!T2_Q.can_dequeue(), "queue should be empty after draining 2 entries");
}

// -----------------------------------------------------------------------
// Test 3: try_enqueue_override cancel on a FULL queue (the bug fix)
// -----------------------------------------------------------------------
#[test]
fn override_eviction_cancel_restores_depth() {
    for i in 0u8..4 {
        let mut buf = T3_Q.try_enqueue().unwrap();
        buf.data()[0] = i;
        *buf.len() = 1;
    }
    assert!(T3_Q.is_full(), "queue must be full before eviction test");

    let buf = T3_Q.try_enqueue_override().unwrap();
    buf.cancel();

    assert!(T3_Q.is_full(), "queue must still be full after eviction+cancel");

    // Entries 0-2 are intact; the evicted slot (entry 3) is discarded — its buffer
    // was handed to the DMA engine so the old data cannot be trusted. It comes back
    // as a 0-length entry.
    for expected in 0u8..3 {
        let buf = T3_Q.try_dequeue().unwrap();
        assert_eq!(buf.data()[0], expected, "entry corrupted after eviction+cancel");
    }
    let discarded = T3_Q.try_dequeue().unwrap();
    assert_eq!(discarded.data().len(), 0, "evicted+cancelled slot must be empty");
    assert!(!T3_Q.can_dequeue());
}

// -----------------------------------------------------------------------
// Test 4: try_enqueue_override commit on a full queue evicts newest entry
// -----------------------------------------------------------------------
#[test]
fn override_eviction_commit_replaces_newest() {
    for i in 0u8..4 {
        let mut buf = T4_Q.try_enqueue().unwrap();
        buf.data()[0] = i;
        *buf.len() = 1;
    }

    {
        let mut buf = T4_Q.try_enqueue_override().unwrap();
        buf.data()[0] = 99;
        *buf.len() = 1;
    }

    assert!(T4_Q.is_full(), "queue must remain full after committed override");

    let expected = [0u8, 1, 2, 99];
    for &exp in &expected {
        let buf = T4_Q.try_dequeue().unwrap();
        assert_eq!(buf.data()[0], exp, "unexpected entry after eviction+commit");
    }
    assert!(!T4_Q.can_dequeue());
}

// -----------------------------------------------------------------------
// Test 5: repeated eviction + cancel keeps queue stable
// -----------------------------------------------------------------------
#[test]
fn repeated_eviction_cancel_depth_stable() {
    for i in 0u8..4 {
        let mut buf = T5_Q.try_enqueue().unwrap();
        buf.data()[0] = i;
        *buf.len() = 1;
    }

    for _ in 0..8 {
        assert!(T5_Q.is_full(), "queue must stay full across repeated eviction+cancel cycles");
        let buf = T5_Q.try_enqueue_override().unwrap();
        buf.cancel();
    }

    assert!(T5_Q.is_full(), "queue must be full after all cancels");

    // Entries 0-2 are intact; the newest slot is repeatedly evicted+cancelled so it
    // always comes back empty (DMA may have touched it).
    for expected in 0u8..3 {
        let buf = T5_Q.try_dequeue().unwrap();
        assert_eq!(buf.data()[0], expected, "entry corrupted by repeated eviction+cancel");
    }
    let discarded = T5_Q.try_dequeue().unwrap();
    assert_eq!(discarded.data().len(), 0, "repeatedly evicted+cancelled slot must be empty");
}

// -----------------------------------------------------------------------
// Test 6: eviction+cancel followed by eviction+commit
// -----------------------------------------------------------------------
#[test]
fn eviction_cancel_then_eviction_commit() {
    for i in 0u8..4 {
        let mut buf = T6_Q.try_enqueue().unwrap();
        buf.data()[0] = i;
        *buf.len() = 1;
    }

    {
        let buf = T6_Q.try_enqueue_override().unwrap();
        buf.cancel();
    }
    assert!(T6_Q.is_full(), "queue must be full after first cancel");

    {
        let mut buf = T6_Q.try_enqueue_override().unwrap();
        buf.data()[0] = 99;
        *buf.len() = 1;
    }
    assert!(T6_Q.is_full(), "queue must be full after commit");

    let expected = [0u8, 1, 2, 99];
    for &exp in &expected {
        let buf = T6_Q.try_dequeue().unwrap();
        assert_eq!(buf.data()[0], exp);
    }
    assert!(!T6_Q.can_dequeue());
}
