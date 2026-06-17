//! Host-side tests for Queue / Buffer.
//!
//! These mirror the embedded defmt_test suite in lib-stm32/tests/queue.rs so
//! the same logic can be verified quickly with `cargo test` on the development
//! machine without flashing hardware.
//!
//! Each test gets its own static buffers — Queue has no reset method and
//! static state persists across the lifetime of the test binary.
//!
//! DEPTH=4, LENGTH=8 for most tests; DEPTH=2 for the minimum-depth test.
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

static T7_BUFS: [SyncUnsafeCell<Buffer<8>>; 4] =
    [const { SyncUnsafeCell::new(Buffer::<8>::new()) }; 4];
static T7_Q: Queue<8, 4> = Queue::new(&T7_BUFS);

static T8_BUFS: [SyncUnsafeCell<Buffer<8>>; 4] =
    [const { SyncUnsafeCell::new(Buffer::<8>::new()) }; 4];
static T8_Q: Queue<8, 4> = Queue::new(&T8_BUFS);

static T9_BUFS: [SyncUnsafeCell<Buffer<8>>; 4] =
    [const { SyncUnsafeCell::new(Buffer::<8>::new()) }; 4];
static T9_Q: Queue<8, 4> = Queue::new(&T9_BUFS);

static T10_BUFS: [SyncUnsafeCell<Buffer<8>>; 4] =
    [const { SyncUnsafeCell::new(Buffer::<8>::new()) }; 4];
static T10_Q: Queue<8, 4> = Queue::new(&T10_BUFS);

// DEPTH=2 for minimum-depth test
static T11_BUFS: [SyncUnsafeCell<Buffer<8>>; 2] =
    [const { SyncUnsafeCell::new(Buffer::<8>::new()) }; 2];
static T11_Q: Queue<8, 2> = Queue::new(&T11_BUFS);

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

// -----------------------------------------------------------------------
// Test 7: index wrap-around
//
// The most common ring buffer bug: write_index and read_index must wrap
// correctly from DEPTH-1 back to 0. Exercises this by doing a full
// fill → full drain → fill again cycle so both indices cross the boundary.
// -----------------------------------------------------------------------
#[test]
fn index_wrap_around() {
    // First fill + drain — advances both indices to DEPTH (wraps to 0)
    for i in 0u8..4 {
        let mut buf = T7_Q.try_enqueue().unwrap();
        buf.data()[0] = i;
        *buf.len() = 1;
    }
    for expected in 0u8..4 {
        let buf = T7_Q.try_dequeue().unwrap();
        assert_eq!(buf.data()[0], expected, "first pass dequeue order mismatch");
    }
    assert!(!T7_Q.can_dequeue(), "queue must be empty after first drain");

    // Second fill + drain — both indices have now wrapped; verify correctness
    for i in 10u8..14 {
        let mut buf = T7_Q.try_enqueue().unwrap();
        buf.data()[0] = i;
        *buf.len() = 1;
    }
    assert!(T7_Q.is_full(), "queue must be full after second fill");
    for expected in 10u8..14 {
        let buf = T7_Q.try_dequeue().unwrap();
        assert_eq!(buf.data()[0], expected, "second pass dequeue order mismatch (wrap bug)");
    }
    assert!(!T7_Q.can_dequeue(), "queue must be empty after second drain");
}

// -----------------------------------------------------------------------
// Test 8: eviction when write_index == 0
//
// try_enqueue_override has a special branch for write_index == 0:
//   if write_index == 0 { write_index = DEPTH - 1 }
// This is only reachable when the write pointer sits at 0 at eviction
// time. Arrange that by: fill → drain one → fill one (write_index now
// at 0 mod DEPTH after wrapping) → fill to full → trigger override.
// -----------------------------------------------------------------------
#[test]
fn eviction_at_write_index_zero() {
    // Fill completely — write_index is now at 0 (wrapped)
    for i in 0u8..4 {
        let mut buf = T8_Q.try_enqueue().unwrap();
        buf.data()[0] = i;
        *buf.len() = 1;
    }
    // Drain all — read_index wraps to 0, write_index already at 0
    for _ in 0u8..4 {
        let _ = T8_Q.try_dequeue().unwrap();
    }
    // Fill again — write_index advances 0→1→2→3→0 (wraps), ending at 0
    for i in 10u8..14 {
        let mut buf = T8_Q.try_enqueue().unwrap();
        buf.data()[0] = i;
        *buf.len() = 1;
    }
    assert!(T8_Q.is_full(), "queue must be full before eviction-at-zero test");

    // write_index is 0 here — this hits the special branch
    {
        let mut buf = T8_Q.try_enqueue_override().unwrap();
        buf.data()[0] = 99;
        *buf.len() = 1;
    }
    assert!(T8_Q.is_full(), "queue must remain full after eviction at write_index==0");

    // Entry 13 (the newest, at index DEPTH-1) was evicted and replaced with 99
    let expected = [10u8, 11, 12, 99];
    for &exp in &expected {
        let buf = T8_Q.try_dequeue().unwrap();
        assert_eq!(buf.data()[0], exp, "wrong entry after eviction at write_index==0");
    }
    assert!(!T8_Q.can_dequeue());
}

// -----------------------------------------------------------------------
// Test 9: dequeue cancel
//
// cancel() on a DequeueRef must leave read_index and size unchanged so
// the same entry can be dequeued again. Verifies the same data is
// returned on the retry and that the queue depth is unaffected.
// -----------------------------------------------------------------------
#[test]
fn dequeue_cancel_retains_entry() {
    {
        let mut buf = T9_Q.try_enqueue().unwrap();
        buf.data()[0] = 42;
        *buf.len() = 1;
    }

    // Dequeue but cancel — entry must not be consumed
    {
        let buf = T9_Q.try_dequeue().unwrap();
        assert_eq!(buf.data()[0], 42);
        buf.cancel();
    }

    assert!(T9_Q.can_dequeue(), "queue must still have an entry after dequeue cancel");

    // Dequeue again — must see the same entry
    let buf = T9_Q.try_dequeue().unwrap();
    assert_eq!(buf.data()[0], 42, "entry missing after dequeue cancel");
    assert!(!T9_Q.can_dequeue(), "queue must be empty after consuming the entry");
}

// -----------------------------------------------------------------------
// Test 10: in-progress guards
//
// try_enqueue while a write is in progress must return InProgress.
// try_dequeue while a read is in progress must return InProgress.
// -----------------------------------------------------------------------
#[test]
fn in_progress_guards() {
    use ateam_lib_crossarch::queue::Error;

    // Enqueue guard: hold the EnqueueRef and try a second enqueue
    {
        let _held = T10_Q.try_enqueue().unwrap();
        assert_eq!(
            T10_Q.try_enqueue().err().unwrap(),
            Error::InProgress,
            "second try_enqueue must return InProgress while first is held"
        );
        assert_eq!(
            T10_Q.try_enqueue_override().err().unwrap(),
            Error::InProgress,
            "try_enqueue_override must return InProgress while enqueue is held"
        );
    } // _held dropped here — write_in_progress cleared

    // Dequeue guard: enqueue one entry then hold the DequeueRef
    {
        let mut buf = T10_Q.try_enqueue().unwrap();
        buf.data()[0] = 7;
        *buf.len() = 1;
    }
    {
        let _held = T10_Q.try_dequeue().unwrap();
        assert_eq!(
            T10_Q.try_dequeue().err().unwrap(),
            Error::InProgress,
            "second try_dequeue must return InProgress while first is held"
        );
    } // _held dropped — read_in_progress cleared, entry consumed
}

// -----------------------------------------------------------------------
// Test 11: try_dequeue on empty queue
//
// Dequeuing an empty queue must return QueueEmpty, not panic or corrupt
// state. Afterwards a normal enqueue+dequeue must still work.
// -----------------------------------------------------------------------
#[test]
fn dequeue_empty_returns_error() {
    use ateam_lib_crossarch::queue::Error;

    assert_eq!(
        T10_Q.try_dequeue().err().unwrap(),
        Error::QueueEmpty,
        "dequeue on empty queue must return QueueEmpty"
    );

    // Queue must still be usable
    {
        let mut buf = T10_Q.try_enqueue().unwrap();
        buf.data()[0] = 55;
        *buf.len() = 1;
    }
    let buf = T10_Q.try_dequeue().unwrap();
    assert_eq!(buf.data()[0], 55);
}

// -----------------------------------------------------------------------
// Test 12: minimum depth (DEPTH=2)
//
// At DEPTH=2 every enqueue fills the queue and every dequeue empties it.
// The eviction decrement always hits the wrap boundary (write_index
// flips between 0 and 1). Verifies both the normal and override paths
// at minimum depth.
// -----------------------------------------------------------------------
#[test]
fn minimum_depth_operations() {
    // Normal fill + drain
    {
        let mut buf = T11_Q.try_enqueue().unwrap();
        buf.data()[0] = 1;
        *buf.len() = 1;
    }
    {
        let mut buf = T11_Q.try_enqueue().unwrap();
        buf.data()[0] = 2;
        *buf.len() = 1;
    }
    assert!(T11_Q.is_full());

    let b = T11_Q.try_dequeue().unwrap();
    assert_eq!(b.data()[0], 1);
    drop(b);
    let b = T11_Q.try_dequeue().unwrap();
    assert_eq!(b.data()[0], 2);
    drop(b);
    assert!(!T11_Q.can_dequeue());

    // Override at DEPTH=2: fill, then evict+commit
    {
        let mut buf = T11_Q.try_enqueue().unwrap();
        buf.data()[0] = 10;
        *buf.len() = 1;
    }
    {
        let mut buf = T11_Q.try_enqueue().unwrap();
        buf.data()[0] = 11;
        *buf.len() = 1;
    }
    {
        let mut buf = T11_Q.try_enqueue_override().unwrap();
        buf.data()[0] = 99;
        *buf.len() = 1;
    }
    assert!(T11_Q.is_full());
    let b = T11_Q.try_dequeue().unwrap();
    assert_eq!(b.data()[0], 10);
    drop(b);
    let b = T11_Q.try_dequeue().unwrap();
    assert_eq!(b.data()[0], 99, "evicted newest slot must be replaced at DEPTH=2");
    drop(b);
    assert!(!T11_Q.can_dequeue());
}
