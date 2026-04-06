#![no_std]
#![no_main]
#![feature(sync_unsafe_cell)]

use defmt_rtt as _;
use panic_probe as _;

#[defmt_test::tests]
mod tests {
    use core::cell::SyncUnsafeCell;
    use ateam_lib_stm32::queue::{Buffer, Queue};

    // Each test gets its own static buffers — Queue has no reset method and
    // static state persists across the lifetime of the test binary.
    //
    // DEPTH=4, LENGTH=8 for all tests (chosen to exercise boundary conditions
    // without being wasteful on target RAM).

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
    //
    // Verifies that normal try_enqueue / try_dequeue maintain FIFO order and
    // correctly track the full flag.
    // -----------------------------------------------------------------------
    #[test]
    fn normal_enqueue_dequeue() {
        for i in 0u8..3 {
            let mut buf = T1_Q.try_enqueue().unwrap();
            buf.data()[0] = i;
            *buf.len() = 1;
        }
        defmt::assert!(!T1_Q.is_full(), "queue should not be full with 3/4 entries");

        {
            let mut buf = T1_Q.try_enqueue().unwrap();
            buf.data()[0] = 3;
            *buf.len() = 1;
        }
        defmt::assert!(T1_Q.is_full(), "queue should be full with 4/4 entries");

        for expected in 0u8..4 {
            let buf = T1_Q.try_dequeue().unwrap();
            defmt::assert_eq!(buf.data()[0], expected, "dequeue order mismatch");
        }
        defmt::assert!(!T1_Q.can_dequeue(), "queue should be empty after draining");
    }

    // -----------------------------------------------------------------------
    // Test 2: try_enqueue_override cancel on a non-full queue
    //
    // When the queue is not full no eviction is performed (evicted=false).
    // Cancelling the override must leave the queue depth exactly unchanged.
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

        // Both original entries must still be dequeue-able
        for expected in 0u8..2 {
            let buf = T2_Q.try_dequeue().unwrap();
            defmt::assert_eq!(buf.data()[0], expected);
        }
        defmt::assert!(!T2_Q.can_dequeue(), "queue should be empty after draining 2 entries");
    }

    // -----------------------------------------------------------------------
    // Test 3: try_enqueue_override cancel on a FULL queue (the bug fix)
    //
    // When the queue is full, try_enqueue_override evicts the newest slot by
    // decrementing write_index and size.  If the caller then cancels (e.g.
    // the DMA returned len==0), cancel_enqueue_restore_eviction must undo
    // those decrements so the queue returns to DEPTH entries — not DEPTH-1.
    // -----------------------------------------------------------------------
    #[test]
    fn override_eviction_cancel_restores_depth() {
        for i in 0u8..4 {
            let mut buf = T3_Q.try_enqueue().unwrap();
            buf.data()[0] = i;
            *buf.len() = 1;
        }
        defmt::assert!(T3_Q.is_full(), "queue must be full before eviction test");

        // Eviction occurs; simulating DMA returning len==0 by cancelling
        let buf = T3_Q.try_enqueue_override().unwrap();
        buf.cancel();

        // Queue must still report full — eviction was a no-op
        defmt::assert!(T3_Q.is_full(), "queue must still be full after eviction+cancel");

        // All 4 original entries must still be intact
        for expected in 0u8..4 {
            let buf = T3_Q.try_dequeue().unwrap();
            defmt::assert_eq!(buf.data()[0], expected, "entry corrupted after eviction+cancel");
        }
        defmt::assert!(!T3_Q.can_dequeue());
    }

    // -----------------------------------------------------------------------
    // Test 4: try_enqueue_override commit on a full queue evicts newest entry
    //
    // Verifies the intended eviction behaviour: the newest (last-written) slot
    // is replaced; the older entries survive in FIFO order.
    // -----------------------------------------------------------------------
    #[test]
    fn override_eviction_commit_replaces_newest() {
        for i in 0u8..4 {
            let mut buf = T4_Q.try_enqueue().unwrap();
            buf.data()[0] = i;
            *buf.len() = 1;
        }

        // Evict entry 3 (newest) and replace with 99
        {
            let mut buf = T4_Q.try_enqueue_override().unwrap();
            buf.data()[0] = 99;
            *buf.len() = 1;
        }

        defmt::assert!(T4_Q.is_full(), "queue must remain full after committed override");

        let expected = [0u8, 1, 2, 99];
        for &exp in &expected {
            let buf = T4_Q.try_dequeue().unwrap();
            defmt::assert_eq!(buf.data()[0], exp, "unexpected entry after eviction+commit");
        }
        defmt::assert!(!T4_Q.can_dequeue());
    }

    // -----------------------------------------------------------------------
    // Test 5: repeated eviction + cancel keeps queue stable
    //
    // Simulates bursts where the UART repeatedly triggers DMA idles with no
    // data while the queue is full.  Each cancel must leave the queue at
    // DEPTH entries — not progressively shrink it.
    // -----------------------------------------------------------------------
    #[test]
    fn repeated_eviction_cancel_depth_stable() {
        for i in 0u8..4 {
            let mut buf = T5_Q.try_enqueue().unwrap();
            buf.data()[0] = i;
            *buf.len() = 1;
        }

        for _ in 0..8 {
            defmt::assert!(
                T5_Q.is_full(),
                "queue must stay full across repeated eviction+cancel cycles"
            );
            let buf = T5_Q.try_enqueue_override().unwrap();
            buf.cancel();
        }

        defmt::assert!(T5_Q.is_full(), "queue must be full after all cancels");

        // All 4 original entries must survive unchanged
        for expected in 0u8..4 {
            let buf = T5_Q.try_dequeue().unwrap();
            defmt::assert_eq!(buf.data()[0], expected, "entry corrupted by repeated eviction+cancel");
        }
    }

    // -----------------------------------------------------------------------
    // Test 6: eviction+cancel followed by eviction+commit
    //
    // Verifies that after a cancel correctly restores state, a subsequent
    // override that commits data still produces the right queue contents.
    // This is the full burst scenario: first DMA idle (no data), then a
    // real packet arrives and overwrites the newest slot.
    // -----------------------------------------------------------------------
    #[test]
    fn eviction_cancel_then_eviction_commit() {
        for i in 0u8..4 {
            let mut buf = T6_Q.try_enqueue().unwrap();
            buf.data()[0] = i;
            *buf.len() = 1;
        }

        // First override: eviction + cancel (DMA got nothing)
        {
            let buf = T6_Q.try_enqueue_override().unwrap();
            buf.cancel();
        }
        defmt::assert!(T6_Q.is_full(), "queue must be full after first cancel");

        // Second override: eviction + commit (real data arrives)
        {
            let mut buf = T6_Q.try_enqueue_override().unwrap();
            buf.data()[0] = 99;
            *buf.len() = 1;
        }
        defmt::assert!(T6_Q.is_full(), "queue must be full after commit");

        // Entries [0, 1, 2, 99] — newest (3) was evicted and replaced
        let expected = [0u8, 1, 2, 99];
        for &exp in &expected {
            let buf = T6_Q.try_dequeue().unwrap();
            defmt::assert_eq!(buf.data()[0], exp);
        }
        defmt::assert!(!T6_Q.can_dequeue());
    }
}
