use core::{
    cell::{SyncUnsafeCell, UnsafeCell},
    future::poll_fn,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
    task::{Poll, Waker},
};
use critical_section;

pub struct Buffer<const LENGTH: usize> {
    pub data: [u8; LENGTH],
    len: usize,
}

impl<const LENGTH: usize> Default for Buffer<LENGTH> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const LENGTH: usize> Buffer<LENGTH> {
    pub const fn new() -> Self {
        Self {
            data: [0_u8; LENGTH],
            len: 0,
        }
    }

    pub const EMPTY: Buffer<LENGTH> = Buffer {
        data: [0_u8; LENGTH],
        len: 0,
    };
}

pub struct DequeueRef<'a, const LENGTH: usize, const DEPTH: usize> {
    queue: &'a Queue<LENGTH, DEPTH>,
    data: &'a [u8],
}

impl<const LENGTH: usize, const DEPTH: usize> DequeueRef<'_, LENGTH, DEPTH> {
    pub fn data(&self) -> &[u8] {
        self.data
    }
    pub fn cancel(self) {
        self.queue.cancel_dequeue();
    }
}

impl<const LENGTH: usize, const DEPTH: usize> Drop for DequeueRef<'_, LENGTH, DEPTH> {
    fn drop(&mut self) {
        self.queue.finish_dequeue();
    }
}

pub struct EnqueueRef<'a, const LENGTH: usize, const DEPTH: usize> {
    queue: &'a Queue<LENGTH, DEPTH>,
    data: &'a mut [u8],
    len: &'a mut usize,
    /// True when try_enqueue_override performed an eviction (decremented write_index and size)
    /// to make room. cancel() must restore those decrements so the queue state is coherent.
    evicted: bool,
}

impl<const LENGTH: usize, const DEPTH: usize> EnqueueRef<'_, LENGTH, DEPTH> {
    pub fn data(&mut self) -> &mut [u8] {
        self.data
    }

    pub fn len(&mut self) -> &mut usize {
        self.len
    }

    pub fn cancel(self) {
        if self.evicted {
            // Restore write_index and size so the queue stays at DEPTH entries.
            // Do NOT restore buf.len: the DMA engine may have already written into
            // this buffer, so the old data is gone. Leave len=0 so the slot is
            // treated as an empty/discarded entry by the next consumer.
            self.queue.cancel_enqueue_restore_eviction();
        } else {
            self.queue.cancel_enqueue();
        }
    }
}

impl<const LENGTH: usize, const DEPTH: usize> Drop for EnqueueRef<'_, LENGTH, DEPTH> {
    fn drop(&mut self) {
        self.queue.finish_enqueue();
    }
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    QueueEmpty,
    QueueFull,
    InProgress,
}

pub struct Queue<const LENGTH: usize, const DEPTH: usize> {
    buffers: &'static [SyncUnsafeCell<Buffer<LENGTH>>; DEPTH],
    read_index: AtomicUsize,
    read_in_progress: AtomicBool,
    write_index: AtomicUsize,
    write_in_progress: AtomicBool,
    size: AtomicUsize,
    enqueue_waker: UnsafeCell<Option<Waker>>,
    dequeue_waker: UnsafeCell<Option<Waker>>,
}

unsafe impl<const LENGTH: usize, const DEPTH: usize> Send for Queue<LENGTH, DEPTH> {}
unsafe impl<const LENGTH: usize, const DEPTH: usize> Sync for Queue<LENGTH, DEPTH> {}

impl<const LENGTH: usize, const DEPTH: usize> Queue<LENGTH, DEPTH> {
    pub const fn new(buffers: &'static [SyncUnsafeCell<Buffer<LENGTH>>; DEPTH]) -> Self {
        // we must at least double buffer in mixed priority execution
        assert!(DEPTH >= 2);
        Self {
            buffers,
            read_index: AtomicUsize::new(0),
            read_in_progress: AtomicBool::new(false),
            write_index: AtomicUsize::new(0),
            write_in_progress: AtomicBool::new(false),
            size: AtomicUsize::new(0),
            enqueue_waker: UnsafeCell::new(None),
            dequeue_waker: UnsafeCell::new(None),
        }
    }

    pub fn is_full(&self) -> bool {
        self.size.load(Ordering::SeqCst) >= DEPTH
    }

    pub fn can_dequeue(&self) -> bool {
        !self.read_in_progress.load(Ordering::Relaxed) && self.size.load(Ordering::Relaxed) > 0
    }

    pub fn try_dequeue(&'_ self) -> Result<DequeueRef<'_, LENGTH, DEPTH>, Error> {
        critical_section::with(|_| {
            if self.read_in_progress.load(Ordering::SeqCst) {
                return Err(Error::InProgress);
            }

            if self.size.load(Ordering::SeqCst) > 0 {
                self.read_in_progress.store(true, Ordering::SeqCst);

                let buf = unsafe { &*self.buffers[self.read_index.load(Ordering::SeqCst)].get() };
                let data = &buf.data[..buf.len];

                Ok(DequeueRef { queue: self, data })
            } else {
                Err(Error::QueueEmpty)
            }
        })
    }

    pub async fn dequeue(&'_ self) -> Result<DequeueRef<'_, LENGTH, DEPTH>, Error> {
        poll_fn(|cx| {
            critical_section::with(|_| {
                match self.try_dequeue() {
                    Err(Error::QueueEmpty) => {
                        unsafe { *self.dequeue_waker.get() = Some(cx.waker().clone()) };
                        Poll::Pending
                    }
                    r => Poll::Ready(r),
                }
            })
        })
        .await
    }

    fn cancel_dequeue(&self) {
        self.read_in_progress.store(false, Ordering::SeqCst);
    }

    fn finish_dequeue(&self) {
        critical_section::with(|_| {
            if self.read_in_progress.load(Ordering::SeqCst) {
                self.read_in_progress.store(false, Ordering::SeqCst);

                self.read_index.store(
                    (self.read_index.load(Ordering::SeqCst) + 1) % DEPTH,
                    Ordering::SeqCst,
                );

                let cur_size = self.size.load(Ordering::SeqCst);
                self.size.store(cur_size - 1, Ordering::SeqCst);
            }

            if let Some(w) = unsafe { (*self.enqueue_waker.get()).take() } {
                w.wake();
            }
        });
    }

    pub fn try_enqueue(&'_ self) -> Result<EnqueueRef<'_, LENGTH, DEPTH>, Error> {
        critical_section::with(|_| {
            if self.write_in_progress.load(Ordering::SeqCst) {
                return Err(Error::InProgress);
            }

            if self.size.load(Ordering::SeqCst) < DEPTH {
                self.write_in_progress.store(true, Ordering::SeqCst);
                let buf =
                    unsafe { &mut *self.buffers[self.write_index.load(Ordering::SeqCst)].get() };
                let data = &mut buf.data;
                buf.len = 0;

                Ok(EnqueueRef {
                    queue: self,
                    data,
                    len: &mut buf.len,
                    evicted: false,
                })
            } else {
                Err(Error::QueueFull)
            }
        })
    }

    pub fn try_enqueue_override(&'_ self) -> Result<EnqueueRef<'_, LENGTH, DEPTH>, Error> {
        critical_section::with(|_| {
            if self.write_in_progress.load(Ordering::SeqCst) {
                return Err(Error::InProgress);
            }

            self.write_in_progress.store(true, Ordering::SeqCst);

            let mut write_index = self.write_index.load(Ordering::SeqCst);
            let cur_size = self.size.load(Ordering::SeqCst);
            let evicted = cur_size >= DEPTH;
            if evicted {
                if write_index == 0 {
                    write_index = DEPTH - 1;
                } else {
                    write_index -= 1;
                }

                self.write_index.store(write_index, Ordering::SeqCst);
                self.size.store(cur_size - 1, Ordering::SeqCst);
            }

            let buf = unsafe { &mut *self.buffers[write_index].get() };
            let data = &mut buf.data;
            buf.len = 0;

            Ok(EnqueueRef {
                queue: self,
                data,
                len: &mut buf.len,
                evicted,
            })
        })
    }

    pub async fn enqueue(&'_ self) -> Result<EnqueueRef<'_, LENGTH, DEPTH>, Error> {
        if critical_section::with(|_| unsafe { (*self.enqueue_waker.get()).is_some() }) {
            return Err(Error::InProgress);
        }

        poll_fn(|cx| {
            critical_section::with(|_| {
                match self.try_enqueue() {
                    Err(Error::QueueFull) => {
                        unsafe { *self.enqueue_waker.get() = Some(cx.waker().clone()) };
                        Poll::Pending
                    }
                    r => Poll::Ready(r),
                }
            })
        })
        .await
    }

    fn cancel_enqueue(&self) {
        self.write_in_progress.store(false, Ordering::SeqCst);
    }

    fn cancel_enqueue_restore_eviction(&self) {
        // The eviction in try_enqueue_override decremented write_index and size to free the
        // newest slot. If the DMA produced no data we must undo those decrements so the queue
        // remains at its pre-eviction depth rather than permanently losing one slot.
        critical_section::with(|_| {
            let write_index = self.write_index.load(Ordering::SeqCst);
            self.write_index
                .store((write_index + 1) % DEPTH, Ordering::SeqCst);

            let cur_size = self.size.load(Ordering::SeqCst);
            self.size.store(cur_size + 1, Ordering::SeqCst);

            self.write_in_progress.store(false, Ordering::SeqCst);
        });
    }

    fn finish_enqueue(&self) {
        critical_section::with(|_| {
            if self.write_in_progress.load(Ordering::SeqCst) {
                self.write_in_progress.store(false, Ordering::SeqCst);

                self.write_index.store(
                    (self.write_index.load(Ordering::SeqCst) + 1) % DEPTH,
                    Ordering::SeqCst,
                );

                let cur_size = self.size.load(Ordering::SeqCst);
                self.size.store(cur_size + 1, Ordering::SeqCst);
            }

            if let Some(w) = unsafe { (*self.dequeue_waker.get()).take() } {
                w.wake();
            }
        });
    }
}
