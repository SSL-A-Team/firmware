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
}

impl<const LENGTH: usize, const DEPTH: usize> EnqueueRef<'_, LENGTH, DEPTH> {
    pub fn data(&mut self) -> &mut [u8] {
        self.data
    }

    pub fn len(&mut self) -> &mut usize {
        self.len
    }

    pub fn cancel(self) {
        self.queue.cancel_enqueue();
    }
}

impl<const LENGTH: usize, const DEPTH: usize> Drop for EnqueueRef<'_, LENGTH, DEPTH> {
    fn drop(&mut self) {
        self.queue.finish_enqueue();
    }
}

#[derive(PartialEq, Eq, Clone, Copy, Debug, defmt::Format)]
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

                /* Safety: the async access to buffer data is guarded by atomic read/write and queue size flags.
                 * The flagging logic should ensure a buffer can only be referenced be a user or a DMA engine but
                 * not both at once.
                 */
                let buf = unsafe { &*self.buffers[self.read_index.load(Ordering::SeqCst)].get() };
                // let len = unsafe { buf.len.assume_init() } ;
                /* Saftey: this is safe because &buf.data is const/static allocated legally in the main.rs file
                 * (where a user can specify the link section) and so the compiler knows the type and satisfied
                 * defined behavior constraints w.r.t data alignment and init values, and therefore referencing
                 * the buffer means the internal data is valid.
                 */
                let data = &buf.data[..buf.len];
                // let data = unsafe { &MaybeUninit::slice_assume_init_ref(&buf.data)[..buf.len] };

                Ok(DequeueRef { queue: self, data })
            } else {
                Err(Error::QueueEmpty)
            }
        })
    }

    pub async fn dequeue(&'_ self) -> Result<DequeueRef<'_, LENGTH, DEPTH>, Error> {
        // TODO: look at this (when uncommented causes issue cancelling dequeue)
        // if critical_section::with(|_| unsafe { (*self.dequeue_waker.get()).is_some() }) {
        //     return Err(Error::InProgress);
        // }

        poll_fn(|cx| {
            critical_section::with(|_| {
                match self.try_dequeue() {
                    Err(Error::QueueEmpty) => {
                        /* Safety: this raw pointer write is safe because the underlying memory is statically allocated
                         * and the total write operation is atomic across tasks because of the critical_section closure
                         */
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

                // NOTE: this was an atomic fetch_add but that isn't supported on
                // thumbv6m
                let cur_size = self.size.load(Ordering::SeqCst);
                self.size.store(cur_size - 1, Ordering::SeqCst);
            }

            /* Safety: this raw pointer write is safe because the underlying memory is statically allocated
             * and the total write operation is atomic across tasks because of the critical_section closure
             */
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
                /* Safety: the async access to buffer data is guarded by atomic read/write and queue size flags.
                 * The flagging logic should ensure a buffer can only be referenced be a user or a DMA engine but
                 * not both at once.
                 */
                let buf =
                    unsafe { &mut *self.buffers[self.write_index.load(Ordering::SeqCst)].get() };
                /* Saftey: this is safe because &buf.data is const/static allocated legally in the main.rs file
                 * (where a user can specify the link section) and so the compiler knows the type and satisfied
                 * defined behavior constraints w.r.t data alignment and init values, and therefore referencing
                 * the buffer means the internal data is valid.
                 */
                // let data = unsafe { MaybeUninit::slice_assume_init_mut(&mut buf.data) };
                let data = &mut buf.data;

                // TODO CHCEK: https://doc.rust-lang.org/std/mem/union.MaybeUninit.html#method.write-1 this should overwrite the value and
                // return a mut ref to the new value
                // let len = buf.len.write(0);
                buf.len = 0;

                Ok(EnqueueRef {
                    queue: self,
                    data,
                    len: &mut buf.len,
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

            // Doesn't check if the queue is full and just pulls the next buffer.

            self.write_in_progress.store(true, Ordering::SeqCst);
            /* Safety: the async access to buffer data is guarded by atomic read/write and queue size flags.
             * The flagging logic should ensure a buffer can only be referenced be a user or a DMA engine but
             * not both at once.
             */

            // we will return an available write buffer
            // if the queue is currently full, we need to evict the tail entry
            let mut write_index = self.write_index.load(Ordering::SeqCst);
            let cur_size = self.size.load(Ordering::SeqCst);
            if cur_size >= DEPTH {
                // queue is full, free the back entry
                if write_index == 0 {
                    write_index = DEPTH - 1;
                } else {
                    write_index = write_index - 1;
                }

                // write back decremented write index. Write index is incremented to the next buffer
                // when any enqueue is finalized. The size guards if an enqueue can actually write into
                // the buffer pointed to by the write index
                self.write_index.store(write_index, Ordering::SeqCst);

                // write back decremented size, this guards the dequeue from using the
                // entry and seeing it as valid. This is valid because we know any pending dequeue
                // must be on another buffer beacuse the queue is full, and the assert in the constructor
                // requres we are *at least* double buffered, so any dequeue in a lower prio thread is
                // pointing to the other buffer meaning we can just purge the current one
                self.size.store(cur_size - 1, Ordering::SeqCst);

                // the memory in the backing buffer intact but it's now available for writing
            }

            let buf = unsafe { &mut *self.buffers[write_index].get() };
            /* Saftey: this is safe because &buf.data is const/static allocated legally in the main.rs file
             * (where a user can specify the link section) and so the compiler knows the type and satisfied
             * defined behavior constraints w.r.t data alignment and init values, and therefore referencing
             * the buffer means the internal data is valid.
             */
            let data = &mut buf.data;

            // TODO CHCEK: https://doc.rust-lang.org/std/mem/union.MaybeUninit.html#method.write-1 this should overwrite the value and
            // return a mut ref to the new value
            // let len = buf.len.write(0);
            buf.len = 0;

            Ok(EnqueueRef {
                queue: self,
                data,
                len: &mut buf.len,
            })
        })
    }

    pub async fn enqueue(&self) -> Result<EnqueueRef<LENGTH, DEPTH>, Error> {
        /* Safety: this raw pointer access is safe because the underlying memory is statically allocated
         * and the total read operation is atomic across tasks because of the critical_section closure
         */
        if critical_section::with(|_| unsafe { (*self.enqueue_waker.get()).is_some() }) {
            return Err(Error::InProgress);
        }

        poll_fn(|cx| {
            critical_section::with(|_| {
                match self.try_enqueue() {
                    Err(Error::QueueFull) => {
                        /* Safety: this raw pointer write is safe because the underlying memory is statically allocated
                         * and the total write operation is atomic across tasks because of the critical_section closure
                         */
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

    fn finish_enqueue(&self) {
        critical_section::with(|_| {
            if self.write_in_progress.load(Ordering::SeqCst) {
                self.write_in_progress.store(false, Ordering::SeqCst);

                self.write_index.store(
                    (self.write_index.load(Ordering::SeqCst) + 1) % DEPTH,
                    Ordering::SeqCst,
                );

                // NOTE: this was an atomic fetch_add but that isn't supported on
                // thumbv6m
                let cur_size = self.size.load(Ordering::SeqCst);
                self.size.store(cur_size + 1, Ordering::SeqCst);
            }

            /* Safety: this raw pointer write is safe because the underlying memory is statically allocated
             * and the total write operation is atomic across tasks because of the critical_section closure
             */
            if let Some(w) = unsafe { (*self.dequeue_waker.get()).take() } {
                w.wake();
            }
        });
    }
}
