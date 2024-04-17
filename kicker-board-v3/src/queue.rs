use core::{
    cell::UnsafeCell, future::poll_fn, mem::MaybeUninit, sync::atomic::{AtomicBool, AtomicUsize, Ordering}, task::{Poll, Waker}
};
use critical_section;

pub struct Buffer<const LENGTH: usize> {
    pub data: [MaybeUninit<u8>; LENGTH],
    pub len: MaybeUninit<usize>,
}

impl<const LENGTH: usize> Buffer<LENGTH> {
    pub const EMPTY: Buffer<LENGTH> = Buffer {
        data: MaybeUninit::uninit_array(),
        len: MaybeUninit::uninit(),
    };
}

pub struct DequeueRef<'a, const LENGTH: usize, const DEPTH: usize> {
    queue: &'a Queue<LENGTH, DEPTH>,
    data: &'a [u8],
}

impl<'a, const LENGTH: usize, const DEPTH: usize> DequeueRef<'a, LENGTH, DEPTH> {
    pub fn data(&self) -> &[u8] {
        self.data
    }
    pub fn cancel(self) {
        self.queue.cancel_dequeue();
    }
}

impl<'a, const LENGTH: usize, const DEPTH: usize> Drop for DequeueRef<'a, LENGTH, DEPTH> {
    fn drop(&mut self) {
        self.queue.finish_dequeue();
    }
}

pub struct EnqueueRef<'a, const LENGTH: usize, const DEPTH: usize> {
    queue: &'a Queue<LENGTH, DEPTH>,
    data: &'a mut [u8],
    len: &'a mut usize,
}

impl<'a, const LENGTH: usize, const DEPTH: usize> EnqueueRef<'a, LENGTH, DEPTH> {
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

impl<'a, const LENGTH: usize, const DEPTH: usize> Drop for EnqueueRef<'a, LENGTH, DEPTH> {
    fn drop(&mut self) {
        self.queue.finish_enqueue();
    }
}

#[derive(PartialEq, Eq, Clone, Copy, Debug, defmt::Format)]
pub enum Error {
    QueueFullEmpty,
    InProgress,
}

pub struct Queue<const LENGTH: usize, const DEPTH: usize> {
    buffers: UnsafeCell<[Buffer<LENGTH>; DEPTH]>,
    read_index: AtomicUsize,
    read_in_progress: AtomicBool,
    write_index: AtomicUsize,
    write_in_progress: AtomicBool,
    size: AtomicUsize,
    enqueue_waker: UnsafeCell<Option<Waker>>,
    dequeue_waker: UnsafeCell<Option<Waker>>,
}

unsafe impl<'a, const LENGTH: usize, const DEPTH: usize> Send for Queue<LENGTH, DEPTH> {}
unsafe impl<'a, const LENGTH: usize, const DEPTH: usize> Sync for Queue<LENGTH, DEPTH> {}

impl<'a, const LENGTH: usize, const DEPTH: usize> Queue<LENGTH, DEPTH> {
    pub const fn new(buffers: UnsafeCell<[Buffer<LENGTH>; DEPTH]>) -> Self {
        Self {
            buffers: buffers,
            read_index: AtomicUsize::new(0),
            read_in_progress: AtomicBool::new(false),
            write_index: AtomicUsize::new(0),
            write_in_progress: AtomicBool::new(false),
            size: AtomicUsize::new(0),
            enqueue_waker: UnsafeCell::new(None),
            dequeue_waker: UnsafeCell::new(None),
        }
    }

    pub fn try_dequeue(&self) -> Result<DequeueRef<LENGTH, DEPTH>, Error> {
        critical_section::with(|_| unsafe {
            if self.read_in_progress.load(Ordering::SeqCst) {
                return Err(Error::InProgress);
            }

            if self.size.load(Ordering::SeqCst) > 0 {
                self.read_in_progress.store(true, Ordering::SeqCst);

                let bufs = &*self.buffers.get();
                let buf = &bufs[self.read_index.load(Ordering::SeqCst)];
                let len = buf.len.assume_init();
                let data = &MaybeUninit::slice_assume_init_ref(&buf.data)[..len];

                Ok(DequeueRef { queue: self, data })
            } else {
                Err(Error::QueueFullEmpty)
            }
        })
    }

    pub async fn dequeue(&self) -> Result<DequeueRef<LENGTH, DEPTH>, Error> {
        // TODO: look at this (when uncommented causes issue cancelling dequeue)
        // if critical_section::with(|_| unsafe { (*self.dequeue_waker.get()).is_some() }) {
        //     return Err(Error::InProgress);
        // }

        poll_fn(|cx| {
            critical_section::with(|_| unsafe {
                match self.try_dequeue() {
                    Err(Error::QueueFullEmpty) => {
                        *self.dequeue_waker.get() = Some(cx.waker().clone());
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
        critical_section::with(|_| unsafe {
            if self.read_in_progress.load(Ordering::SeqCst) {
                self.read_in_progress.store(false, Ordering::SeqCst);

                self.read_index.store((self.read_index.load(Ordering::SeqCst) + 1) % DEPTH, Ordering::SeqCst);
                self.size.fetch_sub(1, Ordering::SeqCst);
            }

            if let Some(w) = (*self.enqueue_waker.get()).take() {
                w.wake();
            }
        });
    }

    pub fn try_enqueue(&self) -> Result<EnqueueRef<LENGTH, DEPTH>, Error> {
        critical_section::with(|_| unsafe {
            if self.write_in_progress.load(Ordering::SeqCst) {
                return Err(Error::InProgress);
            }

            if self.size.load(Ordering::SeqCst) < DEPTH {
                self.write_in_progress.store(true, Ordering::SeqCst);
                let bufs = &mut *self.buffers.get();
                let buf = &mut bufs[self.write_index.load(Ordering::SeqCst)];
                let data = MaybeUninit::slice_assume_init_mut(&mut buf.data);
                let len = buf.len.write(0);

                Ok(EnqueueRef {
                    queue: self,
                    data,
                    len: len,
                })
            } else {
                Err(Error::QueueFullEmpty)
            }
        })
    }

    pub async fn enqueue(&self) -> Result<EnqueueRef<LENGTH, DEPTH>, Error> {
        if critical_section::with(|_| unsafe { (*self.enqueue_waker.get()).is_some() }) {
            return Err(Error::InProgress);
        }

        poll_fn(|cx| {
            critical_section::with(|_| unsafe {
                match self.try_enqueue() {
                    Err(Error::QueueFullEmpty) => {
                        *self.enqueue_waker.get() = Some(cx.waker().clone());
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
        critical_section::with(|_| unsafe {
            if self.write_in_progress.load(Ordering::SeqCst) {
                self.write_in_progress.store(false, Ordering::SeqCst);

                self.write_index.store((self.write_index.load(Ordering::SeqCst) + 1) % DEPTH, Ordering::SeqCst);
                self.size.fetch_add(1, Ordering::SeqCst);
            }

            if let Some(w) = (*self.dequeue_waker.get()).take() {
                w.wake();
            }
        });
    }
}
