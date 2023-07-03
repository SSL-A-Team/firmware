use core::{
    cell::UnsafeCell,
    future::poll_fn,
    mem::MaybeUninit,
    task::{Poll, Waker},
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
    queue: &'a Queue<'a, LENGTH, DEPTH>,
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
    queue: &'a Queue<'a, LENGTH, DEPTH>,
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

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    QueueFullEmpty,
    InProgress,
}

pub struct Queue<'a, const LENGTH: usize, const DEPTH: usize> {
    buffers: &'a [Buffer<LENGTH>; DEPTH],
    read_index: UnsafeCell<usize>,
    read_in_progress: UnsafeCell<bool>,
    write_index: UnsafeCell<usize>,
    write_in_progress: UnsafeCell<bool>,
    size: UnsafeCell<usize>,
    enqueue_waker: UnsafeCell<Option<Waker>>,
    dequeue_waker: UnsafeCell<Option<Waker>>,
}

unsafe impl<'a, const LENGTH: usize, const DEPTH: usize> Send for Queue<'a, LENGTH, DEPTH> {}
unsafe impl<'a, const LENGTH: usize, const DEPTH: usize> Sync for Queue<'a, LENGTH, DEPTH> {}

impl<'a, const LENGTH: usize, const DEPTH: usize> Queue<'a, LENGTH, DEPTH> {
    pub const fn new(buffers: &'a mut [Buffer<LENGTH>; DEPTH]) -> Self {
        Self {
            buffers,
            read_index: UnsafeCell::new(0),
            read_in_progress: UnsafeCell::new(false),
            write_index: UnsafeCell::new(0),
            write_in_progress: UnsafeCell::new(false),
            size: UnsafeCell::new(0),
            enqueue_waker: UnsafeCell::new(None),
            dequeue_waker: UnsafeCell::new(None),
        }
    }

    pub fn try_dequeue(&self) -> Result<DequeueRef<LENGTH, DEPTH>, Error> {
        critical_section::with(|_| unsafe {
            if *self.read_in_progress.get() {
                return Err(Error::InProgress);
            }

            if *self.size.get() > 0 {
                *self.read_in_progress.get() = true;
                let buf = &self.buffers[*self.read_index.get()];
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
        critical_section::with(|_| unsafe {
            *self.read_in_progress.get() = false;
        });
    }

    fn finish_dequeue(&self) {
        critical_section::with(|_| unsafe {
            let read_in_progress = self.read_in_progress.get();
            if *read_in_progress {
                *read_in_progress = false;
                *self.read_index.get() = (*self.read_index.get() + 1) % DEPTH;
                *self.size.get() -= 1;
            }
            if let Some(w) = (*self.enqueue_waker.get()).take() {
                w.wake();
            }
        });
    }

    pub fn try_enqueue(&self) -> Result<EnqueueRef<LENGTH, DEPTH>, Error> {
        critical_section::with(|_| unsafe {
            if *self.write_in_progress.get() {
                return Err(Error::InProgress);
            }

            if *self.size.get() < DEPTH {
                *self.write_in_progress.get() = true;
                let buf = &self.buffers[*self.write_index.get()];
                let buf = &mut *(buf as *const _ as *mut Buffer<LENGTH>);
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
        critical_section::with(|_| unsafe {
            *self.write_in_progress.get() = false;
        });
    }

    fn finish_enqueue(&self) {
        critical_section::with(|_| unsafe {
            let write_in_progress = self.write_in_progress.get();
            if *write_in_progress {
                *write_in_progress = false;
                *self.write_index.get() = (*self.write_index.get() + 1) % DEPTH;
                *self.size.get() += 1;
            }
            if let Some(w) = (*self.dequeue_waker.get()).take() {
                w.wake();
            }
        });
    }
}
