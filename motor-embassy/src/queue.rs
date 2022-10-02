use core::{
    future::{poll_fn, Future},
    mem::MaybeUninit,
    sync::atomic::{AtomicUsize, Ordering},
    task::Poll,
};

use embassy_sync::waitqueue::AtomicWaker;

// TODO: is this neccecary/useful?
pub fn poll_fut<F: Future>(fut: &mut F) -> core::task::Poll<F::Output> {
    static VTABLE: core::task::RawWakerVTable = core::task::RawWakerVTable::new(
        |_| core::task::RawWaker::new(core::ptr::null(), &VTABLE),
        |_| {},
        |_| {},
        |_| {},
    );
    let fut = unsafe { core::pin::Pin::new_unchecked(fut) };

    let raw_waker = core::task::RawWaker::new(core::ptr::null(), &VTABLE);
    let waker = unsafe { core::task::Waker::from_raw(raw_waker) };
    let mut cx = core::task::Context::from_waker(&waker);

    fut.poll(&mut cx)
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum Error {
    QueueFull,
    BufferLen,
}

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

impl<const LENGTH: usize> Buffer<LENGTH> {
    unsafe fn mut_ref<'a>(&'a self) -> &'a mut Self {
        &mut *(self as *const _ as *mut _)
    }
}

pub struct Queue<'b, const LENGTH: usize, const DEPTH: usize> {
    buffers: &'b mut [Buffer<LENGTH>; DEPTH],
    read_index: AtomicUsize,
    write_index: AtomicUsize,
    size: AtomicUsize,
    task_waker: AtomicWaker,
}

impl<'b, const LENGTH: usize, const DEPTH: usize> Queue<'b, LENGTH, DEPTH> {
    pub const fn new(buffers: &'b mut [Buffer<LENGTH>; DEPTH]) -> Self {
        Self {
            buffers,
            read_index: AtomicUsize::new(0),
            write_index: AtomicUsize::new(0),
            size: AtomicUsize::new(0),
            task_waker: AtomicWaker::new(),
        }
    }

    pub fn try_enqueue_copy(&self, source: &[u8]) -> Result<(), Error> {
        if source.len() > LENGTH {
            return Err(Error::BufferLen);
        }
        let mut f = self.enqueue_copy(source);
        match poll_fut(&mut f) {
            Poll::Pending => Err(Error::QueueFull),
            Poll::Ready(_) => Ok(()),
        }
    }

    pub async fn enqueue_copy(&self, source: &[u8]) -> Result<(), Error> {
        if source.len() > LENGTH {
            return Err(Error::BufferLen);
        }
        self.enqueue(async move |dest| {
            dest[..source.len()].copy_from_slice(source);
            source.len()
        }).await;

        Ok(())
    }

    pub async fn enqueue<'a, F: Future<Output=usize>>(&'a self, fn_read: impl FnOnce(&'a mut [u8]) -> F) {
        poll_fn(|cx| {
            if self.size.load(Ordering::Acquire) == DEPTH {
                self.task_waker.register(cx.waker());
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
        
        let write_index = self.write_index.load(Ordering::Acquire);

        let buf = unsafe { (self.buffers)[write_index].mut_ref() };
        let dest = unsafe { MaybeUninit::slice_assume_init_mut(&mut buf.data) };

        let len = fn_read(dest).await;

        buf.len.write(len);

        self.size.fetch_add(1, Ordering::SeqCst);
        self.write_index
            .store((write_index + 1) % DEPTH, Ordering::SeqCst);
        self.task_waker.wake();
    }

    pub async fn dequeue_copy(&self, dest: &mut [u8]) -> Result<usize, Error> {
        let len = self.dequeue(async move |source| {
            dest[..source.len()].copy_from_slice(source);
            source.len()
        }).await;

        Ok(len)
    }

    pub async fn dequeue<F: Future>(&'b self, fn_write: impl FnOnce(&'b [u8]) -> F) -> <F as Future>::Output {
        poll_fn(|cx| {
            if self.size.load(Ordering::Acquire) == 0 {
                self.task_waker.register(cx.waker());
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;

        let read_index = self.read_index.load(Ordering::Acquire);

        let source = unsafe {
            let buf = self.buffers[read_index].mut_ref();
            let len = buf.len.assume_init();

            &MaybeUninit::slice_assume_init_ref(&buf.data)[..len]
        };
        let ret = fn_write(source).await;

        self.size.fetch_sub(1, Ordering::SeqCst);
        self.read_index
            .store((read_index + 1) % DEPTH, Ordering::SeqCst);
        self.task_waker.wake();

        ret
    }
}
