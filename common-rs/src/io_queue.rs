use core::{
    mem,
    cell::{
        RefCell, 
        Ref,
        RefMut, BorrowError, BorrowMutError}, ops::{Deref, DerefMut}};

use embedded_dma;
use cortex_m::interrupt::Mutex;

#[allow(unused)]
use arr_macro::arr;

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub struct IoBuffer<const SIZE: usize> {
    data_len: usize,
    backing_buf: [u8; SIZE],
}

////////////////
//  IoBuffer  //
////////////////

#[allow(dead_code)]
impl<const SIZE: usize> IoBuffer<SIZE> {
    pub const fn init() -> IoBuffer<SIZE> {
        IoBuffer { 
            data_len: 0,
            backing_buf: [0u8; SIZE]
        }
    }

    pub fn max_data_len(&self) -> usize {
        return self.backing_buf.len();
    }

    pub fn len(&self) -> usize {
        return self.data_len;
    }
}

unsafe impl<const SIZE: usize> embedded_dma::WriteTarget for IoBuffer<SIZE> {
    type Word = u8;

    fn as_write_buffer(&mut self) -> (*mut Self::Word, usize) {
        let len = self.max_data_len();
        let ptr = &mut self.backing_buf as *mut _ as *mut Self::Word;
        (ptr, len)
    }
}

impl<const SIZE: usize> core::ops::Index<usize> for IoBuffer<SIZE> {
    type Output = u8;
    fn index<'a>(&'a self, i: usize) -> &'a Self::Output {
        if i > self.len() {
            panic!("index out of range!")
        }

        &self.backing_buf[i]
    }
}

impl<const SIZE: usize> core::ops::IndexMut<usize> for IoBuffer<SIZE> {
    fn index_mut<'a>(&'a mut self, i: usize) -> &'a mut Self::Output {
        if i > self.len() {
            panic!("index out of range!")
        }

        &mut self.backing_buf[i]
    }
}

unsafe impl<const SIZE: usize> Sync for IoBuffer<SIZE> {}
unsafe impl<const SIZE: usize> Send for IoBuffer<SIZE> {}

#[macro_export]
macro_rules! iobuf {
    ($len:expr) => {
        IoBuffer::init()
    }
}

#[macro_export]
macro_rules! ioqueue_storage {
    ($len:expr, $depth:expr) => {
        arr![RefCell::new(iobuf!($len)); $depth]
    };
}

#[macro_export]
macro_rules! ioqueue_storage_var {
    ($name: ident, $len:expr, $depth:expr) => {
        let mut $name: [RefCell<IoBuffer<$len>>; $depth] = arr![RefCell::new(iobuf!($len)); $depth];
    };
}

#[macro_export]
macro_rules! axisram_ioqueue_storage_var {
    ($name: ident, $len:expr, $depth:expr) => {
        #[link_section = ".axisram.buffers"]
        static mut $name: [RefCell<IoBuffer<$len>>; $depth] = arr![RefCell::new(iobuf!($len)); $depth];
    };
}

#[macro_export]
macro_rules! axisram_bidirec_storage_vars {
    ($name: ident, $len:expr, $depth:expr) => {
        #[link_section = ".axisram.buffers"]
        static mut [<$name _rx_sto>]: [RefCell<IoBuffer<$len>>; $depth] = ioqueue_storage!($len, $depth);
        #[link_section = ".axisram.buffers"]
        static mut [<$name _tx_sto>]: [RefCell<IoBuffer<$len>>; $depth] = ioqueue_storage!($len, $depth);
    };
}

///////////////
//  IoQueue  //
///////////////

#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum IoQueueError {
    WriteBackingBufferTooSmall,
    WriteWhenFull,
    WriteDmaBufferInUse,
    RewriteWhenNotFull,
    ReadDestBufferTooSmall,
    DropWhenEmpty,
    ReadWhenEmpty,
    PeekBorrowError,
    ReadDmaBufferInUse,
}

#[allow(dead_code)]
pub struct IoQueue<const LENGTH: usize, const DEPTH: usize> {
    size: usize,
    read_ind: usize,
    write_ind: usize,
    // depricate, ask user to pass in static lifetime buffer linked correctly
    // #[link_section = ".axisram.buffers"]
    backing_store: [IoBuffer<LENGTH>; DEPTH],
}

#[allow(dead_code)]
impl<const LENGTH: usize, const DEPTH: usize> IoQueue<LENGTH, DEPTH> {
    pub const fn new() -> IoQueue<LENGTH, DEPTH> {
        // assert!(backing_store.len() > 0);
        // for iob in backing_store.iter() {
        //     assert!(iob.borrow().max_data_len() > 0);
        // }

        IoQueue {
            size: 0,
            read_ind: 0,
            write_ind: 0,
            backing_store: [IoBuffer { data_len: 0, backing_buf: [0u8; LENGTH] }; DEPTH],
        }
    }

    fn backing_store_capacity(&self) -> usize {
        return self.backing_store.len();
    }

    fn increment_read_ind(&mut self) {
        self.read_ind = (self.read_ind + 1) % self.backing_store_capacity();

    }

    fn increment_write_ind(&mut self) {
        self.write_ind = (self.write_ind + 1) % self.backing_store_capacity();
    }

    fn decrement_write_ind(&mut self) {
        if self.write_ind == 0 {
            self.write_ind = self.backing_store_capacity() -1;
        } else {
            self.write_ind -= 1;
        }
    }

    pub fn depth(&self) -> usize {
        self.backing_store_capacity()
    }

    pub fn length(&self) -> usize {
        return self.backing_store[0].max_data_len();
    }

    pub fn size(&self) -> usize {
        return self.size;
    }

    pub fn size_remaining(&self) -> usize {
        return self.backing_store_capacity() - self.size();
    }

    pub fn full(&self) -> bool {
        self.size_remaining() == 0
    }

    pub fn empty(&self) -> bool {
        self.size() == 0
    }

    pub fn write(&mut self, dat: &[u8]) -> Result<(), IoQueueError> {
        if self.full() {
            return Err(IoQueueError::WriteWhenFull);
        }

        let mut dma_buf = &mut self.backing_store[self.write_ind];
        if dat.len() > dma_buf.max_data_len() {
            return Err(IoQueueError::WriteBackingBufferTooSmall);
        }

        dma_buf.backing_buf[..dat.len()].copy_from_slice(dat);
        dma_buf.data_len = dat.len();

        self.increment_write_ind();
        self.size += 1;

        return Ok(());
    }

    pub fn rewrite(&mut self, dat: &[u8]) -> Result<(), IoQueueError> {
        unimplemented!();
    }

    pub fn peek_write(&mut self) -> Result<&mut IoBuffer<LENGTH>, IoQueueError> {
        if self.full() {
            return Err(IoQueueError::WriteWhenFull);
        }

        return Ok(&mut self.backing_store[self.write_ind]);
    }

    // pub fn peek_rewrite(&mut self) -> Result<&mut IoBuffer<LENGTH>, IoQueueError> {
    //     if !self.full() {
    //         return Err(IoQueueError::RewriteWhenNotFull);
    //     }

    //     // TODO fix this logic
    //     let rewrite_buf = &mut self.backing_store[self.write_ind];
    //     // reclaim the back buffer
    //     self.decrement_write_ind();
    //     self.size -= 1;

    //     return Ok(rewrite_buf);
    // }

    pub fn finalize_wpeek(&mut self, dma_buf: &mut IoBuffer<LENGTH>) -> Result<(), IoQueueError> {
        // TODO check some stuff like the returned buf is identical to the given buf

        // drop ref mut, returning reference to the original RefCell, allowing new reference to read
        drop(dma_buf);

        // that read will now be outwardly visible by updating the size/ptr
        self.increment_write_ind();
        self.size += 1;

        return Ok(());
    }

    pub fn read<'a>(&mut self, dest: &'a mut [u8]) -> Result<&'a [u8], IoQueueError> {
        if self.empty() {
            return Err(IoQueueError::ReadWhenEmpty);
        }

        let dma_buf = self.backing_store[self.read_ind];
        if dest.len() < dma_buf.len() {
            return Err(IoQueueError::ReadDestBufferTooSmall);
        }

        let data_valid_index = dma_buf.len();
        dest[..data_valid_index].copy_from_slice(&dma_buf.backing_buf[..data_valid_index]);
        let ret = &dest[..data_valid_index];

        self.increment_read_ind();
        self.size -= 1;

        return Ok(ret);
    }

    pub fn peek(&self) -> Result<&IoBuffer<LENGTH>, IoQueueError> {
        if self.empty() {
            return Err(IoQueueError::ReadWhenEmpty);
        }

        return  Ok(&self.backing_store[self.read_ind]);
    }

    pub fn peek_mut(&mut self) -> Result<&mut IoBuffer<LENGTH>, IoQueueError> {
        if self.empty() {
            return Err(IoQueueError::ReadWhenEmpty);
        }

        //let p = (&mut self.backing_store[self.read_ind]) as *mut IoBuffer<LENGTH>;
        //let mr: &mut IoBuffer<LENGTH> = & mut (unsafe { *((&mut self.backing_store[self.read_ind]) as *mut IoBuffer<LENGTH>) });
        return Ok(unsafe { &mut *((&mut self.backing_store[self.read_ind]) as *mut IoBuffer<LENGTH>) });
    }

    pub fn finalize_rpeek(&mut self, dma_buf: &mut IoBuffer<LENGTH>) -> Result<(), IoQueueError> {
        // TODO check some stuff like the returned buf is identical to the given buf

        // drop ref mut, returning reference to the original RefCell, allowing new reference to read
        // this is implicit at the end of the function scope, but we're being explicit so users udnerstand the flow
        drop(dma_buf);

        // that read will now be outwardly visible by updating the size/ptr
        self.increment_read_ind();
        self.size -= 1;

        return Ok(());
    }

    pub fn drop(&mut self) -> Result<(), IoQueueError> {
        if self.empty() {
            return Err(IoQueueError::DropWhenEmpty);
        }

        self.increment_read_ind();
        self.size -= 1;

        return Ok(());
    }
}

unsafe impl<const LENGTH: usize, const DEPTH: usize> Sync for IoQueue<LENGTH, DEPTH> {}
unsafe impl<const LENGTH: usize, const DEPTH: usize> Send for IoQueue<LENGTH, DEPTH> {}

/////////////
//  TESTS  //
/////////////

// ehh these tests are probably kinda bad, but I got confident enough in correctness I stopped writing more

// capcaity

#[test]
fn check_capacity_1() {
    let q: IoQueue<16, 1> = IoQueue::new();
    assert_eq!(1, q.depth());
}

#[test]
fn check_capacity_2() {
    let q: IoQueue<16, 2> = IoQueue::new();
    assert_eq!(2, q.depth());
}

#[test]
fn check_capacity_3() {
    ioqueue_storage_var!(buf, 16, 3);
    let q: IoQueue<16, 3> = IoQueue::new();
    assert_eq!(3, q.depth());
}

#[test]
fn check_capacity_10() {
    ioqueue_storage_var!(buf, 16, 10);
    let q: IoQueue<16, 10> = IoQueue::new();
    assert_eq!(10, q.depth());
}

// write tests

#[test]
fn write_capacity_1() {
    let mut q: IoQueue<16, 1> = IoQueue::new();
    assert_eq!(1, q.depth());
    assert_eq!(true, q.empty());
    assert_eq!(false, q.full());
    assert_eq!(0, q.size());
    assert_eq!(1, q.size_remaining());

    let w1 = q.write(&[0xAAu8]);

    assert!(w1.is_ok());
    assert_eq!(false, q.empty());
    assert_eq!(true, q.full());
    assert_eq!(1, q.size());
    assert_eq!(0, q.size_remaining());

    let w2 = q.write(&[0x55u8]);

    assert!(w2.is_err());
    assert_eq!(false, q.empty());
    assert_eq!(true, q.full());
    assert_eq!(1, q.size());
    assert_eq!(0, q.size_remaining());
}

#[test]
fn write_capacity_2() {
    let mut q: IoQueue<16, 2> = IoQueue::new();
    assert_eq!(2, q.depth());
    assert_eq!(true, q.empty());
    assert_eq!(false, q.full());
    assert_eq!(0, q.size());
    assert_eq!(2, q.size_remaining());

    let w1 = q.write(&[0xAAu8]);

    assert_eq!(w1, Ok(()));
    assert_eq!(false, q.empty());
    assert_eq!(false, q.full());
    assert_eq!(1, q.size());
    assert_eq!(1, q.size_remaining());

    let p1 = q.peek();
    assert!(p1.is_ok());
    let p1 = p1.unwrap();
    assert_eq!(0xAA, p1[0]);
    assert_eq!(false, q.empty());
    assert_eq!(false, q.full());
    assert_eq!(1, q.size());
    assert_eq!(1, q.size_remaining());
    drop(p1);

    let w2 = q.write(&[0x55u8]);

    assert!(w2.is_ok());
    assert_eq!(false, q.empty());
    assert_eq!(true, q.full());
    assert_eq!(2, q.size());
    assert_eq!(0, q.size_remaining());

    let p2 = q.peek();
    assert!(p2.is_ok());
    let p2 = p2.unwrap();
    assert_eq!(0xAA, p2[0]);
    assert_eq!(false, q.empty());
    assert_eq!(true, q.full());
    assert_eq!(2, q.size());
    assert_eq!(0, q.size_remaining());
    drop(p2);

    let w3 = q.write(&[0x33u8]);

    assert!(w3.is_err());
    assert_eq!(false, q.empty());
    assert_eq!(true, q.full());
    assert_eq!(2, q.size());
    assert_eq!(0, q.size_remaining());

    let p3 = q.peek();
    assert!(p3.is_ok());
    let p3 = p3.unwrap();
    assert_eq!(0xAA, p3[0]);
    assert_eq!(false, q.empty());
    assert_eq!(true, q.full());
    assert_eq!(2, q.size());
    assert_eq!(0, q.size_remaining());
    drop(p3);
}

#[test]
fn write_capacity_3() {
    let mut q: IoQueue<16, 3> = IoQueue::new();
    assert_eq!(3, q.depth());
    assert_eq!(true, q.empty());
    assert_eq!(false, q.full());
    assert_eq!(0, q.size());
    assert_eq!(3, q.size_remaining());

    let w1 = q.write(&[0xAAu8, 0xFFu8, 0xEEu8, 0xBBu8]);

    assert!(w1.is_ok());
    assert_eq!(false, q.empty());
    assert_eq!(false, q.full());
    assert_eq!(1, q.size());
    assert_eq!(2, q.size_remaining());

    let w2 = q.write(&[0x55u8]);

    assert!(w2.is_ok());
    assert_eq!(false, q.empty());
    assert_eq!(false, q.full());
    assert_eq!(2, q.size());
    assert_eq!(1, q.size_remaining());

    let w3 = q.write(&[0x33u8]);

    assert!(w3.is_ok());
    assert_eq!(false, q.empty());
    assert_eq!(true, q.full());
    assert_eq!(3, q.size());
    assert_eq!(0, q.size_remaining());

    let w4 = q.write(&[0xCCu8]);

    assert!(w4.is_err());
    assert_eq!(false, q.empty());
    assert_eq!(true, q.full());
    assert_eq!(3, q.size());
    assert_eq!(0, q.size_remaining());

    let mut rbuf1 = [0; 2];
    let r1 = q.read(&mut rbuf1);
    
    assert!(r1.is_err());
    assert!(r1.unwrap_err() == IoQueueError::ReadDestBufferTooSmall);

    let mut rbuf2 = [0; 6];
    let r2 = q.read(&mut rbuf2);

    assert!(r2.is_ok());
    let r2 = r2.unwrap();
    // [0xAAu8, 0xFFu8, 0xEEu8, 0xBBu8]
    assert_eq!(4, r2.len());
    assert_eq!(0xAA, r2[0]);
    assert_eq!(0xFF, r2[1]);
    assert_eq!(0xEE, r2[2]);
    assert_eq!(0xBB, r2[3]);
    assert_eq!(false, q.empty());
    assert_eq!(false, q.full());
    assert_eq!(2, q.size());
    assert_eq!(1, q.size_remaining());

    let d1 = q.drop();

    assert!(d1.is_ok());
    assert_eq!(1, q.size());

    let d2 = q.drop();

    assert!(d2.is_ok());
    assert!(q.empty());

    let d3 = q.drop();

    assert!(d3.is_err());

    _ = q.write(&[0x01]);
    _ = q.write(&[0x02]);

    assert_eq!(2, q.size());

    assert_eq!(0x01, q.peek().unwrap()[0]);
}
