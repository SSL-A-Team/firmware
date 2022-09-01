#[allow(unused)]
use arr_macro::arr;

#[derive(PartialEq, Eq, Debug)]
pub struct IoBuffer<'buf_lt> {
    len: usize,
    io_buf: &'buf_lt mut [u8],
}

#[allow(dead_code)]
impl<'buf_lt> IoBuffer<'buf_lt> {
    fn init(buf: &'buf_lt mut [u8]) -> IoBuffer<'buf_lt> {
        IoBuffer { len: 0, io_buf: buf }
    }

    pub fn backing_len(&self) -> usize {
        return self.io_buf.len();
    }

    pub fn len(&self) -> usize {
        return self.len;
    }

    pub fn get_io_buf(&mut self) -> &mut [u8] {
        return self.io_buf;
    }
}

#[macro_export]
macro_rules! iobuf {
    ($len:expr) => {
        IoBuffer { len: 0, io_buf: & mut[0u8; $len] }
    }
}

#[macro_export]
macro_rules! ioqueue_storage {
    ($len:expr, $depth:expr) => {
        arr![iobuf!($len); $depth]
    };
}

#[macro_export]
macro_rules! axisram_ioqueue_storage {
    ($len:expr, $depth:expr) => {
        #[link_section = ".axisram.buffers"]
        ioqueue_storage($len, $depth)
    };
}

#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum IoQueueError {
    WriteBackingBufferTooSmall,
    WriteWhenFull,
    ReadDestBufferTooSmall,
    DropWhenEmpty,
    ReadWhenEmpty,
}

#[allow(dead_code)]
pub struct IoQueue<'buf_lt> {
    size: usize,
    read_ind: usize,
    write_ind: usize,
    // depricate, ask user to pass in static lifetime buffer linked correctly
    // #[link_section = ".axisram.buffers"]
    backing_store: &'buf_lt mut [IoBuffer<'buf_lt>],
}

#[allow(dead_code)]
impl<'buf_lt> IoQueue<'buf_lt> {
    fn init(backing_store: &'buf_lt mut [IoBuffer<'buf_lt>]) -> IoQueue<'buf_lt> {
        assert!(backing_store.len() > 0);
        for iob in backing_store.iter() {
            assert!(iob.io_buf.len() > 0);
        }

        IoQueue {
            size: 0,
            read_ind: 0,
            write_ind: 0,
            backing_store: backing_store,
        }
    }

    fn backing_store_capacity(&self) -> usize {
        return self.backing_store.len();
    }

    fn increment_read_ptr(&mut self) {
        self.read_ind = (self.read_ind + 1) % self.backing_store_capacity();

    }

    fn increment_write_ind(&mut self) {
        self.write_ind = (self.write_ind + 1) % self.backing_store_capacity();
    }

    pub fn depth(&self) -> usize {
        self.backing_store_capacity()
    }

    pub fn length(&self) -> usize {
        return self.backing_store[0].io_buf.len();
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

        if dat.len() > self.backing_store[self.write_ind].io_buf.len() {
            return Err(IoQueueError::WriteBackingBufferTooSmall);
        }

        self.backing_store[self.write_ind].io_buf[..dat.len()].copy_from_slice(dat);
        self.backing_store[self.write_ind].len = dat.len();

        self.increment_write_ind();
        self.size += 1;

        return Ok(());
    }

    pub fn peek(&self) -> Result<&[u8], IoQueueError> {
        if self.empty() {
            return Err(IoQueueError::ReadWhenEmpty);
        }

        let data_valid_index = self.backing_store[self.read_ind].len;
        let ret = &self.backing_store[self.read_ind].io_buf[..data_valid_index];
        return Ok(ret)
    }

    // this seems to invitably face lifetime issues 
    pub fn peek_top_buf(&mut self) -> &IoBuffer {
        let ret = &self.backing_store[self.read_ind];
        return ret
    }

    pub fn drop(&mut self) -> Result<(), IoQueueError> {
        if self.empty() {
            return Err(IoQueueError::DropWhenEmpty);
        }

        self.increment_read_ptr();
        self.size -= 1;

        return Ok(());
    }

    pub fn read<'a>(&mut self, dest: &'a mut [u8]) -> Result<&'a [u8], IoQueueError> {
        if self.empty() {
            return Err(IoQueueError::ReadWhenEmpty);
        }

        if dest.len() < self.backing_store[self.read_ind].len {
            return Err(IoQueueError::ReadDestBufferTooSmall);
        }

        let data_valid_index = self.backing_store[self.read_ind].len;
        dest[..data_valid_index].copy_from_slice(&self.backing_store[self.read_ind].io_buf[..data_valid_index]);
        let ret = &dest[..data_valid_index];

        self.increment_read_ptr();
        self.size -= 1;

        return Ok(ret);
    }

    // pub fn read_into(&mut self, dest: &mut [u8]) -> Result<usize, IoQueueError> {
    //     if self.empty() {
    //         return Err(IoQueueError::ReadWhenEmpty);
    //     }

    //     if dest.len() < self.backing_store[self.read_ind].len {
    //         return Err(IoQueueError::ReadDestBufferTooSmall);
    //     }

    //     let data_valid_index = self.backing_store[self.read_ind].len;
    //     dest[..data_valid_index].copy_from_slice(&self.backing_store[self.read_ind].io_buf[..data_valid_index]);

    //     self.increment_read_ptr();
    //     self.size -= 1;

    //     return Ok(data_valid_index);
    // }
}

/////////////
//  TESTS  //
/////////////

// ehh these tests are probably kinda bad, but I got confident enough in correctness I stopped writing more

// capcaity

#[test]
fn check_capacity_1() {
    let mut buf = ioqueue_storage!(16, 1);
    let q = IoQueue::init(&mut buf);
    assert_eq!(1, q.depth());
}

#[test]
fn check_capacity_2() {
    let mut buf = ioqueue_storage!(16, 2);
    let q = IoQueue::init(&mut buf);
    assert_eq!(2, q.depth());
}

#[test]
fn check_capacity_3() {
    let mut buf = ioqueue_storage!(16, 3);
    let q = IoQueue::init(&mut buf);
    assert_eq!(3, q.depth());
}

#[test]
fn check_capacity_10() {
    let mut buf = ioqueue_storage!(16, 10);
    let q = IoQueue::init(&mut buf);
    assert_eq!(10, q.depth());
}

// write tests

#[test]
fn write_capacity_1() {
    let mut buf = ioqueue_storage!(16, 1);
    let mut q = IoQueue::init(&mut buf);
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
    let mut buf = ioqueue_storage!(16, 2);
    let mut q = IoQueue::init(&mut buf);
    assert_eq!(2, q.depth());
    assert_eq!(true, q.empty());
    assert_eq!(false, q.full());
    assert_eq!(0, q.size());
    assert_eq!(2, q.size_remaining());

    let w1 = q.write(&[0xAAu8]);

    assert!(w1.is_ok());
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
    let mut buf = ioqueue_storage!(16, 3);
    let mut q = IoQueue::init(&mut buf);
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

