
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
pub struct IoQueue<'buf_lt, const LENGTH: usize, const DEPTH: usize> {
    size: usize,
    read_ind: usize,
    write_ind: usize,
    // depricate, ask user to pass in static lifetime buffer linked correctly
    // #[link_section = ".axisram.buffers"]
    buf: &'buf_lt mut [[u8; LENGTH]; DEPTH],
    buf_size_record: [usize; DEPTH],
}

#[allow(dead_code)]
impl<'buf_lt, const LENGTH: usize, const DEPTH: usize> IoQueue<'buf_lt, LENGTH, DEPTH> {
    fn init(buf: &mut [[u8; LENGTH]; DEPTH]) -> IoQueue<LENGTH, DEPTH> {
        IoQueue {
            size: 0,
            read_ind: 0,
            write_ind: 0,
            buf: buf,
            buf_size_record: [0; DEPTH],
        }
    }

    fn increment_read_ptr(&mut self) {
        self.read_ind = (self.read_ind + 1) % DEPTH;

    }

    fn increment_write_ind(&mut self) {
        self.write_ind = (self.write_ind + 1) % DEPTH;
    }

    pub fn capcaity(&self) -> usize {
        DEPTH
    }

    pub fn size(&self) -> usize {
        return self.size;
    }

    pub fn size_remaining(&self) -> usize {
        return DEPTH - self.size();
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

        if dat.len() > LENGTH {
            return Err(IoQueueError::WriteBackingBufferTooSmall);
        }

        self.buf[self.write_ind][..dat.len()].copy_from_slice(dat);
        self.buf_size_record[self.write_ind] = dat.len();

        self.increment_write_ind();
        self.size += 1;

        return Ok(());
    }

    pub fn peek(&self) -> Result<&[u8], IoQueueError> {
        if self.empty() {
            return Err(IoQueueError::ReadWhenEmpty);
        }

        let data_valid_index = self.buf_size_record[self.read_ind];
        let ret = &self.buf[self.read_ind][..data_valid_index];
        return Ok(ret)
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

        if dest.len() < self.buf_size_record[self.read_ind] {
            return Err(IoQueueError::ReadDestBufferTooSmall);
        }

        let data_valid_index = self.buf_size_record[self.read_ind];
        dest[..data_valid_index].copy_from_slice(&self.buf[self.read_ind][..data_valid_index]);
        let ret = &dest[..data_valid_index];

        self.increment_read_ptr();
        self.size -= 1;

        return Ok(ret);
    }
}

/////////////
//  TESTS  //
/////////////

// ehh these tests are probably kinda bad, but I got confident enough in correctness I stopped writing more

// capcaity

#[test]
fn check_capacity_1() {
    let mut buf = [[0u8; 16]; 1];
    let q = IoQueue::init(&mut buf);
    assert_eq!(1, q.capcaity());
}

#[test]
fn check_capacity_2() {
    let mut buf = [[0u8; 16]; 2];
    let q = IoQueue::init(&mut buf);
    assert_eq!(2, q.capcaity());
}

#[test]
fn check_capacity_3() {
    let mut buf = [[0u8; 16]; 3];
    let q = IoQueue::init(&mut buf);
    assert_eq!(3, q.capcaity());
}

#[test]
fn check_capacity_10() {
    let mut buf = [[0u8; 16]; 10];
    let q = IoQueue::init(&mut buf);
    assert_eq!(10, q.capcaity());
}

// write tests

#[test]
fn write_capacity_1() {
    let mut buf = [[0u8; 16]; 1];
    let mut q = IoQueue::init(&mut buf);
    assert_eq!(1, q.capcaity());
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
    let mut buf = [[0u8; 16]; 2];
    let mut q = IoQueue::init(&mut buf);
    assert_eq!(2, q.capcaity());
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
    let mut buf = [[0u8; 16]; 3];
    let mut q = IoQueue::init(&mut buf);
    assert_eq!(3, q.capcaity());
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

