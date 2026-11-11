//! Reassembles discrete frames from a byte stream whose chunk boundaries are not
//! guaranteed to match frame boundaries (e.g. a UART in transparent/pass-through mode).
//!
//! Frame layout is protocol specific, so the caller supplies a check function that
//! inspects the head of the buffered stream and reports whether a full frame is present.

/// Result of inspecting the head of the buffered stream.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FrameCheck {
    /// The bytes so far are a plausible frame prefix; wait for more data.
    NeedMore,
    /// The head of the buffer is not a valid frame start; drop a byte and rescan.
    Invalid,
    /// A complete frame of the given length is at the head of the buffer.
    Complete(usize),
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DeframerError {
    /// Pushed data did not fit; buffered data was discarded.
    Overflow,
}

pub struct StreamDeframer<const N: usize> {
    buf: [u8; N],
    len: usize,
    resync_count: u32,
    overflow_count: u32,
}

impl<const N: usize> Default for StreamDeframer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> StreamDeframer<N> {
    pub const CAPACITY: usize = N;

    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            len: 0,
            resync_count: 0,
            overflow_count: 0,
        }
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Number of bytes dropped while searching for a valid frame start.
    pub fn resync_count(&self) -> u32 {
        self.resync_count
    }

    pub fn overflow_count(&self) -> u32 {
        self.overflow_count
    }

    /// Append stream bytes. If they do not fit, all buffered data is discarded (a partial
    /// frame that cannot complete is useless) and the new data is buffered if it fits on its own.
    pub fn push(&mut self, data: &[u8]) -> Result<(), DeframerError> {
        if self.len + data.len() <= N {
            self.buf[self.len..self.len + data.len()].copy_from_slice(data);
            self.len += data.len();
            return Ok(());
        }

        self.overflow_count = self.overflow_count.wrapping_add(1);
        self.len = 0;
        if data.len() <= N {
            self.buf[..data.len()].copy_from_slice(data);
            self.len = data.len();
        }
        Err(DeframerError::Overflow)
    }

    /// Extract the next complete frame, if any, passing it to `fn_read`.
    /// Invalid leading bytes are dropped one at a time until `check` accepts the head.
    pub fn next_frame<RET>(
        &mut self,
        mut check: impl FnMut(&[u8]) -> FrameCheck,
        fn_read: impl FnOnce(&[u8]) -> RET,
    ) -> Option<RET> {
        while self.len > 0 {
            match check(&self.buf[..self.len]) {
                FrameCheck::NeedMore => return None,
                FrameCheck::Complete(n) if n > self.len => return None,
                FrameCheck::Complete(n) if n > 0 => {
                    let ret = fn_read(&self.buf[..n]);
                    self.consume(n);
                    return Some(ret);
                }
                FrameCheck::Complete(_) | FrameCheck::Invalid => {
                    self.resync_count = self.resync_count.wrapping_add(1);
                    self.consume(1);
                }
            }
        }
        None
    }

    fn consume(&mut self, n: usize) {
        self.buf.copy_within(n..self.len, 0);
        self.len -= n;
    }
}
