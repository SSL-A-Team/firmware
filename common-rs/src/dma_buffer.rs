// #[allow(unused)]
// use arr_macro::arr;

// #[derive(PartialEq, Eq, Debug)]
// pub struct DmaBuffer<const SIZE: usize> {
//     len: RefCell<usize>,
//     io_buf: RefCell<[u8; SIZE]>,
// }

// #[allow(dead_code)]
// impl<const SIZE: usize> DmaBuffer<SIZE> {
//     pub fn backing_len(&self) -> usize {
//         return self.io_buf.borrow().len();
//     }

//     pub fn len(&self) -> usize {
//         return *self.len.borrow();
//     }

//     pub fn try_backing_buf(&self) -> Result<Ref<[u8; SIZE]>, BorrowError> {
//         return self.io_buf.try_borrow();
//     }

//     pub fn try_backing_buf_mut(&self) -> Result<RefMut<[u8; SIZE]>, BorrowMutError> {
//         return self.io_buf.try_borrow_mut();
//     }

//     pub fn get_backing_buf(&self) -> Ref<[u8; SIZE]> {
//         return self.io_buf.borrow();
//     }

//     pub fn get_backing_buf_mut(&self) -> RefMut<[u8; SIZE]> {
//         return self.io_buf.borrow_mut();
//     }

//     pub fn get_backing_buf_refcell(&self) -> &RefCell<[u8; SIZE]> {
//         return &self.io_buf;
//     }
// }