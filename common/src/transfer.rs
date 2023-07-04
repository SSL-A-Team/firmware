pub trait DataSourceRead {
    fn finish_data(&self);
    fn cancel_data(&self);
}

// pub struct DataRefRead<'a, Source: DataSourceRead> {
//     source: &'a Source,
//     data: &'a [u8],
// }

// impl<'a, Source: DataSourceRead> DataRefRead<'a, Source> {
//     pub fn data(&self) -> &[u8] {
//         self.data
//     }
//     pub fn cancel(self) {
//         self.source.cancel_data();
//     }
// }

// impl<'a, Source: DataSourceRead> Drop for DataRefRead<'a, Source> {
//     fn drop(&mut self) {
//         self.source.finish_data();
//     }
// }

// pub trait DataSourceWrite {
//     fn finish_data(&self);
//     fn cancel_data(&self);
// }

// pub struct DataRefWrite<'a, Source: DataSourceWrite> {
//     source: &'a Source,
//     data: &'a mut [u8],
//     len: &'a mut usize,
// }

// impl<'a, Source: DataSourceWrite> DataRefWrite<'a, Source> {
//     pub fn data(&mut self) -> &mut [u8] {
//         self.data
//     }

//     pub fn len(&mut self) -> &mut usize {
//         self.len
//     }
//     pub fn cancel(self) {
//         self.source.cancel_data();
//     }
// }

// impl<'a, Source: DataSourceWrite> Drop for DataRefWrite<'a, Source> {
//     fn drop(&mut self) {
//         self.source.finish_data();
//     }
// }

pub trait DataRefReadTrait {
    fn data(&self) -> &[u8];
    fn cancel(self);
}

pub trait DataRefWriteTrait {
    fn data(&mut self) ->  &mut [u8];
    fn len(&mut self) -> &mut usize;
    fn cancel(self);
}

pub trait Reader2 {
    type DataRefRead: DataRefReadTrait;

    async fn read(&self) -> Result<Self::DataRefRead, ()>;
}

pub trait Writer2 {
    type DataRefWrite: DataRefWriteTrait;

    async fn write(&self) -> Result<Self::DataRefWrite, ()>;
}



pub trait Reader {
    async fn read<RET, FN: FnOnce(&[u8]) -> Result<RET, ()>>(&self, fn_read: FN)
        -> Result<RET, ()>;
}

pub trait Writer {
    async fn write<FN: FnOnce(&mut [u8]) -> Result<usize, ()>>(
        &self,
        fn_write: FN,
    ) -> Result<(), ()>;
}
