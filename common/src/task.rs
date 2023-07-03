use embassy_executor::SpawnToken;
use futures_util::Future;

type TaskFuture<T: Task + 'static> = impl Future + 'static;

pub trait Task<Data = ()> {
    type Data = Data;

    async fn task(data: Self::Data);
}

pub struct TaskStorage<T: Task + 'static> {
    task: embassy_executor::raw::TaskStorage<TaskFuture<T>>,
}

impl<T: Task> TaskStorage<T> {
    pub const fn new() -> Self {
        Self {
            task: embassy_executor::raw::TaskStorage::new(),
        }
    }

    pub fn spawn(&'static self, data: T::Data) -> SpawnToken<impl Sized> {
        self.task.spawn(|| T::task(data))
    }
}
