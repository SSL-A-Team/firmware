// #![feature(type_alias_impl_trait)]

//     use embassy_executor::Executor;
//     use embassy_futures::block_on;
//     use embassy_time::{Timer, Duration};

//     unsafe fn __make_static<T>(t: &mut T) -> &'static mut T {
//         ::core::mem::transmute(t)
//     }

//     #[embassy_executor::task]
//     async fn task1() {
//         // block_on(test1());
//         Timer::after(Duration::from_millis(1000)).await;
//         assert_eq!(2, 4);
//     }

//     async fn test1() {
//         Timer::after(Duration::from_millis(1000)).await;
//         assert_eq!(4, 4);
//     }

//     #[test]
//     fn it_works2() {
//         let mut executor = Executor::new();
//         let executor = unsafe { __make_static(&mut executor) };

//         // static RADIO_TASK: TaskStorage<RadioMock> = TaskStorage::new();

//         executor.run(|spawner| {
//             spawner.spawn(task1()).unwrap();
//         });


//         // let result = add(2, 2);
//         // assert_eq!(result, 4);
//     }

use ateam_common::queue::{Buffer, Queue};
use embassy_futures::block_on;

#[test]
fn queue_test() {
    block_on(test1());
}

async fn test1() {
    let mut buffers = [Buffer::<20>::EMPTY; 10];
    let queue = Queue::new(&mut buffers);

    {
        let mut buf = queue.enqueue().await.unwrap();
        let data = buf.data();
        data[0] = 123;
        let len = buf.len();
        *len = 1;
    }
    {
        let mut buf = queue.enqueue().await.unwrap();
        let data = buf.data();
        data[0] = 234;
        let len = buf.len();
        *len = 1;
    }

    {
        let buf = queue.dequeue().await.unwrap();
        let data = buf.data();
        assert_eq!(data.len(), 1);
        assert_eq!(data[0], 123);
    }
    {
        let buf = queue.dequeue().await.unwrap();
        let data = buf.data();
        assert_eq!(data.len(), 1);
        assert_eq!(data[0], 234);
    }
}