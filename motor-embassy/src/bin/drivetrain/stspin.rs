pub struct WheelMotor<'a,
    UART: usart::BasicInstance,
    DmaRx: usart::RxDma<UART>,
    DmaTx: usart::TxDma<UART>,
    const LEN_RX: usize,
    const LEN_TX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    PIN: Pin,
> {
    stm32_uart_inteface: Stm32Interface<
        'a,
        UartReadQueue<'a, UART, DmaRx, LEN_RX, DEPTH_TX>,
        UartWriteQueue<'a, UART, DmaTx, LEN_TX, DEPTH_RX>,
        PIN,
        PIN
    >
}

impl<
        'a,
        UART: usart::BasicInstance,
        DmaRx: usart::RxDma<UART>,
        DmaTx: usart::TxDma<UART>,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        PIN: Pin,
    > Stspin<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_TX, DEPTH_RX, PIN>
{
    pub async fn new(        
        read_queue: &'a UartReadQueue<'a, UART, DmaRx, LEN_RX, DEPTH_TX>,
        write_queue: &'a UartWriteQueue<'a, UART, DmaTx, LEN_TX, DEPTH_RX>,
        boot0_pin: impl Peripheral<P = PIN> + 'a,
        reset_pin: impl Peripheral<P = PIN> + 'a) {
    }
}