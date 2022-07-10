use stm32h7xx_hal::serial;

/* 
 * This file is just to add traits with function wrappers for 
 * peripherals to use with generics
 * (this really should not be neccecary if stm32h7xx_hal added traits)
 */

pub trait Serial<USART> {
    fn split(self) -> (serial::Tx<USART>, serial::Rx<USART>);
    fn join(tx: serial::Tx<USART>, rx: serial::Rx<USART>) -> Self;
}
pub trait SerialTx<USART> {
    fn enable_dma_tx(&mut self);
}

macro_rules! test_macro {
    ($(
        $USARTX:ident,
    )+) => {
        $(
            type $USARTX = stm32h7xx_hal::stm32::$USARTX;
            impl Serial<$USARTX> for serial::Serial<$USARTX> {
                fn split(self) -> (serial::Tx<$USARTX>, serial::Rx<$USARTX>) {
                    self.split()
                }
                fn join(tx: serial::Tx<$USARTX>, rx: serial::Rx<$USARTX>) -> Self {
                    Self::join(tx, rx)
                }
            }
            impl SerialTx<$USARTX> for serial::Tx<$USARTX> {
                fn enable_dma_tx(&mut self) {
                    self.enable_dma_tx()
                }
            }            
        )+
    }
}


test_macro! {
    USART1,
    USART2,
    USART3,
    USART6,

    UART4,
    UART5,
    UART7,
    UART8,
}
