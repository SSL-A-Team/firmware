use ateam_lib_stm32::make_uart_queue_pair;
use embassy_executor::{SendSpawner, Spawner};

use crate::pins::*;

const MAX_TX_PACKET_SIZE: usize = 64;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 64;
const RX_BUF_DEPTH: usize = 20;

make_uart_queue_pair!(FRONT_LEFT,
    MotorFLUart, MotorFLDmaRx, MotorFLDmaTx,
    MAX_RX_PACKET_SIZE, RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE, TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);

make_uart_queue_pair!(BACK_LEFT,
    MotorBLUart, MotorBLDmaRx, MotorBLDmaTx,
    MAX_RX_PACKET_SIZE, RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE, TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);

make_uart_queue_pair!(BACK_RIGHT,
    MotorBRUart, MotorBRDmaRx, MotorBRDmaTx,
    MAX_RX_PACKET_SIZE, RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE, TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);

make_uart_queue_pair!(FRONT_RIGHT,
    MotorFRUart, MotorFRDmaRx, MotorFRDmaTx,
    MAX_RX_PACKET_SIZE, RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE, TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);

make_uart_queue_pair!(DRIB,
    MotorDUart, MotorDDmaRx, MotorDDmaTx,
    MAX_RX_PACKET_SIZE, RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE, TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);


#[embassy_executor::task]
async fn control_task_entry() {

}

pub async fn start_control_task(
    uart_queue_spawner: SendSpawner,
    control_task_spawner: Spawner,
    motor_fl_uart: MotorFLUart, motor_fl_rx_pin: MotorFLUartRxPin, motor_fl_tx_pin: MotorFLUartTxPin, motor_fl_rx_dma: MotorFLDmaRx, motor_fl_tx_dma: MotorFLDmaTx, motor_fl_boot0_pin: MotorFLBootPin, motor_fl_nrst_pin: MotorFLResetPin,
    motor_bl_uart: MotorFLUart, motor_bl_rx_pin: MotorFLUartRxPin, motor_bl_tx_pin: MotorFLUartTxPin, motor_bl_rx_dma: MotorFLDmaRx, motor_bl_tx_dma: MotorFLDmaTx, motor_bl_boot0_pin: MotorFLBootPin, motor_bl_nrst_pin: MotorFLResetPin,
    motor_br_uart: MotorFLUart, motor_br_rx_pin: MotorFLUartRxPin, motor_br_tx_pin: MotorFLUartTxPin, motor_br_rx_dma: MotorFLDmaRx, motor_br_tx_dma: MotorFLDmaTx, motor_br_boot0_pin: MotorFLBootPin, motor_br_nrst_pin: MotorFLResetPin,
    motor_fr_uart: MotorFLUart, motor_fr_rx_pin: MotorFLUartRxPin, motor_fr_tx_pin: MotorFLUartTxPin, motor_fr_rx_dma: MotorFLDmaRx, motor_fr_tx_dma: MotorFLDmaTx, motor_fr_boot0_pin: MotorFLBootPin, motor_fr_nrst_pin: MotorFLResetPin,
    motor_d_uart: MotorFLUart,  motor_d_rx_pin: MotorFLUartRxPin,  motor_d_tx_pin: MotorFLUartTxPin,  motor_d_rx_dma: MotorFLDmaRx,  motor_d_tx_dma: MotorFLDmaTx,  motor_d_boot0_pin: MotorFLBootPin,  motor_d_nrst_pin: MotorFLResetPin,

) {



}