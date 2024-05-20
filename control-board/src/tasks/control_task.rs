use ateam_lib_stm32::{make_uart_queue_pair, queue_pair_register_and_spawn};
use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::usart::Uart;
use embassy_time::Timer;

use crate::{include_external_cpp_bin, pins::*, robot_state::{self, RobotState}, stm32_interface::{self, Stm32Interface}, stspin_motor::{DribblerMotor, WheelMotor}, SystemIrqs};

include_external_cpp_bin! {WHEEL_FW_IMG, "wheel.bin"}
include_external_cpp_bin! {DRIB_FW_IMG, "dribbler.bin"}

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
async fn control_task_entry(
    robot_state: RobotState,
    command_subscriber: CommandsSubscriber,
    telemetry_publisher: TelemetryPublisher,
    mut motor_fl: WheelMotor<'static, MotorFLUart, MotorFLDmaRx, MotorFLDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
    mut motor_bl: WheelMotor<'static, MotorBLUart, MotorBLDmaRx, MotorBLDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
    mut motor_br: WheelMotor<'static, MotorBRUart, MotorBRDmaRx, MotorBRDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
    mut motor_fr: WheelMotor<'static, MotorFRUart, MotorFRDmaRx, MotorFRDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
    mut motor_drib: DribblerMotor<'static, MotorDUart, MotorDDmaRx, MotorDDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH> 
) {
    defmt::info!("control task init.");

    // wait for the switch state to be read
    while !robot_state.hw_init_state_valid() {
        Timer::after_millis(10).await;
    }

    if robot_state.hw_in_debug_mode() {
        defmt::info!("flashing firmware");

        motor_fl.load_default_firmware_image().await;
        defmt::info!("FL flashed");

        motor_bl.load_default_firmware_image().await;
        defmt::info!("BL flashed");

        motor_br.load_default_firmware_image().await;
        defmt::info!("BR flashed");

        motor_fr.load_default_firmware_image().await;
        defmt::info!("FR flashed");

        motor_drib.load_default_firmware_image().await;
        defmt::info!("DRIB flashed");
    } else {
        let _res = embassy_futures::join::join5(
            motor_fl.load_default_firmware_image(),
            motor_bl.load_default_firmware_image(),
            motor_br.load_default_firmware_image(),
            motor_fr.load_default_firmware_image(),
            motor_drib.load_default_firmware_image(),
        )
        .await;

        defmt::debug!("motor firmware flashed");
    }

    embassy_futures::join::join5(
        motor_fl.leave_reset(),
        motor_bl.leave_reset(),
        motor_br.leave_reset(),
        motor_fr.leave_reset(),
        motor_drib.leave_reset(),
    )
    .await;

    loop {
        Timer::after_millis(1000);
        defmt::info!("motor firmware flashed");
    }
}

pub async fn start_control_task(
    uart_queue_spawner: SendSpawner,
    control_task_spawner: Spawner,
    robot_state: RobotState,
    command_subscriber: CommandsSubscriber,
    telemetry_publisher: TelemetryPublisher,
    motor_fl_uart: MotorFLUart, motor_fl_rx_pin: MotorFLUartRxPin, motor_fl_tx_pin: MotorFLUartTxPin, motor_fl_rx_dma: MotorFLDmaRx, motor_fl_tx_dma: MotorFLDmaTx, motor_fl_boot0_pin: MotorFLBootPin, motor_fl_nrst_pin: MotorFLResetPin,
    motor_bl_uart: MotorBLUart, motor_bl_rx_pin: MotorBLUartRxPin, motor_bl_tx_pin: MotorBLUartTxPin, motor_bl_rx_dma: MotorBLDmaRx, motor_bl_tx_dma: MotorBLDmaTx, motor_bl_boot0_pin: MotorBLBootPin, motor_bl_nrst_pin: MotorBLResetPin,
    motor_br_uart: MotorBRUart, motor_br_rx_pin: MotorBRUartRxPin, motor_br_tx_pin: MotorBRUartTxPin, motor_br_rx_dma: MotorBRDmaRx, motor_br_tx_dma: MotorBRDmaTx, motor_br_boot0_pin: MotorBRBootPin, motor_br_nrst_pin: MotorBRResetPin,
    motor_fr_uart: MotorFRUart, motor_fr_rx_pin: MotorFRUartRxPin, motor_fr_tx_pin: MotorFRUartTxPin, motor_fr_rx_dma: MotorFRDmaRx, motor_fr_tx_dma: MotorFRDmaTx, motor_fr_boot0_pin: MotorFRBootPin, motor_fr_nrst_pin: MotorFRResetPin,
    motor_d_uart: MotorDUart,   motor_d_rx_pin: MotorDUartRxPin,   motor_d_tx_pin: MotorDUartTxPin,   motor_d_rx_dma: MotorDDmaRx,   motor_d_tx_dma: MotorDDmaTx,  motor_d_boot0_pin: MotorDBootPin,  motor_d_nrst_pin: MotorDResetPin,

) {
    let initial_motor_controller_uart_conifg = stm32_interface::get_bootloader_uart_config();

    //////////////////////////
    //  create motor uarts  //
    //////////////////////////

    let fl_uart = Uart::new(motor_fl_uart, motor_fl_rx_pin, motor_fl_tx_pin, SystemIrqs, motor_fl_tx_dma, motor_fl_rx_dma, initial_motor_controller_uart_conifg).unwrap();
    let bl_uart = Uart::new(motor_bl_uart, motor_bl_rx_pin, motor_bl_tx_pin, SystemIrqs, motor_bl_tx_dma, motor_bl_rx_dma, initial_motor_controller_uart_conifg).unwrap();
    let br_uart = Uart::new(motor_br_uart, motor_br_rx_pin, motor_br_tx_pin, SystemIrqs, motor_br_tx_dma, motor_br_rx_dma, initial_motor_controller_uart_conifg).unwrap();
    let fr_uart = Uart::new(motor_fr_uart, motor_fr_rx_pin, motor_fr_tx_pin, SystemIrqs, motor_fr_tx_dma, motor_fr_rx_dma, initial_motor_controller_uart_conifg).unwrap();
    let drib_uart = Uart::new(motor_d_uart, motor_d_rx_pin, motor_d_tx_pin, SystemIrqs, motor_d_tx_dma, motor_d_rx_dma, initial_motor_controller_uart_conifg).unwrap();

    //////////////////////////////////////////////
    //  register motor queues and DMA hardware  //
    //////////////////////////////////////////////

    let (fl_uart_tx, fl_uart_rx) = Uart::split(fl_uart);
    queue_pair_register_and_spawn!(uart_queue_spawner, FRONT_LEFT, fl_uart_rx, fl_uart_tx);
    let (bl_uart_tx, bl_uart_rx) = Uart::split(bl_uart);
    queue_pair_register_and_spawn!(uart_queue_spawner, BACK_LEFT, bl_uart_rx, bl_uart_tx);
    let (br_uart_tx, br_uart_rx) = Uart::split(br_uart);
    queue_pair_register_and_spawn!(uart_queue_spawner, BACK_RIGHT, br_uart_rx, br_uart_tx);
    let (fr_uart_tx, fr_uart_rx) = Uart::split(fr_uart);
    queue_pair_register_and_spawn!(uart_queue_spawner, FRONT_RIGHT, fr_uart_rx, fr_uart_tx);

    let (drib_uart_tx, drib_uart_rx) = Uart::split(drib_uart);
    queue_pair_register_and_spawn!(uart_queue_spawner, DRIB, drib_uart_rx, drib_uart_tx);

    ////////////////////////////////
    //  create motor controllers  //
    ////////////////////////////////
    
    let motor_fl = WheelMotor::new_from_pins(&FRONT_LEFT_RX_UART_QUEUE,  &FRONT_LEFT_TX_UART_QUEUE,  motor_fl_boot0_pin, motor_fl_nrst_pin, WHEEL_FW_IMG);
    let motor_bl = WheelMotor::new_from_pins(&BACK_LEFT_RX_UART_QUEUE,   &BACK_LEFT_TX_UART_QUEUE,   motor_bl_boot0_pin, motor_bl_nrst_pin, WHEEL_FW_IMG);
    let motor_br = WheelMotor::new_from_pins(&BACK_RIGHT_RX_UART_QUEUE,  &BACK_RIGHT_TX_UART_QUEUE,  motor_br_boot0_pin, motor_br_nrst_pin, WHEEL_FW_IMG);
    let motor_fr = WheelMotor::new_from_pins(&FRONT_RIGHT_RX_UART_QUEUE, &FRONT_RIGHT_TX_UART_QUEUE, motor_fr_boot0_pin, motor_fr_nrst_pin, WHEEL_FW_IMG);
    let motor_drib = DribblerMotor::new_from_pins(&DRIB_RX_UART_QUEUE,   &DRIB_TX_UART_QUEUE,        motor_d_boot0_pin,  motor_d_nrst_pin,  DRIB_FW_IMG, 1.0);

    control_task_spawner.spawn(control_task_entry(robot_state,
        command_subscriber, telemetry_publisher,
    motor_fl, motor_bl, motor_br, motor_fr, motor_drib)).unwrap();
}