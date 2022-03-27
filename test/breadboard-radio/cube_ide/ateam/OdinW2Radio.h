/*
 * OdinW2Radio.h
 *
 *  Created on: Feb 13, 2022
 *      Author: guyfl
 */

#ifndef ODINW2RADIO_H_
#define ODINW2RADIO_H_

#include "main.h"

namespace ateam {

enum class ConnectionState {
	DISCONNECTED,
	CONNECTED_PHY_SLOW,
	CONNECTED_PHY_FAST,
	CONNECTED_WIFI_IDLE,
	CONNECTED_WIFI_WAIT_FOR_SOCCER,
	CONNECTED_WIFI_SOCCER
};

enum class UartTxState {
	IDLE,
	DMA_TRANSMITTING,
	DMA_TRANSMIT_COMPLETE_SUCC,
	DMA_TRANSMIT_COMPLETE_ERROR
};

enum class UartRxState {
	IDLE,
	AT_RECEIVING_LINE,
	AT_LINE_RECEIVED_SUCC,
	AT_LINE_RECEIVED_ERROR,
//	DMA_WAIT_HEADER,
//	DMA_WAIT_HEADER_PAYLOAD_LEN,
//	DMA_WAIT_PAYLOAD,
//	DMA_RECEIVED
};

// outward facing state
enum class RadioCommandState {
	IDLE,
	COMMAND_TRANSMITTING,
	WAITING_RESPONSE,
	RESPONSE_OK,
	RESPONSE_ERROR
};

enum class RadioCommand {
	VERIFY_MODEL_NUMBER,
	CONFIGURE_UART,
	CONNECT_WIFI,
	SETUP_UDP_SOCKET,
	SEND_DATA
};

class OdinW2Radio {
public:
	OdinW2Radio(UART_HandleTypeDef *uart, GPIO_TypeDef *reset_pin_bank, uint16_t reset_pin_ind, const int rx_buf_len = 512, const int tx_buf_len = 512);
	OdinW2Radio(UART_HandleTypeDef *uart, DMA_HandleTypeDef *uart_dma, GPIO_TypeDef *reset_pin_bank, uint16_t reset_pin_ind, const int rx_buf_len = 512, const int tx_buf_len = 512);
	virtual ~OdinW2Radio();

	void start();

	void hard_reset(int hold_time_ms = 1000);
//	void soft_reset();
//
//	void set_ssid(std::string ssid);
//	std::string get_ssid();
//
//	void set_password(std::string password);
//	std::string get_password();
//
//	void network_connect_blocking();
//	bool is_network_connected();
//
//	void set_connection_ip(std::string connection_ip);
//	std::string get_connection_ip();
//
//	void set_connection_port(std::string connection_port);
//	std::string get_connection_port();
//
//	void host_connect_blocking();

	// interrupt callbacks
	void __dma_interrupt_tx_complete();
	void __dma_interrupt_tx_error();
	void __dma_interrupt_rx_complete();
	void __dma_interrupt_rx_error();
protected:
	int rx_buf_len, tx_buf_len;

	UART_HandleTypeDef *uart;
	DMA_HandleTypeDef *uart_dma;

	GPIO_TypeDef *reset_pin_bank;
	uint16_t reset_pin_ind;

	uint8_t *rx_buf;
	uint8_t *tx_buf;

	ConnectionState connection_state = ConnectionState::DISCONNECTED;
	UartRxState uart_rx_state = UartRxState::IDLE;
	UartTxState uart_tx_state = UartTxState::IDLE;
	RadioCommandState command_state = RadioCommandState::IDLE;

	void allocate_uart_buffers();
	void clear_tx_buf();
	void clear_rx_buf();

	bool write_uart_dma_nb(int len);
	bool write_uart_dma_b(int len);

	void read_uart_dma_line_nb();
	void read_uart_dma_line_b();
	void read_uart_dma_raw_len_nb(int len);
	void read_uart_dma_raw_len_b();

//	void dispatch_command()


};

} /* namespace ateam */

#endif /* ODINW2RADIO_H_ */
