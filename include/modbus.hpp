#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <hardware/irq.h>
#include <hardware/uart.h>
#include <pico/stdlib.h>
#include <modbus_data.hpp>

/* Constatnts */
#define MB_RX_BUF_SIZE                  64
#define MB_TX_BUF_SIZE                  64
#define MB_TIMEOUT                      100

/* Modbus error codes */
#define MB_NO_ERROR                     0x00
#define MB_ERROR_ILLEGAL_FUNCTION       0x01
#define MB_ERROR_ILLEGAL_DATA_ADDRESS   0x02
#define MB_ERROR_ILLEGAL_DATA_VALUE     0x03
#define MB_ERROR_SLAVE_DEVICE_FAILURE   0x04

/* Modbus command codes */
#define MB_READ_COIL_STATUS             0x01
#define MB_READ_INPUT_STATUS            0x02
#define MB_READ_HOLDING_REGISTERS       0x03
#define MB_READ_INPUT_REGISTERS         0x04
#define MB_WRITE_SINGLE_COIL            0x05
#define MB_WRITE_SINGLE_REGISTER        0x06
#define MB_WRITE_MULTIPLE_COILS         0x0F
#define MB_WRITE_MULTIPLE_REGISTERS     0x10


class ModbusSlave {
public:
	ModbusSlave();
	~ModbusSlave();
	typedef enum { MB_DATA_READY, MB_DATA_INCOMPLETE, MB_INVALID_SLAVE_ADDRESS, MB_INVALID_FUNCTION } mb_state_t;
	typedef enum { COIL, DISCRETE_INPUT, INPUT_REGISTER, HOLDING_REGISTER } mb_type_t;

	void mb_init(uint8_t slave_address, uint8_t uart_num,
	             uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uart_parity_t parity,
	             uint8_t rx_pin, uint8_t tx_pin, uint8_t de_pin);

	bool configure(mb_type_t type, uint16_t start, uint16_t len);
	bool write(mb_type_t type, uint16_t address, uint16_t value);
	bool read(mb_type_t type, uint16_t address, bool *value) const;
	bool read(mb_type_t type, uint16_t address, uint16_t *value) const;
	void mb_rx(uint8_t data);
	void mb_process();

	uint8_t uart_number;

private:
	void mb_response_add(uint16_t value);
	void mb_response_add_without_length(uint16_t value);
	void mb_response_reset(uint8_t fn);

	void mb_tx(uint8_t* data, uint32_t size);

	uint32_t mb_get_tick_ms(void);

	uint8_t mb_read_coil_status(uint16_t start, uint16_t count);
	uint8_t mb_read_input_status(uint16_t start, uint16_t count);
	uint8_t mb_read_holding_registers(uint16_t start, uint16_t count);
	uint8_t mb_read_input_registers(uint16_t start, uint16_t count);
	uint8_t mb_write_single_coil(uint16_t start, uint16_t value);
	uint8_t mb_write_single_register(uint16_t start, uint16_t value);
	uint8_t mb_write_multiple_coils(uint16_t start, uint8_t* values, uint16_t len);
	uint8_t mb_write_multiple_registers(uint16_t start, uint16_t* values, uint16_t len);

	uint16_t mb_calc_crc16(const uint8_t* buf, uint8_t len) const;
	mb_state_t mb_check_buf();

	void mb_reset_buf();
	void mb_error(uint8_t err);
	void mb_rx_rtu();
	void mb_response_tx();

	uint8_t de_pin;
	uint8_t mb_slave_address = 0;

	uint8_t mb_request_buf[MB_RX_BUF_SIZE];
	uint8_t mb_response_buf[MB_TX_BUF_SIZE];

	int mb_request_buf_pos = 0;
	int mb_response_buf_pos = 0;

	uint32_t mb_timeout;

	ModbusCoil *coils;
	ModbusCoil *discrete_input;
	ModbusRegister *input_register;
	ModbusRegister *holding_register;
};

class IRQ {
public:
	IRQ();
	bool install_handler(ModbusSlave *mb, uint8_t id);
	static void uart0_irq_handler();
	static void uart1_irq_handler();
	static IRQ *self;
private:
	void static_uart0_irq_handler();
	void static_uart1_irq_handler();
	ModbusSlave *mb0;
	ModbusSlave *mb1;
};