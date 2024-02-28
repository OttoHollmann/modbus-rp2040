#include "modbus.hpp"

IRQ* IRQ::self = nullptr;
static IRQ irq;

/*
 * Call a member function like a static
 * inspired by https://forums.raspberrypi.com/viewtopic.php?t=336608
 */
IRQ::IRQ() {
	assert(selfptr == nullptr);
	self = this;
}

void IRQ::uart0_irq_handler() {
	self->static_uart0_irq_handler();
}

void IRQ::uart1_irq_handler() {
	self->static_uart1_irq_handler();
}

void IRQ::static_uart0_irq_handler() {
	if (mb0) mb0->mb_rx(uart_getc(uart0));
}

void IRQ::static_uart1_irq_handler() {
	if (mb1) mb1->mb_rx(uart_getc(uart1));
}

bool IRQ::install_handler(ModbusSlave *mb, uint8_t id) {
	switch (id) {
	case 0:
		mb0 = mb;
		irq_set_exclusive_handler(UART0_IRQ, uart0_irq_handler);
		break;
	case 1:
		mb1 = mb;
		irq_set_exclusive_handler(UART1_IRQ, uart1_irq_handler);
		break;
	default:
		return false;
	}
	return mb;
}


ModbusSlave::ModbusSlave():coils(NULL), discrete_input(NULL),
                           input_register(NULL), holding_register(NULL){};

ModbusSlave::~ModbusSlave() {
	if (coils)
		delete coils;
	if (discrete_input)
		delete discrete_input;
	if (input_register)
		delete input_register;
	if (holding_register)
		delete holding_register;
}

bool ModbusSlave::configure(mb_type_t type, uint16_t start, uint16_t len) {
	switch (type) {
	case COIL:
		if (!coils)
			return coils = new ModbusCoil(start, len);
		return coils->add_range(start, len);
	case DISCRETE_INPUT:
		if (!discrete_input)
			return discrete_input = new ModbusCoil(start, len);
		return discrete_input->add_range(start, len);
	case INPUT_REGISTER:
		if (!input_register)
			return input_register = new ModbusRegister(start, len);
		return input_register->add_range(start, len);
	case HOLDING_REGISTER:
		if (!holding_register)
			return holding_register = new ModbusRegister(start, len);
		return holding_register->add_range(start, len);
	}
	return false;
}

bool ModbusSlave::write(mb_type_t type, uint16_t address, uint16_t value) {
	switch (type) {
	case COIL:
		return coils && coils->writeValue(address, value);
	case DISCRETE_INPUT:
		return discrete_input && discrete_input->writeValue(address, value);
	case INPUT_REGISTER:
		return input_register && input_register->writeValue(address, value);
	case HOLDING_REGISTER:
		return holding_register && holding_register->writeValue(address, value);
	}
	return false;
}

bool ModbusSlave::read(mb_type_t type, uint16_t address, bool *value) const {
	switch (type) {
	case COIL:
		return coils && coils->getValue(address, value);
	case DISCRETE_INPUT:
		return discrete_input && discrete_input->getValue(address, value);
	}
	return false;
}

bool ModbusSlave::read(mb_type_t type, uint16_t address, uint16_t *value) const {
	switch (type) {
	case INPUT_REGISTER:
		return input_register && input_register->getValue(address, value);
	case HOLDING_REGISTER:
		return holding_register && holding_register->getValue(address, value);
	}
	return false;
}

uint16_t ModbusSlave::mb_calc_crc16(const uint8_t* buf, uint8_t len) const {
	uint16_t crc = 0xFFFF;
	uint8_t i, j = 0;
	while (j < len) {
		crc ^= buf[j];
		for (i = 0; i < 8; i++) {
			if (crc & 0x01) {
				crc >>= 1;
				crc ^= 0xA001;
			} else {
				crc >>= 1;
			}
		}
		j++;
	}
	return crc;
}

ModbusSlave::mb_state_t ModbusSlave::mb_check_buf() {
	if (mb_request_buf_pos > 4) {
		if (mb_request_buf[0] != mb_slave_address || mb_slave_address == 0) {
			return MB_INVALID_SLAVE_ADDRESS;
		}

		if (mb_request_buf[1] >= 0x01 && mb_request_buf[1] <= 0x06) {
			if (mb_request_buf_pos == 8) {
				return MB_DATA_READY;
			}
		} else if (mb_request_buf[1] == 0x10 || mb_request_buf[1] == 0x0F) {
			if (mb_request_buf_pos == mb_request_buf[6] + 9) {
				return MB_DATA_READY;
			}
		} else {
			return MB_INVALID_FUNCTION;
		}
	}

	return MB_DATA_INCOMPLETE;
}

void ModbusSlave::mb_reset_buf() {
	mb_request_buf_pos = 0;
	memset(mb_request_buf, 0, sizeof(mb_request_buf));
}

void ModbusSlave::mb_response_tx() {
	// Calculate CRC
	uint16_t crc = mb_calc_crc16(mb_response_buf, mb_response_buf_pos);
	mb_response_buf[mb_response_buf_pos++] = crc & 0xFF;
	mb_response_buf[mb_response_buf_pos++] = (crc & 0xFF00) >> 8;

 	// Send RTU packet
	mb_tx(mb_response_buf, mb_response_buf_pos);
}

void ModbusSlave::mb_error(uint8_t err) {
	mb_response_buf_pos = 0;
	mb_response_buf[mb_response_buf_pos++] = mb_slave_address;
	mb_response_buf[mb_response_buf_pos++] = mb_request_buf[1] | 0x80;
	mb_response_buf[mb_response_buf_pos++] = err;
	mb_response_tx();
}

//here I swapped the entries in the registers
void ModbusSlave::mb_response_add(uint16_t value) {
	mb_response_buf[2] += 2;
	mb_response_buf[mb_response_buf_pos++] = (value & 0xFF00) >> 8;
	mb_response_buf[mb_response_buf_pos++] = value & 0xFF;
}

// for write single register
void ModbusSlave::mb_response_add_without_length(uint16_t value) {
	if (mb_response_buf_pos == 3)
		mb_response_buf_pos--; // no need to send message length
	mb_response_buf[mb_response_buf_pos++] = (value & 0xFF00) >> 8;
	mb_response_buf[mb_response_buf_pos++] = value & 0xFF;
}

void ModbusSlave::mb_response_reset(uint8_t fn) {
	mb_response_buf_pos = 0;
	mb_response_buf[mb_response_buf_pos++] = mb_slave_address;
	mb_response_buf[mb_response_buf_pos++] = fn;
	mb_response_buf[mb_response_buf_pos++] = 0;
}

void ModbusSlave::mb_rx_rtu() {
	uint8_t res;

	// Check CRC
	uint16_t crc = mb_calc_crc16(mb_request_buf, mb_request_buf_pos - 2);
	if (memcmp(&crc, &mb_request_buf[mb_request_buf_pos - 2], sizeof(crc)) != 0) {
		mb_reset_buf();
		return;
	}

	mb_response_reset(mb_request_buf[1]);
	switch (mb_request_buf[1]) {
	case MB_READ_COIL_STATUS:
		res = mb_read_coil_status((mb_request_buf[2] << 8) + mb_request_buf[3],
			(mb_request_buf[4] << 8) + mb_request_buf[5]);
		break;
	case MB_READ_INPUT_STATUS:
		res = mb_read_input_status((mb_request_buf[2] << 8) + mb_request_buf[3],
			(mb_request_buf[4] << 8) + mb_request_buf[5]);
		break;
	case MB_READ_HOLDING_REGISTERS:
		res = mb_read_holding_registers((mb_request_buf[2] << 8) + mb_request_buf[3],
			(mb_request_buf[4] << 8) + mb_request_buf[5]);
		break;
	case MB_READ_INPUT_REGISTERS:
		res = mb_read_input_registers((mb_request_buf[2] << 8) + mb_request_buf[3],
			(mb_request_buf[4] << 8) + mb_request_buf[5]);
		break;
	case MB_WRITE_SINGLE_COIL:
		res = mb_write_single_coil((mb_request_buf[2] << 8) + mb_request_buf[3],
			(mb_request_buf[4] << 8) + mb_request_buf[5]);
		break;
	case MB_WRITE_SINGLE_REGISTER:
		res = mb_write_single_register((mb_request_buf[2] << 8) + mb_request_buf[3],
			(mb_request_buf[4] << 8) + mb_request_buf[5]);
		break;
	case MB_WRITE_MULTIPLE_COILS:
		res = mb_write_multiple_coils((mb_request_buf[2] << 8) + mb_request_buf[3], &mb_request_buf[6],
			(mb_request_buf[4] << 8) + mb_request_buf[5]);
		break;
	case MB_WRITE_MULTIPLE_REGISTERS:
		res = mb_write_multiple_registers((mb_request_buf[2] << 8) + mb_request_buf[3], (uint16_t*)&mb_request_buf[6],
			(mb_request_buf[4] << 8) + mb_request_buf[5]);
		break;
	default:
		res = MB_ERROR_ILLEGAL_FUNCTION;
		break;
	}

	if (res == MB_NO_ERROR)
		mb_response_tx();
	else
		mb_error(res);

	mb_reset_buf();
}

void ModbusSlave::mb_init(uint8_t slave_address, uint8_t uart_num,
	uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uart_parity_t parity,
	uint8_t rx_pin, uint8_t tx_pin, uint8_t de_pin) {
	mb_slave_address = slave_address;

	// DE pin init
	gpio_init(de_pin);
	gpio_set_dir(de_pin, GPIO_OUT);
	gpio_put(de_pin, 0);
	this->de_pin = de_pin;

	// UART init
	gpio_set_function(tx_pin, GPIO_FUNC_UART);
	gpio_set_function(rx_pin, GPIO_FUNC_UART);

	if(uart_num == 0) {
		uart_number = 0;
		uart_init(uart0, baudrate);
		uart_set_format(uart0, data_bits, stop_bits, parity);

		uart_set_fifo_enabled(uart0, false);

		irq_set_enabled(UART0_IRQ, true);
		uart_set_irq_enables(uart0, true, false);
	} else if(uart_num == 1) {
		uart_number = 1;
		uart_init(uart1, baudrate);
		uart_set_format(uart1, data_bits, stop_bits, parity);

		uart_set_fifo_enabled(uart1, false);

		irq_set_enabled(UART1_IRQ, true);
		uart_set_irq_enables(uart1, true, false);
	}

	irq.install_handler(this, uart_num);

	mb_reset_buf();
	coils = NULL;
	discrete_input = NULL;
	input_register = NULL;
	holding_register = NULL;
}

void ModbusSlave::mb_rx(uint8_t data) {
	if (mb_get_tick_ms() - mb_timeout > MB_TIMEOUT)
		mb_reset_buf();

	mb_timeout = mb_get_tick_ms();

	if (mb_request_buf_pos < (sizeof(mb_request_buf) - 1))
		mb_request_buf[mb_request_buf_pos++] = data;
}

void ModbusSlave::mb_process() {
	mb_state_t mb_state = mb_check_buf();
	switch (mb_state) {
	case MB_INVALID_FUNCTION:
		mb_error(MB_ERROR_ILLEGAL_FUNCTION);

	case MB_INVALID_SLAVE_ADDRESS:
		mb_reset_buf();
		break;

	case MB_DATA_READY:
		mb_rx_rtu();

	default:
	case MB_DATA_INCOMPLETE:
		break;
	}
}

uint8_t ModbusSlave::mb_read_coil_status(uint16_t start, uint16_t count) {
	bool value;
	if (coils) {
		for (int i = 0; i < count; ++i) {
			if (!coils->getValue(start + i, &value))
				return MB_ERROR_ILLEGAL_DATA_ADDRESS;
			mb_response_add(value);
		}
		return MB_NO_ERROR;
	}
	return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

uint8_t ModbusSlave::mb_read_input_status(uint16_t start, uint16_t count) {
	bool value;
	if (discrete_input) {
		for (int i = 0; i < count; ++i) {
			if (!discrete_input->getValue(start + i, &value))
				return MB_ERROR_ILLEGAL_DATA_ADDRESS;
			mb_response_add(value);
		}
		return MB_NO_ERROR;
	}
	return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

uint8_t ModbusSlave::mb_read_holding_registers(uint16_t start, uint16_t count) {
	uint16_t value;
	if (holding_register) {
		for (int i = 0; i < count; ++i) {
			if (!holding_register->getValue(start + i, &value))
				return MB_ERROR_ILLEGAL_DATA_ADDRESS;
			mb_response_add(value);
		}
		return MB_NO_ERROR;
	}
	return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

uint8_t ModbusSlave::mb_read_input_registers(uint16_t start, uint16_t count) {
	uint16_t value;
	if (input_register) {
		for (int i = 0; i < count; ++i) {
			if (!input_register->getValue(start + i, &value))
				return MB_ERROR_ILLEGAL_DATA_ADDRESS;
			mb_response_add(value);
		}
		return MB_NO_ERROR;
	}
	return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

uint8_t ModbusSlave::mb_write_single_coil(uint16_t start, uint16_t value) {
	if (coils && coils->writeValue(start, value)) {
		mb_response_add_without_length(start);
		mb_response_add_without_length(value);
		return MB_NO_ERROR;
	}
	return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

uint8_t ModbusSlave::mb_write_single_register(uint16_t start, uint16_t value) {
	if (holding_register && holding_register->writeValue(start, value)) {
		mb_response_add_without_length(start);
		mb_response_add_without_length(value);
		return MB_NO_ERROR;
	}
	return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

uint8_t ModbusSlave::mb_write_multiple_coils(uint16_t start, uint8_t* values, uint16_t count) {
	if (coils) {
		uint16_t coils_written = 0;
		for (int i = 0; i < count; ++i) {
			coils_written += coils->writeValue(start + i, values[i]);
		}
		mb_response_add_without_length(start);
		mb_response_add_without_length(coils_written);
		if (coils_written == count)
			return MB_NO_ERROR;
	}
	return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

uint8_t ModbusSlave::mb_write_multiple_registers(uint16_t start, uint16_t *values, uint16_t count) {
	if (holding_register) {
		uint16_t registers_written = 0;
		for (int i = 0; i < count; ++i) {
			registers_written += holding_register->writeValue(start + i, values[i]);
		}
		mb_response_add_without_length(start);
		mb_response_add_without_length(registers_written);
		if (registers_written == count)
			return MB_NO_ERROR;
	}
	return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

uint32_t ModbusSlave::mb_get_tick_ms(void) {
	return time_us_64() / 1000;
}

void ModbusSlave::mb_tx(uint8_t* data, uint32_t size) {
	gpio_put(de_pin, 1);

	if (uart_number == 0)
		uart_write_blocking(uart0, data, size);
	else
		uart_write_blocking(uart1, data, size);

	// Wait until fifo is drained so we now when to turn off the driver enable pin.
	if (uart_number == 0)
		uart_tx_wait_blocking(uart0);
	else
		uart_tx_wait_blocking(uart1);

	gpio_put(de_pin, 0);
}