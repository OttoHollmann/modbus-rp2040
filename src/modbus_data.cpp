#include "modbus_data.hpp"

ModbusData::ModbusData(uint addr, uint size):start(addr), nmemb(size), next(NULL) {
}

ModbusData::~ModbusData() {
	free(data);
	if (next)
		delete next;
}

const ModbusData* ModbusData::find(uint addr) const {
	if (addr >= start && addr < start + nmemb)
		return this;
	if (next)
		return next->find(addr);
	return NULL;
}

bool ModbusData::add_range(uint addr, uint size) {
	if (addr + size > start && start + nmemb > addr)
		return false;
	if (!next) {
		return next = insert_range(addr, size);
	}
	return next->add_range(addr, size);
}

ModbusData* ModbusCoil::insert_range(uint addr, uint size){
	return new ModbusCoil(addr, size);
}

ModbusData* ModbusRegister::insert_range(uint addr, uint size){
	return new ModbusRegister(addr, size);
}





ModbusRegister::ModbusRegister(uint addr, uint size):ModbusData(addr, size) {
	data = (uint16_t*) calloc(size, sizeof(uint16_t));
}

bool ModbusRegister::getValue(uint addr, uint16_t *val) const {
	ModbusRegister *res = (ModbusRegister*) find(addr);
	if (res) {
		*val = res->get_register(addr);
		return true;
	}
	return false;
}

bool ModbusRegister::writeValue(uint addr, uint16_t val) {
	ModbusRegister *res = (ModbusRegister*) find(addr);
	if (res)
		return res->write_register(addr, val);
	return false;
}

bool ModbusRegister::write_register(uint addr, uint16_t val) {
	*((uint16_t*)data + (addr - start)) = val;
	return true;
}

uint16_t ModbusRegister::get_register(uint addr) const {
	return *((uint16_t*)data + (addr - start));
}





ModbusCoil::ModbusCoil(uint addr, uint size):ModbusData(addr, size) {
	size = size/8 + (size%8 != 0);
	data = (uint8_t*) calloc(size, sizeof(uint8_t));
}

bool ModbusCoil::getValue(uint addr, bool *val) const {
	ModbusCoil *res = (ModbusCoil*) find(addr);
	if (res)
		*val = res->get_coil(addr);
		return true;
	return false;
}
bool ModbusCoil::writeValue(uint addr, bool val) {
	ModbusCoil *res = (ModbusCoil*) find(addr);
	if (res)
		return res->write_coil(addr, val);
	return false;
}

bool ModbusCoil::get_coil(uint addr) const {
	uint offset = 0;
	if (addr > start) offset = (addr - start - 1)/8 + 1;
	uint8_t* target = ((uint8_t*)data + offset);
	return *target & (1 << ((addr - start)%8));
}

bool ModbusCoil::write_coil(uint addr, bool val) {
	uint offset = 0;
	if (addr > start) offset = (addr - start - 1)/8 + 1;
	uint8_t* target = ((uint8_t*)data + offset);
	if (val)
		*target |= (1 << ((addr - start)%8));
	else
		*target &= ~(1 << ((addr - start)%8));
	return true;
}