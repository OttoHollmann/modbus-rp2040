#include <stdlib.h>
#include <stdio.h>
#include <cstdint>

class ModbusData {
public:
	ModbusData(uint addr, uint size);
	virtual ~ModbusData();
	bool add_range(uint addr, uint size);
protected:
	const ModbusData* find(uint addr) const;
	virtual ModbusData* insert_range(uint addr, uint size) = 0;
	uint start;
	uint nmemb;
	void *data;
	ModbusData *next;
};

class ModbusRegister:public ModbusData {
public:
	ModbusRegister(uint addr, uint size);
	bool getValue(uint addr, uint16_t *val) const;
	bool writeValue(uint addr, uint16_t val);
private:
	ModbusData* insert_range(uint addr, uint size);
	bool write_register(uint addr, uint16_t val);
	uint16_t get_register(uint addr) const;
};

class ModbusCoil:public ModbusData {
public:
	ModbusCoil(uint addr, uint size);
	bool getValue(uint addr, bool *val) const;
	bool writeValue(uint addr, bool val);
private:
	ModbusData* insert_range(uint addr, uint size);
	bool write_coil(uint addr, bool val);
	bool get_coil(uint addr) const;
};