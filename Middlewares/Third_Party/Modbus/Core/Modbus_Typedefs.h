/*
 * Modbus_Typedefs.h
 *
 *  Created on: Nov 10, 2019
 *      Author: Kaan
 */

#ifndef THIRD_PARTY_MODBUS_CORE_MODBUS_TYPEDEFS_H_
#define THIRD_PARTY_MODBUS_CORE_MODBUS_TYPEDEFS_H_

#include <stdint.h>
#include <stdbool.h>

#define MODBUS_RTU_RX_BUFFER_SIZE   128

typedef uint8_t ModbusID;




typedef void(*ModbusRegisterRead)(void);
typedef void(*ModbusRegisterWrite)(uint16_t value);

typedef enum ModbusMsgTypes {
	READ_COIL_STATUS = 1,
	READ_INPUT_STATUS = 2,
	READ_HOLDING_REGISTERS = 3,
	READ_INPUT_REGISTERS = 4,
	FORCE_SINGLE_COIL = 5,
	PRESET_SINGLE_REGISTER = 6,
	FORCE_MULTIPLE_COILS = 15,
	PRESET_MULTIPLE_REGISTERS = 16
}ModbusMsgTypes;

typedef enum ModbusResult {
	MODBUS_SUCCESS,
	MODBUS_CRC_ERR,
	MODBUS_INVALID_ADDRESS,
	MODBUS_INVALID_OPERATION,
	MODBUS_UNSUPPORTED_OPERATION
}ModbusResult;

typedef enum ModbusException {
	MODBUS_ILLEGAL_FUNCTION = 1,
	MODBUS_ILLEGAL_DATA_ADDRESS = 2,
	MODBUS_ILLEGAL_DATA_VALUE = 3,
	MODBUS_SLAVE_DEVICE_FAILURE = 4,
	MODBUS_ACKNOWLEDGE = 5,
	MODBUS_SLAVE_DEVICE_BUSY = 6,
	MODBUS_NEGATIVE_ACKNOWLEDGE = 7,
	MODBUS_MEMORY_PARITY_ERROR = 8,
	MODBUS_GATEWAY_PATH_UNAVAILABLE = 9,
	MODBUS_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 10
}ModbusException;

typedef enum ModbusOpResult{
	MODBUS_OP_SUCCESS,
	MODBUS_OP_INVALID_REGISTER,
	MODBUS_OP_INVALID_REQUEST,
}ModbusOpResult;

typedef enum ModbusBaudRate{
	MODBUS_9600 = 0,
	MODBUS_115200 = 1
}ModbusBaudRate;

typedef enum ModbusParity{
	MODBUS_EVEN = 0,
	MODBUS_ODD = 1,
	MODBUS_NONE = 2
}ModbusParity;

typedef enum ModbusRegisterType{
	COIL_STATUS = 0,
	INPUT_STATUS = 1,
	HOLDING_REGISTER = 2,
	INPUT_REGISTER = 3
}ModbusRegisterType;

typedef struct ModbusRegister{
	uint16_t address;
	uint16_t value;
	ModbusRegisterType type;
	bool isUsed;
	ModbusRegisterRead onRead;
	ModbusRegisterWrite onWrite;
}ModbusRegister;


typedef struct ModbusRegister_Handle_t{
	int16_t id;
	ModbusRegisterType type;
}ModbusRegister_Handle_t;

typedef struct ModbusBuffer{
	uint8_t buffer[MODBUS_RTU_RX_BUFFER_SIZE];
	uint16_t index;
}ModbusBuffer;

typedef struct ModbusPDU_t{
	ModbusMsgTypes type;
	ModbusBuffer payload;
}ModbusPDU_t;


#endif /* THIRD_PARTY_MODBUS_CORE_MODBUS_TYPEDEFS_H_ */
