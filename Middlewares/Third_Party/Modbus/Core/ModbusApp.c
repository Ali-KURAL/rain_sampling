/*
 * ModbusApp.c
 *
 *  Created on: Nov 15, 2019
 *      Author: Kaan
 */


#include "ModbusApp.h"
#include "Modbus_Config.h"
#include <stddef.h>

ModbusRegister 	_coilStatusRegisters[MODBUS_SLAVE_COIL_STATUS_SIZE];
ModbusRegister 	_inputStatusRegisters[MODBUS_SLAVE_INPUT_STATUS_SIZE];
ModbusRegister 	_holdingRegisters[MODBUS_SLAVE_HOLDING_REGISTER_SIZE];
ModbusRegister 	_inputRegisters[MODBUS_SLAVE_INPUT_REGISTER_SIZE];

ModbusRegister* ModbusSlave_GetRegister( const ModbusRegister_Handle_t* handle );
ModbusRegister* ModbusSlave_GetRegisterByAddress( ModbusRegisterType type, uint16_t address );

ModbusRegister_Handle_t ModbusSlave_CreateCoilStatus( uint16_t address, bool value ){
	ModbusRegister_Handle_t result;
	result.id = -1;
	for( int k = 0; k < MODBUS_SLAVE_COIL_STATUS_SIZE; k++ ){
		if( !_coilStatusRegisters[k].isUsed ){
			_coilStatusRegisters[k].address = address;
			_coilStatusRegisters[k].value = (uint16_t)value;
			_coilStatusRegisters[k].isUsed = true;
			_coilStatusRegisters[k].onRead = NULL;
			_coilStatusRegisters[k].onWrite = NULL;
			result.type = COIL_STATUS;
			result.id = k;
			break;
		}
	}
	return result;
}

ModbusRegister_Handle_t ModbusSlave_CreateInputStatus( uint16_t address, bool value ){
	ModbusRegister_Handle_t result;
	result.id = -1;
	for( int k = 0; k < MODBUS_SLAVE_INPUT_STATUS_SIZE; k++ ){
		if( !_inputStatusRegisters[k].isUsed ){
			_inputStatusRegisters[k].address = address;
			_inputStatusRegisters[k].value = (uint16_t)value;
			_inputStatusRegisters[k].isUsed = true;
			_inputStatusRegisters[k].onRead = NULL;
			_inputStatusRegisters[k].onWrite = NULL;
			result.type = INPUT_STATUS;
			result.id = k;
			break;
		}
	}
	return result;
}

ModbusRegister_Handle_t ModbusSlave_CreateHoldingRegister( uint16_t address, uint16_t value ){
	ModbusRegister_Handle_t result;
	result.id = -1;
	for( int k = 0; k < MODBUS_SLAVE_HOLDING_REGISTER_SIZE; k++ ){
		if( !_holdingRegisters[k].isUsed ){
			_holdingRegisters[k].address = address;
			_holdingRegisters[k].value = value;
			_holdingRegisters[k].isUsed = true;
			_holdingRegisters[k].onRead = NULL;
			_holdingRegisters[k].onWrite = NULL;
			result.id = k;
			result.type = HOLDING_REGISTER;
			break;
		}
	}
	return result;
}

ModbusRegister_Handle_t ModbusSlave_CreateInputRegister( uint16_t address, uint16_t value ){
	ModbusRegister_Handle_t result;
	result.id = -1;
	for( int k = 0; k < MODBUS_SLAVE_INPUT_REGISTER_SIZE; k++ ){
		if( !_inputRegisters[k].isUsed ){
			_inputRegisters[k].address = address;
			_inputRegisters[k].value = value;
			_inputRegisters[k].isUsed = true;
			_inputRegisters[k].onRead = NULL;
			_inputRegisters[k].onWrite = NULL;
			result.id = k;
			result.type = INPUT_REGISTER;
			break;
		}
	}
	return result;
}

ModbusOpResult ModbusSlave_AddOnReadCallback( const ModbusRegister_Handle_t* handle, ModbusRegisterRead readCallback ){
	ModbusOpResult result = MODBUS_OP_INVALID_REGISTER;
	ModbusRegister *_register = ModbusSlave_GetRegister(handle);
	if( _register != NULL ){
		result = MODBUS_OP_SUCCESS;
		_register->onRead = readCallback;
	}
	return result;
}
ModbusOpResult ModbusSlave_ClearOnReadCallback( const ModbusRegister_Handle_t* handle ){
	ModbusOpResult result = MODBUS_OP_INVALID_REGISTER;
	ModbusRegister *_register = ModbusSlave_GetRegister(handle);
	if( _register != NULL ){
		result = MODBUS_OP_SUCCESS;
		_register->onRead = NULL;
	}
	return result;
}


ModbusOpResult ModbusSlave_AddOnWriteCallback( const ModbusRegister_Handle_t* handle, ModbusRegisterWrite writeCallback ){
	ModbusOpResult result = MODBUS_OP_INVALID_REGISTER;
	if( handle->type == INPUT_STATUS || handle->type == INPUT_REGISTER ){
		result = MODBUS_OP_INVALID_REQUEST;
	}
	else{
		ModbusRegister *_register = ModbusSlave_GetRegister(handle);
		if( _register != NULL ){
			result = MODBUS_OP_SUCCESS;
			_register->onWrite = writeCallback;
		}
	}
	return result;
}

ModbusOpResult ModbusSlave_ClearOnWriteCallback( const ModbusRegister_Handle_t* handle ){
	ModbusOpResult result = MODBUS_OP_INVALID_REGISTER;
	if( handle->type == INPUT_STATUS || handle->type == INPUT_REGISTER ){
		result = MODBUS_OP_INVALID_REQUEST;
	}
	else{
		ModbusRegister *_register = ModbusSlave_GetRegister(handle);
		if( _register != NULL ){
			result = MODBUS_OP_SUCCESS;
			_register->onWrite = NULL;
		}
	}
	return result;
}

ModbusOpResult ModbusSlave_SetRegisterValue( const ModbusRegister_Handle_t* handle, uint16_t value ){
	ModbusOpResult result = MODBUS_OP_INVALID_REGISTER;
	ModbusRegister *_register = ModbusSlave_GetRegister(handle);
	if( _register != NULL ){
		result = MODBUS_OP_SUCCESS;
		_register->value = value;
		// fire on write callback
		if( _register->onWrite != NULL ){
			_register->onWrite(value);
		}
	}
	return result;
}

ModbusOpResult ModbusSlave_GetRegisterValue( const ModbusRegister_Handle_t* handle, uint16_t* value ){
	ModbusOpResult result = MODBUS_OP_INVALID_REGISTER;
	ModbusRegister *_register = ModbusSlave_GetRegister(handle);
	if( _register != NULL ){
		result = MODBUS_OP_SUCCESS;
		*value = _register->value;
		// fire on read callback
		if( _register->onRead != NULL ){
			_register->onRead();
		}
	}
	return result;
}

ModbusOpResult ModbusSlave_SetRegisterValueByAddress( ModbusRegisterType type, uint16_t address, uint16_t value ){
	ModbusOpResult result = MODBUS_OP_INVALID_REGISTER;
	ModbusRegister *_register = ModbusSlave_GetRegisterByAddress(type, address);
	if( _register != NULL ){
		result = MODBUS_OP_SUCCESS;
		_register->value = value;
		// fire on read callback
		if( _register->onWrite != NULL ){
			_register->onWrite(value);
		}
	}
	return result;
}

ModbusOpResult ModbusSlave_GetRegisterValueByAddress( ModbusRegisterType type, uint16_t address, uint16_t* value ){
	ModbusOpResult result = MODBUS_OP_INVALID_REGISTER;
	ModbusRegister *_register = ModbusSlave_GetRegisterByAddress(type, address);
	if( _register != NULL ){
		result = MODBUS_OP_SUCCESS;
		*value = _register->value;
		// fire on read callback
		if( _register->onRead != NULL ){
			_register->onRead();
		}
	}
	return result;
}

// Private functions

ModbusRegister* ModbusSlave_GetRegister( const ModbusRegister_Handle_t* handle ){
	ModbusRegister* returnValue = NULL;
	if( handle->type == COIL_STATUS ){
		if( handle->id >= 0 && handle->id < MODBUS_SLAVE_COIL_STATUS_SIZE ){
			if( _coilStatusRegisters[handle->id].isUsed ){
				returnValue = &(_coilStatusRegisters[handle->id]);
			}
		}
	}
	else if( handle->type == INPUT_STATUS ){
		if( handle->id >= 0 && handle->id < MODBUS_SLAVE_INPUT_STATUS_SIZE ){
			if( _inputStatusRegisters[handle->id].isUsed ){
				returnValue = &(_inputStatusRegisters[handle->id]);
			}
		}
	}
	else if( handle->type == HOLDING_REGISTER ){
		if( handle->id >= 0 && handle->id < MODBUS_SLAVE_HOLDING_REGISTER_SIZE ){
			if( _holdingRegisters[handle->id].isUsed ){
				returnValue = &(_holdingRegisters[handle->id]);
			}
		}
	}
	else if( handle->type == INPUT_REGISTER ){
		if( handle->id >= 0 && handle->id < MODBUS_SLAVE_INPUT_REGISTER_SIZE ){
			if( _inputRegisters[handle->id].isUsed ){
				returnValue = &(_inputRegisters[handle->id]);
			}
		}
	}
	return returnValue;
}

ModbusRegister* ModbusSlave_GetRegisterByAddress( ModbusRegisterType type, uint16_t address ){
	ModbusRegister* returnValue = NULL;
	if( type == COIL_STATUS ){
		for( int k = 0; k < MODBUS_SLAVE_COIL_STATUS_SIZE; k++ ){
			if( _coilStatusRegisters[k].isUsed && address == _coilStatusRegisters[k].address ){
				returnValue = &(_coilStatusRegisters[k]);
				break;
			}
		}
	}
	else if( type == INPUT_STATUS ){
		for( int k = 0; k < MODBUS_SLAVE_INPUT_STATUS_SIZE; k++ ){
			if( _inputStatusRegisters[k].isUsed && address == _inputStatusRegisters[k].address ){
				returnValue = &(_inputStatusRegisters[k]);
				break;
			}
		}
	}
	else if( type == HOLDING_REGISTER ){
		for( int k = 0; k < MODBUS_SLAVE_HOLDING_REGISTER_SIZE; k++ ){
			if( _holdingRegisters[k].isUsed && address == _holdingRegisters[k].address ){
				returnValue = &(_holdingRegisters[k]);
				break;
			}
		}
	}
	else if( type == INPUT_REGISTER ){
		for( int k = 0; k < MODBUS_SLAVE_INPUT_REGISTER_SIZE; k++ ){
			if( _inputRegisters[k].isUsed && address == _inputRegisters[k].address ){
				returnValue = &(_inputRegisters[k]);
				break;
			}
		}
	}
	return returnValue;
}
