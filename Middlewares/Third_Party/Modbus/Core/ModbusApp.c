/*
 * ModbusApp.c
 *
 *  Created on: Nov 15, 2019
 *      Author: Kaan
 */


#include "ModbusApp.h"
#include "Modbus_Config.h"

ModbusRegister 	_coilStatusRegisters[MODBUS_SLAVE_COIL_STATUS_SIZE];
ModbusRegister 	_inputStatusRegisters[MODBUS_SLAVE_INPUT_STATUS_SIZE];
ModbusRegister 	_holdingRegisters[MODBUS_SLAVE_HOLDING_REGISTER_SIZE];
ModbusRegister 	_inputRegisters[MODBUS_SLAVE_INPUT_REGISTER_SIZE];


ModbusRegister_Handle_t ModbusSlave_CreateCoilStatus( uint16_t address, bool value ){
	ModbusRegister_Handle_t result = -1;
	for( int k = 0; k < MODBUS_SLAVE_COIL_STATUS_SIZE; k++ ){
		if( !_coilStatusRegisters[k].isUsed ){
			_coilStatusRegisters[k].address = address;
			_coilStatusRegisters[k].value = (uint16_t)value;
			_coilStatusRegisters[k].isUsed = true;
			result = k;
			break;
		}
	}
	return result;
}

ModbusRegister_Handle_t ModbusSlave_CreateInputStatus( uint16_t address, bool value ){
	ModbusRegister_Handle_t result = -1;
	for( int k = 0; k < MODBUS_SLAVE_INPUT_STATUS_SIZE; k++ ){
		if( !_inputStatusRegisters[k].isUsed ){
			_inputStatusRegisters[k].address = address;
			_inputStatusRegisters[k].value = (uint16_t)value;
			_inputStatusRegisters[k].isUsed = true;
			result = k;
			break;
		}
	}
	return result;
}

ModbusRegister_Handle_t ModbusSlave_CreateHoldingRegister( uint16_t address, uint16_t value ){
	ModbusRegister_Handle_t result = -1;
	for( int k = 0; k < MODBUS_SLAVE_HOLDING_REGISTER_SIZE; k++ ){
		if( !_holdingRegisters[k].isUsed ){
			_holdingRegisters[k].address = address;
			_holdingRegisters[k].value = value;
			_holdingRegisters[k].isUsed = true;
			result = k;
			break;
		}
	}
	return result;
}

ModbusRegister_Handle_t ModbusSlave_CreateInputRegister( uint16_t address, uint16_t value ){
	ModbusRegister_Handle_t result = -1;
	for( int k = 0; k < MODBUS_SLAVE_INPUT_REGISTER_SIZE; k++ ){
		if( !_inputRegisters[k].isUsed ){
			_inputRegisters[k].address = address;
			_inputRegisters[k].value = value;
			_inputRegisters[k].isUsed = true;
			result = k;
			break;
		}
	}
	return result;
}


ModbusOpResult ModbusSlave_SetCoilStatus(ModbusRegister_Handle_t handle, bool value ){
	ModbusOpResult result = MODBUS_OP_SUCCESS;
	if( handle >= 0 && handle < MODBUS_SLAVE_COIL_STATUS_SIZE ){
		if( _coilStatusRegisters[handle].isUsed ){
			_coilStatusRegisters[handle].value = (uint16_t)value;
			result =  MODBUS_OP_SUCCESS;
		}
		else{
			result = MODBUS_OP_INVALID_REGISTER;
		}
	}else{
		result = MODBUS_OP_INVALID_REGISTER;
	}
	return result;
}

ModbusOpResult ModbusSlave_GetCoilStatus(ModbusRegister_Handle_t handle, bool *value ){
	ModbusOpResult result = MODBUS_OP_SUCCESS;
	if( handle >= 0 && handle < MODBUS_SLAVE_COIL_STATUS_SIZE ){
		if( _coilStatusRegisters[handle].isUsed ){
			*value = (bool)_coilStatusRegisters[handle].value;
			result =  MODBUS_OP_SUCCESS;
		}
		else{
			result = MODBUS_OP_INVALID_REGISTER;
		}
	}else{
		result = MODBUS_OP_INVALID_REGISTER;
	}
	return result;
}

ModbusOpResult ModbusSlave_GetCoilStatusByAddress( uint16_t address, bool* value ){
	ModbusOpResult result = MODBUS_OP_INVALID_REGISTER;
	for( int k = 0; k < MODBUS_SLAVE_COIL_STATUS_SIZE; k++ ){
		if( _coilStatusRegisters[k].isUsed && address == _coilStatusRegisters[k].address ){
			*value = (bool)_coilStatusRegisters[k].value;
			result = MODBUS_OP_SUCCESS;
			break;
		}
	}
	return result;
}


ModbusOpResult ModbusSlave_SetInputStatus(ModbusRegister_Handle_t handle, bool value ){
	ModbusOpResult result = MODBUS_OP_SUCCESS;
	if( handle >= 0 && handle < MODBUS_SLAVE_INPUT_STATUS_SIZE ){
		if( _inputStatusRegisters[handle].isUsed ){
			_inputStatusRegisters[handle].value = (uint16_t)value;
			result =  MODBUS_OP_SUCCESS;
		}
		else{
			result = MODBUS_OP_INVALID_REGISTER;
		}
	}else{
		result = MODBUS_OP_INVALID_REGISTER;
	}
	return result;
}

ModbusOpResult ModbusSlave_GetInputStatus(ModbusRegister_Handle_t handle, bool *value ){
	ModbusOpResult result = MODBUS_OP_SUCCESS;
	if( handle >= 0 && handle < MODBUS_SLAVE_INPUT_STATUS_SIZE ){
		if( _inputStatusRegisters[handle].isUsed ){
			*value = (bool)_inputStatusRegisters[handle].value;
			result =  MODBUS_OP_SUCCESS;
		}
		else{
			result = MODBUS_OP_INVALID_REGISTER;
		}
	}else{
		result = MODBUS_OP_INVALID_REGISTER;
	}
	return result;
}

ModbusOpResult ModbusSlave_GetInputStatusByAddress( uint16_t address, bool* value ){
	ModbusOpResult result = MODBUS_OP_INVALID_REGISTER;
	for( int k = 0; k < MODBUS_SLAVE_INPUT_STATUS_SIZE; k++ ){
		if( _inputStatusRegisters[k].isUsed && address == _inputStatusRegisters[k].address ){
			*value = (bool)_inputStatusRegisters[k].value;
			result = MODBUS_OP_SUCCESS;
			break;
		}
	}
	return result;
}

ModbusOpResult ModbusSlave_SetHoldingRegister(ModbusRegister_Handle_t handle, uint16_t value ){
	ModbusOpResult result = MODBUS_OP_SUCCESS;
	if( handle >= 0 && handle < MODBUS_SLAVE_HOLDING_REGISTER_SIZE ){
		if( _holdingRegisters[handle].isUsed ){
			_holdingRegisters[handle].value = value;
			result =  MODBUS_OP_SUCCESS;
		}
		else{
			result = MODBUS_OP_INVALID_REGISTER;
		}
	}else{
		result = MODBUS_OP_INVALID_REGISTER;
	}
	return result;

}

ModbusOpResult ModbusSlave_GetHoldingRegister(ModbusRegister_Handle_t handle, uint16_t* value ){
	ModbusOpResult result = MODBUS_OP_SUCCESS;
	if( handle >= 0 && handle < MODBUS_SLAVE_HOLDING_REGISTER_SIZE ){
		if( _holdingRegisters[handle].isUsed ){
			*value =  _holdingRegisters[handle].value;
			result =  MODBUS_OP_SUCCESS;
		}
		else{
			result = MODBUS_OP_INVALID_REGISTER;
		}
	}else{
		result = MODBUS_OP_INVALID_REGISTER;
	}
	return result;
}

ModbusOpResult ModbusSlave_GetHoldingRegisterByAddress( uint16_t address, uint16_t* value ){
	ModbusOpResult result = MODBUS_OP_INVALID_REGISTER;
	for( int k = 0; k < MODBUS_SLAVE_HOLDING_REGISTER_SIZE; k++ ){
		if( _holdingRegisters[k].isUsed && address == _holdingRegisters[k].address ){
			*value = _holdingRegisters[k].value;
			result = MODBUS_OP_SUCCESS;
			break;
		}
	}
	return result;
}

ModbusOpResult ModbusSlave_SetInputRegister(ModbusRegister_Handle_t handle, uint16_t value ){
	ModbusOpResult result = MODBUS_OP_SUCCESS;
	if( handle >= 0 && handle < MODBUS_SLAVE_INPUT_REGISTER_SIZE ){
		if( _inputRegisters[handle].isUsed ){
			_inputRegisters[handle].value = value;
			result =  MODBUS_OP_SUCCESS;
		}
		else{
			result = MODBUS_OP_INVALID_REGISTER;
		}
	}else{
		result = MODBUS_OP_INVALID_REGISTER;
	}
	return result;
}

ModbusOpResult ModbusSlave_GetInputRegister( ModbusRegister_Handle_t handle, uint16_t* value ){
	ModbusOpResult result = MODBUS_OP_SUCCESS;
	if( handle >= 0 && handle < MODBUS_SLAVE_INPUT_REGISTER_SIZE ){
		if( _inputRegisters[handle].isUsed ){
			*value =  _inputRegisters[handle].value;
			result =  MODBUS_OP_SUCCESS;
		}
		else{
			result = MODBUS_OP_INVALID_REGISTER;
		}
	}else{
		result = MODBUS_OP_INVALID_REGISTER;
	}
	return result;
}

ModbusOpResult ModbusSlave_GetInputRegisterByAddress( uint16_t address, uint16_t* value ){
	ModbusOpResult result = MODBUS_OP_INVALID_REGISTER;
	for( int k = 0; k < MODBUS_SLAVE_INPUT_REGISTER_SIZE; k++ ){
		if( _inputRegisters[k].isUsed && address == _inputRegisters[k].address ){
			*value = _inputRegisters[k].value;
			result = MODBUS_OP_SUCCESS;
			break;
		}
	}
	return result;
}
