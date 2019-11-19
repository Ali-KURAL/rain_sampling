/*
 * ModbusCore.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Kaan
 */


#include "ModbusCore.h"
#include "ModbusApp.h"
#include <string.h>

// private function definitions
void ModbusCore_InvalidOperation( ModbusMsgTypes type, ModbusBuffer* outputPdu );

void ModbusCore_ReadCoilStatus( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu );
void ModbusCore_ReadInputStatus( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu );
void ModbusCore_ReadHoldingRegisters( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu );
void ModbusCore_ReadInputRegisters( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu );
void ModbusCore_WriteSingleCoil( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu );
void ModbusCore_WriteSingleHoldingRegister( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu );

void ModbusCore_Process( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu ){
	ModbusMsgTypes rxMsgType = (ModbusMsgTypes)inputPdu->buffer[0];
	switch(rxMsgType){
		case READ_COIL_STATUS:
			ModbusCore_ReadCoilStatus( inputPdu, outputPdu );
			break;
		case READ_INPUT_STATUS:
			ModbusCore_ReadInputStatus( inputPdu, outputPdu );
			break;
		case READ_HOLDING_REGISTERS:
			ModbusCore_ReadHoldingRegisters( inputPdu, outputPdu );
			break;
		case READ_INPUT_REGISTERS:
			ModbusCore_ReadInputRegisters( inputPdu, outputPdu );
			break;
		case FORCE_SINGLE_COIL:
			ModbusCore_WriteSingleCoil( inputPdu, outputPdu );
			break;
		case FORCE_MULTIPLE_COILS:
			ModbusCore_InvalidOperation( rxMsgType, outputPdu );
			break;
		case PRESET_SINGLE_REGISTER:
			ModbusCore_WriteSingleHoldingRegister( inputPdu, outputPdu );
			break;
		case PRESET_MULTIPLE_REGISTERS:
			ModbusCore_InvalidOperation( rxMsgType, outputPdu );
			break;
		default:
			ModbusCore_InvalidOperation( rxMsgType, outputPdu );
			break;
	}
}

void ModbusCore_InvalidOperation(  ModbusMsgTypes type, ModbusBuffer* outputPdu ){
	outputPdu->index = 0;
	outputPdu->buffer[outputPdu->index++] = ( type | 0x80 );
	outputPdu->buffer[outputPdu->index++] = MODBUS_ILLEGAL_FUNCTION;
}

void ModbusCore_ReadCoilStatus( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu ){
	uint16_t startRegAddress = ( inputPdu->buffer[1] << 8 ) | inputPdu->buffer[2];
	uint16_t totalRequestedRegCount = ( inputPdu->buffer[3] << 8 ) | inputPdu->buffer[4];
	outputPdu->index = 0;
	outputPdu->buffer[outputPdu->index++] = inputPdu->buffer[0];
	outputPdu->buffer[outputPdu->index++] = totalRequestedRegCount / 8;
	uint8_t success = true;
	if( totalRequestedRegCount % 8 > 0 ){
		outputPdu->buffer[1] += 1;
	}

	for( int k = 0; k < totalRequestedRegCount; k++ ){
		uint16_t _value = 0;
		ModbusOpResult _result = ModbusSlave_GetRegisterValueByAddress( COIL_STATUS, startRegAddress+k, &_value );
		if( _result == MODBUS_OP_SUCCESS ){
			outputPdu->buffer[outputPdu->index + ( k / 8 )] |= ( _value << ( k % 8 ) );
		}else{
			// illegal data address
			success = false;
		}
	}
	// calculate length at the end
	outputPdu->index = outputPdu->buffer[1] + 2;
	if( !success ){
		outputPdu->index = 0;
		outputPdu->buffer[outputPdu->index++] |= 0x80;
		outputPdu->buffer[outputPdu->index++] = MODBUS_ILLEGAL_DATA_ADDRESS;
	}
}

void ModbusCore_ReadInputStatus( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu ){
	uint16_t startRegAddress = ( inputPdu->buffer[1] << 8 ) | inputPdu->buffer[2];
	uint16_t totalRequestedRegCount = ( inputPdu->buffer[3] << 8 ) | inputPdu->buffer[4];
	outputPdu->index = 0;
	outputPdu->buffer[outputPdu->index++] = inputPdu->buffer[0];
	outputPdu->buffer[outputPdu->index++] = totalRequestedRegCount / 8;
	uint8_t success = true;
	if( totalRequestedRegCount % 8 > 0 ){
		outputPdu->buffer[1] += 1;
	}

	for( int k = 0; k < totalRequestedRegCount; k++ ){
		uint16_t _value = 0;
		ModbusOpResult _result = ModbusSlave_GetRegisterValueByAddress( INPUT_STATUS, startRegAddress+k, &_value );
		if( _result == MODBUS_OP_SUCCESS ){
			outputPdu->buffer[outputPdu->index + ( k / 8 )] |= ( _value << ( k % 8 ) );
		}else{
			// illegal data address
			success = false;
		}
	}
	// calculate length at the end
	outputPdu->index = outputPdu->buffer[1] + 2;
	if( !success ){
		outputPdu->index = 0;
		outputPdu->buffer[outputPdu->index++] |= 0x80;
		outputPdu->buffer[outputPdu->index++] = MODBUS_ILLEGAL_DATA_ADDRESS;
	}
}

void ModbusCore_ReadHoldingRegisters( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu ){
	uint16_t startRegAddress = ( inputPdu->buffer[1] << 8 ) | inputPdu->buffer[2];
	uint16_t totalRequestedRegCount = ( inputPdu->buffer[3] << 8 ) | inputPdu->buffer[4];
	outputPdu->index = 0;
	outputPdu->buffer[outputPdu->index++] = inputPdu->buffer[0];
	outputPdu->buffer[outputPdu->index++] = totalRequestedRegCount * 2;
	uint8_t success = true;
	for(int k = 0; k < totalRequestedRegCount; k++ ){
		uint16_t _value = 0;
		ModbusOpResult _result = ModbusSlave_GetRegisterValueByAddress( HOLDING_REGISTER, startRegAddress+k, &_value );
		if( _result == MODBUS_OP_SUCCESS ){
			outputPdu->buffer[outputPdu->index++] = _value >> 8;
			outputPdu->buffer[outputPdu->index++] = _value & 0xFF;
		}else{
			success = false;
		}
	}
	if( !success ){
		outputPdu->index = 0;
		outputPdu->buffer[outputPdu->index++] |= 0x80;
		outputPdu->buffer[outputPdu->index++] = MODBUS_ILLEGAL_DATA_ADDRESS;
	}
}

void ModbusCore_ReadInputRegisters( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu ){
	uint16_t startRegAddress = ( inputPdu->buffer[1] << 8 ) | inputPdu->buffer[2];
	uint16_t totalRequestedRegCount = ( inputPdu->buffer[3] << 8 ) | inputPdu->buffer[4];
	outputPdu->index = 0;
	outputPdu->buffer[outputPdu->index++] = inputPdu->buffer[0];
	outputPdu->buffer[outputPdu->index++] = totalRequestedRegCount * 2;
	uint8_t success = true;
	for(int k = 0; k < totalRequestedRegCount; k++ ){
		uint16_t _value = 0;
		ModbusOpResult _result = ModbusSlave_GetRegisterValueByAddress( INPUT_REGISTER, startRegAddress+k, &_value );
		if( _result == MODBUS_OP_SUCCESS ){
			outputPdu->buffer[outputPdu->index++] = _value >> 8;
			outputPdu->buffer[outputPdu->index++] = _value & 0xFF;
		}else{
			success = false;
		}
	}
	//
	if( !success ){
		outputPdu->index = 0;
		outputPdu->buffer[outputPdu->index++] |= 0x80;
		outputPdu->buffer[outputPdu->index++] = MODBUS_ILLEGAL_DATA_ADDRESS;
	}
}

void ModbusCore_WriteSingleCoil( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu ){
	uint16_t coilAddress = ( inputPdu->buffer[1] << 8 ) | inputPdu->buffer[2];
	uint16_t statusToWrite = ( inputPdu->buffer[3] << 8 ) | inputPdu->buffer[4];
	uint16_t writeValue = 0;

	if( statusToWrite == 0xFF00 ){
		writeValue = 1;
	}else if( statusToWrite == 0x0000 ){
		writeValue = 0;
	}else
	{
		outputPdu->index = 0;
		outputPdu->buffer[outputPdu->index++] |= 0x80;
		outputPdu->buffer[outputPdu->index++] = MODBUS_ILLEGAL_DATA_ADDRESS;
		return;
	}
	ModbusOpResult _result = ModbusSlave_SetRegisterValueByAddress( COIL_STATUS, coilAddress, writeValue );
	if( _result == MODBUS_OP_SUCCESS ){
		memcpy(outputPdu->buffer,inputPdu->buffer,inputPdu->index);
		outputPdu->index = inputPdu->index;
	}else{
		outputPdu->index = 0;
		outputPdu->buffer[outputPdu->index++] |= 0x80;
		outputPdu->buffer[outputPdu->index++] = MODBUS_ILLEGAL_DATA_ADDRESS;
	}
}

void ModbusCore_WriteSingleHoldingRegister( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu ){
	uint16_t coilAddress = ( inputPdu->buffer[1] << 8 ) | inputPdu->buffer[2];
	uint16_t valueToWrite = ( inputPdu->buffer[3] << 8 ) | inputPdu->buffer[4];

	ModbusOpResult _result = ModbusSlave_SetRegisterValueByAddress( HOLDING_REGISTER, coilAddress, valueToWrite );
	if( _result == MODBUS_OP_SUCCESS ){
		memcpy(outputPdu->buffer,inputPdu->buffer,inputPdu->index);
		outputPdu->index = inputPdu->index;
	}else{
		outputPdu->index = 0;
		outputPdu->buffer[outputPdu->index++] |= 0x80;
		outputPdu->buffer[outputPdu->index++] = MODBUS_ILLEGAL_DATA_ADDRESS;
	}
}


void ModbusCore_ClearPdu( ModbusBuffer* inputPdu ){
	memset(inputPdu->buffer,0,MODBUS_RTU_RX_BUFFER_SIZE );
	inputPdu->index = 0;
}
