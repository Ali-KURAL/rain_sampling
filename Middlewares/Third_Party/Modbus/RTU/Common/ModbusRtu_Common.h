/*
 * ModbusRtu_Common.h
 *
 *  Created on: Nov 12, 2019
 *      Author: Kaan
 */

#ifndef THIRD_PARTY_MODBUS_RTU_COMMON_MODBUSRTU_COMMON_H_
#define THIRD_PARTY_MODBUS_RTU_COMMON_MODBUSRTU_COMMON_H_

#include "../../../Modbus/Core/Modbus_Typedefs.h"

typedef struct ModbusRtuMessage_t{
	ModbusID id;
	ModbusBuffer pdu;
	uint16_t crc;
}ModbusRtuMessage_t;

bool ModbusRtu_CheckCRC( const ModbusRtuMessage_t* rtuMsgStruct );
void ModbusRtu_CreateCRC(  ModbusRtuMessage_t* rtuMsgStruct );
void ModbusRtu_ConstructStruct( const ModbusBuffer* inputBuffer, ModbusRtuMessage_t* outputStr );
void ModbusRtu_ConstructBuffer( const ModbusRtuMessage_t* inputStr, ModbusBuffer* outputBuffer );

#endif /* THIRD_PARTY_MODBUS_RTU_COMMON_MODBUSRTU_COMMON_H_ */
