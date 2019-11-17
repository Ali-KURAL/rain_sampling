/*
 * ModbusApp.h
 *
 *  Created on: Nov 15, 2019
 *      Author: Kaan
 */

#ifndef THIRD_PARTY_MODBUS_CORE_MODBUSAPP_H_
#define THIRD_PARTY_MODBUS_CORE_MODBUSAPP_H_

#include "Modbus_Typedefs.h"

ModbusRegister_Handle_t ModbusSlave_CreateCoilStatus( uint16_t address, bool value );
ModbusRegister_Handle_t ModbusSlave_CreateInputStatus( uint16_t address, bool value );
ModbusRegister_Handle_t ModbusSlave_CreateHoldingRegister( uint16_t address, uint16_t value );
ModbusRegister_Handle_t ModbusSlave_CreateInputRegister( uint16_t address, uint16_t value );

ModbusOpResult ModbusSlave_AddOnReadCallback( const ModbusRegister_Handle_t* handle, ModbusRegisterRead readCallback );
ModbusOpResult ModbusSlave_ClearOnReadCallback( const ModbusRegister_Handle_t* handle );
ModbusOpResult ModbusSlave_AddOnWriteCallback( const ModbusRegister_Handle_t* handle, ModbusRegisterWrite writeCallback );
ModbusOpResult ModbusSlave_ClearOnWriteCallback( const ModbusRegister_Handle_t* handle );

ModbusOpResult ModbusSlave_SetRegisterValue( const ModbusRegister_Handle_t* handle, uint16_t value );
ModbusOpResult ModbusSlave_SetRegisterValueByAddress( ModbusRegisterType type, uint16_t address, uint16_t value );
ModbusOpResult ModbusSlave_GetRegisterValue( const ModbusRegister_Handle_t* handle, uint16_t* value );
ModbusOpResult ModbusSlave_GetRegisterValueByAddress( ModbusRegisterType type, uint16_t address, uint16_t* value );

#endif /* THIRD_PARTY_MODBUS_CORE_MODBUSAPP_H_ */
