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

ModbusOpResult ModbusSlave_SetCoilStatus( ModbusRegister_Handle_t handle, bool value );
ModbusOpResult ModbusSlave_GetCoilStatus( ModbusRegister_Handle_t handle, bool* value );
ModbusOpResult ModbusSlave_GetCoilStatusByAddress( uint16_t address, bool* value );

ModbusOpResult ModbusSlave_SetInputStatus( ModbusRegister_Handle_t handle, bool value );
ModbusOpResult ModbusSlave_GetInputStatus( ModbusRegister_Handle_t handle, bool* value );
ModbusOpResult ModbusSlave_GetInputStatusByAddress( uint16_t address, bool* value );

ModbusOpResult ModbusSlave_SetHoldingRegister( ModbusRegister_Handle_t handle, uint16_t value );
ModbusOpResult ModbusSlave_GetHoldingRegister( ModbusRegister_Handle_t handle, uint16_t* value );
ModbusOpResult ModbusSlave_GetHoldingRegisterByAddress( uint16_t address, uint16_t* value );

ModbusOpResult ModbusSlave_SetInputRegister( ModbusRegister_Handle_t handle, uint16_t value );
ModbusOpResult ModbusSlave_GetInputRegister( ModbusRegister_Handle_t handle, uint16_t* value );
ModbusOpResult ModbusSlave_GetInputRegisterByAddress( uint16_t address, uint16_t* value );
#endif /* THIRD_PARTY_MODBUS_CORE_MODBUSAPP_H_ */
