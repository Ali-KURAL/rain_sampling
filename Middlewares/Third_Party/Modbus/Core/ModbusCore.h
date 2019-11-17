/*
 * ModbusCore.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Kaan
 */


#include "Modbus_Typedefs.h"


void ModbusCore_Process( const ModbusBuffer* inputPdu, ModbusBuffer* outputPdu );
void ModbusCore_ClearPdu( ModbusBuffer* inputPdu );
