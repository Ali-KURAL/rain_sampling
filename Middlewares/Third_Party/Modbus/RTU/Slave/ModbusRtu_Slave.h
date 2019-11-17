/*
 * ModbusRtu_Slave.h
 *
 *  Created on: Nov 10, 2019
 *      Author: Kaan
 */

#ifndef THIRD_PARTY_MODBUS_RTU_SLAVE_MODBUSRTU_SLAVE_H_
#define THIRD_PARTY_MODBUS_RTU_SLAVE_MODBUSRTU_SLAVE_H_

#include <stdint.h>
#include "../../../Modbus/Core/Modbus_Typedefs.h"


typedef void(*ModbusRtu_InitCb)();
typedef void(*ModbusRtu_Transmit)( uint8_t* buffer, uint16_t length );
typedef void(*ModbusRtu_ReadRequest)( uint8_t* buffer, uint16_t length );


typedef struct ModbusRtu_Config_t{
	ModbusID id;
	ModbusBaudRate baudRate;
	ModbusParity parity;
	ModbusRtu_InitCb 			init;
	ModbusRtu_Transmit 			send;
	ModbusRtu_ReadRequest		requestRead;
}ModbusRtu_Config_t;

int 	ModbusRtu_Slave_Init( const ModbusRtu_Config_t* config_t );
void 	ModbusRtu_Slave_Receive( uint8_t rxByte );
void 	ModbusRtu_Slave_onReceive(void);
const ModbusBuffer* ModbusRtu_Slave_GetBuffer(void);


#endif /* THIRD_PARTY_MODBUS_RTU_SLAVE_MODBUSRTU_SLAVE_H_ */
