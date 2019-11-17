/*
 * ModbusRtu_Slave.c
 *
 *  Created on: Nov 10, 2019
 *      Author: Kaan
 */
#include "ModbusRtu_Slave.h"
#include "../../../Modbus/Core/Modbus_Typedefs.h"
#include "../../../Modbus/Core/ModbusCore.h"
#include "../Common/ModbusRtu_Common.h"
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"

#define 					RTU_SLAVE_RX_TASK_PRIORITY 		2

// os definition
TaskHandle_t 				_modbusRtuTaskHandle;
SemaphoreHandle_t 			_rtuRxSmpHandle;
uint64_t 					_lastPackageTick;
uint8_t 					_rxByte;

ModbusRtu_Config_t 			_config;
ModbusBuffer 				_rtuRxBuffer;
int 						expectedFrameSize = 0;

extern UART_HandleTypeDef huart3;

// private
void 	ModbusRtu_Slave_ReceiveTask(void *argument);
void 	ModbusRtu_Slave_Receive( uint8_t rxByte );
void 	ModbusRtu_Slave_Transmit( const ModbusRtuMessage_t* outputMsgStr );
int 	ModbusRtu_Slave_CalculateRxBufferSize( const ModbusBuffer *buffer );

int ModbusRtu_Slave_Init( const ModbusRtu_Config_t* config_t  ){
	_config.id = config_t->id;
	_config.send = config_t->send;
	_config.requestRead = config_t->requestRead;
	_rtuRxBuffer.index = 0;
	_rtuRxSmpHandle = xSemaphoreCreateBinary();
	BaseType_t xReturned = xTaskCreate(  ModbusRtu_Slave_ReceiveTask,
										 (const char* const) "RTU_RX_TASK",
										 512,
										 NULL,
										 RTU_SLAVE_RX_TASK_PRIORITY,
										 &_modbusRtuTaskHandle
									);

	_config.requestRead(&_rxByte,1);


	if( xReturned != pdPASS )
		return -1;
	else
		return 1;
}

const ModbusBuffer* ModbusRtu_Slave_GetBuffer(void){
	return &(_rtuRxBuffer);
}

void ModbusRtu_Slave_ReceiveTask(void *argument){
	ModbusRtuMessage_t receivedMsgStr;
	ModbusRtuMessage_t responseMsgStr;
	while( true ){
		if( xSemaphoreTake( _rtuRxSmpHandle, (TickType_t) 255 ) == pdTRUE ){
			// Modbus ID check
			if( _rtuRxBuffer.index > 6 && _rtuRxBuffer.buffer[0] == _config.id ){
				// Modbus RTU Message Generation
				ModbusCore_ClearPdu(&(receivedMsgStr.pdu));
				ModbusCore_ClearPdu(&(responseMsgStr.pdu));
				ModbusRtu_ConstructStruct( &_rtuRxBuffer, &receivedMsgStr );
				if( ModbusRtu_CheckCRC( &receivedMsgStr )){
					ModbusCore_Process( &(receivedMsgStr.pdu), &(responseMsgStr.pdu ) );
					responseMsgStr.id = _config.id;
					ModbusRtu_CreateCRC( &responseMsgStr );
					// reset input buffer..
					_rtuRxBuffer.index = 0;
					// transmit message
					ModbusRtu_Slave_Transmit( &responseMsgStr );
				}
			}
		}
	}
}

void ModbusRtu_Slave_onReceive(void){
	ModbusRtu_Slave_Receive(_rxByte);
}

void ModbusRtu_Slave_Receive( uint8_t rxByte ){
	static BaseType_t xHigherPriorityTaskWoken;
	if( uwTick> ( _lastPackageTick + 500 ) )
	{
		/* The time gap between last received package is higher than 500ms.
		 * Lets restart buffer index.
		 * */
		_rtuRxBuffer.index = 0;
	}
	_rtuRxBuffer.buffer[_rtuRxBuffer.index++] = rxByte;
	xHigherPriorityTaskWoken = pdFALSE;
	if( _rtuRxBuffer.index == 8 ){
		expectedFrameSize = ModbusRtu_Slave_CalculateRxBufferSize( &_rtuRxBuffer );
	}

	if ( _rtuRxBuffer.index == expectedFrameSize ){
		xSemaphoreGiveFromISR( _rtuRxSmpHandle, &xHigherPriorityTaskWoken );
	}

	_lastPackageTick = uwTick;
	_config.requestRead(&_rxByte,1);
}

void ModbusRtu_Slave_Transmit( const ModbusRtuMessage_t* outputMsgStr ){
	if( _config.send != NULL && outputMsgStr != NULL ){
		ModbusBuffer _tempBuffer;
		_tempBuffer.index = 0;
		_tempBuffer.buffer[_tempBuffer.index++] = outputMsgStr->id;
		for( int k = 0; k < outputMsgStr->pdu.index; k++ ){
			_tempBuffer.buffer[_tempBuffer.index++] = outputMsgStr->pdu.buffer[k];
		}
		_tempBuffer.buffer[_tempBuffer.index++] = outputMsgStr->crc >> 8;
		_tempBuffer.buffer[_tempBuffer.index++] = outputMsgStr->crc & 0xFF;
		_config.send(_tempBuffer.buffer, _tempBuffer.index );
		//HAL_UART_Transmit(&huart3, _tempBuffer.buffer, _tempBuffer.index, 1000 );
	}
}

int ModbusRtu_Slave_CalculateRxBufferSize( const ModbusBuffer *buffer ){
	int expectedFrameSize = 0;
	if( buffer->index < 8 ) {
		return -1;
	}
	switch( buffer->buffer[1] ){
		case READ_COIL_STATUS:
		case READ_INPUT_STATUS:
		case READ_HOLDING_REGISTERS:
		case READ_INPUT_REGISTERS:
		case PRESET_SINGLE_REGISTER:
			expectedFrameSize = 8;
			break;
		case FORCE_MULTIPLE_COILS:
			//kaanbak: burada usendim hesaplamaya ilerde bakariz
			expectedFrameSize = buffer->buffer[5] / 8 + 9;
		case PRESET_MULTIPLE_REGISTERS:
			expectedFrameSize = buffer->buffer[5] * 2 + 9;
			break;
		default:
			expectedFrameSize = 8;
			break;
	}
	return expectedFrameSize;
}





