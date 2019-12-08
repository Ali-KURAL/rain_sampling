#ifndef MAX31865_TYPE_DEFS_H
#define MAX31865_TYPE_DEFS_H

#include <stdint.h>

typedef enum MAX31865_Result{
	MAX31865_SUCCESS = 0,
	MAX31865_FAILED = 1
}MAX31865_Result;

typedef union MAX31865_Value {
  float value;
  char array[4];
}MAX31865_Value;

typedef void( *MAX31865_Read )( uint8_t address, uint8_t* buffer, uint16_t length, uint32_t timeout );
typedef void( *MAX31865_Write )( uint8_t address, uint8_t* buffer, uint16_t length, uint32_t timeout );
typedef void( *MAX31865_Delay )( uint32_t timeout );

typedef struct MAX31865_Config_t{
	MAX31865_Read 			read;
	//MAX31865_ReadAsync 	readAsync;
	MAX31865_Write			write;
	//MAX31865_WriteAsync   writeAsync;
	MAX31865_Delay 			delay;
}MAX31865_Config_t;


typedef enum MAX31865_NumWires
{
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
}MAX31865_NumWires_t;


typedef enum MAX31865_DeviceNumber
{
  MAX31865_1 = 0,
  MAX31865_2 = 1,
  MAX31865_DeviceCount = 2
}MAX31865_DeviceNumber_t;


#endif