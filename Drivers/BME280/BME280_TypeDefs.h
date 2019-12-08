/*
 * BME280_TypeDefs.h
 *
 *  Created on: Dec 7, 2019
 *      Author: Kaan
 */

#ifndef BME280_BME280_TYPEDEFS_H_
#define BME280_BME280_TYPEDEFS_H_

#include <stdint.h>

typedef uint8_t BME280_Id;

typedef enum BME280_Result{
	BME280_SUCCESS = 0,
	BME280_FAILED = 1
}BME280_Result;

typedef union BME280_Value {
  float value;
  char array[4];
}BME280_Value;

typedef void(*BME280_Read )( uint8_t i2c_address, uint8_t* buffer, uint16_t length, uint32_t timeout );
typedef void(*BME280_Write)( uint8_t i2c_address, uint8_t* buffer, uint16_t length, uint32_t timeout );

typedef struct BME280_Config_t{
	BME280_Id 			id;
	BME280_Read 		read;
	//BME280_ReadAsync 	readAsync;
	BME280_Write		write;
	//BME280_WriteAsync   writeAsync;
}BME280_Config_t;



#endif /* BME280_BME280_TYPEDEFS_H_ */
