/*
 * BME280_ModuleDefs.h
 *
 *  Created on: Nov 17, 2019
 *      Author: Kaan
 */

#ifndef BME280_BME280_MODULEDEFS_H_
#define BME280_BME280_MODULEDEFS_H_

#include <stdint.h>

typedef enum BME280_Result{
	BME280_SUCCESS = 0,
	BME280_NOT_EXIST = 1
}BME280_Result;

typedef uint8_t(*BME280_Read)(uint16_t reg, uint8_t* value, uint8_t size );
typedef struct BME280_Handle_t{
	BME280_Read read;
}BME280_Handle_t;





#endif /* BME280_BME280_MODULEDEFS_H_ */
