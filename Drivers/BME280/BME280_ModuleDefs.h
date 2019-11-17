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
	BME280_SUCCESS = 0
}BME280_Result;

typedef BME280_Result(*BME280_ReadTemperature)(float *value);
typedef BME280_Result(*BME280_ReadHumidty)(float *value);
typedef BME280_Result(*BME280_ReadAirPressure)(uint16_t *value);
typedef void(*BME280_I2CRead)(void);
typedef void(*BME280_I2CWrite)(void);

typedef struct BME280_Handle_t{

}BME280_Handle_t;




#endif /* BME280_BME280_MODULEDEFS_H_ */
