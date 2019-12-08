/*
 * BME280.h
 *
 *  Created on: Dec 7, 2019
 *      Author: Kaan
 */

#ifndef BME280_BME280_H_
#define BME280_BME280_H_

#include "BME280_TypeDefs.h"

BME280_Result BME280_Init( const BME280_Config_t* config );

// SYNC API
BME280_Result BME280_ReadFloatPressure( const BME280_Config_t* config, float *pressure );
BME280_Result BME280_ReadFloatAltitudeMeters( const BME280_Config_t* config, float *altitude );
BME280_Result BME280_ReadFloatAltitudeFeet( const BME280_Config_t* config, float *altitude );
BME280_Result BME280_ReadFloatHumidity( const BME280_Config_t* config, float *humidity );
BME280_Result BME280_ReadTempC( const BME280_Config_t* config, float *temperature );
BME280_Result BME280_ReadTempF( const BME280_Config_t* config, float *temperature );


// ASYNC API

#endif /* BME280_BME280_H_ */
