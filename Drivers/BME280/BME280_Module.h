/*
 * BME280_Module.h
 *
 *  Created on: Nov 17, 2019
 *      Author: Kaan
 */

#ifndef BME280_BME280_MODULE_H_
#define BME280_BME280_MODULE_H_

#include "BME280_ModuleDefs.h"

BME280_Result BME280_ReadFloatPressure( const  BME280_Handle_t* handle, float *pressure);
BME280_Result BME280_ReadFloatAltitudeMeters( const  BME280_Handle_t* handle, float *altitude );
BME280_Result BME280_ReadFloatAltitudeFeet( const  BME280_Handle_t* handle, float *altitude);
BME280_Result BME280_ReadFloatHumidity( const  BME280_Handle_t* handle, float *humidity);
BME280_Result BME280_ReadTempC( const  BME280_Handle_t* handle, float *temperature);
BME280_Result BME280_ReadTempF( const  BME280_Handle_t* handle , float *temperature);


#endif /* BME280_BME280_MODULE_H_ */
