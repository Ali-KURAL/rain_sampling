/*
 * BME280_Module.c
 *
 *  Created on: Nov 17, 2019
 *      Author: Kaan
 */
#include "BME280_Module.h"
#include "BME280_RegisterMap.h"

BME280_Result BME280_ReadCoefficients( const  BME280_Handle_t* handle, BME280_CALIB_DATA *bme280_calib_str_ptr );

BME280_Result BME280_ReadRegister( const  BME280_Handle_t* handle, uint8_t reg, uint8_t* value  );
BME280_Result BME280_Read16( const  BME280_Handle_t* handle, uint8_t reg, uint16_t* value );
BME280_Result BME280_Read16_LE( const  BME280_Handle_t* handle, uint8_t reg, uint16_t* value );
BME280_Result BME280_ReadS16( const  BME280_Handle_t* handle, uint8_t reg, int16_t* value );
BME280_Result BME280_ReadS16_LE(const  BME280_Handle_t* handle, uint8_t reg, int16_t* value );
BME280_Result BME280_ReadRegisterInt24(const  BME280_Handle_t* handle, uint8_t reg );

BME280_Result BME280_Init( BME280_Handle_t* handle ){
	/*uint8_t chipID = 0;
	handle->read( BME280_REGISTER_CHIPID, 1, &chipID );
	if (chipID != 0x60)
	{
		return BME280_NOT_EXIST;
	}
	BME280_ReadCoefficients(handle, &bme280CalibData);
	//Set before CONTROL_meas (DS 5.4.3)
	handle->write( BME280_REGISTER_CONTROLHUMID, 0x05 ); //16x oversampling
	handle->write( BME280_REGISTER_CONTROL, 0xB7 ); // 16x ovesampling, normal mode*/
	return BME280_SUCCESS;
}


BME280_Result BME280_ReadFloatPressure( const  BME280_Handle_t* handle, float *pressure){
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadFloatAltitudeMeters( const  BME280_Handle_t* handle, float *altitude ){
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadFloatAltitudeFeet( const  BME280_Handle_t* handle, float *altitude){
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadFloatHumidity( const  BME280_Handle_t* handle, float *humidity){
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadTempC( const  BME280_Handle_t* handle, float *temperature){
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadTempF( const  BME280_Handle_t* handle, float *temperature){
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadCoefficients( const  BME280_Handle_t* handle, BME280_CALIB_DATA *bme280_calib_str_ptr )
{
	BME280_Read16_LE( handle, BME280_REGISTER_DIG_T1, &(bme280_calib_str_ptr->dig_T1));
	BME280_ReadS16_LE( handle, BME280_REGISTER_DIG_T2,&(bme280_calib_str_ptr->dig_T2));
	BME280_ReadS16_LE( handle, BME280_REGISTER_DIG_T3,&(bme280_calib_str_ptr->dig_T3));

	/*bme280_calib_str_ptr->dig_P1 = BME280_Read16_LE( handle, BME280_REGISTER_DIG_P1);
	bme280_calib_str_ptr->dig_P2 = BME280_ReadS16_LE( handle, BME280_REGISTER_DIG_P2);
	bme280_calib_str_ptr->dig_P3 = BME280_ReadS16_LE( handle, BME280_REGISTER_DIG_P3);
	bme280_calib_str_ptr->dig_P4 = BME280_ReadS16_LE( handle, BME280_REGISTER_DIG_P4);
	bme280_calib_str_ptr->dig_P5 = BME280_ReadS16_LE( handle, BME280_REGISTER_DIG_P5);
	bme280_calib_str_ptr->dig_P6 = BME280_ReadS16_LE( handle, BME280_REGISTER_DIG_P6);
	bme280_calib_str_ptr->dig_P7 = BME280_ReadS16_LE( handle, BME280_REGISTER_DIG_P7);
	bme280_calib_str_ptr->dig_P8 = BME280_ReadS16_LE( handle, BME280_REGISTER_DIG_P8);
	bme280_calib_str_ptr->dig_P9 = BME280_ReadS16_LE( handle, BME280_REGISTER_DIG_P9);

	bme280_calib_str_ptr->dig_H1 = BME280_ReadRegister( handle, BME280_REGISTER_DIG_H1);
	bme280_calib_str_ptr->dig_H2 = BME280_ReadS16_LE( handle, BME280_REGISTER_DIG_H2);
	bme280_calib_str_ptr->dig_H3 = BME280_ReadRegister( handle, BME280_REGISTER_DIG_H3);
	bme280_calib_str_ptr->dig_H4 = (BME280_ReadRegister( handle, BME280_REGISTER_DIG_H4) << 4) | ( BME280_ReadRegister( handle, BME280_REGISTER_DIG_H4+1 ) & 0xF );
	bme280_calib_str_ptr->dig_H5 = (BME280_ReadRegister( handle, BME280_REGISTER_DIG_H5+1) << 4) | ( BME280_ReadRegister( handle, BME280_REGISTER_DIG_H5 ) >> 4 );
	bme280_calib_str_ptr->dig_H6 = (int8_t)BME280_ReadRegister( handle, BME280_REGISTER_DIG_H6);*/
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadRegister( const  BME280_Handle_t* handle, uint8_t reg, uint8_t* value  ){
	handle->read(reg, value, 1 );
	return BME280_SUCCESS;
}

BME280_Result BME280_Read16( const  BME280_Handle_t* handle, uint8_t reg, uint16_t* value )
{
	handle->read(reg, value, 2 );
	return BME280_SUCCESS;
}

BME280_Result BME280_Read16_LE( const  BME280_Handle_t* handle, uint8_t reg, uint16_t* value )
{
	BME280_Read16( handle, reg, value );
	*value =  ( *value >> 8) | (*value << 8);
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadS16( const  BME280_Handle_t* handle, uint8_t reg, int16_t* value )
{
	BME280_Read16(handle, reg, (uint16_t*)value );
	*value = (int16_t)*value;
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadS16_LE(const  BME280_Handle_t* handle, uint8_t reg, int16_t* value )
{
	BME280_Read16_LE( handle, reg, (uint16_t*)value );
	*value = (int16_t)*value;
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadRegisterInt24(const  BME280_Handle_t* handle, uint8_t reg )
{
	uint8_t reg_ = reg;
	uint8_t value[3]={0,0,0};
	return BME280_SUCCESS;
}

