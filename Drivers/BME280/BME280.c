/*
 * BME280.c
 *
 *  Created on: Dec 7, 2019
 *      Author: Kaan
 */


#include "BME280_RegisterMap.h"
#include "BME280.h"
#include <math.h>


 /*BSP Layer Functions */
uint8_t  	BME280_ReadRegister( const BME280_Config_t* config, uint8_t offset );
int16_t  	BME280_ReadRegisterInt16( const BME280_Config_t* config, uint8_t offset );

uint16_t  	BME280_Read16( const BME280_Config_t* config, uint8_t reg );
uint16_t  	BME280_Read16_LE( const BME280_Config_t* config, uint8_t reg );

int16_t 	BME280_ReadS16_LE( const BME280_Config_t* config, uint8_t reg );
int16_t    	BME280_ReadS16( const BME280_Config_t* config, uint8_t reg );

uint32_t 	BME280_ReadRegisterInt24( const BME280_Config_t* config, uint8_t reg );
//Writes a byte;
void 		BME280_WriteRegister( const BME280_Config_t* config, uint8_t offset, uint8_t dataToWrite );

//
void BME280_ReadCoefficients( const BME280_Config_t* config, BME280_CALIB_DATA *bme280_calib_str_ptr );

/*Holders */
BME280_CALIB_DATA bme280CalibData;
float BME280_Temperature, BME280_Humidity,BME280_Pressure,BME280_Alttitude;
int32_t t_fine;

BME280_Result BME280_Init( const BME280_Config_t* config ){
	uint8_t chipID = BME280_ReadRegister( config, BME280_REGISTER_CHIPID );
	if (chipID != 0x60){
		return BME280_FAILED;
	}

	BME280_ReadCoefficients( config, &bme280CalibData );
	//Set before CONTROL_meas (DS 5.4.3)
	BME280_WriteRegister(config, BME280_REGISTER_CONTROLHUMID, 0x05 ); //16x oversampling
	BME280_WriteRegister(config, BME280_REGISTER_CONTROL, 0xB7 ); // 16x ovesampling, normal mode
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadFloatPressure( const BME280_Config_t* config, float *pressure ){
	int64_t var1, var2, p;
	float temp_;
	BME280_ReadTempC( config, &temp_); // must be done first to get t_fine
	int32_t adc_P = BME280_ReadRegisterInt24(config, BME280_REGISTER_PRESSUREDATA );
	adc_P >>= 4;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bme280CalibData.dig_P6;
	var2 = var2 + ((var1*(int64_t)bme280CalibData.dig_P5)<<17);
	var2 = var2 + (((int64_t)bme280CalibData.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)bme280CalibData.dig_P3)>>8) +
	((var1 * (int64_t)bme280CalibData.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bme280CalibData.dig_P1)>>33;
	if (var1 == 0)
	{
		return 0;  // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p<<31) - var2)*3125) / var1;
	var1 = (((int64_t)bme280CalibData.dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)bme280CalibData.dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)bme280CalibData.dig_P7)<<4);
	BME280_Pressure = (float)p/256;
	(*pressure) = BME280_Pressure;
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadFloatAltitudeMeters( const BME280_Config_t* config, float *altitude ){
	float heightOutput = 0;
	heightOutput = ((float)-45846.2)*(pow(((float)BME280_ReadFloatPressure( config, &heightOutput)/(float)101325), 0.190263) - (float)1);
	(*altitude) = heightOutput;
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadFloatAltitudeFeet( const BME280_Config_t* config, float *altitude ){
	float heightOutput = 0;
	heightOutput = BME280_ReadFloatAltitudeMeters(config, &heightOutput) * 3.28084;
	(*altitude ) = heightOutput;
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadFloatHumidity( const BME280_Config_t* config, float *humidity ){
	float temp_ = 0;
	BME280_ReadTempC(config, &temp_); // must be done first to get t_fine
	int32_t adc_H = BME280_ReadRegisterInt16(config, BME280_REGISTER_HUMIDDATA);
	int32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme280CalibData.dig_H4) << 20) -
		  (((int32_t)bme280CalibData.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
		   (((((((v_x1_u32r * ((int32_t)bme280CalibData.dig_H6)) >> 10) *
			(((v_x1_u32r * ((int32_t)bme280CalibData.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
		  ((int32_t)2097152)) * ((int32_t)bme280CalibData.dig_H2) + 8192) >> 14));

	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
				 ((int32_t)bme280CalibData.dig_H1)) >> 4));

	v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
	float h = (v_x1_u32r>>12);
	BME280_Humidity =  h / 1024.0;
	( *humidity ) = BME280_Humidity;
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadTempC( const BME280_Config_t* config, float *temperature ){

	int32_t var1, var2;
	int32_t adc_T = BME280_ReadRegisterInt24( config, BME280_REGISTER_TEMPDATA );
	adc_T >>= 4;
	var1  = ((((adc_T>>3) - ((int32_t)bme280CalibData.dig_T1 <<1))) * ((int32_t)bme280CalibData.dig_T2)) >> 11;
	var2  = (((((adc_T>>4) - ((int32_t)bme280CalibData.dig_T1)) * ((adc_T>>4) - ((int32_t)bme280CalibData.dig_T1))) >> 12) * ((int32_t)bme280CalibData.dig_T3)) >> 14;
	t_fine = var1 + var2;

	float T  = (t_fine * 5 + 128) >> 8;
	BME280_Temperature = T/100;
	( *temperature ) = BME280_Temperature;

	return BME280_SUCCESS;
}


BME280_Result BME280_ReadTempF( const BME280_Config_t* config, float *temperature ){
	float outputTemp;
	float output = BME280_ReadTempC(config, &outputTemp);
	output = (output * 9) / 5 + 32;
	(*temperature ) = output;
	return BME280_SUCCESS;
}

/*Internal Level Functions */
void BME280_ReadCoefficients( const BME280_Config_t* config, BME280_CALIB_DATA *bme280_calib_str_ptr )
{
	bme280_calib_str_ptr->dig_T1 = BME280_Read16_LE( config, BME280_REGISTER_DIG_T1 );
	bme280_calib_str_ptr->dig_T2 = BME280_ReadS16_LE( config, BME280_REGISTER_DIG_T2 );
	bme280_calib_str_ptr->dig_T3 = BME280_ReadS16_LE( config, BME280_REGISTER_DIG_T3 );

	bme280_calib_str_ptr->dig_P1 = BME280_Read16_LE( config, BME280_REGISTER_DIG_P1 );
	bme280_calib_str_ptr->dig_P2 = BME280_ReadS16_LE( config, BME280_REGISTER_DIG_P2 );
	bme280_calib_str_ptr->dig_P3 = BME280_ReadS16_LE( config, BME280_REGISTER_DIG_P3 );
	bme280_calib_str_ptr->dig_P4 = BME280_ReadS16_LE( config, BME280_REGISTER_DIG_P4 );
	bme280_calib_str_ptr->dig_P5 = BME280_ReadS16_LE( config, BME280_REGISTER_DIG_P5 );
	bme280_calib_str_ptr->dig_P6 = BME280_ReadS16_LE( config, BME280_REGISTER_DIG_P6 );
	bme280_calib_str_ptr->dig_P7 = BME280_ReadS16_LE( config, BME280_REGISTER_DIG_P7 );
	bme280_calib_str_ptr->dig_P8 = BME280_ReadS16_LE( config, BME280_REGISTER_DIG_P8 );
	bme280_calib_str_ptr->dig_P9 = BME280_ReadS16_LE( config, BME280_REGISTER_DIG_P9 );

	bme280_calib_str_ptr->dig_H1 = BME280_ReadRegister( config, BME280_REGISTER_DIG_H1 );
	bme280_calib_str_ptr->dig_H2 = BME280_ReadS16_LE( config, BME280_REGISTER_DIG_H2 );
	bme280_calib_str_ptr->dig_H3 = BME280_ReadRegister( config, BME280_REGISTER_DIG_H3 );
	bme280_calib_str_ptr->dig_H4 = (BME280_ReadRegister( config, BME280_REGISTER_DIG_H4 ) << 4) | ( BME280_ReadRegister( config, BME280_REGISTER_DIG_H4+1 ) & 0xF );
	bme280_calib_str_ptr->dig_H5 = (BME280_ReadRegister( config, BME280_REGISTER_DIG_H5+1 ) << 4) | ( BME280_ReadRegister( config, BME280_REGISTER_DIG_H5 ) >> 4 );
	bme280_calib_str_ptr->dig_H6 = (int8_t)BME280_ReadRegister( config, BME280_REGISTER_DIG_H6 );
}


// BSP Layer Functions
uint8_t  	BME280_ReadRegister( const BME280_Config_t* config, uint8_t offset ){
	uint8_t value;
	uint8_t reg_=offset;
	config->write( config->id<<1, &reg_, 1, 1000);
	config->read( config->id<<1 ,&value, 1, 1000);
	return value;
}

int16_t  	BME280_ReadRegisterInt16( const BME280_Config_t* config, uint8_t offset ){
	uint8_t reg_ = offset;
	uint8_t value[2]={0,0};
	config->write( config->id<<1, &reg_, 1, 1000 );
	config->read( config->id<<1 ,value, 2, 1000 );
	return (value[0]<<8)|value[1];
}

uint16_t  	BME280_Read16( const BME280_Config_t* config, uint8_t reg ){
	uint8_t reg_ = reg;
	uint8_t value[2]={0,0};
	config->write( config->id<<1, &reg_, 1, 1000 );
	config->read( config->id<<1 , value, 2, 1000 );
	return (value[0]<<8)|value[1];
}

uint16_t  	BME280_Read16_LE( const BME280_Config_t* config, uint8_t reg ){
	uint16_t temp = BME280_Read16(config, reg );
	return (temp >> 8) | (temp << 8);
}

int16_t    	BME280_ReadS16( const BME280_Config_t* config, uint8_t reg ){
	return (int16_t)BME280_Read16( config, reg );
}

int16_t 	BME280_ReadS16_LE( const BME280_Config_t* config, uint8_t reg ){
	return (int16_t)BME280_Read16_LE(config, reg );
}

uint32_t 	BME280_ReadRegisterInt24( const BME280_Config_t* config, uint8_t reg ){
	uint8_t reg_ = reg;
	uint8_t value[3]={0,0,0};
	config->write( config->id<<1, &reg_, 1, 1000 );
	config->read( config->id<<1, value, 3, 1000);
	return (value[0]<<16)|(value[1]<<8)|value[2];
}

//Writes a byte;
void 		BME280_WriteRegister( const BME280_Config_t* config, uint8_t offset, uint8_t dataToWrite ){
	uint8_t data[2]={ offset, dataToWrite };
	config->write( config->id<<1, (uint8_t*) data, 2, 1000 );
}
