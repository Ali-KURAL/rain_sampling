/*
 * BME280.c
 *
 *  Created on: Dec 7, 2019
 *      Author: Kaan
 */


#include "BME280_RegisterMap.h"
#include "BME280.h"



 /*BSP Layer Functions */
uint8_t  	BME280_ReadRegister( const BME280_Config_t* config, uint8_t offset );
uint16_t  	BME280_ReadRegisterInt16( const BME280_Config_t* config, uint8_t offset );
uint16_t  	BME280_Read16_LE( const BME280_Config_t* config, uint8_t reg );
int16_t 	BME280_ReadS16_LE( const BME280_Config_t* config, uint8_t reg );
int16_t    	BME280_ReadS16( const BME280_Config_t* config, uint8_t reg );
uint32_t 	BME280_ReadRegisterInt24( const BME280_Config_t* config, uint8_t reg );
//Writes a byte;
void 		BME280_WriteRegister( const BME280_Config_t* config, uint8_t offset, uint8_t dataToWrite );



BME280_Result BME280_Init( const BME280_Config_t* config ){
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadFloatPressure( const BME280_Config_t* config, float *pressure ){
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadFloatAltitudeMeters( const BME280_Config_t* config, float *altitude ){
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadFloatAltitudeFeet( const BME280_Config_t* config, float *altitude ){
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadFloatHumidity( const BME280_Config_t* config, float *humidity ){
	return BME280_SUCCESS;
}

BME280_Result BME280_ReadTempC( const BME280_Config_t* config, float *temperature ){
	return BME280_SUCCESS;
}


BME280_Result BME280_ReadTempF( const BME280_Config_t* config, float *temperature ){
	return BME280_SUCCESS;
}


// BSP Layer Functions
uint8_t  	BME280_ReadRegister( const BME280_Config_t* config, uint8_t offset ){
	return 0;
}

uint16_t  	BME280_ReadRegisterInt16( const BME280_Config_t* config, uint8_t offset ){
	return 0;
}

uint16_t  	BME280_Read16_LE( const BME280_Config_t* config, uint8_t reg ){
	return 0;
}

int16_t 	BME280_ReadS16_LE( const BME280_Config_t* config, uint8_t reg ){
	return 0;
}

int16_t    	BME280_ReadS16( const BME280_Config_t* config, uint8_t reg ){
	return 0;
}

uint32_t 	BME280_ReadRegisterInt24( const BME280_Config_t* config, uint8_t reg ){
	return 0;
}

//Writes a byte;
void 		BME280_WriteRegister( const BME280_Config_t* config, uint8_t offset, uint8_t dataToWrite ){
	uint8_t data[2]={ offset, dataToWrite };
	config->write( config->id<<1, (uint8_t*) data, 2, 1000 );
}
