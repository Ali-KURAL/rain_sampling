#include "MAX31865.h"

/*Internal Functions */
uint16_t 	MAX31865_ReadRTD( const MAX31865_Config_t* dev );

/*BSP Functions*/
void 		MAX31865_ReadRegisterN( const MAX31865_Config_t* dev,uint8_t reg, uint8_t buffer[], uint8_t n );
uint8_t  	MAX31865_ReadRegister8( const MAX31865_Config_t* dev, uint8_t reg );
uint16_t   	MAX31865_ReadRegister16( const MAX31865_Config_t* dev, uint8_t reg );
void    	MAX31865_WriteRegister8( const MAX31865_Config_t* dev, uint8_t reg, uint8_t data );

bool MAX31865_Begin( const MAX31865_Config_t* dev, MAX31865_NumWires_t wires){
	MAX31865_SetWires( dev, wires );
	MAX31865_EnableBias( dev, false );
	MAX31865_AutoConvert( dev, false );
	MAX31865_ClearFault( dev );
	// read some read only reg?
	return true;
}		

uint8_t MAX31865_ReadFault( const MAX31865_Config_t* dev ){
	uint8_t retVal = 0x00;
	retVal =  MAX31865_ReadRegister8( dev ,MAX31856_FAULTSTAT_REG );
	return retVal;
}		
		
void MAX31865_ClearFault( const MAX31865_Config_t* dev ){
	uint8_t t = MAX31865_ReadRegister8( dev, MAX31856_CONFIG_REG );
	t &= ~0x2C;
	t |= MAX31856_CONFIG_FAULTSTAT;
	MAX31865_WriteRegister8( dev, MAX31856_CONFIG_REG, t );
}		
		
void MAX31865_SetWires( const MAX31865_Config_t* dev, MAX31865_NumWires_t wires ){
	uint8_t t = MAX31865_ReadRegister8( dev, MAX31856_CONFIG_REG );
	if ( MAX31865_3WIRE == wires ){
		t |= MAX31856_CONFIG_3WIRE;
	}
	else{
		t &= ~MAX31856_CONFIG_3WIRE; // 2 or 4 wire
	}
	MAX31865_WriteRegister8( dev, MAX31856_CONFIG_REG, t );
}

void MAX31865_AutoConvert( const MAX31865_Config_t* dev, bool option ){
	uint8_t t = MAX31865_ReadRegister8( dev,MAX31856_CONFIG_REG );
	if( option ){
		t |= MAX31856_CONFIG_MODEAUTO;       // enable autoconvert
	}
	else{
		t &= ~MAX31856_CONFIG_MODEAUTO;       // disable autoconvert
	}
	MAX31865_WriteRegister8( dev, MAX31856_CONFIG_REG, t );
}

void MAX31865_EnableBias( const MAX31865_Config_t* dev, bool option ){
	uint8_t t = MAX31865_ReadRegister8( dev, MAX31856_CONFIG_REG );
	if (option){
		t |= MAX31856_CONFIG_BIAS;       // enable bias
	}
	else{
		t &= ~MAX31856_CONFIG_BIAS;       // disable bias
	}
	MAX31865_WriteRegister8(dev, MAX31856_CONFIG_REG, t );
}

MAX31865_Result		MAX31865_ReadTemperature( const MAX31865_Config_t* dev, float RTDnominal, float refResistor, float *outTemperature );



uint16_t MAX31865_ReadRTD( const MAX31865_Config_t* dev ){
	MAX31865_ClearFault( dev );
	MAX31865_EnableBias( dev,true );	
	
	dev->delay(10);
  	
  	uint8_t t = MAX31865_ReadRegister8( dev, MAX31856_CONFIG_REG );
  	t |= MAX31856_CONFIG_1SHOT;
  	MAX31865_WriteRegister8( dev, MAX31856_CONFIG_REG, t );

  	dev->delay(100);

  	uint16_t rtd = MAX31865_ReadRegister16( dev, MAX31856_RTDMSB_REG );
  	// remove fault
  	rtd >>= 1;
  	return rtd;
}

/*BSP Functions*/
void  MAX31865_ReadRegisterN( const MAX31865_Config_t* dev, uint8_t reg, uint8_t buffer[], uint8_t n ){
	dev->read( reg, buffer, n, 1000 );
}

uint8_t  MAX31865_ReadRegister8( const MAX31865_Config_t* dev, uint8_t reg ){
	uint8_t retValue;
	dev->read( reg, &retValue, 1, 1000 );
	return retValue;
}

uint16_t MAX31865_ReadRegister16( const MAX31865_Config_t* dev, uint8_t reg ){
	uint8_t value[2]={0,0};
	dev->read( reg, value, 2, 1000 );
	return value[0]<<8 | value[1];
}

void  MAX31865_WriteRegister8( const MAX31865_Config_t* dev, uint8_t reg, uint8_t data ){
	dev->write( reg, &data, 1, 1000 );
}
