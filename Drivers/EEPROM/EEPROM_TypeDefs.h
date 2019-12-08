#ifndef EEPROM_TYPE_DEFS_H
#define EEPROM_TYPE_DEFS_H


typedef void(*EEPROM_Read )( uint8_t i2c_address, uint8_t* buffer, uint16_t length, uint32_t timeout );
typedef void(*EEPROM_Write)( uint8_t i2c_address, uint8_t* buffer, uint16_t length, uint32_t timeout );

typedef struct EEPROM_Config_t{
	EEPROM_Id 			id;
	EEPROM_Read 		read;
	//EEPROM_ReadAsync 	readAsync;
	EEPROM_Write		write;
	//EEPROM_WriteAsync   writeAsync;
}EEPROM_Config_t;

#endif //