#ifndef MAX31865_H
#define MAX31865_H

#include "MAX31865_RegisterMap.h"
#include "MAX31865_TypeDefs.h"
#include <stdbool.h>

float 	MAX31865_temperature;

/*Functions*/
/*User Level Functions*/
bool 		MAX31865_Begin( const MAX31865_Config_t* dev, MAX31865_NumWires_t wires);
uint8_t 	MAX31865_ReadFault( const MAX31865_Config_t* dev );
void 		MAX31865_ClearFault( const MAX31865_Config_t* dev );
void 		MAX31865_SetWires( const MAX31865_Config_t* dev, MAX31865_NumWires_t wires );
void 		MAX31865_AutoConvert( const MAX31865_Config_t* dev, bool option );
void 		MAX31865_EnableBias( const MAX31865_Config_t* dev, bool option );
uint8_t		MAX31865_ReadTemperature( const MAX31865_Config_t* dev, float RTDnominal, float refResistor, float *outTemperature );


#endif // endif
