#ifndef MODBUS_REGISTER_MAP_H
#define MODBUS_REGISTER_MAP_H

#include "Core/ModbusApp.h"

// Modbus Register Configuration
#define RAINING_STATUS_REGISTER 			10
#define RAIN_BOX_FULL_STATUS_REGISTER 		11
#define RAIN_BOX_EMPTY_STATUS_REGISTER		12
#define SAMPLE_BOX_FULL_STATUS_REGISTER		13

#define DISCHARGING_VALVE_STATUS_REGISTER  	14
#define SAMPLE_BOX_VALVE_STATUS_REGISTER	15

#define RAIN_BOX_COVER_STATUS_REGISTER  	16
#define DUST_BOX_COVER_STATUS_REGISTER		17

#define MOTOR_DIR_DUST_BOX_REGISTER      	18
#define MOTOR_DIR_RAIN_BOX_REGISTER			19
#define MOTOR_IS_TURNING_REGISTER		  	20

#define BME280_1_TEMPERATURE_1_REG			10
#define BME280_1_TEMPERATURE_2_REG			11
#define BME280_1_HUMIDITY_1_REG				12
#define BME280_1_HUMIDITY_2_REG				13
#define BME280_1_AIRPRESSURE_1_REG			14
#define BME280_1_AIRPRESSURE_2_REG			15

#define BME280_2_TEMPERATURE_1_REG			16
#define BME280_2_TEMPERATURE_2_REG			17
#define BME280_2_HUMIDITY_1_REG				18
#define BME280_2_HUMIDITY_2_REG				19
#define BME280_2_AIRPRESSURE_1_REG			20
#define BME280_2_AIRPRESSURE_2_REG			21

ModbusRegister_Handle_t rainingRegHandle = { -1 ,0 };

ModbusRegister_Handle_t rainBoxFullRegHandle = { -1 ,0 };
ModbusRegister_Handle_t rainBoxEmptyRegHandle = { -1 ,0 };
ModbusRegister_Handle_t sampleBoxFullRegHandle = { -1 ,0 };

ModbusRegister_Handle_t dischargeValveRegHandle = { -1 ,0 };
ModbusRegister_Handle_t sampleBoxValveRegHandle = { -1 ,0 };

ModbusRegister_Handle_t rainBoxCoverRegHandle = { -1 ,0 };
ModbusRegister_Handle_t dustBoxCoverRegHandle = { -1 ,0 };

ModbusRegister_Handle_t motorTurningDustBoxRegHandle = { -1 ,0 };
ModbusRegister_Handle_t motorTurningRainBoxRegHandle = { -1 ,0 };
ModbusRegister_Handle_t motorIsTurningRegHandle = { -1 ,0 };


ModbusRegister_Handle_t userLed1RegHandle = { -1 ,0 };
ModbusRegister_Handle_t userLed2RegHandle = { -1 ,0 };
ModbusRegister_Handle_t userLed3RegHandle = { -1 ,0 };

// BME280 Registers
ModbusRegister_Handle_t bme280_1_TemperatureReg1 = { -1 ,0 };
ModbusRegister_Handle_t bme280_1_TemperatureReg2 = { -1 ,0 };
ModbusRegister_Handle_t bme280_1_HumidityReg1 = { -1 ,0 };
ModbusRegister_Handle_t bme280_1_HumidityReg2 = { -1 ,0 };
ModbusRegister_Handle_t bme280_1_AirPressureReg1 = { -1 ,0 };
ModbusRegister_Handle_t bme280_1_AirPressureReg2 = { -1 ,0 };


ModbusRegister_Handle_t bme280_2_TemperatureReg1 = { -1 ,0 };
ModbusRegister_Handle_t bme280_2_TemperatureReg2 = { -1 ,0 };
ModbusRegister_Handle_t bme280_2_HumidityReg1 = { -1 ,0 };
ModbusRegister_Handle_t bme280_2_HumidityReg2 = { -1 ,0 };
ModbusRegister_Handle_t bme280_2_AirPressureReg1 = { -1 ,0 };
ModbusRegister_Handle_t bme280_2_AirPressureReg2 = { -1 ,0 };


#endif
