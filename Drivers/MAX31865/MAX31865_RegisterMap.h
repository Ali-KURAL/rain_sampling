#ifndef MAX31865_REGISTER_MAP_H
#define MAX31865_REGISTER_MAP_H


#define MAX31856_CONFIG_REG            0x00
#define MAX31856_CONFIG_BIAS           0x80
#define MAX31856_CONFIG_MODEAUTO       0x40
#define MAX31856_CONFIG_MODEOFF        0x00
#define MAX31856_CONFIG_1SHOT          0x20
#define MAX31856_CONFIG_3WIRE          0x10
#define MAX31856_CONFIG_24WIRE         0x00
#define MAX31856_CONFIG_FAULTSTAT      0x02
#define MAX31856_CONFIG_FILT50HZ       0x01
#define MAX31856_CONFIG_FILT60HZ       0x00

#define MAX31856_RTDMSB_REG           0x01
#define MAX31856_RTDLSB_REG           0x02
#define MAX31856_HFAULTMSB_REG        0x03
#define MAX31856_HFAULTLSB_REG        0x04
#define MAX31856_LFAULTMSB_REG        0x05
#define MAX31856_LFAULTLSB_REG        0x06
#define MAX31856_FAULTSTAT_REG        0x07


#define MAX31865_FAULT_HIGHTHRESH     0x80
#define MAX31865_FAULT_LOWTHRESH      0x40
#define MAX31865_FAULT_REFINLOW       0x20
#define MAX31865_FAULT_REFINHIGH      0x10
#define MAX31865_FAULT_RTDINLOW       0x08
#define MAX31865_FAULT_OVUV           0x04


#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7
#define RREF 400

#define REG_CONFIG                  0x00
#define REG_RTD_MSB                 0x01
#define REG_RTD_LSB                 0x02
#define REG_HIGH_FAULT_THR_MSB      0x03
#define REG_HIGH_FAULT_THR_LSB      0x04
#define REG_LOW_FAULT_THR_MSB       0x05
#define REG_LOW_FAULT_THR_LSB       0x06
#define REG_FAULT_STATUS            0x07
#define WR(reg)                     ( (reg) | 0x80 )



#endif // endif