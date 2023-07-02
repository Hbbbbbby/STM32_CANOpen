#ifndef __SHT_CONFIG_H
#define __SHT_CONFIG_H

#include "main.h"

#define SHT1					hi2c1
#define SHT2					1
#define SHT3					hi2c2

#define SHT20_ADDR		0x80

#define SHT20_ADDR_WR	0x80
#define SHT20_ADDR_RD	0x81

#define TRIG_T_HM			0xE3
#define TRIG_RH_HM		0xE5

float SHTx_T_conversion(uint16_t val);

float SHTx_HM_conversion(uint16_t val);

float SHTx_read_general(I2C_HandleTypeDef sht,uint8_t cmd);

float SHTx_read_mimiciic(uint8_t cmd);

float SHTx_read_T_avg(void);

float SHTx_read_HM_avg(void);



#endif
