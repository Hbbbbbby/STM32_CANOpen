#include "sht_config.h"

float SHTx_read_T_avg(void)
{
	float sht1_T = 0;
	float sht3_T = 0;
	float sht2_T = 0;
	
	sht1_T = SHTx_read_general(SHT1, TRIG_T_HM);
	sht3_T = SHTx_read_general(SHT3, TRIG_T_HM);
	
	sht2_T = SHTx_read_mimiciic(TRIG_T_HM);
	
	return (sht1_T+ sht2_T + sht3_T)/3;
}

float SHTx_read_HM_avg(void)
{
	float sht1_HM = 0;
	float sht2_HM = 0;
	float sht3_HM = 0;
	
	sht1_HM = SHTx_read_general(SHT1, TRIG_RH_HM);
	sht3_HM= SHTx_read_general(SHT3, TRIG_RH_HM);
	
	sht2_HM= SHTx_read_mimiciic(TRIG_RH_HM);
	
	return (sht1_HM+sht3_HM+sht2_HM)/3;
}

float SHTx_read_general(I2C_HandleTypeDef sht,uint8_t cmd)
{
	uint16_t val = 0; 
	
	uint8_t SHTx_DataBuf[2];

	HAL_I2C_Mem_Read(&sht, SHT20_ADDR, TRIG_T_HM, I2C_MEMADD_SIZE_8BIT, SHTx_DataBuf, 2, 1000);
	
	val = ((uint16_t)SHTx_DataBuf[0]<<8) + (SHTx_DataBuf[1] & 0xfe);
	
	if (TRIG_T_HM == cmd) 
		return SHTx_T_conversion(val);
	if (TRIG_RH_HM == cmd)
		return SHTx_HM_conversion(val);
	else
		return 0;
}

float SHTx_read_mimiciic(uint8_t cmd)
{
	uint16_t val = 0;
	
	uint8_t SHTx_DataBuf[2];
	
	iic_start();
	iic_send_byte(SHT20_ADDR_WR);
	iic_wait_ack();
	iic_send_byte(cmd);
	iic_wait_ack();
	
	iic_start();
	iic_send_byte(SHT20_ADDR_RD);
	iic_wait_ack();
//	IIC_SDA(0);
	SDA_Mode_IN();
//	for (int i = 0; i<1000; i++)
//	{
		HAL_Delay(500);
//		if (IIC_READ_SDA == 1) break;
//	}
	SDA_Mode_OUT();
		
	SHTx_DataBuf[0] = iic_read_byte(1);
	SHTx_DataBuf[1] = iic_read_byte(0);
	
	iic_stop();
	
	val = ((uint16_t)SHTx_DataBuf[0]<<8) + (SHTx_DataBuf[1] & 0xfe);
	
	if (TRIG_T_HM == cmd) 
		return SHTx_T_conversion(val);
	if (TRIG_RH_HM == cmd)
		return SHTx_HM_conversion(val);
	else
		return 0;
}

float SHTx_T_conversion(uint16_t val)
{
	float cov_val = 0;
	
	cov_val = (val * 175.72 /65536) - 46.85;
	
	return cov_val;
}

float SHTx_HM_conversion(uint16_t val)
{
	float cov_val = 0;
	
	cov_val = (val * 125.00 /65536) - 6;
	
return cov_val;
}
