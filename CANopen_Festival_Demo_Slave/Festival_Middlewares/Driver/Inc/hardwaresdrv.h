#ifndef _HARDWARESDRV_H
#define _HARDWARESDRV_H

#include "main.h"
#include "canfestival.h"
#include "timer.h"

#include "objacces.h"


#include "SlaveNode_Sensor.h"


typedef struct
{
	CAN_TxHeaderTypeDef header;
	uint8_t Data[8];
}CanTxMsg;

typedef struct
{
	CAN_RxHeaderTypeDef header;
	uint8_t Data[8];
}CanRxMsg;

#define SLAVE_NODE_ID 20
#define CANOPEN_HBT ((SLAVE_NODE_ID*50)+2000) //ms

#define CANOPEN_NODE									(SlaveNode_Sensor_Data)
#define CANOPEN_NODE_ID							SLAVE_NODE_ID

#define CANOPEN_TPDO_TIME						SLAVE_NODE_ID+20

#define CANOPEN_TIMx								htim3
#define CANOPEN_TIM_PERIOD          65535

#define CANx												hcan

void TIMx_DispatchFromISR(void);

#endif
