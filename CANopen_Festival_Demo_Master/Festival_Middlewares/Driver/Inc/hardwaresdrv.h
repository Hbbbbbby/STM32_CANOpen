#ifndef _HARDWARESDRV_H
#define _HARDWARESDRV_H

#include "main.h"
#include "canfestival.h"
#include "timer.h"

#include "MasterNode_Sensor.h"

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

#define MASTER_NODE_ID 0
#define Producer_Heartbeat_Time 500 //ms

#define CANOPENNODE									(MasterNode_Sensor_Data)
#define CANOPENNODE_ID							MASTER_NODE_ID

#define CANOPEN_TIMx								htim3
#define CANOPEN_TIM_PERIOD          65535

#define CANx												hcan

void TIMx_DispatchFromISR(void);

#endif
