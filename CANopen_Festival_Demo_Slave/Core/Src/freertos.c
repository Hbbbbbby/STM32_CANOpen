/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "objacces.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static TIMEVAL last_counter_val = 0;
static TIMEVAL elapsed_time = 0;

/* USER CODE END Variables */
/* Definitions for CanSen_Task */
osThreadId_t CanSen_TaskHandle;
const osThreadAttr_t CanSen_Task_attributes = {
  .name = "CanSen_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CanRev_Task */
osThreadId_t CanRev_TaskHandle;
const osThreadAttr_t CanRev_Task_attributes = {
  .name = "CanRev_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CANOpen_Task */
osThreadId_t CANOpen_TaskHandle;
const osThreadAttr_t CANOpen_Task_attributes = {
  .name = "CANOpen_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for xCANRcvQueue */
osMessageQueueId_t xCANRcvQueueHandle;
const osMessageQueueAttr_t xCANRcvQueue_attributes = {
  .name = "xCANRcvQueue"
};
/* Definitions for xCANSenQueue */
osMessageQueueId_t xCANSenQueueHandle;
const osMessageQueueAttr_t xCANSenQueue_attributes = {
  .name = "xCANSenQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Task_CanSen(void *argument);
void Task_CanRev(void *argument);
void Task_CANOpen(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xCANRcvQueue */
  xCANRcvQueueHandle = osMessageQueueNew (10, sizeof(CanRxMsg), &xCANRcvQueue_attributes);

  /* creation of xCANSenQueue */
  xCANSenQueueHandle = osMessageQueueNew (10, sizeof(CanTxMsg), &xCANSenQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CanSen_Task */
  CanSen_TaskHandle = osThreadNew(Task_CanSen, NULL, &CanSen_Task_attributes);

  /* creation of CanRev_Task */
  CanRev_TaskHandle = osThreadNew(Task_CanRev, NULL, &CanRev_Task_attributes);

  /* creation of CANOpen_Task */
  CANOpen_TaskHandle = osThreadNew(Task_CANOpen, NULL, &CANOpen_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Task_CanSen */
/**
  * @brief  Function implementing the CanSen_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task_CanSen */
void Task_CanSen(void *argument)
{
  /* USER CODE BEGIN Task_CanSen */
	
	static CanTxMsg TxMsg;
	static uint32_t TxMailbox = CAN_TX_MAILBOX0;
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
		if (xQueueReceive(xCANSenQueueHandle, &TxMsg, 100) == pdTRUE) {
			HAL_CAN_AddTxMessage(&CANx, &TxMsg.header, TxMsg.Data, &TxMailbox);
			while (HAL_CAN_GetTxMailboxesFreeLevel(&CANx) != 3);
		}
  }
  /* USER CODE END Task_CanSen */
}

/* USER CODE BEGIN Header_Task_CanRev */
/**
* @brief Function implementing the CanRev_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_CanRev */
void Task_CanRev(void *argument)
{
  /* USER CODE BEGIN Task_CanRev */
	
	static CanRxMsg RxMsg;
	static Message msg;
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
		if (xQueueReceive(xCANRcvQueueHandle, &RxMsg, 100) == pdTRUE) {
			msg.cob_id = RxMsg.header.StdId;
			
			if(CAN_RTR_REMOTE == RxMsg.header.RTR) {
				msg.rtr = 1;
			}
			else {
				msg.rtr = 0;
			}
			msg.len = (UNS8)RxMsg.header.DLC;
			memcpy(msg.data, RxMsg.Data, 8);
			canDispatch(&CANOPEN_NODE, &msg);
//			canSend(&CANx, &msg);
		}
  }
  /* USER CODE END Task_CanRev */
}

/* USER CODE BEGIN Header_Task_CANOpen */
/**
* @brief Function implementing the CANOpen_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_CANOpen */
void Task_CANOpen(void *argument)
{
  /* USER CODE BEGIN Task_CANOpen */
	
	HAL_TIM_Base_Start_IT(&CANOPEN_TIMx);
	
	UNS8 Transmission_Type;
	UNS16 Producer_Heartbeat_Time;
	
	UNS32 size_U8 = sizeof(UNS8);
	UNS32 size_U16 = sizeof(UNS16);
	
	Transmission_Type = CANOPEN_TPDO_TIME;
	Producer_Heartbeat_Time = CANOPEN_HBT;
	
	writeLocalDict(&CANOPEN_NODE, (UNS16)0x1800, (UNS8)2, &Transmission_Type, &size_U8, 1);
	writeLocalDict(&CANOPEN_NODE, (UNS16)0x1017, (UNS8)0, &Producer_Heartbeat_Time, &size_U16, 1);

	
	setNodeId(&CANOPEN_NODE, CANOPEN_NODE_ID);
	
	setState(&CANOPEN_NODE, Initialisation);
	
//	setState(&CANOPEN_NODE, Operational);
		
  /* Infinite loop */
  for(;;)
  {
		
		switch(getState(&CANOPEN_NODE))
		{
			case Pre_operational: {
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				osDelay(100);
				break;
			}
			case Operational: {
				AvgData_Temperature = SHTx_read_T_avg();
				AvgData_Humidity = SHTx_read_HM_avg();
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				osDelay(CANOPEN_HBT);
				break;
			}
			case Stopped: {
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				break;
			}
			default: {
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				osDelay(10);
			}
		}
		
  }
  /* USER CODE END Task_CANOpen */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
unsigned char canSend(CAN_PORT notused, Message *m)
{
  static CanTxMsg TxMsg;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  TxMsg.header.StdId = m->cob_id;

  if(m->rtr)
      TxMsg.header.RTR = CAN_RTR_REMOTE;
  else
      TxMsg.header.RTR = CAN_RTR_DATA;

  TxMsg.header.IDE = CAN_ID_STD;
  TxMsg.header.DLC = m->len;

  memcpy(TxMsg.Data, m->data, 8);

  if(0 == __get_CONTROL())
  {
    if(xQueueSendFromISR(xCANSenQueueHandle, &TxMsg, &xHigherPriorityTaskWoken) != pdPASS)
    {
      return 0xFF;
    }
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
  }
  else
  {
    if(xQueueSend(xCANSenQueueHandle, &TxMsg, 100) != pdPASS)
    {
      return 0xFF;
    }
  }
  return 0;
}

void setTimer(TIMEVAL value)
{
	__HAL_TIM_SetAutoreload(&CANOPEN_TIMx, value);
}

TIMEVAL getElapsedTime(void)
{
	uint32_t timer = __HAL_TIM_GET_COUNTER(&CANOPEN_TIMx);

	if(timer < last_counter_val)
	timer += CANOPEN_TIM_PERIOD;

	TIMEVAL elapsed = timer - last_counter_val + elapsed_time;

	return elapsed;
}

void TIMx_DispatchFromISR(void)
{
	last_counter_val = 0;
	elapsed_time = 0;
	TimeDispatch();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CANx)
{
		static CanRxMsg RxMsg;
	if (HAL_CAN_GetRxMessage(CANx, CAN_RX_FIFO0, &RxMsg.header, RxMsg.Data) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(CANx, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
	/* Notification Error */
		Error_Handler();
	}

	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if(NULL != xCANRcvQueueHandle)
	{
		xQueueSendFromISR(xCANRcvQueueHandle, &RxMsg, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
	
}

/* USER CODE END Application */

