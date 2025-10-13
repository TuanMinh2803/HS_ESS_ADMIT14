/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "fdcan.h"
#include "gpdma.h"
#include "i2c.h"
#include"tim.h"
#include "gpio.h"

#include "spi.h"

#include <math.h>
#include "medianfilter.h"
#include "CustomFunc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_CONNECTION_LOST_PERIOD 500   ///Set allowed time for connection lost here, in ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

volatile uint8_t prev_var=0, receiveFlag=0; //Checking CAN Msg receive
uint32_t last_check_time = 0;

//For Median filter
uint16_t ADC_Value_Front_Filtered = 0;
uint16_t ADC_Value_Rear_Filtered = 0;
uint16_t ADC_Value_Left_Filtered = 0;
uint16_t ADC_Value_Right_Filtered = 0;

FilterTypeDef FilterStruct_Front;
FilterTypeDef FilterStruct_Rear;
FilterTypeDef FilterStruct_Left;
FilterTypeDef FilterStruct_Right;





/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for PWMControl */
osThreadId_t PWMControlHandle;
const osThreadAttr_t PWMControl_attributes = {
  .name = "PWMControl",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for DistSensProcess */
osThreadId_t DistSensProcessHandle;
const osThreadAttr_t DistSensProcess_attributes = {
  .name = "DistSensProcess",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for IMUProcess */
osThreadId_t IMUProcessHandle;
const osThreadAttr_t IMUProcess_attributes = {
  .name = "IMUProcess",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of PWMControl */
  PWMControlHandle = osThreadNew(StartPWMTask, NULL, &PWMControl_attributes);

  /* creation of DistSensProcess */
  DistSensProcessHandle = osThreadNew(StartDistSensTask, NULL, &DistSensProcess_attributes);

  /* creation of IMUProcess */
  IMUProcessHandle = osThreadNew(StartIMUTask, NULL, &IMUProcess_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  /* Infinite loop */
  for(;;)
  {
	  if (HAL_GetTick() - last_check_time >= CAN_CONNECTION_LOST_PERIOD) {  //check after how many ms
		        if (prev_var == receiveFlag) {
		            // Variable has not changed for 700ms
		            // Take desired action here
		        	//Reset RxData buffer
		        	RxData[0]=100;				//Set to Neutral value for Throttle
		        	RxData[1]=100;				//Set to Neutral value for Steering Servo

		        } else {
		            // Variable changed, reset check
		        	//Do nothing, passed check
		            prev_var = receiveFlag;
		        }
		        last_check_time = HAL_GetTick();  // Reset timer
		    }
    osDelay(1);
  }
  /* USER CODE END defaultTask */
}

/* USER CODE BEGIN Header_StartPWMTask */
/**
* @brief Function implementing the PWMControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPWMTask */
void StartPWMTask(void *argument)
{
  /* USER CODE BEGIN PWMControl */
  /* Infinite loop */
  for(;;)
  {
	  TIM1->CCR1=ConvertToPWM_Signal(RxData[0], 0, 200, 103, 204);		//Assign 1st PWM signal, for Throttle
	  TIM1->CCR2=ConvertToPWM_Signal(RxData[1], 0, 200, 103, 223);		//Assign 2nd PWM signal, for Steering

    osDelay(1);
  }
  /* USER CODE END PWMControl */
}

/* USER CODE BEGIN Header_StartDistSensTask */
/**
* @brief Function implementing the DistSensProcess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDistSensTask */
void StartDistSensTask(void *argument)
{
  /* USER CODE BEGIN DistSensProcess */
  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_Start_DMA(&hadc1, &ADC_input, 4);


		Dist[0]=CalcDist(ADC_Value_Front_Filtered);
		Dist[1]=CalcDist(ADC_Value_Rear_Filtered);
		Dist[2]=CalcDist(ADC_Value_Left_Filtered);
		Dist[3]=CalcDist(ADC_Value_Right_Filtered);

	  /////Send CAN message function here
    osDelay(1);
  }
  /* USER CODE END DistSensProcess */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUProcess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN IMUProcess */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMUProcess */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

//void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan){					//Get CAN Msg
//
//	HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
//
//	receiveFlag++; if(receiveFlag==254) receiveFlag=0;
//}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){					//Get CAN Msg

	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, &RxData);
	receiveFlag++; if(receiveFlag==254) receiveFlag=0;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	ADC_Value_Front_Filtered = Moving_Average_Compute(ADC_input[0], &FilterStruct_Front);
	ADC_Value_Rear_Filtered  = Moving_Average_Compute(ADC_input[1], &FilterStruct_Rear);
	ADC_Value_Left_Filtered  = Moving_Average_Compute(ADC_input[2], &FilterStruct_Left);
	ADC_Value_Right_Filtered = Moving_Average_Compute(ADC_input[3], &FilterStruct_Right);
}
/* USER CODE END Application */

