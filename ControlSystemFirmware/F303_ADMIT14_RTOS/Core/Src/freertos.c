/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

#include<math.h> //for Pow Function
#include "lsm9ds1.h" //IMU Library
#include "medianfilter.h"
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

//int16_t ADC_input[4]; //storing 4 adc values for sensors
//uint8_t Dist[4];      //storing 4 distance of sensors, order: Front, rear, left, right - PIN 1,2,4,12
//
//float Coeff[]={-5.736e-15, 6.946e-11, -3.316e-07, 0.0007958, -1.012, 644.3};
//
//float ax, ay, az		//Accel
//	 ,gx, gy, gz;		//Gyro
//	 //,mx, my, mz;		//Magnetometer
//
////For CAN Bus
//CAN_RxHeaderTypeDef RxHeader;				//Currently only receive 1 message ID 101
//
//CAN_TxHeaderTypeDef TxHeader_Sensor;		//Header for DistSens Message
//CAN_TxHeaderTypeDef TxHeader_IMU_Accel; 	//Header for IMU_Accel Message
//CAN_TxHeaderTypeDef TxHeader_IMU_Gyro;		//Header for IMU_Gyro Message
//CAN_TxHeaderTypeDef TxHeader_IMU_Mag;		//Header for IMU_Mag Message
//
//uint8_t RxData[8];  //For receiving CAN Message
//uint8_t ThrottleVal, SteeringVal;
//
//uint32_t TxMailbox;		//Mailbox for transmission
//
//volatile uint8_t receiveFlag=0, prev_var=0; //Checking CAN Msg receive
//uint32_t last_check_time = 0;
////uint8_t changed_or_not;



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
osThreadId defaultTaskHandle;
osThreadId PWMControlHandle;
osThreadId DistSensProcessHandle;
osThreadId IMUProcessHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){					//Get CAN Msg
//
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
//	receiveFlag++; if(receiveFlag==254) receiveFlag=0;
//}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);					//Get CAN Msg
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartPWMTask(void const * argument);
void StartDistSensTask(void const * argument);
void StartIMUTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of PWMControl */
  osThreadDef(PWMControl, StartPWMTask, osPriorityNormal, 0, 128);
  PWMControlHandle = osThreadCreate(osThread(PWMControl), NULL);

  /* definition and creation of DistSensProcess */
  osThreadDef(DistSensProcess, StartDistSensTask, osPriorityIdle, 0, 128);
  DistSensProcessHandle = osThreadCreate(osThread(DistSensProcess), NULL);

  /* definition and creation of IMUProcess */
  osThreadDef(IMUProcess, StartIMUTask, osPriorityIdle, 0, 128);
  IMUProcessHandle = osThreadCreate(osThread(IMUProcess), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {

	  if (HAL_GetTick() - last_check_time >= 700) {  //check after how many ms
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
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartPWMTask */
/**
* @brief Function implementing the PWMControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPWMTask */
void StartPWMTask(void const * argument)
{
  /* USER CODE BEGIN StartPWMTask */
  /* Infinite loop */
  for(;;)
  {
	  TIM1->CCR1=ConvertToPWM_Signal(RxData[0], 0, 200, 103, 204);		//Assign 1st PWM signal, for Throttle
	  TIM1->CCR2=ConvertToPWM_Signal(RxData[1], 0, 200, 103, 223);		//Assign 2nd PWM signal, for Steering
    osDelay(1);
  }
  /* USER CODE END StartPWMTask */
}

/* USER CODE BEGIN Header_StartDistSensTask */
/**
* @brief Function implementing the DistSensProcess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDistSensTask */
void StartDistSensTask(void const * argument)
{
  /* USER CODE BEGIN StartDistSensTask */
  /* Infinite loop */
  for(;;)
  {
	  //Calc Dist, send to CAN MSG 102
	HAL_ADC_Start_DMA(&hadc1, &ADC_input, 4);   //Start sampling at the beginning of task, auto stop after complete sampling of 4 channels

	Dist[0]=CalcDist(ADC_Value_Front_Filtered);
	Dist[1]=CalcDist(ADC_Value_Rear_Filtered);
	Dist[2]=CalcDist(ADC_Value_Left_Filtered);
	Dist[3]=CalcDist(ADC_Value_Right_Filtered);

	  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader_Sensor, Dist, &TxMailbox)!= HAL_OK){
		  Error_Handler();																//Transmit CAN Message, if not succeed then error handle
	  }				//Transmit CAN Message, if not succeed, the nerror handle
    osDelay(1);
  }
  /* USER CODE END StartDistSensTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUProcess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void const * argument)
{
  /* USER CODE BEGIN StartIMUTask */
  /* Infinite loop */
  for(;;)
  {
	readAccel(&ax, &ay, &az);
	Accel[0]=(ax+4)*30;	Accel[1]=(ay+4)*30;	Accel[2]=(az+4)*30;

	if(HAL_CAN_AddTxMessage(&hcan, &TxHeader_IMU_Accel, Accel, &TxMailbox)!= HAL_OK){
			  Error_Handler();																//Transmit CAN Message, if not succeed then error handle
	}

    osDelay(1);

	readGyro(&gx, &gy, &gz);
	Gyro[0]=(gx+245)*0.5 ;	Gyro[1]=(gy+245)*0.5 ;		Gyro[2]=(gz+245)*0.5 ;

	if(HAL_CAN_AddTxMessage(&hcan, &TxHeader_IMU_Gyro, Gyro, &TxMailbox)!= HAL_OK){
		  Error_Handler();																//Transmit CAN Message, if not succeed then error handle
	}
	osDelay(1);

////// REMOVE THIS PART IF BUG HAPPEN
	readMag(&mx, &my, &mz);
	Magne[0]=(mx+128)*100; Magne[1]=(my+128)*100; Magne[2]=(mz+128)*100;

	if(HAL_CAN_AddTxMessage(&hcan, &TxHeader_IMU_Mag, Magne, &TxMailbox)!= HAL_OK){
			  Error_Handler();																//Transmit CAN Message, if not succeed then error handle
		}

	osDelay(1);
  }

  /* USER CODE END StartIMUTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){					//Get CAN Msg

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	receiveFlag++; if(receiveFlag==254) receiveFlag=0;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
		ADC_Value_Front_Filtered = Moving_Average_Compute(ADC_input[0], &FilterStruct_Front);
		ADC_Value_Rear_Filtered  = Moving_Average_Compute(ADC_input[1], &FilterStruct_Rear);
		ADC_Value_Left_Filtered  = Moving_Average_Compute(ADC_input[2], &FilterStruct_Left);
		ADC_Value_Right_Filtered = Moving_Average_Compute(ADC_input[3], &FilterStruct_Right);
}

/* USER CODE END Application */

