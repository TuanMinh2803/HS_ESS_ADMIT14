/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<math.h> //for Pow Function
#include "lsm9ds1.h" //IMU Library
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

/* USER CODE BEGIN PV */
uint16_t ADC_input[4]; //storing 4 adc values for sensors
uint8_t Dist[4];      //storing 4 distance of sensors, order: Front, rear, left, right - PIN 1,2,4,12

float Coeff[6]={-5.736e-15, 6.946e-11, -3.316e-07, 0.0007958, -1.012, 644.3};


//For IMU
float ax, ay, az		//Accel
	 ,gx, gy, gz		//Gyro
	 ,mx, my, mz;		//Magnetometer

uint8_t Accel[3], Gyro[3], Magne[3];	//Array to send to CAN message, must be uint8


//For CAN Bus
CAN_RxHeaderTypeDef RxHeader;				//Currently only receive 1 message ID 101

CAN_TxHeaderTypeDef TxHeader_Sensor;		//Header for DistSens Message
CAN_TxHeaderTypeDef TxHeader_IMU_Accel; 	//Header for IMU_Accel Message
CAN_TxHeaderTypeDef TxHeader_IMU_Gyro;		//Header for IMU_Gyro Message
CAN_TxHeaderTypeDef TxHeader_IMU_Mag;		//Header for IMU_Mag Message

uint8_t RxData[8];  //For receiving CAN Message
uint8_t ThrottleVal, SteeringVal;

uint32_t TxMailbox;		//Mailbox for transmission

volatile uint8_t prev_var=0, receiveFlag=0; //Checking CAN Msg receive
uint32_t last_check_time = 0;
//uint8_t changed_or_not;




//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){					//Get CAN Msg
//
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
//	receiveFlag++; if(receiveFlag==254) receiveFlag=0;
//}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
//uint16_t CalcDist(uint16_t ADC12In);  	//Calculate ADC12 to Dist in mm
//uint8_t ConvertToPWM_Signal(uint8_t OutVal, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);
//CAN Header Init Functions
void TxHeaderInit_Sensor();
void TxHeaderInit_IMU_Accel();
void TxHeaderInit_IMU_Gyro();
void TxHeaderInit_IMU_Mag();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);			//Start PWM generation for 1st channel
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);			//Start PWM generation for 2nd channel


  //I2C LSM9DS1 code init
  begin(0x6B, 0x1E, &hi2c1);	//Init I2C connection to IMU, 0x6B for Accel, Gyro; 0x1E for Magnetometer
  setMagScale(4);					//	4 8 12 16 gauss
  setAccelScale(4);					//	2 4 8 16 g
  setGyroScale(245);				//  245 500 2000 deg/sec

  //Init CAN Headers

  HAL_CAN_Start(&hcan);

  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  TxHeaderInit_Sensor();
  TxHeaderInit_IMU_Accel();
  TxHeaderInit_IMU_Gyro();
  TxHeaderInit_IMU_Mag();

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  //Receive CAN MSG 101, get Throttle and Steering value, translate into PWM signal
//
//
//
//	    if (HAL_GetTick() - last_check_time >= 700) {  //check after how many ms
//	        if (prev_var == receiveFlag) {
//	            // Variable has not changed for 700ms
//	            // Take desired action here
//	        	//Reset RxData buffer
//	        	RxData[0]=100;				//Set to Neutral value for Throttle
//	        	RxData[1]=100;				//Set to Neutral value for Steering Servo
//
//	        } else {
//	            // Variable changed, reset check
//	        	//Do nothing, passed check
//	            prev_var = receiveFlag;
//	        }
//	        last_check_time = HAL_GetTick();  // Reset timer
//	    }
//
//	    //Convert received Throttle value and Steering value into PWM to control
//
//	    ThrottleVal=ConvertToPWM_Signal(RxData[0], 0, 200, 103, 204);		//Only for debug, see what are the values
//		SteeringVal=ConvertToPWM_Signal(RxData[1], 0, 200, 103, 223);  		//Customizable range, 223 so car can turn right the same amount as turning left
//
//	  TIM1->CCR1=ConvertToPWM_Signal(RxData[0], 0, 200, 103, 204);		//Assign 1st PWM signal, for Throttle
//	  TIM1->CCR2=ConvertToPWM_Signal(RxData[1], 0, 200, 103, 223);		//Assign 2nd PWM signal, for Steering
	// TIM1->CCR1=ThrottleVal;  	//in PWM value, divide by 2048 for Duty cycle percentage
	//TIM1->CCR2=SteeringVal;

	 /////////////////////////////////////////////////////////////////////////////

	  // Calc Dist, send to CAN MSG 102
//	 for(int i=0; i<4;i++){
//		 Dist[i]=CalcDist(ADC_input[i]);
//	 }
//
//	 /////////////////////////////////////////////////////////////////////////////
//
//	  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader_Sensor, Dist, &TxMailbox)!= HAL_OK){
//		  Error_Handler();																//Transmit CAN Message, if not succeed then error handle
//	  }

	  /////////////////////////////////////////////////////////////////////////////
	  //Get data from IMU

	  //Get Accel Info from IMU, send to CAN MSG 103
//	  readAccel(&ax, &ay, &az);
//	  Accel[0]=(ax+4)*30;	Accel[1]=(ay+4)*30;	Accel[2]=(az+4)*30;
//
//
//
//	  //Get Gyro Info from IMU, send to CAN MSG 104
//	  readGyro(&gx, &gy, &gz);
//	  Gyro[0]=(gx+245)*0.5 ;	Gyro[1]=(gy+245)*0.5 ;		Gyro[2]=(gz+245)*0.5 ;
//
//
//	  /////////////////////////////////////////////////////////////////////////////
//	  //Add CAN Messages
//
//	  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader_IMU_Accel, Accel, &TxMailbox)!= HAL_OK){
//			  Error_Handler();																//Transmit CAN Message, if not succeed then error handle
//		  }
//
//	  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader_IMU_Gyro, Gyro, &TxMailbox)!= HAL_OK){
//		  Error_Handler();																//Transmit CAN Message, if not succeed then error handle
//	  }

	  // readMag(&mx, &my, &mz);
	  //  Magne[0]=mx*100 ;	Magne[1]=my*100 ;			Magne[2]=mz*100 ;


	 // HAL_Delay(100);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t CalcDist(uint16_t ADC12In){
	uint16_t TempResult;
	TempResult=Coeff[0]*pow(ADC12In,5)+Coeff[1]*pow(ADC12In,4)+Coeff[2]*pow(ADC12In,3)+Coeff[3]*pow(ADC12In,2)+Coeff[4]*pow(ADC12In,1)+Coeff[5];

	if(TempResult>=40&&TempResult<=290){
		return TempResult-35;			//to fit into uint8 range
	}
	else {
		if(TempResult<40) return TempResult;   //0 àter debug
			else return TempResult;				//1
		}
}

uint8_t ConvertToPWM_Signal(uint8_t OutVal, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max){
	return (OutVal - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void TxHeaderInit_Sensor(){
	TxHeader_Sensor.StdId=102;
	TxHeader_Sensor.ExtId=0;
	TxHeader_Sensor.IDE=CAN_ID_STD;
	TxHeader_Sensor.RTR=CAN_RTR_DATA;
	TxHeader_Sensor.DLC=4;
	TxHeader_Sensor.TransmitGlobalTime=DISABLE;
}

void TxHeaderInit_IMU_Accel(){
	TxHeader_IMU_Accel.StdId=103;
	TxHeader_IMU_Accel.ExtId=0;
	TxHeader_IMU_Accel.IDE=CAN_ID_STD;
	TxHeader_IMU_Accel.RTR=CAN_RTR_DATA;
	TxHeader_IMU_Accel.DLC=3;
	TxHeader_IMU_Accel.TransmitGlobalTime=DISABLE;
}

void TxHeaderInit_IMU_Gyro(){
	TxHeader_IMU_Gyro.StdId=104;
	TxHeader_IMU_Gyro.ExtId=0;
	TxHeader_IMU_Gyro.IDE=CAN_ID_STD;
	TxHeader_IMU_Gyro.RTR=CAN_RTR_DATA;
	TxHeader_IMU_Gyro.DLC=3;
	TxHeader_IMU_Gyro.TransmitGlobalTime=DISABLE;
}

void TxHeaderInit_IMU_Mag(){
	TxHeader_IMU_Mag.StdId=105;
	TxHeader_IMU_Mag.ExtId=0;
	TxHeader_IMU_Mag.IDE=CAN_ID_STD;
	TxHeader_IMU_Mag.RTR=CAN_RTR_DATA;
	TxHeader_IMU_Mag.DLC=3;
	TxHeader_IMU_Mag.TransmitGlobalTime=DISABLE;
}



/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
