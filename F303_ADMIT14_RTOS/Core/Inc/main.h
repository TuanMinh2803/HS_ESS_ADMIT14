/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define ADC_RearSens_Pin GPIO_PIN_0
#define ADC_RearSens_GPIO_Port GPIOA
#define ADC_RightSens_Pin GPIO_PIN_1
#define ADC_RightSens_GPIO_Port GPIOA
#define ADC_FrontSens_Pin GPIO_PIN_2
#define ADC_FrontSens_GPIO_Port GPIOA
#define ADC_LeftSens_Pin GPIO_PIN_3
#define ADC_LeftSens_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
extern uint16_t ADC_input[4];
extern uint8_t Dist[4];
extern float Coeff[6];

extern volatile uint8_t prev_var;
extern uint32_t last_check_time;

extern CAN_RxHeaderTypeDef RxHeader;				//Currently only receive 1 message ID 101


extern CAN_TxHeaderTypeDef TxHeader_Sensor;		//Header for DistSens Message
extern CAN_TxHeaderTypeDef TxHeader_IMU_Accel; 	//Header for IMU_Accel Message
extern CAN_TxHeaderTypeDef TxHeader_IMU_Gyro;		//Header for IMU_Gyro Message
extern CAN_TxHeaderTypeDef TxHeader_IMU_Mag;		//Header for IMU_Mag Message

extern uint8_t RxData[8];
extern volatile uint8_t receiveFlag;

extern uint32_t TxMailbox;

extern float ax, ay, az			//Accel
			,gx, gy, gz			//Gyro
			,mx, my, mz;		//Magnetometer

extern uint8_t Accel[3], Gyro[3], Magne[3];


uint16_t CalcDist(uint16_t ADC12In);  	//Calculate ADC12 to Dist in mm
uint8_t ConvertToPWM_Signal(uint8_t OutVal, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
