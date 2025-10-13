/*
 * CustomFunc.h
 *
 *  Created on: Oct 13, 2025
 *      Author: Minh
 */

//Header file for custom functions and variables


#ifndef INC_CUSTOMFUNC_H_
#define INC_CUSTOMFUNC_H_

#include<stdint.h>
#include "stm32h5xx_hal.h"


extern uint16_t ADC_input[4];
extern uint8_t Dist[4];
extern float Coeff[6];


extern FDCAN_RxHeaderTypeDef RxHeader;				//Currently only receive 1 message ID 101


extern FDCAN_TxHeaderTypeDef TxHeader_Sensor;		//Header for DistSens Message
extern FDCAN_TxHeaderTypeDef TxHeader_IMU_Accel; 	//Header for IMU_Accel Message
extern FDCAN_TxHeaderTypeDef TxHeader_IMU_Gyro;		//Header for IMU_Gyro Message
extern FDCAN_TxHeaderTypeDef TxHeader_IMU_Mag;		//Header for IMU_Mag Message

extern uint8_t RxData[8];
extern volatile uint8_t receiveFlag;

extern uint32_t TxMailbox;


void TxHeaderInit_Sensor();
void TxHeaderInit_IMU_Accel();
void TxHeaderInit_IMU_Gyro();
void TxHeaderInit_IMU_Mag();

#endif /* INC_CUSTOMFUNC_H_ */
