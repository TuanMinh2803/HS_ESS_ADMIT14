/*
 * CustomFunc.c
 *
 *  Created on: Oct 13, 2025
 *      Author: Minh
 */

//Source file for custom functions, variables

#include"CustomFunc.h"



										//VARIABLES
//ADC
uint16_t ADC_input[4]; //storing 4 adc values for sensors
uint8_t Dist[4];

float Coeff[6]={-5.736e-15, 6.946e-11, -3.316e-07, 0.0007958, -1.012, 644.3};

//CAN
FDCAN_RxHeaderTypeDef RxHeader;				//Currently only receive 1 message ID 101

FDCAN_TxHeaderTypeDef TxHeader_Sensor;		//Header for DistSens Message
FDCAN_TxHeaderTypeDef TxHeader_IMU_Accel; 	//Header for IMU_Accel Message
FDCAN_TxHeaderTypeDef TxHeader_IMU_Gyro;		//Header for IMU_Gyro Message
FDCAN_TxHeaderTypeDef TxHeader_IMU_Mag;		//Header for IMU_Mag Message

uint8_t RxData[8];  //For receiving CAN Message
uint8_t ThrottleVal, SteeringVal;

uint32_t TxMailbox;


										//FUCTIONS
//Calculation functions for Distance sensor

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

//Convert Raw values from CAN message 101 to PWM values
uint8_t ConvertToPWM_Signal(uint8_t OutVal, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max){
	return (OutVal - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Generate headers for CAN messages

void TxHeaderInit_Sensor(){
	TxHeader_Sensor.Identifier = 102;
	TxHeader_Sensor.IdType= 0;
	TxHeader_Sensor.DataLength=4;
}

void TxHeaderInit_IMU_Accel(){
	TxHeader_IMU_Accel.Identifier=103;
	TxHeader_IMU_Accel.IdType= 0;
	TxHeader_IMU_Accel.DataLength=3;
}

void TxHeaderInit_IMU_Gyro(){
	TxHeader_IMU_Gyro.Identifier=104;
	TxHeader_IMU_Gyro.IdType= 0;
	TxHeader_IMU_Gyro.DataLength=3;
}

void TxHeaderInit_IMU_Mag(){
	TxHeader_IMU_Mag.Identifier=105;
	TxHeader_IMU_Mag.IdType= 0;
	TxHeader_IMU_Mag.DataLength=3;
}




