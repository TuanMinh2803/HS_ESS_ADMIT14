/*
 * EKF.h
 *
 *  Created on: Oct 15, 2025
 *      Author: Minh
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

#include <math.h>

typedef struct{
	float phi_r;
	float theta_r;

	float P[2][2];

	float Q[2];
	float R[3];
} EKF;

void EKF_Init(EKF *ekf, float P[2], float Q[2], float R[3]);

void EKF_Predit(EKF *ekf, float p_rps, float q_rps, float r_rps, float SampleTime_s);

void EKF_Update(EKF *ekf, float ax_mps2, float ay_mps2, float az_mps2);

#endif /* INC_EKF_H_ */
