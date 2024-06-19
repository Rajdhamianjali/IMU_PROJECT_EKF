/*
 * EKF.h
 *
 *  Created on: Jun 18, 2024
 *      Author: Anjali
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

#include "math.h"

typedef struct Eekf_ {

	float phi_r;
	float theta_r;

	float P[2][2];

	float Q[2];
	float R[3];

} Eekf;

void EKF_Init(Eekf *ekf_1, float P[2], float Q[2], float R[3]);

void EKF_Predict(Eekf *ekf_1, float p_rps, float q_rps, float r_rps,
		float sampleTime_s);

void EKF_Update(Eekf *ekf_1, float ax_mps2, float ay_mps2, float az_mps2);

#endif /* INC_EKF_H_ */
