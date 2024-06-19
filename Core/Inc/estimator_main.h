/*
 * estimator_main.h
 *
 *  Created on: Jun 18, 2024
 *      Author: Anjali
 */

#ifndef INC_ESTIMATOR_MAIN_H_
#define INC_ESTIMATOR_MAIN_H_

#include "USER_FUNCTIONS.h"
#include "RCFilter.h"
#include "EKF.h"
#include "IMU.h"

typedef struct imu_filter_ {
	//filtered accelermeter data
	float ax_mps2;
	float ay_mps2;
	float az_mps2;

	//filtered gyroscope data
	float p_rps;
	float q_rps;
	float r_rps;
} imu_filter;

typedef struct sat_attitude_ {
	float accel_phiHat;		//accel roll data
	float accel_thetaHat;	//accel pitch data
	float gyro_phiHat;		//gyro roll data
	float gyro_thetaHat;		//gyro pitch data
} sat_attitude;

typedef struct sat_att_combined_ {
	float roll;
	float pitch;
	float yaw;
} sat_att_combined;

imu_filter IMU_RCFilter(MPU6500_t *imu);

void Attitude_genEstimate(imu_filter *filt, sat_attitude *att);

void Attitude_compleEstimate(imu_filter *filt, sat_att_combined *att);

void Attitude_ekfEstimate(imu_filter *filt, sat_att_combined *att);

#endif /* INC_ESTIMATOR_MAIN_H_ */
