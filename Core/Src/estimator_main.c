/*
 * estimator_main.c
 *
 *  Created on: Jun 18, 2024
 *      Author: Anjali
 */

#include "estimator_main.h"

#define G_MPS2 				9.8100000000f

#define SAMPLE_TIME_MS_USB_ 		1000
#define COMP_FILT_ALPHA 		0.0500000000f

#define KALMAN_PREDICT_PERIOD_MS 	10
#define KALMAN_UPDATE_PERIOD_MS 	100

imu_filter IMU_RCFilter(MPU6500_t *imu) {
	RCFilter lpfAcc[3];
	RCFilter lpfGyr[3];
	imu_filter filt_imu;

	for (int n = 0; n < 3; n++) {
		RCFilter_Init(&lpfAcc[n], 5.0f, 0.01f);
		RCFilter_Init(&lpfGyr[n], 25.0f, 0.01f);
	}

//	/* Filter accelerometer data */
//	RCFilter_Update(&lpfAcc[0], imu->acc_mps2[0]);
//	RCFilter_Update(&lpfAcc[1], imu->acc_mps2[1]);
//	RCFilter_Update(&lpfAcc[2], imu->acc_mps2[2]);

	/* Filter accelerometer data */
	RCFilter_Update(&lpfAcc[0], imu->sensorData.ax);
	RCFilter_Update(&lpfAcc[1], imu->sensorData.ay);
	RCFilter_Update(&lpfAcc[2], imu->sensorData.az);

//	/* Filter gyroscope data */
//	RCFilter_Update(&lpfGyr[0], imu->gyr_rps[0]);
//	RCFilter_Update(&lpfGyr[1], imu->gyr_rps[1]);
//	RCFilter_Update(&lpfGyr[2], imu->gyr_rps[2]);

	/* Filter gyroscope data */
	RCFilter_Update(&lpfGyr[0], imu->sensorData.gx);
	RCFilter_Update(&lpfGyr[1], imu->sensorData.gy);
	RCFilter_Update(&lpfGyr[2], imu->sensorData.gz);

	//Filtered accelerometer measurement
	filt_imu.ax_mps2 = lpfAcc[0].out[0];
	filt_imu.ay_mps2 = lpfAcc[1].out[0];
	filt_imu.az_mps2 = lpfAcc[2].out[0];
	//Filtered Gyroscope measurement
	filt_imu.p_rps = lpfGyr[0].out[0];
	filt_imu.q_rps = lpfGyr[1].out[0];
	filt_imu.r_rps = lpfGyr[2].out[0];
	return filt_imu;
}

void Attitude_genEstimate(imu_filter *filt, sat_attitude *att) {
	float phiHat_deg_ = 0.0f;
	float thetaHat_deg_ = 0.0f;

	/*Calculate roll (phi) and pitch(theta) angle estimates using filtered accelerometer readings*/
	phiHat_deg_ = atanf(filt->ay_mps2 / filt->az_mps2) * RAD2DEG;
	thetaHat_deg_ = asinf(filt->ax_mps2 / G_MPS2) * RAD2DEG;

//	phiHat_deg_ = atanf(filt->ay_mps2 / filt->az_mps2);
//	thetaHat_deg_ = asinf(filt->ax_mps2 / G_MPS2);

	myprintf("Accel phiHat_deg : %.2f\n", (double) phiHat_deg_);
	myprintf("Accel thetahat_deg : %.2f\n", (double) thetaHat_deg_);

	att->accel_phiHat = phiHat_deg_;
	att->accel_thetaHat = thetaHat_deg_;

	phiHat_deg_ = 0.0f;
	thetaHat_deg_ = 0.0f;

	//Transform body rates to Euler rates to get estimate of roll and pitch angles using filtered gyroscope reading
	float phiDot_rps = filt->p_rps
			+ tanf(thetaHat_deg_)
					* (sinf(phiHat_deg_) * filt->q_rps
							+ cosf(phiHat_deg_) * filt->r_rps);
	float thetaDot_rps = cosf(phiHat_deg_) * filt->q_rps
			- sinf(phiHat_deg_) * filt->r_rps;

	//Integrate Euler rates to get estimate of roll and pitch angles
	phiHat_deg_ = (phiHat_deg_ + (SAMPLE_TIME_MS_USB_ / 1000.0f) * phiDot_rps)
			* RAD2DEG;
	thetaHat_deg_ = (thetaHat_deg_
	+ (SAMPLE_TIME_MS_USB_ / 1000.0F) * thetaDot_rps) * RAD2DEG;


	myprintf("Gyro phiHat_deg: %.2f\n", (double) phiHat_deg_);
	myprintf("Gyro thetaHat_deg: %.2f\n", (double) thetaHat_deg_);
	att->gyro_phiHat = phiHat_deg_;
	att->gyro_thetaHat = thetaHat_deg_;
	return;
}

void Attitude_compleEstimate(imu_filter *filt, sat_att_combined *att) {

	float thetaHat_rad_comb = 0.0f;
	float phiHat_rad_comb = 0.0f;

	/*Calculate roll (phi) and pitch(theta) angle estimates using filtered accelerometer readings*/
	float phiHat_acc_rad = atanf(filt->ay_mps2 / filt->az_mps2);
	float thetaHat_acc_rad = asinf(filt->ax_mps2 / G_MPS2);

	myprintf("accel phiHat_rad : %.2f\n", (double) phiHat_acc_rad);
	myprintf("accel thetaHat_rad : %.2f\n", (double) thetaHat_acc_rad);

	//Transform body rates to Euler rates to get estimate of roll and pitch angles using filtered gyroscope readings
	float phiDot_rps = filt->p_rps
			+ tanf(thetaHat_rad_comb)
					* (sinf(phiHat_rad_comb) * filt->q_rps
							+ cosf(phiHat_rad_comb) * filt->r_rps);
	float thetaDot_rps = cosf(phiHat_rad_comb) * filt->q_rps
			- sinf(phiHat_rad_comb) * filt->r_rps;

	myprintf("Gryo phidot_rps : %.2f\n", (double) phiDot_rps);
	myprintf("Gyro thetadot_rps : %.2f\n", (double) thetaDot_rps);

	//Combining Accel and Gyro data for complementary filter
	phiHat_rad_comb = (COMP_FILT_ALPHA * phiHat_acc_rad
			+ (1.0f - COMP_FILT_ALPHA)
					* (phiHat_rad_comb
							+ (SAMPLE_TIME_MS_USB_ / 1000.0f) * phiDot_rps));
	thetaHat_rad_comb = (COMP_FILT_ALPHA * thetaHat_acc_rad
			+ (1.0f - COMP_FILT_ALPHA)
					* (thetaHat_rad_comb
							+ (SAMPLE_TIME_MS_USB_ / 1000.0f) * thetaDot_rps));
	att->pitch = RAD2DEG * thetaHat_rad_comb;
	att->roll = RAD2DEG * phiHat_rad_comb;
	myprintf("Complementary filter roll: %.2f deg\n", (double) att->roll);
	myprintf("Complementary filter pitch: %.2f deg\n", (double) att->pitch);

}

void Attitude_ekfEstimate(imu_filter *filt, sat_att_combined *att) {

	float KALMAN_P_INIT = 0.1f;
	float KALMAN_Q = 0.001f;
	float KALMAN_R = 0.011f;
	float KalmanQ[2] = { KALMAN_Q, KALMAN_Q };
	float KalmanR[3] = { KALMAN_R, KALMAN_R, KALMAN_R };
	float Kalman_P_Init[2] = { KALMAN_P_INIT, KALMAN_P_INIT };
	Eekf ekf_1;

	//Filtered accelerometer measurement
	float ax_mps2 = 0.0f;
	float ay_mps2 = 0.0f;
	float az_mps2 = 0.0f;
	//Filtered accelerometer measurement
	float p_rps = 0.0f;
	float q_rps = 0.0f;
	float r_rps = 0.0f;

	//Remapping axis data of Accel and Gyro
	ax_mps2 = -(filt->ay_mps2);
	ay_mps2 = filt->ax_mps2;
	az_mps2 = -(filt->az_mps2);
	p_rps = -(filt->q_rps);
	q_rps = filt->p_rps;
	r_rps = -(filt->r_rps);

	//Initialize kalman filter
	EKF_Init(&ekf_1, Kalman_P_Init, KalmanQ, KalmanR);
	//Prediction step using filtered gyro data
	EKF_Predict(&ekf_1, p_rps, q_rps, r_rps, 0.001f * KALMAN_PREDICT_PERIOD_MS);

	//Update step using Accel data
	EKF_Update(&ekf_1, ax_mps2, ay_mps2, az_mps2);
	att->pitch = RAD2DEG * ekf_1.theta_r;
	att->roll = RAD2DEG * ekf_1.phi_r;


	myprintf("EKF Update : pitch : %.2f deg\n", (double) att->pitch);
	myprintf("EKF Update : roll : %.2f deg\n", (double) att->roll);
}
