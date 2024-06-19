/*
 * IMU.c
 *
 *  Created on: Jun 17, 2024
 *      Author: Anjali
 */

#include "IMU.h"
#include "variables.h"
#include "USER_FUNCTIONS.h"
#include "estimator_main.h"

extern MPU6500_t MPU6500;
extern lsm9ds1_t lsm9ds1;

//MPU definations

const uint8_t READWRITE_CMD = 0x80;

// MPU6500 registers
const uint8_t ACCEL_OUT = 0x3B;
const uint8_t GYRO_OUT = 0x43;
const uint8_t TEMP_OUT = 0x41;

const uint8_t EXT_SENS_DATA_00 = 0x49;

const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_CONFIG2 = 0x1D;

const uint8_t ACCEL_FS_SEL_2G = 0x00;
const uint8_t ACCEL_FS_SEL_4G = 0x08;
const uint8_t ACCEL_FS_SEL_8G = 0x10;
const uint8_t ACCEL_FS_SEL_16G = 0x18;

const uint8_t GYRO_FS_SEL_250DPS = 0x00;
const uint8_t GYRO_FS_SEL_500DPS = 0x08;
const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
const uint8_t GYRO_FS_SEL_2000DPS = 0x18;

const uint8_t CONFIG = 0x1A;

#define WHO_AM_I_6500_ANS 0x70
#define WHO_AM_I          0x75
#define USER_CTRL         0x6A
#define PWR_MGMT_1        0x6B

#define READWRITE         0x80
#define SPI_TIMOUT_MS     1000

static uint8_t _buffer[14];
uint8_t _buffer1;
uint8_t _buffer2;

sat_att_combined combined_sat_att;

/**
 * @brief  Read data from Specific Register address of LSM9DS1
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @param  add Register address from which data is to be read
 */
uint8_t LSM9DS1_ReadReg(SPI_HandleTypeDef *hspi, uint8_t add) {
	uint8_t val;
	add |= 0x80;  // set the MSB to indicate a read operation
	HAL_GPIO_WritePin(GPIOA, CS_MAG_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &add, 1, 100);
	HAL_SPI_Receive(hspi, &val, 1, 100);
	HAL_GPIO_WritePin(GPIOA, CS_MAG_Pin, GPIO_PIN_SET);
	return val;
}

/**
 * @brief  Write on Specific Register address of LSM9DS1
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @param  add Register address where certain value is to be written
 */
void LSM9DS1_WriteReg(SPI_HandleTypeDef *hspi, uint8_t add, uint8_t val) {
	add &= 0x7F;  // clear the MSB to indicate a write operation
	HAL_GPIO_WritePin(GPIOA, CS_MAG_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &add, 1, 100);
	HAL_SPI_Transmit(hspi, &val, 1, 100);
	HAL_GPIO_WritePin(GPIOA, CS_MAG_Pin, GPIO_PIN_SET);

}

/**
 * @brief  Initialize LSM9DS1 to work in 16-bit, 1.25Hz ODR, ±4 Gauss and Continuous conversion Mode
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @param  ctrl2 Control Register Value to choose LSM9DS1 sensor Scale
 */
void LSM9DS1_Init(SPI_HandleTypeDef *hspi, uint8_t ctrl2) {
	uint8_t ctrl1 = 0x74; // set the magnetic resolution to 16-bit, 20 Hz ODR, UHP mode in X-Y axis
	LSM9DS1_WriteReg(hspi, LSM9DS1_CTRL_REG1_M, ctrl1);

	/* Change the full-scale range to ±4 Gauss */

	//value to set the full-scale range
	LSM9DS1_WriteReg(hspi, LSM9DS1_CTRL_REG2_M, ctrl2);

	/* Change the control register 3 to continuous conversion mode */

	uint8_t ctrl3 = 0x00; // value to set the continuous conversion mode
	LSM9DS1_WriteReg(hspi, LSM9DS1_CTRL_REG3_M, ctrl3);

	uint8_t ctrl4 = 0x0C; // value to set the UHP mode on Z-axis
	LSM9DS1_WriteReg(hspi, LSM9DS1_CTRL_REG4_M, ctrl4);
}

//MPU6500 Functions

uint8_t MPU_begin(SPI_HandleTypeDef *SPIx, MPU6500_t *pMPU6500) {
	// Initialize variables
	uint8_t addr, val;
//	 Confirm device
//	HAL_Delay(100);
	whoAmI();
	readRegisters(WHO_AM_I, 1, &check);
	if (check == WHO_AM_I_6500_ANS) {
//	 Startup / reset the sensor
		addr = PWR_MGMT_1;
		val = 0x00;
		writeRegister(addr, val);

		// Disable I2C (SPI only)
		addr = USER_CTRL;
		val = 0x10;
		writeRegister(addr, val);

		// Configure DLPF value()
		val = 0x11;
		MPU6500_SetDLPFBandwidth(DLPF_BANDWIDTH_20HZ);

		// Set the full scale ranges
		MPU_writeAccFullScaleRange(pMPU6500,
				pMPU6500->settings.aFullScaleRange);
		MPU_writeGyroFullScaleRange(pMPU6500,
				pMPU6500->settings.gFullScaleRange);
		return 1;
	} else {
		return 0;
	}
}

void MPU_CS(uint8_t state) {
	HAL_GPIO_WritePin(GPIOB, CS_MPU_Pin, state);
}

uint8_t SPIx_WriteRead(uint8_t Byte) {
	uint8_t receivedbyte = 0;
	if (HAL_SPI_TransmitReceive(&IMU_STREAM, (uint8_t*) &Byte,
			(uint8_t*) &receivedbyte, 1, 0x1000) != HAL_OK) {
		return -1;
	} else {
	}
	return receivedbyte;
}

void MPU_SPI_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite) {
	MPU_CS(CS_SEL);
	SPIx_WriteRead(WriteAddr);
	while (NumByteToWrite >= 0x01) {
		SPIx_WriteRead(*pBuffer);
		NumByteToWrite--;
		pBuffer++;
	}
	MPU_CS(CS_DES);
}

void MPU_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead) {

	MPU_CS(CS_SEL);
	uint8_t data = ReadAddr | READWRITE_CMD;
	HAL_SPI_Transmit(&IMU_STREAM, &data, 1, 100);
	HAL_SPI_Receive(&IMU_STREAM, pBuffer, NumByteToRead, 100);
	MPU_CS(CS_DES);
}

/* writes a byte to MPU6500 register given a register address and data */
void writeRegister(uint8_t subAddress, uint8_t data) {
	MPU_SPI_Write(&data, subAddress, 1);
	HAL_Delay(10);
}

/* reads registers from MPU6500 given a starting register address, number of bytes, and a pointer to store data */
void readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest) {
	MPU_SPI_Read(dest, subAddress, count);
}

/* gets the MPU6500 WHO_AM_I register value, expected to be 0x71 */
uint8_t whoAmI() {
	// read the WHO AM I register
	readRegisters(WHO_AM_I, 1, &_buffer1);

	// return the register value
	return _buffer1;
}

/* sets the DLPF bandwidth to values other than default */
void MPU6500_SetDLPFBandwidth(DLPFBandwidth bandwidth) {
	writeRegister(ACCEL_CONFIG2, bandwidth);
	writeRegister(CONFIG, bandwidth);
}

/// @brief Set the accelerometer full scale range
/// @param SPIx Pointer to SPI structure config
/// @param pMPU6500 Pointer to master MPU6500 struct
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g
void MPU_writeAccFullScaleRange(MPU6500_t *pMPU6500, uint8_t aScale) {
	// Variable init
	uint8_t addr = ACCEL_CONFIG;
	uint8_t val;

	// Set the value
	switch (aScale) {
	case AFSR_2G:
		pMPU6500->sensorData.aScaleFactor = 16384.0;
		val = 0x00;
		writeRegister(addr, val);
		break;
	case AFSR_4G:
		pMPU6500->sensorData.aScaleFactor = 8192.0;
		val = 0x08;
		writeRegister(addr, val);
		break;
	case AFSR_8G:
		pMPU6500->sensorData.aScaleFactor = 4096.0;
		val = 0x10;
		writeRegister(addr, val);
		break;
	case AFSR_16G:
		pMPU6500->sensorData.aScaleFactor = 2048.0;
		val = 0x18;
		writeRegister(addr, val);
		break;
	default:
		pMPU6500->sensorData.aScaleFactor = 8192.0;
		val = 0x08;
		writeRegister(addr, val);
		break;
	}
}

/// @brief Set the gyroscope full scale range
/// @param SPIx Pointer to SPI structure config
/// @param pMPU6500 Pointer to master MPU6500 struct
/// @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s
void MPU_writeGyroFullScaleRange(MPU6500_t *pMPU6500, uint8_t gScale) {
	// Variable init
	uint8_t addr = GYRO_CONFIG;
	uint8_t val;

	// Set the value
	switch (gScale) {
	case GFSR_250DPS:
		pMPU6500->sensorData.gScaleFactor = 131.0;
		val = 0x00;
		writeRegister(addr, val);
		break;
	case GFSR_500DPS:
		pMPU6500->sensorData.gScaleFactor = 65.5;
		val = 0x08;
		writeRegister(addr, val);
		break;
	case GFSR_1000DPS:
		pMPU6500->sensorData.gScaleFactor = 32.8;
		val = 0x10;
		writeRegister(addr, val);
		break;
	case GFSR_2000DPS:
		pMPU6500->sensorData.gScaleFactor = 16.4;
		val = 0x18;
		writeRegister(addr, val);
		break;
	default:
		pMPU6500->sensorData.gScaleFactor = 65.5;
		val = 0x08;
		writeRegister(addr, val);
		break;
	}
}

uint8_t MPU6500_ReadReg(SPI_HandleTypeDef *hspi, uint8_t add) {
	uint8_t val;
	add |= 0x80;  // set the MSB to indicate a read operation
	MPU_CS(CS_SEL);
	HAL_SPI_Transmit(hspi, &add, 1, 100);
	HAL_SPI_Receive(hspi, &val, 1, 100);
	MPU_CS(CS_DES);
	return val;
}

/* read the data, each argument should point to a array for x, y, and x */
void MPU6500_GetData(MPU6500_t *pMPU6500) {
	// grab the data from the MPU6500
	readRegisters(ACCEL_OUT, 14, _buffer);

	// combine into 16 bit values
	pMPU6500->rawData.ax = (((int16_t) _buffer[0]) << 8) | _buffer[1];
	pMPU6500->rawData.ay = (((int16_t) _buffer[2]) << 8) | _buffer[3];
	pMPU6500->rawData.az = (((int16_t) _buffer[4]) << 8) | _buffer[5];

//	Accelerometer after offset reduction
//	pMPU6500->rawData.ax = pMPU6500->rawData.ax - 32767;
//	pMPU6500->rawData.ay = pMPU6500->rawData.ay - 32767;
//	pMPU6500->rawData.az = pMPU6500->rawData.az - 15650 + 16384;

	pMPU6500->rawData.gx = (((int16_t) _buffer[8]) << 8) | _buffer[9];
	pMPU6500->rawData.gy = (((int16_t) _buffer[10]) << 8) | _buffer[11];
	pMPU6500->rawData.gz = (((int16_t) _buffer[12]) << 8) | _buffer[13];
}

/// @brief Find offsets for each axis of gyroscope
/// @param SPIx Pointer to SPI structure config
/// @param pMPU6500 Pointer to master MPU6500 struct
/// @param numCalPoints Number of data points to average
void MPU_calibrateGyro(MPU6500_t *pMPU6500, uint16_t numCalPoints) {
	// Init
	int32_t x = 0;
	int32_t y = 0;
	int32_t z = 0;

	// Zero guard
	if (numCalPoints == 0) {
		numCalPoints = 1;
	}

	// Save specified number of points
	for (uint16_t ii = 0; ii < numCalPoints; ii++) {
		MPU6500_GetData(pMPU6500);

		x += pMPU6500->rawData.gx;
		y += pMPU6500->rawData.gy;
		z += pMPU6500->rawData.gz;
		HAL_Delay(3);
	}

	// Average the saved data points to find the gyroscope offset
	pMPU6500->gyroCal.x = (float) x / (float) numCalPoints;
	pMPU6500->gyroCal.y = (float) y / (float) numCalPoints;
	pMPU6500->gyroCal.z = (float) z / (float) numCalPoints;
}

/// @brief Calculate the real world sensor values
/// @param SPIx Pointer to SPI structure config
/// @param pMPU6500 Pointer to master MPU6500 struct
void MPU_readProcessedData(MPU6500_t *pMPU6500) {
	// Get raw values from the IMU
	MPU6500_GetData(pMPU6500);

	// Convert accelerometer values to g's
	pMPU6500->sensorData.ax = pMPU6500->rawData.ax
			/ pMPU6500->sensorData.aScaleFactor;
	pMPU6500->sensorData.ay = pMPU6500->rawData.ay
			/ pMPU6500->sensorData.aScaleFactor;
	pMPU6500->sensorData.az = pMPU6500->rawData.az
			/ pMPU6500->sensorData.aScaleFactor;

	// Compensate for gyro offset
	pMPU6500->sensorData.gx = pMPU6500->rawData.gx - pMPU6500->gyroCal.x;
	pMPU6500->sensorData.gy = pMPU6500->rawData.gy - pMPU6500->gyroCal.y;
	pMPU6500->sensorData.gz = pMPU6500->rawData.gz - pMPU6500->gyroCal.z;

//	// Convert gyro values to deg/s
//	pMPU6500->sensorData.gx /= pMPU6500->sensorData.gScaleFactor;
//	pMPU6500->sensorData.gy /= pMPU6500->sensorData.gScaleFactor;
//	pMPU6500->sensorData.gz /= pMPU6500->sensorData.gScaleFactor;

	// Convert gyro values to rad/s
	pMPU6500->sensorData.gx = (pMPU6500->sensorData.gx
			/ pMPU6500->sensorData.gScaleFactor) * DEG2RAD;
	pMPU6500->sensorData.gy = (pMPU6500->sensorData.gy
			/ pMPU6500->sensorData.gScaleFactor) * DEG2RAD;
	pMPU6500->sensorData.gz = (pMPU6500->sensorData.gz
			/ pMPU6500->sensorData.gScaleFactor) * DEG2RAD;
}

/// @brief Calculate the attitude of the sensor in degrees using a complementary filter
/// @param SPIx Pointer to SPI structure config
/// @param pMPU6500 Pointer to master MPU6500 struct
void MPU_calcAttitude(MPU6500_t *pMPU6500) {
	// Read processed data
	MPU_readProcessedData(pMPU6500);

	imu_filter filt_imu;

	sat_attitude attitude_sat;

	myprintf("Sensor data\n");
	myprintf(" ax = %.2f\r\n", pMPU6500->sensorData.ax);
	myprintf(" ay = %.2f\r\n", pMPU6500->sensorData.ay);
	myprintf(" az = %.2f\r\n", pMPU6500->sensorData.az);
	myprintf(" gx = %.2f\r\n", pMPU6500->sensorData.gx);
	myprintf(" gy = %.2f\r\n", pMPU6500->sensorData.gy);
	myprintf(" gz = %.2f\r\n", pMPU6500->sensorData.gz);

	filt_imu = IMU_RCFilter(pMPU6500);

	myprintf("Filtered accelerometer data\n");
	myprintf(" ax_mps2 = %.2f\r\n", filt_imu.ax_mps2);
	myprintf(" ay_mps2 = %.2f\r\n", filt_imu.ay_mps2);
	myprintf(" az_mps2 = %.2f\r\n", filt_imu.az_mps2);

	myprintf("Filtered gyroscope data\n");
	myprintf(" p_rps = %.2f\r\n", filt_imu.p_rps);
	myprintf(" q_rps = %.2f\r\n", filt_imu.q_rps);
	myprintf(" r_rps = %.2f\r\n", filt_imu.r_rps);

	Attitude_genEstimate(&filt_imu, &attitude_sat);
	Attitude_compleEstimate(&filt_imu, &combined_sat_att);
	Attitude_ekfEstimate(&filt_imu, &combined_sat_att);

	// Complementary filter
	accelPitch = atan2(pMPU6500->sensorData.ay,
			pMPU6500->sensorData.az) * RAD2DEG;
	accelRoll = atan2(pMPU6500->sensorData.ax,
			pMPU6500->sensorData.az) * RAD2DEG;

	pMPU6500->attitude.r = pMPU6500->attitude.tau
			* (pMPU6500->attitude.r
					+ pMPU6500->sensorData.gy * pMPU6500->attitude.dt)
			+ (1 - pMPU6500->attitude.tau) * accelRoll;
	pMPU6500->attitude.p = pMPU6500->attitude.tau
			* (pMPU6500->attitude.p
					- pMPU6500->sensorData.gx * pMPU6500->attitude.dt)
			+ (1 - pMPU6500->attitude.tau) * accelPitch;
	pMPU6500->attitude.y += (pMPU6500->sensorData.gz * pMPU6500->attitude.dt);

	float magx = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTX_L_M);
	float magy = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTY_L_M);
	float magz = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTZ_L_M);

	float rollRad = pMPU6500->attitude.r / 180 * 3.14159265359;
	float pitchRad = pMPU6500->attitude.p / 180 * 3.14159265359;

	float Xm = magx * cos(pitchRad) - magy * sin(rollRad) * sin(pitchRad)
			+ magz * cos(rollRad) * sin(pitchRad);

	float Ym = magy * cos(rollRad) + magz * sin(rollRad);

	pMPU6500->attitude.y = atan2(Ym, Xm) * RAD2DEG;

}
uint8_t Mag_Data[6];
void MagRead(lsm9ds1_t *plsm9ds1) {

	double Total = 0;
	Mag_Data[0] = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTX_L_M);
	Mag_Data[1] = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTX_H_M);
	Mag_Data[2] = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTY_L_M);
	Mag_Data[3] = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTY_H_M);
	Mag_Data[4] = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTZ_L_M);
	Mag_Data[5] = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTZ_H_M);

	plsm9ds1->mag.mx = (int16_t) ((Mag_Data[1] << 8) | Mag_Data[0]);
	plsm9ds1->mag.my = (int16_t) ((Mag_Data[3] << 8) | Mag_Data[2]);
	plsm9ds1->mag.mz = (int16_t) ((Mag_Data[5] << 8) | Mag_Data[4]);

	float temp_mx = (float) (plsm9ds1->mag.mx * LSM9DS1_SENSITIVITY);
	float temp_my = (float) (plsm9ds1->mag.my * LSM9DS1_SENSITIVITY);
	float temp_mz = (float) (plsm9ds1->mag.mz * LSM9DS1_SENSITIVITY);

	Total = sqrt(
			(float) temp_mx * temp_mx + temp_my * temp_my + temp_mz * temp_mz);

	myprintf("Magnetometer data\n");
	myprintf(" mx = %.2f\r\n", temp_mx);
	myprintf(" my = %.2f\r\n", temp_my);
	myprintf(" mz = %.2f\r\n", temp_my);
	myprintf(" Total = %.2f\r\n", (double) Total);

	myprintf(" roll = %.2f\r\n", combined_sat_att.roll);
	myprintf(" pitch = %.2f\r\n", combined_sat_att.pitch);
	combined_sat_att.yaw = 0;
	myprintf(" yaw = %.2f\r\n", combined_sat_att.yaw);

	float Xm = temp_mx * cos(combined_sat_att.pitch)
			- temp_my * sin(combined_sat_att.roll) * sin(combined_sat_att.pitch)
			+ temp_mz * cos(combined_sat_att.roll)
					* sin(combined_sat_att.pitch);

	float Ym = temp_my * cos(combined_sat_att.roll)
			+ temp_mz * sin(combined_sat_att.roll);

	combined_sat_att.yaw = atan2(Ym, Xm);

	myprintf(" roll = %.2f\r\n", combined_sat_att.roll);
	myprintf(" pitch = %.2f\r\n", combined_sat_att.pitch);
	myprintf(" yaw = %.2f\r\n", combined_sat_att.yaw);

}

void IMU_Setup() {

	LSM9DS1_Init(&IMU_STREAM, 0x60); // ±16 Gauss full scale, 20Hz ODR, Continuous conversion mode
	HAL_Delay(1000);

	if (MPU_begin(&IMU_STREAM, &MPU6500) != TRUE) {
		sprintf((char*) serialBuf, "ERROR!\r\n");
		HAL_UART_Transmit(&DEBUG_STREAM, serialBuf, strlen((char*) serialBuf),
				100);
	} else {
		myprintf("MPU6500 Initialization Complete..");
	}

	//	 Calibrate the IMU
	myprintf("\nNow, Calibrating sensors.\r\n");
	HAL_Delay(1);
	MPU_calibrateGyro(&MPU6500, 1500);
	HAL_Delay(1000);
	myprintf("Calibration  Complete!!!\n\n");

}

void IMU_Data() {

	//Gyro, Accel and Mag register data

	SAT_IMU[0] = MPU6500_ReadReg(&IMU_STREAM, ACCEL_XOUT_H);
	SAT_IMU[1] = MPU6500_ReadReg(&IMU_STREAM, ACCEL_XOUT_L);
	SAT_IMU[2] = MPU6500_ReadReg(&IMU_STREAM, ACCEL_YOUT_H);
	SAT_IMU[3] = MPU6500_ReadReg(&IMU_STREAM, ACCEL_YOUT_L);
	SAT_IMU[4] = MPU6500_ReadReg(&IMU_STREAM, ACCEL_ZOUT_H);
	SAT_IMU[5] = MPU6500_ReadReg(&IMU_STREAM, ACCEL_ZOUT_L);

	SAT_IMU[6] = MPU6500_ReadReg(&IMU_STREAM, GYRO_XOUT_H);
	SAT_IMU[7] = MPU6500_ReadReg(&IMU_STREAM, GYRO_XOUT_L);
	SAT_IMU[8] = MPU6500_ReadReg(&IMU_STREAM, GYRO_YOUT_H);
	SAT_IMU[9] = MPU6500_ReadReg(&IMU_STREAM, GYRO_YOUT_L);
	SAT_IMU[10] = MPU6500_ReadReg(&IMU_STREAM, GYRO_ZOUT_H);
	SAT_IMU[11] = MPU6500_ReadReg(&IMU_STREAM, GYRO_ZOUT_L);

	SAT_IMU[12] = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTX_L_M);
	SAT_IMU[13] = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTX_H_M);
	SAT_IMU[14] = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTY_L_M);
	SAT_IMU[15] = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTY_H_M);
	SAT_IMU[16] = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTZ_L_M);
	SAT_IMU[17] = LSM9DS1_ReadReg(&IMU_STREAM, LSM9DS1_OUTZ_H_M);

	HAL_UART_Transmit(&DEBUG_STREAM, SAT_IMU, sizeof(SAT_IMU), 1000);
}

void HK_IMU() {
	IMU_Setup();
	HAL_Delay(1000);
	IMU_Data();
}
