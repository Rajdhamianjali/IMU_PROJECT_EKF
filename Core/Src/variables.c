/*
 * variables.c
 *
 *  Created on: Jun 17, 2024
 *      Author: Anjali
 */

#include "variables.h"

HAL_StatusTypeDef UART_STATE;
HAL_StatusTypeDef SPI_STATE;
HAL_StatusTypeDef HAL_STATUS;

// storing the temperature, converted to uin8_t, in 2D format row separates to the next sensor
uint8_t SAT_IMU[18];


//IMU Variables
uint8_t serialBuf[100];
uint8_t device_id_m = 0x0F;
uint8_t IMU[18];



uint8_t DEBUG_DATA_TX_FLAG;
// MPU6500 Variables
//MPU6500_t MPU6500;
uint8_t check;
float accelPitch;
float accelRoll;


uint8_t testt[100];
