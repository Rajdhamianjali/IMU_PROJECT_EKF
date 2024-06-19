/*
 * variables.h
 *
 *  Created on: Jun 17, 2024
 *      Author: Anjali
 */

#ifndef INC_VARIABLES_H_
#define INC_VARIABLES_H_
#pragma once


#include "main.h"
#include "MAIN_CONFIG.h"
#include "IMU.h"

//IMU initialization variables
//Initialization variables
#define CS_SEL		0
#define CS_DES     1
#define TRUE  1
#define FALSE 0

extern uint8_t serialBuf[100];
extern uint8_t IMU[18];

extern uint8_t DEBUG_DATA_TX_FLAG;

extern uint8_t SAT_IMU[18];

//MPU Variables
extern uint8_t check;
extern float accelPitch;
extern float accelRoll;

extern uint8_t testt[100];

#endif /* INC_VARIABLES_H_ */
