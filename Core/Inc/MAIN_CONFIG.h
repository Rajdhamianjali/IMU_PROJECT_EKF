/*
 * MAIN_CONFIG.h
 *
 *  Created on: Jun 17, 2024
 *      Author: Anjali
 */

#ifndef INC_MAIN_CONFIG_H_
#define INC_MAIN_CONFIG_H_

#pragma once

#include "main.h"

#define DEBUG_MODE
//#define TRACE_MODE

//#define CDC_USB_DEBUG
#define UART_DEBUG
#define BAUDRATE_4800_BPS
//#define BAUDRATE_2400_BPS

//#define
#define DEBUG_STREAM			huart1
#define IMU_STREAM				hspi1

#define MAIN_STREAM				huart6

#endif /* INC_MAIN_CONFIG_H_ */
