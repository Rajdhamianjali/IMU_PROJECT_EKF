/*
 * USER_FUNCTIONS.h
 *
 *  Created on: Jun 17, 2024
 *      Author: Anjali
 */

#ifndef INC_USER_FUNCTIONS_H_
#define INC_USER_FUNCTIONS_H_

#pragma once

#include "variables.h"
#ifdef CDC_USB_DEBUG
#include "usbd_cdc_if.h"
#endif
#include <stdarg.h>
//#include "mem_addr.h"
//#include "res_table.h"
//#include "../../COM/Com_Operations.h"

int buffersize(char *buff);

int buffersize(char *buff);
#ifdef DEBUG_MODE
#ifndef TRACE_MODE
void myprintf(const char *fmt, ...);
void myprintf_(const char *fmt, ...);
#endif
#ifdef TRACE_MODE
int write(int32_t file, uint8_t *ptr, int32_t len);
#endif
#endif


#endif /* INC_USER_FUNCTIONS_H_ */
