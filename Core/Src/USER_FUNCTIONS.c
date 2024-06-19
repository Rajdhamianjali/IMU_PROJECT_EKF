/*
 * USER_FUNCTIONS.c
 *
 *  Created on: Jun 17, 2024
 *      Author: Anjali
 */


#include "USER_FUNCTIONS.h"
/*
 * @brief	counts the number of non-null data in given array
 *
 * @param	buff	pointer to the array of data to be counted
 * @retval	int		number of non-null values in the array
 */
int buffersize(char *buff) {
	int i = 0;
	while (*buff++ != '\0')
		i++;
	return i;
}


/*
 * @brief	Outputs the string data to the PC, via USB CDC
 *
 * @param	fmt	pointer the array of characters data to be transmitted
 *
 * @retval	none
 */
void myprintf(const char *fmt, ...) {
	static char temp[100];
		va_list args;
		va_start(args, fmt);
		vsnprintf(temp, sizeof(temp), fmt, args);
		va_end(args);
		int len = buffersize(temp);
		HAL_UART_Transmit(&DEBUG_STREAM, (uint8_t*) temp, len, 1000);
}
