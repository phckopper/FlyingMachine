/*
 * PMW3901.h
 *
 *  Created on: Feb 8, 2018
 *      Author: phckopper
 */

#ifndef PMW3901_H_
#define PMW3901_H_

#include <stdint.h>

#define SQUAL_MIN 0x19
#define SHUTTER_MAX 0x1f

#define FRAME_SIZE (35*35)

typedef enum {
	PMW_OK,
	PMW_BROKEN_CONNECTION,
	PMW_INVALID_DATA,
	PMW_TIMEOUT
} PMW_Return_Code;

extern uint8_t RAW_FRAME[FRAME_SIZE];

PMW_Return_Code PMW3901_Get_Frame(void);
uint8_t PMW3901_Debug(void);
uint8_t PMW3901_RawData_Sum(void);
PMW_Return_Code PMW3901_Read_Motion(int16_t *deltaX, int16_t *deltaY);
PMW_Return_Code PMW3901_Init(void);

#endif /* PMW3901_H_ */
