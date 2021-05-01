/*
 * switchMatrix.h
 *
 *  Created on: 8 avr. 2021
 *      Author: Anthony
 */

#ifndef INC_SWITCHMATRIX_H_
#define INC_SWITCHMATRIX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void bWriteColumnState(int, uint8_t);
int bReadRowState(int);
int intReadRowState(void);

#endif /* INC_SWITCHMATRIX_H_ */
