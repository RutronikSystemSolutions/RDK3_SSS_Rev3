/*
 * d6t32l01a.h
 *
 *  Created on: 2023-11-07
 *      Author: Gintaras Drukteinis
 */

#ifndef D6T32L01A_H_
#define D6T32L01A_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* D6T32L01A defines */
#define D6T_ADDR 				0x0A
#define D6T_CMD 				0x4D
#define D6T_SET_ADD 			0x01
#define N_ROW 					32
#define N_PIXEL 				(32 * 32)
#define N_READ 					((N_PIXEL + 1) * 2 + 1)
#define D6T_IIR 				0x0F
#define D6T_AVERAGE 			0x00
#define D6T_READ_FAIL			23
#define D6T_WRITE_FAIL			24
#define D6T_PEC_FAIL			0x01
#define D6T_OK					0x00
#define D6T_I2C_TIMEOUT			5U

/*Exported Function Prototypes*/
uint32_t d6t32_init(void);
int D6T_getvalue(uint8_t *buf, float *ptat, float *pix_data);

#endif /* D6T32L01A_H_ */
