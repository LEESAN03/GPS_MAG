#ifndef __QMC5883L_H
#define __QMC5883L_H

#include "stm32f10x.h"
#include "IOI2C.h"
#include "delay.h"

#define  QMC58X3_ADDR   0x18//0x3C // 7 bit address of the HMC58X3 used with the Wire library

#define  X_OUTPUT_LSB           0x00
#define  X_OUTPUT_MSB           0x01
#define  Y_OUTPUT_LSB           0x02
#define  Y_OUTPUT_MSB           0x03
#define  Z_OUTPUT_LSB           0x04
#define  Z_OUTPUT_MSB           0x05

#define  QMC5883_STATUS_ADDR    0x06
#define  QMC5883_CONFIG_ADDR    0x09
#define  QMC5883_CONTROL_ADDR   0x0A
#define  QMC5883_RESET_ADDR     0x0B


void QMC5883L_SetUp(void);	//≥ı ºªØ

void QMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z);
void Read_5883(void);

extern float x_mag,y_mag,z_mag;
extern int magSampleCount;
#endif


