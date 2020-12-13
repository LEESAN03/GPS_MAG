#ifndef __COMPUTE_H
#define __COMPUTE_H	 

#include "stdint.h"

#define EARTH_R          6378500   // m
#define RAD_PER_DEG      (float)0.0174533
#define DEG_PER_RAD      (flaot)52.29578
  

float Math_fConstrain(float value, float min, float max);
int16_t Math_Constrain(int16_t value,int16_t min,int16_t max);
int16_t Math_abs(int16_t value);
int16_t Math_min(int16_t value1,int16_t value2);
int16_t Math_max(int16_t value1,int16_t value2);


void buf2float(float *tfloat, unsigned char *buf);
void buf2long(long *tfloat, unsigned char *buf);
void buf2int(int *tint, unsigned char *buf);
void buf2short(short *tshort, unsigned char *buf);
void float2buf(unsigned char *buf,float *tfloat);
void int2buf(unsigned char *buf,int *tint);
void short2buf(unsigned char *buf,short *tshort);
int brinv(float a[],int n); 

uint16_t Get_Crc16(uint8_t *puchMsg, uint16_t usDataLen);
uint8_t Get_Crc8(uint8_t *ptr, uint16_t len);
uint16_t Checksum_Sum(uint8_t *buf, uint16_t len);
unsigned int cal_crc16(unsigned char buffer[], unsigned char len);
#endif
//------------------End of File----------------------------



