#ifndef __GPS_H
#define __GPS_H

#include "stm32f10x.h"

#define   GPS_A_ERROR_NOT_EXIST          0x01
#define   GPS_A_ERROR_LINK_MASK          0x02
#define   GPS_A_ERROR_DATA_MASK          0x04

#define   TOTAL_GPS_BUF_LEN              200

extern unsigned char GPS_STA_Num;
extern int GPS_Altitude, GPS_Course;   
extern int32_t Latitude_GPS, Longitude_GPS; //纬度	 单位为度  //经度  单位为度
extern int GPS_Vel_N, GPS_Vel_E, GPS_Vel_D, GPS_GroundSpeed;
extern unsigned char Gps_Month, Gps_Day, Gps_Hour, Gps_Min, Gps_Sec;
extern unsigned short Gps_Year;
extern unsigned int GPS_MiniSec;
extern unsigned char GPS_exsit, gpsArrived, gps_status, gps_checksum_ok;
extern float GPS_PDOP;
extern unsigned int error_link_occurs_count, error_data_occurs_count;
extern unsigned short readIndex, total_gps_len, recieve_gps_len;

void Config_Gps(void);
void GPS_Decode(void);
void Check_Gps_Status(void);
#endif


