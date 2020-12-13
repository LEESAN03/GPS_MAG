#include "delay.h"
#include "string.h"
#include "compute.h"
#include "UART1.h"
#include "GPS.h"
#include "TIMER_CONFIG.h"
#include "chipFlash.h"

// GPS配置信息指令
unsigned char setubx[28]={0xb5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xd0,0x08,0x00,0x00,0x00,0xc2,0x01,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xb8,0x42};	//set UBX protocol
unsigned char enableMsg[16] = {0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1};
unsigned char initc[14]= {0xb5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x00,0x00,0x79,0x10};
//unsigned char enable_beidou[68] = {0xB5,0x62,0x06,0x3E,0x3C,0x00,0x00,0x00,0x20,0x07,0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,0x01,0x01,0x03,0x00,0x01,
//											  0x00,0x01,0x01,0x02,0x04,0x08,0x00,0x00,0x00,0x01,0x01,0x03,0x08,0x10,0x00,0x01,0x00,0x01,0x01,0x04,0x00,0x08,0x00,
//											  0x00,0x00,0x01,0x01,0x05,0x00,0x03,0x00,0x01,0x00,0x01,0x01,0x06,0x08,0x0E,0x00,0x00,0x00,0x01,0x01,0x2F,0xA1};
	
unsigned char enable_beidou[52] = {0xB5,0x62,0x06,0x3E,0x2C,0x00,0x00,0x00,0x20,0x05,0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,0x01,0x01,0x03,0x00,0x00,
                                 0x00,0x01,0x01,0x03,0x08,0x10,0x00,0x01,0x00,0x01,0x01,0x05,0x00,0x03,0x00,0x00,0x00,0x01,0x01,0x06,0x08,0x0E,0x00,
                                 0x00,0x00,0x01,0x01,0xFD,0x25};

void Config_Gps(void)
{
   unsigned char checkSumA = 0, checkSumB = 0;
   unsigned char i = 0;
   
   UART1_Send_Buf(setubx,28);       // 配置gps接口波特率 115200；
	delay_ms(50);
   
	UART1_Init(UART1_Baudrate);     // 串口3通信，用于和gps通信
	delay_ms(50);
   
   for(i=2; i<14;i++)
   {
      checkSumA += enableMsg[i];
      checkSumB += checkSumA;
   }
   enableMsg[14] = checkSumA;
   enableMsg[15] = checkSumB;  
   UART1_Send_Buf(enableMsg,16);
   delay_ms(50);
 
   checkSumA = 0;
   checkSumB = 0;
   for(i=2; i<12;i++)
   {
      checkSumA += initc[i];
      checkSumB += checkSumA;
   }
   initc[12] = checkSumA;
   initc[13] = checkSumB;
   UART1_Send_Buf(initc,14);
   delay_ms(50);
	
   //UART1_Send_Buf(enable_beidou,52);
}

/**************************实现函数********************************************
*函数原型:		void GPS_Decode()
*功　　能:		将刚刚接收到的帧数据必要的信息提取出来。
*******************************************************************************/
unsigned char total_gps_buf[TOTAL_GPS_BUF_LEN];
unsigned short readIndex = 0, total_gps_len = 0, recieve_gps_len = 0;
unsigned char GPS_exsit = 0, gpsArrived = 0, position_not_changed_count = 0, gps_checksum_ok = 0;
int GPS_Altitude = 0, GPS_Course = 0 , Ci_Bias_Gps=0, alt_level = 0;
int GPS_Vel_N = 0, GPS_Vel_E = 0, GPS_Vel_D = 0, GPS_GroundSpeed = 0;  
unsigned char Gps_Month, Gps_Day, Gps_Hour, Gps_Min, Gps_Sec, fixType, GPS_STA_Num;
unsigned short Gps_Year;
unsigned int GPS_MiniSec, gpsArriveTime = 0;
float GPS_PDOP = 25.0f;
int Latitude_GPS=0,Longitude_GPS=0,Latitude_GPS_Last=0,Longitude_GPS_Last=0;
void GPS_Decode(void)
{
   unsigned int j;
	short temp;
	int temp_Int;
	char cheskSumA, cheskSumB;
	unsigned short rdTemp = 0;
   
   if(total_gps_len + recieve_gps_len <= TOTAL_GPS_BUF_LEN)
   {
      memcpy(&total_gps_buf[total_gps_len], USART1_DMA_Read_Buf, recieve_gps_len);
      total_gps_len += recieve_gps_len;
   }
   else
   {
      memcpy(total_gps_buf, USART1_DMA_Read_Buf, recieve_gps_len);
      total_gps_len = recieve_gps_len;
   }
 
   if(total_gps_len <= 50)
   {
      return;
   }
   
	//=========================== 以下是读取数据 ====================================    
	for(rdTemp = 0; rdTemp < (total_gps_len-5); )
	{
      if((total_gps_buf[rdTemp] == 0xB5) && (total_gps_buf[rdTemp+1] == 0x62) && (total_gps_buf[rdTemp+2] == 0x01) &&
		(total_gps_buf[rdTemp+3] == 0x07) && (total_gps_buf[rdTemp+4] == 0x5C) && (total_gps_buf[rdTemp+5] == 0x00))
		{
			if(rdTemp+6+92+2 > total_gps_len)
			{
				break;
			}
         
			cheskSumA = 0; 
			cheskSumB = 0;	
			for(j=rdTemp+2; j<rdTemp+2+96; j++)
			{
				cheskSumA += total_gps_buf[j];
		    	cheskSumB += cheskSumA;
			}
         
			if((cheskSumA == total_gps_buf[rdTemp+2+96]) && (cheskSumB == total_gps_buf[rdTemp+2+97]))
			{
            GPS_exsit = 1;
            gps_checksum_ok = 1;
            
            gpsArriveTime = micros();
            
            buf2int((int*)&GPS_MiniSec, (unsigned char*)(&total_gps_buf[rdTemp+6+0]));            
            buf2short((short*)&Gps_Year, (unsigned char*)(&total_gps_buf[rdTemp+6+4]));
            
            Gps_Month = total_gps_buf[rdTemp+6+6];
            Gps_Day = total_gps_buf[rdTemp+6+7];
            Gps_Hour = total_gps_buf[rdTemp+6+8];
            Gps_Min = total_gps_buf[rdTemp+6+9];
            Gps_Sec = total_gps_buf[rdTemp+6+10];
            
            fixType = total_gps_buf[rdTemp+6+20];            
				GPS_STA_Num	= total_gps_buf[rdTemp+6+23];
            
            // 经、纬、高
            buf2int(&Longitude_GPS, (unsigned char*)(&total_gps_buf[rdTemp+6+24]));    // 1e-7 deg
            buf2int(&Latitude_GPS, (unsigned char*)(&total_gps_buf[rdTemp+6+28]));     // 1e-7 deg
            buf2int(&GPS_Altitude, (unsigned char*)(&total_gps_buf[rdTemp+6+36]));     // mm
            
            // check gps data error
            if((Longitude_GPS == Longitude_GPS_Last) && (Latitude_GPS == Latitude_GPS_Last) && (Longitude_GPS != 0))
            {
               // record the data error counts
               if(position_not_changed_count == 19)
               {
                  gps_error_record.error_data_occurs_count++;
                  FLASH_WriteMoreData(GPS_ERROR_SAVE_ADDR, (uint16_t *)&gps_error_record, sizeof(gps_error_record));
               }
               
               position_not_changed_count++;
               if(position_not_changed_count >= 20)
               {
                  position_not_changed_count = 20;
                  gps_status |= GPS_A_ERROR_DATA_MASK;
               }
            }
            else
            {
               position_not_changed_count = 0;
               gps_status &= ~GPS_A_ERROR_DATA_MASK;
            }
            
            Longitude_GPS_Last = Longitude_GPS;
            Latitude_GPS_Last = Latitude_GPS;
            
            buf2int(&alt_level, (unsigned char*)(&total_gps_buf[rdTemp+6+36]));     // mm
            
            // 东北天速度
            buf2int(&GPS_Vel_N, (unsigned char*)(&total_gps_buf[rdTemp+6+48]));    // mm/s
            buf2int(&GPS_Vel_E, (unsigned char*)(&total_gps_buf[rdTemp+6+52]));    // mm/s
            buf2int(&GPS_Vel_D, (unsigned char*)(&total_gps_buf[rdTemp+6+56]));    // mm/s
        
            buf2int(&GPS_GroundSpeed, (unsigned char*)(&total_gps_buf[rdTemp+6+60]));    // mm/s
            buf2int(&temp_Int, (unsigned char*)(&total_gps_buf[rdTemp+6+64]));           // 1e-5
            GPS_Course = temp_Int / 10000;                                                     // 单位：0.1度
                           
            buf2short(&temp, (unsigned char*)(&total_gps_buf[rdTemp+6+76]));           // 0.01
            GPS_PDOP = (float)temp / 100.0f;
          
            rdTemp += (6+92+2);
			}
         else
         {
            rdTemp += 6;
         }
		}
      else
      {
         rdTemp++;
      }
	}

   memcpy(total_gps_buf, &total_gps_buf[rdTemp], total_gps_len - rdTemp);
   total_gps_len -= rdTemp;
}

/*********************************************************************************************
function: check gps status
para:
return:
**********************************************************************************************/
unsigned char gps_status = 0;
void Check_Gps_Status(void)
{
   // gps not exist
   if(GPS_exsit == 0)
   {
      gps_status |= GPS_A_ERROR_NOT_EXIST;
   }
   else
   {
      gps_status &= ~GPS_A_ERROR_NOT_EXIST;
   }
   
   // gps has not link now
   if((unsigned int)(micros() - gpsArriveTime) > 2000000 && (GPS_exsit == 1))
   {
      // record the error count and 
      if((gps_status & GPS_A_ERROR_LINK_MASK) != GPS_A_ERROR_LINK_MASK)
      {
         gps_error_record.error_link_occurs_count++;
         FLASH_WriteMoreData(GPS_ERROR_SAVE_ADDR, (uint16_t *)&gps_error_record, sizeof(gps_error_record));
      }
      
      gps_status |= GPS_A_ERROR_LINK_MASK;
   }
   else
   {
      gps_status &= ~GPS_A_ERROR_LINK_MASK;
   }
}


