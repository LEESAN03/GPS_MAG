#include "stm32f10x.h"
#include "UART1.h"
#include "uart2.h"
#include "IOI2C.h"
#include "delay.h"
#include "QMC5883L.h"
#include "TIMER_CONFIG.h"
#include "GPS.h"
#include "compute.h"
#include "can_to_imu.h"
#include "chipFlash.h"

int main(void)
{
   system_init();          /* ����ϵͳʱ��Ϊ48M, ʹ���ڲ�HSIʱ��Դ*/
   delay_init(36);         //��ʱ��ʼ��
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); // �����ж����ȼ�
   
   Initial_Timer3();   
   delay_ms(500);      //�ȴ������ϵ�
 
   IIC_Init();        //��ʼ��I2C�ӿ�
   delay_ms(500);      //�ȴ������ϵ�

   // ��ʼ��������
   QMC5883L_SetUp();
   delay_ms(500);

   // ==================== ����GPS ===================
   UART1_Init(9600L); //��gpsͨ�Ŵ��ڣ���ʼ����9600
   Config_Gps();
   UART1_RX_DMA_Config();
   
   // imuͨ�Ŵ�������
   //UART2_Init(115200L); //��imuͨ�Ŵ���
   delay_ms(100);
   // ��ʼ��CAN���ߣ����ں�IMUͨ��
   CAN1_Mode_Init();
   
   FLASH_ReadMoreData(GPS_ERROR_SAVE_ADDR, (uint16_t *)&gps_error_record, sizeof(gps_error_record));
   
   //check if the data is real
   if(gps_error_record.available_flag != AVAILABLE_FLAG)
   {
      gps_error_record.available_flag = AVAILABLE_FLAG;
      gps_error_record.error_data_occurs_count = 0;
      gps_error_record.error_link_occurs_count = 0;
      
      FLASH_WriteMoreData(GPS_ERROR_SAVE_ADDR, (uint16_t *)&gps_error_record, sizeof(gps_error_record));
   }
   
   while (1)
   {
      if(gpsArrived == 1)
      {
         gpsArrived = 0;
         GPS_Decode();
         
         if(gps_checksum_ok == 1)
         {
            gps_checksum_ok = 0;
            Send_Gps();
         }
      }
      else if(HZ_50_FLAG == 1)
      {
         HZ_50_FLAG = 0;  
         Read_5883();
         Send_Mag();
         
         Check_Gps_Status();
      }
   }
}
