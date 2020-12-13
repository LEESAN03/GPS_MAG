#include "stm32f10x.h"
#include "can_to_imu.h"
#include "stm32f10x_can.h"
#include "TIMER_CONFIG.h"
#include "GPS.h"
#include "compute.h"
#include "QMC5883L.h"
#include "delay.h"
#include "chipFlash.h"

// CAN初始化
//每一位的Tq数目 = 1 (固定SYNC_SEG) +  3 (BS1) + 2 (BS2) = 6
//波特率 = APB2_CLOCK/((CAN_SJW+ CAN_BS1+ CAN_BS2+ 3)* CAN_Prescaler)
//则波特率为:36M/((1+3+2)*12)=500Kbps
// APB2_CLOCK的时钟在初始化的时候设置为72M,如果设置分频大于2则APB1_CLOCK为36M
//则波特率为:36M/((1+3+2)*12)=500Kbps
const uint32_t Gps_To_Imu_HEADER = ((uint32_t)TRANSFER_TYPR_DATA << 26) | ((uint32_t)IMU_CAN_ADDR << 12) | ((uint32_t)GPS_CAN_ADDR << 5); 
const uint32_t Imu_To_Gps_HEADER = ((uint32_t)TRANSFER_TYPR_DATA << 26) | ((uint32_t)GPS_CAN_ADDR << 12) | ((uint32_t)IMU_CAN_ADDR << 5);
void CAN1_Mode_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 复用功能和GPIOA端口时钟使能*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

    /* CAN1 模块时钟使能 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    // CAN时钟复位默认值
    CAN_DeInit(CAN1);

    /* Configure CAN pin: RX */ // PA11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure CAN pin: TX */ // PA12
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
                                                    //输出必须设置速率
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //#define GPIO_Remap_CAN GPIO_Remap1_CAN1 本实验没有用到重映射I/O
     GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

    // CAN单元设置复位默认值
    CAN_StructInit(&CAN_InitStructure);
    // CAN单元设置
    CAN_InitStructure.CAN_TTCM = DISABLE; //非时间触发通信模式
    CAN_InitStructure.CAN_ABOM = ENABLE;  //软件自动离线管理
    CAN_InitStructure.CAN_AWUM = DISABLE; //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_InitStructure.CAN_NART = DISABLE; //开启自动重传
    CAN_InitStructure.CAN_RFLM = DISABLE; //报文不锁定,当出现溢出时新的覆盖旧的
    CAN_InitStructure.CAN_TXFP = DISABLE; //优先级由报文标识符决定
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; //模式设置-正常模式
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq; //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位
                     // CAN_SJW_1tq~CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq; // Tbs1范围CAN_BS1_1tq
                                             // ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq; // Tbs2范围CAN_BS2_1tq ~
                                             // CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler = 12;    //分频系数(Fdiv)，波特率 = 36/(6*12) = 1M
    CAN_Init(CAN1, &CAN_InitStructure);      //初始化CAN1

    /*
     STM32F4的过滤器最多有28个，每个过滤器由2个32位的寄存器CAN_FxR1和CAN_FxR2组成
     每个过滤器的位宽可以独立设置为16位或32位，过滤器模式可设置为屏蔽位模式和标识符列表模式
     模式与位宽组合可以有4中过滤方式
     1ID-32位屏蔽位模式       2ID-32位标识符列表模式      2ID-16位屏蔽位模式
     4ID-16位标识符列表模式
     CAN_FxR1 : ID[31-0]      CAN_FxR1 : ID[31-0]         CAN_FxR1 :
     MASK1[31-16] ID1[15-0]        CAN_FxR1 : ID2[31-16] ID1[15-0]
     CAN_FxR2 : MASK[31-0]    CAN_FxR2 : ID2[31-0]        CAN_FxR2 :
     MASK2[31-16] ID2[15-0]        CAN_FxR2 : ID4[31-16] ID3[15-0]

     ID帧的格式  标准帧11+空+IDE+RTR+空
     IDE(标准帧/扩展帧)-----RTR(数据帧/远程帧)
                 扩展帧29+IDE+RTR+空
    */
    //配置过滤器
    CAN_FilterInitStructure.CAN_FilterNumber = 0; //指定了28个过滤器(0-27)中待初始化的过滤器0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //指定了过滤器将被初始化到的模式为标识符屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //给出了过滤器位宽32位过滤器
    CAN_FilterInitStructure.CAN_FilterIdHigh = (uint16_t)(Imu_To_Gps_HEADER >> 13);  //用来设定过滤器标识符（32位位宽时为其高段位，16位位宽时为第一个)
    CAN_FilterInitStructure.CAN_FilterIdLow = (uint16_t)(((Imu_To_Gps_HEADER << 3) & 0x0000FFFF) | CAN_ID_EXT);
    
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (uint16_t)(CAN_DES_FILTER_MASK >> 16); // 32位MASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = (uint16_t)(CAN_DES_FILTER_MASK & 0x0000FFFF);
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; //过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //激活过滤器0
    CAN_FilterInit(&CAN_FilterInitStructure);              //滤波器初始化

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); // FIFO0消息挂号中断允许.

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn; // FIFO 0的中断跟FIFO 1的中断函数名不一样
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 主优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/***============================================================================
*
*can数据发送
*
================================================================================*/
void USER_CAN_TRANSMIT(unsigned char *data, unsigned char msg_type, unsigned sequence)
{
   unsigned short can_tcout = 0, i = 0;
   uint8_t TransmitMailbox = 0;
   CanTxMsg TxMessage;
   unsigned int canExId = 0;
   
   if(msg_type == SEND_GPS_MSG)      // 消息类型为GPS
   {
      canExId = Gps_To_Imu_HEADER | ((uint32_t)MSG_ID_GPS << 19);
      if(sequence == 0)
      {
         TxMessage.ExtId = canExId | SOT_FLAG | (uint32_t)sequence; // 设定标准标识符(11位，扩展的为29位)
      }
      else if(sequence == 5)
      {
         TxMessage.ExtId = canExId | EOT_FLAG | (uint32_t)sequence; // 设定标准标识符(11位，扩展的为29位)
      }
      else
      {
         TxMessage.ExtId = canExId | (uint32_t)sequence;
      }
   }
   else if(msg_type == MSG_ID_MAG)   // 消息类型为MAG
   {
      canExId = Gps_To_Imu_HEADER | ((uint32_t)MSG_ID_MAG << 19);
      if(sequence == 0)
      {
         TxMessage.ExtId = canExId | SOT_FLAG | (uint32_t)sequence; // 设定标准标识符(11位，扩展的为29位)
      }
      else if(sequence == 1)
      {
         TxMessage.ExtId = canExId | EOT_FLAG | (uint32_t)sequence; // 设定标准标识符(11位，扩展的为29位)
      }
      else
      {
         TxMessage.ExtId = canExId | (uint32_t)sequence;
      }
   }
          
   TxMessage.RTR = CAN_RTR_DATA;       // 传输消息的帧类型为数据帧(还有远程帧)
   TxMessage.IDE = CAN_ID_EXT;        // 消息标志符实验标准标识符
   TxMessage.DLC = 8;               // 发送8个字节，一字节8位
   
   for(i=0; i<8; i++)
     TxMessage.Data[i] = *(data+i);

   TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);

    // 用于检查消息传输是否正常
   while((TransmitMailbox == CAN_TxStatus_NoMailBox) && (can_tcout <= 800))
   {
      can_tcout++;
      TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
   }

   can_tcout = 0;
   // 检查返回的挂号的信息数目
   while((CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK) && (can_tcout <= 800))
   {
      can_tcout++;
   }
}

// 接收中断服务函数
CanRxMsg CAN1_RxMessage;

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CAN_Receive(CAN1, CAN_FIFO0, &CAN1_RxMessage);

    //释放FIFO0
    CAN_FIFORelease(CAN1, CAN_FIFO0);
    /* 清零中断标志位  */
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
}

unsigned char sendGpsBuf[SEND_GPS_LEN];
void Send_Gps(void)
{
   unsigned short checkSum = 0;
   short temp = 0;
   
   int2buf((unsigned char *)(&sendGpsBuf[0]), (int*)&Longitude_GPS); // 经度
   int2buf((unsigned char *)(&sendGpsBuf[4]), (int*)&Latitude_GPS);  // 纬度

   int2buf((unsigned char *)(&sendGpsBuf[8]), &GPS_Vel_E); // 东西速度
   int2buf((unsigned char *)(&sendGpsBuf[12]), &GPS_Vel_N); // 南北速度
   int2buf((unsigned char *)(&sendGpsBuf[16]), &GPS_Vel_D); // 天向速度

   temp = (short)GPS_Course;                                         // 单位： 0.1度  范围：0--360, 北偏东为正
   short2buf((unsigned char *)(&sendGpsBuf[20]), &temp); 

   int2buf((unsigned char *)(&sendGpsBuf[22]), (int*)&GPS_Altitude); // GPS高度   单位：mm
   int2buf((unsigned char *)(&sendGpsBuf[26]), &GPS_GroundSpeed); // GPS速度	 单位mm/s
   sendGpsBuf[30] = GPS_STA_Num; // GPS星数

   if(GPS_PDOP >= 2.5f)
      sendGpsBuf[31] = 25; // PDOP精度
   else
      sendGpsBuf[31] = (unsigned char)(GPS_PDOP * 10);
   
   short2buf((unsigned char *)(&sendGpsBuf[32]), (short *)&Gps_Year);       //时间：年
   sendGpsBuf[34] = Gps_Month; // 时间：月
   sendGpsBuf[35] = Gps_Day;   // 时间：日

   sendGpsBuf[36] = Gps_Hour; // 时间：时
   sendGpsBuf[37] = Gps_Min;  // 时间：分
   sendGpsBuf[38] = Gps_Sec;  // 时间：秒

   int2buf((unsigned char *)(&sendGpsBuf[39]), (int*)&GPS_MiniSec);
   
   sendGpsBuf[43] = gps_status;
   
   if(gps_error_record.error_data_occurs_count < 255)
      sendGpsBuf[44] = (unsigned char)gps_error_record.error_data_occurs_count;
   else
      sendGpsBuf[44] = 255;
   
   if(gps_error_record.error_link_occurs_count < 255)
      sendGpsBuf[45] = (unsigned char)gps_error_record.error_link_occurs_count;
   else
      sendGpsBuf[45] = 255;
   
   checkSum = Get_Crc16(sendGpsBuf, SEND_GPS_LEN-2);
   short2buf((unsigned char *)(&sendGpsBuf[SEND_GPS_LEN-2]), (short*)&checkSum);
   can_transmit_gps();
}


void can_transmit_gps(void)
{
   unsigned char packSequence = 0;
   unsigned short sendGpsIndex = 0; 
   while(sendGpsIndex < SEND_GPS_LEN)
   {
      packSequence = sendGpsIndex / 8;
      if(sendGpsIndex + 8 <= SEND_GPS_LEN)
      {
         USER_CAN_TRANSMIT(&sendGpsBuf[sendGpsIndex],SEND_GPS_MSG, packSequence);
         sendGpsIndex += 8;
      }
   }
}

unsigned char sendMagBuf[SEND_MAG_LEN];
void Send_Mag(void)
{
   unsigned short checkSum = 0;
   
   float2buf((unsigned char *)(&sendMagBuf[0]), &x_mag);
   float2buf((unsigned char *)(&sendMagBuf[4]), &y_mag);
   float2buf((unsigned char *)(&sendMagBuf[8]), &z_mag);
   
   sendMagBuf[12] = 3;
   sendMagBuf[13] = 0;
   
   checkSum = Get_Crc16(sendMagBuf, SEND_MAG_LEN-2);
   short2buf((unsigned char *)(&sendMagBuf[SEND_MAG_LEN-2]), (short*)&checkSum);
   can_transmit_mag();
}


void can_transmit_mag(void)
{
   unsigned char packSequence = 0;
   unsigned short sendMagIndex = 0; 
   while(sendMagIndex < SEND_MAG_LEN)
   {
      packSequence = sendMagIndex / 8;
      if(sendMagIndex + 8 <= SEND_MAG_LEN)
      {
         USER_CAN_TRANSMIT(&sendMagBuf[sendMagIndex], SEND_MAG_MSG, packSequence);
         sendMagIndex += 8;
      }
   }
}

