#include "stm32f10x.h"
#include "can_to_imu.h"
#include "stm32f10x_can.h"
#include "TIMER_CONFIG.h"
#include "GPS.h"
#include "compute.h"
#include "QMC5883L.h"
#include "delay.h"
#include "chipFlash.h"

// CAN��ʼ��
//ÿһλ��Tq��Ŀ = 1 (�̶�SYNC_SEG) +  3 (BS1) + 2 (BS2) = 6
//������ = APB2_CLOCK/((CAN_SJW+ CAN_BS1+ CAN_BS2+ 3)* CAN_Prescaler)
//������Ϊ:36M/((1+3+2)*12)=500Kbps
// APB2_CLOCK��ʱ���ڳ�ʼ����ʱ������Ϊ72M,������÷�Ƶ����2��APB1_CLOCKΪ36M
//������Ϊ:36M/((1+3+2)*12)=500Kbps
const uint32_t Gps_To_Imu_HEADER = ((uint32_t)TRANSFER_TYPR_DATA << 26) | ((uint32_t)IMU_CAN_ADDR << 12) | ((uint32_t)GPS_CAN_ADDR << 5); 
const uint32_t Imu_To_Gps_HEADER = ((uint32_t)TRANSFER_TYPR_DATA << 26) | ((uint32_t)GPS_CAN_ADDR << 12) | ((uint32_t)IMU_CAN_ADDR << 5);
void CAN1_Mode_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* ���ù��ܺ�GPIOA�˿�ʱ��ʹ��*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

    /* CAN1 ģ��ʱ��ʹ�� */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    // CANʱ�Ӹ�λĬ��ֵ
    CAN_DeInit(CAN1);

    /* Configure CAN pin: RX */ // PA11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // ��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure CAN pin: TX */ // PA12
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // �����������
                                                    //���������������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //#define GPIO_Remap_CAN GPIO_Remap1_CAN1 ��ʵ��û���õ���ӳ��I/O
     GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

    // CAN��Ԫ���ø�λĬ��ֵ
    CAN_StructInit(&CAN_InitStructure);
    // CAN��Ԫ����
    CAN_InitStructure.CAN_TTCM = DISABLE; //��ʱ�䴥��ͨ��ģʽ
    CAN_InitStructure.CAN_ABOM = ENABLE;  //����Զ����߹���
    CAN_InitStructure.CAN_AWUM = DISABLE; //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
    CAN_InitStructure.CAN_NART = DISABLE; //�����Զ��ش�
    CAN_InitStructure.CAN_RFLM = DISABLE; //���Ĳ�����,���������ʱ�µĸ��Ǿɵ�
    CAN_InitStructure.CAN_TXFP = DISABLE; //���ȼ��ɱ��ı�ʶ������
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; //ģʽ����-����ģʽ
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq; //����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ
                     // CAN_SJW_1tq~CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq; // Tbs1��ΧCAN_BS1_1tq
                                             // ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq; // Tbs2��ΧCAN_BS2_1tq ~
                                             // CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler = 12;    //��Ƶϵ��(Fdiv)�������� = 36/(6*12) = 1M
    CAN_Init(CAN1, &CAN_InitStructure);      //��ʼ��CAN1

    /*
     STM32F4�Ĺ����������28����ÿ����������2��32λ�ļĴ���CAN_FxR1��CAN_FxR2���
     ÿ����������λ����Զ�������Ϊ16λ��32λ��������ģʽ������Ϊ����λģʽ�ͱ�ʶ���б�ģʽ
     ģʽ��λ����Ͽ�����4�й��˷�ʽ
     1ID-32λ����λģʽ       2ID-32λ��ʶ���б�ģʽ      2ID-16λ����λģʽ
     4ID-16λ��ʶ���б�ģʽ
     CAN_FxR1 : ID[31-0]      CAN_FxR1 : ID[31-0]         CAN_FxR1 :
     MASK1[31-16] ID1[15-0]        CAN_FxR1 : ID2[31-16] ID1[15-0]
     CAN_FxR2 : MASK[31-0]    CAN_FxR2 : ID2[31-0]        CAN_FxR2 :
     MASK2[31-16] ID2[15-0]        CAN_FxR2 : ID4[31-16] ID3[15-0]

     ID֡�ĸ�ʽ  ��׼֡11+��+IDE+RTR+��
     IDE(��׼֡/��չ֡)-----RTR(����֡/Զ��֡)
                 ��չ֡29+IDE+RTR+��
    */
    //���ù�����
    CAN_FilterInitStructure.CAN_FilterNumber = 0; //ָ����28��������(0-27)�д���ʼ���Ĺ�����0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //ָ���˹�����������ʼ������ģʽΪ��ʶ������λģʽ
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //�����˹�����λ��32λ������
    CAN_FilterInitStructure.CAN_FilterIdHigh = (uint16_t)(Imu_To_Gps_HEADER >> 13);  //�����趨��������ʶ����32λλ��ʱΪ��߶�λ��16λλ��ʱΪ��һ��)
    CAN_FilterInitStructure.CAN_FilterIdLow = (uint16_t)(((Imu_To_Gps_HEADER << 3) & 0x0000FFFF) | CAN_ID_EXT);
    
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (uint16_t)(CAN_DES_FILTER_MASK >> 16); // 32λMASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = (uint16_t)(CAN_DES_FILTER_MASK & 0x0000FFFF);
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; //������0������FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //���������0
    CAN_FilterInit(&CAN_FilterInitStructure);              //�˲�����ʼ��

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); // FIFO0��Ϣ�Һ��ж�����.

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn; // FIFO 0���жϸ�FIFO 1���жϺ�������һ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // �����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // �����ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/***============================================================================
*
*can���ݷ���
*
================================================================================*/
void USER_CAN_TRANSMIT(unsigned char *data, unsigned char msg_type, unsigned sequence)
{
   unsigned short can_tcout = 0, i = 0;
   uint8_t TransmitMailbox = 0;
   CanTxMsg TxMessage;
   unsigned int canExId = 0;
   
   if(msg_type == SEND_GPS_MSG)      // ��Ϣ����ΪGPS
   {
      canExId = Gps_To_Imu_HEADER | ((uint32_t)MSG_ID_GPS << 19);
      if(sequence == 0)
      {
         TxMessage.ExtId = canExId | SOT_FLAG | (uint32_t)sequence; // �趨��׼��ʶ��(11λ����չ��Ϊ29λ)
      }
      else if(sequence == 5)
      {
         TxMessage.ExtId = canExId | EOT_FLAG | (uint32_t)sequence; // �趨��׼��ʶ��(11λ����չ��Ϊ29λ)
      }
      else
      {
         TxMessage.ExtId = canExId | (uint32_t)sequence;
      }
   }
   else if(msg_type == MSG_ID_MAG)   // ��Ϣ����ΪMAG
   {
      canExId = Gps_To_Imu_HEADER | ((uint32_t)MSG_ID_MAG << 19);
      if(sequence == 0)
      {
         TxMessage.ExtId = canExId | SOT_FLAG | (uint32_t)sequence; // �趨��׼��ʶ��(11λ����չ��Ϊ29λ)
      }
      else if(sequence == 1)
      {
         TxMessage.ExtId = canExId | EOT_FLAG | (uint32_t)sequence; // �趨��׼��ʶ��(11λ����չ��Ϊ29λ)
      }
      else
      {
         TxMessage.ExtId = canExId | (uint32_t)sequence;
      }
   }
          
   TxMessage.RTR = CAN_RTR_DATA;       // ������Ϣ��֡����Ϊ����֡(����Զ��֡)
   TxMessage.IDE = CAN_ID_EXT;        // ��Ϣ��־��ʵ���׼��ʶ��
   TxMessage.DLC = 8;               // ����8���ֽڣ�һ�ֽ�8λ
   
   for(i=0; i<8; i++)
     TxMessage.Data[i] = *(data+i);

   TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);

    // ���ڼ����Ϣ�����Ƿ�����
   while((TransmitMailbox == CAN_TxStatus_NoMailBox) && (can_tcout <= 800))
   {
      can_tcout++;
      TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
   }

   can_tcout = 0;
   // ��鷵�صĹҺŵ���Ϣ��Ŀ
   while((CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK) && (can_tcout <= 800))
   {
      can_tcout++;
   }
}

// �����жϷ�����
CanRxMsg CAN1_RxMessage;

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CAN_Receive(CAN1, CAN_FIFO0, &CAN1_RxMessage);

    //�ͷ�FIFO0
    CAN_FIFORelease(CAN1, CAN_FIFO0);
    /* �����жϱ�־λ  */
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
}

unsigned char sendGpsBuf[SEND_GPS_LEN];
void Send_Gps(void)
{
   unsigned short checkSum = 0;
   short temp = 0;
   
   int2buf((unsigned char *)(&sendGpsBuf[0]), (int*)&Longitude_GPS); // ����
   int2buf((unsigned char *)(&sendGpsBuf[4]), (int*)&Latitude_GPS);  // γ��

   int2buf((unsigned char *)(&sendGpsBuf[8]), &GPS_Vel_E); // �����ٶ�
   int2buf((unsigned char *)(&sendGpsBuf[12]), &GPS_Vel_N); // �ϱ��ٶ�
   int2buf((unsigned char *)(&sendGpsBuf[16]), &GPS_Vel_D); // �����ٶ�

   temp = (short)GPS_Course;                                         // ��λ�� 0.1��  ��Χ��0--360, ��ƫ��Ϊ��
   short2buf((unsigned char *)(&sendGpsBuf[20]), &temp); 

   int2buf((unsigned char *)(&sendGpsBuf[22]), (int*)&GPS_Altitude); // GPS�߶�   ��λ��mm
   int2buf((unsigned char *)(&sendGpsBuf[26]), &GPS_GroundSpeed); // GPS�ٶ�	 ��λmm/s
   sendGpsBuf[30] = GPS_STA_Num; // GPS����

   if(GPS_PDOP >= 2.5f)
      sendGpsBuf[31] = 25; // PDOP����
   else
      sendGpsBuf[31] = (unsigned char)(GPS_PDOP * 10);
   
   short2buf((unsigned char *)(&sendGpsBuf[32]), (short *)&Gps_Year);       //ʱ�䣺��
   sendGpsBuf[34] = Gps_Month; // ʱ�䣺��
   sendGpsBuf[35] = Gps_Day;   // ʱ�䣺��

   sendGpsBuf[36] = Gps_Hour; // ʱ�䣺ʱ
   sendGpsBuf[37] = Gps_Min;  // ʱ�䣺��
   sendGpsBuf[38] = Gps_Sec;  // ʱ�䣺��

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

