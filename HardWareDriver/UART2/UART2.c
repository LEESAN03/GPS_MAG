#include "UART2.h"
#include "stm32f10x_dma.h"
#include "GPS.h"
#include "compute.h"
#include "QMC5883L.h"

#define SRC_USART2_DR (&(USART2->DR)) //����data val�Ĵ�����ַ
volatile char USART2_DMA_Send_Buf[UART2_SEND_LENGTH];

static void dma1_channel7_tx_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


static void uart2_gpio_config(unsigned int baudrate)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   USART_InitTypeDef USART_InitStructure;

   /* ʹ�� UART2 ģ���ʱ��  ʹ�� UART2��Ӧ�����Ŷ˿�PA��ʱ��*/
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
   /* ����UART2 �ķ�������
   ����PA9 Ϊ�������  ˢ��Ƶ��50MHz
   */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   /*
     ����UART2 �Ľ�������
     ����PA10Ϊ��������
   */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /*
     UART2������:
     1.������Ϊ���ó���ָ�������� baudrate;
     2. 8λ����			  USART_WordLength_8b;
     3.һ��ֹͣλ			  USART_StopBits_1;
     4. ����żЧ��			  USART_Parity_No ;
     5.��ʹ��Ӳ��������	  USART_HardwareFlowControl_None;
     6.ʹ�ܷ��ͺͽ��չ���	  USART_Mode_Rx | USART_Mode_Tx;
   */
   USART_InitStructure.USART_BaudRate = baudrate;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   //Ӧ�����õ�UART2
   USART_Init(USART2, &USART_InitStructure);
   //����UART2
   USART_Cmd(USART2, ENABLE);
   USART_ITConfig(USART2, USART_IT_TC, DISABLE); 
}

static void dma_uart2_config(void)
{
    // ���ô���2DMA����
    DMA_InitTypeDef DMA_InitStructure;                 //����DMA�ṹ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //ʹ��DMAʱ��

    DMA_DeInit(DMA1_Channel7);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART2_DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART2_DMA_Send_Buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;

    DMA_InitStructure.DMA_BufferSize = UART2_SEND_LENGTH;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ�Ĵ���������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //�ڴ��ַ����

    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�����ֽ�Ϊ��λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�ڴ��ֽ�Ϊ��λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //���δ��䷽ʽ

    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; // 4���ȼ�֮һ��(������)
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel7, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE); // DMA2��������ж�
    DMA_Cmd(DMA1_Channel7, DISABLE);

    // ���ô���2ʹ��DMA��ʽ����
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}


void UART2_Init(unsigned int baudrate)
{
    uart2_gpio_config(baudrate);
    dma_uart2_config();
    dma1_channel7_tx_nvic_config();
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART3_Put_Char(unsigned char DataToSend)
*��������:		ͨ�� UART3 ����һ���ֽ� ������
����
unsigned char DataToSend Ҫ���͵��ֽ�
*******************************************************************************/
void UART2_Put_Char(unsigned char DataToSend)
{
    USART_SendData(USART2, DataToSend);
    //�ȴ��������.
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
    {
    } //�ȴ����ͽ���
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART3_Send_Buf(unsigned char *buf,unsigned char len)
*��������:		ͨ�� UART3 ���ͼ����ֽ� ������
unsigned char *buf  ������ݵ�����ָ��
unsigned char len   Ҫ���͵��ֽ���
*******************************************************************************/
void UART2_Send_Buf(unsigned char *buf, unsigned char len)
{
   unsigned char i;
   for (i = 0; i < len; i++, buf++)
   {
       UART2_Put_Char(*buf);
   }
}


void DMA1_Channel7_IRQHandler(void)
{
   if(DMA_GetITStatus(DMA1_IT_TC7) != RESET)
   {
      DMA_ClearITPendingBit(DMA1_IT_TC7);
      DMA_Cmd(DMA1_Channel7, DISABLE);
   }
   DMA_ClearITPendingBit(DMA1_IT_TE7);
}

void open_dma1_channel7_tx(unsigned short count)
{
    DMA_Cmd(DMA1_Channel7, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel7, UART2_SEND_LENGTH);
    DMA_Cmd(DMA1_Channel7, ENABLE);
}

void UART2_Report_GPS(void)
{
    unsigned char checkSum = 0, i;
    short temp = 0;

    USART2_DMA_Send_Buf[0] = '$';
    USART2_DMA_Send_Buf[1] = 'G';
    USART2_DMA_Send_Buf[2] = 'P';
    USART2_DMA_Send_Buf[3] = 'S';

    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[4]), &Longitude_GPS); // ����
    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[8]), &Latitude_GPS);  // γ��

    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[12]), &GPS_Vel_E); // �����ٶ�
    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[16]), &GPS_Vel_N); // �ϱ��ٶ�
    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[20]), &GPS_Vel_D); // �����ٶ�

    temp = (short)GPS_Course;                                         // ��λ�� 0.1��  ��Χ��0--360, ��ƫ��Ϊ��
    short2buf((unsigned char *)(&USART2_DMA_Send_Buf[24]), &temp); 

    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[26]), &GPS_Altitude); // GPS�߶�   ��λ��mm
    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[30]), &GPS_GroundSpeed); // GPS�ٶ�	 ��λmm/s

    USART2_DMA_Send_Buf[34] = GPS_STA_Num; // GPS����

    if (GPS_PDOP >= 2.5f)
        USART2_DMA_Send_Buf[35] = 25; // PDOP����
    else
        USART2_DMA_Send_Buf[35] = (unsigned char)(GPS_PDOP * 10);

    for (i = 0; i < (UART2_SEND_LENGTH - 1); i++)
        checkSum += USART2_DMA_Send_Buf[i];
    USART2_DMA_Send_Buf[UART2_SEND_LENGTH - 1] = checkSum;
    
    open_dma1_channel7_tx(UART2_SEND_LENGTH);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_Report_GPS(void)
*��������:		���GPS��Ϣ
*******************************************************************************/
void UART2_Report_MAG(void)
{
    unsigned char checkSum = 0, i;

    USART2_DMA_Send_Buf[0] = '$';
    USART2_DMA_Send_Buf[1] = 'M';
    USART2_DMA_Send_Buf[2] = 'A';
    USART2_DMA_Send_Buf[3] = 'G';

    float2buf((unsigned char *)(&USART2_DMA_Send_Buf[4]), &x_mag);
    float2buf((unsigned char *)(&USART2_DMA_Send_Buf[8]), &y_mag);
    float2buf((unsigned char *)(&USART2_DMA_Send_Buf[12]), &z_mag);

    short2buf((unsigned char *)(&USART2_DMA_Send_Buf[16]), (short *)&Gps_Year);       //ʱ�䣺��
    USART2_DMA_Send_Buf[18] = Gps_Month; // ʱ�䣺��
    USART2_DMA_Send_Buf[19] = Gps_Day;   // ʱ�䣺��

    USART2_DMA_Send_Buf[20] = Gps_Hour; // ʱ�䣺ʱ
    USART2_DMA_Send_Buf[21] = Gps_Min;  // ʱ�䣺��
    USART2_DMA_Send_Buf[22] = Gps_Sec;  // ʱ�䣺��

    USART2_DMA_Send_Buf[23] = 0;
    USART2_DMA_Send_Buf[24] = 0;
    USART2_DMA_Send_Buf[25] = 0;

    for (i = 0; i < (UART2_SEND_LENGTH - 1); i++)
        checkSum += USART2_DMA_Send_Buf[i];
    USART2_DMA_Send_Buf[UART2_SEND_LENGTH - 1] = checkSum;
    
    open_dma1_channel7_tx(UART2_SEND_LENGTH);
}

