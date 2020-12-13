//����1��gps,ʹ��DMA+���ڿ����жϷ�ʽ����
#include "UART1.h"
#include "stm32f10x.h"
#include "GPS.h"
#include "string.h"

#define SRC_USART1_DR (&(USART1->DR)) //���ڽ��ռĴ�����ΪԴͷ
unsigned char USART1_DMA_Read_Buf[UART1_READ_LENGHT];
void UART1_RX_DMA_Config(void)
{
   DMA_InitTypeDef DMA_InitStructure;                     //����DMA�ṹ��
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);   //ʹ��DMAʱ��

   // ============= ����uart1�Ľ��ܹ��� ==============
   DMA_DeInit(DMA1_Channel5);
   DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART1_DR;//ԴͷBUF
   DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART1_DMA_Read_Buf;//Ŀ��BUF
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;			//������Դͷ

   DMA_InitStructure.DMA_BufferSize = UART1_READ_LENGHT;				    //BUF��С
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ�Ĵ���������
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		//�ڴ��ַ����

   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�����ֽ�Ϊ��λ
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�ڴ��ֽ�Ϊ��λ
   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//ѭ��ģʽ

   DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//4���ȼ�֮һ��(������)
   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

   DMA_Init(DMA1_Channel5, &DMA_InitStructure);
   DMA_Cmd(DMA1_Channel5, ENABLE);

   USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
}

static void UART1_NVIC_Configuration(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}


void UART1_Init(unsigned int baudrate)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   USART_InitTypeDef USART_InitStructure;

   /* ʹ�� UART1 ģ���ʱ��  ʹ�� UART1��Ӧ�����Ŷ˿�PA��ʱ��*/
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

   /* ����UART1 �ķ�������
   ����PA9 Ϊ�������  ˢ��Ƶ��50MHz
   */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   /*
     ����UART1 �Ľ�������
     ����PA10Ϊ��������
   */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /*
     UART1������:
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
   //Ӧ�����õ�UART1
   USART_Init(USART1, &USART_InitStructure);   
   //ʹ�ܴ���1�����ж�
   USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);    
   //����UART1
   USART_Cmd(USART1, ENABLE);
   
   UART1_NVIC_Configuration();
}

void UART1_Send_Buf(unsigned char *buf, unsigned char len)
{
   unsigned char i;
   for (i = 0; i < len; i++, buf++)
   {
      UART1_Put_Char(*buf);
   }
}

void UART1_Put_Char(unsigned char DataToSend)
{
   USART_SendData(USART1, DataToSend);
   //�ȴ��������.
   while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
   {
   } //�ȴ����ͽ���
}


//����1�жϴ�����
void USART1_IRQHandler()
{
   // �ж��Ƿ��ǿ����ж�
   if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
   {
      gpsArrived = 1;      
      recieve_gps_len = UART1_READ_LENGHT - DMA_GetCurrDataCounter(DMA1_Channel5);
      
      DMA_Cmd(DMA1_Channel5, DISABLE);
      DMA_SetCurrDataCounter(DMA1_Channel5, UART1_READ_LENGHT);
      DMA_Cmd(DMA1_Channel5, ENABLE);
   }
   USART1->SR;
   USART1->DR;
}

