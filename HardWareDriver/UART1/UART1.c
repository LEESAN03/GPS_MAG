//串口1接gps,使用DMA+串口空闲中断方式接收
#include "UART1.h"
#include "stm32f10x.h"
#include "GPS.h"
#include "string.h"

#define SRC_USART1_DR (&(USART1->DR)) //串口接收寄存器作为源头
unsigned char USART1_DMA_Read_Buf[UART1_READ_LENGHT];
void UART1_RX_DMA_Config(void)
{
   DMA_InitTypeDef DMA_InitStructure;                     //定义DMA结构体
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);   //使能DMA时钟

   // ============= 配置uart1的接受功能 ==============
   DMA_DeInit(DMA1_Channel5);
   DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART1_DR;//源头BUF
   DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART1_DMA_Read_Buf;//目标BUF
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;			//外设作源头

   DMA_InitStructure.DMA_BufferSize = UART1_READ_LENGHT;				    //BUF大小
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不递增
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		//内存地址递增

   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设字节为单位
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//内存字节为单位
   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//循环模式

   DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//4优先级之一的(高优先)
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

   /* 使能 UART1 模块的时钟  使能 UART1对应的引脚端口PA的时钟*/
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

   /* 配置UART1 的发送引脚
   配置PA9 为复用输出  刷新频率50MHz
   */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   /*
     配置UART1 的接收引脚
     配置PA10为浮地输入
   */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /*
     UART1的配置:
     1.波特率为调用程序指定的输入 baudrate;
     2. 8位数据			  USART_WordLength_8b;
     3.一个停止位			  USART_StopBits_1;
     4. 无奇偶效验			  USART_Parity_No ;
     5.不使用硬件流控制	  USART_HardwareFlowControl_None;
     6.使能发送和接收功能	  USART_Mode_Rx | USART_Mode_Tx;
   */
   USART_InitStructure.USART_BaudRate = baudrate;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   //应用配置到UART1
   USART_Init(USART1, &USART_InitStructure);   
   //使能串口1空闲中断
   USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);    
   //启动UART1
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
   //等待发送完成.
   while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
   {
   } //等待发送结束
}


//串口1中断处理函数
void USART1_IRQHandler()
{
   // 判断是否是空闲中断
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

