#include "UART2.h"
#include "stm32f10x_dma.h"
#include "GPS.h"
#include "compute.h"
#include "QMC5883L.h"

#define SRC_USART2_DR (&(USART2->DR)) //串口data val寄存器地址
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

   /* 使能 UART2 模块的时钟  使能 UART2对应的引脚端口PA的时钟*/
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
   /* 配置UART2 的发送引脚
   配置PA9 为复用输出  刷新频率50MHz
   */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   /*
     配置UART2 的接收引脚
     配置PA10为浮地输入
   */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /*
     UART2的配置:
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
   //应用配置到UART2
   USART_Init(USART2, &USART_InitStructure);
   //启动UART2
   USART_Cmd(USART2, ENABLE);
   USART_ITConfig(USART2, USART_IT_TC, DISABLE); 
}

static void dma_uart2_config(void)
{
    // 配置串口2DMA发送
    DMA_InitTypeDef DMA_InitStructure;                 //定义DMA结构体
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能DMA时钟

    DMA_DeInit(DMA1_Channel7);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART2_DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART2_DMA_Send_Buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;

    DMA_InitStructure.DMA_BufferSize = UART2_SEND_LENGTH;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址寄存器不递增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //内存地址递增

    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设字节为单位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //内存字节为单位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //单次传输方式

    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; // 4优先级之一的(高优先)
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel7, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE); // DMA2传输完成中断
    DMA_Cmd(DMA1_Channel7, DISABLE);

    // 配置串口2使用DMA方式发送
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}


void UART2_Init(unsigned int baudrate)
{
    uart2_gpio_config(baudrate);
    dma_uart2_config();
    dma1_channel7_tx_nvic_config();
}
/**************************实现函数********************************************
*函数原型:		void UART3_Put_Char(unsigned char DataToSend)
*功　　能:		通过 UART3 发送一个字节 的数据
输入
unsigned char DataToSend 要发送的字节
*******************************************************************************/
void UART2_Put_Char(unsigned char DataToSend)
{
    USART_SendData(USART2, DataToSend);
    //等待发送完成.
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
    {
    } //等待发送结束
}

/**************************实现函数********************************************
*函数原型:		void UART3_Send_Buf(unsigned char *buf,unsigned char len)
*功　　能:		通过 UART3 发送几个字节 的数据
unsigned char *buf  存放数据的数组指针
unsigned char len   要发送的字节数
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

    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[4]), &Longitude_GPS); // 经度
    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[8]), &Latitude_GPS);  // 纬度

    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[12]), &GPS_Vel_E); // 东西速度
    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[16]), &GPS_Vel_N); // 南北速度
    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[20]), &GPS_Vel_D); // 天向速度

    temp = (short)GPS_Course;                                         // 单位： 0.1度  范围：0--360, 北偏东为正
    short2buf((unsigned char *)(&USART2_DMA_Send_Buf[24]), &temp); 

    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[26]), &GPS_Altitude); // GPS高度   单位：mm
    int2buf((unsigned char *)(&USART2_DMA_Send_Buf[30]), &GPS_GroundSpeed); // GPS速度	 单位mm/s

    USART2_DMA_Send_Buf[34] = GPS_STA_Num; // GPS星数

    if (GPS_PDOP >= 2.5f)
        USART2_DMA_Send_Buf[35] = 25; // PDOP精度
    else
        USART2_DMA_Send_Buf[35] = (unsigned char)(GPS_PDOP * 10);

    for (i = 0; i < (UART2_SEND_LENGTH - 1); i++)
        checkSum += USART2_DMA_Send_Buf[i];
    USART2_DMA_Send_Buf[UART2_SEND_LENGTH - 1] = checkSum;
    
    open_dma1_channel7_tx(UART2_SEND_LENGTH);
}

/**************************实现函数********************************************
*函数原型:		void UART1_Report_GPS(void)
*功　　能:		输出GPS信息
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

    short2buf((unsigned char *)(&USART2_DMA_Send_Buf[16]), (short *)&Gps_Year);       //时间：年
    USART2_DMA_Send_Buf[18] = Gps_Month; // 时间：月
    USART2_DMA_Send_Buf[19] = Gps_Day;   // 时间：日

    USART2_DMA_Send_Buf[20] = Gps_Hour; // 时间：时
    USART2_DMA_Send_Buf[21] = Gps_Min;  // 时间：分
    USART2_DMA_Send_Buf[22] = Gps_Sec;  // 时间：秒

    USART2_DMA_Send_Buf[23] = 0;
    USART2_DMA_Send_Buf[24] = 0;
    USART2_DMA_Send_Buf[25] = 0;

    for (i = 0; i < (UART2_SEND_LENGTH - 1); i++)
        checkSum += USART2_DMA_Send_Buf[i];
    USART2_DMA_Send_Buf[UART2_SEND_LENGTH - 1] = checkSum;
    
    open_dma1_channel7_tx(UART2_SEND_LENGTH);
}

