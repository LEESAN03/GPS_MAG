#ifndef __UART2_H
#define __UART2_H

#define UART2_Baudrate        115200L
#define UART2_SEND_LENGTH     37

extern volatile char USART2_DMA_Send_Buf[UART2_SEND_LENGTH];
void UART2_Init(unsigned int baudrate);
void UART2_Put_Char(unsigned char DataToSend);
void UART2_Send_Buf(unsigned char *buf,unsigned char len);
void UART2_Report_GPS(void);
void UART2_Report_MAG(void);
#endif

