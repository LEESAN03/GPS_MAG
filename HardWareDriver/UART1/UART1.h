#ifndef __UART1_H
#define __UART1_H

#define UART1_Baudrate       115200L
#define UART1_READ_LENGHT    120

extern unsigned char USART1_DMA_Read_Buf[UART1_READ_LENGHT];

void UART1_Init(unsigned int baudrate);
void UART1_Send_Buf(unsigned char *buf, unsigned char len);
void UART1_Put_Char(unsigned char DataToSend);
void UART1_RX_DMA_Config(void);
#endif


