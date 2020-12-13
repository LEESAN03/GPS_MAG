#ifndef __CAN_TO_IMU_H
#define __CAN_TO_IMU_H

#define   GPS_TYPE_A

#define   IMU_CAN_ADDR               0x01  

#ifdef GPS_TYPE_A
   #define   GPS_CAN_ADDR            0x02    
#elif defined GPS_TYPE_B
   #define   GPS_CAN_ADDR            0x04  
#endif

#define   GPS_CAN_ADDR_A_MASK        0x00000040
#define   GPS_CAN_ADDR_B_MASK        0x00000080
#define   CAN_DES_FILTER_MASK        0x003F8006  //只接受目标地址匹配的消息
#define   TRANSFER_TYPR_DATA         0x00
#define   TRANSFER_TYPR_REQ          0x01

#define   MSG_ID_GPS                 0x01 
#define   MSG_ID_MAG                 0x02
#define   MSG_ID_GPS_MASK            0x00080000
#define   MSG_ID_MAG_MASK            0x00100000

#define   SOT_FLAG                   0x00000010
#define   EOT_FLAG                   0x00000008

#define   SEND_GPS_LEN               48
#define   SEND_MAG_LEN               16

#define   SEND_GPS_MSG               0x01
#define   SEND_MAG_MSG               0x02

void CAN1_Mode_Init(void);
void USER_CAN_TRANSMIT(unsigned char *data, unsigned char msg_type, unsigned sequence);
void Send_Gps(void);
void can_transmit_gps(void);
void Send_Mag(void);
void can_transmit_mag(void);
#endif
