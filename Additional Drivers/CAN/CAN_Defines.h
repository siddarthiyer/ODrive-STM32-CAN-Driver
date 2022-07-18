
#ifndef CAN_DEFINES_H_
#define CAN_DEFINES_H_

#include "main.h"

#define CAN_BAUDRATE_1000_KBPS	0x001a0002


#define CAN_BAUDRATE_500_KBPS   0x001a0005
#define CAN_BAUDRATE_250_KBPS   0x001a000b
#define CAN_BAUDRATE_125_KBPS   0x001c0014
#define CAN_BAUDRATE_100_KBPS   0x001b001b
#define CAN_BAUDRATE_50_KBPS    0x001b0037
#define CAN_BAUDRATE_10_KBPS    0x001b0117

#define CAN_Frame_Data 0
#define CAN_Frame_Remote 10
#define CAN_ID_Standard 0
#define CAN_ID_Extended 10

#define CAN_FILTER_MASK_MODE 0
#define CAN_FILTER_LIST_MODE 10

#define CAN_Timestamp_Enable 1
#define CAN_Timestamp_Disable 0


#define CAN_Initialization_Mode	    0x10
#define CAN_Sleep_Mode			    0xEF
#define CAN_Normal_Mode			    0x45

#define CAN_TX_Mailbox_0 0x00
#define CAN_TX_Mailbox_1 0xAA
#define CAN_TX_Mailbox_2 0xFF

#define CAN_Timestamp_Enable 1
#define CAN_Timestamp_Disable 0

#define CAN_Filter_Scale_16bit 0
#define CAN_Filter_Scale_32bit 1

#define CAN_Filter_Bank_FIFO0	0
#define CAN_Filter_Bank_FIFO1	1

#define CAN_TX_Buffer_0 1
#define CAN_TX_Buffer_1 2
#define CAN_TX_Buffer_2 3

#define CAN_RX_Buffer_1 4
#define CAN_RX_Buffer_2 5


#define CAN_TX_All_Buffers 10
#define CAN_RX_ALL_Buffers 20




#endif /* CAN_DEFINES_H_ */
