
#ifndef _CHIP_FLASH_H
#define _CHIP_FLASH_H

#include "stdint.h"

#define FLASH_SIZE 128 //所选MCU的FLASH容量大小(单位为K)

#if FLASH_SIZE < 256
#define SECTOR_SIZE 1024 //字节
#else
#define SECTOR_SIZE 2048 //字节
#endif

// the last 1kb used save the gps error
#define GPS_ERROR_SAVE_ADDR      (0x0800FC00 - 1024)
#define AVAILABLE_FLAG      (unsigned int)0xA77A

#pragma pack(push,1)
typedef struct
{
   unsigned int available_flag;
   unsigned int error_data_occurs_count;
   unsigned int error_link_occurs_count;
} GPS_ERROR_RECORD;
#pragma pack(pop)

extern GPS_ERROR_RECORD gps_error_record;

uint16_t FLASH_ReadHalfWord(uint32_t address);

void FLASH_ReadMoreData(uint32_t startAddress, uint16_t *readData, uint16_t countToRead);
void FLASH_WriteMoreData(uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite);

#endif
