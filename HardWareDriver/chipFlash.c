
#include "chipFlash.h"
#include "stm32f10x.h"

GPS_ERROR_RECORD gps_error_record;

//��ָ����ַ��ʼ��ȡ�������
void FLASH_ReadMoreData(uint32_t startAddress, uint16_t *readData, uint16_t countToRead)
{
    //���ݴ�С������˫�ֽڶ��롣
    if (countToRead % 2 != 0)
        return;

    uint16_t dataIndex = 0;
    for (dataIndex = 0; dataIndex < (countToRead / 2); dataIndex++)
    {
        readData[dataIndex] = FLASH_ReadHalfWord(startAddress + dataIndex * 2);
    }
}

//��ȡָ����ַ�İ���(16λ����)
uint16_t FLASH_ReadHalfWord(uint32_t address)
{
    return *(__IO uint16_t *)address;
}

//��ȡָ����ַ��ȫ��(32λ����)
uint32_t FLASH_ReadWord(uint32_t address)
{
    uint32_t temp1 = 0, temp2 = 0;
    temp1 = *(__IO uint16_t *)address;
    temp2 = *(__IO uint16_t *)(address + 2);
    return (temp2 << 16) + temp1;
}

//��ָ����ַ��ʼд��������
void FLASH_WriteMoreData(uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite)
{
    if (startAddress < FLASH_BASE || ((startAddress + countToWrite) >= (FLASH_BASE + 1024 * FLASH_SIZE)))
    {
        return; //�Ƿ���ַ
    }

    //���ݴ�С������˫�ֽڶ��롣
    if (countToWrite % 2 != 0)
        return;

    FLASH_Unlock();                                        //����д����
    uint32_t offsetAddress = startAddress - FLASH_BASE;    //����ȥ��0X08000000���ʵ��ƫ�Ƶ�ַ
    uint32_t sectorPosition = offsetAddress / SECTOR_SIZE; //����������ַ������STM32F103VET6Ϊ0~255

    uint32_t sectorStartAddress = sectorPosition * SECTOR_SIZE + FLASH_BASE; //��Ӧ�������׵�ַ

    FLASH_ErasePage(sectorStartAddress); //�����������

    uint16_t dataIndex = 0;
    for (dataIndex = 0; dataIndex < (countToWrite / 2); dataIndex++)
    {
        FLASH_ProgramHalfWord(startAddress + dataIndex * 2, writeData[dataIndex]);
    }

    FLASH_Lock(); //����д����
}
