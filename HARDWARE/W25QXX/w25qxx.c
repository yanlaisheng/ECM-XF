
#include "w25qxx.h"
#include "w25qxxConf.h"
#include "typedef.h"
#include "GlobalV_Extern.h" // ȫ�ֱ�������

#if (_W25QXX_DEBUG == 1)
#include <stdio.h>
#endif

#define W25QXX_DUMMY_BYTE 0xA5
#define SPI_FLASH_PageSize 256

//W25QXXʹ��
#define SPI_W25QXX_CS_ON HAL_GPIO_WritePin(_W25QXX_CS_GPIO, _W25QXX_CS_PIN, GPIO_PIN_RESET) //SPI_W25QXXʹ��
#define SPI_W25QXX_CS_OFF HAL_GPIO_WritePin(_W25QXX_CS_GPIO, _W25QXX_CS_PIN, GPIO_PIN_SET)	//SPI_W25QXX��ֹʹ��

/* Private define ------------------------------------------------------------*/
#define WRITE 0x02 /* Write to Memory instruction */	  //дָ��
#define WRSR 0x01 /* Write Status Register instruction */ //д״̬�Ĵ���ָ��
#define WREN 0x06 /* Write enable instruction */		  //дʹ��ָ��

#define READ 0x03 /* Read from Memory instruction */	  //��ָ��
#define RDSR 0x05 /* Read Status Register instruction  */ //��״̬�Ĵ���ָ��
#define RDID 0x9F /* Read identification */				  //��ID��ʾ��
#define SE 0xD8 /* Sector Erase instruction */			  //��������ָ��
#define BE 0xC7 /* Bulk Erase instruction */			  //ȫ������ָ��

#define WIP_Flag 0x01 /* Write In Progress (WIP) flag */ //�ڱ��д��־

#define Dummy_Byte 0xA5 //α�ֽ�

w25qxx_t w25qxx;

#if (_W25QXX_USE_FREERTOS == 1)
#define W25qxx_Delay(delay) osDelay(delay)
#include "cmsis_os.h"
#else
#define W25qxx_Delay(delay) HAL_Delay(delay)
#endif
//###################################################################################################################
uint8_t W25qxx_Spi(uint8_t Data)
{
	uint8_t ret;
	HAL_SPI_TransmitReceive(&_W25QXX_SPI, &Data, &ret, 1, 100);
	return ret;
}
//###################################################################################################################
uint32_t W25qxx_ReadID(void)
{
	uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
	SPI_W25QXX_CS_ON;
	//#define READ_JEDEC_ID_CMD                    0x9F
	W25qxx_Spi(0x9F);
	Temp0 = W25qxx_Spi(W25QXX_DUMMY_BYTE); //0xA5
	Temp1 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
	Temp2 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
	SPI_W25QXX_CS_OFF;
	Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
	return Temp;
}
//###################################################################################################################
void W25qxx_ReadUniqID(void)
{
	SPI_W25QXX_CS_ON;
	//#define Read Unique ID 0x4B
	W25qxx_Spi(0x4B);
	for (uint8_t i = 0; i < 4; i++)
		W25qxx_Spi(W25QXX_DUMMY_BYTE);
	for (uint8_t i = 0; i < 8; i++)
		w25qxx.UniqID[i] = W25qxx_Spi(W25QXX_DUMMY_BYTE);
	SPI_W25QXX_CS_OFF;
}
//###################################################################################################################
void W25qxx_WriteEnable(void)
{
	SPI_W25QXX_CS_ON;
	//WRITE_ENABLE_CMD 0x06
	W25qxx_Spi(0x06);
	SPI_W25QXX_CS_OFF;
	W25qxx_Delay(1);
}
//###################################################################################################################
void W25qxx_WriteDisable(void)
{
	SPI_W25QXX_CS_ON;
	//#define WRITE_DISABLE_CMD                    0x04
	W25qxx_Spi(0x04);
	SPI_W25QXX_CS_OFF;
	W25qxx_Delay(1);
}
//###################################################################################################################
uint8_t W25qxx_ReadStatusRegister(uint8_t SelectStatusRegister_1_2_3)
{
	uint8_t status = 0;
	SPI_W25QXX_CS_ON;
	if (SelectStatusRegister_1_2_3 == 1)
	{
		//#define READ_STATUS_REG1_CMD                  0x05
		W25qxx_Spi(0x05);
		status = W25qxx_Spi(W25QXX_DUMMY_BYTE);
		w25qxx.StatusRegister1 = status;
	}
	else if (SelectStatusRegister_1_2_3 == 2)
	{
		//#define READ_STATUS_REG2_CMD                  0x35
		W25qxx_Spi(0x35);
		status = W25qxx_Spi(W25QXX_DUMMY_BYTE);
		w25qxx.StatusRegister2 = status;
	}
	else
	{
		//#define READ_STATUS_REG3_CMD                  0x15
		W25qxx_Spi(0x15);
		status = W25qxx_Spi(W25QXX_DUMMY_BYTE);
		w25qxx.StatusRegister3 = status;
	}
	SPI_W25QXX_CS_OFF;
	return status;
}
//###################################################################################################################
void W25qxx_WriteStatusRegister(uint8_t SelectStatusRegister_1_2_3, uint8_t Data)
{
	SPI_W25QXX_CS_ON;
	if (SelectStatusRegister_1_2_3 == 1)
	{
		//#define WRITE_STATUS_REG1_CMD                 0x01
		W25qxx_Spi(0x01);
		w25qxx.StatusRegister1 = Data;
	}
	else if (SelectStatusRegister_1_2_3 == 2)
	{
		//#define WRITE_STATUS_REG2_CMD                 0x31
		W25qxx_Spi(0x31);
		w25qxx.StatusRegister2 = Data;
	}
	else
	{
		//#define WRITE_STATUS_REG3_CMD                 0x11
		W25qxx_Spi(0x11);
		w25qxx.StatusRegister3 = Data;
	}
	W25qxx_Spi(Data);
	SPI_W25QXX_CS_OFF;
}
//###################################################################################################################
void W25qxx_WaitForWriteEnd(void)
{
	W25qxx_Delay(1);
	SPI_W25QXX_CS_ON;
	//#define READ_STATUS_REG1_CMD                  0x05
	W25qxx_Spi(0x05);
	do
	{
		w25qxx.StatusRegister1 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
		W25qxx_Delay(1);
	} while ((w25qxx.StatusRegister1 & 0x01) == 0x01);
	SPI_W25QXX_CS_OFF;
}
//###################################################################################################################
bool W25qxx_Init(void)
{
	w25qxx.Lock = 1;
	while (HAL_GetTick() < 100)
		W25qxx_Delay(1);
	SPI_W25QXX_CS_OFF;
	W25qxx_Delay(100);
	uint32_t id;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx Init Begin...\r\n");
#endif
	//READ_JEDEC_ID_CMD  0x9F
	id = W25qxx_ReadID();

#if (_W25QXX_DEBUG == 1)
	printf("w25qxx ID:0x%X\r\n", id);
#endif
	switch (id & 0x0000FFFF)
	{
	case 0x401A: // 	w25q512
		w25qxx.ID = W25Q512;
		w25qxx.BlockCount = 1024;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q512\r\n");
#endif
		break;
	case 0x4019: // 	w25q256
		w25qxx.ID = W25Q256;
		w25qxx.BlockCount = 512;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q256\r\n");
#endif
		break;
	case 0x4018: // 	w25q128
		w25qxx.ID = W25Q128;
		w25qxx.BlockCount = 256;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q128\r\n");
#endif
		break;
	case 0x4017: //	w25q64
		w25qxx.ID = W25Q64;
		w25qxx.BlockCount = 128;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q64\r\n");
#endif
		break;
	case 0x4016: //	w25q32
		w25qxx.ID = W25Q32;
		w25qxx.BlockCount = 64;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q32\r\n");
#endif
		break;
	case 0x4015: //	w25q16
		w25qxx.ID = W25Q16;
		w25qxx.BlockCount = 32;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q16\r\n");
#endif
		break;
	case 0x4014: //	w25q80
		w25qxx.ID = W25Q80;
		w25qxx.BlockCount = 16;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q80\r\n");
#endif
		break;
	case 0x4013: //	w25q40
		w25qxx.ID = W25Q40;
		w25qxx.BlockCount = 8;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q40\r\n");
#endif
		break;
	case 0x4012: //	w25q20
		w25qxx.ID = W25Q20;
		w25qxx.BlockCount = 4;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q20\r\n");
#endif
		break;
	case 0x4011: //	w25q10
		w25qxx.ID = W25Q10;
		w25qxx.BlockCount = 2;
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Chip: w25q10\r\n");
#endif
		break;
	default:
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx Unknown ID\r\n");
#endif
		w25qxx.Lock = 0;
		return false;
	}
	w25qxx.PageSize = 256;
	w25qxx.SectorSize = 0x1000;
	w25qxx.SectorCount = w25qxx.BlockCount * 16;
	w25qxx.PageCount = (w25qxx.SectorCount * w25qxx.SectorSize) / w25qxx.PageSize;
	w25qxx.BlockSize = w25qxx.SectorSize * 16;
	w25qxx.CapacityInKiloByte = (w25qxx.SectorCount * w25qxx.SectorSize) / 1024;
	W25qxx_ReadUniqID();
	W25qxx_ReadStatusRegister(1);
	W25qxx_ReadStatusRegister(2);
	W25qxx_ReadStatusRegister(3);
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx Page Size: %d Bytes\r\n", w25qxx.PageSize);
	printf("w25qxx Page Count: %d\r\n", w25qxx.PageCount);
	printf("w25qxx Sector Size: %d Bytes\r\n", w25qxx.SectorSize);
	printf("w25qxx Sector Count: %d\r\n", w25qxx.SectorCount);
	printf("w25qxx Block Size: %d Bytes\r\n", w25qxx.BlockSize);
	printf("w25qxx Block Count: %d\r\n", w25qxx.BlockCount);
	printf("w25qxx Capacity: %d KiloBytes\r\n", w25qxx.CapacityInKiloByte);
	printf("w25qxx Init Done\r\n");
#endif
	w25qxx.Lock = 0;
	return true;
}
//###################################################################################################################
//��������оƬ
//�ȴ�ʱ�䳬��...
void W25qxx_EraseChip(void)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = HAL_GetTick();
	printf("w25qxx EraseChip Begin...\r\n");
#endif
	W25qxx_WriteEnable(); //����д��
	SPI_W25QXX_CS_ON;

	//#define CHIP_ERASE_CMD                       0xC7
	W25qxx_Spi(0xC7);
	SPI_W25QXX_CS_OFF;
	W25qxx_WaitForWriteEnd(); //�ȴ�����/д�����
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx EraseBlock done after %d ms!\r\n", HAL_GetTick() - StartTime);
#endif
	W25qxx_Delay(10);
	w25qxx.Lock = 0;
}
//###################################################################################################################
//����һ������
//SectorAddr:������ַ ����ʵ����������
//�����������1����SectorAddr=1����ʵ�ʵ�ַҪ����4096
//����һ������������ʱ��:150ms
void W25qxx_EraseSector(uint32_t SectorAddr)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = HAL_GetTick();
	printf("w25qxx EraseSector %d Begin...\r\n", SectorAddr);
#endif
	W25qxx_WaitForWriteEnd();					 //�ȴ�����/д�����
	SectorAddr = SectorAddr * w25qxx.SectorSize; //ע�⣺��ַҪ����4096
	W25qxx_WriteEnable();
	SPI_W25QXX_CS_ON;
	//#define SECTOR_ERASE_CMD                     0x20
	W25qxx_Spi(0x20); //�Ȳ�������������0x20+Ҫ������������ַ
	if (w25qxx.ID >= W25Q256)
		W25qxx_Spi((SectorAddr & 0xFF000000) >> 24);
	W25qxx_Spi((SectorAddr & 0xFF0000) >> 16);
	W25qxx_Spi((SectorAddr & 0xFF00) >> 8);
	W25qxx_Spi(SectorAddr & 0xFF);
	SPI_W25QXX_CS_OFF;
	W25qxx_WaitForWriteEnd(); //�ȴ�����/д�����
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx EraseSector done after %d ms\r\n", HAL_GetTick() - StartTime);
#endif
	W25qxx_Delay(1);
	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_EraseBlock(uint32_t BlockAddr)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx EraseBlock %d Begin...\r\n", BlockAddr);
	W25qxx_Delay(100);
	uint32_t StartTime = HAL_GetTick();
#endif
	W25qxx_WaitForWriteEnd();
	BlockAddr = BlockAddr * w25qxx.SectorSize * 16;
	W25qxx_WriteEnable();
	SPI_W25QXX_CS_ON;
	//Block Erase(64KB)  0xD8
	W25qxx_Spi(0xD8);
	if (w25qxx.ID >= W25Q256)
		W25qxx_Spi((BlockAddr & 0xFF000000) >> 24);
	W25qxx_Spi((BlockAddr & 0xFF0000) >> 16);
	W25qxx_Spi((BlockAddr & 0xFF00) >> 8);
	W25qxx_Spi(BlockAddr & 0xFF);
	SPI_W25QXX_CS_OFF;
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx EraseBlock done after %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	W25qxx_Delay(1);
	w25qxx.Lock = 0;
}
//###################################################################################################################
uint32_t W25qxx_PageToSector(uint32_t PageAddress)
{
	return ((PageAddress * w25qxx.PageSize) / w25qxx.SectorSize);
}
//###################################################################################################################
uint32_t W25qxx_PageToBlock(uint32_t PageAddress)
{
	return ((PageAddress * w25qxx.PageSize) / w25qxx.BlockSize);
}
//###################################################################################################################
uint32_t W25qxx_SectorToBlock(uint32_t SectorAddress)
{
	return ((SectorAddress * w25qxx.SectorSize) / w25qxx.BlockSize);
}
//###################################################################################################################
uint32_t W25qxx_SectorToPage(uint32_t SectorAddress)
{
	return (SectorAddress * w25qxx.SectorSize) / w25qxx.PageSize;
}
//###################################################################################################################
uint32_t W25qxx_BlockToPage(uint32_t BlockAddress)
{
	return (BlockAddress * w25qxx.BlockSize) / w25qxx.PageSize;
}
//###################################################################################################################
bool W25qxx_IsEmptyPage(uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
	if (((NumByteToCheck_up_to_PageSize + OffsetInByte) > w25qxx.PageSize) || (NumByteToCheck_up_to_PageSize == 0))
		NumByteToCheck_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckPage:%d, Offset:%d, Bytes:%d begin...\r\n", Page_Address, OffsetInByte, NumByteToCheck_up_to_PageSize);
	W25qxx_Delay(100);
	uint32_t StartTime = HAL_GetTick();
#endif
	uint8_t pBuffer[32];
	uint32_t WorkAddress;
	uint32_t i;
	for (i = OffsetInByte; i < w25qxx.PageSize; i += sizeof(pBuffer))
	{
		SPI_W25QXX_CS_ON;
		WorkAddress = (i + Page_Address * w25qxx.PageSize);
		//#define FAST_READ_CMD                        0x0B
		W25qxx_Spi(0x0B);
		if (w25qxx.ID >= W25Q256)
			W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
		W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(WorkAddress & 0xFF);
		W25qxx_Spi(0);
		HAL_SPI_Receive(&_W25QXX_SPI, pBuffer, sizeof(pBuffer), 100);
		SPI_W25QXX_CS_OFF;
		for (uint8_t x = 0; x < sizeof(pBuffer); x++)
		{
			if (pBuffer[x] != 0xFF)
				goto NOT_EMPTY;
		}
	}
	if ((w25qxx.PageSize + OffsetInByte) % sizeof(pBuffer) != 0)
	{
		i -= sizeof(pBuffer);
		for (; i < w25qxx.PageSize; i++)
		{
			SPI_W25QXX_CS_ON;
			WorkAddress = (i + Page_Address * w25qxx.PageSize);
			W25qxx_Spi(0x0B);
			if (w25qxx.ID >= W25Q256)
				W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
			W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(WorkAddress & 0xFF);
			W25qxx_Spi(0);
			HAL_SPI_Receive(&_W25QXX_SPI, pBuffer, 1, 100);
			SPI_W25QXX_CS_OFF;
			if (pBuffer[0] != 0xFF)
				goto NOT_EMPTY;
		}
	}
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckPage is Empty in %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
	return true;
NOT_EMPTY:
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckPage is Not Empty in %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
	return false;
}
//###################################################################################################################
bool W25qxx_IsEmptySector(uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
	if ((NumByteToCheck_up_to_SectorSize > w25qxx.SectorSize) || (NumByteToCheck_up_to_SectorSize == 0))
		NumByteToCheck_up_to_SectorSize = w25qxx.SectorSize;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckSector:%d, Offset:%d, Bytes:%d begin...\r\n", Sector_Address, OffsetInByte, NumByteToCheck_up_to_SectorSize);
	W25qxx_Delay(100);
	uint32_t StartTime = HAL_GetTick();
#endif
	uint8_t pBuffer[32];
	uint32_t WorkAddress;
	uint32_t i;
	for (i = OffsetInByte; i < w25qxx.SectorSize; i += sizeof(pBuffer))
	{
		SPI_W25QXX_CS_ON;
		WorkAddress = (i + Sector_Address * w25qxx.SectorSize);
		W25qxx_Spi(0x0B);
		if (w25qxx.ID >= W25Q256)
			W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
		W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(WorkAddress & 0xFF);
		W25qxx_Spi(0);
		HAL_SPI_Receive(&_W25QXX_SPI, pBuffer, sizeof(pBuffer), 100);
		SPI_W25QXX_CS_OFF;
		for (uint8_t x = 0; x < sizeof(pBuffer); x++)
		{
			if (pBuffer[x] != 0xFF)
				goto NOT_EMPTY;
		}
	}
	if ((w25qxx.SectorSize + OffsetInByte) % sizeof(pBuffer) != 0)
	{
		i -= sizeof(pBuffer);
		for (; i < w25qxx.SectorSize; i++)
		{
			SPI_W25QXX_CS_ON;
			WorkAddress = (i + Sector_Address * w25qxx.SectorSize);
			W25qxx_Spi(0x0B);
			if (w25qxx.ID >= W25Q256)
				W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
			W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(WorkAddress & 0xFF);
			W25qxx_Spi(0);
			HAL_SPI_Receive(&_W25QXX_SPI, pBuffer, 1, 100);
			SPI_W25QXX_CS_OFF;
			if (pBuffer[0] != 0xFF)
				goto NOT_EMPTY;
		}
	}
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckSector is Empty in %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
	return true;
NOT_EMPTY:
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckSector is Not Empty in %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
	return false;
}
//###################################################################################################################
bool W25qxx_IsEmptyBlock(uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
	if ((NumByteToCheck_up_to_BlockSize > w25qxx.BlockSize) || (NumByteToCheck_up_to_BlockSize == 0))
		NumByteToCheck_up_to_BlockSize = w25qxx.BlockSize;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckBlock:%d, Offset:%d, Bytes:%d begin...\r\n", Block_Address, OffsetInByte, NumByteToCheck_up_to_BlockSize);
	W25qxx_Delay(100);
	uint32_t StartTime = HAL_GetTick();
#endif
	uint8_t pBuffer[32];
	uint32_t WorkAddress;
	uint32_t i;
	for (i = OffsetInByte; i < w25qxx.BlockSize; i += sizeof(pBuffer))
	{
		SPI_W25QXX_CS_ON;
		WorkAddress = (i + Block_Address * w25qxx.BlockSize);
		W25qxx_Spi(0x0B);
		if (w25qxx.ID >= W25Q256)
			W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
		W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(WorkAddress & 0xFF);
		W25qxx_Spi(0);
		HAL_SPI_Receive(&_W25QXX_SPI, pBuffer, sizeof(pBuffer), 100);
		SPI_W25QXX_CS_OFF;
		for (uint8_t x = 0; x < sizeof(pBuffer); x++)
		{
			if (pBuffer[x] != 0xFF)
				goto NOT_EMPTY;
		}
	}
	if ((w25qxx.BlockSize + OffsetInByte) % sizeof(pBuffer) != 0)
	{
		i -= sizeof(pBuffer);
		for (; i < w25qxx.BlockSize; i++)
		{
			SPI_W25QXX_CS_ON;
			WorkAddress = (i + Block_Address * w25qxx.BlockSize);
			W25qxx_Spi(0x0B);
			if (w25qxx.ID >= W25Q256)
				W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
			W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(WorkAddress & 0xFF);
			W25qxx_Spi(0);
			HAL_SPI_Receive(&_W25QXX_SPI, pBuffer, 1, 100);
			SPI_W25QXX_CS_OFF;
			if (pBuffer[0] != 0xFF)
				goto NOT_EMPTY;
		}
	}
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckBlock is Empty in %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
	return true;
NOT_EMPTY:
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx CheckBlock is Not Empty in %d ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif
	w25qxx.Lock = 0;
	return false;
}

//###################################################################################################################
//�ӵ�ַWriteAddr_inBytesд��1���ֽ�
//����ȷ����д�ĵ�ַ������Ϊ0XFF�������ڷ�0XFF��д������ݽ�ʧ��!
void W25qxx_WriteByte(uint8_t pBuffer, uint32_t WriteAddr_inBytes)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = HAL_GetTick();
	printf("w25qxx WriteByte 0x%02X at address %d begin...", pBuffer, WriteAddr_inBytes);
#endif
	W25qxx_WaitForWriteEnd();
	W25qxx_WriteEnable();
	SPI_W25QXX_CS_ON;
	//#define PAGE_PROG_CMD                        0x02
	W25qxx_Spi(0x02);
	if (w25qxx.ID >= W25Q256)
		W25qxx_Spi((WriteAddr_inBytes & 0xFF000000) >> 24);
	W25qxx_Spi((WriteAddr_inBytes & 0xFF0000) >> 16);
	W25qxx_Spi((WriteAddr_inBytes & 0xFF00) >> 8);
	W25qxx_Spi(WriteAddr_inBytes & 0xFF);
	W25qxx_Spi(pBuffer);
	SPI_W25QXX_CS_OFF;
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx WriteByte done after %d ms\r\n", HAL_GetTick() - StartTime);
#endif
	w25qxx.Lock = 0;
}

//###################################################################################################################
//SPI��һҳ��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!
//��У���Ƿ�Ϊ0xFF��ֱ��д
void W25qxx_WriteBytes_Page(u8 *pBuffer, u32 WriteAddr_inBytes, u16 NumByteToWrite)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = HAL_GetTick();
	printf("w25qxx WriteByte 0x%02X at address %d begin...\r\n", pBuffer, WriteAddr_inBytes);
#endif
	W25qxx_WaitForWriteEnd();
	W25qxx_WriteEnable();
	SPI_W25QXX_CS_ON;
	//#define PAGE_PROG_CMD                        0x02
	W25qxx_Spi(0x02);
	if (w25qxx.ID >= W25Q256)
		W25qxx_Spi((WriteAddr_inBytes & 0xFF000000) >> 24);
	W25qxx_Spi((WriteAddr_inBytes & 0xFF0000) >> 16);
	W25qxx_Spi((WriteAddr_inBytes & 0xFF00) >> 8);
	W25qxx_Spi(WriteAddr_inBytes & 0xFF);

	HAL_SPI_Transmit(&_W25QXX_SPI, pBuffer, NumByteToWrite, 100);
	SPI_W25QXX_CS_OFF;
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx WriteByte done after %d ms\r\n", HAL_GetTick() - StartTime);
#endif
	w25qxx.Lock = 0;
}

//�޼���дSPI FLASH
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ����
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void W25QXX_Write_NoCheck(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
	u16 pageremain;
	pageremain = 256 - WriteAddr % 256; //��ҳʣ����ֽ���
	if (NumByteToWrite <= pageremain)
		pageremain = NumByteToWrite; //������256���ֽ�
	while (1)
	{
		W25qxx_WriteBytes_Page(pBuffer, WriteAddr, pageremain); //ֻ�ܰ�ҳд
		if (NumByteToWrite == pageremain)
			break; //д�������
		else	   //NumByteToWrite>pageremain
		{
			pBuffer += pageremain;
			WriteAddr += pageremain;

			NumByteToWrite -= pageremain; //��ȥ�Ѿ�д���˵��ֽ���
			if (NumByteToWrite > 256)
				pageremain = 256; //һ�ο���д��256���ֽ�
			else
				pageremain = NumByteToWrite; //����256���ֽ���
		}
	};
}

//###################################################################################################################
//����ҳ����д
//���256���ֽ�
//ƫ����+Ҫд����������ܴ���256
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
void W25qxx_WritePage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
	if (((NumByteToWrite_up_to_PageSize + OffsetInByte) > w25qxx.PageSize) || (NumByteToWrite_up_to_PageSize == 0))
		NumByteToWrite_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
	if ((OffsetInByte + NumByteToWrite_up_to_PageSize) > w25qxx.PageSize)
		NumByteToWrite_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx WritePage:%d, Offset:%d ,Writes %d Bytes, begin...\r\n", Page_Address, OffsetInByte, NumByteToWrite_up_to_PageSize);
	W25qxx_Delay(100);
	uint32_t StartTime = HAL_GetTick();
#endif
	W25qxx_WaitForWriteEnd();
	W25qxx_WriteEnable();
	SPI_W25QXX_CS_ON;
	W25qxx_Spi(0x02);
	Page_Address = (Page_Address * w25qxx.PageSize) + OffsetInByte;
	if (w25qxx.ID >= W25Q256)
		W25qxx_Spi((Page_Address & 0xFF000000) >> 24);
	W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
	W25qxx_Spi((Page_Address & 0xFF00) >> 8);
	W25qxx_Spi(Page_Address & 0xFF);
	HAL_SPI_Transmit(&_W25QXX_SPI, pBuffer, NumByteToWrite_up_to_PageSize, 100);
	SPI_W25QXX_CS_OFF;
	W25qxx_WaitForWriteEnd();
#if (_W25QXX_DEBUG == 1)
	StartTime = HAL_GetTick() - StartTime;
	for (uint32_t i = 0; i < NumByteToWrite_up_to_PageSize; i++)
	{
		if ((i % 8 == 0) && (i > 2))
		{
			printf("\r\n");
			W25qxx_Delay(10);
		}
		printf("0x%02X,", pBuffer[i]);
	}
	printf("\r\n");
	printf("w25qxx WritePage done after %d ms\r\n", StartTime);
	W25qxx_Delay(100);
#endif
	W25qxx_Delay(1);
	w25qxx.Lock = 0;
}
//###################################################################################################################
//����������д

void W25qxx_WriteSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize)
{
	if ((NumByteToWrite_up_to_SectorSize > w25qxx.SectorSize) || (NumByteToWrite_up_to_SectorSize == 0))
		NumByteToWrite_up_to_SectorSize = w25qxx.SectorSize;
#if (_W25QXX_DEBUG == 1)
	printf("+++w25qxx WriteSector:%d, Offset:%d ,Write %d Bytes, begin...\r\n", Sector_Address, OffsetInByte, NumByteToWrite_up_to_SectorSize);
	W25qxx_Delay(100);
#endif
	if (OffsetInByte >= w25qxx.SectorSize)
	{
#if (_W25QXX_DEBUG == 1)
		printf("---w25qxx WriteSector Faild!\r\n");
		W25qxx_Delay(100);
#endif
		return;
	}
	uint32_t StartPage;
	int32_t BytesToWrite;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToWrite_up_to_SectorSize) > w25qxx.SectorSize)
		BytesToWrite = w25qxx.SectorSize - OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_SectorSize;
	StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx.PageSize);
	LocalOffset = OffsetInByte % w25qxx.PageSize;
	do
	{
		W25qxx_WritePage(pBuffer, StartPage, LocalOffset, BytesToWrite);
		StartPage++;
		BytesToWrite -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToWrite > 0);
#if (_W25QXX_DEBUG == 1)
	printf("---w25qxx WriteSector Done\r\n");
	W25qxx_Delay(100);
#endif
}

//###################################################################################################################
//�����ַ��д
void W25qxx_WriteBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize)
{
	if ((NumByteToWrite_up_to_BlockSize > w25qxx.BlockSize) || (NumByteToWrite_up_to_BlockSize == 0))
		NumByteToWrite_up_to_BlockSize = w25qxx.BlockSize;
#if (_W25QXX_DEBUG == 1)
	printf("+++w25qxx WriteBlock:%d, Offset:%d ,Write %d Bytes, begin...\r\n", Block_Address, OffsetInByte, NumByteToWrite_up_to_BlockSize);
	W25qxx_Delay(100);
#endif
	if (OffsetInByte >= w25qxx.BlockSize)
	{
#if (_W25QXX_DEBUG == 1)
		printf("---w25qxx WriteBlock Faild!\r\n");
		W25qxx_Delay(100);
#endif
		return;
	}
	uint32_t StartPage;
	int32_t BytesToWrite;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToWrite_up_to_BlockSize) > w25qxx.BlockSize)
		BytesToWrite = w25qxx.BlockSize - OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_BlockSize;
	StartPage = W25qxx_BlockToPage(Block_Address) + (OffsetInByte / w25qxx.PageSize);
	LocalOffset = OffsetInByte % w25qxx.PageSize;
	do
	{
		W25qxx_WritePage(pBuffer, StartPage, LocalOffset, BytesToWrite);
		StartPage++;
		BytesToWrite -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToWrite > 0);
#if (_W25QXX_DEBUG == 1)
	printf("---w25qxx WriteBlock Done\r\n");
	W25qxx_Delay(100);
#endif
}
//###################################################################################################################
void W25qxx_ReadByte(uint8_t *pBuffer, uint32_t Bytes_Address)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = HAL_GetTick();
	printf("w25qxx ReadByte at address %d begin...\r\n", Bytes_Address);
#endif
	SPI_W25QXX_CS_ON;
	//���ٶ����� 0x0B
	W25qxx_Spi(0x0B);
	if (w25qxx.ID >= W25Q256)
		W25qxx_Spi((Bytes_Address & 0xFF000000) >> 24);
	W25qxx_Spi((Bytes_Address & 0xFF0000) >> 16);
	W25qxx_Spi((Bytes_Address & 0xFF00) >> 8);
	W25qxx_Spi(Bytes_Address & 0xFF);
	W25qxx_Spi(0);
	*pBuffer = W25qxx_Spi(W25QXX_DUMMY_BYTE);
	SPI_W25QXX_CS_OFF;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx ReadByte 0x%02X done after %d ms\r\n", *pBuffer, HAL_GetTick() - StartTime);
#endif
	w25qxx.Lock = 0;
}
//###################################################################################################################
//������ֽڣ���ൽ����оƬ����
void W25qxx_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
#if (_W25QXX_DEBUG == 1)
	uint32_t StartTime = HAL_GetTick();
	printf("w25qxx ReadBytes at Address:%d, %d Bytes  begin...\r\n", ReadAddr, NumByteToRead);
#endif
	SPI_W25QXX_CS_ON;
	W25qxx_Spi(0x0B);
	if (w25qxx.ID >= W25Q256)
		W25qxx_Spi((ReadAddr & 0xFF000000) >> 24);
	W25qxx_Spi((ReadAddr & 0xFF0000) >> 16);
	W25qxx_Spi((ReadAddr & 0xFF00) >> 8);
	W25qxx_Spi(ReadAddr & 0xFF);
	W25qxx_Spi(0);
	HAL_SPI_Receive(&_W25QXX_SPI, pBuffer, NumByteToRead, 2000);
	SPI_W25QXX_CS_OFF;
#if (_W25QXX_DEBUG == 1)
	StartTime = HAL_GetTick() - StartTime;
	for (uint32_t i = 0; i < NumByteToRead; i++)
	{
		if ((i % 8 == 0) && (i > 2))
		{
			printf("\r\n");
			W25qxx_Delay(10);
		}
		printf("0x%02X,", pBuffer[i]);
	}
	printf("\r\n");
	printf("w25qxx ReadBytes done after %d ms\r\n", StartTime);
	W25qxx_Delay(100);
#endif
	W25qxx_Delay(1);
	w25qxx.Lock = 0;
}
//###################################################################################################################
//��ҳ������
//ֻ����ҳ�ڣ����256���ֽ�

void W25qxx_ReadPage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize)
{
	while (w25qxx.Lock == 1)
		W25qxx_Delay(1);
	w25qxx.Lock = 1;
	if ((NumByteToRead_up_to_PageSize > w25qxx.PageSize) || (NumByteToRead_up_to_PageSize == 0))
		NumByteToRead_up_to_PageSize = w25qxx.PageSize;
	if ((OffsetInByte + NumByteToRead_up_to_PageSize) > w25qxx.PageSize)
		NumByteToRead_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
#if (_W25QXX_DEBUG == 1)
	printf("w25qxx ReadPage:%d, Offset:%d ,Read %d Bytes, begin...\r\n", Page_Address, OffsetInByte, NumByteToRead_up_to_PageSize);
	W25qxx_Delay(100);
	uint32_t StartTime = HAL_GetTick();
#endif
	Page_Address = Page_Address * w25qxx.PageSize + OffsetInByte;
	SPI_W25QXX_CS_ON;
	W25qxx_Spi(0x0B);
	if (w25qxx.ID >= W25Q256)
		W25qxx_Spi((Page_Address & 0xFF000000) >> 24);
	W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
	W25qxx_Spi((Page_Address & 0xFF00) >> 8);
	W25qxx_Spi(Page_Address & 0xFF);
	W25qxx_Spi(0);
	HAL_SPI_Receive(&_W25QXX_SPI, pBuffer, NumByteToRead_up_to_PageSize, 100);
	SPI_W25QXX_CS_OFF;
#if (_W25QXX_DEBUG == 1)
	StartTime = HAL_GetTick() - StartTime;
	for (uint32_t i = 0; i < NumByteToRead_up_to_PageSize; i++)
	{
		if ((i % 8 == 0) && (i > 2))
		{
			printf("\r\n");
			W25qxx_Delay(10);
		}
		printf("0x%02X,", pBuffer[i]);
	}
	printf("\r\n");
	printf("w25qxx ReadPage done after %d ms\r\n", StartTime);
	W25qxx_Delay(100);
#endif
	W25qxx_Delay(1);
	w25qxx.Lock = 0;
}
//###################################################################################################################
void W25qxx_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize)
{
	if ((NumByteToRead_up_to_SectorSize > w25qxx.SectorSize) || (NumByteToRead_up_to_SectorSize == 0))
		NumByteToRead_up_to_SectorSize = w25qxx.SectorSize;
#if (_W25QXX_DEBUG == 1)
	printf("+++w25qxx ReadSector:%d, Offset:%d ,Read %d Bytes, begin...\r\n", Sector_Address, OffsetInByte, NumByteToRead_up_to_SectorSize);
	W25qxx_Delay(100);
#endif
	if (OffsetInByte >= w25qxx.SectorSize)
	{
#if (_W25QXX_DEBUG == 1)
		printf("---w25qxx ReadSector Faild!\r\n");
		W25qxx_Delay(100);
#endif
		return;
	}
	uint32_t StartPage;
	int32_t BytesToRead;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToRead_up_to_SectorSize) > w25qxx.SectorSize)
		BytesToRead = w25qxx.SectorSize - OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_SectorSize;
	StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx.PageSize);
	LocalOffset = OffsetInByte % w25qxx.PageSize;
	do
	{
		W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
		StartPage++;
		BytesToRead -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToRead > 0);
#if (_W25QXX_DEBUG == 1)
	printf("---w25qxx ReadSector Done\r\n");
	W25qxx_Delay(100);
#endif
}
//###################################################################################################################
void W25qxx_ReadBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize)
{
	if ((NumByteToRead_up_to_BlockSize > w25qxx.BlockSize) || (NumByteToRead_up_to_BlockSize == 0))
		NumByteToRead_up_to_BlockSize = w25qxx.BlockSize;
#if (_W25QXX_DEBUG == 1)
	printf("+++w25qxx ReadBlock:%d, Offset:%d ,Read %d Bytes, begin...\r\n", Block_Address, OffsetInByte, NumByteToRead_up_to_BlockSize);
	W25qxx_Delay(100);
#endif
	if (OffsetInByte >= w25qxx.BlockSize)
	{
#if (_W25QXX_DEBUG == 1)
		printf("w25qxx ReadBlock Faild!\r\n");
		W25qxx_Delay(100);
#endif
		return;
	}
	uint32_t StartPage;
	int32_t BytesToRead;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToRead_up_to_BlockSize) > w25qxx.BlockSize)
		BytesToRead = w25qxx.BlockSize - OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_BlockSize;
	StartPage = W25qxx_BlockToPage(Block_Address) + (OffsetInByte / w25qxx.PageSize);
	LocalOffset = OffsetInByte % w25qxx.PageSize;
	do
	{
		W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
		StartPage++;
		BytesToRead -= w25qxx.PageSize - LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToRead > 0);
#if (_W25QXX_DEBUG == 1)
	printf("---w25qxx ReadBlock Done\r\n");
	W25qxx_Delay(100);
#endif
}
//###################################################################################################################
//дSPI FLASH
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//���ѵ�ʱ��Ƚϳ�����ʹд1���ֽڣ�ҲҪ��������ȫ����������Ȼ��ÿ���ֽ��ж��Ƿ�Ϊ0xFF��Ȼ�����������Ȼ����д����������
u8 W25QXX_BUFFER[4096];
void W25QXX_Write_WithErase(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
	u32 secpos;
	u16 secoff;
	u16 secremain;
	u16 i;
	u8 *W25QXX_BUF;
	W25QXX_BUF = W25QXX_BUFFER;
	secpos = WriteAddr / 4096; //������ַ
	secoff = WriteAddr % 4096; //�������ڵ�ƫ��
	secremain = 4096 - secoff; //����ʣ��ռ��С
#if (_W25QXX_DEBUG == 1)
	printf("W25QXX_Write_WithErase address:%X,number bytes:%X\r\n", WriteAddr, NumByteToWrite); //������
#endif
	if (NumByteToWrite <= secremain)
		secremain = NumByteToWrite; //������4096���ֽ�
	while (1)
	{
		W25qxx_ReadBytes(W25QXX_BUF, secpos * 4096, 4096); //������������������
		for (i = 0; i < secremain; i++)					   //У������
		{
			if (W25QXX_BUF[secoff + i] != 0XFF)
				break; //�ж��Ƿ�Ϊ��
		}
		if (i < secremain) //��Ҫ����
		{
			W25qxx_EraseSector(secpos);		//�����������
			for (i = 0; i < secremain; i++) //����
			{
				W25QXX_BUF[i + secoff] = pBuffer[i];
			}

			W25QXX_Write_NoCheck(W25QXX_BUF, secpos * 4096, 4096); //д����������
		}
		else
			W25QXX_Write_NoCheck(pBuffer, WriteAddr, secremain); //д�Ѿ������˵�,ֱ��д������ʣ������.

		if (NumByteToWrite == secremain)
			break; //д�������
		else	   //д��δ����
		{
			secpos++;	//������ַ��1
			secoff = 0; //ƫ��λ��Ϊ0

			pBuffer += secremain;		 //ָ��ƫ��
			WriteAddr += secremain;		 //д��ַƫ��
			NumByteToWrite -= secremain; //�ֽ����ݼ�
			if (NumByteToWrite > 4096)
				secremain = 4096; //��һ����������д����
			else
				secremain = NumByteToWrite; //��һ����������д����
		}
	};
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferWrite  ����д���Ը���д�������д��Ҳ����
* Description    : Writes block of data to the FLASH. In this function, the
*                  number of WRITE cycles are reduced, using Page WRITE sequence.  ʹ��ҳд˳��
* Input          : - pBuffer : pointer to the buffer  containing the data to be 
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
//ֱ��д��������Ƿ�Ϊ�գ����д65535���ֽ�
//����ʱ��Ҫ��һЩ��ʡȥ�˶�������������Ȼ��ÿ���ֽ��жϵĹ���
//������ǰ����Ҫд������
void SPI_FLASH_BufferWrite(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
	u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

	Addr = WriteAddr % SPI_FLASH_PageSize;			   //���࣬��:5%4=1
	count = SPI_FLASH_PageSize - Addr;				   //4-1=3;����ҳ���пռ���
	NumOfPage = NumByteToWrite / SPI_FLASH_PageSize;   //26/4=6  ����ҳ
	NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize; //26%4=2	����һҳ���ֽ���

	if (Addr == 0) /* WriteAddr is SPI_FLASH_PageSize aligned  */ //һ��������ҳ����ʼ��
	{
		if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */ //��ʾҪд���ֽ�������һҳ��256���ֽڣ�
		{
			W25qxx_WriteBytes_Page(pBuffer, WriteAddr, NumByteToWrite); //�����ֱ��д�루�����Ƿ��Ѿ�������������ִ���������ǰҪ�Ȳ�����
		}
		else /* NumByteToWrite > SPI_FLASH_PageSize */
		{
			while (NumOfPage--) //�������һҳ
			{
				W25qxx_WriteBytes_Page(pBuffer, WriteAddr, SPI_FLASH_PageSize); //д����ҳ
				WriteAddr += SPI_FLASH_PageSize;
				pBuffer += SPI_FLASH_PageSize;
			}

			W25qxx_WriteBytes_Page(pBuffer, WriteAddr, NumOfSingle); //д����һҳ��ʣ�ಿ��
		}
	}
	else /* WriteAddr is not SPI_FLASH_PageSize aligned  */ //������Ǵ�ҳ����ʼ�㿪ʼд
	{
		if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */ //�������һҳ
		{
			//���Ҫд���ֽ���������ҳʣ��ռ���ֽ���
			if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize */ //�˴���I2C��һ��
			{
				temp = NumOfSingle - count; //count��1.��ҳ�����ֽ���

				W25qxx_WriteBytes_Page(pBuffer, WriteAddr, count); //2.д��ҳָ����ַ��ҳβ
				WriteAddr += count;
				pBuffer += count;

				W25qxx_WriteBytes_Page(pBuffer, WriteAddr, temp); //3.������һҳдʣ�ಿ��
			}
			else
			{
				W25qxx_WriteBytes_Page(pBuffer, WriteAddr, NumByteToWrite); //Ҫд���ֽ���С�ڱ�ҳʣ���ֽڣ�����ֱ��д
			}
		}
		else /* NumByteToWrite > SPI_FLASH_PageSize */
		{
			NumByteToWrite -= count;						   //count��1.��ҳ��Ҫд��ʣ����
			NumOfPage = NumByteToWrite / SPI_FLASH_PageSize;   //2.��ҳ��
			NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize; //3.����Ҫд�Ĳ���һҳ��ʣ����

			W25qxx_WriteBytes_Page(pBuffer, WriteAddr, count); //1.д��ҳʣ���ֽڣ��ֽ�����COUNT��
			WriteAddr += count;
			pBuffer += count;

			while (NumOfPage--)
			{
				W25qxx_WriteBytes_Page(pBuffer, WriteAddr, SPI_FLASH_PageSize); //2.����д��ҳ
				WriteAddr += SPI_FLASH_PageSize;
				pBuffer += SPI_FLASH_PageSize;
			}

			if (NumOfSingle != 0)
			{
				W25qxx_WriteBytes_Page(pBuffer, WriteAddr, NumOfSingle); //3.д����һҳ��ʣ�ಿ��
			}
		}
	}
}

//����Ϊ����RAM FM25L16 ����-------------------------------------------------------------------------
//###################################################################################################################
uint8_t SPI_FLASH_SendByte(uint8_t Data)
{
	uint8_t ret;
	HAL_SPI_TransmitReceive(&_W25QXX_SPI, &Data, &ret, 1, 100);
	return ret;
}

/*******************************************************************************
* Function Name  : SPI_FMRAM_BufferWrite  ���绺��д-��������д
* Description    : Writes block of data to the FLASH. In this function, the
*                  number of WRITE cycles are reduced, using Page WRITE sequence.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FMRAM_BufferWrite(u8 *pBuffer, u16 WriteAddr, u16 NumByteToWrite)
{
	B_SelCS = CS_FMRAM1; //ZCL 2020.3.18

	/* Enable the write access to the FLASH */ //0. ʹ��FLASHдʹ��
	SPI_FLASH_WriteEnable();

	/* Select the FLASH: Chip Select low */ //1. ʹ��FLASHƬѡ
	SPI_FLASH_CS_LOW();
	/* Send "Write to Memory " instruction */ //2. ����дָ��
	SPI_FLASH_SendByte(WRITE);
	/* Send WriteAddr medium nibble address byte to write to */ //3.�����м��ֽڵ�ַ(û�и��ֽ�)
	SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
	/* Send WriteAddr low nibble address byte to write to */ //4.���͵��ֽڵ�ַ
	SPI_FLASH_SendByte(WriteAddr & 0xFF);

	/* while there is data to be written on the FLASH */
	while (NumByteToWrite--) // 5. ѭ��д
	{
		/* Send the current byte */
		SPI_FLASH_SendByte(*pBuffer); //6. �����ֽ�
		/* Point on the next byte to be written */
		pBuffer++;
	}

	/* Deselect the FLASH: Chip Select high */ //7.��ֹFLASHƬѡ
	SPI_FLASH_CS_HIGH();

	/* Wait the end of Flash writing */ //8. �ȴ�FLASHд����
	SPI_FLASH_WaitForWriteEnd();		//�ܳ��ڼǺţ�д����д����
}

/*******************************************************************************
* Function Name  : SPI_FMRAM_BufferRead     ���绺���-�Ƚϼ�
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FMRAM_BufferRead(u8 *pBuffer, u16 ReadAddr, u16 NumByteToRead)
{
	/* Select the FLASH: Chip Select low */
	B_SelCS = CS_FMRAM1; //ZCL 2020.3.18
	SPI_FLASH_CS_LOW();

	/* Send "Read from Memory " instruction */ //���Ͷ�ָ��
	SPI_FLASH_SendByte(READ);

	/* Send ReadAddr medium nibble address byte to read from */
	SPI_FLASH_SendByte((ReadAddr & 0xFF00) >> 8);
	/* Send ReadAddr low nibble address byte to read from */
	SPI_FLASH_SendByte(ReadAddr & 0xFF);

	while (NumByteToRead--) /* while there is data to be read */
	{
		/* Read a byte from the FLASH */
		*pBuffer = SPI_FLASH_SendByte(Dummy_Byte);
		/* Point to the next location where the byte read will be saved */
		pBuffer++;
	}

	/* Deselect the FLASH: Chip Select high */
	SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_WriteEnable		FLASHдʹ��ָ��
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WriteEnable(void)
{
	/* Select the FLASH: Chip Select low */ //1. ʹ��FLASHƬѡ
	SPI_FLASH_CS_LOW();

	/* Send "Write Enable" instruction */ //2. ����WRENָ��
	SPI_FLASH_SendByte(WREN);

	/* Deselect the FLASH: Chip Select high */ //3. ��ֹFLASHƬѡ
	SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_WaitForWriteEnd		�ȴ�д����
* Description    : Polls the status of the Write In Progress (WIP) flag in the  
*                  FLASH's status  register  and  loop  until write  opertaion
*                  has completed.  
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WaitForWriteEnd(void)
{
	u8 FLASH_Status = 0;

	/* Select the FLASH: Chip Select low */ //1. ʹ��FLASHƬѡ
	SPI_FLASH_CS_LOW();

	/* Send "Read Status Register" instruction */ //2. ����RDSRָ��
	SPI_FLASH_SendByte(RDSR);

	/* Loop as long as the memory is busy with a write cycle */
	do
	{
		/* Send a dummy byte to generate the clock needed by the FLASH 
    and put the value of the status register in FLASH_Status variable */
		FLASH_Status = SPI_FLASH_SendByte(Dummy_Byte); //3. ����α�ֽڣ���״̬�Ĵ���ֵ��FLASH_Status

	} while ((FLASH_Status & WIP_Flag) == SET); /* Write in progress */ //FLASH_Status=0,˵��д��

	/* Deselect the FLASH: Chip Select high */ //4. ��ֹFLASHƬѡ
	SPI_FLASH_CS_HIGH();
}

void SPI_FLASH_CS_LOW(void) //Ƭѡ��
{
	if (B_SelCS == CS_Flash1)
		SPI_W25QXX_CS_ON;
	else if (B_SelCS == CS_FMRAM1)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

void SPI_FLASH_CS_HIGH(void) //Ƭѡ��
{
	SPI_W25QXX_CS_OFF;									 //W25Q16
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //FM25L16
}
