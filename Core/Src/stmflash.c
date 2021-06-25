#include "stmflash.h"

//读取指定地址的半字(16位数据)
//faddr:读地址(此地址必须为2的倍数!!)
//返回值:对应数据.
//u16 STMFLASH_ReadHalfWord(u32 faddr)
//{
//	return *(vu16*)faddr;
//}

//按双字读，即读4个字节
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32 *)faddr;
}
#if STM32_FLASH_WREN //如果使能了写
//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
// void STMFLASH_Write_NoCheck(u32 WriteAddr, u32 *pBuffer, u16 NumToWrite)
// {
// 	u16 i;
// 	for (i = 0; i < NumToWrite; i++)
// 	{
// 		FLASH_ProgramWord(WriteAddr, pBuffer[i]);
// 		WriteAddr += 4; //地址增加2.
// 	}
// }
//从指定地址开始写入指定长度的数据
//WriteAddr:起始地址(此地址必须为2的倍数!!)
//pBuffer:数据指针
//NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
// #if STM32_FLASH_SIZE < 256
// #define STM_SECTOR_SIZE 1024 //字节
// #else
// #define STM_SECTOR_SIZE 2048
// #endif
// u32 STMFLASH_BUF[STM_SECTOR_SIZE / 4]; //最多是2K字节
// void STMFLASH_Write(u32 WriteAddr, u32 *pBuffer, u16 NumToWrite)
// {
// 	//初始化FLASH_EraseInitTypeDef
// 	FLASH_EraseInitTypeDef f;
// 	uint32_t SectorError = 0;
// 	u32 secpos;	   //扇区地址
// 	u16 secoff;	   //扇区内偏移地址(16位字计算)
// 	u16 secremain; //扇区内剩余地址(16位字计算)
// 	u16 i;
// 	u32 offaddr; //去掉0X08000000后的地址

// 	//1、解锁FLASH
// 	HAL_FLASH_Unlock();
// 	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
// 						   FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
// 	offaddr = WriteAddr - STM32_FLASH_BASE;	  //实际偏移地址.
// 	secpos = offaddr / STM_SECTOR_SIZE;		  //扇区地址  0~127 for STM32F103RBT6
// 	secoff = (offaddr % STM_SECTOR_SIZE) / 4; //在扇区内的偏移(2个字节为基本单位.)
// 	secremain = STM_SECTOR_SIZE / 4 - secoff; //扇区剩余空间大小
// 	if (NumToWrite <= secremain)
// 		secremain = NumToWrite; //不大于该扇区范围
// 	while (1)
// 	{
// 		STMFLASH_Read(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 4); //读出整个扇区的内容
// 		for (i = 0; i < secremain; i++)																   //校验数据
// 		{
// 			if (STMFLASH_BUF[secoff + i] != 0XFFFFFFFF)
// 				break; //需要擦除
// 		}
// 		if (i < secremain) //需要擦除
// 		{
// 			// FLASH_ErasePage(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE); //擦除这个扇区

// 			for (i = 0; i < secremain; i++) //复制
// 			{
// 				STMFLASH_BUF[i + secoff] = pBuffer[i];
// 			}
// 			STMFLASH_Write_NoCheck(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 4); //写入整个扇区
// 		}
// 		else
// 			STMFLASH_Write_NoCheck(WriteAddr, pBuffer, secremain); //写已经擦除了的,直接写入扇区剩余区间.
// 		if (NumToWrite == secremain)
// 			break; //写入结束了
// 		else	   //写入未结束
// 		{
// 			secpos++;				  //扇区地址增1
// 			secoff = 0;				  //偏移位置为0
// 			pBuffer += secremain;	  //指针偏移
// 			WriteAddr += (secremain); //写地址偏移
// 			NumToWrite -= secremain;  //字节(16位)数递减
// 			if (NumToWrite > (STM_SECTOR_SIZE / 4))
// 				secremain = STM_SECTOR_SIZE / 4; //下一个扇区还是写不完
// 			else
// 				secremain = NumToWrite; //下一个扇区可以写完了
// 		}
// 	};

// 	//4、锁住FLASH
// 	HAL_FLASH_Lock();
// }
#endif

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:双字(32位)数
void STMFLASH_Read(u32 ReadAddr, u32 *pBuffer, u16 NumToRead)
{
	u16 i;
	for (i = 0; i < NumToRead; i++)
	{
		pBuffer[i] = STMFLASH_ReadWord(ReadAddr); //读取2个字.
		ReadAddr += 4;							  //偏移4个字节.
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//WriteAddr:起始地址
//WriteData:要写入的数据
void Test_Write(u32 WriteAddr, u32 WriteData)
{
	// STMFLASH_Write(WriteAddr, &WriteData, 1); //写入一个字
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void MEM_If_Init_FS(void)
{

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
						   FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

void MEM_If_DeInit_FS(void)
{
	HAL_FLASH_Lock();
}

//只删除start_Add所在的当前扇区
uint16_t MEM_If_Erase_FS(uint32_t start_Add)
{
	/* USER CODE BEGIN 3 */
	uint32_t UserStartSector;
	uint32_t SectorError;
	FLASH_EraseInitTypeDef pEraseInit;

	/* Unlock the Flash to enable the flash control register access *************/
	MEM_If_Init_FS();

	/* Get the sector where start the user flash area */
	UserStartSector = GetSector(start_Add);

	pEraseInit.TypeErase = TYPEERASE_SECTORS;
	pEraseInit.Sector = UserStartSector;
	pEraseInit.NbSectors = 1;
	pEraseInit.VoltageRange = VOLTAGE_RANGE_3;

	if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK)
	{
		/* Error occurred while page erase */
		return (HAL_ERROR); // Return 1
	}
	return (HAL_OK); //Return 0

	/* USER CODE END 3 */
}

/**
  * @brief  Gets the sector of a given address
  * @param  Address: Flash address
  * @retval The sector of a given address
  */
uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;

	if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_SECTOR_0;
	}
	else if ((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_SECTOR_1;
	}
	else if ((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_SECTOR_2;
	}
	else if ((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_SECTOR_3;
	}
	else if ((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_SECTOR_4;
	}
	else if ((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_SECTOR_5;
	}
	else if ((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_SECTOR_6;
	}
	else if ((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
	{
		sector = FLASH_SECTOR_7;
	}
	else if ((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
	{
		sector = FLASH_SECTOR_8;
	}
	else if ((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
	{
		sector = FLASH_SECTOR_9;
	}
	else if ((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
	{
		sector = FLASH_SECTOR_10;
	}
	else
	{
		sector = FLASH_SECTOR_11;
	}

	return sector;
}

uint16_t MEM_If_Write_FS(uint8_t *src, uint8_t *dest, uint32_t Len)
{
	/* USER CODE BEGIN 3 */
	uint32_t i = 0;

	for (i = 0; i < Len; i += 4)
	{
		/* Device voltage range supposed to be [2.7V to 3.6V], the operation will
           be done by byte */
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(dest + i), *(uint32_t *)(src + i)) == HAL_OK)
		{
			/* Check the written value */
			if (*(uint32_t *)(src + i) != *(uint32_t *)(dest + i))
			{
				/* Flash content doesn't match SRAM content */
				return 2;
			}
		}
		else
		{
			/* Error occurred while writing data in Flash memory */
			return 1;
		}
	}
	return (HAL_OK);
	/* USER CODE END 3 */
}

uint8_t *MEM_If_Read_FS(uint8_t *src, uint8_t *dest, uint32_t Len)
{
	/* Return a valid address to avoid HardFault */
	/* USER CODE BEGIN 4 */
	uint32_t i = 0;
	uint8_t *psrc = src;

	for (i = 0; i < Len; i++)
	{
		dest[i] = *psrc++;
	}
	return HAL_OK;

	/* USER CODE END 4 */
}