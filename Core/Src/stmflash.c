#include "stmflash.h"

//��ȡָ����ַ�İ���(16λ����)
//faddr:����ַ(�˵�ַ����Ϊ2�ı���!!)
//����ֵ:��Ӧ����.
//u16 STMFLASH_ReadHalfWord(u32 faddr)
//{
//	return *(vu16*)faddr;
//}

//��˫�ֶ�������4���ֽ�
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32 *)faddr;
}
#if STM32_FLASH_WREN //���ʹ����д
//������д��
//WriteAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
// void STMFLASH_Write_NoCheck(u32 WriteAddr, u32 *pBuffer, u16 NumToWrite)
// {
// 	u16 i;
// 	for (i = 0; i < NumToWrite; i++)
// 	{
// 		FLASH_ProgramWord(WriteAddr, pBuffer[i]);
// 		WriteAddr += 4; //��ַ����2.
// 	}
// }
//��ָ����ַ��ʼд��ָ�����ȵ�����
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
//pBuffer:����ָ��
//NumToWrite:����(16λ)��(����Ҫд���16λ���ݵĸ���.)
// #if STM32_FLASH_SIZE < 256
// #define STM_SECTOR_SIZE 1024 //�ֽ�
// #else
// #define STM_SECTOR_SIZE 2048
// #endif
// u32 STMFLASH_BUF[STM_SECTOR_SIZE / 4]; //�����2K�ֽ�
// void STMFLASH_Write(u32 WriteAddr, u32 *pBuffer, u16 NumToWrite)
// {
// 	//��ʼ��FLASH_EraseInitTypeDef
// 	FLASH_EraseInitTypeDef f;
// 	uint32_t SectorError = 0;
// 	u32 secpos;	   //������ַ
// 	u16 secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
// 	u16 secremain; //������ʣ���ַ(16λ�ּ���)
// 	u16 i;
// 	u32 offaddr; //ȥ��0X08000000��ĵ�ַ

// 	//1������FLASH
// 	HAL_FLASH_Unlock();
// 	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
// 						   FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
// 	offaddr = WriteAddr - STM32_FLASH_BASE;	  //ʵ��ƫ�Ƶ�ַ.
// 	secpos = offaddr / STM_SECTOR_SIZE;		  //������ַ  0~127 for STM32F103RBT6
// 	secoff = (offaddr % STM_SECTOR_SIZE) / 4; //�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
// 	secremain = STM_SECTOR_SIZE / 4 - secoff; //����ʣ��ռ��С
// 	if (NumToWrite <= secremain)
// 		secremain = NumToWrite; //�����ڸ�������Χ
// 	while (1)
// 	{
// 		STMFLASH_Read(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 4); //������������������
// 		for (i = 0; i < secremain; i++)																   //У������
// 		{
// 			if (STMFLASH_BUF[secoff + i] != 0XFFFFFFFF)
// 				break; //��Ҫ����
// 		}
// 		if (i < secremain) //��Ҫ����
// 		{
// 			// FLASH_ErasePage(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE); //�����������

// 			for (i = 0; i < secremain; i++) //����
// 			{
// 				STMFLASH_BUF[i + secoff] = pBuffer[i];
// 			}
// 			STMFLASH_Write_NoCheck(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 4); //д����������
// 		}
// 		else
// 			STMFLASH_Write_NoCheck(WriteAddr, pBuffer, secremain); //д�Ѿ������˵�,ֱ��д������ʣ������.
// 		if (NumToWrite == secremain)
// 			break; //д�������
// 		else	   //д��δ����
// 		{
// 			secpos++;				  //������ַ��1
// 			secoff = 0;				  //ƫ��λ��Ϊ0
// 			pBuffer += secremain;	  //ָ��ƫ��
// 			WriteAddr += (secremain); //д��ַƫ��
// 			NumToWrite -= secremain;  //�ֽ�(16λ)���ݼ�
// 			if (NumToWrite > (STM_SECTOR_SIZE / 4))
// 				secremain = STM_SECTOR_SIZE / 4; //��һ����������д����
// 			else
// 				secremain = NumToWrite; //��һ����������д����
// 		}
// 	};

// 	//4����סFLASH
// 	HAL_FLASH_Lock();
// }
#endif

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:˫��(32λ)��
void STMFLASH_Read(u32 ReadAddr, u32 *pBuffer, u16 NumToRead)
{
	u16 i;
	for (i = 0; i < NumToRead; i++)
	{
		pBuffer[i] = STMFLASH_ReadWord(ReadAddr); //��ȡ2����.
		ReadAddr += 4;							  //ƫ��4���ֽ�.
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//WriteAddr:��ʼ��ַ
//WriteData:Ҫд�������
void Test_Write(u32 WriteAddr, u32 WriteData)
{
	// STMFLASH_Write(WriteAddr, &WriteData, 1); //д��һ����
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

//ֻɾ��start_Add���ڵĵ�ǰ����
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