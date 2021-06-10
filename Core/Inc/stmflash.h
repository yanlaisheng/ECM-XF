#ifndef __STMFLASH_H__
#define __STMFLASH_H__
//#include "sys.h"
#include "LinkTable.h"
#include "typedef.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "stm32_hal_legacy.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////
//用户根据自己的需要设置
#define STM32_FLASH_SIZE 256 //所选STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN 1   //使能FLASH写入(0，不是能;1，使能)
//////////////////////////////////////////////////////////////////////////////////////////////////////

//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 //STM32 FLASH的起始地址

#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000)  /* Base @ of Sector 0, 16 Kbyte */
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08004000)  /* Base @ of Sector 1, 16 Kbyte */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08008000)  /* Base @ of Sector 2, 16 Kbyte */
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000)  /* Base @ of Sector 3, 16 Kbyte */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08010000)  /* Base @ of Sector 4, 64 Kbyte */
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08020000)  /* Base @ of Sector 5, 128 Kbyte */
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08040000)  /* Base @ of Sector 6, 128 Kbyte */
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000)  /* Base @ of Sector 7, 128 Kbyte */
#define ADDR_FLASH_SECTOR_8 ((uint32_t)0x08080000)  /* Base @ of Sector 8, 128 Kbyte */
#define ADDR_FLASH_SECTOR_9 ((uint32_t)0x080A0000)  /* Base @ of Sector 9, 128 Kbyte */
#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbyte */
#define ADDR_FLASH_SECTOR_11 ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbyte */
//FLASH解锁键值

//u16 STMFLASH_ReadHalfWord(u32 faddr);		  //读出半字
u32 STMFLASH_ReadWord(u32 faddr);
void STMFLASH_WriteLenByte(u32 WriteAddr, u32 DataToWrite, u16 Len); //指定地址开始写入指定长度的数据
u32 STMFLASH_ReadLenByte(u32 ReadAddr, u16 Len);                     //指定地址开始读取指定长度数据
void STMFLASH_Write(u32 WriteAddr, u32 *pBuffer, u16 NumToWrite);    //从指定地址开始写入指定长度的数据
void STMFLASH_Read(u32 ReadAddr, u32 *pBuffer, u16 NumToRead);       //从指定地址开始读出指定长度的数据

//测试写入
void Test_Write(u32 WriteAddr, u32 WriteData);

uint32_t GetSector(uint32_t Address);

#endif
