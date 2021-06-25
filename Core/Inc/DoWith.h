/**
  ******************************************************************************
  * @file    DoWith.h
  * @author  ChengLei Zhou  - 周成磊
  * @version V1.27
  * @date    2014-01-03
  * @brief   数字量输入检测，数字量输出,模拟量输入检测，模拟量输出,其他本机操作
	******************************************************************************
	*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __DOWITH_H
#define __DOWITH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "typedef.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void ReadWriteRealTime(void); // 读写实时时钟 ISL1208
void delay(void);             // 延时
void Variable_Init(void);     // 变量初始化
void ParLst_Init(void);       // RAM中参数表列 初始化 (读出)
void ParArrayRead_Word(uint32_t *p_Top, uc32 *p_Base, uint w_ReadSize);
void FilterDI(void); // 过滤开关量输入
void DigitalIn(void);
//void DOConfigValue(uchar DOValue,uchar DO_BitNo);			// DO_BitNo:DO 位号
void ParLimit(void); // 参数限制
//void DoWith(void);					// 一些数据,记录的处理
void Manual_Control(void); //手动控制启停

void ParArrayWrite(uint16_t *p_Top, uint16_t *p_Base, uint16_t w_WriteSize);
void ParArrayRead(uint16_t *p_Top, uc16 *p_Base, uint16_t w_ReadSize);
void Boot_ParLst(void);       // 初始化设定参数
void SavePar_Prompt(void);    // 保存参数+状态提示
void ForceTime_SavePar(void); // 强制保存参数

void Time_Output(void); // 软件时钟输出	 2008.10.21
void EquipStatus(void); // 设备状态
void KglStatus(void);   // 开关量状态

void SPI_FMRAM_BufferRead(u8 *pBuffer, u16 ReadAddr, u16 NumByteToRead);
void SPI_FMRAM_BufferWrite(u8 *pBuffer, u16 WriteAddr, u16 NumByteToWrite);
void W25qxx_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
void W25QXX_Write_WithErase(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite);
void read_show_Fram(u16 start_addr, u16 dataLen);
void read_show_exFLASH(u16 start_addr, u16 dataLen);
void PosCMD_SaveTo_exFLASH(void);

void MEM_If_Init_FS(void);
void MEM_If_DeInit_FS(void);
uint16_t MEM_If_Erase_FS(uint32_t start_Add);
uint16_t MEM_If_Write_FS(uint8_t *src, uint8_t *dest, uint32_t Len);
uint8_t *MEM_If_Read_FS(uint8_t *src, uint8_t *dest, uint32_t Len);

#endif /* __DOWITH_H */

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
