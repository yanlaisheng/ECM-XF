/** 
  ******************************************************************************
  * @file    com3_232.c
  * @author  YLS
  * @version V1.00
  * @date    2021-04-03
  * @brief   
	******************************************************************************
	*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __COM3_232_H
#define __COM3_232_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "GlobalConst.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Com3_RcvProcess(void);
void Com3_SlaveSend(void);  // 串口0从机发送
void Com3_MasterSend(void); // 串口0主发送程序　
extern uint16_t CRC16(uint8_t *pCrcData, uint8_t CrcDataLen);

#endif /* __COM3_232_H */

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
