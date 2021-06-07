/** 
  ******************************************************************************
  * @file    com1_232.c
  * @author  YLS
  * @version V1.00
  * @date    2021-04-03
  * @brief   
	******************************************************************************
	*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __COM1_232_H
#define __COM1_232_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "GlobalConst.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Com1_RcvProcess(void);
void Com1_SlaveSend(void);  // ����0�ӻ�����
void Com1_MasterSend(void); // ����0�����ͳ���
uint16_t CRC16(uint8_t *pCrcData, uint8_t CrcDataLen);

#endif /* __COM1_232_H */

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
