/** 
  ******************************************************************************
  * @file    com2_232.c
  * @author  YLS
  * @version V1.00
  * @date    2021-04-03
  * @brief   
	******************************************************************************
	*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __COM2_232_H
#define __COM2_232_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "GlobalConst.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Com2_RcvProcess(void);
void Com2_SlaveSend(void);  // ����0�ӻ�����
void Com2_MasterSend(void); // ����0�����ͳ���
extern uint16_t CRC16(uint8_t *pCrcData, uint8_t CrcDataLen);

#endif /* __COM2_232_H */

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
