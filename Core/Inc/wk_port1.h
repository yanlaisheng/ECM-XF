/**
  ******************************************************************************

	******************************************************************************
	*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __WK_PORT1_H
#define __WK_PORT1_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "GlobalConst.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void WK_Com1_RcvProcess(void);
void WK_Com1_SlaveSend(void);
extern uint16_t CRC16(uint8_t *pCrcData, uint8_t CrcDataLen);
#endif /* __WK_PORT1_H */

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
