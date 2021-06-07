/**
  ******************************************************************************
  * @file    DoWith.h
  * @author  ChengLei Zhou  - �ܳ���
  * @version V1.27
  * @date    2014-01-03
  * @brief   �����������⣬���������,ģ���������⣬ģ�������,������������
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
void ReadWriteRealTime(void); // ��дʵʱʱ�� ISL1208
void delay(void);             // ��ʱ
void Variable_Init(void);     // ������ʼ��
void ParLst_Init(void);       // RAM�в������� ��ʼ�� (����)
void ParArrayRead_Word(uint32_t *p_Top, uc32 *p_Base, uint w_ReadSize);
void FilterDI(void); // ���˿���������
void DigitalIn(void);
//void DOConfigValue(uchar DOValue,uchar DO_BitNo);			// DO_BitNo:DO λ��
void ParLimit(void); // ��������
//void DoWith(void);					// һЩ����,��¼�Ĵ���
void Manual_Control(void); //�ֶ�������ͣ

void ParArrayWrite(uint16_t *p_Top, uint16_t *p_Base, uint16_t w_WriteSize);
void ParArrayRead(uint16_t *p_Top, uc16 *p_Base, uint16_t w_ReadSize);
void Boot_ParLst(void);    // ��ʼ���趨����
void SavePar_Prompt(void); // �������+״̬��ʾ
void ForceSavePar(void);   // ǿ�Ʊ������

void Time_Output(void);       // ���ʱ�����	 2008.10.21
void EquipStatus(void);       // �豸״̬
void ReadWriteRealTime(void); // ��дʵʱʱ�� ISL12087
void KglStatus(void);         // ������״̬

#endif /* __DOWITH_H */

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
