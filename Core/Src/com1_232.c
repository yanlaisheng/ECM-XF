/** 
  ******************************************************************************
  * @file    com1_232.c
  * @author  YLS
  * @version V1.00
  * @date    2021-04-03
  * @brief   
	******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "com1_232.h"
#include "GlobalV_Extern.h" // ȫ�ֱ�������
//#include "GlobalConst.h"
#include <stdio.h>
#include "CRCdata.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

extern UART_HandleTypeDef huart1, huart2;

/* Private function prototypes -----------------------------------------------*/
extern void PowerDelay(uint16_t nCount);

/* Private functions ---------------------------------------------------------*/

//���մ������ У�����
void Com1_RcvProcess(void)
{
	uint8_t k, s, i = 0; // ��ʱ����
	uint16_t j;
	//��Ϊ����,ָ������ʱ�䵽��,�Ϳ��Դ�����յ����ַ�����
	// ��û�յ������ַ���ʱ�䳬���趨ʱ�����ԶԽ��ջ�����д�����
	// **********************************rcv_counter<>0,�յ��ַ����ܴ���
	if (Rcv1Counter > 0 && T_NoRcv1Count != SClk1Ms)
	{							 // ���մ������
		T_NoRcv1Count = SClk1Ms; //
		C_NoRcv1Count++;
		if (C_NoRcv1Count > NORCVMAXMS) //
		{
			/* Disable the UART Parity Error Interrupt and RXNE interrupt*/
			//			CLEAR_BIT(huart1->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
			BakRcv1Count = Rcv1Counter; // �� Rcv1Counter ����
			C_NoRcv1Count = 0;			// ��û�н��ռ�����
			//
			if (BakRcv1Count <= RCV1_MAX) // ���ճ�����ȷ,��������.
			{
				// �ӵ�ַ��⣭���յ�����λ����ѯָ��  ���ı�ͨѶ
				if (Rcv1Buffer[0] == Pw_EquipmentNo1)
				{
					j = CRC16(Rcv1Buffer, BakRcv1Count - 2); // CRC У��
					k = j >> 8;
					s = j;
					if (k == Rcv1Buffer[BakRcv1Count - 2] && s == Rcv1Buffer[BakRcv1Count - 1])
					{							// CRCУ����ȷ
						if (Rcv1Buffer[1] == 3) // 03��ȡ���ּĴ���
						{
							B_Com1Cmd03 = 1;
							j = Rcv1Buffer[2];
							w_Com1RegAddr = (j << 8) + Rcv1Buffer[3];
						}
						else if (Rcv1Buffer[1] == 16) // 16Ԥ�ö�Ĵ���
						{
							//							C_ForceSavPar=0;		// ǿ�Ʊ������������=0
							B_Com1Cmd16 = 1;
							j = Rcv1Buffer[2];
							w_Com1RegAddr = (j << 8) + Rcv1Buffer[3];
						}
						else if (Rcv1Buffer[1] == 1) // 01��ȡ��Ȧ״̬
						{
							B_Com1Cmd01 = 1;
						}
						else if (Rcv1Buffer[1] == 6) // 06Ԥ�õ��Ĵ���
						{
							//							C_ForceSavPar=0;		// ǿ�Ʊ������������=0
							B_Com1Cmd06 = 1;
							j = Rcv1Buffer[2];
							w_Com1RegAddr = (j << 8) + Rcv1Buffer[3];
						}

						else
							i = 1;
					}
					else
						i = 2;
				}
			}
			else
				i = 4;
			//			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
			HAL_UART_Receive_IT(&huart1, (uint8_t *)&Tmp_Rxd1Buffer, 1);
			Rcv1Counter = 0; // ׼���´ν��յ����濪ʼ
		}
	}
	if (i > 0)
	{
		for (j = 0; j < 20; j++)
		{
			Rcv1Buffer[j] = 0;
		}
		//		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&Tmp_Rxd1Buffer, 1);
	}
}

void Com1_SlaveSend(void) // ����1�ӻ�����
{
	uint16_t m, n;
	uint8_t j = 0, k;
	uint16_t *p_wRead;
	s32 *p_wRead2;
	uint8_t *p_bMove;
	uint8_t *p_bGen;
	uint16_t *p_wTarget; // ָ��Ŀ���ַ�����xdata zcl
	s32 *p_wTarget2;

	//
	if (B_Com1Cmd03) // ��ȡ���ּĴ���
	{
		Txd1Buffer[0] = Rcv1Buffer[0];	   // �豸�ӵ�ַPw_EquipmentNo
		Txd1Buffer[1] = Rcv1Buffer[1];	   // ������
		Txd1Buffer[2] = Rcv1Buffer[5] * 2; // Rcv1Buffer[5]=���� ��
		//
		if (w_Com1RegAddr < 0x800 && Pw_ComBufType == 1) // �����ѯ Pw_ComBufType==1 2016.4.21
		{
			p_wRead = w_ParLst; // PAR��
			p_bMove = Txd1Buffer;
			//
			for (k = 0; k < Rcv1Buffer[5]; k++) // ����ѯ����
			{
				m = *(p_wRead + w_Com1RegAddr + k);
				*(p_bMove + 3 + k * 2) = m >> 8;
				*(p_bMove + 3 + k * 2 + 1) = m;
			}
		}
		else if (w_Com1RegAddr >= 5000 && Pw_ComBufType == 1) // ��ȡ����趨������������ַ��5000
		{
			p_wRead2 = w_ParLst_Drive; // PAR��
			p_bMove = Txd1Buffer;
			//
			for (k = 0; k < Rcv1Buffer[5]; k++) // ����ѯ����
			{
				m = *(p_wRead2 + w_Com1RegAddr - 5000 + k); //������ַ��5000
				*(p_bMove + 3 + k * 2) = m >> 8;
				*(p_bMove + 3 + k * 2 + 1) = m;
			}
		}
		//
		w_Txd1ChkSum = CRC16(Txd1Buffer, Txd1Buffer[2] + 3);
		Txd1Buffer[Txd1Buffer[2] + 3] = w_Txd1ChkSum >> 8; // /256
		Txd1Buffer[Txd1Buffer[2] + 4] = w_Txd1ChkSum;	   // ��λ�ֽ�
		Txd1Max = Txd1Buffer[2] + 5;
		//
		B_Com1Cmd03 = 0;
		Txd1Counter = 0;
		HAL_UART_Transmit(&huart1, (u8 *)&Txd1Buffer, Txd1Max, 0xffff);
		HAL_GPIO_TogglePin(LED_H64_GPIO_Port, LED_H64_Pin);
	}
	//
	else if (B_Com1Cmd16 || B_Com1Cmd06) // 16Ԥ�ö�Ĵ���
	{
		if (w_Com1RegAddr <= 6000) //YLS 2020.06.23����������Ϳ����޸Ĳ���
		{
			j = 1;
		}
		// ��Ҫ����ſ����޸ĵĲ���
		else if (Pw_ModPar == 2000) // ��Ҫ�Ȱ�Pw_ModPar �޸ĳɹ涨ֵ�������޸ĵĲ���
		{
			j = 1;
		}

		// �޸Ĳ�����Ԫ
		if (j)
		{
			if (w_Com1RegAddr >= 45 && w_Com1RegAddr <= 51) // �޸�ʱ��
			{
				w_ModRealTimeNo = w_Com1RegAddr - 45;
				F_ModRealTime = 1;
			}
			//
			if (B_Com1Cmd06) // Ԥ�õ���
			{
				if (w_Com1RegAddr < 0x800)
				{
					m = Rcv1Buffer[4];
					w_ParLst[w_Com1RegAddr] = (m << 8) + Rcv1Buffer[5];
				}
				//-------------------------
				else if (w_Com1RegAddr >= 5000) // �޸��ŷ��������
				{
					m = Rcv1Buffer[4];
					w_ParLst_Drive[w_Com1RegAddr - 5000] = (m << 8) + Rcv1Buffer[5]; //��ַ-5000
				}
				//-------------------------
			}
			else if (B_Com1Cmd16) // Ԥ�ö��
			{
				if (Rcv1Buffer[5] < 100)
				{
					if (w_Com1RegAddr < 0x800)
					{
						p_bGen = Rcv1Buffer;
						p_wTarget = w_ParLst;
						for (k = 0; k < Rcv1Buffer[5]; k++) // Rcv1Buffer[5]=����
						{
							m = *(p_bGen + 7 + k * 2);
							n = *(p_bGen + 7 + k * 2 + 1);
							*(p_wTarget + w_Com1RegAddr + k) = (m << 8) + n;
						}
					}
					else if (w_Com1RegAddr >= 5000) // �޸��ŷ��������
					{
						p_bGen = Rcv1Buffer;
						p_wTarget2 = w_ParLst_Drive;
						for (k = 0; k < Rcv1Buffer[5]; k++) // Rcv1Buffer[5]=����
						{
							m = *(p_bGen + 7 + k * 2);
							n = *(p_bGen + 7 + k * 2 + 1);
							*(p_wTarget2 + w_Com1RegAddr - 5000 + k) = (m << 8) + n;
						}
					}
				}
			}
		}

		// -------------------------
		// ��������
		Txd1Buffer[0] = 2;			   // �豸�ӵ�ַ
		Txd1Buffer[1] = Rcv1Buffer[1]; // ������
		Txd1Buffer[2] = Rcv1Buffer[2]; // ��ʼ��ַ��λ�ֽ�
		Txd1Buffer[3] = Rcv1Buffer[3]; // ��ʼ��ַ��λ�ֽ�
		Txd1Buffer[4] = Rcv1Buffer[4]; // �Ĵ���������λ
		Txd1Buffer[5] = Rcv1Buffer[5]; // �Ĵ���������λ
		if (j == 0)					   // ������ܱ�����Ԥ�ã�����FFFF zcl
		{
			Txd1Buffer[4] = 0xff; // �Ĵ���������λ��Ԥ������
			Txd1Buffer[5] = 0xff; // �Ĵ���������λ��Ԥ������
		}
		w_Txd1ChkSum = CRC16(Txd1Buffer, 6);
		Txd1Buffer[6] = w_Txd1ChkSum >> 8; // /256
		Txd1Buffer[7] = w_Txd1ChkSum;	   // ��λ�ֽ�
		Txd1Max = 8;

		B_Com1Cmd16 = 0;
		B_Com1Cmd06 = 0;
		Txd1Counter = 0;
		HAL_UART_Transmit(&huart1, (u8 *)&Txd1Buffer, Txd1Max, 0xffff);
		//		HAL_GPIO_TogglePin(LED_H64_GPIO_Port,LED_H64_Pin);
	} // 06��16Ԥ�üĴ��� ����
}

uint16_t CRC16(uint8_t *pCrcData, uint8_t CrcDataLen)
{
	uint8_t CRC16Hi = 0xff; /* ��CRC�ֽڳ�ʼ�� */
	uint8_t CRC16Lo = 0xff; /* ��CRC�ֽڳ�ʼ��*/
	uint8_t Index = 0;
	uint16_t w_CRC16 = 0;
	while (CrcDataLen--)
	{
		Index = CRC16Hi ^ *pCrcData++;
		CRC16Hi = CRC16Lo ^ CRC_H[Index];
		CRC16Lo = CRC_L[Index];
	}
	w_CRC16 = (CRC16Hi << 8) | CRC16Lo;
	return (w_CRC16);
}

/******************* (C) COPYRIGHT 2021 SANLI *****END OF FILE****/
