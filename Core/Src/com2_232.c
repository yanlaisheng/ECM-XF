/** 
  ******************************************************************************
  * @file    com2_232.c
  * @author  YLS
  * @version V1.00
  * @date    2021-04-03
  * @brief   
	******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "com2_232.h"
#include "GlobalV_Extern.h" // ȫ�ֱ�������
#include "GlobalConst.h"
#include <stdio.h>
//#include "CRCdata.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
extern void PowerDelay(uint16_t nCount);

/* Private functions ---------------------------------------------------------*/

//���մ������ У�����
void Com2_RcvProcess(void)
{
	uint8_t k, s, i = 0; // ��ʱ����
	uint16_t j;
	//��Ϊ����,ָ������ʱ�䵽��,�Ϳ��Դ�����յ����ַ�����
	// ��û�յ������ַ���ʱ�䳬���趨ʱ�����ԶԽ��ջ�����д�����
	// **********************************rcv_counter<>0,�յ��ַ����ܴ���
	if (Rcv2Counter > 0 && T_NoRcv2Count != SClk1Ms)
	{							 // ���մ������
		T_NoRcv2Count = SClk1Ms; //
		C_NoRcv2Count++;
		if (C_NoRcv2Count > NORCVMAXMS) //
		{
			/* Disable the UART Parity Error Interrupt and RXNE interrupt*/
			//			CLEAR_BIT(huart1->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
			BakRcv2Count = Rcv2Counter; // �� Rcv2Counter ����
			C_NoRcv2Count = 0;			// ��û�н��ռ�����
			//
			if (BakRcv2Count <= RCV2_MAX) // ���ճ�����ȷ,��������.
			{
				// �ӵ�ַ��⣭���յ�����λ����ѯָ��  ���ı�ͨѶ
				if (Rcv2Buffer[0] == Pw_EquipmentNo2)
				{
					j = CRC16(Rcv2Buffer, BakRcv2Count - 2); // CRC У��
					k = j >> 8;
					s = j;
					if (k == Rcv2Buffer[BakRcv2Count - 2] && s == Rcv2Buffer[BakRcv2Count - 1])
					{							// CRCУ����ȷ
						if (Rcv2Buffer[1] == 3) // 03��ȡ���ּĴ���
						{
							B_Com2Cmd03 = 1;
							j = Rcv2Buffer[2];
							w_Com2RegAddr = (j << 8) + Rcv2Buffer[3];
						}
						else if (Rcv2Buffer[1] == 16) // 16Ԥ�ö�Ĵ���
						{
							//							C_ForceSavPar=0;		// ǿ�Ʊ������������=0
							B_Com2Cmd16 = 1;
							j = Rcv2Buffer[2];
							w_Com2RegAddr = (j << 8) + Rcv2Buffer[3];
						}
						else if (Rcv2Buffer[1] == 1) // 01��ȡ��Ȧ״̬
						{
							B_Com2Cmd01 = 1;
						}
						else if (Rcv2Buffer[1] == 6) // 06Ԥ�õ��Ĵ���
						{
							//							C_ForceSavPar=0;		// ǿ�Ʊ������������=0
							B_Com2Cmd06 = 1;
							j = Rcv2Buffer[2];
							w_Com2RegAddr = (j << 8) + Rcv2Buffer[3];
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
			HAL_UART_Receive_IT(&huart2, (uint8_t *)&Tmp_Rxd2Buffer, 1);
			Rcv2Counter = 0; // ׼���´ν��յ����濪ʼ
		}
	}
	if (i > 0)
	{
		for (j = 0; j < 20; j++)
		{
			Rcv2Buffer[j] = 0;
		}
		//		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&Tmp_Rxd2Buffer, 1);
	}
}

void Com2_SlaveSend(void) // ����1�ӻ�����
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
	if (B_Com2Cmd03) // ��ȡ���ּĴ���
	{
		Txd2Buffer[0] = Rcv2Buffer[0];	   // �豸�ӵ�ַPw_EquipmentNo
		Txd2Buffer[1] = Rcv2Buffer[1];	   // ������
		Txd2Buffer[2] = Rcv2Buffer[5] * 2; // Rcv2Buffer[5]=���� ��
		//
		if (w_Com2RegAddr < 0x800) // �����ѯ
		{
			p_wRead = w_ParLst; // PAR��
			p_bMove = Txd2Buffer;
			//
			for (k = 0; k < Rcv2Buffer[5]; k++) // ����ѯ����
			{
				m = *(p_wRead + w_Com2RegAddr + k);
				*(p_bMove + 3 + k * 2) = m >> 8;
				*(p_bMove + 3 + k * 2 + 1) = m;
			}
		}
		else if (w_Com2RegAddr >= 5000) // ��ȡ����趨������������ַ��5000
		{
			p_wRead2 = w_ParLst_Drive; // PAR��
			p_bMove = Txd2Buffer;
			//
			for (k = 0; k < Rcv2Buffer[5]; k++) // ����ѯ����
			{
				m = *(p_wRead2 + w_Com2RegAddr - 5000 + k); //������ַ��5000
				*(p_bMove + 3 + k * 2) = m >> 8;
				*(p_bMove + 3 + k * 2 + 1) = m;
			}
		}
		//
		w_Txd2ChkSum = CRC16(Txd2Buffer, Txd2Buffer[2] + 3);
		Txd2Buffer[Txd2Buffer[2] + 3] = w_Txd2ChkSum >> 8; // /256
		Txd2Buffer[Txd2Buffer[2] + 4] = w_Txd2ChkSum;	   // ��λ�ֽ�
		Txd2Max = Txd2Buffer[2] + 5;
		//
		B_Com2Cmd03 = 0;
		Txd2Counter = 0;
		HAL_UART_Transmit(&huart2, (u8 *)&Txd2Buffer, Txd2Max, 0xffff);
		HAL_GPIO_TogglePin(LED_H65_GPIO_Port, LED_H65_Pin);
	}
	//
	else if (B_Com2Cmd16 || B_Com2Cmd06) // 16Ԥ�ö�Ĵ���
	{
		if (w_Com2RegAddr <= 6000) //YLS 2020.06.23����������Ϳ����޸Ĳ���
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
			if (w_Com2RegAddr >= 45 && w_Com2RegAddr <= 51) // �޸�ʱ��
			{
				w_ModRealTimeNo = w_Com2RegAddr - 45;
				F_ModRealTime = 1;
			}
			//
			if (B_Com2Cmd06) // Ԥ�õ���
			{
				if (w_Com2RegAddr < 0x800)
				{
					m = Rcv2Buffer[4];
					w_ParLst[w_Com2RegAddr] = (m << 8) + Rcv2Buffer[5];
				}
				//-------------------------
				else if (w_Com2RegAddr >= 5000) // �޸��ŷ��������
				{
					m = Rcv2Buffer[4];
					w_ParLst_Drive[w_Com2RegAddr - 5000] = (m << 8) + Rcv2Buffer[5]; //��ַ-5000
				}
				//-------------------------
			}
			else if (B_Com2Cmd16) // Ԥ�ö��
			{
				if (Rcv2Buffer[5] < 100)
				{
					if (w_Com2RegAddr < 0x800)
					{
						p_bGen = Rcv2Buffer;
						p_wTarget = w_ParLst;
						for (k = 0; k < Rcv2Buffer[5]; k++) // Rcv2Buffer[5]=����
						{
							m = *(p_bGen + 7 + k * 2);
							n = *(p_bGen + 7 + k * 2 + 1);
							*(p_wTarget + w_Com2RegAddr + k) = (m << 8) + n;
						}
					}
					else if (w_Com2RegAddr >= 5000) // �޸��ŷ��������
					{
						p_bGen = Rcv2Buffer;
						p_wTarget2 = w_ParLst_Drive;
						for (k = 0; k < Rcv2Buffer[5]; k++) // Rcv2Buffer[5]=����
						{
							m = *(p_bGen + 7 + k * 2);
							n = *(p_bGen + 7 + k * 2 + 1);
							*(p_wTarget2 + w_Com2RegAddr - 5000 + k) = (m << 8) + n;
						}
					}
				}
			}
		}

		// -------------------------
		// ��������
		Txd2Buffer[0] = 2;			   // �豸�ӵ�ַ
		Txd2Buffer[1] = Rcv2Buffer[1]; // ������
		Txd2Buffer[2] = Rcv2Buffer[2]; // ��ʼ��ַ��λ�ֽ�
		Txd2Buffer[3] = Rcv2Buffer[3]; // ��ʼ��ַ��λ�ֽ�
		Txd2Buffer[4] = Rcv2Buffer[4]; // �Ĵ���������λ
		Txd2Buffer[5] = Rcv2Buffer[5]; // �Ĵ���������λ
		if (j == 0)					   // ������ܱ�����Ԥ�ã�����FFFF zcl
		{
			Txd2Buffer[4] = 0xff; // �Ĵ���������λ��Ԥ������
			Txd2Buffer[5] = 0xff; // �Ĵ���������λ��Ԥ������
		}
		w_Txd2ChkSum = CRC16(Txd2Buffer, 6);
		Txd2Buffer[6] = w_Txd2ChkSum >> 8; // /256
		Txd2Buffer[7] = w_Txd2ChkSum;	   // ��λ�ֽ�
		Txd2Max = 8;

		B_Com2Cmd16 = 0;
		B_Com2Cmd06 = 0;
		Txd2Counter = 0;
		HAL_UART_Transmit(&huart2, (u8 *)&Txd2Buffer, Txd2Max, 0xffff);
		//		HAL_GPIO_TogglePin(LED_H64_GPIO_Port,LED_H64_Pin);
	} // 06��16Ԥ�üĴ��� ����
}

/******************* (C) COPYRIGHT 2021 SANLI *****END OF FILE****/
