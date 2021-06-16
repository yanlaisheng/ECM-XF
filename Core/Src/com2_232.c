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
#include "GlobalV_Extern.h" // 全局变量声明
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

//接收处理程序 校验程序
void Com2_RcvProcess(void)
{
	uint8_t k, s, i = 0; // 临时变量
	uint16_t j;
	//作为主机,指定接收时间到了,就可以处理接收到的字符串了
	// 在没收到串行字符的时间超过设定时，可以对接收缓存进行处理了
	// **********************************rcv_counter<>0,收到字符才能处理
	if (Rcv2Counter > 0 && T_NoRcv2Count != SClk1Ms)
	{							 // 接收处理过程
		T_NoRcv2Count = SClk1Ms; //
		C_NoRcv2Count++;
		if (C_NoRcv2Count > NORCVMAXMS) //
		{
			/* Disable the UART Parity Error Interrupt and RXNE interrupt*/
			//			CLEAR_BIT(huart1->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
			BakRcv2Count = Rcv2Counter; // 把 Rcv2Counter 保存
			C_NoRcv2Count = 0;			// 清没有接收计数器
			//
			if (BakRcv2Count <= RCV2_MAX) // 接收长度正确,继续处理.
			{
				// 从地址检测－接收到的上位机查询指令  与文本通讯
				if (Rcv2Buffer[0] == Pw_EquipmentNo2)
				{
					j = CRC16(Rcv2Buffer, BakRcv2Count - 2); // CRC 校验
					k = j >> 8;
					s = j;
					if (k == Rcv2Buffer[BakRcv2Count - 2] && s == Rcv2Buffer[BakRcv2Count - 1])
					{							// CRC校验正确
						if (Rcv2Buffer[1] == 3) // 03读取保持寄存器
						{
							B_Com2Cmd03 = 1;
							j = Rcv2Buffer[2];
							w_Com2RegAddr = (j << 8) + Rcv2Buffer[3];
						}
						else if (Rcv2Buffer[1] == 16) // 16预置多寄存器
						{
							//							C_ForceSavPar=0;		// 强制保存参数计数器=0
							B_Com2Cmd16 = 1;
							j = Rcv2Buffer[2];
							w_Com2RegAddr = (j << 8) + Rcv2Buffer[3];
						}
						else if (Rcv2Buffer[1] == 1) // 01读取线圈状态
						{
							B_Com2Cmd01 = 1;
						}
						else if (Rcv2Buffer[1] == 6) // 06预置单寄存器
						{
							//							C_ForceSavPar=0;		// 强制保存参数计数器=0
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
			Rcv2Counter = 0; // 准备下次接收到缓存开始
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

void Com2_SlaveSend(void) // 串口1从机发送
{
	uint16_t m, n;
	uint8_t j = 0, k;
	uint16_t *p_wRead;
	s32 *p_wRead2;
	uint8_t *p_bMove;
	uint8_t *p_bGen;
	uint16_t *p_wTarget; // 指向目标字符串　xdata zcl
	s32 *p_wTarget2;

	//
	if (B_Com2Cmd03) // 读取保持寄存器
	{
		Txd2Buffer[0] = Rcv2Buffer[0];	   // 设备从地址Pw_EquipmentNo
		Txd2Buffer[1] = Rcv2Buffer[1];	   // 功能码
		Txd2Buffer[2] = Rcv2Buffer[5] * 2; // Rcv2Buffer[5]=字数 　
		//
		if (w_Com2RegAddr < 0x800) // 常规查询
		{
			p_wRead = w_ParLst; // PAR区
			p_bMove = Txd2Buffer;
			//
			for (k = 0; k < Rcv2Buffer[5]; k++) // 填充查询内容
			{
				m = *(p_wRead + w_Com2RegAddr + k);
				*(p_bMove + 3 + k * 2) = m >> 8;
				*(p_bMove + 3 + k * 2 + 1) = m;
			}
		}
		else if (w_Com2RegAddr >= 5000) // 读取电机设定参数，参数地址减5000
		{
			p_wRead2 = w_ParLst_Drive; // PAR区
			p_bMove = Txd2Buffer;
			//
			for (k = 0; k < Rcv2Buffer[5]; k++) // 填充查询内容
			{
				m = *(p_wRead2 + w_Com2RegAddr - 5000 + k); //参数地址减5000
				*(p_bMove + 3 + k * 2) = m >> 8;
				*(p_bMove + 3 + k * 2 + 1) = m;
			}
		}
		//
		w_Txd2ChkSum = CRC16(Txd2Buffer, Txd2Buffer[2] + 3);
		Txd2Buffer[Txd2Buffer[2] + 3] = w_Txd2ChkSum >> 8; // /256
		Txd2Buffer[Txd2Buffer[2] + 4] = w_Txd2ChkSum;	   // 低位字节
		Txd2Max = Txd2Buffer[2] + 5;
		//
		B_Com2Cmd03 = 0;
		Txd2Counter = 0;
		HAL_UART_Transmit(&huart2, (u8 *)&Txd2Buffer, Txd2Max, 0xffff);
		HAL_GPIO_TogglePin(LED_H65_GPIO_Port, LED_H65_Pin);
	}
	//
	else if (B_Com2Cmd16 || B_Com2Cmd06) // 16预置多寄存器
	{
		if (w_Com2RegAddr <= 6000) //YLS 2020.06.23，不用密码就可以修改参数
		{
			j = 1;
		}
		// 需要口令才可以修改的参数
		else if (Pw_ModPar == 2000) // 需要先把Pw_ModPar 修改成规定值，才能修改的参数
		{
			j = 1;
		}

		// 修改参数单元
		if (j)
		{
			if (w_Com2RegAddr >= 45 && w_Com2RegAddr <= 51) // 修改时间
			{
				w_ModRealTimeNo = w_Com2RegAddr - 45;
				F_ModRealTime = 1;
			}
			//
			if (B_Com2Cmd06) // 预置单个
			{
				if (w_Com2RegAddr < 0x800)
				{
					m = Rcv2Buffer[4];
					w_ParLst[w_Com2RegAddr] = (m << 8) + Rcv2Buffer[5];
				}
				//-------------------------
				else if (w_Com2RegAddr >= 5000) // 修改伺服电机参数
				{
					m = Rcv2Buffer[4];
					w_ParLst_Drive[w_Com2RegAddr - 5000] = (m << 8) + Rcv2Buffer[5]; //地址-5000
				}
				//-------------------------
			}
			else if (B_Com2Cmd16) // 预置多个
			{
				if (Rcv2Buffer[5] < 100)
				{
					if (w_Com2RegAddr < 0x800)
					{
						p_bGen = Rcv2Buffer;
						p_wTarget = w_ParLst;
						for (k = 0; k < Rcv2Buffer[5]; k++) // Rcv2Buffer[5]=字数
						{
							m = *(p_bGen + 7 + k * 2);
							n = *(p_bGen + 7 + k * 2 + 1);
							*(p_wTarget + w_Com2RegAddr + k) = (m << 8) + n;
						}
					}
					else if (w_Com2RegAddr >= 5000) // 修改伺服电机参数
					{
						p_bGen = Rcv2Buffer;
						p_wTarget2 = w_ParLst_Drive;
						for (k = 0; k < Rcv2Buffer[5]; k++) // Rcv2Buffer[5]=字数
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
		// 返回数据
		Txd2Buffer[0] = 2;			   // 设备从地址
		Txd2Buffer[1] = Rcv2Buffer[1]; // 功能码
		Txd2Buffer[2] = Rcv2Buffer[2]; // 开始地址高位字节
		Txd2Buffer[3] = Rcv2Buffer[3]; // 开始地址低位字节
		Txd2Buffer[4] = Rcv2Buffer[4]; // 寄存器数量高位
		Txd2Buffer[5] = Rcv2Buffer[5]; // 寄存器数量低位
		if (j == 0)					   // 如果不能被正常预置，返回FFFF zcl
		{
			Txd2Buffer[4] = 0xff; // 寄存器数量高位、预置数据
			Txd2Buffer[5] = 0xff; // 寄存器数量低位、预置数据
		}
		w_Txd2ChkSum = CRC16(Txd2Buffer, 6);
		Txd2Buffer[6] = w_Txd2ChkSum >> 8; // /256
		Txd2Buffer[7] = w_Txd2ChkSum;	   // 低位字节
		Txd2Max = 8;

		B_Com2Cmd16 = 0;
		B_Com2Cmd06 = 0;
		Txd2Counter = 0;
		HAL_UART_Transmit(&huart2, (u8 *)&Txd2Buffer, Txd2Max, 0xffff);
		//		HAL_GPIO_TogglePin(LED_H64_GPIO_Port,LED_H64_Pin);
	} // 06、16预置寄存器 结束
}

/******************* (C) COPYRIGHT 2021 SANLI *****END OF FILE****/
