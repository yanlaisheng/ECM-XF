/** 
  ******************************************************************************

	******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "wk_port2.h"
#include "GlobalV_Extern.h" // 全局变量声明
// #include "GlobalConst.h"
#include <stdio.h> //加上此句可以用printf
//#include "CRCdata.h"
#include "wk2xxx.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

///* Private functions ---------------------------------------------------------*/

//接收处理程序 校验程序
void WK_Com2_RcvProcess(void)
{
  uint8_t k, s, i = 0; // 临时变量
  uint16_t j;
  unsigned char scr;
  //作为主机,指定接收时间到了,就可以处理接收到的字符串了
  // 在没收到串行字符的时间超过设定时，可以对接收缓存进行处理了
  // **********************************rcv_counter<>0,收到字符才能处理
  if (WK_Rcv2Counter > 0 && T_WK_NoRcv2Count != SClk1Ms)
  {                             // 接收处理过程
    T_WK_NoRcv2Count = SClk1Ms; //
    C_WK_NoRcv2Count++;
    if (C_WK_NoRcv2Count > NORCVMAXMS) //
    {
      /* Disable the UART Parity Error Interrupt and RXNE interrupt*/
      //			CLEAR_BIT(huart1->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
      //禁止子串口的接收使能
      scr = Wk2xxxReadReg(WK2XXX_PORT2, WK2XXX_SCR);
      scr &= ~WK2XXX_RXEN;
      Wk2xxxWriteReg(WK2XXX_PORT2, WK2XXX_SCR, scr);

      WK_BakRcv2Count = WK_Rcv2Counter; // 把 WK_RRcv2Counter 保存
      C_WK_NoRcv2Count = 0;             // 清没有接收计数器
      //
      if (WK_BakRcv2Count <= RCV1_MAX) // 接收长度正确,继续处理.
      {
        // 从地址检测－接收到的上位机查询指令  与文本通讯
        if (WK_Rcv2Buffer[0] == Pw_EquipmentNo1)
        {
          j = CRC16(WK_Rcv2Buffer, WK_BakRcv2Count - 2); // CRC 校验
          k = j >> 8;
          s = j;
          if (k == WK_Rcv2Buffer[WK_BakRcv2Count - 2] && s == WK_Rcv2Buffer[WK_BakRcv2Count - 1])
          {                            // CRC校验正确
            if (WK_Rcv2Buffer[1] == 3) // 03读取保持寄存器
            {
              WK_B_Com2Cmd03 = 1;
              j = WK_Rcv2Buffer[2];
              WK_w_Com2RegAddr = (j << 8) + WK_Rcv2Buffer[3];
            }
            else if (WK_Rcv2Buffer[1] == 16) // 16预置多寄存器
            {
              //							C_ForceSavPar=0;		// 强制保存参数计数器=0
              WK_B_Com2Cmd16 = 1;
              j = WK_Rcv2Buffer[2];
              WK_w_Com2RegAddr = (j << 8) + WK_Rcv2Buffer[3];
            }
            else if (WK_Rcv2Buffer[1] == 1) // 01读取线圈状态
            {
              WK_B_Com2Cmd01 = 1;
            }
            else if (WK_Rcv2Buffer[1] == 6) // 06预置单寄存器
            {
              //							C_ForceSavPar=0;		// 强制保存参数计数器=0
              WK_B_Com2Cmd06 = 1;
              j = WK_Rcv2Buffer[2];
              WK_w_Com2RegAddr = (j << 8) + WK_Rcv2Buffer[3];
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
      // HAL_UART_Receive_IT(&huart1, (uint8_t *)&WK_Tmp_Rxd1Buffer, 1);
      scr = Wk2xxxReadReg(WK2XXX_PORT2, WK2XXX_SCR);
      scr |= WK2XXX_RXEN;
      Wk2xxxWriteReg(WK2XXX_PORT2, WK2XXX_SCR, scr);
      WK_Rcv2Counter = 0; // 准备下次接收到缓存开始
    }
  }
  if (i > 0)
  {
    for (j = 0; j < 20; j++)
    {
      WK_Rcv2Buffer[j] = 0;
    }
    //		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    // HAL_UART_Receive_IT(&huart1, (uint8_t *)&WK_Tmp_Rxd1Buffer, 1);
    scr = Wk2xxxReadReg(WK2XXX_PORT2, WK2XXX_SCR);
    scr |= WK2XXX_RXEN;
    Wk2xxxWriteReg(WK2XXX_PORT2, WK2XXX_SCR, scr);
  }
}

void WK_Com2_SlaveSend(void) // 串口1从机发送
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
  if (WK_B_Com2Cmd03) // 读取保持寄存器
  {
    WK_Txd2Buffer[0] = WK_Rcv2Buffer[0];     // 设备从地址Pw_EquipmentNo
    WK_Txd2Buffer[1] = WK_Rcv2Buffer[1];     // 功能码
    WK_Txd2Buffer[2] = WK_Rcv2Buffer[5] * 2; // WK_Rcv2Buffer[5]=字数 　
    //
    if (WK_w_Com2RegAddr < 0x800 && Pw_ComBufType == 1) // 常规查询 Pw_ComBufType==1 2016.4.21
    {
      p_wRead = w_ParLst; // PAR区
      p_bMove = WK_Txd2Buffer;
      //
      for (k = 0; k < WK_Rcv2Buffer[5]; k++) // 填充查询内容
      {
        m = *(p_wRead + WK_w_Com2RegAddr + k);
        *(p_bMove + 3 + k * 2) = m >> 8;
        *(p_bMove + 3 + k * 2 + 1) = m;
      }
    }
    else if (WK_w_Com2RegAddr >= 5000 && Pw_ComBufType == 1) // 读取电机设定参数，参数地址减5000
    {
      p_wRead2 = w_ParLst_Drive; // PAR区
      p_bMove = WK_Txd2Buffer;
      //
      for (k = 0; k < WK_Rcv2Buffer[5]; k++) // 填充查询内容
      {
        m = *(p_wRead2 + WK_w_Com2RegAddr - 5000 + k); //参数地址减5000
        *(p_bMove + 3 + k * 2) = m >> 8;
        *(p_bMove + 3 + k * 2 + 1) = m;
      }
    }
    //
    WK_w_Txd2ChkSum = CRC16(WK_Txd2Buffer, WK_Txd2Buffer[2] + 3);
    WK_Txd2Buffer[WK_Txd2Buffer[2] + 3] = WK_w_Txd2ChkSum >> 8; // /256
    WK_Txd2Buffer[WK_Txd2Buffer[2] + 4] = WK_w_Txd2ChkSum;      // 低位字节
    WK_Txd2Max = WK_Txd2Buffer[2] + 5;
    //
    WK_B_Com2Cmd03 = 0;
    WK_Txd2Counter = 0;

    // while (huart1.gState != HAL_UART_STATE_READY)
    //   ;
    // HAL_UART_Transmit_IT(&huart1, (uint8_t *)&WK_Txd2Buffer[WK_Txd2Counter++], 1);
    Wk2xxxSendBuf(WK2XXX_PORT2, WK_Txd2Buffer, WK_Txd2Max);
  }
  //
  else if (WK_B_Com2Cmd16 || WK_B_Com2Cmd06) // 16预置多寄存器
  {
    if (WK_w_Com2RegAddr <= 6000) //YLS 2020.06.23，不用密码就可以修改参数
    {
      j = 1;
    }
    // -------------------------
    // 修改参数单元
    if (j)
    {
      if (WK_B_Com2Cmd06) // 预置单个
      {
        if (WK_w_Com2RegAddr < 0x800)
        {
          m = WK_Rcv2Buffer[4];
          w_ParLst[WK_w_Com2RegAddr] = (m << 8) + WK_Rcv2Buffer[5];
        }
        //-------------------------
        else if (WK_w_Com2RegAddr >= 5000) // 修改伺服电机参数
        {
          m = WK_Rcv2Buffer[4];
          w_ParLst_Drive[WK_w_Com2RegAddr - 5000] = (m << 8) + WK_Rcv2Buffer[5]; //地址-5000
        }
        //-------------------------
      }
      else if (WK_B_Com2Cmd16) // 预置多个
      {
        if (WK_Rcv2Buffer[5] < 100)
        {
          if (WK_w_Com2RegAddr < 0x800)
          {
            p_bGen = WK_Rcv2Buffer;
            p_wTarget = w_ParLst;
            for (k = 0; k < WK_Rcv2Buffer[5]; k++) // WK_Rcv2Buffer[5]=字数
            {
              m = *(p_bGen + 7 + k * 2);
              n = *(p_bGen + 7 + k * 2 + 1);
              *(p_wTarget + WK_w_Com2RegAddr + k) = (m << 8) + n;
            }
          }
          else if (WK_w_Com2RegAddr >= 5000) // 修改伺服电机参数
          {
            p_bGen = WK_Rcv2Buffer;
            p_wTarget2 = w_ParLst_Drive;
            for (k = 0; k < WK_Rcv2Buffer[5]; k++) // WK_Rcv2Buffer[5]=字数
            {
              m = *(p_bGen + 7 + k * 2);
              n = *(p_bGen + 7 + k * 2 + 1);
              *(p_wTarget2 + WK_w_Com2RegAddr - 5000 + k) = (m << 8) + n;
            }
          }
        }
      }
    }

    // -------------------------
    // 返回数据
    WK_Txd2Buffer[0] = 2;                // 设备从地址
    WK_Txd2Buffer[1] = WK_Rcv2Buffer[1]; // 功能码
    WK_Txd2Buffer[2] = WK_Rcv2Buffer[2]; // 开始地址高位字节
    WK_Txd2Buffer[3] = WK_Rcv2Buffer[3]; // 开始地址低位字节
    WK_Txd2Buffer[4] = WK_Rcv2Buffer[4]; // 寄存器数量高位
    WK_Txd2Buffer[5] = WK_Rcv2Buffer[5]; // 寄存器数量低位
    if (j == 0)                          // 如果不能被正常预置，返回FFFF zcl
    {
      WK_Txd2Buffer[4] = 0xff; // 寄存器数量高位、预置数据
      WK_Txd2Buffer[5] = 0xff; // 寄存器数量低位、预置数据
    }
    WK_w_Txd2ChkSum = CRC16(WK_Txd2Buffer, 6);
    WK_Txd2Buffer[6] = WK_w_Txd2ChkSum >> 8; // /256
    WK_Txd2Buffer[7] = WK_w_Txd2ChkSum;      // 低位字节
    WK_Txd2Max = 8;

    WK_B_Com2Cmd16 = 0;
    WK_B_Com2Cmd06 = 0;
    WK_Txd2Counter = 0;

    // while (huart1.gState != HAL_UART_STATE_READY)
    //   ;
    // HAL_UART_Transmit_IT(&huart1, (uint8_t *)&WK_Txd2Buffer[WK_Txd2Counter++], 1);
    Wk2xxxSendBuf(WK2XXX_PORT2, WK_Txd2Buffer, WK_Txd2Max);

  } // 06、16预置寄存器 结束
}

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
