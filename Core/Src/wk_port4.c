/** 
  ******************************************************************************

	******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "wk_port4.h"
#include "GlobalV_Extern.h" // ȫ�ֱ�������
// #include "GlobalConst.h"
#include <stdio.h> //���ϴ˾������printf
//#include "CRCdata.h"
#include "wk2xxx.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

///* Private functions ---------------------------------------------------------*/

//���մ������ У�����
void WK_Com4_RcvProcess(void)
{
  uint8_t k, s, i = 0; // ��ʱ����
  uint16_t j;
  unsigned char scr;
  //��Ϊ����,ָ������ʱ�䵽��,�Ϳ��Դ�����յ����ַ�����
  // ��û�յ������ַ���ʱ�䳬���趨ʱ�����ԶԽ��ջ�����д�����
  // **********************************rcv_counter<>0,�յ��ַ����ܴ���
  if (WK_Rcv4Counter > 0 && T_WK_NoRcv4Count != SClk1Ms)
  {                             // ���մ������
    T_WK_NoRcv4Count = SClk1Ms; //
    C_WK_NoRcv4Count++;
    if (C_WK_NoRcv4Count > NORCVMAXMS) //
    {
      /* Disable the UART Parity Error Interrupt and RXNE interrupt*/
      //			CLEAR_BIT(huart1->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
      //��ֹ�Ӵ��ڵĽ���ʹ��
      scr = Wk2xxxReadReg(WK2XXX_PORT4, WK2XXX_SCR);
      scr &= ~WK2XXX_RXEN;
      Wk2xxxWriteReg(WK2XXX_PORT4, WK2XXX_SCR, scr);

      WK_BakRcv4Count = WK_Rcv4Counter; // �� WK_RRcv4Counter ����
      C_WK_NoRcv4Count = 0;             // ��û�н��ռ�����
      //
      if (WK_BakRcv4Count <= RCV1_MAX) // ���ճ�����ȷ,��������.
      {
        // �ӵ�ַ��⣭���յ�����λ����ѯָ��  ���ı�ͨѶ
        if (WK_Rcv4Buffer[0] == Pw_EquipmentNo1)
        {
          j = CRC16(WK_Rcv4Buffer, WK_BakRcv4Count - 2); // CRC У��
          k = j >> 8;
          s = j;
          if (k == WK_Rcv4Buffer[WK_BakRcv4Count - 2] && s == WK_Rcv4Buffer[WK_BakRcv4Count - 1])
          {                            // CRCУ����ȷ
            if (WK_Rcv4Buffer[1] == 3) // 03��ȡ���ּĴ���
            {
              WK_B_Com4Cmd03 = 1;
              j = WK_Rcv4Buffer[2];
              WK_w_Com4RegAddr = (j << 8) + WK_Rcv4Buffer[3];
            }
            else if (WK_Rcv4Buffer[1] == 16) // 16Ԥ�ö�Ĵ���
            {
              //							C_ForceSavPar=0;		// ǿ�Ʊ������������=0
              WK_B_Com4Cmd16 = 1;
              j = WK_Rcv4Buffer[2];
              WK_w_Com4RegAddr = (j << 8) + WK_Rcv4Buffer[3];
            }
            else if (WK_Rcv4Buffer[1] == 1) // 01��ȡ��Ȧ״̬
            {
              WK_B_Com4Cmd01 = 1;
            }
            else if (WK_Rcv4Buffer[1] == 6) // 06Ԥ�õ��Ĵ���
            {
              //							C_ForceSavPar=0;		// ǿ�Ʊ������������=0
              WK_B_Com4Cmd06 = 1;
              j = WK_Rcv4Buffer[2];
              WK_w_Com4RegAddr = (j << 8) + WK_Rcv4Buffer[3];
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
      scr = Wk2xxxReadReg(WK2XXX_PORT4, WK2XXX_SCR);
      scr |= WK2XXX_RXEN;
      Wk2xxxWriteReg(WK2XXX_PORT4, WK2XXX_SCR, scr);
      WK_Rcv4Counter = 0; // ׼���´ν��յ����濪ʼ
    }
  }
  if (i > 0)
  {
    for (j = 0; j < 20; j++)
    {
      WK_Rcv4Buffer[j] = 0;
    }
    //		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    // HAL_UART_Receive_IT(&huart1, (uint8_t *)&WK_Tmp_Rxd1Buffer, 1);
    scr = Wk2xxxReadReg(WK2XXX_PORT4, WK2XXX_SCR);
    scr |= WK2XXX_RXEN;
    Wk2xxxWriteReg(WK2XXX_PORT4, WK2XXX_SCR, scr);
  }
}

void WK_Com4_SlaveSend(void) // ����1�ӻ�����
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
  if (WK_B_Com4Cmd03) // ��ȡ���ּĴ���
  {
    WK_Txd4Buffer[0] = WK_Rcv4Buffer[0];     // �豸�ӵ�ַPw_EquipmentNo
    WK_Txd4Buffer[1] = WK_Rcv4Buffer[1];     // ������
    WK_Txd4Buffer[2] = WK_Rcv4Buffer[5] * 2; // WK_Rcv4Buffer[5]=���� ��
    //
    if (WK_w_Com4RegAddr < 0x800 && Pw_ComBufType == 1) // �����ѯ Pw_ComBufType==1 2016.4.21
    {
      p_wRead = w_ParLst; // PAR��
      p_bMove = WK_Txd4Buffer;
      //
      for (k = 0; k < WK_Rcv4Buffer[5]; k++) // ����ѯ����
      {
        m = *(p_wRead + WK_w_Com4RegAddr + k);
        *(p_bMove + 3 + k * 2) = m >> 8;
        *(p_bMove + 3 + k * 2 + 1) = m;
      }
    }
    else if (WK_w_Com4RegAddr >= 5000 && Pw_ComBufType == 1) // ��ȡ����趨������������ַ��5000
    {
      p_wRead2 = w_ParLst_Drive; // PAR��
      p_bMove = WK_Txd4Buffer;
      //
      for (k = 0; k < WK_Rcv4Buffer[5]; k++) // ����ѯ����
      {
        m = *(p_wRead2 + WK_w_Com4RegAddr - 5000 + k); //������ַ��5000
        *(p_bMove + 3 + k * 2) = m >> 8;
        *(p_bMove + 3 + k * 2 + 1) = m;
      }
    }
    //
    WK_w_Txd4ChkSum = CRC16(WK_Txd4Buffer, WK_Txd4Buffer[2] + 3);
    WK_Txd4Buffer[WK_Txd4Buffer[2] + 3] = WK_w_Txd4ChkSum >> 8; // /256
    WK_Txd4Buffer[WK_Txd4Buffer[2] + 4] = WK_w_Txd4ChkSum;      // ��λ�ֽ�
    WK_Txd4Max = WK_Txd4Buffer[2] + 5;
    //
    WK_B_Com4Cmd03 = 0;
    WK_Txd4Counter = 0;

    // while (huart1.gState != HAL_UART_STATE_READY)
    //   ;
    // HAL_UART_Transmit_IT(&huart1, (uint8_t *)&WK_Txd4Buffer[WK_Txd4Counter++], 1);
    Wk2xxxSendBuf(WK2XXX_PORT4, WK_Txd4Buffer, WK_Txd4Max);
  }
  //
  else if (WK_B_Com4Cmd16 || WK_B_Com4Cmd06) // 16Ԥ�ö�Ĵ���
  {
    if (WK_w_Com4RegAddr <= 6000) //YLS 2020.06.23����������Ϳ����޸Ĳ���
    {
      j = 1;
    }
    // -------------------------
    // �޸Ĳ�����Ԫ
    if (j)
    {
      if (WK_B_Com4Cmd06) // Ԥ�õ���
      {
        if (WK_w_Com4RegAddr < 0x800)
        {
          m = WK_Rcv4Buffer[4];
          w_ParLst[WK_w_Com4RegAddr] = (m << 8) + WK_Rcv4Buffer[5];
        }
        //-------------------------
        else if (WK_w_Com4RegAddr >= 5000) // �޸��ŷ��������
        {
          m = WK_Rcv4Buffer[4];
          w_ParLst_Drive[WK_w_Com4RegAddr - 5000] = (m << 8) + WK_Rcv4Buffer[5]; //��ַ-5000
        }
        //-------------------------
      }
      else if (WK_B_Com4Cmd16) // Ԥ�ö��
      {
        if (WK_Rcv4Buffer[5] < 100)
        {
          if (WK_w_Com4RegAddr < 0x800)
          {
            p_bGen = WK_Rcv4Buffer;
            p_wTarget = w_ParLst;
            for (k = 0; k < WK_Rcv4Buffer[5]; k++) // WK_Rcv4Buffer[5]=����
            {
              m = *(p_bGen + 7 + k * 2);
              n = *(p_bGen + 7 + k * 2 + 1);
              *(p_wTarget + WK_w_Com4RegAddr + k) = (m << 8) + n;
            }
          }
          else if (WK_w_Com4RegAddr >= 5000) // �޸��ŷ��������
          {
            p_bGen = WK_Rcv4Buffer;
            p_wTarget2 = w_ParLst_Drive;
            for (k = 0; k < WK_Rcv4Buffer[5]; k++) // WK_Rcv4Buffer[5]=����
            {
              m = *(p_bGen + 7 + k * 2);
              n = *(p_bGen + 7 + k * 2 + 1);
              *(p_wTarget2 + WK_w_Com4RegAddr - 5000 + k) = (m << 8) + n;
            }
          }
        }
      }
    }

    // -------------------------
    // ��������
    WK_Txd4Buffer[0] = 2;                // �豸�ӵ�ַ
    WK_Txd4Buffer[1] = WK_Rcv4Buffer[1]; // ������
    WK_Txd4Buffer[2] = WK_Rcv4Buffer[2]; // ��ʼ��ַ��λ�ֽ�
    WK_Txd4Buffer[3] = WK_Rcv4Buffer[3]; // ��ʼ��ַ��λ�ֽ�
    WK_Txd4Buffer[4] = WK_Rcv4Buffer[4]; // �Ĵ���������λ
    WK_Txd4Buffer[5] = WK_Rcv4Buffer[5]; // �Ĵ���������λ
    if (j == 0)                          // ������ܱ�����Ԥ�ã�����FFFF zcl
    {
      WK_Txd4Buffer[4] = 0xff; // �Ĵ���������λ��Ԥ������
      WK_Txd4Buffer[5] = 0xff; // �Ĵ���������λ��Ԥ������
    }
    WK_w_Txd4ChkSum = CRC16(WK_Txd4Buffer, 6);
    WK_Txd4Buffer[6] = WK_w_Txd4ChkSum >> 8; // /256
    WK_Txd4Buffer[7] = WK_w_Txd4ChkSum;      // ��λ�ֽ�
    WK_Txd4Max = 8;

    WK_B_Com4Cmd16 = 0;
    WK_B_Com4Cmd06 = 0;
    WK_Txd4Counter = 0;

    // while (huart1.gState != HAL_UART_STATE_READY)
    //   ;
    // HAL_UART_Transmit_IT(&huart1, (uint8_t *)&WK_Txd4Buffer[WK_Txd4Counter++], 1);
    Wk2xxxSendBuf(WK2XXX_PORT4, WK_Txd4Buffer, WK_Txd4Max);

  } // 06��16Ԥ�üĴ��� ����
}

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
