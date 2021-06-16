/** 
  ******************************************************************************

	******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "wk_port1.h"
#include "GlobalV_Extern.h" // 全局变量声明
// #include "GlobalConst.h"
#include <stdio.h> //加上此句可以用printf
//#include "CRCdata.h"
#include "wk2xxx.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

//
//uint8_t		B_Com1Send;

//uint8_t		B_Com1Cmd03;
//uint8_t		B_Com1Cmd16;
//uint8_t		B_Com1Cmd01;
//uint8_t		B_Com1Cmd06;
//uint8_t		B_Com1Cmd73;
//uint16_t		T_NoRcv1Count;						// 没有接收计数器
//uint16_t		C_NoRcv1Count;
//uint16_t     C_Com1_MasterSendDelay;
//uint16_t	   T_Com1_MasterSendDelay;
//uint16_t     C_Com1_MasterSendDelay2;
//uint16_t	   T_Com1_MasterSendDelay2;
//uint16_t     C_Com1_MasterSendDelay5;
//uint16_t	   T_Com1_MasterSendDelay5;

//uint8_t	Com1_Query_PC=0;		//COM1查询缓冲区指针
//uint8_t	Com1_CMD_PC=0;			//COM1命令缓冲区指针

//s16	Com1_Driver1_Queue_Rear;			//命令队列中的命令数量，队尾指针
//s16	Com1_Driver2_Queue_Rear;			//命令队列中的命令数量，队尾指针

//s16	Com1_Driver1_Queue_Front;			//命令队列中的当前要出队的，队头指针
//s16	Com1_Driver2_Queue_Front;			//命令队列中的当前要出队的，队头指针

//uint8_t	Com1_QReceiveDataTargetLength;	//接收的数据应该的长度

//extern	UART_HandleTypeDef huart1;
////判断COM1命令队列是否为空
//extern	uint8_t Com1_Driver1_Queue_isEmpty(void);
//extern	uint8_t Com1_Driver2_Queue_isEmpty(void);
//extern	uint8_t Com2_Driver3_Queue_isEmpty(void);
//extern	uint8_t Com2_Driver4_Queue_isEmpty(void);
//extern	uint8_t Com3_Driver5_Queue_isEmpty(void);
//extern	uint8_t Com3_Driver6_Queue_isEmpty(void);

////判断COM1命令队列是否为满
//extern	uint8_t Com1_Driver1_Queue_isFull(void);
//extern	uint8_t Com1_Driver2_Queue_isFull(void);
//extern	uint8_t Com2_Driver3_Queue_isFull(void);
//extern	uint8_t Com2_Driver4_Queue_isFull(void);
//extern	uint8_t Com3_Driver5_Queue_isFull(void);
//extern	uint8_t Com3_Driver6_Queue_isFull(void);

////定义每台电机的写命令缓冲区
////每条定义：序号+命令长度+命令内容
//extern	uint8_t	Com1_Driver1_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];	//COM_CMD_SIZE=30，COM_CMD_NUM=6.定义6条写命令，每条30个字节
//extern	uint8_t	Com1_Driver2_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];
//extern	uint8_t	Com2_Driver3_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];
//extern	uint8_t	Com2_Driver4_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];
//extern	uint8_t	Com3_Driver5_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];
//extern	uint8_t	Com3_Driver6_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];

////定义每台电机的上一条命令
////定义：序号+命令长度+命令内容
//extern	uint8_t	Com1_LAST_CMD_BUFF[COM_CMD_SIZE];	//定义1条，30个字节
//extern	uint8_t	Com2_LAST_CMD_BUFF[COM_CMD_SIZE];
//extern	uint8_t	Com3_LAST_CMD_BUFF[COM_CMD_SIZE];
//extern	uint8_t	Com4_LAST_CMD_BUFF[COM_CMD_SIZE];

////查询指令表
//extern	uint8_t	Com1_Query_Data[];
//extern	uint8_t	Com2_Query_Data[];
//extern	uint8_t	Com3_Query_Data[];

//extern	uint16_t	Driver1_RcvCount;		//接收计数
//extern	uint16_t	Driver2_RcvCount;		//接收计数

//uint16_t	Com1_Send_Sort=0;				//Com1发送命令顺序，是发1#命令队列，还是发2#命令队列
//uint16_t	Com1_Send_CMD_Type_Sort=0;	//Com1是发送查询还是发送控制命令，=0，查询；=1，控制命令

///* Private function prototypes -----------------------------------------------*/
//void GPIO_Com1_Configuration(void);							//GPIO配置
//void Com1_config(void);
//void Com1_Driver1_Send_CMD(void);					//发送命令
//void Com1_Driver2_Send_CMD(void);					//发送命令
//uint16_t CRC16(uint8_t * pCrcData,uint8_t CrcDataLen);
//void Delay_MS(vu16 nCount);
//uint16_t Address(uint16_t *p, uint16_t Area); 					//绝对地址
//extern	void PowerDelay(uint16_t nCount);

//extern	uint8_t	F_Sync_6_axis;						//6轴同步输出标志

//extern	uint8_t	F_Driver1_Send_Cmd;					//控制器给1#伺服驱动器发出位置命令标志，=1表示已发送命令
//extern	uint8_t	F_Driver2_Send_Cmd;
//extern	uint8_t	F_Driver3_Send_Cmd;
//extern	uint8_t	F_Driver4_Send_Cmd;
//extern	uint8_t	F_Driver5_Send_Cmd;
//extern	uint8_t	F_Driver6_Send_Cmd;

//extern	uint8_t	F_Driver1_Timeout;					//1#伺服驱动器发送命令超时标志，=1表示超时
//extern	uint8_t	F_Driver2_Timeout;
//extern	uint8_t	F_Driver3_Timeout;
//extern	uint8_t	F_Driver4_Timeout;
//extern	uint8_t	F_Driver5_Timeout;
//extern	uint8_t	F_Driver6_Timeout;

//extern	uint8_t	F_Driver1_Cmd_Err;					//1#伺服驱动器位置命令错误标志，=1表示错误
//extern	uint8_t	F_Driver2_Cmd_Err;
//extern	uint8_t	F_Driver3_Cmd_Err;
//extern	uint8_t	F_Driver4_Cmd_Err;
//extern	uint8_t	F_Driver5_Cmd_Err;
//extern	uint8_t	F_Driver6_Cmd_Err;

//extern	uint8_t	F_Driver1_Cmd_Con_Err;					//1#伺服驱动器控制命令错误标志，=1表示错误
//extern	uint8_t	F_Driver2_Cmd_Con_Err;
//extern	uint8_t	F_Driver3_Cmd_Con_Err;
//extern	uint8_t	F_Driver4_Cmd_Con_Err;
//extern	uint8_t	F_Driver5_Cmd_Con_Err;
//extern	uint8_t	F_Driver6_Cmd_Con_Err;

//extern	uint8_t	Driver1_Cmd_PosNo;					//1#伺服驱动器发送的命令位置号，位置0或者位置1
//extern	uint8_t	Driver2_Cmd_PosNo;
//extern	uint8_t	Driver3_Cmd_PosNo;
//extern	uint8_t	Driver4_Cmd_PosNo;
//extern	uint8_t	Driver5_Cmd_PosNo;
//extern	uint8_t	Driver6_Cmd_PosNo;

//extern	uint8_t	Driver1_Cmd_Status;					//1#伺服驱动器P8910状态
//extern	uint8_t	Driver2_Cmd_Status;
//extern	uint8_t	Driver3_Cmd_Status;
//extern	uint8_t	Driver4_Cmd_Status;
//extern	uint8_t	Driver5_Cmd_Status;
//extern	uint8_t	Driver6_Cmd_Status;

//extern	uint16_t	T_Driver1_WriteCMD;
//extern	uint16_t	C_Driver1_WriteCMD;
//extern	uint16_t	T_Driver2_WriteCMD;
//extern	uint16_t	C_Driver2_WriteCMD;
//extern	uint16_t	T_Driver3_WriteCMD;
//extern	uint16_t	C_Driver3_WriteCMD;
//extern	uint16_t	T_Driver4_WriteCMD;
//extern	uint16_t	C_Driver4_WriteCMD;
//extern	uint16_t	T_Driver5_WriteCMD;
//extern	uint16_t	C_Driver5_WriteCMD;
//extern	uint16_t	T_Driver6_WriteCMD;
//extern	uint16_t	C_Driver6_WriteCMD;

//extern	uint8_t Driver1_Cmd_Data[9];							//1#伺服驱动器命令数据,4个字节为脉冲，2个字节为速度，2个字节为加减速时间，最后一个字节为位置号（0/1）
//extern	uint8_t Driver2_Cmd_Data[9];
//extern	uint8_t Driver3_Cmd_Data[9];
//extern	uint8_t Driver4_Cmd_Data[9];
//extern	uint8_t Driver5_Cmd_Data[9];
//extern	uint8_t Driver6_Cmd_Data[9];

//extern	s32 *arr_p1;

//extern	uint8_t Driver1_Pos_Start_Sort;						//1#伺服控制是发位置指令，还是发启动命令。=0，发位置；=1，发启动命令
//extern	uint8_t Driver2_Pos_Start_Sort;
//extern	uint8_t Driver3_Pos_Start_Sort;
//extern	uint8_t Driver4_Pos_Start_Sort;
//extern	uint8_t Driver5_Pos_Start_Sort;
//extern	uint8_t Driver6_Pos_Start_Sort;

//extern	uint8_t Driver1_Status_Sort;							//1#伺服，=2，表示已经发送；=3，表示已经接收
//extern	uint8_t Driver2_Status_Sort;
//extern	uint8_t Driver3_Status_Sort;
//extern	uint8_t Driver4_Status_Sort;
//extern	uint8_t Driver5_Status_Sort;
//extern	uint8_t Driver6_Status_Sort;

///* Private functions ---------------------------------------------------------*/

//接收处理程序 校验程序
void WK_Com1_RcvProcess(void)
{
  uint8_t k, s, i = 0; // 临时变量
  uint16_t j;
  unsigned char scr;
  //作为主机,指定接收时间到了,就可以处理接收到的字符串了
  // 在没收到串行字符的时间超过设定时，可以对接收缓存进行处理了
  // **********************************rcv_counter<>0,收到字符才能处理
  if (WK_Rcv1Counter > 0 && T_WK_NoRcv1Count != SClk1Ms)
  {                             // 接收处理过程
    T_WK_NoRcv1Count = SClk1Ms; //
    C_WK_NoRcv1Count++;
    if (C_WK_NoRcv1Count > NORCVMAXMS) //
    {
      /* Disable the UART Parity Error Interrupt and RXNE interrupt*/
      //			CLEAR_BIT(huart1->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
      //禁止子串口的接收使能
      scr = Wk2xxxReadReg(WK2XXX_PORT1, WK2XXX_SCR);
      scr &= ~WK2XXX_RXEN;
      Wk2xxxWriteReg(WK2XXX_PORT1, WK2XXX_SCR, scr);

      WK_BakRcv1Count = WK_Rcv1Counter; // 把 WK_RRcv1Counter 保存
      C_WK_NoRcv1Count = 0;             // 清没有接收计数器
      //
      if (WK_BakRcv1Count <= RCV1_MAX) // 接收长度正确,继续处理.
      {
        // 从地址检测－接收到的上位机查询指令  与文本通讯
        if (WK_Rcv1Buffer[0] == Pw_EquipmentNo1)
        {
          j = CRC16(WK_Rcv1Buffer, WK_BakRcv1Count - 2); // CRC 校验
          k = j >> 8;
          s = j;
          if (k == WK_Rcv1Buffer[WK_BakRcv1Count - 2] && s == WK_Rcv1Buffer[WK_BakRcv1Count - 1])
          {                            // CRC校验正确
            if (WK_Rcv1Buffer[1] == 3) // 03读取保持寄存器
            {
              WK_B_Com1Cmd03 = 1;
              j = WK_Rcv1Buffer[2];
              WK_w_Com1RegAddr = (j << 8) + WK_Rcv1Buffer[3];
            }
            else if (WK_Rcv1Buffer[1] == 16) // 16预置多寄存器
            {
              //							C_ForceSavPar=0;		// 强制保存参数计数器=0
              WK_B_Com1Cmd16 = 1;
              j = WK_Rcv1Buffer[2];
              WK_w_Com1RegAddr = (j << 8) + WK_Rcv1Buffer[3];
            }
            else if (WK_Rcv1Buffer[1] == 1) // 01读取线圈状态
            {
              WK_B_Com1Cmd01 = 1;
            }
            else if (WK_Rcv1Buffer[1] == 6) // 06预置单寄存器
            {
              //							C_ForceSavPar=0;		// 强制保存参数计数器=0
              WK_B_Com1Cmd06 = 1;
              j = WK_Rcv1Buffer[2];
              WK_w_Com1RegAddr = (j << 8) + WK_Rcv1Buffer[3];
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
      scr = Wk2xxxReadReg(WK2XXX_PORT1, WK2XXX_SCR);
      scr |= WK2XXX_RXEN;
      Wk2xxxWriteReg(WK2XXX_PORT1, WK2XXX_SCR, scr);
      WK_Rcv1Counter = 0; // 准备下次接收到缓存开始
    }
  }
  if (i > 0)
  {
    for (j = 0; j < 20; j++)
    {
      WK_Rcv1Buffer[j] = 0;
    }
    //		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    // HAL_UART_Receive_IT(&huart1, (uint8_t *)&WK_Tmp_Rxd1Buffer, 1);
    scr = Wk2xxxReadReg(WK2XXX_PORT1, WK2XXX_SCR);
    scr |= WK2XXX_RXEN;
    Wk2xxxWriteReg(WK2XXX_PORT1, WK2XXX_SCR, scr);
  }
}

void WK_Com1_SlaveSend(void) // 串口1从机发送
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
  if (WK_B_Com1Cmd03) // 读取保持寄存器
  {
    WK_Txd1Buffer[0] = WK_Rcv1Buffer[0];     // 设备从地址Pw_EquipmentNo
    WK_Txd1Buffer[1] = WK_Rcv1Buffer[1];     // 功能码
    WK_Txd1Buffer[2] = WK_Rcv1Buffer[5] * 2; // WK_Rcv1Buffer[5]=字数 　
    //
    if (WK_w_Com1RegAddr < 0x800) // 常规查询
    {
      p_wRead = w_ParLst; // PAR区
      p_bMove = WK_Txd1Buffer;
      //
      for (k = 0; k < WK_Rcv1Buffer[5]; k++) // 填充查询内容
      {
        m = *(p_wRead + WK_w_Com1RegAddr + k);
        *(p_bMove + 3 + k * 2) = m >> 8;
        *(p_bMove + 3 + k * 2 + 1) = m;
      }
    }
    else if (WK_w_Com1RegAddr >= 5000) // 读取电机设定参数，参数地址减5000
    {
      p_wRead2 = w_ParLst_Drive; // PAR区
      p_bMove = WK_Txd1Buffer;
      //
      for (k = 0; k < WK_Rcv1Buffer[5]; k++) // 填充查询内容
      {
        m = *(p_wRead2 + WK_w_Com1RegAddr - 5000 + k); //参数地址减5000
        *(p_bMove + 3 + k * 2) = m >> 8;
        *(p_bMove + 3 + k * 2 + 1) = m;
      }
    }
    //
    WK_w_Txd1ChkSum = CRC16(WK_Txd1Buffer, WK_Txd1Buffer[2] + 3);
    WK_Txd1Buffer[WK_Txd1Buffer[2] + 3] = WK_w_Txd1ChkSum >> 8; // /256
    WK_Txd1Buffer[WK_Txd1Buffer[2] + 4] = WK_w_Txd1ChkSum;      // 低位字节
    WK_Txd1Max = WK_Txd1Buffer[2] + 5;
    //
    WK_B_Com1Cmd03 = 0;
    WK_Txd1Counter = 0;

    // while (huart1.gState != HAL_UART_STATE_READY)
    //   ;
    // HAL_UART_Transmit_IT(&huart1, (uint8_t *)&WK_Txd1Buffer[WK_Txd1Counter++], 1);
    Wk2xxxSendBuf(WK2XXX_PORT1, WK_Txd1Buffer, WK_Txd1Max);
  }
  //
  else if (WK_B_Com1Cmd16 || WK_B_Com1Cmd06) // 16预置多寄存器
  {
    if (WK_w_Com1RegAddr <= 6000) //YLS 2020.06.23，不用密码就可以修改参数
    {
      j = 1;
    }
    // -------------------------
    // 修改参数单元
    if (j)
    {
      if (WK_B_Com1Cmd06) // 预置单个
      {
        if (WK_w_Com1RegAddr < 0x800)
        {
          m = WK_Rcv1Buffer[4];
          w_ParLst[WK_w_Com1RegAddr] = (m << 8) + WK_Rcv1Buffer[5];
        }
        //-------------------------
        else if (WK_w_Com1RegAddr >= 5000) // 修改伺服电机参数
        {
          m = WK_Rcv1Buffer[4];
          w_ParLst_Drive[WK_w_Com1RegAddr - 5000] = (m << 8) + WK_Rcv1Buffer[5]; //地址-5000
        }
        //-------------------------
      }
      else if (WK_B_Com1Cmd16) // 预置多个
      {
        if (WK_Rcv1Buffer[5] < 100)
        {
          if (WK_w_Com1RegAddr < 0x800)
          {
            p_bGen = WK_Rcv1Buffer;
            p_wTarget = w_ParLst;
            for (k = 0; k < WK_Rcv1Buffer[5]; k++) // WK_Rcv1Buffer[5]=字数
            {
              m = *(p_bGen + 7 + k * 2);
              n = *(p_bGen + 7 + k * 2 + 1);
              *(p_wTarget + WK_w_Com1RegAddr + k) = (m << 8) + n;
            }
          }
          else if (WK_w_Com1RegAddr >= 5000) // 修改伺服电机参数
          {
            p_bGen = WK_Rcv1Buffer;
            p_wTarget2 = w_ParLst_Drive;
            for (k = 0; k < WK_Rcv1Buffer[5]; k++) // WK_Rcv1Buffer[5]=字数
            {
              m = *(p_bGen + 7 + k * 2);
              n = *(p_bGen + 7 + k * 2 + 1);
              *(p_wTarget2 + WK_w_Com1RegAddr - 5000 + k) = (m << 8) + n;
            }
          }
        }
      }
    }

    // -------------------------
    // 返回数据
    WK_Txd1Buffer[0] = 2;                // 设备从地址
    WK_Txd1Buffer[1] = WK_Rcv1Buffer[1]; // 功能码
    WK_Txd1Buffer[2] = WK_Rcv1Buffer[2]; // 开始地址高位字节
    WK_Txd1Buffer[3] = WK_Rcv1Buffer[3]; // 开始地址低位字节
    WK_Txd1Buffer[4] = WK_Rcv1Buffer[4]; // 寄存器数量高位
    WK_Txd1Buffer[5] = WK_Rcv1Buffer[5]; // 寄存器数量低位
    if (j == 0)                          // 如果不能被正常预置，返回FFFF zcl
    {
      WK_Txd1Buffer[4] = 0xff; // 寄存器数量高位、预置数据
      WK_Txd1Buffer[5] = 0xff; // 寄存器数量低位、预置数据
    }
    WK_w_Txd1ChkSum = CRC16(WK_Txd1Buffer, 6);
    WK_Txd1Buffer[6] = WK_w_Txd1ChkSum >> 8; // /256
    WK_Txd1Buffer[7] = WK_w_Txd1ChkSum;      // 低位字节
    WK_Txd1Max = 8;

    WK_B_Com1Cmd16 = 0;
    WK_B_Com1Cmd06 = 0;
    WK_Txd1Counter = 0;

    // while (huart1.gState != HAL_UART_STATE_READY)
    //   ;
    // HAL_UART_Transmit_IT(&huart1, (uint8_t *)&WK_Txd1Buffer[WK_Txd1Counter++], 1);
    Wk2xxxSendBuf(WK2XXX_PORT1, WK_Txd1Buffer, WK_Txd1Max);

  } // 06、16预置寄存器 结束
}

//uint16_t CRC16(uint8_t * pCrcData,uint8_t CrcDataLen)
//{
//	uint8_t CRC16Hi=0xff;                   /* 高CRC字节初始化 */
//  uint8_t CRC16Lo=0xff;                   /* 低CRC字节初始化*/
//	uint8_t Index=0;
//	uint16_t  w_CRC16=0;
//	while(CrcDataLen--)
//	{
//		Index = CRC16Hi ^* pCrcData++;
//		CRC16Hi = CRC16Lo ^ CRC_H[Index];
//		CRC16Lo = CRC_L[Index];
//	}
//	w_CRC16 = (CRC16Hi << 8) | CRC16Lo;
//	return (w_CRC16);
//}

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
