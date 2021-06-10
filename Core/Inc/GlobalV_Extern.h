
/************************************************************************
*			CopyRight  2006.3.24   技术中心硬件室   				    *
************************************************************************
* 文件名称: GlobalV_Extern.h						
* 文件版本：1.00
* 创建人员：周成磊					  							
* 创建日期：2006.3.24											
* 功能描述：全局变量声明。
* 相关硬件：C80F1F020,ISD4004-08M,ISL1208,M25P80			
* 仿真环境：KEILC51 V7.09
* C 编译器：KEIL V6.12 COMPILER						
* 修改记录：2006/04/01 08:19   周成磊
*			
*			 
************************************************************************/
#ifndef __GLOBALV_EXTERN_H
#define __GLOBALV_EXTERN_H

#include "stm32f4xx.h"
#include "typedef.h"
#include "GlobalConst.h"

// (定义变量)
extern uint8_t VfNo;        // 变频泵序号
extern uint8_t VfNext;      // 可以启动的下一台变频泵
extern uint8_t SoftClock[]; // 辅助时钟
//
extern uint8_t Vf_No;
extern uint8_t F_AskStop1;
extern uint8_t RealClock[];      // 实时时钟
extern uint8_t B_PumpBad[];      // B_PumpBad[0]未使用,泵1-5坏标志
extern uint8_t B_PumpManu[];     // 泵手动状态1-5标志(检修用)
extern uint8_t B_SaveSetP;       // 保存设定压力 标志字节
extern uint8_t B_SaveVoiceValue; // 保存音量大小 标志字节
extern uint8_t B_ReadVfPar;      // 读取变频器数据
extern uint8_t B_ModSelParValue; // 修改选择的参数值
extern uint8_t F_ComErrorPump1;
extern uint8_t F_ComErrorPump2;
extern uint8_t B_ComErrorPump1;
extern uint8_t B_ComErrorPump2;
extern uint8_t B_RemoteStop1;
extern uint8_t B_RemoteStop2;
extern uint8_t B_MaxSupplyStop1;
extern uint8_t B_MaxSupplyStop2;
extern uint8_t F_ComErrorPump3;
extern uint8_t F_ComErrorPump4;
extern uint8_t B_ComErrorPump3;
extern uint8_t B_ComErrorPump4;
extern uint8_t B_RemoteStop3;
extern uint8_t B_RemoteStop4;
extern uint8_t B_MaxSupplyStop3;
extern uint8_t B_MaxSupplyStop4;
extern uint8_t F_HighYCSetP;
extern uint8_t S_HighYCSetP;
extern uint8_t F_HaveWater_MaxSupply;
extern uint8_t F_InPPid;
extern uint8_t F_HengYa;
extern uint8_t F_waterYeWeiHigh;
extern uint8_t F_ZhuoDuHigh;
extern uint8_t F_YuLvHigh;
extern uint8_t F_PhHigh;
extern uint8_t F_PhLow;

extern uint8_t F_Pump1RunOverTime;
extern uint8_t F_Pump2RunOverTime;
extern uint8_t F_Pump3RunOverTime;
extern uint8_t F_Pump4RunOverTime;
extern uint8_t F_YeJianSmall;
extern uint8_t F_HengVfPanDuan_S;
extern uint8_t F_HengVfPanDuan_L;

extern uint8_t F_PumpExit;
extern uint8_t PumpExit[];

extern uint8_t DInb[];  // 开关量输入参数列表区
extern uint8_t DOutb[]; // 开关量输出参数列表区
//extern uint8_t  NBComBuf[];		// 宁波通讯缓存 2007.10.30
extern uint16_t w_AI[]; // 模拟量输入参数列表区
//extern uint16_t  w_AQ[];			// 模拟量输出参数列表区
extern uint16_t w_ParLst[];     // 参数字列表区
extern s32 w_ParLst_Drive[];    //YLS 2020.06.23
extern uint32_t w_ParLst_POS[]; //YLS 2020.11.27

// 2008.12.11
extern uint8_t B_SelCS;
extern uint16_t w_FaultNo[];
extern uint16_t w_ModRealTimeNo; // 修改实时时钟的序号
extern uint16_t w_PreRecNo;      // 上次记录号

//
// 以下为用外部字节定义的开关量输入，开关量输出部分
// 硬件定义
//1  开关量输入 使用
extern uint8_t K_ManuAuto; // 手动.自动
extern uint8_t Q_VfStop;   // 空转
extern uint8_t Q_RunGPRS;
extern uint8_t F_ManualRunStop; // 手动启动停止标志
extern uint8_t F_TouchRunStop;  // 触摸启动停止

extern uint8_t F_DealSmall; // 正在处理小流量标志

extern uint8_t F_AskStop; // 请求停机标志
//extern uint8_t  F_AskExchange;			// 请求交换标志
//extern uint8_t  F_DelayCheckVvvfAlarm;	// 第一次上电延时检测变频器报警5秒种后再检测

extern uint8_t F_ModRealTime; // 修改实时时钟

//************com1**************
extern uint8_t Txd1Buffer[TXD1_MAX]; // 发送缓冲区
extern uint8_t Rcv1Buffer[RCV1_MAX]; // 接收缓冲区
extern uint8_t Tmp_Txd1Buffer[1], Tmp_Rxd1Buffer;
extern uint16_t T_NoRcv1Count; // 没有接收计数器
extern uint16_t C_NoRcv1Count;
extern uint16_t Rcv1Counter;   // 接收计数器//
extern uint16_t Txd1Counter;   // 发送计数器//
extern uint16_t BakRcv1Count;  // 接收计数器//
extern uint16_t Txd1Max;       // 有多少个字符需要发送//
extern uint16_t w_Txd1ChkSum;  // 发送校验和，lo,hi 两位//
extern uint16_t w_Com1RegAddr; // 串口1寄存器地址
//
extern uint8_t B_Com1Send;

extern uint8_t B_Com1Cmd03;
extern uint8_t B_Com1Cmd16;
extern uint8_t B_Com1Cmd01;
extern uint8_t B_Com1Cmd06;
extern uint8_t B_Com1Cmd73;

extern uint16_t C_Com1_MasterSendDelay;
extern uint16_t T_Com1_MasterSendDelay;
extern uint16_t C_Com1_MasterSendDelay2;
extern uint16_t T_Com1_MasterSendDelay2;
extern uint16_t C_Com1_MasterSendDelay3;
extern uint16_t T_Com1_MasterSendDelay3;

extern s16 Com1_Driver1_Queue_Rear;  //命令队列中的命令数量，队尾指针
extern s16 Com1_Driver2_Queue_Rear;  //命令队列中的命令数量，队尾指针
extern s16 Com1_Driver1_Queue_Front; //命令队列中的当前要出队的，队头指针
extern s16 Com1_Driver2_Queue_Front; //命令队列中的当前要出队的，队头指针

extern s16 Com2_Driver3_Queue_Rear;  //命令队列中的命令数量，队尾指针
extern s16 Com2_Driver4_Queue_Rear;  //命令队列中的命令数量，队尾指针
extern s16 Com2_Driver3_Queue_Front; //命令队列中的当前要出队的，队头指针
extern s16 Com2_Driver4_Queue_Front; //命令队列中的当前要出队的，队头指针

extern s16 Com3_Driver5_Queue_Rear;
extern s16 Com3_Driver6_Queue_Rear;
extern s16 Com3_Driver5_Queue_Front;
extern s16 Com3_Driver6_Queue_Front;

extern s16 Com4_Queue_Rear;  //命令队列中的命令队尾指针
extern s16 Com4_Queue_Front; //命令队列中的当前要出队的，队头指针

//************com2**************
extern uint8_t Txd2Buffer[TXD2_MAX]; // 发送缓冲区
extern uint8_t Rcv2Buffer[RCV2_MAX]; // 接收缓冲区
extern uint8_t Tmp_Txd2Buffer[1], Tmp_Rxd2Buffer;
extern uint16_t Rcv2Counter;   // 接收计数器//
extern uint16_t Txd2Counter;   // 发送计数器//
extern uint16_t BakRcv2Count;  // 接收计数器//
extern uint16_t Txd2Max;       // 有多少个字符需要发送//
extern uint16_t w_Txd2ChkSum;  // 发送校验和，lo,hi 两位//
extern uint16_t w_Com2RegAddr; // 串口1寄存器地址
//
extern uint8_t B_Com2Send;

extern uint8_t B_Com2Cmd03;
extern uint8_t B_Com2Cmd16;
extern uint8_t B_Com2Cmd01;
extern uint8_t B_Com2Cmd06;
extern uint8_t B_Com2Cmd73;
extern uint16_t T_NoRcv2Count; // 没有接收计数器
extern uint16_t C_NoRcv2Count;
extern uint16_t C_Com2_MasterSendDelay;
extern uint8_t T_Com2_MasterSendDelay;
extern uint16_t C_Com2_MasterSendDelay2;
extern uint8_t T_Com2_MasterSendDelay2;
extern uint16_t C_Com2_MasterSendDelay3;
extern uint8_t T_Com2_MasterSendDelay3;

//************com3**************
extern uint8_t Txd3Buffer[TXD3_MAX]; // 发送缓冲区
extern uint8_t Rcv3Buffer[RCV3_MAX]; // 接收缓冲区
extern uint8_t Tmp_Txd3Buffer[1], Tmp_Rxd3Buffer;
extern uint16_t Rcv3Counter;   // 接收计数器//
extern uint16_t Txd3Counter;   // 发送计数器//
extern uint16_t BakRcv3Count;  // 接收计数器//
extern uint16_t Txd3Max;       // 有多少个字符需要发送//
extern uint16_t w_Txd3ChkSum;  // 发送校验和，lo,hi 两位//
extern uint16_t w_Com3RegAddr; // 串口1寄存器地址
//
extern uint8_t B_Com3Send;

extern uint8_t B_Com3Cmd03;
extern uint8_t B_Com3Cmd16;
extern uint8_t B_Com3Cmd01;
extern uint8_t B_Com3Cmd06;
extern uint8_t B_Com3Cmd73;
extern uint16_t T_NoRcv3Count; // 没有接收计数器
extern uint16_t C_NoRcv3Count;
extern uint16_t C_Com3_MasterSendDelay;
extern uint8_t T_Com3_MasterSendDelay;
extern uint16_t C_Com3_MasterSendDelay2;
extern uint8_t T_Com3_MasterSendDelay2;
extern uint16_t C_Com3_MasterSendDelay3;
extern uint8_t T_Com3_MasterSendDelay3;

//************com4**************
extern uint8_t Txd4Buffer[TXD4_MAX]; // 发送缓冲区
extern uint8_t Rcv4Buffer[RCV4_MAX]; // 接收缓冲区
extern uint8_t Tmp_Rxd4Buffer;
extern uint16_t Rcv4Counter;   // 接收计数器//
extern uint16_t Txd4Counter;   // 发送计数器//
extern uint16_t BakRcv4Count;  // 接收计数器//
extern uint16_t Txd4Max;       // 有多少个字符需要发送//
extern uint16_t w_Txd4ChkSum;  // 发送校验和，lo,hi 两位//
extern uint16_t w_Com4RegAddr; // 串口1寄存器地址
//
extern uint8_t B_Com4Send;

extern uint8_t B_Com4Cmd03;
extern uint8_t B_Com4Cmd16;
extern uint8_t B_Com4Cmd01;
extern uint8_t B_Com4Cmd06;
extern uint8_t B_Com4Cmd73;
extern uint16_t T_NoRcv4Count; // 没有接收计数器
extern uint16_t C_NoRcv4Count;
extern uint16_t C_Com4_MasterSendDelay;
extern uint8_t T_Com4_MasterSendDelay;
extern uint16_t C_Com4_MasterSendDelay2;
extern uint8_t T_Com4_MasterSendDelay2;
extern uint16_t C_Com4_MasterSendDelay3;
extern uint8_t T_Com4_MasterSendDelay3;

//************com5**************
extern uint8_t Txd5Buffer[TXD5_MAX]; // 发送缓冲区
extern uint8_t Rcv5Buffer[RCV5_MAX]; // 接收缓冲区
extern uint8_t Tmp_Rxd5Buffer;
extern uint16_t Rcv5Counter;   // 接收计数器//
extern uint16_t Txd5Counter;   // 发送计数器//
extern uint16_t BakRcv5Count;  // 接收计数器//
extern uint16_t Txd5Max;       // 有多少个字符需要发送//
extern uint16_t w_Txd5ChkSum;  // 发送校验和，lo,hi 两位//
extern uint16_t w_Com5RegAddr; // 串口1寄存器地址
//
extern uint8_t B_Com5Send;

extern uint8_t B_Com5Cmd03;
extern uint8_t B_Com5Cmd16;
extern uint8_t B_Com5Cmd01;
extern uint8_t B_Com5Cmd06;
extern uint8_t B_Com5Cmd73;
extern uint16_t T_NoRcv5Count; // 没有接收计数器
extern uint16_t C_NoRcv5Count;
extern uint16_t C_Com5_MasterSendDelay;
extern uint8_t T_Com5_MasterSendDelay;
extern uint16_t C_Com5_MasterSendDelay2;
extern uint8_t T_Com5_MasterSendDelay2;
extern uint16_t C_Com5_MasterSendDelay3;
extern uint8_t T_Com5_MasterSendDelay3;

//************com6**************
extern uint8_t Txd6Buffer[TXD6_MAX]; // 发送缓冲区
extern uint8_t Rcv6Buffer[RCV6_MAX]; // 接收缓冲区
extern uint8_t Tmp_Rxd6Buffer;
extern uint16_t Rcv6Counter;   // 接收计数器//
extern uint16_t Txd6Counter;   // 发送计数器//
extern uint16_t BakRcv6Count;  // 接收计数器//
extern uint16_t Txd6Max;       // 有多少个字符需要发送//
extern uint16_t w_Txd6ChkSum;  // 发送校验和，lo,hi 两位//
extern uint16_t w_Com6RegAddr; // 串口1寄存器地址
//
extern uint8_t B_Com6Send;

extern uint8_t B_Com6Cmd03;
extern uint8_t B_Com6Cmd16;
extern uint8_t B_Com6Cmd01;
extern uint8_t B_Com6Cmd06;
extern uint8_t B_Com6Cmd73;
extern uint16_t T_NoRcv6Count; // 没有接收计数器
extern uint16_t C_NoRcv6Count;
extern uint16_t C_Com6_MasterSendDelay;
extern uint8_t T_Com6_MasterSendDelay;
extern uint16_t C_Com6_MasterSendDelay2;
extern uint8_t T_Com6_MasterSendDelay2;
extern uint16_t C_Com6_MasterSendDelay3;
extern uint8_t T_Com6_MasterSendDelay3;

//**********WK_COM1**********
extern uint8_t WK_Txd1Buffer[TXD1_MAX]; // 发送缓冲区
extern uint8_t WK_Rcv1Buffer[RCV1_MAX]; // 接收缓冲区
extern uint8_t WK_Tmp_Txd1Buffer[1], WK_Tmp_Rxd1Buffer;
extern uint16_t WK_Rcv1Counter;   // 接收计数器//
extern uint16_t WK_Txd1Counter;   // 发送计数器//
extern uint16_t WK_BakRcv1Count;  // 接收计数器//
extern uint16_t WK_Txd1Max;       // 有多少个字符需要发送//
extern uint16_t WK_w_Txd1ChkSum;  // 发送校验和，lo,hi 两位//
extern uint16_t WK_w_Com1RegAddr; // 串口1寄存器地址

//
extern uint8_t WK_B_Com1Cmd03;
extern uint8_t WK_B_Com1Cmd16;
extern uint8_t WK_B_Com1Cmd01;
extern uint8_t WK_B_Com1Cmd06;
extern uint16_t T_WK_NoRcv1Count; // 没有接收计数器
extern uint16_t C_WK_NoRcv1Count;

//**********WK_COM2**********
extern uint8_t WK_Txd2Buffer[TXD2_MAX]; // 发送缓冲区
extern uint8_t WK_Rcv2Buffer[RCV2_MAX]; // 接收缓冲区
extern uint8_t WK_Tmp_Txd2Buffer[1], WK_Tmp_Rxd2Buffer;
extern uint16_t WK_Rcv2Counter;   // 接收计数器//
extern uint16_t WK_Txd2Counter;   // 发送计数器//
extern uint16_t WK_BakRcv2Count;  // 接收计数器//
extern uint16_t WK_Txd2Max;       // 有多少个字符需要发送//
extern uint16_t WK_w_Txd2ChkSum;  // 发送校验和，lo,hi 两位//
extern uint16_t WK_w_Com2RegAddr; // 串口2寄存器地址

//
extern uint8_t WK_B_Com2Cmd03;
extern uint8_t WK_B_Com2Cmd16;
extern uint8_t WK_B_Com2Cmd01;
extern uint8_t WK_B_Com2Cmd06;
extern uint16_t T_WK_NoRcv2Count; // 没有接收计数器
extern uint16_t C_WK_NoRcv2Count;

//**********WK_COM3**********
extern uint8_t WK_Txd3Buffer[TXD3_MAX]; // 发送缓冲区
extern uint8_t WK_Rcv3Buffer[RCV3_MAX]; // 接收缓冲区
extern uint8_t WK_Tmp_Txd3Buffer[1], WK_Tmp_Rxd3Buffer;
extern uint16_t WK_Rcv3Counter;   // 接收计数器//
extern uint16_t WK_Txd3Counter;   // 发送计数器//
extern uint16_t WK_BakRcv3Count;  // 接收计数器//
extern uint16_t WK_Txd3Max;       // 有多少个字符需要发送//
extern uint16_t WK_w_Txd3ChkSum;  // 发送校验和，lo,hi 两位//
extern uint16_t WK_w_Com3RegAddr; // 串口3寄存器地址

//
extern uint8_t WK_B_Com3Cmd03;
extern uint8_t WK_B_Com3Cmd16;
extern uint8_t WK_B_Com3Cmd01;
extern uint8_t WK_B_Com3Cmd06;
extern uint16_t T_WK_NoRcv3Count; // 没有接收计数器
extern uint16_t C_WK_NoRcv3Count;

//**********WK_COM4**********
extern uint8_t WK_Txd4Buffer[TXD4_MAX]; // 发送缓冲区
extern uint8_t WK_Rcv4Buffer[RCV4_MAX]; // 接收缓冲区
extern uint8_t WK_Tmp_Txd4Buffer[1], WK_Tmp_Rxd4Buffer;
extern uint16_t WK_Rcv4Counter;   // 接收计数器//
extern uint16_t WK_Txd4Counter;   // 发送计数器//
extern uint16_t WK_BakRcv4Count;  // 接收计数器//
extern uint16_t WK_Txd4Max;       // 有多少个字符需要发送//
extern uint16_t WK_w_Txd4ChkSum;  // 发送校验和，lo,hi 两位//
extern uint16_t WK_w_Com4RegAddr; // 串口4寄存器地址

//
extern uint8_t WK_B_Com4Cmd03;
extern uint8_t WK_B_Com4Cmd16;
extern uint8_t WK_B_Com4Cmd01;
extern uint8_t WK_B_Com4Cmd06;
extern uint16_t T_WK_NoRcv4Count; // 没有接收计数器
extern uint16_t C_WK_NoRcv4Count;

extern uint16_t T_SavePos_Delay1;
extern uint16_t C_SavePos_Delay1;
extern uint16_t T_SavePos_Delay2;
extern uint16_t C_SavePos_Delay2;
extern uint16_t T_SavePos_Delay3;
extern uint16_t C_SavePos_Delay3;
extern uint16_t T_SavePos_Delay4;
extern uint16_t C_SavePos_Delay4;
extern uint16_t T_SavePos_Delay5;
extern uint16_t C_SavePos_Delay5;
extern uint16_t T_SavePos_Delay6;
extern uint16_t C_SavePos_Delay6;

extern uint8_t Driver1_Save_P;
extern uint8_t Driver2_Save_P;
extern uint8_t Driver3_Save_P;
extern uint8_t Driver4_Save_P;
extern uint8_t Driver5_Save_P;
extern uint8_t Driver6_Save_P;

extern uint8_t F_Driver1_WrPos_ErrStatus;
extern uint8_t F_Driver2_WrPos_ErrStatus;
extern uint8_t F_Driver3_WrPos_ErrStatus;
extern uint8_t F_Driver4_WrPos_ErrStatus;
extern uint8_t F_Driver5_WrPos_ErrStatus;
extern uint8_t F_Driver6_WrPos_ErrStatus;

extern uint8_t Driver1_delay_F; //电机暂停延时标志，=1表示正在暂停；=0,表示暂停结束

extern uint8_t DO_delay_F; //DO输出延时标志
extern uint8_t F_Open_Hand;
extern uint8_t F_Close_Hand;

extern uint32_t Driver1_delay_ms_Count; //电机暂停延时计数，ms
extern uint32_t Driver2_delay_ms_Count;
extern uint32_t Driver3_delay_ms_Count;
extern uint32_t Driver4_delay_ms_Count;
extern uint32_t Driver5_delay_ms_Count;
extern uint32_t Driver6_delay_ms_Count;

extern uint32_t T_Driver1_delay;      //延时定时器
extern uint32_t C_Driver1_delayCount; //延时计数
extern uint32_t T_Driver2_delay;
extern uint32_t C_Driver2_delayCount;
extern uint32_t T_Driver3_delay;
extern uint32_t C_Driver3_delayCount;
extern uint32_t T_Driver4_delay;
extern uint32_t C_Driver4_delayCount;
extern uint32_t T_Driver5_delay;
extern uint32_t C_Driver5_delayCount;
extern uint32_t T_Driver6_delay;
extern uint32_t C_Driver6_delayCount;

extern uint32_t T_DO_delay;           //DO输出延时定时器
extern uint32_t C_DO_delayCount;      //DO输出延时计数
extern uint32_t T_DO_Open_delay;      //DO输出持续定时器
extern uint32_t C_DO_Open_delayCount; //DO输出持续计数

extern uint16_t T_Driver1_FillCMD;
extern uint16_t C_Driver1_FillCMD;
extern uint16_t T_Driver2_FillCMD;
extern uint16_t C_Driver2_FillCMD;
extern uint16_t T_Driver3_FillCMD;
extern uint16_t C_Driver3_FillCMD;
extern uint16_t T_Driver4_FillCMD;
extern uint16_t C_Driver4_FillCMD;
extern uint16_t T_Driver5_FillCMD;
extern uint16_t C_Driver5_FillCMD;
extern uint16_t T_Driver6_FillCMD;
extern uint16_t C_Driver6_FillCMD;

extern uint16_t T_StepAutoDelay;
extern uint16_t C_T_StepAutoDelayDelay;
extern uint16_t T_StepAutoDelay2;
extern uint16_t C_T_StepAutoDelayDelay2;

extern uint16_t T_ResetDelay;
extern uint16_t C_ResetDelay;
extern uint16_t T_ResetDelay2;
extern uint16_t C_ResetDelay2;
extern uint16_t T_ResetDelay3;
extern uint16_t C_ResetDelay3;
extern uint8_t F_SendStopCMD2;

extern uint16_t T_Reset_Driver;
extern uint16_t C_Reset_Driver;
extern uint16_t T_Reset_Driver2;
extern uint16_t C_Reset_Driver2;
extern uint8_t F_AutoReset;
extern uint8_t F_SendStopCMD;

extern uint8_t Driver1_Write_Sort; //驱动器1写位置段顺序，0写第1段，1写第2段
extern uint8_t Driver2_Write_Sort;
extern uint8_t Driver3_Write_Sort;
extern uint8_t Driver4_Write_Sort;
extern uint8_t Driver5_Write_Sort;
extern uint8_t Driver6_Write_Sort;
extern uint8_t F_Hand_Status; //电磁阀状态

extern uint8_t F_Driver1_notBrake; //1#伺服刹车信号
extern uint8_t F_Driver2_notBrake;
extern uint8_t F_Driver3_notBrake;
extern uint8_t F_Driver4_notBrake;
extern uint8_t F_Driver5_notBrake;
extern uint8_t F_Driver6_notBrake;
extern uint8_t F_Driver_All_notBrake;

extern uint16_t C_AllStop;

extern uint8_t F_Sync_6_axis; //6轴同步输出标志
extern uint8_t F_Resetting;   //正在复位标志

extern uint8_t F_Driver1_Send_Cmd; //控制器给1#伺服驱动器发出位置命令标志，=1表示已发送命令
extern uint8_t F_Driver2_Send_Cmd;
extern uint8_t F_Driver3_Send_Cmd;
extern uint8_t F_Driver4_Send_Cmd;
extern uint8_t F_Driver5_Send_Cmd;
extern uint8_t F_Driver6_Send_Cmd;

extern uint8_t F_Driver1_Timeout; //1#伺服驱动器发送命令超时标志，=1表示超时
extern uint8_t F_Driver2_Timeout;
extern uint8_t F_Driver3_Timeout;
extern uint8_t F_Driver4_Timeout;
extern uint8_t F_Driver5_Timeout;
extern uint8_t F_Driver6_Timeout;

extern uint8_t F_Driver1_Cmd_Err; //1#伺服驱动器位置命令错误标志，=1表示错误
extern uint8_t F_Driver2_Cmd_Err;
extern uint8_t F_Driver3_Cmd_Err;
extern uint8_t F_Driver4_Cmd_Err;
extern uint8_t F_Driver5_Cmd_Err;
extern uint8_t F_Driver6_Cmd_Err;
extern uint8_t F_HaveDriver_Cmd_Err; //有伺服写入位置错误标志

extern uint8_t F_Driver1_Cmd_Con_Err; //1#伺服驱动器控制命令错误标志，=1表示错误
extern uint8_t F_Driver2_Cmd_Con_Err;
extern uint8_t F_Driver3_Cmd_Con_Err;
extern uint8_t F_Driver4_Cmd_Con_Err;
extern uint8_t F_Driver5_Cmd_Con_Err;
extern uint8_t F_Driver6_Cmd_Con_Err;
extern uint8_t F_HaveDriver_Cmd_Con_Err; //有伺服写入位置错误标志

extern uint8_t Driver1_Cmd_PosNo; //1#伺服驱动器发送的命令位置号，位置0或者位置1
extern uint8_t Driver2_Cmd_PosNo;
extern uint8_t Driver3_Cmd_PosNo;
extern uint8_t Driver4_Cmd_PosNo;
extern uint8_t Driver5_Cmd_PosNo;
extern uint8_t Driver6_Cmd_PosNo;

extern uint8_t Driver1_Cmd_Status; //1#伺服驱动器P8910状态
extern uint8_t Driver2_Cmd_Status;
extern uint8_t Driver3_Cmd_Status;
extern uint8_t Driver4_Cmd_Status;
extern uint8_t Driver5_Cmd_Status;
extern uint8_t Driver6_Cmd_Status;

extern uint16_t T_Driver1_WriteCMD;
extern uint16_t C_Driver1_WriteCMD;
extern uint16_t T_Driver2_WriteCMD;
extern uint16_t C_Driver2_WriteCMD;
extern uint16_t T_Driver3_WriteCMD;
extern uint16_t C_Driver3_WriteCMD;
extern uint16_t T_Driver4_WriteCMD;
extern uint16_t C_Driver4_WriteCMD;
extern uint16_t T_Driver5_WriteCMD;
extern uint16_t C_Driver5_WriteCMD;
extern uint16_t T_Driver6_WriteCMD;
extern uint16_t C_Driver6_WriteCMD;

extern uint8_t Driver1_Cmd_Data[9]; //1#伺服驱动器命令数据,4个字节为脉冲，2个字节为速度，2个字节为加减速时间，最后一个字节为位置号（0/1）
extern uint8_t Driver2_Cmd_Data[9];
extern uint8_t Driver3_Cmd_Data[9];
extern uint8_t Driver4_Cmd_Data[9];
extern uint8_t Driver5_Cmd_Data[9];
extern uint8_t Driver6_Cmd_Data[9];

extern u8 Driver1_Pos_Start_Sort; //1#伺服，=0，表示还未写入到命令缓冲区；=1，表示已经写入到命令缓冲区；=2，表示已经发送
extern u8 Driver2_Pos_Start_Sort;
extern u8 Driver3_Pos_Start_Sort;
extern u8 Driver4_Pos_Start_Sort;
extern u8 Driver5_Pos_Start_Sort;
extern u8 Driver6_Pos_Start_Sort;

extern uint8_t Driver1_Status_Sort; //1#伺服，=2，表示已经发送；=3，表示已经接收
extern uint8_t Driver2_Status_Sort;
extern uint8_t Driver3_Status_Sort;
extern uint8_t Driver4_Status_Sort;
extern uint8_t Driver5_Status_Sort;
extern uint8_t Driver6_Status_Sort;

//YLS 2020.06.25
extern uint8_t F_TouchReSet;      //触摸复位
extern uint8_t F_AskReset;        //请求复位
extern uint8_t F_Reseting;        //正在复位标志
extern uint8_t F_TouchForceReSet; //强制复位
extern uint8_t F_AskForceReset;   //请求复位
extern uint8_t F_ForceReseting;   //正在复位标志
extern uint8_t F_Reseting;        //正在复位标志

extern uint8_t F_StepMode;   //=1，进入手动模式标志
extern uint8_t F_PowerOnRun; //是否上电自动运行标志

extern uint8_t F_Stoping;  //正在停止标志
extern uint8_t F_Starting; //正在启动标志

extern uint8_t F_Encoder_Read; //编码器数据已读取标志
extern uint8_t F_Status_Read;  //VDO状态读取标志

extern s32 *arr_p1;
extern s32 *arrp_p1_Last; //保存上一条指令的命令指针

extern uint32_t Reset_time_Max; //复位运行时间
extern uint8_t F_AllRdy;        //所有电机都Ready标志

extern uint8_t F_AllRun;  //所有电机处于RUN标志
extern uint8_t F_Reseted; //复位到原点标志

extern uint8_t *Com_Write_p;
//定义每台电机的写命令缓冲区
//每条定义：序号+命令长度+命令内容
extern uint8_t Com1_Driver1_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM]; //COM_CMD_SIZE=20，COM_CMD_NUM=30.定义30条写命令，每条20个字节
extern uint8_t Com1_Driver2_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM]; //COM_CMD_SIZE=20，COM_CMD_NUM=30.定义30条写命令，每条20个字节
extern uint8_t Com2_Driver3_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];
extern uint8_t Com2_Driver4_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];
extern uint8_t Com3_Driver5_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];
extern uint8_t Com3_Driver6_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];
extern uint8_t Com4_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];

extern uc16 w_ParBootLst[];
extern uint32_t Driver1_Cmd_Group1[];
extern uint32_t Driver2_Cmd_Group1[];
extern uint32_t Driver3_Cmd_Group1[];
extern uint32_t Driver4_Cmd_Group1[];
extern uint32_t Driver5_Cmd_Group1[];
extern uint32_t Driver6_Cmd_Group1[];

extern sc32 Pos_Init[];
extern sc32 Pos_Cmd_Group1[];
extern sc32 Pos_Cmd_Group2[];
extern sc32 Pos_Cmd_Group3[];
extern sc32 Pos_Cmd_Group4[];
extern sc32 Pos_Cmd_Group5[];

// extern	uint32_t step_to_run; //要匀速运行的步数       总共运行步数 = ACCELERATED_SPEED_LENGTH*2 + step_to_run
// extern	float fre[ACCELERATED_SPEED_LENGTH]; //数组存储加速过程中每一步的频率
// extern	unsigned short period[ACCELERATED_SPEED_LENGTH]; //数组储存加速过程中每一步定时器的自动装载值

#endif /* __GLOBALV_EXTERN_H */
