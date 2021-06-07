
/************************************************************************
*			CopyRight  2006.3.24   ��������Ӳ����   				    *
************************************************************************
* �ļ�����: GlobalV_Extern.h						
* �ļ��汾��1.00
* ������Ա���ܳ���					  							
* �������ڣ�2006.3.24											
* ����������ȫ�ֱ���������
* ���Ӳ����C80F1F020,ISD4004-08M,ISL1208,M25P80			
* ���滷����KEILC51 V7.09
* C ��������KEIL V6.12 COMPILER						
* �޸ļ�¼��2006/04/01 08:19   �ܳ���
*			
*			 
************************************************************************/
#ifndef __GLOBALV_EXTERN_H
#define __GLOBALV_EXTERN_H

#include "stm32f4xx.h"
#include "typedef.h"
#include "GlobalConst.h"

// (�������)
extern uint8_t VfNo;        // ��Ƶ�����
extern uint8_t VfNext;      // ������������һ̨��Ƶ��
extern uint8_t SoftClock[]; // ����ʱ��
//
extern uint8_t Vf_No;
extern uint8_t F_AskStop1;
extern uint8_t RealClock[];      // ʵʱʱ��
extern uint8_t B_PumpBad[];      // B_PumpBad[0]δʹ��,��1-5����־
extern uint8_t B_PumpManu[];     // ���ֶ�״̬1-5��־(������)
extern uint8_t B_SaveSetP;       // �����趨ѹ�� ��־�ֽ�
extern uint8_t B_SaveVoiceValue; // ����������С ��־�ֽ�
extern uint8_t B_ReadVfPar;      // ��ȡ��Ƶ������
extern uint8_t B_ModSelParValue; // �޸�ѡ��Ĳ���ֵ
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

extern uint8_t DInb[];  // ��������������б���
extern uint8_t DOutb[]; // ��������������б���
//extern uint8_t  NBComBuf[];		// ����ͨѶ���� 2007.10.30
extern uint16_t w_AI[]; // ģ������������б���
//extern uint16_t  w_AQ[];			// ģ������������б���
extern uint16_t w_ParLst[];     // �������б���
extern s32 w_ParLst_Drive[];    //YLS 2020.06.23
extern uint32_t w_ParLst_POS[]; //YLS 2020.11.27

// 2008.12.11
extern uint8_t B_SelCS;
extern uint16_t w_FaultNo[];
extern uint16_t w_ModRealTimeNo; // �޸�ʵʱʱ�ӵ����
extern uint16_t w_PreRecNo;      // �ϴμ�¼��

//
// ����Ϊ���ⲿ�ֽڶ���Ŀ��������룬�������������
// Ӳ������
//1  ���������� ʹ��
extern uint8_t K_ManuAuto; // �ֶ�.�Զ�
extern uint8_t Q_VfStop;   // ��ת
extern uint8_t Q_RunGPRS;
extern uint8_t F_ManualRunStop; // �ֶ�����ֹͣ��־
extern uint8_t F_TouchRunStop;  // ��������ֹͣ

extern uint8_t F_DealSmall; // ���ڴ���С������־

extern uint8_t F_AskStop; // ����ͣ����־
//extern uint8_t  F_AskExchange;			// ���󽻻���־
//extern uint8_t  F_DelayCheckVvvfAlarm;	// ��һ���ϵ���ʱ����Ƶ������5���ֺ��ټ��

extern uint8_t F_ModRealTime; // �޸�ʵʱʱ��

//************com1**************
extern uint8_t Txd1Buffer[TXD1_MAX]; // ���ͻ�����
extern uint8_t Rcv1Buffer[RCV1_MAX]; // ���ջ�����
extern uint8_t Tmp_Txd1Buffer[1], Tmp_Rxd1Buffer;
extern uint16_t T_NoRcv1Count; // û�н��ռ�����
extern uint16_t C_NoRcv1Count;
extern uint16_t Rcv1Counter;   // ���ռ�����//
extern uint16_t Txd1Counter;   // ���ͼ�����//
extern uint16_t BakRcv1Count;  // ���ռ�����//
extern uint16_t Txd1Max;       // �ж��ٸ��ַ���Ҫ����//
extern uint16_t w_Txd1ChkSum;  // ����У��ͣ�lo,hi ��λ//
extern uint16_t w_Com1RegAddr; // ����1�Ĵ�����ַ
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

extern s16 Com1_Driver1_Queue_Rear;  //��������е�������������βָ��
extern s16 Com1_Driver2_Queue_Rear;  //��������е�������������βָ��
extern s16 Com1_Driver1_Queue_Front; //��������еĵ�ǰҪ���ӵģ���ͷָ��
extern s16 Com1_Driver2_Queue_Front; //��������еĵ�ǰҪ���ӵģ���ͷָ��

extern s16 Com2_Driver3_Queue_Rear;  //��������е�������������βָ��
extern s16 Com2_Driver4_Queue_Rear;  //��������е�������������βָ��
extern s16 Com2_Driver3_Queue_Front; //��������еĵ�ǰҪ���ӵģ���ͷָ��
extern s16 Com2_Driver4_Queue_Front; //��������еĵ�ǰҪ���ӵģ���ͷָ��

extern s16 Com3_Driver5_Queue_Rear;
extern s16 Com3_Driver6_Queue_Rear;
extern s16 Com3_Driver5_Queue_Front;
extern s16 Com3_Driver6_Queue_Front;

extern s16 Com4_Queue_Rear;  //��������е������βָ��
extern s16 Com4_Queue_Front; //��������еĵ�ǰҪ���ӵģ���ͷָ��

//************com2**************
extern uint8_t Txd2Buffer[TXD2_MAX]; // ���ͻ�����
extern uint8_t Rcv2Buffer[RCV2_MAX]; // ���ջ�����
extern uint8_t Tmp_Txd2Buffer[1], Tmp_Rxd2Buffer;
extern uint16_t Rcv2Counter;   // ���ռ�����//
extern uint16_t Txd2Counter;   // ���ͼ�����//
extern uint16_t BakRcv2Count;  // ���ռ�����//
extern uint16_t Txd2Max;       // �ж��ٸ��ַ���Ҫ����//
extern uint16_t w_Txd2ChkSum;  // ����У��ͣ�lo,hi ��λ//
extern uint16_t w_Com2RegAddr; // ����1�Ĵ�����ַ
//
extern uint8_t B_Com2Send;

extern uint8_t B_Com2Cmd03;
extern uint8_t B_Com2Cmd16;
extern uint8_t B_Com2Cmd01;
extern uint8_t B_Com2Cmd06;
extern uint8_t B_Com2Cmd73;
extern uint16_t T_NoRcv2Count; // û�н��ռ�����
extern uint16_t C_NoRcv2Count;
extern uint16_t C_Com2_MasterSendDelay;
extern uint8_t T_Com2_MasterSendDelay;
extern uint16_t C_Com2_MasterSendDelay2;
extern uint8_t T_Com2_MasterSendDelay2;
extern uint16_t C_Com2_MasterSendDelay3;
extern uint8_t T_Com2_MasterSendDelay3;

//************com3**************
extern uint8_t Txd3Buffer[TXD3_MAX]; // ���ͻ�����
extern uint8_t Rcv3Buffer[RCV3_MAX]; // ���ջ�����
extern uint8_t Tmp_Txd3Buffer[1], Tmp_Rxd3Buffer;
extern uint16_t Rcv3Counter;   // ���ռ�����//
extern uint16_t Txd3Counter;   // ���ͼ�����//
extern uint16_t BakRcv3Count;  // ���ռ�����//
extern uint16_t Txd3Max;       // �ж��ٸ��ַ���Ҫ����//
extern uint16_t w_Txd3ChkSum;  // ����У��ͣ�lo,hi ��λ//
extern uint16_t w_Com3RegAddr; // ����1�Ĵ�����ַ
//
extern uint8_t B_Com3Send;

extern uint8_t B_Com3Cmd03;
extern uint8_t B_Com3Cmd16;
extern uint8_t B_Com3Cmd01;
extern uint8_t B_Com3Cmd06;
extern uint8_t B_Com3Cmd73;
extern uint16_t T_NoRcv3Count; // û�н��ռ�����
extern uint16_t C_NoRcv3Count;
extern uint16_t C_Com3_MasterSendDelay;
extern uint8_t T_Com3_MasterSendDelay;
extern uint16_t C_Com3_MasterSendDelay2;
extern uint8_t T_Com3_MasterSendDelay2;
extern uint16_t C_Com3_MasterSendDelay3;
extern uint8_t T_Com3_MasterSendDelay3;

//************com4**************
extern uint8_t Txd4Buffer[TXD4_MAX]; // ���ͻ�����
extern uint8_t Rcv4Buffer[RCV4_MAX]; // ���ջ�����
extern uint8_t Tmp_Rxd4Buffer;
extern uint16_t Rcv4Counter;   // ���ռ�����//
extern uint16_t Txd4Counter;   // ���ͼ�����//
extern uint16_t BakRcv4Count;  // ���ռ�����//
extern uint16_t Txd4Max;       // �ж��ٸ��ַ���Ҫ����//
extern uint16_t w_Txd4ChkSum;  // ����У��ͣ�lo,hi ��λ//
extern uint16_t w_Com4RegAddr; // ����1�Ĵ�����ַ
//
extern uint8_t B_Com4Send;

extern uint8_t B_Com4Cmd03;
extern uint8_t B_Com4Cmd16;
extern uint8_t B_Com4Cmd01;
extern uint8_t B_Com4Cmd06;
extern uint8_t B_Com4Cmd73;
extern uint16_t T_NoRcv4Count; // û�н��ռ�����
extern uint16_t C_NoRcv4Count;
extern uint16_t C_Com4_MasterSendDelay;
extern uint8_t T_Com4_MasterSendDelay;
extern uint16_t C_Com4_MasterSendDelay2;
extern uint8_t T_Com4_MasterSendDelay2;
extern uint16_t C_Com4_MasterSendDelay3;
extern uint8_t T_Com4_MasterSendDelay3;

//************com5**************
extern uint8_t Txd5Buffer[TXD5_MAX]; // ���ͻ�����
extern uint8_t Rcv5Buffer[RCV5_MAX]; // ���ջ�����
extern uint8_t Tmp_Rxd5Buffer;
extern uint16_t Rcv5Counter;   // ���ռ�����//
extern uint16_t Txd5Counter;   // ���ͼ�����//
extern uint16_t BakRcv5Count;  // ���ռ�����//
extern uint16_t Txd5Max;       // �ж��ٸ��ַ���Ҫ����//
extern uint16_t w_Txd5ChkSum;  // ����У��ͣ�lo,hi ��λ//
extern uint16_t w_Com5RegAddr; // ����1�Ĵ�����ַ
//
extern uint8_t B_Com5Send;

extern uint8_t B_Com5Cmd03;
extern uint8_t B_Com5Cmd16;
extern uint8_t B_Com5Cmd01;
extern uint8_t B_Com5Cmd06;
extern uint8_t B_Com5Cmd73;
extern uint16_t T_NoRcv5Count; // û�н��ռ�����
extern uint16_t C_NoRcv5Count;
extern uint16_t C_Com5_MasterSendDelay;
extern uint8_t T_Com5_MasterSendDelay;
extern uint16_t C_Com5_MasterSendDelay2;
extern uint8_t T_Com5_MasterSendDelay2;
extern uint16_t C_Com5_MasterSendDelay3;
extern uint8_t T_Com5_MasterSendDelay3;

//************com6**************
extern uint8_t Txd6Buffer[TXD6_MAX]; // ���ͻ�����
extern uint8_t Rcv6Buffer[RCV6_MAX]; // ���ջ�����
extern uint8_t Tmp_Rxd6Buffer;
extern uint16_t Rcv6Counter;   // ���ռ�����//
extern uint16_t Txd6Counter;   // ���ͼ�����//
extern uint16_t BakRcv6Count;  // ���ռ�����//
extern uint16_t Txd6Max;       // �ж��ٸ��ַ���Ҫ����//
extern uint16_t w_Txd6ChkSum;  // ����У��ͣ�lo,hi ��λ//
extern uint16_t w_Com6RegAddr; // ����1�Ĵ�����ַ
//
extern uint8_t B_Com6Send;

extern uint8_t B_Com6Cmd03;
extern uint8_t B_Com6Cmd16;
extern uint8_t B_Com6Cmd01;
extern uint8_t B_Com6Cmd06;
extern uint8_t B_Com6Cmd73;
extern uint16_t T_NoRcv6Count; // û�н��ռ�����
extern uint16_t C_NoRcv6Count;
extern uint16_t C_Com6_MasterSendDelay;
extern uint8_t T_Com6_MasterSendDelay;
extern uint16_t C_Com6_MasterSendDelay2;
extern uint8_t T_Com6_MasterSendDelay2;
extern uint16_t C_Com6_MasterSendDelay3;
extern uint8_t T_Com6_MasterSendDelay3;

//**********WK_COM1**********
extern uint8_t WK_Txd1Buffer[TXD1_MAX]; // ���ͻ�����
extern uint8_t WK_Rcv1Buffer[RCV1_MAX]; // ���ջ�����
extern uint8_t WK_Tmp_Txd1Buffer[1], WK_Tmp_Rxd1Buffer;
extern uint16_t WK_Rcv1Counter;   // ���ռ�����//
extern uint16_t WK_Txd1Counter;   // ���ͼ�����//
extern uint16_t WK_BakRcv1Count;  // ���ռ�����//
extern uint16_t WK_Txd1Max;       // �ж��ٸ��ַ���Ҫ����//
extern uint16_t WK_w_Txd1ChkSum;  // ����У��ͣ�lo,hi ��λ//
extern uint16_t WK_w_Com1RegAddr; // ����1�Ĵ�����ַ

//
extern uint8_t WK_B_Com1Cmd03;
extern uint8_t WK_B_Com1Cmd16;
extern uint8_t WK_B_Com1Cmd01;
extern uint8_t WK_B_Com1Cmd06;
extern uint16_t T_WK_NoRcv1Count; // û�н��ռ�����
extern uint16_t C_WK_NoRcv1Count;

//**********WK_COM2**********
extern uint8_t WK_Txd2Buffer[TXD2_MAX]; // ���ͻ�����
extern uint8_t WK_Rcv2Buffer[RCV2_MAX]; // ���ջ�����
extern uint8_t WK_Tmp_Txd2Buffer[1], WK_Tmp_Rxd2Buffer;
extern uint16_t WK_Rcv2Counter;   // ���ռ�����//
extern uint16_t WK_Txd2Counter;   // ���ͼ�����//
extern uint16_t WK_BakRcv2Count;  // ���ռ�����//
extern uint16_t WK_Txd2Max;       // �ж��ٸ��ַ���Ҫ����//
extern uint16_t WK_w_Txd2ChkSum;  // ����У��ͣ�lo,hi ��λ//
extern uint16_t WK_w_Com2RegAddr; // ����2�Ĵ�����ַ

//
extern uint8_t WK_B_Com2Cmd03;
extern uint8_t WK_B_Com2Cmd16;
extern uint8_t WK_B_Com2Cmd01;
extern uint8_t WK_B_Com2Cmd06;
extern uint16_t T_WK_NoRcv2Count; // û�н��ռ�����
extern uint16_t C_WK_NoRcv2Count;

//**********WK_COM3**********
extern uint8_t WK_Txd3Buffer[TXD3_MAX]; // ���ͻ�����
extern uint8_t WK_Rcv3Buffer[RCV3_MAX]; // ���ջ�����
extern uint8_t WK_Tmp_Txd3Buffer[1], WK_Tmp_Rxd3Buffer;
extern uint16_t WK_Rcv3Counter;   // ���ռ�����//
extern uint16_t WK_Txd3Counter;   // ���ͼ�����//
extern uint16_t WK_BakRcv3Count;  // ���ռ�����//
extern uint16_t WK_Txd3Max;       // �ж��ٸ��ַ���Ҫ����//
extern uint16_t WK_w_Txd3ChkSum;  // ����У��ͣ�lo,hi ��λ//
extern uint16_t WK_w_Com3RegAddr; // ����3�Ĵ�����ַ

//
extern uint8_t WK_B_Com3Cmd03;
extern uint8_t WK_B_Com3Cmd16;
extern uint8_t WK_B_Com3Cmd01;
extern uint8_t WK_B_Com3Cmd06;
extern uint16_t T_WK_NoRcv3Count; // û�н��ռ�����
extern uint16_t C_WK_NoRcv3Count;

//**********WK_COM4**********
extern uint8_t WK_Txd4Buffer[TXD4_MAX]; // ���ͻ�����
extern uint8_t WK_Rcv4Buffer[RCV4_MAX]; // ���ջ�����
extern uint8_t WK_Tmp_Txd4Buffer[1], WK_Tmp_Rxd4Buffer;
extern uint16_t WK_Rcv4Counter;   // ���ռ�����//
extern uint16_t WK_Txd4Counter;   // ���ͼ�����//
extern uint16_t WK_BakRcv4Count;  // ���ռ�����//
extern uint16_t WK_Txd4Max;       // �ж��ٸ��ַ���Ҫ����//
extern uint16_t WK_w_Txd4ChkSum;  // ����У��ͣ�lo,hi ��λ//
extern uint16_t WK_w_Com4RegAddr; // ����4�Ĵ�����ַ

//
extern uint8_t WK_B_Com4Cmd03;
extern uint8_t WK_B_Com4Cmd16;
extern uint8_t WK_B_Com4Cmd01;
extern uint8_t WK_B_Com4Cmd06;
extern uint16_t T_WK_NoRcv4Count; // û�н��ռ�����
extern uint16_t C_WK_NoRcv4Count;

// extern	uint32_t step_to_run; //Ҫ�������еĲ���       �ܹ����в��� = ACCELERATED_SPEED_LENGTH*2 + step_to_run
// extern	float fre[ACCELERATED_SPEED_LENGTH]; //����洢���ٹ�����ÿһ����Ƶ��
// extern	unsigned short period[ACCELERATED_SPEED_LENGTH]; //���鴢����ٹ�����ÿһ����ʱ�����Զ�װ��ֵ

#endif /* __GLOBALV_EXTERN_H */
