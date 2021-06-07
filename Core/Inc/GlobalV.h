
/************************************************************************
*			CopyRight  2006.3.24   ��������Ӳ����   				    *
************************************************************************
* �ļ�����:GlobalV.h
* �ļ��汾��1.00
* ������Ա���ܳ���					  							
* �������ڣ�2006.3.24											
* ����������ȫ�ֱ������塣
* ���Ӳ����C80F1F020,ISD4004-08M,ISL1208,M25P80			
* ���滷����KEILC51 V7.09
* C ��������KEIL V6.12 COMPILER						
* �޸ļ�¼��2006/04/01 08:19   �ܳ���
*			
*			 
************************************************************************/
#ifndef __GLOBALV_H
#define __GLOBALV_H

#include "GlobalConst.h"
#include "stm32f4xx.h"
#include "typedef.h"

// (�������)
uint8_t VfNo;             // ��Ƶ�����
uint8_t VfNext;           // ������������һ̨��Ƶ��
uint8_t SoftClock[8];     // ����ʱ��
uint16_t w_MinRunFreqHex; // Ƶ��������СHexֵ
float f_I_Mx;             // ������ǰֵ
//
uint8_t Vf_No;
uint8_t F_AskStop1;
uint8_t RealClock[9];     // ʵʱʱ��
uint8_t B_PumpBad[6];     // B_PumpBad[0]δʹ��,��1-5����־
uint8_t B_PumpManu[6];    // ���ֶ�״̬1-5��־(������)
uint8_t B_SaveSetP;       // �����趨ѹ�� ��־�ֽ�
uint8_t B_SaveVoiceValue; // ����������С ��־�ֽ�
uint8_t B_ReadVfPar;      // ��ȡ��Ƶ������
uint8_t B_ModSelParValue; // �޸�ѡ��Ĳ���ֵ
uint8_t F_ComErrorPump1;
uint8_t F_ComErrorPump2;
uint8_t B_ComErrorPump1;
uint8_t B_ComErrorPump2;
uint8_t B_RemoteStop1;
uint8_t B_RemoteStop2;
uint8_t B_MaxSupplyStop1;
uint8_t B_MaxSupplyStop2;
uint8_t F_HaveWater_MaxSupply;
uint8_t F_InPPid;
uint8_t F_HengYa;
uint8_t F_waterYeWeiHigh;
uint8_t F_ZhuoDuHigh;
uint8_t F_YuLvHigh;
uint8_t F_PhHigh;
uint8_t F_PhLow;

uint8_t F_ComErrorPump3;
uint8_t F_ComErrorPump4;
uint8_t B_ComErrorPump3;
uint8_t B_ComErrorPump4;
uint8_t B_RemoteStop3;
uint8_t B_RemoteStop4;
uint8_t B_MaxSupplyStop3;
uint8_t B_MaxSupplyStop4;
uint8_t F_PumpExit;
uint8_t PumpExit[6];
uint8_t F_HighYCSetP;
uint8_t S_HighYCSetP;

uint8_t F_Pump1RunOverTime;
uint8_t F_Pump2RunOverTime;
uint8_t F_Pump3RunOverTime;
uint8_t F_Pump4RunOverTime;
uint8_t F_YeJianSmall;
uint8_t F_HengVfPanDuan_S;
uint8_t F_HengVfPanDuan_L;

//uint32_t  w_ParLst_POS[POS_SIZE*POS_NUM];				//20*15=300��˫�� //40*30=1200��˫��
s32 w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 6 + FLASH_POS_SIZE + FLASH_POS_CMD_SIZE]; // �ŷ�����������б���(300+600*6+300+1200)*4=5400*4=21600���ֽ�

//ִ�������˳��
s32 Pos_CMD_Sort_Queue[POS_CMD_NUM];

uint8_t DInb[18];  // ��������������б���			//ZCL 2019.11.23 �Ӵ󳤶ȣ�һ��SZM220��������4���ӻ�����4��JDQģ�飬��9��ģ��
uint8_t DOutb[18]; // ��������������б���		//ZCL 2019.11.23 �Ӵ󳤶ȣ�һ��SZM220��������4���ӻ�����4��JDQģ�飬��9��ģ��
//uint8_t  NBComBuf[52];		// ����ͨѶ���� 2007.10.30
uint16_t w_AI[36]; // ģ������������б���		//ZCL 2019.11.23 �Ӵ󳤶ȣ�һ��SZM220��������4���ӻ�����4��JDQģ�飬��9��ģ��
//uint16_t  w_AQ[36];			// ģ������������б���		//ZCL 2019.11.23 �Ӵ󳤶ȣ�һ��SZM220��������4���ӻ�����4��JDQģ�飬��9��ģ��

uint16_t w_ParLst[840]; // �������б���

//����ÿ̨����Ĳ�ѯ�������
//ÿ�����壺���+�����+��������
//uint8_t	Com1_Driver12_Query_BUFF[COM_QUERY_SIZE*COM_QUERY_NUM];	//COM_QUERY_SIZE=20��COM_QUERY_NUM=10.����10����ѯ���ÿ��20���ֽ�
//uint8_t	Com2_Driver34_Query_BUFF[COM_QUERY_SIZE*COM_QUERY_NUM];
//uint8_t	Com3_Driver5_Query_BUFF[COM_QUERY_SIZE*COM_QUERY_NUM];

//����ÿ̨�����д�������
//ÿ�����壺���+�����+��������
uint8_t Com1_Driver1_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM]; //COM_CMD_SIZE=20��COM_CMD_NUM=30.����30��д���ÿ��20���ֽ�
uint8_t Com1_Driver2_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM]; //COM_CMD_SIZE=20��COM_CMD_NUM=30.����30��д���ÿ��20���ֽ�
uint8_t Com2_Driver3_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];
uint8_t Com2_Driver4_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];
uint8_t Com3_Driver5_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];
uint8_t Com3_Driver6_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];
uint8_t Com4_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];

uint8_t *Com_Write_p;

//����ÿ̨�������һ������
//���壺���+�����+��������
uint8_t Com1_LAST_CMD_BUFF[COM_CMD_SIZE]; //����1����30���ֽ�
uint8_t Com2_LAST_CMD_BUFF[COM_CMD_SIZE];
uint8_t Com3_LAST_CMD_BUFF[COM_CMD_SIZE];
uint8_t Com4_LAST_CMD_BUFF[COM_CMD_SIZE];

//uint8_t* pByteParLst=(uint8_t *)(&w_ParLst[0]);
uint8_t B_SelCS;
uint16_t w_FaultNo[4];

uint8_t B_ComErrWithVvvf;    // ��־ ���Ƶ��ͨѶʧ�� ��־�ֽ�
uint8_t B_ComErrWithLLJ;     // ��־ ��������ͨѶʧ�ܡ�LLJ:������
uint8_t B_ComErrWithModule1; // ��־ ����ģ��1ͨѶʧ��
uint8_t B_ComErrWithModule2; // ��־ ����ģ��2ͨѶʧ��

uint8_t F_Com2SendNext;        // ���ڷ�����һ����־
uint8_t F_Com2RCVCommandOk;    // ���ڽ�������OK��־
uint8_t F_SlaveNoRcvMasterCMD; // �ӻ�û�н��յ���������

uint16_t w_PreVoiceVal;   // �ϴ�������С
uint16_t w_ModRealTimeNo; // �޸�ʵʱʱ�ӵ����
uint16_t w_PreRecNo;      // �ϴμ�¼��

uint32_t Reset_time_Max; //��λ����ʱ��
//
// ����Ϊ���ⲿ�ֽڶ���Ŀ��������룬�������������
// Ӳ������
//1  ���������� ʹ��
uint8_t K_ManuAuto;       // �ֶ�.�Զ�
uint8_t K_StopRun;        // ����ֹͣ
uint8_t K_OverP;          // ��ѹ����
uint8_t K_BCQErr;         // ������Ԥ��
uint8_t K_VvvfERR;        // ��Ƶ������
uint8_t K_WaterLeverHigh; // ˮ��ˮλ����
uint8_t K_NoWater;        // Һλ����ˮͣ��
uint8_t K_HaveWater;      // Һλ����ˮ����
uint8_t K_PumpHot[6];     // 1-5�ű��ȹ�������
uint8_t K_PowerErr;       // ��Դ����
uint8_t K_Fire;           // ��������
uint8_t K_ResetVvvf;      // ��Ƶ����λ
uint8_t K_FluxPluseOn;
uint8_t K_DDPluseOn;
uint8_t K_DDFOpen;
uint8_t K_DDFClose;

//					//2  ���������� ���Ҳ���
uint8_t K_1PumpManu;    // 1�ű��ֶ�����
uint8_t K_2PumpManu;    // 2�ű��ֶ�����
uint8_t K_3PumpManu;    // 3�ű��ֶ�����
uint8_t K_4PumpManu;    // 4�ű��ֶ�����
uint8_t K_5PumpManu;    // 5�ű��ֶ�����
uint8_t K_1PumpGp;      // 1�űù�Ƶ
uint8_t K_2PumpGp;      // 2�űù�Ƶ
uint8_t K_3PumpGp;      // 3�űù�Ƶ
uint8_t K_4PumpGp;      // 4�űù�Ƶ
uint8_t K_5PumpGp;      // 5�űù�Ƶ
uint8_t K_1PumpVf;      // 1�űñ�Ƶ
uint8_t K_2PumpVf;      // 2�űñ�Ƶ
uint8_t K_3PumpVf;      // 3�űñ�Ƶ
uint8_t K_4PumpVf;      // 4�űñ�Ƶ
uint8_t K_5PumpVf;      // 5�űñ�Ƶ
uint8_t K_Term13;       // ����13 DI
uint8_t K_Term14;       // ����14 DI
uint8_t K_Term15;       // ����15 DI
                        //3  ��������� ʹ��
uint8_t Q_VfStop;       // ��ת
uint8_t Q_RstOn;        // ��λ�̵���
uint8_t Q_PowerOnModem; // ��MODEM�ϵ�
uint8_t Q_BcqErr;       // ������Ԥ��
uint8_t Q_VfOn[6];      // ��Ƶ1--5
uint8_t Q_GpOn[6];      // ��Ƶ1--5
uint8_t Q_AlarmError;   // ��������̵���
uint8_t Q_FanMotor;     // ���ȵ���̵���
uint8_t Q_RunGPRS;
//4  λ��־���� 5���ֽ�
// ͣ��״̬��־
uint8_t F_NoWater;            // ��ˮͣ����־
uint8_t F_ManualRunStop;      // �ֶ�����ֹͣ��־
uint8_t F_ManualAuto;         // �ֶ����Զ� ��=1,�ֶ���
uint8_t F_VvvfAlarm;          // ��Ƶ��������־
uint8_t F_PowerFault;         // ��Դ����
uint8_t F_WaterLeverHigh;     // ����Һλͣ����־
uint8_t F_HardOverP;          // Ӳ����ѹ��־
uint8_t F_AllPumpBad;         // ���бû���־
uint8_t F_SmallStop;          // С����ͣ����־
uint8_t F_OutMeterBad;        // ��ˮ��Զ��ѹ����ͣ����־  // ��û��
uint8_t F_InPBigSetStop;      // ��ˮѹ����ͣ������ˮѹ>�趨ѹͣ��
uint8_t F_TimeRunStop;        // ��ʱ����ֹͣ��־
uint8_t F_TimePwdStop;        // ��ʱ����ͣ����־
uint8_t F_RemoteStop;         // Զ��ң��ͣ����־
uint8_t F_InPBigOutStop;      // ��ˮ���ڳ�ˮͣ����־����ˮѹ>��ˮѹͣ��
uint8_t F_MaxSupplyStop;      // ���ˮ����ͣ����־
uint8_t F_Rtcf;               // �󱸵��ʧЧ
uint8_t F_ComErrWithVvvf;     // ���Ƶ��ͨѶʧ��
uint8_t F_ComErrWithLLJ;      // ��������ͨѶʧ�ܡ�LLJ:������
uint8_t F_ComErrWithModule1;  // ����ģ��1ͨѶʧ��
uint8_t F_ComErrWithModule2;  // ����ģ��2ͨѶʧ��
uint8_t F_TouchAutoManu;      // �����Զ��ֶ�
uint8_t F_TouchRunStop;       // ��������ֹͣ
uint8_t F_PSensorProtectStop; // ѹ������������ͣ��

//YLS 2020.06.25
uint8_t F_TouchReSet;      //������λ
uint8_t F_AskReset;        //����λ
uint8_t F_Reseting;        //���ڸ�λ��־
uint8_t F_TouchForceReSet; //ǿ�Ƹ�λ
uint8_t F_AskForceReset;   //����λ
uint8_t F_ForceReseting;   //���ڸ�λ��־
uint8_t F_Reseting;        //���ڸ�λ��־

uint8_t F_StepMode;   //=1�������ֶ�ģʽ��־
uint8_t F_PowerOnRun; //�Ƿ��ϵ��Զ����б�־

uint8_t F_Stoping;  //����ֹͣ��־
uint8_t F_Starting; //����������־

uint8_t F_Encoder_Read; //�����������Ѷ�ȡ��־
uint8_t F_Status_Read;  //VDO״̬��ȡ��־

// �����������ͣ����־
//---------------------
// �ڲ�״̬��־
uint8_t F_CycleCheck; // ����Ѳ���־
uint8_t F_DealSmall;  // ���ڴ���С������־
uint8_t F_Fire;       // ����״̬��־
// �����������״̬��־
//---------------------
// �ڲ�������ϱ�־
uint8_t F_VvvfBad;   // ��Ƶ������־
uint8_t F_SmallBad;  // С�����ƻ���־
uint8_t F_BcqErr;    // ������Ԥ��
uint8_t F_SoftOverP; // �����ѹ��־
// �ڲ�ת����־
uint8_t F_AskStop; // ����ͣ����־
//uint8_t  F_AskExchange;			// ���󽻻���־
uint8_t F_VfToGp;        // ��Ƶת��Ƶ��־
uint8_t F_VfToVf;        // ��Ƶ������־
uint8_t F_TimerExchange; // ��ʱ������־
//
uint8_t F_SmallBetween; // С���������־
//uint8_t  F_DelayCheckVvvfAlarm;	// ��һ���ϵ���ʱ����Ƶ������5���ֺ��ټ��
uint8_t F_Inital;                  // ��ʼ����־
uint8_t F_VvvfAlarmREC;            // ��Ƶ���������м�¼
uint8_t F_TimeREC;                 // ��ʱд��¼
uint8_t F_ModRealTime;             // �޸�ʵʱʱ��
uint8_t F_FlashRecNoNum;           // FLASH��¼����ź�����
uint8_t F_FirstPowerNoPlayNoWater; // ��һ���ϵ粻������ˮͣ��

uint8_t F_AllRdy; //���е����Ready��־

uint8_t F_AllRun;  //���е������RUN��־
uint8_t F_Reseted; //��λ��ԭ���־

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

s32 *arr_p1;
s32 *arrp_p1_Last; //������һ��ָ�������ָ��

s32 *Driver12_arr_p;
s32 *Driver34_arr_p;
s32 *Driver56_arr_p;

uint16_t T_SavePos_Delay1;
uint16_t C_SavePos_Delay1;
uint16_t T_SavePos_Delay2;
uint16_t C_SavePos_Delay2;
uint16_t T_SavePos_Delay3;
uint16_t C_SavePos_Delay3;
uint16_t T_SavePos_Delay4;
uint16_t C_SavePos_Delay4;
uint16_t T_SavePos_Delay5;
uint16_t C_SavePos_Delay5;
uint16_t T_SavePos_Delay6;
uint16_t C_SavePos_Delay6;

uint8_t Driver1_Save_P;
uint8_t Driver2_Save_P;
uint8_t Driver3_Save_P;
uint8_t Driver4_Save_P;
uint8_t Driver5_Save_P;
uint8_t Driver6_Save_P;

uint8_t F_Driver1_WrPos_ErrStatus;
uint8_t F_Driver2_WrPos_ErrStatus;
uint8_t F_Driver3_WrPos_ErrStatus;
uint8_t F_Driver4_WrPos_ErrStatus;
uint8_t F_Driver5_WrPos_ErrStatus;
uint8_t F_Driver6_WrPos_ErrStatus;

uint8_t Driver1_delay_F; //�����ͣ��ʱ��־��=1��ʾ������ͣ��=0,��ʾ��ͣ����

uint8_t DO_delay_F; //DO�����ʱ��־
uint8_t F_Open_Hand;
uint8_t F_Close_Hand;

uint32_t Driver1_delay_ms_Count; //�����ͣ��ʱ������ms
uint32_t Driver2_delay_ms_Count;
uint32_t Driver3_delay_ms_Count;
uint32_t Driver4_delay_ms_Count;
uint32_t Driver5_delay_ms_Count;
uint32_t Driver6_delay_ms_Count;

uint32_t T_Driver1_delay;      //��ʱ��ʱ��
uint32_t C_Driver1_delayCount; //��ʱ����
uint32_t T_Driver2_delay;
uint32_t C_Driver2_delayCount;
uint32_t T_Driver3_delay;
uint32_t C_Driver3_delayCount;
uint32_t T_Driver4_delay;
uint32_t C_Driver4_delayCount;
uint32_t T_Driver5_delay;
uint32_t C_Driver5_delayCount;
uint32_t T_Driver6_delay;
uint32_t C_Driver6_delayCount;

uint32_t T_DO_delay;           //DO�����ʱ��ʱ��
uint32_t C_DO_delayCount;      //DO�����ʱ����
uint32_t T_DO_Open_delay;      //DO���������ʱ��
uint32_t C_DO_Open_delayCount; //DO�����������

uint16_t T_Driver1_FillCMD;
uint16_t C_Driver1_FillCMD;
uint16_t T_Driver2_FillCMD;
uint16_t C_Driver2_FillCMD;
uint16_t T_Driver3_FillCMD;
uint16_t C_Driver3_FillCMD;
uint16_t T_Driver4_FillCMD;
uint16_t C_Driver4_FillCMD;
uint16_t T_Driver5_FillCMD;
uint16_t C_Driver5_FillCMD;
uint16_t T_Driver6_FillCMD;
uint16_t C_Driver6_FillCMD;

uint16_t T_StepAutoDelay;
uint16_t C_T_StepAutoDelayDelay;
uint16_t T_StepAutoDelay2;
uint16_t C_T_StepAutoDelayDelay2;

uint16_t T_ResetDelay;
uint16_t C_ResetDelay;
uint16_t T_ResetDelay2;
uint16_t C_ResetDelay2;
uint16_t T_ResetDelay3;
uint16_t C_ResetDelay3;
uint8_t F_SendStopCMD2;

uint16_t T_Reset_Driver;
uint16_t C_Reset_Driver;
uint16_t T_Reset_Driver2;
uint16_t C_Reset_Driver2;
uint8_t F_AutoReset;
uint8_t F_SendStopCMD;

uint8_t Driver1_Write_Sort; //������1дλ�ö�˳��0д��1�Σ�1д��2��
uint8_t Driver2_Write_Sort;
uint8_t Driver3_Write_Sort;
uint8_t Driver4_Write_Sort;
uint8_t Driver5_Write_Sort;
uint8_t Driver6_Write_Sort;
uint8_t F_Hand_Status; //��ŷ�״̬

uint8_t F_Driver1_notBrake; //1#�ŷ�ɲ���ź�
uint8_t F_Driver2_notBrake;
uint8_t F_Driver3_notBrake;
uint8_t F_Driver4_notBrake;
uint8_t F_Driver5_notBrake;
uint8_t F_Driver6_notBrake;
uint8_t F_Driver_All_notBrake;

uint16_t C_AllStop;

uint8_t F_Sync_6_axis; //6��ͬ�������־
uint8_t F_Resetting;   //���ڸ�λ��־

uint8_t F_Driver1_Send_Cmd; //��������1#�ŷ�����������λ�������־��=1��ʾ�ѷ�������
uint8_t F_Driver2_Send_Cmd;
uint8_t F_Driver3_Send_Cmd;
uint8_t F_Driver4_Send_Cmd;
uint8_t F_Driver5_Send_Cmd;
uint8_t F_Driver6_Send_Cmd;

uint8_t F_Driver1_Timeout; //1#�ŷ��������������ʱ��־��=1��ʾ��ʱ
uint8_t F_Driver2_Timeout;
uint8_t F_Driver3_Timeout;
uint8_t F_Driver4_Timeout;
uint8_t F_Driver5_Timeout;
uint8_t F_Driver6_Timeout;

uint8_t F_Driver1_Cmd_Err; //1#�ŷ�������λ����������־��=1��ʾ����
uint8_t F_Driver2_Cmd_Err;
uint8_t F_Driver3_Cmd_Err;
uint8_t F_Driver4_Cmd_Err;
uint8_t F_Driver5_Cmd_Err;
uint8_t F_Driver6_Cmd_Err;
uint8_t F_HaveDriver_Cmd_Err; //���ŷ�д��λ�ô����־

uint8_t F_Driver1_Cmd_Con_Err; //1#�ŷ�������������������־��=1��ʾ����
uint8_t F_Driver2_Cmd_Con_Err;
uint8_t F_Driver3_Cmd_Con_Err;
uint8_t F_Driver4_Cmd_Con_Err;
uint8_t F_Driver5_Cmd_Con_Err;
uint8_t F_Driver6_Cmd_Con_Err;
uint8_t F_HaveDriver_Cmd_Con_Err; //���ŷ�д��λ�ô����־

uint8_t Driver1_Cmd_PosNo; //1#�ŷ����������͵�����λ�úţ�λ��0����λ��1
uint8_t Driver2_Cmd_PosNo;
uint8_t Driver3_Cmd_PosNo;
uint8_t Driver4_Cmd_PosNo;
uint8_t Driver5_Cmd_PosNo;
uint8_t Driver6_Cmd_PosNo;

uint8_t Driver1_Cmd_Status; //1#�ŷ�������P8910״̬
uint8_t Driver2_Cmd_Status;
uint8_t Driver3_Cmd_Status;
uint8_t Driver4_Cmd_Status;
uint8_t Driver5_Cmd_Status;
uint8_t Driver6_Cmd_Status;

uint16_t T_Driver1_WriteCMD;
uint16_t C_Driver1_WriteCMD;
uint16_t T_Driver2_WriteCMD;
uint16_t C_Driver2_WriteCMD;
uint16_t T_Driver3_WriteCMD;
uint16_t C_Driver3_WriteCMD;
uint16_t T_Driver4_WriteCMD;
uint16_t C_Driver4_WriteCMD;
uint16_t T_Driver5_WriteCMD;
uint16_t C_Driver5_WriteCMD;
uint16_t T_Driver6_WriteCMD;
uint16_t C_Driver6_WriteCMD;

uint8_t Driver1_Cmd_Data[9]; //1#�ŷ���������������,4���ֽ�Ϊ���壬2���ֽ�Ϊ�ٶȣ�2���ֽ�Ϊ�Ӽ���ʱ�䣬���һ���ֽ�Ϊλ�úţ�0/1��
uint8_t Driver2_Cmd_Data[9];
uint8_t Driver3_Cmd_Data[9];
uint8_t Driver4_Cmd_Data[9];
uint8_t Driver5_Cmd_Data[9];
uint8_t Driver6_Cmd_Data[9];

uint8_t Driver1_Status_Sort; //1#�ŷ���=2����ʾ�Ѿ����ͣ�=3����ʾ�Ѿ�����
uint8_t Driver2_Status_Sort;
uint8_t Driver3_Status_Sort;
uint8_t Driver4_Status_Sort;
uint8_t Driver5_Status_Sort;
uint8_t Driver6_Status_Sort;

//**********COM1**********
uint8_t Txd1Buffer[TXD1_MAX]; // ���ͻ�����
uint8_t Rcv1Buffer[RCV1_MAX]; // ���ջ�����
uint8_t Tmp_Txd1Buffer[1], Tmp_Rxd1Buffer;
uint16_t Rcv1Counter;   // ���ռ�����//
uint16_t Txd1Counter;   // ���ͼ�����//
uint16_t BakRcv1Count;  // ���ռ�����//
uint16_t Txd1Max;       // �ж��ٸ��ַ���Ҫ����//
uint16_t w_Txd1ChkSum;  // ����У��ͣ�lo,hi ��λ//
uint16_t w_Com1RegAddr; // ����1�Ĵ�����ַ

//
uint8_t B_Com1Cmd03;
uint8_t B_Com1Cmd16;
uint8_t B_Com1Cmd01;
uint8_t B_Com1Cmd06;
uint16_t T_NoRcv1Count; // û�н��ռ�����
uint16_t C_NoRcv1Count;

//**********COM2**********
uint8_t Txd2Buffer[TXD2_MAX]; // ���ͻ�����
uint8_t Rcv2Buffer[RCV2_MAX]; // ���ջ�����
uint8_t Tmp_Txd2Buffer[1], Tmp_Rxd2Buffer;
uint16_t Rcv2Counter;   // ���ռ�����//
uint16_t Txd2Counter;   // ���ͼ�����//
uint16_t BakRcv2Count;  // ���ռ�����//
uint16_t Txd2Max;       // �ж��ٸ��ַ���Ҫ����//
uint16_t w_Txd2ChkSum;  // ����У��ͣ�lo,hi ��λ//
uint16_t w_Com2RegAddr; // ����2�Ĵ�����ַ

uint8_t B_Com2Cmd03;
uint8_t B_Com2Cmd16;
uint8_t B_Com2Cmd01;
uint8_t B_Com2Cmd06;
uint16_t T_NoRcv2Count; // û�н��ռ�����
uint16_t C_NoRcv2Count;

//**********COM3**********
uint8_t Txd3Buffer[TXD3_MAX]; // ���ͻ�����
uint8_t Rcv3Buffer[RCV3_MAX]; // ���ջ�����
uint8_t Tmp_Txd3Buffer[1], Tmp_Rxd3Buffer;
uint16_t Rcv3Counter;   // ���ռ�����//
uint16_t Txd3Counter;   // ���ͼ�����//
uint16_t BakRcv3Count;  // ���ռ�����//
uint16_t Txd3Max;       // �ж��ٸ��ַ���Ҫ����//
uint16_t w_Txd3ChkSum;  // ����У��ͣ�lo,hi ��λ//
uint16_t w_Com3RegAddr; // ����3�Ĵ�����ַ

uint8_t B_Com3Cmd03;
uint8_t B_Com3Cmd16;
uint8_t B_Com3Cmd01;
uint8_t B_Com3Cmd06;
uint16_t T_NoRcv3Count; // û�н��ռ�����
uint16_t C_NoRcv3Count;

//**********COM4**********
uint8_t Txd4Buffer[TXD4_MAX]; // ���ͻ�����
uint8_t Rcv4Buffer[RCV4_MAX]; // ���ջ�����
uint8_t Tmp_Txd4Buffer[1], Tmp_Rxd4Buffer;
uint16_t Rcv4Counter;   // ���ռ�����//
uint16_t Txd4Counter;   // ���ͼ�����//
uint16_t BakRcv4Count;  // ���ռ�����//
uint16_t Txd4Max;       // �ж��ٸ��ַ���Ҫ����//
uint16_t w_Txd4ChkSum;  // ����У��ͣ�lo,hi ��λ//
uint16_t w_Com4RegAddr; // ����4�Ĵ�����ַ

uint8_t B_Com4Cmd03;
uint8_t B_Com4Cmd16;
uint8_t B_Com4Cmd01;
uint8_t B_Com4Cmd06;
uint16_t T_NoRcv4Count; // û�н��ռ�����
uint16_t C_NoRcv4Count;

//**********COM5**********
uint8_t Txd5Buffer[TXD5_MAX]; // ���ͻ�����
uint8_t Rcv5Buffer[RCV5_MAX]; // ���ջ�����
uint8_t Tmp_Txd5Buffer[1], Tmp_Rxd5Buffer;
uint16_t Rcv5Counter;   // ���ռ�����//
uint16_t Txd5Counter;   // ���ͼ�����//
uint16_t BakRcv5Count;  // ���ռ�����//
uint16_t Txd5Max;       // �ж��ٸ��ַ���Ҫ����//
uint16_t w_Txd5ChkSum;  // ����У��ͣ�lo,hi ��λ//
uint16_t w_Com5RegAddr; // ����5�Ĵ�����ַ

uint8_t B_Com5Cmd03;
uint8_t B_Com5Cmd16;
uint8_t B_Com5Cmd01;
uint8_t B_Com5Cmd06;
uint16_t T_NoRcv5Count; // û�н��ռ�����
uint16_t C_NoRcv5Count;

//**********COM6**********
uint8_t Txd6Buffer[TXD6_MAX]; // ���ͻ�����
uint8_t Rcv6Buffer[RCV6_MAX]; // ���ջ�����
uint8_t Tmp_Txd6Buffer[1], Tmp_Rxd6Buffer;
uint16_t Rcv6Counter;   // ���ռ�����//
uint16_t Txd6Counter;   // ���ͼ�����//
uint16_t BakRcv6Count;  // ���ռ�����//
uint16_t Txd6Max;       // �ж��ٸ��ַ���Ҫ����//
uint16_t w_Txd6ChkSum;  // ����У��ͣ�lo,hi ��λ//
uint16_t w_Com6RegAddr; // ����6�Ĵ�����ַ

uint8_t B_Com6Cmd03;
uint8_t B_Com6Cmd16;
uint8_t B_Com6Cmd01;
uint8_t B_Com6Cmd06;
uint16_t T_NoRcv6Count; // û�н��ռ�����
uint16_t C_NoRcv6Count;

uint32_t Motor1_current_pos; //��ǰλ��
uint32_t Motor2_current_pos;
uint32_t Motor3_current_pos;
uint32_t Motor4_current_pos;
uint32_t Motor5_current_pos;
uint32_t Motor6_current_pos;

int Motor1_startflag; //�������б�־
int Motor2_startflag;
int Motor3_startflag;
int Motor4_startflag;
int Motor5_startflag;
int Motor6_startflag;

//**********WK_COM1**********
uint8_t WK_Txd1Buffer[TXD1_MAX]; // ���ͻ�����
uint8_t WK_Rcv1Buffer[RCV1_MAX]; // ���ջ�����
uint8_t WK_Tmp_Txd1Buffer[1], WK_Tmp_Rxd1Buffer;
uint16_t WK_Rcv1Counter;   // ���ռ�����//
uint16_t WK_Txd1Counter;   // ���ͼ�����//
uint16_t WK_BakRcv1Count;  // ���ռ�����//
uint16_t WK_Txd1Max;       // �ж��ٸ��ַ���Ҫ����//
uint16_t WK_w_Txd1ChkSum;  // ����У��ͣ�lo,hi ��λ//
uint16_t WK_w_Com1RegAddr; // ����1�Ĵ�����ַ

//
uint8_t WK_B_Com1Cmd03;
uint8_t WK_B_Com1Cmd16;
uint8_t WK_B_Com1Cmd01;
uint8_t WK_B_Com1Cmd06;
uint16_t T_WK_NoRcv1Count; // û�н��ռ�����
uint16_t C_WK_NoRcv1Count;

//**********WK_COM2**********
uint8_t WK_Txd2Buffer[TXD2_MAX]; // ���ͻ�����
uint8_t WK_Rcv2Buffer[RCV2_MAX]; // ���ջ�����
uint8_t WK_Tmp_Txd2Buffer[1], WK_Tmp_Rxd2Buffer;
uint16_t WK_Rcv2Counter;   // ���ռ�����//
uint16_t WK_Txd2Counter;   // ���ͼ�����//
uint16_t WK_BakRcv2Count;  // ���ռ�����//
uint16_t WK_Txd2Max;       // �ж��ٸ��ַ���Ҫ����//
uint16_t WK_w_Txd2ChkSum;  // ����У��ͣ�lo,hi ��λ//
uint16_t WK_w_Com2RegAddr; // ����2�Ĵ�����ַ

//
uint8_t WK_B_Com2Cmd03;
uint8_t WK_B_Com2Cmd16;
uint8_t WK_B_Com2Cmd01;
uint8_t WK_B_Com2Cmd06;
uint16_t T_WK_NoRcv2Count; // û�н��ռ�����
uint16_t C_WK_NoRcv2Count;

//**********WK_COM3**********
uint8_t WK_Txd3Buffer[TXD3_MAX]; // ���ͻ�����
uint8_t WK_Rcv3Buffer[RCV3_MAX]; // ���ջ�����
uint8_t WK_Tmp_Txd3Buffer[1], WK_Tmp_Rxd3Buffer;
uint16_t WK_Rcv3Counter;   // ���ռ�����//
uint16_t WK_Txd3Counter;   // ���ͼ�����//
uint16_t WK_BakRcv3Count;  // ���ռ�����//
uint16_t WK_Txd3Max;       // �ж��ٸ��ַ���Ҫ����//
uint16_t WK_w_Txd3ChkSum;  // ����У��ͣ�lo,hi ��λ//
uint16_t WK_w_Com3RegAddr; // ����3�Ĵ�����ַ

//
uint8_t WK_B_Com3Cmd03;
uint8_t WK_B_Com3Cmd16;
uint8_t WK_B_Com3Cmd01;
uint8_t WK_B_Com3Cmd06;
uint16_t T_WK_NoRcv3Count; // û�н��ռ�����
uint16_t C_WK_NoRcv3Count;

//**********WK_COM4**********
uint8_t WK_Txd4Buffer[TXD4_MAX]; // ���ͻ�����
uint8_t WK_Rcv4Buffer[RCV4_MAX]; // ���ջ�����
uint8_t WK_Tmp_Txd4Buffer[1], WK_Tmp_Rxd4Buffer;
uint16_t WK_Rcv4Counter;   // ���ռ�����//
uint16_t WK_Txd4Counter;   // ���ͼ�����//
uint16_t WK_BakRcv4Count;  // ���ռ�����//
uint16_t WK_Txd4Max;       // �ж��ٸ��ַ���Ҫ����//
uint16_t WK_w_Txd4ChkSum;  // ����У��ͣ�lo,hi ��λ//
uint16_t WK_w_Com4RegAddr; // ����4�Ĵ�����ַ

//
uint8_t WK_B_Com4Cmd03;
uint8_t WK_B_Com4Cmd16;
uint8_t WK_B_Com4Cmd01;
uint8_t WK_B_Com4Cmd06;
uint16_t T_WK_NoRcv4Count; // û�н��ռ�����
uint16_t C_WK_NoRcv4Count;

#endif /* __GLOBALV_H */
