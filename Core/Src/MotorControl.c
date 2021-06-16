

/* Includes ------------------------------------------------------------------*/
#include "MotorControl.h"
#include "GlobalConst.h"
#include "GlobalV_Extern.h" // ȫ�ֱ�������
#include "typedef.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private variables extern --------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

void IWDG_Init(u8 prer, u16 rlr); //�������Ź���ʼ�� 2013.7.3
void IWDG_Feed(void);			  //�������Ź�ι��	 2013.7.3
void PowerDelay(u16 nCount);
void Driver_Control(void); //������п���
u8 IS_Driver_Stop(void);
void Normal_Run(void);
u8 Locate_Rle_1(u8 driver_no, s32 num, u16 speed, u16 acc_dec_time, s32 Cmd_No); //��Զ�λ����
u8 Fill_Pos_Data(u8 driver_no, s32 num, u16 speed, u16 acc_dec_time, s32 Cmd_No);
u8 Send_Start_Cmd(u8 driver_no);
void Run_Driver1(void);
void Run_Driver2(void);
void Run_Driver3(void);
void Run_Driver4(void);
void Run_Driver5(void);
void Run_Driver6(void);
//�ж�COM1��������Ƿ�Ϊ��
u8 Com1_Driver1_Queue_isEmpty(void);
u8 Com1_Driver2_Queue_isEmpty(void);
u8 Com2_Driver3_Queue_isEmpty(void);
u8 Com2_Driver4_Queue_isEmpty(void);
u8 Com3_Driver5_Queue_isEmpty(void);
u8 Com3_Driver6_Queue_isEmpty(void);

//�ж�COM1��������Ƿ�Ϊ��
u8 Com1_Driver1_Queue_isFull(void);
u8 Com1_Driver2_Queue_isFull(void);
u8 Com2_Driver3_Queue_isFull(void);
u8 Com2_Driver4_Queue_isFull(void);
u8 Com3_Driver5_Queue_isFull(void);
u8 Com3_Driver6_Queue_isFull(void);

void Reset_Routine(void);						  //��λ�ӳ���
void Reset_Drivers(void);						  //��λ�������ʼλ��
void Stop_Driver(u8 driver_no);					  //����ͣ������
void Control_Hand(void);						  //���Ƶ�ŷ�(��е��)��ͨ���ض�
void BRAKE_Control(void);						  //ɲ������
void is_Reseted(void);							  //�жϸ�λ��ԭ���ӳ���
void Send_StopCMD(void);						  //����ͣ�������ӳ���
void PosCMD_ReadFrom_FLASH(void);				  //������в�����ʼ��
void Driver_Save_Pos(void);						  //д��λ�ò������ŷ�������
s16 Move_Cmd_Queue(u8 driver_no);				  //�ƶ�����ָ��
void Clear_Cmd_Queue(void);						  //���������
void sort(s32 *a, int l);						  //����������
void sort_f(float *a, int l);					  //����������������
u8 Cal_Pos_Speed(void);							  //�������庯��
void Cal_One_Pos_Speed(s32 Pos_No1, s32 Pos_No2); //����һ��λ�õ��������ٶ�
void Record_Current_Pos(void);					  //��¼��ǰλ��
void Run_to_One_Pos(s32 Cmd_No);				  //���е�ĳһ����Ŷ�Ӧ��λ�õ�
void Fill_Cmd(void);							  //�������
void Cal_Sum_Err(u8 Driver_No);					  //��������������������
void Cal_Run_to_Next_Pos(s32 Cmd_No);			  //�ӵ�ǰλ��ͬ�����е�ĳһ����Ŷ�Ӧ��λ�õ�
void Send_Driver_CMD(u8 driver_no, u8 Modbus_CMD_No, u32 Par_addr, u32 Par_content);
//����λ�ü�λ������
extern void ParLst_Init_Group2Zero(void);
//����λ��
void ParLst_Init_Pos2Zero(void);
//������λ���ֶ���������
void Pos_Manual_Adj(void);
//����λ��
void Adj_Pos(u8 driver_no, s32 PosErr_Muti, s32 PosErr_Sing);
//����λ������
void Send_Pos_Data(void);
//У��λ�������Ƿ���ȷ
void Verify_Pos_CMD(void);
void Limit_Max_Pos(void); //���������Сλ�÷�Χ
//�ж�ĳ̨����Ƿ񳬳�λ�÷�Χ
u8 Judge_OverPos(u8 Driver_No);
/* Private functions ---------------------------------------------------------*/

void Driver_Control(void) // �������
{
	s32 *tmp_arr_p1;
	//Pw_TouchRunStop=0������
	if (Pw_StepAutoMode == 0 && F_ForceReseting == 0) //Pw_StepAutoMode=1���ֶ�ģʽ��Ĭ��ֵ����=0��ȫ�Զ�ģʽ
	{
		F_StepMode = 0; //=0�������Զ�ģʽ��=1�������ֶ�ģʽ

		Pw_EquipStatus = Pw_EquipStatus & 0xFFF7; //�崥��������ͣ��״̬λ
		Pw_EquipStatus = Pw_EquipStatus & 0xFFEF; //���ֶ�״̬λ

		if (F_PowerOnRun == 1) //=1�������ϵ��Զ�����
		{
			//����ϵ������������������ڻ�û������������ʱ��λ
			if (Pw_TouchRunStop == 0 && F_TouchRunStop == 1) //Pw_TouchRunStop=0������;=1ֹͣ
			{
				if (F_SendStopCMD2 == 0) //�����û�з���ͣ�������־
				{
					Send_StopCMD();		//����ͣ�����ֻʹ���ŷ���
					F_SendStopCMD2 = 1; //���Ѿ�����ͣ�������־
				}

				Pr_RUN_Count = 0; //��������Ȧ��
								  //��ʱ
				if (T_StepAutoDelay != SClk10Ms)
				{
					T_StepAutoDelay = SClk10Ms;
					C_T_StepAutoDelayDelay++;
					if (C_T_StepAutoDelayDelay > 30)
					{
						C_T_StepAutoDelayDelay = 0;

						//��ʱ�󣬿�ɲ��
						Pr_BRAKE_Control = 0; //��ɲ��
						STOP_BRAKE_SYSTEM;	  //��ɲ��ϵͳ������

						//����ǰ��ִ�и�λ
						//							if(Pr_F_AllStopped )		//if(Pr_F_AllStopped && F_AllRdy)
						//							{
						if (F_Starting == 0 && F_Stoping == 0 && F_ForceReseting == 0)
						{
							F_AskStop = 0;						  //��ʱ��ϣ�������������
							F_TouchRunStop = 0;					  //����������״̬
							F_Starting = 1;						  //����������־
							Pw_Current_Run_Time = Pr_Reset_Delay; //��λ��ʱ
							if (F_Reseted == 0)					  //���û���ڳ�ʼ�㣬����г�ʼ��λ
								Reset_Routine();				  //����ǰ����ִ�и�λ����
						}
						//							}
					}
				}
			}

			//�ж��Ƿ���������������
			//����ǣ��������ʱ
			if (Pw_TouchRunStop == 0 && F_AskStop == 0 && F_Starting == 1)
			{
				if (F_Reseted == 0) //���û���ڳ�ʼ�㣬����ʱ
				{
					if (T_ResetDelay2 != SClk10Ms)
					{
						T_ResetDelay2 = SClk10Ms;
						C_ResetDelay2++;
						if (C_ResetDelay2 > Pr_Reset_Delay && Pr_F_AllStopped != 0) //��ʱ3s
						{
							C_ResetDelay2 = 0;
							F_Starting = 0; //�������̽���
						}
					}
				}
				else				//����Ѿ��ڳ�ʼ�㣬������ʱ
					F_Starting = 0; //�������̽���
			}
			//����������̽������������������
			else if (Pw_TouchRunStop == 0 && F_AskStop == 0 && F_Starting == 0 && F_Stoping == 0 && F_ForceReseting == 0) //���������
			{																											  //�����ŷ�����ͣ�����ܣ�=1��ͣ����=0����ͣ������Ȼ����
				//����û�дﵽָ������Ȧ��
				if ((Pw_Fault_Stop & Pr_F_HaveFault) == 0 && (Pw_ComErr_Stop & Pr_F_ComErr) == 0)
				{
					if (Pr_RUN_Count_Set == 0 || (Pr_RUN_Count_Set > 0 && Pr_RUN_Count < Pr_RUN_Count_Set))
					{
						Send_Pos_Data();						  //ֻ����λ������
						F_Sync_6_axis = 1;						  //��6��ͬ�����б�־
						Normal_Run();							  //��������������������
						Pw_EquipStatus = Pw_EquipStatus & 0xFFF8; //��3��״̬λ
					}
					else
					{
						Pw_TouchRunStop = 1; //ֻ�л���ֹͣ״̬�����建������Ҳ��ɲ��
						F_SendStopCMD2 = 0;
						F_Sync_6_axis = 0; //��6��ͬ�����б�־
						if (Pr_RUN_Count >= Pr_RUN_Count_Set)
						{
							Pw_EquipStatus = Pw_EquipStatus | 0x0004; //=4���ﵽ����Ȧ��ͣ��
						}
					}
				}
				else
				{
					Clear_Cmd_Queue();	 //���������
					Pw_StepAutoMode = 1; //�ֶ�ģʽ
					Pw_TouchRunStop = 1; //ֹͣģʽ
					F_SendStopCMD2 = 0;
					Pr_BRAKE_Control = 1; //�й��ϣ�ɲ��
					START_BRAKE_SYSTEM;	  //ɲ��

					if ((Pw_Fault_Stop & Pr_F_HaveFault) != 0)
					{
						Pw_EquipStatus = Pw_EquipStatus | 0x0001; //=1���ŷ�����ͣ��
					}
					else if ((Pw_ComErr_Stop & Pr_F_ComErr) != 0)
					{
						Pw_EquipStatus = Pw_EquipStatus | 0x0002; //=2��ͨѶ����ͣ��
					}
				}
			}
		}

		//ִ�д���ͣ��
		if (Pw_TouchRunStop == 1 && F_TouchRunStop == 0 && F_Stoping == 0)
		{
			F_PowerOnRun = 1;
			F_TouchRunStop = 1;
			F_AskStop = 1;
			F_Stoping = 1;
			Pw_EquipStatus = Pw_EquipStatus | 0x0008; //=8������ͣ��
			CLOSE_HAND;

			F_Sync_6_axis = 0; //��6��ͬ�����б�־
			Pr_Send_Data_F = 0;

			F_Driver1_Timeout = 0; //���־λ
			Pr_Driver1_Cmd_OK_F = 0;
			F_Driver1_Cmd_Err = 0;
			Driver1_Pos_Start_Sort = 0; //=0����ʾ��δд�뵽�������
			Pr_Driver1_Control_OK_F = 0;
			Driver1_Status_Sort = 0; //=0����ʾ��δд�뵽�������
			F_Driver1_Cmd_Con_Err = 0;

			F_Driver2_Timeout = 0; //���־λ
			Pr_Driver2_Cmd_OK_F = 0;
			F_Driver2_Cmd_Err = 0;
			Driver2_Pos_Start_Sort = 0;
			Pr_Driver2_Control_OK_F = 0;
			Driver2_Status_Sort = 0; //=0����ʾ��δд�뵽�������
			F_Driver2_Cmd_Con_Err = 0;

			F_Driver3_Timeout = 0; //���־λ
			Pr_Driver3_Cmd_OK_F = 0;
			F_Driver3_Cmd_Err = 0;
			Driver3_Pos_Start_Sort = 0;
			Pr_Driver3_Control_OK_F = 0;
			Driver3_Status_Sort = 0; //=0����ʾ��δд�뵽�������
			F_Driver3_Cmd_Con_Err = 0;

			F_Driver4_Timeout = 0; //���־λ
			Pr_Driver4_Cmd_OK_F = 0;
			F_Driver4_Cmd_Err = 0;
			Driver4_Pos_Start_Sort = 0;
			Pr_Driver4_Control_OK_F = 0;
			Driver4_Status_Sort = 0; //=0����ʾ��δд�뵽�������
			F_Driver4_Cmd_Con_Err = 0;

			F_Driver5_Timeout = 0; //���־λ
			Pr_Driver5_Cmd_OK_F = 0;
			F_Driver5_Cmd_Err = 0;
			Driver5_Pos_Start_Sort = 0;
			Pr_Driver5_Control_OK_F = 0;
			Driver5_Status_Sort = 0; //=0����ʾ��δд�뵽�������
			F_Driver5_Cmd_Con_Err = 0;

			F_Driver6_Timeout = 0; //���־λ
			Pr_Driver6_Cmd_OK_F = 0;
			F_Driver6_Cmd_Err = 0;
			Driver6_Pos_Start_Sort = 0;
			Pr_Driver6_Control_OK_F = 0;
			Driver6_Status_Sort = 0; //=0����ʾ��δд�뵽�������
			F_Driver6_Cmd_Con_Err = 0;
		}

		//�ж��Ƿ�ͣ������
		//����ǣ��������ʱ
		if (F_AskStop == 1 && F_Stoping == 1)
		{
			if (T_ResetDelay3 != SClk10Ms)
			{
				T_ResetDelay3 = SClk10Ms;
				C_ResetDelay3++;
				if (C_ResetDelay3 > 10 && Pr_F_AllStopped != 0) //���е����ֹͣ��������ʱ10*10ms=100ms
				{
					C_ResetDelay3 = 0;
					F_Stoping = 0; //=0����ʾͣ�����
					F_AskStop = 0;

					arr_p1 = &w_ParLst_Pos_CMD; //ִ�е�һ��ָ��
					arrp_p1_Last = arr_p1;
				}
			}
			else
				F_Stoping = 1;
		}
	}
	else if (Pw_StepAutoMode == 1 || (Pw_StepAutoMode == 0 && F_StepMode == 0 && F_ForceReseting == 1)) //=1���ֶ�ģʽ
	{
		F_TouchRunStop = 1;
		Pw_EquipStatus = Pw_EquipStatus | 0x0010; //=16���ֶ�״̬

		if (F_StepMode == 0) //�����û��λ����û�����ֶ�ģʽ����ִ�и�λ
		{
			if (!F_TouchForceReSet && F_Stoping == 0) //w_ParLst[317]=2��ǿ�Ƹ�λ�������
			{
				//					Send_StopCMD();								//����ͣ������29

				//��ʱ
				if (T_StepAutoDelay2 != SClk10Ms)
				{
					T_StepAutoDelay2 = SClk10Ms;

					C_T_StepAutoDelayDelay2++;
					if (C_T_StepAutoDelayDelay2 > 200) //��ʱ2s
					{
						C_T_StepAutoDelayDelay2 = 0;

						F_TouchForceReSet = 1;
						F_AskForceReset = 1;
						F_ForceReseting = 1;
						F_PowerOnRun = 1;
						F_Stoping = 1;
					}
				}
			}

			if (F_AskForceReset == 1 && F_TouchForceReSet == 1 && F_ForceReseting == 1)
			{
				if (T_ResetDelay2 != SClk10Ms)
				{
					T_ResetDelay2 = SClk10Ms;
					C_ResetDelay2++;
					if (C_ResetDelay2 > Pr_Reset_Delay && Pr_F_AllStopped != 0) //��ʱ2s
					{
						C_ResetDelay2 = 0;

						F_ForceReseting = 0;
						F_AskForceReset = 0;
						F_Stoping = 0; //=0����ʾͣ�����
						F_TouchForceReSet = 0;
						//					Pw_TouchRunStop=0;
						F_StepMode = 1; //��λͣ���󣬽����ֶ�ģʽ

						arr_p1 = &w_ParLst_Pos_CMD; //ִ�е�һ��ָ��
						arrp_p1_Last = arr_p1;
						Pr_Send_Data_F = 0;
					}
				}
			}
		}
		else
		{
			CLOSE_HAND;
			if (Pw_Step_Pos_CMD != 0 && Pw_Running_Pos_CMD >= 1 && Pw_Running_Pos_CMD <= (COM_CMD_NUM - 1) && Pr_F_AllStopped == 1)
			{
				tmp_arr_p1 = &w_ParLst_Pos_CMD;
				Run_to_One_Pos(tmp_arr_p1[(Pw_Running_Pos_CMD - 1) * POS_CMD_SIZE + 1]); //���ֶ���ʽͬ�����е�ĳһ��
				Pw_Step_Pos_CMD = 0;
			}
			else if (Pw_Step_Pos_CMD == 0 && Pr_F_AllStopped == 1)
			{
				//1#�ֶ�����ת
				if (Pw_Driver1_Enable == 1 && Pw_Driver1_R_Enable == 0)
				{
					Locate_Rle_1(Pw_EquipmentNo1, (Pw_Driver1_Pluse_HW << 16) + Pw_Driver1_Pluse, Pw_Driver1_Speed, Pw_Driver1_AccTime, 0);
					Pw_Driver1_Enable = 0;
					Driver1_Pos_Start_Sort = 1; //=1����ʾ�Ѿ�д�뵽�������
					Pr_Send_Data_F &= 0xFE;
				}
				else if (Pw_Driver1_Enable == 0 && Pw_Driver1_R_Enable == 1) //1#�ֶ�����ת
				{
					Locate_Rle_1(Pw_EquipmentNo1, -((Pw_Driver1_Pluse_HW << 16) + Pw_Driver1_Pluse), Pw_Driver1_Speed, Pw_Driver1_AccTime, 0);
					Pw_Driver1_R_Enable = 0;
					Driver1_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xFE;
				}

				//2#
				if (Pw_Driver2_Enable == 1 && Pw_Driver2_R_Enable == 0)
				{
					Locate_Rle_1(Pw_EquipmentNo2, (Pw_Driver2_Pluse_HW << 16) + Pw_Driver2_Pluse, Pw_Driver2_Speed, Pw_Driver2_AccTime, 0);
					Pw_Driver2_Enable = 0;
					Driver2_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xFD;
				}
				else if (Pw_Driver2_Enable == 0 && Pw_Driver2_R_Enable == 1)
				{
					Locate_Rle_1(Pw_EquipmentNo2, -((Pw_Driver2_Pluse_HW << 16) + Pw_Driver2_Pluse), Pw_Driver2_Speed, Pw_Driver2_AccTime, 0);
					Pw_Driver2_R_Enable = 0;
					Driver2_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xFD;
				}

				//3#
				if (Pw_Driver3_Enable == 1 && Pw_Driver3_R_Enable == 0)
				{
					Locate_Rle_1(Pw_EquipmentNo3, (Pw_Driver3_Pluse_HW << 16) + Pw_Driver3_Pluse, Pw_Driver3_Speed, Pw_Driver3_AccTime, 0);
					Pw_Driver3_Enable = 0;
					Driver3_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xFB;
				}
				else if (Pw_Driver3_Enable == 0 && Pw_Driver3_R_Enable == 1)
				{
					Locate_Rle_1(Pw_EquipmentNo3, -((Pw_Driver3_Pluse_HW << 16) + Pw_Driver3_Pluse), Pw_Driver3_Speed, Pw_Driver3_AccTime, 0);
					Pw_Driver3_R_Enable = 0;
					Driver3_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xFB;
				}

				//4#
				if (Pw_Driver4_Enable == 1 && Pw_Driver4_R_Enable == 0)
				{
					Locate_Rle_1(Pw_EquipmentNo4, (Pw_Driver4_Pluse_HW << 16) + Pw_Driver4_Pluse, Pw_Driver4_Speed, Pw_Driver4_AccTime, 0);
					Pw_Driver4_Enable = 0;
					Driver4_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xF7;
				}
				else if (Pw_Driver4_Enable == 0 && Pw_Driver4_R_Enable == 1)
				{
					Locate_Rle_1(Pw_EquipmentNo4, -((Pw_Driver4_Pluse_HW << 16) + Pw_Driver4_Pluse), Pw_Driver4_Speed, Pw_Driver4_AccTime, 0);
					Pw_Driver4_R_Enable = 0;
					Driver4_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xF7;
				}

				//5#
				if (Pw_Driver5_Enable == 1 && Pw_Driver5_R_Enable == 0)
				{
					Locate_Rle_1(Pw_EquipmentNo5, (Pw_Driver5_Pluse_HW << 16) + Pw_Driver5_Pluse, Pw_Driver5_Speed, Pw_Driver5_AccTime, 0);
					Pw_Driver5_Enable = 0;
					Driver5_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xEF;
				}
				else if (Pw_Driver5_Enable == 0 && Pw_Driver5_R_Enable == 1)
				{
					Locate_Rle_1(Pw_EquipmentNo5, -((Pw_Driver5_Pluse_HW << 16) + Pw_Driver5_Pluse), Pw_Driver5_Speed, Pw_Driver5_AccTime, 0);
					Pw_Driver5_R_Enable = 0;
					Driver5_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xEF;
				}

				//6#
				if (Pw_Driver6_Enable == 1 && Pw_Driver6_R_Enable == 0)
				{
					Locate_Rle_1(Pw_EquipmentNo6, (Pw_Driver6_Pluse_HW << 16) + Pw_Driver6_Pluse, Pw_Driver6_Speed, Pw_Driver6_AccTime, 0);
					Pw_Driver6_Enable = 0;
					Driver6_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xDF;
				}
				else if (Pw_Driver6_Enable == 0 && Pw_Driver6_R_Enable == 1)
				{
					Locate_Rle_1(Pw_EquipmentNo6, -((Pw_Driver6_Pluse_HW << 16) + Pw_Driver6_Pluse), Pw_Driver6_Speed, Pw_Driver6_AccTime, 0);
					Pw_Driver6_R_Enable = 0;
					Driver6_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xDF;
				}
			}
		}
	}
}

//����λ������
void Send_Pos_Data(void)
{
	if (Pw_TouchRunStop == 0 && F_Sync_6_axis == 1)
	{
		//����λ��ָ��ֻ���һ�Σ�=0�ſ�����䣬��������ϱ�Ϊ1
		if (Driver1_Pos_Start_Sort == 0)
		{
			Fill_Pos_Data(Pw_EquipmentNo1, 0, 0, 0, arr_p1[1]); //ͨ��ͨѶ��������ָ��
			Driver1_Pos_Start_Sort = 1;							//=1����ʾ�Ѿ�д�뵽�������
			Pr_Send_Data_F &= 0xFE;
		}

		if (Driver2_Pos_Start_Sort == 0)
		{
			Fill_Pos_Data(Pw_EquipmentNo2, 0, 0, 0, arr_p1[1]); //ͨ��ͨѶ��������ָ��
			Driver2_Pos_Start_Sort = 1;
			Pr_Send_Data_F &= 0xFD;
		}

		if (Driver3_Pos_Start_Sort == 0)
		{
			Fill_Pos_Data(Pw_EquipmentNo3, 0, 0, 0, arr_p1[1]); //ͨ��ͨѶ��������ָ��
			Driver3_Pos_Start_Sort = 1;
			Pr_Send_Data_F &= 0xFB;
		}

		if (Driver4_Pos_Start_Sort == 0)
		{
			Fill_Pos_Data(Pw_EquipmentNo4, 0, 0, 0, arr_p1[1]); //ͨ��ͨѶ��������ָ��
			Driver4_Pos_Start_Sort = 1;
			Pr_Send_Data_F &= 0xF7;
		}

		if (Driver5_Pos_Start_Sort == 0)
		{
			Fill_Pos_Data(Pw_EquipmentNo5, 0, 0, 0, arr_p1[1]); //ͨ��ͨѶ��������ָ��
			Driver5_Pos_Start_Sort = 1;
			Pr_Send_Data_F &= 0xEF;
		}

		if (Driver6_Pos_Start_Sort == 0)
		{
			Fill_Pos_Data(Pw_EquipmentNo6, 0, 0, 0, arr_p1[1]); //ͨ��ͨѶ��������ָ��
			Driver6_Pos_Start_Sort = 1;
			Pr_Send_Data_F &= 0xDF;
		}
	}
}

//У��λ�������Ƿ���ȷ
void Verify_Pos_CMD(void)
{
	if (Pw_TouchRunStop == 0 && F_Sync_6_axis == 1)
	{
		//���������֣������ж�
		if ((((Pw_ComWriteErr_Stop & 0x01) == 0x01) & F_Driver1_Cmd_Con_Err) ||
			(((Pw_ComWriteErr_Stop & 0x02) == 0x02) & F_Driver2_Cmd_Con_Err) ||
			(((Pw_ComWriteErr_Stop & 0x04) == 0x04) & F_Driver3_Cmd_Con_Err) ||
			(((Pw_ComWriteErr_Stop & 0x08) == 0x08) & F_Driver4_Cmd_Con_Err) ||
			(((Pw_ComWriteErr_Stop & 0x10) == 0x10) & F_Driver5_Cmd_Con_Err) ||
			(((Pw_ComWriteErr_Stop & 0x20) == 0x20) & F_Driver6_Cmd_Con_Err))
			F_HaveDriver_Cmd_Con_Err = 1;
		else
			F_HaveDriver_Cmd_Con_Err = 0;

		//�ж��Ƿ�д��ɹ�
		//Pw_ComWriteErr_Stop�ĵ�6λ��Ϊ����λ����Ӧλ=0�����Σ������Ƿ�дOK�������У�����Ӧλ=1�������Σ���Ȼ��ԭ״ִ̬�У�
		if (((((Pw_ComWriteErr_Stop & 0x01) == 0x01) & !Pr_Driver1_Cmd_OK_F)) ||
			((((Pw_ComWriteErr_Stop & 0x02) == 0x02) & !Pr_Driver2_Cmd_OK_F)) ||
			((((Pw_ComWriteErr_Stop & 0x04) == 0x04) & !Pr_Driver3_Cmd_OK_F)) ||
			((((Pw_ComWriteErr_Stop & 0x08) == 0x08) & !Pr_Driver4_Cmd_OK_F)) ||
			((((Pw_ComWriteErr_Stop & 0x10) == 0x10) & !Pr_Driver5_Cmd_OK_F)) ||
			((((Pw_ComWriteErr_Stop & 0x20) == 0x20) & !Pr_Driver6_Cmd_OK_F)))
			Pr_AllDriver_Cmd_OK_F = 0;
		else
			Pr_AllDriver_Cmd_OK_F = 1;

		//�ж��Ƿ���д�����
		if ((((Pw_ComWriteErr_Stop & 0x01) == 0x01) & F_Driver1_Cmd_Err) ||
			(((Pw_ComWriteErr_Stop & 0x02) == 0x02) & F_Driver2_Cmd_Err) ||
			(((Pw_ComWriteErr_Stop & 0x04) == 0x04) & F_Driver3_Cmd_Err) ||
			(((Pw_ComWriteErr_Stop & 0x08) == 0x08) & F_Driver4_Cmd_Err) ||
			(((Pw_ComWriteErr_Stop & 0x10) == 0x10) & F_Driver5_Cmd_Err) ||
			(((Pw_ComWriteErr_Stop & 0x20) == 0x20) & F_Driver6_Cmd_Err))
			//		if(F_Driver4_Cmd_Err)
			F_HaveDriver_Cmd_Err = 1;
		else
			F_HaveDriver_Cmd_Err = 0;

		//���д����󣬻��߳�ʱ����ͣ�����ñ�־λ
		if ((Pw_ComWriteErr_Stop != 0) & (F_HaveDriver_Cmd_Err || F_HaveDriver_Cmd_Con_Err))
		{
			Pr_HaveDriver_Cmd_Err_F = 1;

			Clear_Cmd_Queue();	 //���������
								 //			Pw_StepAutoMode=1;				//�ֶ�ģʽ
			Pw_TouchRunStop = 1; //ֹͣģʽ
			F_SendStopCMD2 = 0;
			Pr_BRAKE_Control = 1;					  //�й��ϣ�ɲ��
			START_BRAKE_SYSTEM;						  //ɲ��
			Pw_EquipStatus = Pw_EquipStatus | 0x0200; //=512��λ��д�����ͣ��
		}
		else
		{
			Pr_HaveDriver_Cmd_Err_F = 0;
			Pw_EquipStatus = Pw_EquipStatus & 0xFDFF; //��λ��д�����ͣ����־λ
		}
	}
}

//��������λ�������ʼλ��
void Reset_Drivers(void)
{
	if (Pw_ResetCMD) //����и�λ����
	{
		if (Pw_StepAutoMode == 1 && Pr_AllRun == 0) //������ֶ�ģʽ�����������е����ֹͣ
		{
			Reset_Routine();
			Pw_ResetCMD = 0;
		}
	}
}

//��������λ�ӳ���
void Reset_Routine(void)
{
	float num1, num2, num3, num4, num5, num6;
	float tmp1_f;
	s16 temp_MUTI3, temp_MUTI4, temp_MUTI5, temp_MUTI6;
	float temp_SINGLE1, temp_SINGLE2, temp_SINGLE3, temp_SINGLE4, temp_SINGLE5, temp_SINGLE6;
	s32 temp_pulse1, temp_pulse2, temp_pulse3, temp_pulse4, temp_pulse5, temp_pulse6;
	float Reset_time_float[6];
	float tmp_time;
	s32 Reset_time_I[6];

	Pr_BRAKE_Control = 0; //��ɲ��
	STOP_BRAKE_SYSTEM;	  //��������
	Clear_Cmd_Queue();	  //���������

	F_Resetting = 1; //�����ڸ�λ��־
	Pr_Send_Data_F = 0;

	arr_p1 = &w_ParLst_Pos_CMD; //ִ�е�һ��ָ��
	arrp_p1_Last = arr_p1;
	Pr_Driver_Running_No = 0;  //��ǰ����ָ���=0
	Pr_Driver_Previous_No = 0; //ǰһ��ִ��ָ���

	//����1#����
	//		temp_MUTI=Pr_Drive1_MultiData_Init-Pr_Drive1_MultiData;
	temp_SINGLE1 = ((Pr_Drive1_singleData_Init_HW << 16) + Pr_Drive1_singleData_Init) - ((Pr_Drive1_singleData_HW << 16) + Pr_Drive1_singleData);
	//		num=temp_MUTI*PULSE_NUM+temp_SINGLE*PULSE_NUM/ELEC_GEAR;
	//		num=temp_SINGLE*PULSE_NUM/ELEC_GEAR_US200;
	num1 = (float)temp_SINGLE1;
	num1 = (float)num1 * PULSE_NUM;
	num1 = (float)num1 / ELEC_GEAR_US200;

	if (num1 > 0.0f)
		temp_pulse1 = (s32)(num1 + 0.5f);
	else
		temp_pulse1 = (s32)(num1 - 0.5f);
	Locate_Rle_1(Pw_EquipmentNo1, temp_pulse1, Pw_Driver1_Speed, Pw_Driver1_AccTime, 0);
	//		Fill_Pos_Data(Pw_EquipmentNo1,temp_pulse1,Pw_Driver1_Speed,Pw_Driver1_AccTime,0);
	Driver1_Pos_Start_Sort = 1;

	//����1#����ʱ��
	Reset_time_float[0] = abs(temp_pulse1);
	Reset_time_float[0] = (float)Reset_time_float[0] * 60000;
	Reset_time_float[0] = (float)Reset_time_float[0] / (PULSE_NUM * Pw_Driver1_Speed);
	tmp_time = (float)Pw_Driver1_AccTime;
	tmp_time = (float)tmp_time * Pw_Acc_Delay_Ratio / 100;
	Reset_time_float[0] = (float)(Reset_time_float[0] + tmp_time);

	//����2#����
	//		temp_MUTI=Pr_Drive2_MultiData_Init-Pr_Drive2_MultiData;
	temp_SINGLE2 = ((Pr_Drive2_singleData_Init_HW << 16) + Pr_Drive2_singleData_Init) - ((Pr_Drive2_singleData_HW << 16) + Pr_Drive2_singleData);
	//		num=temp_MUTI*PULSE_NUM+temp_SINGLE*PULSE_NUM/ELEC_GEAR;
	//		num=temp_SINGLE*PULSE_NUM/ELEC_GEAR_US200;
	num2 = (float)temp_SINGLE2;
	num2 = (float)num2 * PULSE_NUM;
	num2 = (float)num2 / ELEC_GEAR_US200;
	if (num2 > 0.0f)
		temp_pulse2 = (s32)(num2 + 0.5f);
	else
		temp_pulse2 = (s32)(num2 - 0.5f);
	Locate_Rle_1(Pw_EquipmentNo2, temp_pulse2, Pw_Driver2_Speed, Pw_Driver2_AccTime, 0);
	Driver2_Pos_Start_Sort = 1;

	//����2#����ʱ��
	Reset_time_float[1] = abs(temp_pulse2);
	Reset_time_float[1] = (float)Reset_time_float[1] * 60000;
	Reset_time_float[1] = (float)Reset_time_float[1] / (PULSE_NUM * Pw_Driver2_Speed);
	tmp_time = (float)Pw_Driver2_AccTime;
	tmp_time = (float)tmp_time * Pw_Acc_Delay_Ratio / 100;
	Reset_time_float[1] = (float)(Reset_time_float[1] + tmp_time);

	//����3#����
	temp_MUTI3 = Pr_Drive3_MultiData_Init - Pr_Drive3_MultiData;
	temp_SINGLE3 = ((Pr_Drive3_singleData_Init_HW << 16) + Pr_Drive3_singleData_Init) - ((Pr_Drive3_singleData_HW << 16) + Pr_Drive3_singleData);
	//		num=temp_MUTI*PULSE_NUM+temp_SINGLE*PULSE_NUM/ELEC_GEAR;
	num3 = (float)temp_MUTI3;
	num3 = (float)num3 * PULSE_NUM;
	tmp1_f = (float)temp_SINGLE3;
	tmp1_f = (float)tmp1_f * PULSE_NUM / ELEC_GEAR;
	num3 = (float)(num3 + tmp1_f);

	if (num3 > 0.0f)
		temp_pulse3 = (s32)(num3 + 0.5f);
	else
		temp_pulse3 = (s32)(num3 - 0.5f);
	Locate_Rle_1(Pw_EquipmentNo3, temp_pulse3, Pw_Driver3_Speed, Pw_Driver3_AccTime, 0);
	Driver3_Pos_Start_Sort = 1;

	//����3#����ʱ��
	Reset_time_float[2] = abs(temp_pulse3);
	Reset_time_float[2] = (float)Reset_time_float[2] * 60000;
	Reset_time_float[2] = (float)Reset_time_float[2] / (PULSE_NUM * Pw_Driver3_Speed);
	tmp_time = (float)Pw_Driver3_AccTime;
	tmp_time = (float)tmp_time * Pw_Acc_Delay_Ratio / 100;
	Reset_time_float[2] = (float)(Reset_time_float[2] + tmp_time);

	//����4#����
	temp_MUTI4 = Pr_Drive4_MultiData_Init - Pr_Drive4_MultiData;
	temp_SINGLE4 = ((Pr_Drive4_singleData_Init_HW << 16) + Pr_Drive4_singleData_Init) - ((Pr_Drive4_singleData_HW << 16) + Pr_Drive4_singleData);
	//		num=temp_MUTI*PULSE_NUM+temp_SINGLE*PULSE_NUM/ELEC_GEAR;
	num4 = (float)temp_MUTI4;
	num4 = (float)num4 * PULSE_NUM;
	tmp1_f = (float)temp_SINGLE4;
	tmp1_f = (float)tmp1_f * PULSE_NUM / ELEC_GEAR;
	num4 = (float)(num4 + tmp1_f);

	if (num4 > 0.0f)
		temp_pulse4 = (s32)(num4 + 0.5f);
	else
		temp_pulse4 = (s32)(num4 - 0.5f);
	Locate_Rle_1(Pw_EquipmentNo4, temp_pulse4, Pw_Driver4_Speed, Pw_Driver4_AccTime, 0);
	Driver4_Pos_Start_Sort = 1;

	//����4#����ʱ��
	Reset_time_float[3] = abs(temp_pulse4);
	Reset_time_float[3] = (float)Reset_time_float[3] * 60000;
	Reset_time_float[3] = (float)Reset_time_float[3] / (PULSE_NUM * Pw_Driver4_Speed);
	tmp_time = (float)Pw_Driver4_AccTime;
	tmp_time = (float)tmp_time * Pw_Acc_Delay_Ratio / 100;
	Reset_time_float[3] = (float)(Reset_time_float[3] + tmp_time);

	//����5#����
	temp_MUTI5 = Pr_Drive5_MultiData_Init - Pr_Drive5_MultiData;
	temp_SINGLE5 = ((Pr_Drive5_singleData_Init_HW << 16) + Pr_Drive5_singleData_Init) - ((Pr_Drive5_singleData_HW << 16) + Pr_Drive5_singleData);
	//		num=temp_MUTI*PULSE_NUM+temp_SINGLE*PULSE_NUM/ELEC_GEAR;
	num5 = (float)temp_MUTI5;
	num5 = (float)num5 * PULSE_NUM;
	tmp1_f = (float)temp_SINGLE5;
	tmp1_f = (float)tmp1_f * PULSE_NUM / ELEC_GEAR;
	num5 = (float)(num5 + tmp1_f);

	if (num5 > 0.0f)
		temp_pulse5 = (s32)(num5 + 0.5f);
	else
		temp_pulse5 = (s32)(num5 - 0.5f);
	Locate_Rle_1(Pw_EquipmentNo5, temp_pulse5, Pw_Driver5_Speed, Pw_Driver5_AccTime, 0);
	Driver5_Pos_Start_Sort = 1;

	//����5#����ʱ��
	Reset_time_float[4] = abs(temp_pulse5);
	Reset_time_float[4] = (float)Reset_time_float[4] * 60000;
	Reset_time_float[4] = (float)Reset_time_float[4] / (PULSE_NUM * Pw_Driver5_Speed);
	tmp_time = (float)Pw_Driver5_AccTime;
	tmp_time = (float)tmp_time * Pw_Acc_Delay_Ratio / 100;
	Reset_time_float[4] = (float)(Reset_time_float[4] + tmp_time);

	//����6#����
	temp_MUTI6 = Pr_Drive6_MultiData_Init - Pr_Drive6_MultiData;
	temp_SINGLE6 = ((Pr_Drive6_singleData_Init_HW << 16) + Pr_Drive6_singleData_Init) - ((Pr_Drive6_singleData_HW << 16) + Pr_Drive6_singleData);
	//		num=temp_MUTI*PULSE_NUM+temp_SINGLE*PULSE_NUM/ELEC_GEAR;
	num6 = (float)temp_MUTI6;
	num6 = (float)num6 * PULSE_NUM;
	tmp1_f = (float)temp_SINGLE6;
	tmp1_f = (float)tmp1_f * PULSE_NUM / ELEC_GEAR;
	num6 = (float)(num6 + tmp1_f);

	if (num6 > 0.0f)
		temp_pulse6 = (s32)(num6 + 0.5f);
	else
		temp_pulse6 = (s32)(num6 - 0.5f);
	Locate_Rle_1(Pw_EquipmentNo6, temp_pulse6, Pw_Driver6_Speed, Pw_Driver6_AccTime, 0);
	Driver6_Pos_Start_Sort = 1;

	//����6#����ʱ��
	Reset_time_float[5] = abs(temp_pulse6);
	Reset_time_float[5] = (float)Reset_time_float[5] * 60000;
	Reset_time_float[5] = (float)Reset_time_float[5] / (PULSE_NUM * Pw_Driver6_Speed);
	tmp_time = (float)Pw_Driver6_AccTime;
	tmp_time = (float)tmp_time * Pw_Acc_Delay_Ratio / 100;
	Reset_time_float[5] = (float)(Reset_time_float[5] + tmp_time);

	Reset_time_I[0] = (s32)Reset_time_float[0];
	Reset_time_I[1] = (s32)Reset_time_float[1];
	Reset_time_I[2] = (s32)Reset_time_float[2];
	Reset_time_I[3] = (s32)Reset_time_float[3];
	Reset_time_I[4] = (s32)Reset_time_float[4];
	Reset_time_I[5] = (s32)Reset_time_float[5];

	sort(Reset_time_I, 6); //�õ�����ʱ������ֵ���ŵ����һ����
	Reset_time_Max = abs(Reset_time_I[5]);
	Pw_Current_Run_Time = Reset_time_Max / 10;
}

//�жϸ�λ��ԭ���ӳ���
void is_Reseted(void)
{
	u32 temp_PosError_Set;
	float num1, num2, num3, num4, num5, num6;
	s16 temp_MUTI;
	float temp_SINGLE;

	//λ��ƫ��
	temp_PosError_Set = (Pw_PosError_Set_HW << 16) + Pw_PosError_Set;
	if (Pr_AllRun == 0) //���е����ֹͣ���ж��Ƿ���ԭ��
	{
		//		temp_MUTI=Pr_Drive1_MultiData_Init-Pr_Drive1_MultiData;
		temp_SINGLE = (float)((Pr_Drive1_singleData_Init_HW << 16) + Pr_Drive1_singleData_Init) - ((Pr_Drive1_singleData_HW << 16) + Pr_Drive1_singleData);
		//		num1=temp_MUTI*ELEC_GEAR+temp_SINGLE;
		num1 = (float)temp_SINGLE;

		//		temp_MUTI=Pr_Drive2_MultiData_Init-Pr_Drive2_MultiData;
		temp_SINGLE = (float)((Pr_Drive2_singleData_Init_HW << 16) + Pr_Drive2_singleData_Init) - ((Pr_Drive2_singleData_HW << 16) + Pr_Drive2_singleData);
		//		num2=temp_MUTI*ELEC_GEAR+temp_SINGLE;
		num2 = (float)temp_SINGLE;

		temp_MUTI = Pr_Drive3_MultiData_Init - Pr_Drive3_MultiData;
		temp_SINGLE = (float)((Pr_Drive3_singleData_Init_HW << 16) + Pr_Drive3_singleData_Init) - ((Pr_Drive3_singleData_HW << 16) + Pr_Drive3_singleData);
		num3 = (float)temp_MUTI;
		num3 = (float)(num3 * ELEC_GEAR + temp_SINGLE);

		temp_MUTI = Pr_Drive4_MultiData_Init - Pr_Drive4_MultiData;
		temp_SINGLE = (float)((Pr_Drive4_singleData_Init_HW << 16) + Pr_Drive4_singleData_Init) - ((Pr_Drive4_singleData_HW << 16) + Pr_Drive4_singleData);
		num4 = (float)temp_MUTI;
		num4 = (float)(num4 * ELEC_GEAR + temp_SINGLE);

		temp_MUTI = Pr_Drive5_MultiData_Init - Pr_Drive5_MultiData;
		temp_SINGLE = (float)((Pr_Drive5_singleData_Init_HW << 16) + Pr_Drive5_singleData_Init) - ((Pr_Drive5_singleData_HW << 16) + Pr_Drive5_singleData);
		num5 = (float)temp_MUTI;
		num5 = (float)(num5 * ELEC_GEAR + temp_SINGLE);

		temp_MUTI = Pr_Drive6_MultiData_Init - Pr_Drive6_MultiData;
		temp_SINGLE = (float)((Pr_Drive6_singleData_Init_HW << 16) + Pr_Drive6_singleData_Init) - ((Pr_Drive6_singleData_HW << 16) + Pr_Drive6_singleData);
		num6 = (float)temp_MUTI;
		num6 = (float)(num6 * ELEC_GEAR + temp_SINGLE);

		if (fabsf(num1) < temp_PosError_Set && fabsf(num2) < temp_PosError_Set && fabsf(num3) < temp_PosError_Set && fabsf(num4) < temp_PosError_Set && fabsf(num5) < temp_PosError_Set && fabsf(num6) < temp_PosError_Set)
		{
			F_Reseted = 1; //=1��λ�ýӽ��������Ѿ�ͣ�������ø�λ��ԭ���־
		}
		else
			F_Reseted = 0;
	}
}

//����ͣ�������ӳ���
void Send_StopCMD(void)
{
	Stop_Driver(Pw_EquipmentNo1);
	Stop_Driver(Pw_EquipmentNo2);
	Stop_Driver(Pw_EquipmentNo3);
	Stop_Driver(Pw_EquipmentNo4);
	Stop_Driver(Pw_EquipmentNo5);
	Stop_Driver(Pw_EquipmentNo6);
}

//д��������������
//���룺��������driver_no�������CMD_No��������ַPar_addr����������Par_content
void Send_Driver_CMD(u8 driver_no, u8 Modbus_CMD_No, u32 Par_addr, u32 Par_content)
{
	s16 Queue_Rear;

	if (driver_no == Pw_EquipmentNo1)
	{
		Com_Write_p = &Com1_Driver1_Write_BUFF[0];
	}
	else if (driver_no == Pw_EquipmentNo2)
	{
		Com_Write_p = &Com1_Driver2_Write_BUFF[0];
	}
	else if (driver_no == Pw_EquipmentNo3)
	{
		Com_Write_p = &Com2_Driver3_Write_BUFF[0];
	}
	else if (driver_no == Pw_EquipmentNo4)
	{
		Com_Write_p = &Com2_Driver4_Write_BUFF[0];
	}
	else if (driver_no == Pw_EquipmentNo5)
	{
		Com_Write_p = &Com3_Driver5_Write_BUFF[0];
	}
	else if (driver_no == Pw_EquipmentNo6)
	{
		Com_Write_p = &Com3_Driver6_Write_BUFF[0];
	}

	if (driver_no >= 1 && driver_no <= 6 && (Modbus_CMD_No == 3 || Modbus_CMD_No == 6))
	{
		Queue_Rear = Move_Cmd_Queue(driver_no);
		if (Queue_Rear != 0xFF)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear] = 12;
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 1] = 6;						   //ָ���6
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 2] = driver_no;				   //�����ַ
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 3] = Modbus_CMD_No;			   //Modbus���ܺ�03��06
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = (Par_addr & 0xFF00) >> 8; //US200��RA1�ŷ�������
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = Par_addr & 0x00FF;
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 6] = (Par_content & 0xFF00) >> 8; //д1����
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 7] = Par_content & 0x00FF;
		}
	}
}

//�������У���ֵ����PWM��
void Normal_Run(void)
{
	//�ж��Ƿ�Ҫ��ʱ
	if (arr_p1[5] > 0 && T_Driver1_delay != SClk10Ms && Driver1_delay_F == 0)
	{
		if (T_Driver1_delay != SClk10Ms)
		{
			T_Driver1_delay = SClk10Ms; //
			C_Driver1_delayCount++;
			Pr_pausetime_show = C_Driver1_delayCount;
			if (C_Driver1_delayCount > arr_p1[5]) //��ʱ��ϣ��ñ�־λ
			{
				C_Driver1_delayCount = 0;
				Pr_pausetime_show = 0;
				Driver1_delay_F = 1; //��ʱ������־λ
			}
		}
	}
	else if (((arr_p1[5]) == 0) || (((arr_p1[5]) > 0) && (Driver1_delay_F == 1))) //����ʱ������ʱ����
	{
		T_Driver1_delay = 1000;
		C_Driver1_delayCount = 0;

		//����Ȧ���趨
		if (Pr_RUN_Count_Set == 0 || (Pr_RUN_Count_Set > 0 && Pr_RUN_Count < Pr_RUN_Count_Set))
		{
			if ((arr_p1[6]) == 0) //ֹͣ��־λ=0����ʾ����ִ�У�����ֹͣ����
			{
				if ((arr_p1[7] & DI2) == 0 && (arr_p1[8] & DI3) == 0) //����1=1������DI2���źţ������У�����2=1������DI3���źţ������У���������
				{
					//6���������Ϊ�ղ�����䣬���κ�һ����Ϊ�գ�Ҳ�����
					if (Com1_Driver1_Queue_isEmpty() && Com1_Driver2_Queue_isEmpty() && Com2_Driver3_Queue_isEmpty() &&
						Com2_Driver4_Queue_isEmpty() && Com3_Driver5_Queue_isEmpty() && Com3_Driver6_Queue_isEmpty())
					{
						if (T_Driver1_FillCMD != SClk10Ms)
						{
							T_Driver1_FillCMD = SClk10Ms;
							C_Driver1_FillCMD++;
							Pr_runtime_show = C_Driver1_FillCMD;
							//							 Pw_Current_Run_Time=arrp_p1_Last[36];

							if (C_Driver1_FillCMD > Pw_Current_Run_Time) //Pw_Current_Run_Time  �����������Ϊ�պ�����ʱ��ָ������ʱ�䡱�ٷ�����һ��ָ��
							{
								//����ط��ټ�һ����ʱ�����е����ֹͣ����ʱһ��ʱ�䣬�ٷ�ָ��
								if (Pr_F_AllStopped != 0) //�������е��ֹͣ�����ܷ���һ��ָ��
								{
									C_AllStop++;
									if (C_AllStop > Pw_AllStopped_Delay) //�ټ�һ����ʱ�жϣ��൱��ȫ��ͣ��������ʱPw_AllStopped_Delay*10ms
									{
										C_AllStop = 0;
										C_Driver1_FillCMD = 0;

										//λ�������Ѿ��������
										if (Driver1_Pos_Start_Sort == 2 && Driver2_Pos_Start_Sort == 2 && Driver3_Pos_Start_Sort == 2 &&
											Driver4_Pos_Start_Sort == 2 && Driver5_Pos_Start_Sort == 2 && Driver6_Pos_Start_Sort == 2)
										{
											//����д����ȷ�����������������
											if (((Pw_ComWriteErr_Stop != 0) & (!Pr_AllDriver_Cmd_OK_F)) == 0) //�������λ������д����ȷ���Ż�����
											{
												Fill_Cmd();
												Driver1_delay_F = 0;
												C_DO_delayCount = 0;
												C_DO_Open_delayCount = 0;
												F_Close_Hand = 0;						  //���־λ�������ŷ���
												Pw_EquipStatus = Pw_EquipStatus & 0xFDFF; //���־λ��λ��д�����ͣ��
											}
											else
											{
												Pw_EquipStatus = Pw_EquipStatus | 0x0200; //=512��λ��д�����ͣ��
											}
										}
									}
								}
							}
						}
					}
					else
					{
						C_Driver1_FillCMD = 0;
					}
				}
				else
				{
					if ((arr_p1[7] & DI2) != 0)
					{
						Pw_EquipStatus = Pw_EquipStatus | 0x0040; //����1ͣ��
					}
					else if ((arr_p1[8] & DI3) != 0)
					{
						Pw_EquipStatus = Pw_EquipStatus | 0x0080; //����2ͣ��
					}
				}
			}
			else
			{
				Pw_EquipStatus = Pw_EquipStatus | 0x0100; //ͣ������ͣ��
			}
		}
	}
}

//�������
void Fill_Cmd(void)
{
	u32 temp_count;

	if (arr_p1[1] >= 1 && arr_p1[1] <= COM_CMD_NUM - 1) //��1���ֽ�Ϊ����ţ���1-29֮���ִ�У�=0��ִ��
	{
		if (arr_p1[2] >= 1 && arr_p1[2] <= POS_NUM) //��2���ֽ�Ϊλ�úţ���1-15֮���ִ�У�=0��ִ��
		{
			Pr_Driver_Previous_No = Pr_Driver_Running_No; //ǰһ��ִ��ָ���
			Pr_Driver_Running_No = arr_p1[1];			  //ĳ��ָ������ִ�У�Ҳ����ʾ��һ��ָ��ִ�����
			arrp_p1_Last = arr_p1;						  //������һ��ָ�������ָ��

			Send_Start_Cmd(Pw_EquipmentNo1);
			Send_Start_Cmd(Pw_EquipmentNo2);
			Send_Start_Cmd(Pw_EquipmentNo3);
			Send_Start_Cmd(Pw_EquipmentNo4);
			Send_Start_Cmd(Pw_EquipmentNo5);
			Send_Start_Cmd(Pw_EquipmentNo6);

			Pw_Current_Run_Time = arr_p1[D_RUN_TIME]; //����ʱ��

			Pw_EquipStatus = Pw_EquipStatus & 0xFFDF; //�����λ������ͣ��״̬

			Driver1_Pos_Start_Sort = 3;
			Driver2_Pos_Start_Sort = 3;
			Driver3_Pos_Start_Sort = 3;
			Driver4_Pos_Start_Sort = 3;
			Driver5_Pos_Start_Sort = 3;
			Driver6_Pos_Start_Sort = 3;

			//ָ����һ��Ҫִ�е�ָ��
			if ((arr_p1[POS_CMD_SIZE + 1]) != 0) //��һ��Ҫִ�е�ָ��
				arr_p1 += POS_CMD_SIZE;
			else
			{
				arr_p1 = &w_ParLst_Pos_CMD;
				Pr_RUN_Count++; //����Ȧ��+1

				temp_count = (Pw_Total_RUN_Count_HW << 16) + Pw_Total_RUN_Count;
				temp_count++; //�ۼ�����Ȧ��+1
				Pw_Total_RUN_Count = temp_count & 0x0000FFFF;
				Pw_Total_RUN_Count_HW = (temp_count & 0xFFFF0000) >> 16;
			}

			Pr_Send_Data_F = 0; //���ͱ�־��0�����Է���λ��
		}
	}
}

/********************************************
//��Զ�λ���� 
//num -1073741824~1073741824
//speed: 0~6000rpm
//acc_dec_time: 0~65535 �Ӽ���ʱ��
//����ֵ=1����ʾ���������ɣ�����ֵ=0����ʾ������
//----------λ��1---------λ��2
//1#��2#----100C----------1011
//3#~6#-----1307----------130C
*********************************************/
u8 Locate_Rle_1(u8 driver_no, s32 num, u16 speed, u16 acc_dec_time, s32 Cmd_No) //��Զ�λ����
{
	s16 Queue_Rear;
	s32 *tmp_addr_pulse;
	s32 *tmp_addr_speed;
	s32 *tmp_addr_NeverRun;
	u16 *tmp_addr_ComErr;
	s32 *temp_arr_p;
	u8 *arr_Driver_Write_Sort;
	s32 tmp_pulse_num;
	u16 tmp_run_speed;
	u16 tmp_acc_time;

	if (driver_no == Pw_EquipmentNo1)
	{
		Com_Write_p = &Com1_Driver1_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver1_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo2)
	{
		Com_Write_p = &Com1_Driver2_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver2_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo3)
	{
		Com_Write_p = &Com2_Driver3_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver3_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo4)
	{
		Com_Write_p = &Com2_Driver4_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver4_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo5)
	{
		Com_Write_p = &Com3_Driver5_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver5_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo6)
	{
		Com_Write_p = &Com3_Driver6_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver6_Write_Sort;
	}

	if (Cmd_No == 0)
	{
		Queue_Rear = Move_Cmd_Queue(driver_no);
		if (Queue_Rear == 0xFF)
			return 0;

		//��ʼ���
		//��1����дλ��ָ��
		Com_Write_p[COM_CMD_SIZE * Queue_Rear] = 12;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 1] = 15;		//ָ���15
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 2] = driver_no; //�����ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 3] = 0x10;		//д����Ĵ������ܺ�16
		if (driver_no == 1 || driver_no == 2)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x10; //US200�ŷ�����P10.12��ַ��ʼ
			if (*arr_Driver_Write_Sort == 0)
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //��1��λ�ã�P10.12
			else
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x11; //��2��λ�ã�P10.17
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x13; //RA1�ŷ�
			if (*arr_Driver_Write_Sort == 0)
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x07; //��1��λ��
			else
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //��2��λ��
		}
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 6] = 0x00; //д4����
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 7] = 0x04;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 8] = 0x08;							//д8���ֽ�
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 9] = (num & 0x0000FF00) >> 8;		//λ�øߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 10] = (num & 0x000000FF);			//λ�õ͵�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 11] = (num & 0xFF000000) >> 24;		//λ�øߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 12] = (num & 0x00FF0000) >> 16;		//λ�õ͵�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 13] = (speed & 0xFF00) >> 8;		//�ٶȸߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 14] = (speed & 0xFF);				//�ٶȵ͵�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 15] = (acc_dec_time & 0xFF00) >> 8; //�Ӽ��ٸߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 16] = (acc_dec_time & 0xFF);		//�Ӽ��ٵ͵�ַ

		//��ǰ��������
		//			tmp_addr=&w_ParLst_Drive[51];								//ָ��1#�ŷ�����趨ֵ����
		tmp_addr_pulse = &Pw_Driver1_SetValue;
		tmp_addr_pulse[(driver_no - 1) * 2] = num & 0x0000FFFF;
		tmp_addr_pulse[(driver_no - 1) * 2 + 1] = (num & 0xFFFF0000) >> 16;

		//��ǰ�����ٶ�
		//			tmp_addr=&w_ParLst_Drive[240];								//ָ��1#�ŷ���������ٶ�
		tmp_addr_speed = &Pw_Driver1_AutoSpeed;
		tmp_addr_speed[driver_no - 1] = speed;

		tmp_addr_NeverRun = &Pr_Driver1_NeverRun;						 //ָ��1#�ŷ������û���б�־
		tmp_addr_ComErr = &Pr_Driver1_ComErr;							 //ָ��1#�ŷ����ͨѶ���ϱ�־
		if (tmp_addr_ComErr[driver_no - 1] == 0 && num > 0 && speed > 0) //���ͨѶ��������������ֵ���ٶȶ�����0
			tmp_addr_NeverRun[driver_no - 1] = 0;						 //������̨����Ĵ���û�����б�־

		//��2��ָ�λ�ò�ѯ
		if (driver_no == 1 || driver_no == 2) //US200�ŷ�
		{
			Send_Driver_CMD(driver_no, 03, 0x100C, 0x0A);
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Send_Driver_CMD(driver_no, 03, 0x1307, 0x0A);
		}

		//��3��ָ�ֹͣ
		if (driver_no == 1 || driver_no == 2)
			Send_Driver_CMD(driver_no, 06, 0x3100, 1); //US200�ŷ���дP31.00=1
		else if (driver_no >= 3 && driver_no <= 6)
			Send_Driver_CMD(driver_no, 06, 0x8910, 1); //RA1�ŷ���дP8910=1

		//��4��ָ�ֹͣ״̬��ѯ
		if (driver_no == 1 || driver_no == 2)
			Send_Driver_CMD(driver_no, 03, 0x3100, 1); //US200�ŷ�����P31.00����״̬
		else if (driver_no >= 3 && driver_no <= 6)
			Send_Driver_CMD(driver_no, 03, 0x8910, 1); //RA1�ŷ�����ѯP8910����״̬

		//��5��ָ���λ��1��ֱ��ʹ�ܣ���λ��2������˶�ָ���л�
		if (driver_no == 1 || driver_no == 2) //US200�ŷ�
		{
			if (*arr_Driver_Write_Sort == 0)
				Send_Driver_CMD(driver_no, 06, 0x3100, 0x03);
			else
				Send_Driver_CMD(driver_no, 06, 0x3100, 0x05);
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			if (*arr_Driver_Write_Sort == 0)
				Send_Driver_CMD(driver_no, 06, 0x8910, 0x03);
			else
				Send_Driver_CMD(driver_no, 06, 0x8910, 0x05);
		}

		//��6��ָ���λ��2��ʹ�ܶ��λ��+ʹ���ŷ�
		if (driver_no == 1 || driver_no == 2) //US200�ŷ�
		{
			if (*arr_Driver_Write_Sort == 1)
				Send_Driver_CMD(driver_no, 06, 0x3100, 0x07);
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			if (*arr_Driver_Write_Sort == 1)
				Send_Driver_CMD(driver_no, 06, 0x8910, 0x07);
		}

		//��7��ָ�����״̬��ѯ
		if (driver_no == 1 || driver_no == 2)
			Send_Driver_CMD(driver_no, 03, 0x3100, 1); //US200�ŷ�����P31.00����״̬
		else if (driver_no >= 3 && driver_no <= 6)
			Send_Driver_CMD(driver_no, 03, 0x8910, 1); //RA1�ŷ�����ѯP8910����״̬

		//����дλ��0��λ��1
		(*arr_Driver_Write_Sort)++;
		if (*arr_Driver_Write_Sort > 1)
			*arr_Driver_Write_Sort = 0;

		return 1;
	}
	else if (Cmd_No >= 1 && Cmd_No <= COM_CMD_NUM - 1) //Ĭ��29��ָ��
	{
		//��ǰ��������
		//			tmp_addr=&w_ParLst_Drive[51];						//ָ��1#�ŷ�����趨ֵ����
		tmp_addr_pulse = &Pw_Driver1_SetValue;
		temp_arr_p = &w_ParLst_Pos_CMD;
		tmp_addr_pulse[(driver_no - 1) * 2] = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 12];
		tmp_addr_pulse[(driver_no - 1) * 2 + 1] = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 13];
		tmp_pulse_num = (tmp_addr_pulse[(driver_no - 1) * 2 + 1] << 16) + tmp_addr_pulse[(driver_no - 1) * 2];

		//��ǰ�����ٶ�
		//			tmp_addr=&w_ParLst_Drive[240];						//ָ��1#�ŷ���������ٶ�
		tmp_addr_speed = &Pw_Driver1_AutoSpeed;
		temp_arr_p = &w_ParLst_Pos_CMD;
		tmp_addr_speed[driver_no - 1] = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 14];
		tmp_run_speed = tmp_addr_speed[driver_no - 1];

		//�Ӽ���
		tmp_acc_time = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 15];

		tmp_addr_NeverRun = &Pr_Driver1_NeverRun;										   //ָ��1#�ŷ������û���б�־
		tmp_addr_ComErr = &Pr_Driver1_ComErr;											   //ָ��1#�ŷ����ͨѶ���ϱ�־
		if (tmp_addr_ComErr[driver_no - 1] == 0 && tmp_pulse_num > 0 && tmp_run_speed > 0) //���ͨѶ��������������ֵ���ٶȶ�����0
			tmp_addr_NeverRun[driver_no - 1] = 0;										   //������̨����Ĵ���û�����б�־

		Queue_Rear = Move_Cmd_Queue(driver_no);
		if (Queue_Rear == 0xFF)
			return 0;

		//��ʼ���
		//��1��ָ����λ��
		Com_Write_p[COM_CMD_SIZE * Queue_Rear] = 12;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 1] = 15;		//ָ���15
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 2] = driver_no; //�����ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 3] = 0x10;		//д����Ĵ������ܺ�16
		if (driver_no == 1 || driver_no == 2)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x10; //US200�ŷ�����P10.12��ַ��ʼ
			if (*arr_Driver_Write_Sort == 0)
			{
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //��1��λ�ã�P10.12
			}
			else
			{
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x11; //��2��λ�ã�P10.17
			}
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x13; //RA1�ŷ�
			if (*arr_Driver_Write_Sort == 0)
			{
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x07; //��1��λ��
			}
			else
			{
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //��2��λ��
			}
		}

		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 6] = 0x00; //д4����
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 7] = 0x04;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 8] = 0x08;								  //д8���ֽ�
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 9] = (tmp_pulse_num & 0x0000FF00) >> 8;	  //λ�øߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 10] = (tmp_pulse_num & 0x000000FF);		  //λ�õ͵�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 11] = (tmp_pulse_num & 0xFF000000) >> 24; //λ�øߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 12] = (tmp_pulse_num & 0x00FF0000) >> 16; //λ�õ͵�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 13] = (tmp_run_speed & 0xFF00) >> 8;	  //�ٶȸߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 14] = (tmp_run_speed & 0xFF);			  //�ٶȵ͵�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 15] = (tmp_acc_time & 0xFF00) >> 8;		  //�Ӽ��ٸߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 16] = (tmp_acc_time & 0xFF);			  //�Ӽ��ٵ͵�ַ

		//��2��ָ�λ�ò�ѯ
		if (driver_no == 1 || driver_no == 2) //US200�ŷ�
		{
			Send_Driver_CMD(driver_no, 03, 0x100C, 0x0A);
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Send_Driver_CMD(driver_no, 03, 0x1307, 0x0A);
		}

		//��3��ָ�ֹͣ
		if (driver_no == 1 || driver_no == 2)
			Send_Driver_CMD(driver_no, 06, 0x3100, 1); //US200�ŷ���дP31.00=1
		else if (driver_no >= 3 && driver_no <= 6)
			Send_Driver_CMD(driver_no, 06, 0x8910, 1); //RA1�ŷ���дP8910=1

		//��4��ָ�ֹͣ״̬��ѯ
		if (driver_no == 1 || driver_no == 2)
			Send_Driver_CMD(driver_no, 03, 0x3100, 1); //US200�ŷ�����P31.00����״̬
		else if (driver_no >= 3 && driver_no <= 6)
			Send_Driver_CMD(driver_no, 03, 0x8910, 1); //RA1�ŷ�����ѯP8910����״̬

		//��5��ָ���λ��1��ֱ��ʹ�ܣ���λ��2������˶�ָ���л�
		if (driver_no == 1 || driver_no == 2) //US200�ŷ�
		{
			if (*arr_Driver_Write_Sort == 0)
				Send_Driver_CMD(driver_no, 06, 0x3100, 0x03);
			else
				Send_Driver_CMD(driver_no, 06, 0x3100, 0x05);
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			if (*arr_Driver_Write_Sort == 0)
				Send_Driver_CMD(driver_no, 06, 0x8910, 0x03);
			else
				Send_Driver_CMD(driver_no, 06, 0x8910, 0x05);
		}

		//��6��ָ���λ��2��ʹ�ܶ��λ��+ʹ���ŷ�
		if (driver_no == 1 || driver_no == 2) //US200�ŷ�
		{
			if (*arr_Driver_Write_Sort == 1)
				Send_Driver_CMD(driver_no, 06, 0x3100, 0x07);
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			if (*arr_Driver_Write_Sort == 1)
				Send_Driver_CMD(driver_no, 06, 0x8910, 0x07);
		}

		//��7��ָ�����״̬��ѯ
		if (driver_no == 1 || driver_no == 2)
			Send_Driver_CMD(driver_no, 03, 0x3100, 1); //US200�ŷ�����P31.00����״̬
		else if (driver_no >= 3 && driver_no <= 6)
			Send_Driver_CMD(driver_no, 03, 0x8910, 1); //RA1�ŷ�����ѯP8910����״̬

		//����дλ��0��λ��1
		(*arr_Driver_Write_Sort)++;
		if (*arr_Driver_Write_Sort > 1)
			*arr_Driver_Write_Sort = 0;

		return 1;
	}

	return 0;
}

/********************************************
//ֻ����λ������ 
//num -1073741824~1073741824
//speed: 0~6000rpm
//acc_dec_time: 0~65535 �Ӽ���ʱ��
//����ֵ=1����ʾ���������ɣ�����ֵ=0����ʾ������
//----------λ��1---------λ��2
//1#��2#----100C----------1011
//3#~6#-----1307----------130C
*********************************************/
u8 Fill_Pos_Data(u8 driver_no, s32 num, u16 speed, u16 acc_dec_time, s32 Cmd_No)
{
	s16 Queue_Rear;
	s32 *tmp_addr_pulse;
	s32 *tmp_addr_speed;
	s32 *tmp_addr_NeverRun;
	u16 *tmp_addr_ComErr;
	s32 *temp_arr_p;
	u8 *arr_Driver_Write_Sort;
	s32 tmp_pulse_num;
	u16 tmp_run_speed;
	u16 tmp_acc_time;

	if (driver_no == Pw_EquipmentNo1)
	{
		Com_Write_p = &Com1_Driver1_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver1_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo2)
	{
		Com_Write_p = &Com1_Driver2_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver2_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo3)
	{
		Com_Write_p = &Com2_Driver3_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver3_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo4)
	{
		Com_Write_p = &Com2_Driver4_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver4_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo5)
	{
		Com_Write_p = &Com3_Driver5_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver5_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo6)
	{
		Com_Write_p = &Com3_Driver6_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver6_Write_Sort;
	}

	if (Cmd_No == 0)
	{
		Queue_Rear = Move_Cmd_Queue(driver_no);
		if (Queue_Rear == 0xFF)
			return 0;

		//��ʼ���
		//��1����дλ��ָ��
		Com_Write_p[COM_CMD_SIZE * Queue_Rear] = 12;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 1] = 15;		//ָ���15
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 2] = driver_no; //�����ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 3] = 0x10;		//д����Ĵ������ܺ�16

		if (driver_no == 1 || driver_no == 2)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x10; //US200�ŷ�����P10.12��ַ��ʼ
			if (*arr_Driver_Write_Sort == 0)
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //��1��λ�ã�P10.12
			else
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x11; //��2��λ�ã�P10.17
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x13; //RA1�ŷ�
			if (*arr_Driver_Write_Sort == 0)
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x07; //��1��λ��
			else
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //��2��λ��
		}

		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 6] = 0x00; //д4����
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 7] = 0x04;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 8] = 0x08;							//д8���ֽ�
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 9] = (num & 0x0000FF00) >> 8;		//λ�øߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 10] = (num & 0x000000FF);			//λ�õ͵�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 11] = (num & 0xFF000000) >> 24;		//λ�øߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 12] = (num & 0x00FF0000) >> 16;		//λ�õ͵�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 13] = (speed & 0xFF00) >> 8;		//�ٶȸߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 14] = (speed & 0xFF);				//�ٶȵ͵�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 15] = (acc_dec_time & 0xFF00) >> 8; //�Ӽ��ٸߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 16] = (acc_dec_time & 0xFF);		//�Ӽ��ٵ͵�ַ

		//��ǰ��������
		//			tmp_addr=&w_ParLst_Drive[51];								//ָ��1#�ŷ�����趨ֵ����
		tmp_addr_pulse = &Pw_Driver1_SetValue;
		tmp_addr_pulse[(driver_no - 1) * 2] = num & 0x0000FFFF;
		tmp_addr_pulse[(driver_no - 1) * 2 + 1] = (num & 0xFFFF0000) >> 16;

		//��ǰ�����ٶ�
		//			tmp_addr=&w_ParLst_Drive[240];								//ָ��1#�ŷ���������ٶ�
		tmp_addr_speed = &Pw_Driver1_AutoSpeed;
		tmp_addr_speed[driver_no - 1] = speed;

		tmp_addr_NeverRun = &Pr_Driver1_NeverRun;						 //ָ��1#�ŷ������û���б�־
		tmp_addr_ComErr = &Pr_Driver1_ComErr;							 //ָ��1#�ŷ����ͨѶ���ϱ�־
		if (tmp_addr_ComErr[driver_no - 1] == 0 && num > 0 && speed > 0) //���ͨѶ��������������ֵ���ٶȶ�����0
			tmp_addr_NeverRun[driver_no - 1] = 0;						 //������̨����Ĵ���û�����б�־

		//��2��ָ�λ�ò�ѯ
		if (driver_no == 1 || driver_no == 2) //US200�ŷ�
		{
			Send_Driver_CMD(driver_no, 03, 0x100C, 0x0A);
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Send_Driver_CMD(driver_no, 03, 0x1307, 0x0A);
		}

		return 1;
	}
	else if (Cmd_No >= 1 && Cmd_No <= COM_CMD_NUM - 1) //Ĭ��29��ָ��
	{
		//��ǰ��������
		//			tmp_addr=&w_ParLst_Drive[51];						//ָ��1#�ŷ�����趨ֵ����
		tmp_addr_pulse = &Pw_Driver1_SetValue;
		temp_arr_p = &w_ParLst_Pos_CMD;
		tmp_addr_pulse[(driver_no - 1) * 2] = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 12];
		tmp_addr_pulse[(driver_no - 1) * 2 + 1] = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 13];
		tmp_pulse_num = (tmp_addr_pulse[(driver_no - 1) * 2 + 1] << 16) + tmp_addr_pulse[(driver_no - 1) * 2];

		//��ǰ�����ٶ�
		//			tmp_addr=&w_ParLst_Drive[240];						//ָ��1#�ŷ���������ٶ�
		tmp_addr_speed = &Pw_Driver1_AutoSpeed;
		temp_arr_p = &w_ParLst_Pos_CMD;
		tmp_addr_speed[driver_no - 1] = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 14];
		tmp_run_speed = tmp_addr_speed[driver_no - 1];

		//�Ӽ���
		tmp_acc_time = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 15];

		tmp_addr_NeverRun = &Pr_Driver1_NeverRun;										   //ָ��1#�ŷ������û���б�־
		tmp_addr_ComErr = &Pr_Driver1_ComErr;											   //ָ��1#�ŷ����ͨѶ���ϱ�־
		if (tmp_addr_ComErr[driver_no - 1] == 0 && tmp_pulse_num > 0 && tmp_run_speed > 0) //���ͨѶ��������������ֵ���ٶȶ�����0
			tmp_addr_NeverRun[driver_no - 1] = 0;										   //������̨����Ĵ���û�����б�־

		Queue_Rear = Move_Cmd_Queue(driver_no);
		if (Queue_Rear == 0xFF)
			return 0;

		//��ʼ���
		Com_Write_p[COM_CMD_SIZE * Queue_Rear] = 12;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 1] = 15;		//ָ���15
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 2] = driver_no; //�����ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 3] = 0x10;		//д����Ĵ������ܺ�16
		if (driver_no == 1 || driver_no == 2)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x10; //US200�ŷ�����P10.12��ַ��ʼ
			if (*arr_Driver_Write_Sort == 0)
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //��1��λ�ã�P10.12
			else
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x11; //��2��λ�ã�P10.17
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x13; //RA1�ŷ�
			if (*arr_Driver_Write_Sort == 0)
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x07; //��1��λ��
			else
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //��2��λ��
		}

		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 6] = 0x00; //д4����
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 7] = 0x04;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 8] = 0x08;								  //д8���ֽ�
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 9] = (tmp_pulse_num & 0x0000FF00) >> 8;	  //λ�ã����֣����ֽڵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 10] = (tmp_pulse_num & 0x000000FF);		  //λ�ã����֣����ֽڵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 11] = (tmp_pulse_num & 0xFF000000) >> 24; //λ�ã����֣����ֽڵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 12] = (tmp_pulse_num & 0x00FF0000) >> 16; //λ�ã����֣����ֽڵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 13] = (tmp_run_speed & 0xFF00) >> 8;	  //�ٶȸߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 14] = (tmp_run_speed & 0xFF);			  //�ٶȵ͵�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 15] = (tmp_acc_time & 0xFF00) >> 8;		  //�Ӽ��ٸߵ�ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 16] = (tmp_acc_time & 0xFF);			  //�Ӽ��ٵ͵�ַ

		//��2��ָ�λ�ò�ѯ
		if (driver_no == 1 || driver_no == 2) //US200�ŷ�
		{
			Send_Driver_CMD(driver_no, 03, 0x100C, 0x0A);
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Send_Driver_CMD(driver_no, 03, 0x1307, 0x0A);
		}

		return 1;
	}

	return 0;
}

/********************************************
//������������� 
*********************************************/
u8 Send_Start_Cmd(u8 driver_no)
{
	u8 *arr_Driver_Write_Sort;

	if (driver_no == Pw_EquipmentNo1)
	{
		Com_Write_p = &Com1_Driver1_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver1_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo2)
	{
		Com_Write_p = &Com1_Driver2_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver2_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo3)
	{
		Com_Write_p = &Com2_Driver3_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver3_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo4)
	{
		Com_Write_p = &Com2_Driver4_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver4_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo5)
	{
		Com_Write_p = &Com3_Driver5_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver5_Write_Sort;
	}
	else if (driver_no == Pw_EquipmentNo6)
	{
		Com_Write_p = &Com3_Driver6_Write_BUFF[0];
		arr_Driver_Write_Sort = &Driver6_Write_Sort;
	}

	//��1��ָ�ֹͣ
	if (driver_no == 1 || driver_no == 2)
		Send_Driver_CMD(driver_no, 06, 0x3100, 1); //US200�ŷ���дP31.00=1
	else if (driver_no >= 3 && driver_no <= 6)
		Send_Driver_CMD(driver_no, 06, 0x8910, 1); //RA1�ŷ���дP8910=1

	//��2��ָ�ֹͣ״̬��ѯ
	if (driver_no == 1 || driver_no == 2)
		Send_Driver_CMD(driver_no, 03, 0x3100, 1); //US200�ŷ�����P31.00����״̬
	else if (driver_no >= 3 && driver_no <= 6)
		Send_Driver_CMD(driver_no, 03, 0x8910, 1); //RA1�ŷ�����ѯP8910����״̬

	//��3��ָ���λ��1��ֱ��ʹ�ܣ���λ��2������˶�ָ���л�
	if (driver_no == 1 || driver_no == 2) //US200�ŷ�
	{
		if (*arr_Driver_Write_Sort == 0)
			Send_Driver_CMD(driver_no, 06, 0x3100, 0x03);
		else
			Send_Driver_CMD(driver_no, 06, 0x3100, 0x05);
	}
	else if (driver_no >= 3 && driver_no <= 6)
	{
		if (*arr_Driver_Write_Sort == 0)
			Send_Driver_CMD(driver_no, 06, 0x8910, 0x03);
		else
			Send_Driver_CMD(driver_no, 06, 0x8910, 0x05);
	}

	//��4��ָ���λ��2��ʹ�ܶ��λ��+ʹ���ŷ�
	if (driver_no == 1 || driver_no == 2) //US200�ŷ�
	{
		if (*arr_Driver_Write_Sort == 1)
			Send_Driver_CMD(driver_no, 06, 0x3100, 0x07);
	}
	else if (driver_no >= 3 && driver_no <= 6)
	{
		if (*arr_Driver_Write_Sort == 1)
			Send_Driver_CMD(driver_no, 06, 0x8910, 0x07);
	}

	//��5��ָ�����״̬��ѯ
	if (driver_no == 1 || driver_no == 2)
		Send_Driver_CMD(driver_no, 03, 0x3100, 1); //US200�ŷ�����P31.00����״̬
	else if (driver_no >= 3 && driver_no <= 6)
		Send_Driver_CMD(driver_no, 03, 0x8910, 1); //RA1�ŷ�����ѯP8910����״̬

	//����дλ��0��λ��1
	(*arr_Driver_Write_Sort)++;
	if (*arr_Driver_Write_Sort > 1)
		*arr_Driver_Write_Sort = 0;

	return 1;
}

//�������+1
//���ض�βָ���ʾ�ɹ�������0xFF��ʾʧ��
s16 Move_Cmd_Queue(u8 driver_no)
{
	if (driver_no == Pw_EquipmentNo1)
	{
		if (Com1_Driver1_Queue_isFull())
			return 0xFF;

		if (Com1_Driver1_Queue_isEmpty())
		{
			Com1_Driver1_Queue_Front = 0;
		}
		Com1_Driver1_Queue_Rear = (Com1_Driver1_Queue_Rear + 1) % COM_CMD_NUM;
		return Com1_Driver1_Queue_Rear;
	}
	else if (driver_no == Pw_EquipmentNo2)
	{
		if (Com1_Driver2_Queue_isFull())
			return 0xFF;

		if (Com1_Driver2_Queue_isEmpty())
		{
			Com1_Driver2_Queue_Front = 0;
		}
		Com1_Driver2_Queue_Rear = (Com1_Driver2_Queue_Rear + 1) % COM_CMD_NUM;
		return Com1_Driver2_Queue_Rear;
	}
	else if (driver_no == Pw_EquipmentNo3)
	{
		if (Com2_Driver3_Queue_isFull())
			return 0xFF;

		if (Com2_Driver3_Queue_isEmpty())
		{
			Com2_Driver3_Queue_Front = 0;
		}
		Com2_Driver3_Queue_Rear = (Com2_Driver3_Queue_Rear + 1) % COM_CMD_NUM;
		return Com2_Driver3_Queue_Rear;
	}
	else if (driver_no == Pw_EquipmentNo4)
	{
		if (Com2_Driver4_Queue_isFull())
			return 0xFF;

		if (Com2_Driver4_Queue_isEmpty())
		{
			Com2_Driver4_Queue_Front = 0;
		}
		Com2_Driver4_Queue_Rear = (Com2_Driver4_Queue_Rear + 1) % COM_CMD_NUM;
		return Com2_Driver4_Queue_Rear;
	}
	else if (driver_no == Pw_EquipmentNo5)
	{
		if (Com3_Driver5_Queue_isFull())
			return 0xFF;

		if (Com3_Driver5_Queue_isEmpty())
		{
			Com3_Driver5_Queue_Front = 0;
		}
		Com3_Driver5_Queue_Rear = (Com3_Driver5_Queue_Rear + 1) % COM_CMD_NUM;
		return Com3_Driver5_Queue_Rear;
	}
	else if (driver_no == Pw_EquipmentNo6)
	{
		if (Com3_Driver6_Queue_isFull())
			return 0xFF;

		if (Com3_Driver6_Queue_isEmpty())
		{
			Com3_Driver6_Queue_Front = 0;
		}
		Com3_Driver6_Queue_Rear = (Com3_Driver6_Queue_Rear + 1) % COM_CMD_NUM;
		return Com3_Driver6_Queue_Rear;
	}
	return 0xFF;
}

//�ж�COM1��������Ƿ�Ϊ��
u8 Com1_Driver1_Queue_isEmpty(void)
{
	return Com1_Driver1_Queue_Front == -1;
}

u8 Com1_Driver2_Queue_isEmpty(void)
{
	return Com1_Driver2_Queue_Front == -1;
}

//�ж�COM2��������Ƿ�Ϊ��
u8 Com2_Driver3_Queue_isEmpty(void)
{
	return Com2_Driver3_Queue_Front == -1;
}

u8 Com2_Driver4_Queue_isEmpty(void)
{
	return Com2_Driver4_Queue_Front == -1;
}

//�ж�COM3��������Ƿ�Ϊ��
u8 Com3_Driver5_Queue_isEmpty(void)
{
	return Com3_Driver5_Queue_Front == -1;
}

u8 Com3_Driver6_Queue_isEmpty(void)
{
	return Com3_Driver6_Queue_Front == -1;
}

//�ж�COM4��������Ƿ�Ϊ��
u8 Com4_Queue_isEmpty(void)
{
	return Com4_Queue_Front == -1;
}

//�ж�COM1��������Ƿ�Ϊ��
u8 Com1_Driver1_Queue_isFull(void)
{
	return (Com1_Driver1_Queue_Front == (Com1_Driver1_Queue_Rear + 1) % COM_CMD_NUM);
}

u8 Com1_Driver2_Queue_isFull(void)
{
	return (Com1_Driver2_Queue_Front == (Com1_Driver2_Queue_Rear + 1) % COM_CMD_NUM);
}

//�ж�COM2��������Ƿ�Ϊ��
u8 Com2_Driver3_Queue_isFull(void)
{
	return (Com2_Driver3_Queue_Front == (Com2_Driver3_Queue_Rear + 1) % COM_CMD_NUM);
}

u8 Com2_Driver4_Queue_isFull(void)
{
	return (Com2_Driver4_Queue_Front == (Com2_Driver4_Queue_Rear + 1) % COM_CMD_NUM);
}
//�ж�COM3��������Ƿ�Ϊ��
u8 Com3_Driver5_Queue_isFull(void)
{
	return (Com3_Driver5_Queue_Front == (Com3_Driver5_Queue_Rear + 1) % COM_CMD_NUM);
}

u8 Com3_Driver6_Queue_isFull(void)
{
	return (Com3_Driver6_Queue_Front == (Com3_Driver6_Queue_Rear + 1) % COM_CMD_NUM);
}

//�ж�COM4��������Ƿ�Ϊ��
u8 Com4_Queue_isFull(void)
{
	return (Com4_Queue_Front == (Com4_Queue_Rear + 1) % COM_CMD_NUM);
}

//���ͣ�������ӳ���
void Stop_Driver(u8 driver_no)
{
	s16 Queue_Rear;

	if (driver_no == Pw_EquipmentNo1)
	{
		Com_Write_p = &Com1_Driver1_Write_BUFF[0];
	}
	else if (driver_no == Pw_EquipmentNo2)
	{
		Com_Write_p = &Com1_Driver2_Write_BUFF[0];
	}
	else if (driver_no == Pw_EquipmentNo3)
	{
		Com_Write_p = &Com2_Driver3_Write_BUFF[0];
	}
	else if (driver_no == Pw_EquipmentNo4)
	{
		Com_Write_p = &Com2_Driver4_Write_BUFF[0];
	}
	else if (driver_no == Pw_EquipmentNo5)
	{
		Com_Write_p = &Com3_Driver5_Write_BUFF[0];
	}
	else if (driver_no == Pw_EquipmentNo6)
	{
		Com_Write_p = &Com3_Driver6_Write_BUFF[0];
	}

	Queue_Rear = Move_Cmd_Queue(driver_no);
	if (Queue_Rear != 0xFF)
	{
		Com_Write_p[COM_CMD_SIZE * Queue_Rear] = 12;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 1] = 6;			//ָ���6
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 2] = driver_no; //�����ַ
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 3] = 0x06;		//д�����Ĵ������ܺ�06

		if (driver_no == 1 || driver_no == 2)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x31; //US200�ŷ���дP31.00=1
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x00;
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x89; //RA1�ŷ���дP8910=1
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x10;
		}

		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 6] = 0x00; //д1����
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 7] = 0x01;
	}
}

//���Ƶ�ŷ�(��е��)��ͨ���ضϣ�����������ҵ
//���ܣ�1��ִ�е�ĳ��ָ��ʱ��������ʱ�ٿ���ŷ�
//2����ͨ��ŷ��󣬿����趨������ʱ��
//3������趨�ĳ���ʱ��̫������ִ�е���رյ�ŷ���ָ��ʱ�����Ϲر�
void Control_Hand(void)
{
	//�ж�״̬
	if (HAND_STATUS)
		F_Hand_Status = 0; //0��ŷ��ر�
	else
		F_Hand_Status = 1; //��ŷ���

	if (Pw_StepAutoMode == 0 && Pw_TouchRunStop == 0 && F_AskStop == 0 && F_Starting == 0 && F_Stoping == 0 && arrp_p1_Last[11] != 0 && F_Hand_Status == 0 && !F_HaveDriver_Cmd_Err)
	{
		if (F_Close_Hand == 0)
		{
			if (arrp_p1_Last[9] > 0 && T_DO_delay != SClk10Ms && DO_delay_F == 0)
			{
				if (T_DO_delay != SClk10Ms)
				{
					T_DO_delay = SClk10Ms; //
					C_DO_delayCount++;
					if (C_DO_delayCount > arrp_p1_Last[9]) //��ʱ��ϣ��ñ�־λ
					{
						C_DO_delayCount = 0;
						T_DO_delay = 1000;
						DO_delay_F = 1; //��ʱ������־λ
					}
				}
			}
			else if (((arrp_p1_Last[9]) == 0) || (((arrp_p1_Last[9]) > 0) && (DO_delay_F == 1))) //����ʱ������ʱ����
			{
				T_DO_delay = 1000;
				C_DO_delayCount = 0;
				OPEN_HAND;
				DO_delay_F = 0;
			}
		}
	}
	else if (arrp_p1_Last[11] == 0)
	{
		C_DO_delayCount = 0;
		T_DO_delay = 1000;
		CLOSE_HAND;
	}

	//DO����ʱ��
	if (arrp_p1_Last[11] != 0 && F_Hand_Status == 1)
	{
		if (T_DO_Open_delay != SClk10Ms)
		{
			T_DO_Open_delay = SClk10Ms; //
			C_DO_Open_delayCount++;
			if (C_DO_Open_delayCount > arrp_p1_Last[10]) //��ʱ��ϣ��ñ�־λ
			{
				C_DO_Open_delayCount = 0;
				T_DO_Open_delay = 1000;
				CLOSE_HAND;
				F_Close_Hand = 1;
			}
		}
	}
	else
	{
		C_DO_Open_delayCount = 0;
		T_DO_Open_delay = 1000;
	}
}

//ɲ������
u16 T_BRAKE;
u16 C_BRAKE;

void BRAKE_Control(void)
{
	if (Pr_BRAKE_Control == 0) //=1��ɲ����=0����ɲ��
	{
		if (T_BRAKE != SClk10Ms)
		{
			T_BRAKE = SClk10Ms;
			C_BRAKE++;
			if (C_BRAKE > 100) //����ʱ1s�󣬿�ɲ����������ת
			{
				STOP_BRAKE_SYSTEM;
			}
		}
	}
	else
		START_BRAKE_SYSTEM; //ɲ���ƶ�
}

//��������л�����
void Clear_Cmd_Queue(void)
{
	Com1_Driver1_Queue_Front = -1;
	Com1_Driver1_Queue_Rear = -1;

	Com1_Driver2_Queue_Front = -1;
	Com1_Driver2_Queue_Rear = -1;

	Com2_Driver3_Queue_Front = -1;
	Com2_Driver3_Queue_Rear = -1;

	Com2_Driver4_Queue_Front = -1;
	Com2_Driver4_Queue_Rear = -1;

	Com3_Driver5_Queue_Front = -1;
	Com3_Driver5_Queue_Rear = -1;

	Com3_Driver6_Queue_Front = -1;
	Com3_Driver6_Queue_Rear = -1;
}

//��¼��ǰλ��
void Record_Current_Pos(void)
{
	if (Pw_Define_Save_Pos >= 1 && Pw_Define_Save_Pos <= POS_NUM)
	{
		Pw_Read_CurrentPos = 63; //=63����ʾ��6���ŷ��ĵ�ǰλ�á�����󣬻ָ�Ϊ0
		Pw_Define_Save_Pos = 0;
	}
}

//����λ�ú��ٶ�
//40*30=120��˫��
//���(0)+�����(1)+λ�ú�(2)+�����ٶ�(3)+�Ӽ���ʱ��(4)+��ͣ(5)+ֹͣ(6)+����1(7)+����2(8)+ѭ��1(9)+ѭ��2(10)+���(11)
//+1#����_��(12)+1#����_��(13)+1#�ٶ�(14)+1#�Ӽ���(15)+2#����_��(16)+2#����_��(17)+2#�ٶ�(18)+2#�Ӽ���(19)
//+3#����_��(20)+3#����_��(21)+3#�ٶ�(22)+3#�Ӽ���(23)+4#����_��(24)+4#����_��(25)+4#�ٶ�(26)+4#�Ӽ���(27)
//+5#����_��(28)+5#����_��(29)+5#�ٶ�(30)+5#�Ӽ���(31)+6#����_��(32)+6#����_��(33)+6#�ٶ�(34)+6#�Ӽ���(35)
//����ֵ0Ϊ����ɹ���1Ϊ����ʧ��
u8 Cal_Pos_Speed(void)
{
	int i, k;
	s32 *temp_arr_Cmd1;
	s32 *temp_arr_Cmd2;
	s32 *temp_arr_Cmd_Last;
	s32 Cmd_No1, Pos_No1;
	s32 Cmd_No2, Pos_No2;
	s32 *arr_P1;

	temp_arr_Cmd1 = &w_ParLst_Pos_CMD;
	temp_arr_Cmd2 = &w_ParLst_Pos_CMD;
	if (temp_arr_Cmd1[2] != 1) //��1�е�λ�úű���Ϊ1
		temp_arr_Cmd1[2] = 1;

	//λ��1�����ݱ�����ԭ�㣬��ʼλ�ã�ǿ�Ƹ�ֵ
	arr_P1 = &w_ParLst_PosPar;
	arr_P1[1] = Pr_Drive1_MultiData_Init;
	arr_P1[2] = Pr_Drive1_singleData_Init; //��ʼ��λ�ã�Ĭ��ͬʱ���浽λ��1
	arr_P1[3] = Pr_Drive1_singleData_Init_HW;

	arr_P1[4] = Pr_Drive2_MultiData_Init;
	arr_P1[5] = Pr_Drive2_singleData_Init;
	arr_P1[6] = Pr_Drive2_singleData_Init_HW;

	arr_P1[7] = Pr_Drive3_MultiData_Init;
	arr_P1[8] = Pr_Drive3_singleData_Init;
	arr_P1[9] = Pr_Drive3_singleData_Init_HW;

	arr_P1[10] = Pr_Drive4_MultiData_Init;
	arr_P1[11] = Pr_Drive4_singleData_Init;
	arr_P1[12] = Pr_Drive4_singleData_Init_HW;

	arr_P1[13] = Pr_Drive5_MultiData_Init;
	arr_P1[14] = Pr_Drive5_singleData_Init;
	arr_P1[15] = Pr_Drive5_singleData_Init_HW;

	arr_P1[16] = Pr_Drive6_MultiData_Init;
	arr_P1[17] = Pr_Drive6_singleData_Init;
	arr_P1[18] = Pr_Drive6_singleData_Init_HW;

	//����Ĭ��ֵ
	for (k = 0; k < POS_CMD_NUM; k++)
	{
		for (i = 12; i <= 35; i++)
		{
			temp_arr_Cmd1[k * POS_CMD_SIZE + i] = 0; //������λ�õ����塢�ٶȵ�������0
		}
	}
	//����Ĭ��ֵ
	for (k = 0; k < POS_CMD_NUM; k++)
	{
		for (i = 14; i <= 35; i = i + 4)
		{
			temp_arr_Cmd1[k * POS_CMD_SIZE + i] = 200;		//��Ϊ�ٶȲ���д��0������Ĭ��д200
			temp_arr_Cmd1[k * POS_CMD_SIZE + i + 1] = 1000; //�Ӽ���Ĭ��д1000
		}
	}

	//����
	while (temp_arr_Cmd2[1] != 0)
	{
		temp_arr_Cmd1 = temp_arr_Cmd2;
		temp_arr_Cmd_Last = temp_arr_Cmd2;
		Cmd_No1 = temp_arr_Cmd1[1];
		Pos_No1 = temp_arr_Cmd1[2];
		if (Cmd_No1 >= 1 && Cmd_No1 <= COM_CMD_NUM - 1 && Pos_No1 >= 1 && Pos_No1 <= POS_NUM)
		{

			temp_arr_Cmd2 += POS_CMD_SIZE;
			Cmd_No2 = temp_arr_Cmd2[1];
			Pos_No2 = temp_arr_Cmd2[2];
			if (Cmd_No2 >= 1 && Cmd_No2 <= COM_CMD_NUM - 1 && Pos_No2 >= 1 && Pos_No2 <= POS_NUM)
			{
				Cal_One_Pos_Speed(Cmd_No1, Cmd_No2);
			}
		}
		else
		{
			temp_arr_Cmd_Last = temp_arr_Cmd2;
			temp_arr_Cmd2 += POS_CMD_SIZE;
		}
	}

	//���ͷ��β��λ�ú���ȣ����ж�������ŷ��Ƿ�==0
	temp_arr_Cmd1 = &w_ParLst_Pos_CMD;
	if (temp_arr_Cmd_Last[2] == temp_arr_Cmd1[2])
	{
		//������Ϊ����������ܲ��������
		Cal_Sum_Err(1);
		Cal_Sum_Err(2);
		Cal_Sum_Err(3);
		Cal_Sum_Err(4);
		Cal_Sum_Err(5);
		Cal_Sum_Err(6);
	}

	return 0;
}

//��������������������
void Cal_Sum_Err(u8 Driver_No)
{
	s32 *temp_arr_Cmd_Last;
	s32 *temp_arr_Cmd1;
	s32 *temp_arr_Cmd2;
	s32 temp1, temp2;
	s32 Sum = 0;
	s32 abs_Sum;
	s32 temp_pulse;

	temp_arr_Cmd1 = &w_ParLst_Pos_CMD;
	temp_arr_Cmd2 = &w_ParLst_Pos_CMD;
	while (temp_arr_Cmd2[1] != 0)
	{
		temp1 = temp_arr_Cmd2[(Driver_No - 1) * 4 + 12];
		temp2 = temp_arr_Cmd2[(Driver_No - 1) * 4 + 13];
		Sum += ((temp2 << 16) + temp1);

		temp_arr_Cmd_Last = temp_arr_Cmd2;
		temp_arr_Cmd2 += POS_CMD_SIZE;
	}

	//���ͷ��β��λ�ú���ȣ����ж�������ŷ��Ƿ�==0
	if (temp_arr_Cmd_Last[2] == temp_arr_Cmd1[2])
	{
		abs_Sum = abs(Sum);
		if (abs_Sum >= 1 && abs_Sum <= 3)
		{
			temp1 = temp_arr_Cmd_Last[(Driver_No - 1) * 4 + 12];
			temp2 = temp_arr_Cmd_Last[(Driver_No - 1) * 4 + 13];
			temp_pulse = (temp2 << 16) + temp1;
			temp_pulse -= Sum;

			temp_arr_Cmd_Last[(Driver_No - 1) * 4 + 12] = temp_pulse & 0x0000FFFF;
			temp_arr_Cmd_Last[(Driver_No - 1) * 4 + 13] = (temp_pulse & 0xFFFF0000) >> 16;
		}
	}
}

void sort(s32 *a, int l) //aΪ�����ַ��lΪ���鳤�ȡ�
{
	int i, j;
	s32 v;
	//��������
	for (i = 0; i < l - 1; i++)
		for (j = i + 1; j < l; j++)
		{
			if (a[i] > a[j]) //��ǰ��ıȺ���Ĵ��򽻻���
			{
				v = a[i];
				a[i] = a[j];
				a[j] = v;
			}
		}
}

void sort_f(float *a, int l) //aΪ�����ַ��lΪ���鳤�ȡ�
{
	int i, j;
	float v;
	//��������
	for (i = 0; i < l - 1; i++)
		for (j = i + 1; j < l; j++)
		{
			if (a[i] > a[j]) //��ǰ��ıȺ���Ĵ��򽻻���
			{
				v = a[i];
				a[i] = a[j];
				a[j] = v;
			}
		}
}

//����һ�����������ٶ�
/*
˼·��2����������е������������
2���ȸ���ÿ̨������趨�ٶȼ����ÿ̨���������ʱ�䣻
3�����������ʱ��Ϊ��׼��������������������ٶȣ������������һ������ȵ���̨�����ʹ���е��������ʱ�������ȣ�
4����������������ٶȣ��õ�ÿ̨�����ʵ������ʱ��
5����ʵ������ʱ�����򣬵õ��ʱ�䣬��Ϊ����ָ��ĵ������ʱ��
*/
void Cal_One_Pos_Speed(s32 Cmd_No1, s32 Cmd_No2)
{
	s32 *temp_arr_Cmd1;
	s32 *temp_arr_Cmd2;
	s32 *arr_P1;
	s32 Pos_No1, Pos_No2;

	float num1, tmp1_f;
	s32 tmp_num1;
	//	s16 temp_MUTI1;
	float temp_SINGLE1;

	float num2, tmp2_f;
	s32 tmp_num2;
	//	s16 temp_MUTI2;
	float temp_SINGLE2;

	float num3, tmp3_f;
	s32 tmp_num3;
	s16 temp_MUTI3;
	float temp_SINGLE3;

	float num4, tmp4_f;
	s32 tmp_num4;
	s16 temp_MUTI4;
	float temp_SINGLE4;

	float num5, tmp5_f;
	s32 tmp_num5;
	s16 temp_MUTI5;
	float temp_SINGLE5;

	float num6, tmp6_f;
	s32 tmp_num6;
	s16 temp_MUTI6;
	float temp_SINGLE6;

	//	s32 pulse_num[6];
	u32 speed1, speed2, speed3, speed4, speed5, speed6;
	float f_speed1, f_speed2, f_speed3, f_speed4, f_speed5, f_speed6;
	u32 Run_Speed_Ratio;
	float Run_time; //����ָ�������ʱ��
	float Run_time1, Run_time2, Run_time3, Run_time4, Run_time5, Run_time6;
	float tmp_time1, tmp_time2, tmp_time3, tmp_time4, tmp_time5, tmp_time6;
	float time[6];

	temp_arr_Cmd1 = &w_ParLst_Pos_CMD;
	temp_arr_Cmd2 = &w_ParLst_Pos_CMD;

	Pos_No1 = temp_arr_Cmd1[(Cmd_No1 - 1) * POS_CMD_SIZE + 2];
	Pos_No2 = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 2];

	arr_P1 = &w_ParLst_PosPar;
	//	last_arr_cmd=&w_ParLst_LastPos_CMD;
	if (Pos_No1 >= 1 && Pos_No1 <= POS_NUM)
		Pos_No1--;
	if (Pos_No2 >= 1 && Pos_No2 <= POS_NUM)
		Pos_No2--;

	//	temp_MUTI1=(arr_P1[Pos_No2*POS_SIZE+1]-arr_P1[Pos_No1*POS_SIZE+1]);
	temp_SINGLE1 = ((arr_P1[Pos_No2 * POS_SIZE + 3] << 16) + arr_P1[Pos_No2 * POS_SIZE + 2] - ((arr_P1[Pos_No1 * POS_SIZE + 3] << 16) + arr_P1[Pos_No1 * POS_SIZE + 2]));
	//	num1=temp_MUTI1*PULSE_NUM+temp_SINGLE1*PULSE_NUM/ELEC_GEAR;
	//	num1=temp_SINGLE1*PULSE_NUM/ELEC_GEAR_US200;
	tmp1_f = (float)temp_SINGLE1; //�ֿ���������Ϊ������˳�����Χ������Ҫ��ת��Ϊ������
	tmp1_f = (float)tmp1_f * PULSE_NUM;
	tmp1_f = (float)tmp1_f / ELEC_GEAR_US200;
	num1 = (float)tmp1_f;

	//	temp_MUTI2=(arr_P1[Pos_No2*POS_SIZE+4]-arr_P1[Pos_No1*POS_SIZE+4]);
	temp_SINGLE2 = ((arr_P1[Pos_No2 * POS_SIZE + 6] << 16) + arr_P1[Pos_No2 * POS_SIZE + 5] - ((arr_P1[Pos_No1 * POS_SIZE + 6] << 16) + arr_P1[Pos_No1 * POS_SIZE + 5]));
	//	num2=temp_MUTI2*PULSE_NUM+temp_SINGLE2*PULSE_NUM/ELEC_GEAR;
	tmp2_f = (float)temp_SINGLE2; //�ֿ���������Ϊ������˳�����Χ������Ҫ��ת��Ϊ������
	tmp2_f = (float)tmp2_f * PULSE_NUM;
	tmp2_f = (float)tmp2_f / ELEC_GEAR_US200;
	num2 = (float)tmp2_f;

	temp_MUTI3 = (arr_P1[Pos_No2 * POS_SIZE + 7] - arr_P1[Pos_No1 * POS_SIZE + 7]);
	temp_SINGLE3 = (float)((arr_P1[Pos_No2 * POS_SIZE + 9] << 16) + arr_P1[Pos_No2 * POS_SIZE + 8] - ((arr_P1[Pos_No1 * POS_SIZE + 9] << 16) + arr_P1[Pos_No1 * POS_SIZE + 8]));
	//	num3=temp_MUTI3*PULSE_NUM+temp_SINGLE3*PULSE_NUM/ELEC_GEAR;
	num3 = (float)temp_MUTI3;
	num3 = (float)num3 * PULSE_NUM;
	tmp3_f = (float)temp_SINGLE3;
	tmp3_f = (float)tmp3_f * PULSE_NUM;
	tmp3_f = (float)tmp3_f / ELEC_GEAR;
	num3 = (float)(num3 + tmp3_f);

	temp_MUTI4 = (arr_P1[Pos_No2 * POS_SIZE + 10] - arr_P1[Pos_No1 * POS_SIZE + 10]);
	temp_SINGLE4 = (float)((arr_P1[Pos_No2 * POS_SIZE + 12] << 16) + arr_P1[Pos_No2 * POS_SIZE + 11] - ((arr_P1[Pos_No1 * POS_SIZE + 12] << 16) + arr_P1[Pos_No1 * POS_SIZE + 11]));
	//	num4=temp_MUTI4*PULSE_NUM+temp_SINGLE4*PULSE_NUM/ELEC_GEAR;
	num4 = (float)temp_MUTI4;
	num4 = (float)num4 * PULSE_NUM;
	tmp4_f = (float)temp_SINGLE4;
	tmp4_f = (float)tmp4_f * PULSE_NUM;
	tmp4_f = (float)tmp4_f / ELEC_GEAR;
	num4 = (float)(num4 + tmp4_f);

	temp_MUTI5 = (arr_P1[Pos_No2 * POS_SIZE + 13] - arr_P1[Pos_No1 * POS_SIZE + 13]);
	temp_SINGLE5 = (float)((arr_P1[Pos_No2 * POS_SIZE + 15] << 16) + arr_P1[Pos_No2 * POS_SIZE + 14] - ((arr_P1[Pos_No1 * POS_SIZE + 15] << 16) + arr_P1[Pos_No1 * POS_SIZE + 14]));
	//	num5=temp_MUTI5*PULSE_NUM+temp_SINGLE5*PULSE_NUM/ELEC_GEAR;
	num5 = (float)temp_MUTI5;
	num5 = (float)num5 * PULSE_NUM;
	tmp5_f = (float)temp_SINGLE5;
	tmp5_f = (float)tmp5_f * PULSE_NUM;
	tmp5_f = (float)tmp5_f / ELEC_GEAR;
	num5 = (float)(num5 + tmp5_f);

	temp_MUTI6 = (arr_P1[Pos_No2 * POS_SIZE + 16] - arr_P1[Pos_No1 * POS_SIZE + 16]);
	temp_SINGLE6 = (float)((arr_P1[Pos_No2 * POS_SIZE + 18] << 16) + arr_P1[Pos_No2 * POS_SIZE + 17] - ((arr_P1[Pos_No1 * POS_SIZE + 18] << 16) + arr_P1[Pos_No1 * POS_SIZE + 17]));
	//	num6=temp_MUTI6*PULSE_NUM+temp_SINGLE6*PULSE_NUM/ELEC_GEAR;
	num6 = (float)temp_MUTI6;
	num6 = (float)num6 * PULSE_NUM;
	tmp6_f = (float)temp_SINGLE6;
	tmp6_f = (float)tmp6_f * PULSE_NUM;
	tmp6_f = (float)tmp6_f / ELEC_GEAR;
	num6 = (float)(num6 + tmp6_f);

	//ʵ����������
	if (num1 > 0.0f)
		tmp_num1 = (s32)(num1 + 0.5f);
	else
		tmp_num1 = (s32)(num1 - 0.5f);

	if (num2 > 0.0f)
		tmp_num2 = (s32)(num2 + 0.5f);
	else
		tmp_num2 = (s32)(num2 - 0.5f);

	if (num3 > 0.0f)
		tmp_num3 = (s32)(num3 + 0.5f);
	else
		tmp_num3 = (s32)(num3 - 0.5f);

	if (num4 > 0.0f)
		tmp_num4 = (s32)(num4 + 0.5f);
	else
		tmp_num4 = (s32)(num4 - 0.5f);

	if (num5 > 0.0f)
		tmp_num5 = (s32)(num5 + 0.5f);
	else
		tmp_num5 = (s32)(num5 - 0.5f);

	if (num6 > 0.0f)
		tmp_num6 = (s32)(num6 + 0.5f);
	else
		tmp_num6 = (s32)(num6 - 0.5f);

	//2���ȸ���ÿ̨������趨�ٶȼ����ÿ̨���������ʱ��
	if (Pw_Set_Run_Speed1 > 0)
		time[0] = (float)abs(tmp_num1) / Pw_Set_Run_Speed1;

	if (Pw_Set_Run_Speed2 > 0)
		time[1] = (float)abs(tmp_num2) / Pw_Set_Run_Speed2;

	if (Pw_Set_Run_Speed3 > 0)
		time[2] = (float)abs(tmp_num3) / Pw_Set_Run_Speed3;

	if (Pw_Set_Run_Speed4 > 0)
		time[3] = (float)abs(tmp_num4) / Pw_Set_Run_Speed4;

	if (Pw_Set_Run_Speed5 > 0)
		time[4] = (float)abs(tmp_num5) / Pw_Set_Run_Speed5;

	if (Pw_Set_Run_Speed6 > 0)
		time[5] = (float)abs(tmp_num6) / Pw_Set_Run_Speed6;

	//3�����������ʱ��Ϊ��׼��������������������ٶ�3�����������ʱ��Ϊ��׼��������������������ٶ�
	sort_f(time, 6);												   //���򣬵õ��������ʱ��time_d[5]
	Run_Speed_Ratio = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 3]; //�õ�������б���
	if (time[5] > 1)
	{
		f_speed1 = (float)fabs(num1);
		f_speed1 = (float)f_speed1 * Run_Speed_Ratio; //�ֿ���������Ϊ������˳�����Χ������Ҫ��ת��Ϊ������
		f_speed1 = (float)f_speed1 / (100 * time[5]);

		f_speed2 = (float)fabs(num2);
		f_speed2 = (float)f_speed2 * Run_Speed_Ratio; //�ֿ���������Ϊ������˳�����Χ������Ҫ��ת��Ϊ������
		f_speed2 = (float)f_speed2 / (100 * time[5]);

		f_speed3 = (float)fabs(num3);
		f_speed3 = (float)f_speed3 * Run_Speed_Ratio; //�ֿ���������Ϊ������˳�����Χ������Ҫ��ת��Ϊ������
		f_speed3 = (float)f_speed3 / (100 * time[5]);

		f_speed4 = (float)fabs(num4);
		f_speed4 = (float)f_speed4 * Run_Speed_Ratio; //�ֿ���������Ϊ������˳�����Χ������Ҫ��ת��Ϊ������
		f_speed4 = (float)f_speed4 / (100 * time[5]);

		f_speed5 = (float)fabs(num5);
		f_speed5 = (float)f_speed5 * Run_Speed_Ratio; //�ֿ���������Ϊ������˳�����Χ������Ҫ��ת��Ϊ������
		f_speed5 = (float)f_speed5 / (100 * time[5]);

		f_speed6 = (float)fabs(num6);
		f_speed6 = (float)f_speed6 * Run_Speed_Ratio; //�ֿ���������Ϊ������˳�����Χ������Ҫ��ת��Ϊ������
		f_speed6 = (float)f_speed6 / (100 * time[5]);
	}
	else
	{
		f_speed1 = Pw_Driver_Run_MinSpeed;
		f_speed2 = Pw_Driver_Run_MinSpeed;
		f_speed3 = Pw_Driver_Run_MinSpeed;
		f_speed4 = Pw_Driver_Run_MinSpeed;
		f_speed5 = Pw_Driver_Run_MinSpeed;
		f_speed6 = Pw_Driver_Run_MinSpeed;
	}

	//ʵ����������
	if (f_speed1 > 0.0f)
		speed1 = (u32)(f_speed1 + 0.5f);
	else
		speed1 = (u32)(f_speed1 - 0.5f);

	if (f_speed2 > 0.0f)
		speed2 = (u32)(f_speed2 + 0.5f);
	else
		speed2 = (u32)(f_speed2 - 0.5f);

	if (f_speed3 > 0.0f)
		speed3 = (u32)(f_speed3 + 0.5f);
	else
		speed3 = (u32)(f_speed3 - 0.5f);

	if (f_speed4 > 0.0f)
		speed4 = (u32)(f_speed4 + 0.5f);
	else
		speed4 = (u32)(f_speed4 - 0.5f);

	if (f_speed5 > 0.0f)
		speed5 = (u32)(f_speed5 + 0.5f);
	else
		speed5 = (u32)(f_speed5 - 0.5f);

	if (f_speed6 > 0.0f)
		speed6 = (u32)(f_speed6 + 0.5f);
	else
		speed6 = (u32)(f_speed6 - 0.5f);

	if (speed1 < Pw_Driver_Run_MinSpeed)
		speed1 = Pw_Driver_Run_MinSpeed;

	if (speed1 > Pw_Set_Run_Speed1)
		speed1 = Pw_Set_Run_Speed1;

	if (speed2 < Pw_Driver_Run_MinSpeed)
		speed2 = Pw_Driver_Run_MinSpeed;

	if (speed2 > Pw_Set_Run_Speed2)
		speed2 = Pw_Set_Run_Speed2;

	if (speed3 < Pw_Driver_Run_MinSpeed)
		speed3 = Pw_Driver_Run_MinSpeed;

	if (speed3 > Pw_Set_Run_Speed3)
		speed3 = Pw_Set_Run_Speed3;

	if (speed4 < Pw_Driver_Run_MinSpeed)
		speed4 = Pw_Driver_Run_MinSpeed;

	if (speed4 > Pw_Set_Run_Speed4)
		speed4 = Pw_Set_Run_Speed4;

	if (speed5 < Pw_Driver_Run_MinSpeed)
		speed5 = Pw_Driver_Run_MinSpeed;

	if (speed5 > Pw_Set_Run_Speed5)
		speed5 = Pw_Set_Run_Speed5;

	if (speed6 < Pw_Driver_Run_MinSpeed)
		speed6 = Pw_Driver_Run_MinSpeed;

	if (speed6 > Pw_Set_Run_Speed6)
		speed6 = Pw_Set_Run_Speed6;

	//	tmp_num1=num1;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 12] = tmp_num1 & 0x0000FFFF;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 13] = (tmp_num1 & 0xFFFF0000) >> 16;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 14] = speed1;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 15] = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 4]; //�Ӽ���ʱ��ֱ�Ӹ����ܵ�

	//	tmp_num2=num2;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 16] = tmp_num2 & 0x0000FFFF;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 17] = (tmp_num2 & 0xFFFF0000) >> 16;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 18] = speed2;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 19] = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 4]; //�Ӽ���ʱ��ֱ�Ӹ����ܵ�

	//	tmp_num3=num3;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 20] = tmp_num3 & 0x0000FFFF;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 21] = (tmp_num3 & 0xFFFF0000) >> 16;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 22] = speed3;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 23] = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 4]; //�Ӽ���ʱ��ֱ�Ӹ����ܵ�

	//	tmp_num4=num4;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 24] = tmp_num4 & 0x0000FFFF;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 25] = (tmp_num4 & 0xFFFF0000) >> 16;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 26] = speed4;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 27] = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 4]; //�Ӽ���ʱ��ֱ�Ӹ����ܵ�

	//	tmp_num5=num5;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 28] = tmp_num5 & 0x0000FFFF;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 29] = (tmp_num5 & 0xFFFF0000) >> 16;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 30] = speed5;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 31] = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 4]; //�Ӽ���ʱ��ֱ�Ӹ����ܵ�

	//	tmp_num6=num6;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 32] = tmp_num6 & 0x0000FFFF;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 33] = (tmp_num6 & 0xFFFF0000) >> 16;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 34] = speed6;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 35] = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 4]; //�Ӽ���ʱ��ֱ�Ӹ����ܵ�

	//4����������������ٶȣ��õ�ÿ̨�����ʵ������ʱ��
	//		Run_time=abs(tmp_num1)*60000/(PULSE_NUM*speed1)+temp_arr_Cmd2[(Cmd_No2-1)*POS_CMD_SIZE+15]*Pw_Acc_Delay_Ratio/100;
	Run_time1 = abs(tmp_num1);
	Run_time1 = (float)Run_time1 * 60000;
	Run_time1 = (float)Run_time1 / (PULSE_NUM * speed1);
	tmp_time1 = (float)temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 15];
	tmp_time1 = (float)tmp_time1 * Pw_Acc_Delay_Ratio / 100;
	Run_time1 = (float)(Run_time1 + tmp_time1); //������ʱ��=��������ʱ��+�Ӽ���ʱ��*����

	//		Run_time=abs(tmp_num2)*60000/(PULSE_NUM*speed2)+temp_arr_Cmd2[(Cmd_No2-1)*POS_CMD_SIZE+19]*Pw_Acc_Delay_Ratio/100;
	Run_time2 = abs(tmp_num2);
	Run_time2 = (float)Run_time2 * 60000;
	Run_time2 = (float)Run_time2 / (PULSE_NUM * speed2);
	tmp_time2 = (float)temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 19];
	tmp_time2 = (float)tmp_time2 * Pw_Acc_Delay_Ratio / 100;
	Run_time2 = (float)(Run_time2 + tmp_time2);

	//		Run_time=abs(tmp_num3)*60000/(PULSE_NUM*speed3)+temp_arr_Cmd2[(Cmd_No2-1)*POS_CMD_SIZE+23]*Pw_Acc_Delay_Ratio/100;
	Run_time3 = abs(tmp_num3);
	Run_time3 = (float)Run_time3 * 60000;
	Run_time3 = (float)Run_time3 / (PULSE_NUM * speed3);
	tmp_time3 = (float)temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 23];
	tmp_time3 = (float)tmp_time3 * Pw_Acc_Delay_Ratio / 100;
	Run_time3 = (float)(Run_time3 + tmp_time3);

	//		Run_time=abs(tmp_num4)*60000/(PULSE_NUM*speed4)+temp_arr_Cmd2[(Cmd_No2-1)*POS_CMD_SIZE+27]*Pw_Acc_Delay_Ratio/100;
	Run_time4 = abs(tmp_num4);
	Run_time4 = (float)Run_time4 * 60000;
	Run_time4 = (float)Run_time4 / (PULSE_NUM * speed4);
	tmp_time4 = (float)temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 27];
	tmp_time4 = (float)tmp_time4 * Pw_Acc_Delay_Ratio / 100;
	Run_time4 = (float)(Run_time4 + tmp_time4);

	//		Run_time=abs(tmp_num5)*60000/(PULSE_NUM*speed5)+temp_arr_Cmd2[(Cmd_No2-1)*POS_CMD_SIZE+31]*Pw_Acc_Delay_Ratio/100;
	Run_time5 = abs(tmp_num5);
	Run_time5 = (float)Run_time5 * 60000;
	Run_time5 = (float)Run_time5 / (PULSE_NUM * speed5);
	tmp_time5 = (float)temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 31];
	tmp_time5 = (float)tmp_time5 * Pw_Acc_Delay_Ratio / 100;
	Run_time5 = (float)(Run_time5 + tmp_time5);

	//		Run_time=abs(tmp_num6)*60000/(PULSE_NUM*speed6)+temp_arr_Cmd2[(Cmd_No2-1)*POS_CMD_SIZE+35]*Pw_Acc_Delay_Ratio/100;
	Run_time6 = abs(tmp_num6);
	Run_time6 = (float)Run_time6 * 60000;
	Run_time6 = (float)Run_time6 / (PULSE_NUM * speed6);
	tmp_time6 = (float)temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 35];
	tmp_time6 = (float)tmp_time6 * Pw_Acc_Delay_Ratio / 100;
	Run_time6 = (float)(Run_time6 + tmp_time6);

	time[0] = Run_time1;
	time[1] = Run_time2;
	time[2] = Run_time3;
	time[3] = Run_time4;
	time[4] = Run_time5;
	time[5] = Run_time6;

	sort_f(time, 6); //���򣬵õ��������ʱ��time[5]
	Run_time = time[5] / 10 + 1;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 36] = (s32)Run_time;
}

//���е�ĳһ����Ŷ�Ӧ��λ�õ�
//���������Cmd_No�������
void Run_to_One_Pos(s32 Cmd_No)
{
	float num1, num2, num3, num4, num5, num6;
	float tmp1_f, tmp2_f, tmp3_f, tmp4_f, tmp5_f, tmp6_f;
	//	s16 temp_MUTI1,temp_MUTI2;
	s16 temp_MUTI3, temp_MUTI4, temp_MUTI5, temp_MUTI6;
	float temp_SINGLE1, temp_SINGLE2, temp_SINGLE3, temp_SINGLE4, temp_SINGLE5, temp_SINGLE6;
	s32 *tmp_arr_Cmd;
	s32 *tmp_arr_Pos;
	s32 Pos_No;
	s32 speed1, speed2, speed3, speed4, speed5, speed6;
	s32 acc_dec_time;
	s32 temp_pulse;

	Pr_Send_Data_F = 0;
	tmp_arr_Cmd = &w_ParLst_Pos_CMD;
	tmp_arr_Pos = &w_ParLst_PosPar;

	if (Cmd_No >= 1 && Cmd_No <= COM_CMD_NUM - 1)
		Cmd_No--;

	Pos_No = tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + 2];
	if (Pos_No >= 1 && Pos_No <= POS_NUM)
		Pos_No--;

	acc_dec_time = tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + 4];

	//����1#����
	//	temp_MUTI1=(tmp_arr_Pos[Pos_No*POS_SIZE+1]-Pr_Drive1_MultiData);
	temp_SINGLE1 = (float)(((tmp_arr_Pos[Pos_No * POS_SIZE + 3] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + 2]) - ((Pr_Drive1_singleData_HW << 16) + Pr_Drive1_singleData));
	//	num1=temp_MUTI1*PULSE_NUM+temp_SINGLE1*PULSE_NUM/ELEC_GEAR;
	//	num1=temp_MUTI1;												//�ֿ���������Ϊ������˳�����Χ������Ҫ��ת��Ϊ������
	//	num1=num1*PULSE_NUM;
	tmp1_f = (float)temp_SINGLE1;
	tmp1_f = (float)tmp1_f * PULSE_NUM;
	tmp1_f = (float)tmp1_f / ELEC_GEAR_US200;
	num1 = (float)(tmp1_f);

	if (num1 > 0.0f)
		temp_pulse = (s32)(num1 + 0.5f);
	else
		temp_pulse = (s32)(num1 - 0.5f);
	speed1 = tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + 14];
	if (temp_pulse != 0 && speed1 != 0)
		Locate_Rle_1(Pw_EquipmentNo1, temp_pulse, speed1, acc_dec_time, 0);
	Driver1_Pos_Start_Sort = 1;

	//����2#����
	//	temp_MUTI2=(tmp_arr_Pos[Pos_No*POS_SIZE+4]-Pr_Drive2_MultiData);
	temp_SINGLE2 = (float)(((tmp_arr_Pos[Pos_No * POS_SIZE + 6] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + 5]) - ((Pr_Drive2_singleData_HW << 16) + Pr_Drive2_singleData));
	//	num2=temp_MUTI2*PULSE_NUM+temp_SINGLE2*PULSE_NUM/ELEC_GEAR;
	//	num2=temp_MUTI2;
	//	num2=num2*PULSE_NUM;
	tmp2_f = (float)temp_SINGLE2;
	tmp2_f = (float)tmp2_f * PULSE_NUM;
	tmp2_f = (float)tmp2_f / ELEC_GEAR_US200;
	num2 = (float)(tmp2_f);

	if (num2 > 0.0f)
		temp_pulse = (s32)(num2 + 0.5f);
	else
		temp_pulse = (s32)(num2 - 0.5f);
	speed2 = tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + 18];
	if (temp_pulse != 0 && speed2 != 0)
		Locate_Rle_1(Pw_EquipmentNo2, temp_pulse, speed2, acc_dec_time, 0);
	Driver2_Pos_Start_Sort = 1;

	//����1#����
	temp_MUTI3 = (tmp_arr_Pos[Pos_No * POS_SIZE + 7] - Pr_Drive3_MultiData);
	temp_SINGLE3 = (float)(((tmp_arr_Pos[Pos_No * POS_SIZE + 9] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + 8]) - ((Pr_Drive3_singleData_HW << 16) + Pr_Drive3_singleData));
	//	num3=temp_MUTI3*PULSE_NUM+temp_SINGLE3*PULSE_NUM/ELEC_GEAR;
	num3 = (float)temp_MUTI3;
	num3 = (float)num3 * PULSE_NUM;
	tmp3_f = (float)temp_SINGLE3;
	tmp3_f = (float)tmp3_f * PULSE_NUM;
	tmp3_f = (float)tmp3_f / ELEC_GEAR;
	num3 = (float)(num3 + tmp3_f);

	if (num3 > 0.0f)
		temp_pulse = (s32)(num3 + 0.5f);
	else
		temp_pulse = (s32)(num3 - 0.5f);
	speed3 = tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + 22];
	if (temp_pulse != 0 && speed3 != 0)
		Locate_Rle_1(Pw_EquipmentNo3, temp_pulse, speed3, acc_dec_time, 0);
	Driver3_Pos_Start_Sort = 1;

	//����4#����
	temp_MUTI4 = (tmp_arr_Pos[Pos_No * POS_SIZE + 10] - Pr_Drive4_MultiData);
	temp_SINGLE4 = (float)(((tmp_arr_Pos[Pos_No * POS_SIZE + 12] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + 11]) - ((Pr_Drive4_singleData_HW << 16) + Pr_Drive4_singleData));
	//	num4=temp_MUTI4*PULSE_NUM+temp_SINGLE4*PULSE_NUM/ELEC_GEAR;
	num4 = (float)temp_MUTI4;
	num4 = (float)num4 * PULSE_NUM;
	tmp4_f = (float)temp_SINGLE4;
	tmp4_f = (float)tmp4_f * PULSE_NUM;
	tmp4_f = (float)tmp4_f / ELEC_GEAR;
	num4 = (float)(num4 + tmp4_f);

	if (num4 > 0.0f)
		temp_pulse = (s32)(num4 + 0.5f);
	else
		temp_pulse = (s32)(num4 - 0.5f);
	speed4 = tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + 26];
	if (temp_pulse != 0 && speed4 != 0)
		Locate_Rle_1(Pw_EquipmentNo4, temp_pulse, speed4, acc_dec_time, 0);
	Driver4_Pos_Start_Sort = 1;

	//����5#����
	temp_MUTI5 = (tmp_arr_Pos[Pos_No * POS_SIZE + 13] - Pr_Drive5_MultiData);
	temp_SINGLE5 = (float)(((tmp_arr_Pos[Pos_No * POS_SIZE + 15] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + 14]) - ((Pr_Drive5_singleData_HW << 16) + Pr_Drive5_singleData));
	//	num5=temp_MUTI5*PULSE_NUM+temp_SINGLE5*PULSE_NUM/ELEC_GEAR;
	num5 = (float)temp_MUTI5;
	num5 = (float)num5 * PULSE_NUM;
	tmp5_f = (float)temp_SINGLE5;
	tmp5_f = (float)tmp5_f * PULSE_NUM;
	tmp5_f = (float)tmp5_f / ELEC_GEAR;
	num5 = (float)(num5 + tmp5_f);
	if (num5 > 0.0f)
		temp_pulse = (s32)(num5 + 0.5f);
	else
		temp_pulse = (s32)(num5 - 0.5f);
	speed5 = tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + 30];
	if (temp_pulse != 0 && speed5 != 0)
		Locate_Rle_1(Pw_EquipmentNo5, temp_pulse, speed5, acc_dec_time, 0);
	Driver5_Pos_Start_Sort = 1;

	//����6#����
	temp_MUTI6 = (tmp_arr_Pos[Pos_No * POS_SIZE + 16] - Pr_Drive6_MultiData);
	temp_SINGLE6 = (float)(((tmp_arr_Pos[Pos_No * POS_SIZE + 18] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + 17]) - ((Pr_Drive6_singleData_HW << 16) + Pr_Drive6_singleData));
	//	num6=temp_MUTI6*PULSE_NUM+temp_SINGLE6*PULSE_NUM/ELEC_GEAR;
	num6 = (float)temp_MUTI6;
	num6 = (float)num6 * PULSE_NUM;
	tmp6_f = (float)temp_SINGLE6;
	tmp6_f = (float)tmp6_f * PULSE_NUM;
	tmp6_f = (float)tmp6_f / ELEC_GEAR;
	num6 = (float)(num6 + tmp6_f);

	if (num6 > 0.0f)
		temp_pulse = (s32)(num6 + 0.5f);
	else
		temp_pulse = (s32)(num6 - 0.5f);
	speed6 = tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + 34];
	if (temp_pulse != 0 && speed6 != 0)
		Locate_Rle_1(Pw_EquipmentNo6, temp_pulse, speed6, acc_dec_time, 0);
	Driver6_Pos_Start_Sort = 1;
}

//������λ���ֶ���������
void Pos_Manual_Adj(void)
{
	if (Pw_TouchRunStop == 1 && Pw_Pos_Adj_Cmd == 1) //����ͣ��״̬�£����ܵ���
	{
		Pw_Pos_Adj_Cmd = 0;

		if (Pw_Driver1_PosErr_Muti != 0 || Pw_Driver1_PosErr_Sing != 0 || Pw_Driver1_PosErr_Sing_HW != 0)
		{
			Adj_Pos(1, Pw_Driver1_PosErr_Muti, (Pw_Driver1_PosErr_Sing_HW << 16) + Pw_Driver1_PosErr_Sing);
			Pw_Driver1_PosErr_Muti = 0;
			Pw_Driver1_PosErr_Sing = 0;
			Pw_Driver1_PosErr_Sing_HW = 0;
		}

		if (Pw_Driver2_PosErr_Muti != 0 || Pw_Driver2_PosErr_Sing != 0 || Pw_Driver2_PosErr_Sing_HW != 0)
		{
			Adj_Pos(2, Pw_Driver2_PosErr_Muti, (Pw_Driver2_PosErr_Sing_HW << 16) + Pw_Driver2_PosErr_Sing);
			Pw_Driver2_PosErr_Muti = 0;
			Pw_Driver2_PosErr_Sing = 0;
			Pw_Driver2_PosErr_Sing_HW = 0;
		}

		if (Pw_Driver3_PosErr_Muti != 0 || Pw_Driver3_PosErr_Sing != 0 || Pw_Driver3_PosErr_Sing_HW != 0)
		{
			Adj_Pos(3, Pw_Driver3_PosErr_Muti, (Pw_Driver3_PosErr_Sing_HW << 16) + Pw_Driver3_PosErr_Sing);
			Pw_Driver3_PosErr_Muti = 0;
			Pw_Driver3_PosErr_Sing = 0;
			Pw_Driver3_PosErr_Sing_HW = 0;
		}

		if (Pw_Driver4_PosErr_Muti != 0 || Pw_Driver4_PosErr_Sing != 0 || Pw_Driver4_PosErr_Sing_HW != 0)
		{
			Adj_Pos(4, Pw_Driver4_PosErr_Muti, (Pw_Driver4_PosErr_Sing_HW << 16) + Pw_Driver4_PosErr_Sing);
			Pw_Driver4_PosErr_Muti = 0;
			Pw_Driver4_PosErr_Sing = 0;
			Pw_Driver4_PosErr_Sing_HW = 0;
		}

		if (Pw_Driver5_PosErr_Muti != 0 || Pw_Driver5_PosErr_Sing != 0 || Pw_Driver5_PosErr_Sing_HW != 0)
		{
			Adj_Pos(5, Pw_Driver5_PosErr_Muti, (Pw_Driver5_PosErr_Sing_HW << 16) + Pw_Driver5_PosErr_Sing);
			Pw_Driver5_PosErr_Muti = 0;
			Pw_Driver5_PosErr_Sing = 0;
			Pw_Driver5_PosErr_Sing_HW = 0;
		}

		if (Pw_Driver6_PosErr_Muti != 0 || Pw_Driver6_PosErr_Sing != 0 || Pw_Driver6_PosErr_Sing_HW != 0)
		{
			Adj_Pos(6, Pw_Driver6_PosErr_Muti, (Pw_Driver6_PosErr_Sing_HW << 16) + Pw_Driver6_PosErr_Sing);
			Pw_Driver6_PosErr_Muti = 0;
			Pw_Driver6_PosErr_Sing = 0;
			Pw_Driver6_PosErr_Sing_HW = 0;
		}
	}
}

//����λ��
void Adj_Pos(u8 driver_no, s32 PosErr_Muti, s32 PosErr_Sing)
{
	s32 *tmp_arr_Pos;
	u8 k;
	s32 tmp_Pos;

	//	driver_no--;
	tmp_arr_Pos = &w_ParLst_PosPar;
	for (k = 0; k < POS_NUM - 1; k++)
	{
		if (tmp_arr_Pos[k * POS_SIZE] != 0)
		{
			tmp_Pos = (tmp_arr_Pos[k * POS_SIZE + (driver_no - 1) * 3 + 3] << 16) + tmp_arr_Pos[k * POS_SIZE + (driver_no - 1) * 3 + 2];
			tmp_Pos += PosErr_Sing;
			if (driver_no > 2)
			{
				tmp_Pos = tmp_Pos & 0x7FFFFF; //ȡ�࣬��RA1�ŷ�����Ȧ�����8388608
			}
			tmp_arr_Pos[k * POS_SIZE + (driver_no - 1) * 3 + 2] = tmp_Pos & 0x0000FFFF;			//��Ȧ����
			tmp_arr_Pos[k * POS_SIZE + (driver_no - 1) * 3 + 3] = (tmp_Pos & 0xFFFF0000) >> 16; //��Ȧ����
			tmp_arr_Pos[k * POS_SIZE + (driver_no - 1) * 3 + 1] += PosErr_Muti;					//��Ȧ
		}
		else
			break;
	}
}

//���������Сλ�÷�Χ
u32 T_Driver1_OverPos;
u32 C_Driver1_OverPos;
u32 T_Driver2_OverPos;
u32 C_Driver2_OverPos;
u32 T_Driver3_OverPos;
u32 C_Driver3_OverPos;
u32 T_Driver4_OverPos;
u32 C_Driver4_OverPos;
u32 T_Driver5_OverPos;
u32 C_Driver5_OverPos;
u32 T_Driver6_OverPos;
u32 C_Driver6_OverPos;

void Limit_Max_Pos(void)
{
	if (Pw_TouchRunStop == 0 && F_Starting == 0 && Pr_Driver_Previous_No > 0) //��������Զ�״̬�����е������ʱ�����Ҳ����������̣����ж�
	{
		//!=0,��λ�����ƹ���
		Pr_OverMaxPos_F = 0;
		if (((Pw_Limit_MaxPos & 0x01) == 0x01) && Pr_F_Drive1_Runing != 0)
		{
			if (T_Driver1_OverPos != SClk10Ms)
			{
				T_Driver1_OverPos = SClk10Ms; //
				C_Driver1_OverPos++;
				if (C_Driver1_OverPos > 2) //��ʱ�ж�
				{
					C_Driver1_OverPos = 0;
					T_Driver1_OverPos = 1000;

					if (Judge_OverPos(Pw_EquipmentNo1))
					{
						Pr_OverMaxPos_F = 1; //�ó���λ�����Ʊ�־λ
					}
				}
			}
		}

		if (((Pw_Limit_MaxPos & 0x02) == 0x02) && Pr_F_Drive2_Runing != 0)
		{
			if (T_Driver2_OverPos != SClk10Ms)
			{
				T_Driver2_OverPos = SClk10Ms; //
				C_Driver2_OverPos++;
				if (C_Driver2_OverPos > 2) //��ʱ�ж�
				{
					C_Driver2_OverPos = 0;
					T_Driver2_OverPos = 1000;

					if (Judge_OverPos(Pw_EquipmentNo2))
					{
						Pr_OverMaxPos_F = 1; //�ó���λ�����Ʊ�־λ
					}
				}
			}
		}

		if (((Pw_Limit_MaxPos & 0x04) == 0x04) && Pr_F_Drive3_Runing != 0)
		{
			if (T_Driver3_OverPos != SClk10Ms)
			{
				T_Driver3_OverPos = SClk10Ms; //
				C_Driver3_OverPos++;
				if (C_Driver3_OverPos > 2) //��ʱ�ж�
				{
					C_Driver3_OverPos = 0;
					T_Driver3_OverPos = 1000;

					if (Judge_OverPos(Pw_EquipmentNo3))
					{
						Pr_OverMaxPos_F = 1; //�ó���λ�����Ʊ�־λ
					}
				}
			}
		}

		if (((Pw_Limit_MaxPos & 0x08) == 0x08) && Pr_F_Drive4_Runing != 0)
		{
			if (T_Driver4_OverPos != SClk10Ms)
			{
				T_Driver4_OverPos = SClk10Ms; //
				C_Driver4_OverPos++;
				if (C_Driver4_OverPos > 2) //��ʱ�ж�
				{
					C_Driver4_OverPos = 0;
					T_Driver4_OverPos = 1000;

					if (Judge_OverPos(Pw_EquipmentNo4))
					{
						Pr_OverMaxPos_F = 1; //�ó���λ�����Ʊ�־λ
					}
				}
			}
		}

		if (((Pw_Limit_MaxPos & 0x01) == 0x10) && Pr_F_Drive5_Runing != 0)
		{
			if (T_Driver5_OverPos != SClk10Ms)
			{
				T_Driver5_OverPos = SClk10Ms; //
				C_Driver5_OverPos++;
				if (C_Driver5_OverPos > 2) //��ʱ�ж�
				{
					C_Driver5_OverPos = 0;
					T_Driver5_OverPos = 1000;

					if (Judge_OverPos(Pw_EquipmentNo5))
					{
						Pr_OverMaxPos_F = 1; //�ó���λ�����Ʊ�־λ
					}
				}
			}
		}

		if (((Pw_Limit_MaxPos & 0x20) == 0x20) && Pr_F_Drive6_Runing != 0)
		{
			if (T_Driver6_OverPos != SClk10Ms)
			{
				T_Driver6_OverPos = SClk10Ms; //
				C_Driver6_OverPos++;
				if (C_Driver6_OverPos > 2) //��ʱ�ж�
				{
					C_Driver6_OverPos = 0;
					T_Driver6_OverPos = 1000;

					if (Judge_OverPos(Pw_EquipmentNo6))
					{
						Pr_OverMaxPos_F = 1; //�ó���λ�����Ʊ�־λ
					}
				}
			}
		}

		if (Pr_OverMaxPos_F)
		{
			Clear_Cmd_Queue();	 //���������
			Pw_TouchRunStop = 1; //ֹͣģʽ
			F_SendStopCMD2 = 0;
			Pr_BRAKE_Control = 1;					  //�й��ϣ�ɲ��
			START_BRAKE_SYSTEM;						  //ɲ��
			Pw_EquipStatus = Pw_EquipStatus | 0x0020; //=32������λ������ͣ��
		}
		else
		{
			Pr_OverMaxPos_F = 0;					  //�峬��λ�����Ʊ�־λ
			Pw_EquipStatus = Pw_EquipStatus & 0xFFDF; //���־λ
		}
	}
}

//�ж�ĳ̨����Ƿ񳬳�λ�÷�Χ
//���룺Driver_No
//���:=1��������Χ��=0��û�г�����Χ
u8 Judge_OverPos(u8 Driver_No)
{
	s32 *temp_arr_Cmd1;
	s32 *arr_Pos1;
	s32 *arr_Pos2;
	s32 Pos_No1, Pos_No2;
	s32 Cmd_No1, Cmd_No2;
	u32 temp_PosError_Set;
	s32 *P_Real_Muti_Pos;
	s32 *P_Real_Single_Pos;
	u32 Tmp_ELEC_Gear;
	float f_Current_Pos, f_Set_Pos1, f_Set_Pos2, f_Tmp_Pos;

	//�õ�λ��ƫ��
	temp_PosError_Set = (Pw_PosError_Set_HW << 16) + Pw_PosError_Set;

	//�õ������
	Cmd_No1 = Pr_Driver_Previous_No;
	Cmd_No2 = arrp_p1_Last[1];

	//�õ�λ�ú�
	temp_arr_Cmd1 = &w_ParLst_Pos_CMD;
	Pos_No1 = temp_arr_Cmd1[(Cmd_No1 - 1) * POS_CMD_SIZE + 2];
	Pos_No2 = temp_arr_Cmd1[(Cmd_No2 - 1) * POS_CMD_SIZE + 2];

	//ָ��ǰ�����λ��
	arr_Pos1 = &w_ParLst_PosPar;
	arr_Pos1 += ((Pos_No1 - 1) * POS_SIZE + (Driver_No - 1) * 3 + 1);

	//ָ����һ�������λ��
	arr_Pos2 = &w_ParLst_PosPar;
	arr_Pos2 += ((Pos_No2 - 1) * POS_SIZE + (Driver_No - 1) * 3 + 1);

	//�õ���ǰʵʱλ��
	//�ȵõ���ǰλ��ָ��
	if (Driver_No == Pw_EquipmentNo1)
	{
		P_Real_Muti_Pos = &Pr_Drive1_MultiData;
		P_Real_Single_Pos = &Pr_Drive1_singleData;
	}
	else if (Driver_No == Pw_EquipmentNo2)
	{
		P_Real_Muti_Pos = &Pr_Drive2_MultiData;
		P_Real_Single_Pos = &Pr_Drive2_singleData;
	}
	else if (Driver_No == Pw_EquipmentNo3)
	{
		P_Real_Muti_Pos = &Pr_Drive3_MultiData;
		P_Real_Single_Pos = &Pr_Drive3_singleData;
	}
	else if (Driver_No == Pw_EquipmentNo4)
	{
		P_Real_Muti_Pos = &Pr_Drive4_MultiData;
		P_Real_Single_Pos = &Pr_Drive4_singleData;
	}
	else if (Driver_No == Pw_EquipmentNo5)
	{
		P_Real_Muti_Pos = &Pr_Drive5_MultiData;
		P_Real_Single_Pos = &Pr_Drive5_singleData;
	}
	else if (Driver_No == Pw_EquipmentNo6)
	{
		P_Real_Muti_Pos = &Pr_Drive6_MultiData;
		P_Real_Single_Pos = &Pr_Drive6_singleData;
	}

	if (Driver_No == Pw_EquipmentNo1 || Driver_No == Pw_EquipmentNo2)
		Tmp_ELEC_Gear = ELEC_GEAR_US200;
	else
		Tmp_ELEC_Gear = ELEC_GEAR;

	//�õ���ǰλ��
	f_Current_Pos = (float)((*P_Real_Muti_Pos) * Tmp_ELEC_Gear + (*(P_Real_Single_Pos + 1) << 16) + (*P_Real_Single_Pos));
	//�õ�λ��1
	f_Set_Pos1 = (float)((*arr_Pos1) * Tmp_ELEC_Gear + (*(arr_Pos1 + 2) << 16) + (*(arr_Pos1 + 1)));
	//�õ�λ��2
	f_Set_Pos2 = (float)((*arr_Pos2) * Tmp_ELEC_Gear + (*(arr_Pos2 + 2) << 16) + (*(arr_Pos2 + 1)));

	if (f_Set_Pos1 > f_Set_Pos2)
	{
		f_Tmp_Pos = f_Set_Pos1;
		f_Set_Pos1 = f_Set_Pos2;
		f_Set_Pos2 = f_Tmp_Pos;
	}

	if (f_Current_Pos < (f_Set_Pos1 - temp_PosError_Set) || f_Current_Pos > (f_Set_Pos2 + temp_PosError_Set))
		return 1; //����1��������Χ
	else
		return 0; //����0��û�г�����Χ
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
