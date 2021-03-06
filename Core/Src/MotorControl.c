

/* Includes ------------------------------------------------------------------*/
#include "MotorControl.h"
#include "GlobalConst.h"
#include "GlobalV_Extern.h" // 全局变量声明
#include "typedef.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>
#include "global_varial.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private variables extern --------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

void IWDG_Init(u8 prer, u16 rlr); //独立看门狗初始化 2013.7.3
void IWDG_Feed(void);			  //独立看门狗喂狗	 2013.7.3
void PowerDelay(u16 nCount);
void Driver_Control(void); //电机运行控制
u8 IS_Driver_Stop(void);
void Normal_Run(void);
u8 Locate_Rle_1(u8 driver_no, s32 num, u16 speed, u16 acc_dec_time, s32 Cmd_No); //相对定位函数
u8 Fill_Pos_Data(u8 driver_no, s32 num, u16 speed, u16 acc_dec_time, s32 Cmd_No);
u8 Send_Start_Cmd(u8 driver_no);
void Run_Driver1(void);
void Run_Driver2(void);
void Run_Driver3(void);
void Run_Driver4(void);
void Run_Driver5(void);
void Run_Driver6(void);
//判断COM1命令队列是否为空
u8 Com1_Driver1_Queue_isEmpty(void);
u8 Com1_Driver2_Queue_isEmpty(void);
u8 Com2_Driver3_Queue_isEmpty(void);
u8 Com2_Driver4_Queue_isEmpty(void);
u8 Com3_Driver5_Queue_isEmpty(void);
u8 Com3_Driver6_Queue_isEmpty(void);

//判断COM1命令队列是否为满
u8 Com1_Driver1_Queue_isFull(void);
u8 Com1_Driver2_Queue_isFull(void);
u8 Com2_Driver3_Queue_isFull(void);
u8 Com2_Driver4_Queue_isFull(void);
u8 Com3_Driver5_Queue_isFull(void);
u8 Com3_Driver6_Queue_isFull(void);

void Reset_Routine(void);						  //复位子程序
void Reset_Drivers(void);						  //复位电机到初始位置
void Stop_Driver(u8 driver_no);					  //发送停机命令
void Control_Hand(void);						  //控制电磁阀(机械手)开通，关断
void BRAKE_Control(void);						  //刹车控制
void is_Reseted(void);							  //判断复位到原点子程序
void Send_StopCMD(void);						  //发送停机命令子程序
void PosCMD_ReadFrom_FLASH(void);				  //按组进行参数初始化
void Driver_Save_Pos(void);						  //写入位置参数到伺服驱动器
s16 Move_Cmd_Queue(u8 driver_no);				  //移动队列指针
void Clear_Cmd_Queue(void);						  //清命令缓冲区
void sort(s32 *a, int l);						  //数组排序函数
void sort_f(float *a, int l);					  //浮点数数组排序函数
u8 Cal_Pos_Speed(void);							  //计算脉冲函数
void Cal_One_Pos_Speed(s32 Pos_No1, s32 Pos_No2); //计算一个位置的脉冲与速度
void Record_Current_Pos(void);					  //记录当前位置
void Run_to_One_Pos(s32 Cmd_No);				  //运行到某一命令号对应的位置点
void Fill_Cmd(void);							  //填充命令
void Cal_Sum_Err(u8 Driver_No);					  //计算四舍五入带来的误差
void Cal_Run_to_Next_Pos(s32 Cmd_No);			  //从当前位置同步运行到某一命令号对应的位置点
void Send_Driver_CMD(u8 driver_no, u8 Modbus_CMD_No, u32 Par_addr, u32 Par_content);
//清零位置及位置命令
extern void ParLst_Init_Group2Zero(void);
//清零位置
void ParLst_Init_Pos2Zero(void);
//编码器位置手动调整函数
void Pos_Manual_Adj(void);
//调整位置
void Adj_Pos(u8 driver_no, s32 PosErr_Muti, s32 PosErr_Sing);
//发送位置数据
void Send_Pos_Data(void);
void Limit_Max_Pos(void); //限制最大最小位置范围
//判断某台电机是否超出位置范围
u8 Judge_OverPos(u8 Driver_No);
/* Private functions ---------------------------------------------------------*/

void Driver_Control(void) // 电机控制
{
	s32 *tmp_arr_p1;
	//Pw_TouchRunStop=0，启动
	if (Pw_StepAutoMode == 0 && F_ForceReseting == 0) //Pw_StepAutoMode=1，手动模式（默认值）；=0，全自动模式
	{
		F_StepMode = 0; //=0，进入自动模式；=1，进入手动模式

		Pw_EquipStatus = Pw_EquipStatus & 0xFFF7; //清触摸屏控制停机状态位
		Pw_EquipStatus = Pw_EquipStatus & 0xFFEF; //清手动状态位

		if (F_PowerOnRun == 1) //=1，允许上电自动运行
		{
			//如果上电允许启动，并且现在还没有启动，则延时复位
			if (Pw_TouchRunStop == 0 && F_TouchRunStop == 1) //Pw_TouchRunStop=0，启动;=1停止
			{
				if (F_SendStopCMD2 == 0) //如果还没有发送停机命令标志
				{
					Send_StopCMD();		//则发送停机命令（只使能伺服）
					F_SendStopCMD2 = 1; //置已经发送停机命令标志
				}

				Pr_RUN_Count = 0; //清零运行圈数
								  //延时
				if (T_StepAutoDelay != SClk10Ms)
				{
					T_StepAutoDelay = SClk10Ms;
					C_T_StepAutoDelayDelay++;
					if (C_T_StepAutoDelayDelay > 30)
					{
						C_T_StepAutoDelayDelay = 0;

						//延时后，开刹车
						Pr_BRAKE_Control = 0; //开刹车
						STOP_BRAKE_SYSTEM;	  //打开刹车系统，运行

						//启动前先执行复位
						//							if(Pr_F_AllStopped )		//if(Pr_F_AllStopped && F_AllRdy)
						//							{
						if (F_Starting == 0 && F_Stoping == 0 && F_ForceReseting == 0)
						{
							F_AskStop = 0;						  //延时完毕，设置启动命令
							F_TouchRunStop = 0;					  //现在是启动状态
							F_Starting = 1;						  //正在启动标志
							Pw_Current_Run_Time = Pr_Reset_Delay; //复位延时
							if (F_Reseted == 0)					  //如果没有在初始点，则进行初始复位
								Reset_Routine();				  //启动前，先执行复位程序
						}
						//							}
					}
				}
			}

			//判断是否是正在启动过程
			//如果是，则进行延时
			if (Pw_TouchRunStop == 0 && F_AskStop == 0 && F_Starting == 1)
			{
				if (F_Reseted == 0) //如果没有在初始点，则延时
				{
					if (T_ResetDelay2 != SClk10Ms)
					{
						T_ResetDelay2 = SClk10Ms;
						C_ResetDelay2++;
						if (C_ResetDelay2 > Pr_Reset_Delay && Pr_F_AllStopped != 0) //延时3s
						{
							C_ResetDelay2 = 0;
							F_Starting = 0; //启动过程结束
						}
					}
				}
				else				//如果已经在初始点，则不用延时
					F_Starting = 0; //启动过程结束
			}
			//如果启动过程结束，则进行正常运行
			else if (Pw_TouchRunStop == 0 && F_AskStop == 0 && F_Starting == 0 && F_Stoping == 0 && F_ForceReseting == 0) //已启动完毕
			{																											  //发生伺服故障停机功能，=1，停机；=0，不停机，仍然运行
				//并且没有达到指定运行圈数
				if ((Pw_Fault_Stop & Pr_F_HaveFault) == 0 && (Pw_ComErr_Stop & Pr_F_ComErr) == 0)
				{
					if (Pr_RUN_Count_Set == 0 || (Pr_RUN_Count_Set > 0 && Pr_RUN_Count < Pr_RUN_Count_Set))
					{
						// Send_Pos_Data();						  //只发送位置数据
						F_Sync_6_axis = 1;						  //置6轴同步运行标志
						Normal_Run();							  //根据条件发送启动命令
						Pw_EquipStatus = Pw_EquipStatus & 0xFFF8; //清3个状态位
					}
					else
					{
						Pw_TouchRunStop = 1; //只切换到停止状态，不清缓冲区，也不刹车
						F_SendStopCMD2 = 0;
						F_Sync_6_axis = 0; //清6轴同步运行标志
						if (Pr_RUN_Count >= Pr_RUN_Count_Set)
						{
							Pw_EquipStatus = Pw_EquipStatus | 0x0004; //=4，达到运行圈数停机
						}
					}
				}
				else
				{
					Clear_Cmd_Queue();	 //清命令缓冲区
					Pw_StepAutoMode = 1; //手动模式
					Pw_TouchRunStop = 1; //停止模式
					F_SendStopCMD2 = 0;
					Pr_BRAKE_Control = 1; //有故障，刹车
					START_BRAKE_SYSTEM;	  //刹车

					if ((Pw_Fault_Stop & Pr_F_HaveFault) != 0)
					{
						Pw_EquipStatus = Pw_EquipStatus | 0x0001; //=1，伺服故障停机
					}
					else if ((Pw_ComErr_Stop & Pr_F_ComErr) != 0)
					{
						Pw_EquipStatus = Pw_EquipStatus | 0x0002; //=2，通讯故障停机
					}
				}
			}
		}

		//执行触摸停机
		if (Pw_TouchRunStop == 1 && F_TouchRunStop == 0 && F_Stoping == 0)
		{
			F_PowerOnRun = 1;
			F_TouchRunStop = 1;
			F_AskStop = 1;
			F_Stoping = 1;
			Pw_EquipStatus = Pw_EquipStatus | 0x0008; //=8，触摸停机
			CLOSE_HAND;

			F_Sync_6_axis = 0; //清6轴同步运行标志
			Pr_Send_Data_F = 0;

			F_Driver1_Timeout = 0; //清标志位
			Pr_Driver1_Cmd_OK_F = 0;
			F_Driver1_Cmd_Err = 0;
			Driver1_Pos_Start_Sort = 0; //=0，表示还未写入到命令缓冲区
			Pr_Driver1_Control_OK_F = 0;
			Driver1_Status_Sort = 0; //=0，表示还未写入到命令缓冲区
			F_Driver1_Cmd_Con_Err = 0;

			F_Driver2_Timeout = 0; //清标志位
			Pr_Driver2_Cmd_OK_F = 0;
			F_Driver2_Cmd_Err = 0;
			Driver2_Pos_Start_Sort = 0;
			Pr_Driver2_Control_OK_F = 0;
			Driver2_Status_Sort = 0; //=0，表示还未写入到命令缓冲区
			F_Driver2_Cmd_Con_Err = 0;

			F_Driver3_Timeout = 0; //清标志位
			Pr_Driver3_Cmd_OK_F = 0;
			F_Driver3_Cmd_Err = 0;
			Driver3_Pos_Start_Sort = 0;
			Pr_Driver3_Control_OK_F = 0;
			Driver3_Status_Sort = 0; //=0，表示还未写入到命令缓冲区
			F_Driver3_Cmd_Con_Err = 0;

			F_Driver4_Timeout = 0; //清标志位
			Pr_Driver4_Cmd_OK_F = 0;
			F_Driver4_Cmd_Err = 0;
			Driver4_Pos_Start_Sort = 0;
			Pr_Driver4_Control_OK_F = 0;
			Driver4_Status_Sort = 0; //=0，表示还未写入到命令缓冲区
			F_Driver4_Cmd_Con_Err = 0;

			F_Driver5_Timeout = 0; //清标志位
			Pr_Driver5_Cmd_OK_F = 0;
			F_Driver5_Cmd_Err = 0;
			Driver5_Pos_Start_Sort = 0;
			Pr_Driver5_Control_OK_F = 0;
			Driver5_Status_Sort = 0; //=0，表示还未写入到命令缓冲区
			F_Driver5_Cmd_Con_Err = 0;

			F_Driver6_Timeout = 0; //清标志位
			Pr_Driver6_Cmd_OK_F = 0;
			F_Driver6_Cmd_Err = 0;
			Driver6_Pos_Start_Sort = 0;
			Pr_Driver6_Control_OK_F = 0;
			Driver6_Status_Sort = 0; //=0，表示还未写入到命令缓冲区
			F_Driver6_Cmd_Con_Err = 0;
		}

		//判断是否停机过程
		//如果是，则进行延时
		if (F_AskStop == 1 && F_Stoping == 1)
		{
			if (T_ResetDelay3 != SClk10Ms)
			{
				T_ResetDelay3 = SClk10Ms;
				C_ResetDelay3++;
				if (C_ResetDelay3 > 10 && Pr_F_AllStopped != 0) //所有电机都停止，并且延时10*10ms=100ms
				{
					C_ResetDelay3 = 0;
					F_Stoping = 0; //=0，表示停机完成
					F_AskStop = 0;

					arr_p1 = &w_ParLst_Pos_CMD; //执行第一条指令
					arrp_p1_Last = arr_p1;
				}
			}
			else
				F_Stoping = 1;
		}
	}
	else if (Pw_StepAutoMode == 1 || (Pw_StepAutoMode == 0 && F_StepMode == 0 && F_ForceReseting == 1)) //=1，手动模式
	{
		F_TouchRunStop = 1;
		Pw_EquipStatus = Pw_EquipStatus | 0x0010; //=16，手动状态

		if (F_StepMode == 0) //如果还没复位，还没进入手动模式，则执行复位
		{
			if (!F_TouchForceReSet && F_Stoping == 0) //w_ParLst[317]=2，强制复位命令参数
			{
				//					Send_StopCMD();								//发送停机命令29

				//延时
				if (T_StepAutoDelay2 != SClk10Ms)
				{
					T_StepAutoDelay2 = SClk10Ms;

					C_T_StepAutoDelayDelay2++;
					if (C_T_StepAutoDelayDelay2 > 200) //延时2s
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
					if (C_ResetDelay2 > Pr_Reset_Delay && Pr_F_AllStopped != 0) //延时2s
					{
						C_ResetDelay2 = 0;

						F_ForceReseting = 0;
						F_AskForceReset = 0;
						F_Stoping = 0; //=0，表示停机完成
						F_TouchForceReSet = 0;
						//					Pw_TouchRunStop=0;
						F_StepMode = 1; //复位停机后，进入手动模式

						arr_p1 = &w_ParLst_Pos_CMD; //执行第一条指令
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
				HAL_Delay(100);
				tmp_arr_p1 = &w_ParLst_Pos_CMD;
				Run_to_One_Pos(tmp_arr_p1[(Pw_Running_Pos_CMD - 1) * POS_CMD_SIZE + 1]); //用手动方式同步运行到某一点
				Pw_Step_Pos_CMD = 0;
			}
			else if (Pw_Step_Pos_CMD == 0 && Pr_F_AllStopped == 1)
			{
				//1#手动，正转
				if (Pw_Driver1_Enable == 1 && Pw_Driver1_R_Enable == 0)
				{
					HAL_Delay(100);
					// Locate_Rle_1(Pw_EquipmentNo1, (Pw_Driver1_Pluse_HW << 16) + Pw_Driver1_Pluse, Pw_Driver1_Speed, Pw_Driver1_AccTime, 0);
					Run_Motor_S(Pw_EquipmentNo1, M1_CLOCKWISE, (Pw_Driver1_Pluse_HW << 16) + Pw_Driver1_Pluse, Pw_Motor1_StartSpeed, Pw_Driver1_Speed, Pw_Driver1_AccTime);

					Pw_Driver1_Enable = 0;
					Driver1_Pos_Start_Sort = 1; //=1，表示已经写入到命令缓冲区
					Pr_Send_Data_F &= 0xFE;
				}
				else if (Pw_Driver1_Enable == 0 && Pw_Driver1_R_Enable == 1) //1#手动，反转
				{
					HAL_Delay(100);
					// Locate_Rle_1(Pw_EquipmentNo1, -((Pw_Driver1_Pluse_HW << 16) + Pw_Driver1_Pluse), Pw_Driver1_Speed, Pw_Driver1_AccTime, 0);
					Run_Motor_S(Pw_EquipmentNo1, M1_UNCLOCKWISE, -((Pw_Driver1_Pluse_HW << 16) + Pw_Driver1_Pluse), Pw_Motor1_StartSpeed, Pw_Driver1_Speed, Pw_Driver1_AccTime);

					Pw_Driver1_R_Enable = 0;
					Driver1_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xFE;
				}

				//2#
				if (Pw_Driver2_Enable == 1 && Pw_Driver2_R_Enable == 0)
				{
					HAL_Delay(100);
					// Locate_Rle_1(Pw_EquipmentNo2, (Pw_Driver2_Pluse_HW << 16) + Pw_Driver2_Pluse, Pw_Driver2_Speed, Pw_Driver2_AccTime, 0);
					Run_Motor_S(Pw_EquipmentNo2, M2_CLOCKWISE, (Pw_Driver2_Pluse_HW << 16) + Pw_Driver2_Pluse, Pw_Motor2_StartSpeed, Pw_Driver2_Speed, Pw_Driver2_AccTime);
					Pw_Driver2_Enable = 0;
					Driver2_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xFD;
				}
				else if (Pw_Driver2_Enable == 0 && Pw_Driver2_R_Enable == 1)
				{
					HAL_Delay(100);
					// Locate_Rle_1(Pw_EquipmentNo2, -((Pw_Driver2_Pluse_HW << 16) + Pw_Driver2_Pluse), Pw_Driver2_Speed, Pw_Driver2_AccTime, 0);
					Run_Motor_S(Pw_EquipmentNo2, M2_UNCLOCKWISE, -((Pw_Driver2_Pluse_HW << 16) + Pw_Driver2_Pluse), Pw_Motor2_StartSpeed, Pw_Driver2_Speed, Pw_Driver2_AccTime);
					Pw_Driver2_R_Enable = 0;
					Driver2_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xFD;
				}

				//3#
				if (Pw_Driver3_Enable == 1 && Pw_Driver3_R_Enable == 0)
				{
					HAL_Delay(100);
					// Locate_Rle_1(Pw_EquipmentNo3, (Pw_Driver3_Pluse_HW << 16) + Pw_Driver3_Pluse, Pw_Driver3_Speed, Pw_Driver3_AccTime, 0);
					Run_Motor_S(Pw_EquipmentNo3, M3_CLOCKWISE, (Pw_Driver3_Pluse_HW << 16) + Pw_Driver3_Pluse, Pw_Motor3_StartSpeed, Pw_Driver3_Speed, Pw_Driver3_AccTime);
					Pw_Driver3_Enable = 0;
					Driver3_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xFB;
				}
				else if (Pw_Driver3_Enable == 0 && Pw_Driver3_R_Enable == 1)
				{
					HAL_Delay(100);
					// Locate_Rle_1(Pw_EquipmentNo3, -((Pw_Driver3_Pluse_HW << 16) + Pw_Driver3_Pluse), Pw_Driver3_Speed, Pw_Driver3_AccTime, 0);
					Run_Motor_S(Pw_EquipmentNo3, M3_UNCLOCKWISE, (Pw_Driver3_Pluse_HW << 16) + Pw_Driver3_Pluse, Pw_Motor3_StartSpeed, Pw_Driver3_Speed, Pw_Driver3_AccTime);
					Pw_Driver3_R_Enable = 0;
					Driver3_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xFB;
				}

				//4#
				if (Pw_Driver4_Enable == 1 && Pw_Driver4_R_Enable == 0)
				{
					HAL_Delay(100);
					// Locate_Rle_1(Pw_EquipmentNo4, (Pw_Driver4_Pluse_HW << 16) + Pw_Driver4_Pluse, Pw_Driver4_Speed, Pw_Driver4_AccTime, 0);
					Run_Motor_S(Pw_EquipmentNo4, M4_CLOCKWISE, (Pw_Driver4_Pluse_HW << 16) + Pw_Driver4_Pluse, Pw_Motor4_StartSpeed, Pw_Driver4_Speed, Pw_Driver4_AccTime);
					Pw_Driver4_Enable = 0;
					Driver4_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xF7;
				}
				else if (Pw_Driver4_Enable == 0 && Pw_Driver4_R_Enable == 1)
				{
					HAL_Delay(100);
					// Locate_Rle_1(Pw_EquipmentNo4, -((Pw_Driver4_Pluse_HW << 16) + Pw_Driver4_Pluse), Pw_Driver4_Speed, Pw_Driver4_AccTime, 0);
					Run_Motor_S(Pw_EquipmentNo4, M4_UNCLOCKWISE, (Pw_Driver4_Pluse_HW << 16) + Pw_Driver4_Pluse, Pw_Motor4_StartSpeed, Pw_Driver4_Speed, Pw_Driver4_AccTime);
					Pw_Driver4_R_Enable = 0;
					Driver4_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xF7;
				}

				//5#
				if (Pw_Driver5_Enable == 1 && Pw_Driver5_R_Enable == 0)
				{
					HAL_Delay(100);
					// Locate_Rle_1(Pw_EquipmentNo5, (Pw_Driver5_Pluse_HW << 16) + Pw_Driver5_Pluse, Pw_Driver5_Speed, Pw_Driver5_AccTime, 0);
					Run_Motor_S(Pw_EquipmentNo5, M5_CLOCKWISE, (Pw_Driver5_Pluse_HW << 16) + Pw_Driver5_Pluse, Pw_Motor5_StartSpeed, Pw_Driver5_Speed, Pw_Driver5_AccTime);
					Pw_Driver5_Enable = 0;
					Driver5_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xEF;
				}
				else if (Pw_Driver5_Enable == 0 && Pw_Driver5_R_Enable == 1)
				{
					HAL_Delay(100);
					// Locate_Rle_1(Pw_EquipmentNo5, -((Pw_Driver5_Pluse_HW << 16) + Pw_Driver5_Pluse), Pw_Driver5_Speed, Pw_Driver5_AccTime, 0);
					Run_Motor_S(Pw_EquipmentNo5, M5_UNCLOCKWISE, (Pw_Driver5_Pluse_HW << 16) + Pw_Driver5_Pluse, Pw_Motor5_StartSpeed, Pw_Driver5_Speed, Pw_Driver5_AccTime);
					Pw_Driver5_R_Enable = 0;
					Driver5_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xEF;
				}

				//6#
				if (Pw_Driver6_Enable == 1 && Pw_Driver6_R_Enable == 0)
				{
					HAL_Delay(100);
					// Locate_Rle_1(Pw_EquipmentNo6, (Pw_Driver6_Pluse_HW << 16) + Pw_Driver6_Pluse, Pw_Driver6_Speed, Pw_Driver6_AccTime, 0);
					Run_Motor_S(Pw_EquipmentNo6, M6_CLOCKWISE, (Pw_Driver6_Pluse_HW << 16) + Pw_Driver6_Pluse, Pw_Motor6_StartSpeed, Pw_Driver6_Speed, Pw_Driver6_AccTime);
					Pw_Driver6_Enable = 0;
					Driver6_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xDF;
				}
				else if (Pw_Driver6_Enable == 0 && Pw_Driver6_R_Enable == 1)
				{
					HAL_Delay(100);
					// Locate_Rle_1(Pw_EquipmentNo6, -((Pw_Driver6_Pluse_HW << 16) + Pw_Driver6_Pluse), Pw_Driver6_Speed, Pw_Driver6_AccTime, 0);
					Run_Motor_S(Pw_EquipmentNo6, M6_UNCLOCKWISE, (Pw_Driver6_Pluse_HW << 16) + Pw_Driver6_Pluse, Pw_Motor6_StartSpeed, Pw_Driver6_Speed, Pw_Driver6_AccTime);
					Pw_Driver6_R_Enable = 0;
					Driver6_Pos_Start_Sort = 1;
					Pr_Send_Data_F &= 0xDF;
				}
			}
		}
	}
}

//发送位置数据
void Send_Pos_Data(void)
{
	if (Pw_TouchRunStop == 0 && F_Sync_6_axis == 1)
	{
		//本条位置指令只填充一次，=0才可以填充，填充完马上变为1
		if (Driver1_Pos_Start_Sort == 0)
		{
			Fill_Pos_Data(Pw_EquipmentNo1, 0, 0, 0, arr_p1[1]); //通过通讯发出运行指令
			Driver1_Pos_Start_Sort = 1;							//=1，表示已经写入到命令缓冲区
			Pr_Send_Data_F &= 0xFE;
		}

		if (Driver2_Pos_Start_Sort == 0)
		{
			Fill_Pos_Data(Pw_EquipmentNo2, 0, 0, 0, arr_p1[1]); //通过通讯发出运行指令
			Driver2_Pos_Start_Sort = 1;
			Pr_Send_Data_F &= 0xFD;
		}

		if (Driver3_Pos_Start_Sort == 0)
		{
			Fill_Pos_Data(Pw_EquipmentNo3, 0, 0, 0, arr_p1[1]); //通过通讯发出运行指令
			Driver3_Pos_Start_Sort = 1;
			Pr_Send_Data_F &= 0xFB;
		}

		if (Driver4_Pos_Start_Sort == 0)
		{
			Fill_Pos_Data(Pw_EquipmentNo4, 0, 0, 0, arr_p1[1]); //通过通讯发出运行指令
			Driver4_Pos_Start_Sort = 1;
			Pr_Send_Data_F &= 0xF7;
		}

		if (Driver5_Pos_Start_Sort == 0)
		{
			Fill_Pos_Data(Pw_EquipmentNo5, 0, 0, 0, arr_p1[1]); //通过通讯发出运行指令
			Driver5_Pos_Start_Sort = 1;
			Pr_Send_Data_F &= 0xEF;
		}

		if (Driver6_Pos_Start_Sort == 0)
		{
			Fill_Pos_Data(Pw_EquipmentNo6, 0, 0, 0, arr_p1[1]); //通过通讯发出运行指令
			Driver6_Pos_Start_Sort = 1;
			Pr_Send_Data_F &= 0xDF;
		}
	}
}

//有条件复位电机到初始位置
void Reset_Drivers(void)
{
	if (Pw_ResetCMD) //如果有复位命令
	{
		if (Pw_StepAutoMode == 1 && Pr_AllRun == 0) //如果是手动模式，并且是所有电机都停止
		{
			Reset_Routine();
			Pw_ResetCMD = 0;
		}
	}
}

//无条件复位子程序
void Reset_Routine(void)
{
	float num1, num2, num3, num4, num5, num6;
	float tmp_num3, tmp_num4, tmp_num5, tmp_num6;
	s16 temp_MUTI3, temp_MUTI4, temp_MUTI5, temp_MUTI6;
	signed long long temp_SINGLE1, temp_SINGLE2, temp_SINGLE3, temp_SINGLE4, temp_SINGLE5, temp_SINGLE6;
	u8 dir1, dir2, dir3, dir4, dir5, dir6;
	s32 pulse_num1, pulse_num2, pulse_num3, pulse_num4, pulse_num5, pulse_num6;
	u32 pulse_num1_P, pulse_num2_P, pulse_num3_P, pulse_num4_P, pulse_num5_P, pulse_num6_P;

	Pr_BRAKE_Control = 0; //开刹车
	STOP_BRAKE_SYSTEM;	  //允许运行
	Clear_Cmd_Queue();	  //清命令缓冲区

	F_Resetting = 1; //置正在复位标志
	Pr_Send_Data_F = 0;

	arr_p1 = &w_ParLst_Pos_CMD; //执行第一条指令
	arrp_p1_Last = arr_p1;
	Pr_Driver_Running_No = 0;  //当前运行指令号=0
	Pr_Driver_Previous_No = 0; //前一个执行指令号

#if (POS_TYPE == 0) //=0表示位置数据类型为编码器数据
	//计算1#脉冲
	//		temp_MUTI=Pr_Drive1_MultiData_Init-Pr_Drive1_MultiData;
	temp_SINGLE1 = ((Pr_Drive1_singleData_Init_HW << 16) + Pr_Drive1_singleData_Init) - ((Pr_Drive1_singleData_HW << 16) + Pr_Drive1_singleData);
	//		num=temp_MUTI*PULSE_NUM+temp_SINGLE*PULSE_NUM/ELEC_GEAR;
	num1 = (float)temp_SINGLE1;
	num1 *= PULSE_NUM;
	num1 /= ELEC_GEAR_US200;

	if (num1 > 0.0f)
		pulse_num1 = (s32)(num1 + 0.5f);
	else
		pulse_num1 = (s32)(num1 - 0.5f);

	//计算2#脉冲
	//		temp_MUTI=Pr_Drive2_MultiData_Init-Pr_Drive2_MultiData;
	temp_SINGLE2 = ((Pr_Drive2_singleData_Init_HW << 16) + Pr_Drive2_singleData_Init) - ((Pr_Drive2_singleData_HW << 16) + Pr_Drive2_singleData);
	//		num=temp_MUTI*PULSE_NUM+temp_SINGLE*PULSE_NUM/ELEC_GEAR;
	num2 = (float)temp_SINGLE2;
	num2 *= PULSE_NUM;
	num2 /= ELEC_GEAR_US200;

	if (num2 > 0.0f)
		pulse_num2 = (s32)(num2 + 0.5f);
	else
		pulse_num2 = (s32)(num2 - 0.5f);

	//计算3#脉冲
	temp_MUTI3 = Pr_Drive3_MultiData_Init - Pr_Drive3_MultiData;
	temp_SINGLE3 = ((Pr_Drive3_singleData_Init_HW << 16) + Pr_Drive3_singleData_Init) - ((Pr_Drive3_singleData_HW << 16) + Pr_Drive3_singleData);
	// num3 = temp_MUTI3 * PULSE_NUM + temp_SINGLE3 * PULSE_NUM / ELEC_GEAR;
	num3 = (float)temp_MUTI3;
	num3 *= PULSE_NUM;
	tmp_num3 = (float)temp_SINGLE3;
	tmp_num3 *= PULSE_NUM;
	num3 = (float)(num3 + tmp_num3 / ELEC_GEAR);

	if (num3 > 0.0f)
		pulse_num3 = (s32)(num3 + 0.5f);
	else
		pulse_num3 = (s32)(num3 - 0.5f);

	//计算4#脉冲
	temp_MUTI4 = Pr_Drive4_MultiData_Init - Pr_Drive4_MultiData;
	temp_SINGLE4 = ((Pr_Drive4_singleData_Init_HW << 16) + Pr_Drive4_singleData_Init) - ((Pr_Drive4_singleData_HW << 16) + Pr_Drive4_singleData);
	// num4 = temp_MUTI4 * PULSE_NUM + temp_SINGLE4 * PULSE_NUM / ELEC_GEAR;
	num4 = (float)temp_MUTI4;
	num4 *= PULSE_NUM;
	tmp_num4 = (float)temp_SINGLE4;
	tmp_num4 *= PULSE_NUM;
	num4 = (float)(num4 + tmp_num4 / ELEC_GEAR);

	if (num4 > 0.0f)
		pulse_num4 = (s32)(num4 + 0.5f);
	else
		pulse_num4 = (s32)(num4 - 0.5f);

	//计算5#脉冲
	temp_MUTI5 = Pr_Drive5_MultiData_Init - Pr_Drive5_MultiData;
	temp_SINGLE5 = ((Pr_Drive5_singleData_Init_HW << 16) + Pr_Drive5_singleData_Init) - ((Pr_Drive5_singleData_HW << 16) + Pr_Drive5_singleData);
	// num5 = temp_MUTI5 * PULSE_NUM + temp_SINGLE5 * PULSE_NUM / ELEC_GEAR;
	num5 = (float)temp_MUTI5;
	num5 *= PULSE_NUM;
	tmp_num5 = (float)temp_SINGLE5;
	tmp_num5 *= PULSE_NUM;
	num5 = (float)(num5 + tmp_num5 / ELEC_GEAR);

	if (num5 > 0.0f)
		pulse_num5 = (s32)(num5 + 0.5f);
	else
		pulse_num5 = (s32)(num4 - 0.5f);

	//计算6#脉冲
	temp_MUTI6 = Pr_Drive6_MultiData_Init - Pr_Drive6_MultiData;
	temp_SINGLE6 = ((Pr_Drive6_singleData_Init_HW << 16) + Pr_Drive6_singleData_Init) - ((Pr_Drive6_singleData_HW << 16) + Pr_Drive6_singleData);
	// num6 = temp_MUTI6 * PULSE_NUM + temp_SINGLE6 * PULSE_NUM / ELEC_GEAR;
	num6 = (float)temp_MUTI6;
	num6 *= PULSE_NUM;
	tmp_num6 = (float)temp_SINGLE6;
	tmp_num6 *= PULSE_NUM;
	num6 = (float)(num6 + tmp_num6 / ELEC_GEAR);

	if (num6 > 0.0f)
		pulse_num6 = (s32)(num6 + 0.5f);
	else
		pulse_num6 = (s32)(num6 - 0.5f);

#else //=0表示位置数据类型为编码器数据;=1表示位置数据类型为脉冲值
	pulse_num1 = ((Pr_Drive1_singleData_Init_HW << 16) + Pr_Drive1_singleData_Init) - ((Pr_Drive1_singleData_HW << 16) + Pr_Drive1_singleData);
	pulse_num2 = ((Pr_Drive2_singleData_Init_HW << 16) + Pr_Drive2_singleData_Init) - ((Pr_Drive2_singleData_HW << 16) + Pr_Drive2_singleData);
	pulse_num3 = ((Pr_Drive3_singleData_Init_HW << 16) + Pr_Drive3_singleData_Init) - ((Pr_Drive3_singleData_HW << 16) + Pr_Drive3_singleData);
	pulse_num4 = ((Pr_Drive4_singleData_Init_HW << 16) + Pr_Drive4_singleData_Init) - ((Pr_Drive4_singleData_HW << 16) + Pr_Drive4_singleData);
	pulse_num5 = ((Pr_Drive5_singleData_Init_HW << 16) + Pr_Drive5_singleData_Init) - ((Pr_Drive5_singleData_HW << 16) + Pr_Drive5_singleData);
	pulse_num6 = ((Pr_Drive6_singleData_Init_HW << 16) + Pr_Drive6_singleData_Init) - ((Pr_Drive6_singleData_HW << 16) + Pr_Drive6_singleData);
#endif

	if (pulse_num1 >= 0)
	{
		dir1 = M1_CLOCKWISE;
		pulse_num1_P = pulse_num1;
	}
	else
	{
		dir1 = M1_UNCLOCKWISE;
		pulse_num1_P = -pulse_num1;
	}

	if (pulse_num2 >= 0)
	{
		dir2 = M2_CLOCKWISE;
		pulse_num2_P = pulse_num2;
	}
	else
	{
		dir2 = M2_UNCLOCKWISE;
		pulse_num2_P = -pulse_num2;
	}

	if (pulse_num3 >= 0)
	{
		dir3 = M3_CLOCKWISE;
		pulse_num3_P = pulse_num3;
	}
	else
	{
		dir3 = M3_UNCLOCKWISE;
		pulse_num3_P = pulse_num3;
	}

	if (pulse_num4 >= 0)
	{
		dir4 = M4_CLOCKWISE;
		pulse_num4_P = pulse_num4;
	}
	else
	{
		dir4 = M4_UNCLOCKWISE;
		pulse_num4_P = -pulse_num4;
	}

	if (pulse_num5 >= 0)
	{
		dir5 = M5_CLOCKWISE;
		pulse_num5_P = pulse_num5;
	}
	else
	{
		dir5 = M5_UNCLOCKWISE;
		pulse_num5_P = -pulse_num5;
	}

	if (pulse_num6 >= 0)
	{
		dir6 = M6_CLOCKWISE;
		pulse_num6_P = pulse_num6;
	}
	else
	{
		dir6 = M6_UNCLOCKWISE;
		pulse_num6_P = -pulse_num6;
	}
	//停止后才能继续发送
	if (motor1.running == 0 && motor2.running == 0 && motor3.running == 0 &&
		motor4.running == 0 && motor5.running == 0 && motor6.running == 0)
	{
		HAL_Delay(100);

		Run_Motors_sync(dir1, pulse_num1_P, Pw_Motor1_StartSpeed, Pw_Motor1_SetSpeed, Pw_Motor1_ACCSpeed,
						dir2, pulse_num2_P, Pw_Motor2_StartSpeed, Pw_Motor2_SetSpeed, Pw_Motor2_ACCSpeed,
						dir3, pulse_num3_P, Pw_Motor3_StartSpeed, Pw_Motor3_SetSpeed, Pw_Motor3_ACCSpeed,
						dir4, pulse_num4_P, Pw_Motor4_StartSpeed, Pw_Motor4_SetSpeed, Pw_Motor4_ACCSpeed,
						dir5, pulse_num5_P, Pw_Motor5_StartSpeed, Pw_Motor5_SetSpeed, Pw_Motor5_ACCSpeed,
						dir6, pulse_num6_P, Pw_Motor6_StartSpeed, Pw_Motor6_SetSpeed, Pw_Motor6_ACCSpeed);
	}
}

//判断复位到原点子程序
void is_Reseted(void)
{
	u32 temp_PosError_Set;
	signed long long num1, num2, num3, num4, num5, num6;
	s16 temp_MUTI;
	signed long long temp_SINGLE;

#if (POS_TYPE == 0)
	//位置偏差
	temp_PosError_Set = (Pw_PosError_Set_HW << 16) + Pw_PosError_Set;

	num1 = ((Pr_Drive1_singleData_Init_HW << 16) + Pr_Drive1_singleData_Init) - ((Pr_Drive1_singleData_HW << 16) + Pr_Drive1_singleData);

	num2 = (float)((Pr_Drive2_singleData_Init_HW << 16) + Pr_Drive2_singleData_Init) - ((Pr_Drive2_singleData_HW << 16) + Pr_Drive2_singleData);

	temp_MUTI = Pr_Drive3_MultiData_Init - Pr_Drive3_MultiData;
	temp_SINGLE = ((Pr_Drive3_singleData_Init_HW << 16) + Pr_Drive3_singleData_Init) - ((Pr_Drive3_singleData_HW << 16) + Pr_Drive3_singleData);
	num3 = temp_MUTI * ELEC_GEAR;
	num3 += temp_SINGLE;

	temp_MUTI = Pr_Drive4_MultiData_Init - Pr_Drive4_MultiData;
	temp_SINGLE = ((Pr_Drive4_singleData_Init_HW << 16) + Pr_Drive4_singleData_Init) - ((Pr_Drive4_singleData_HW << 16) + Pr_Drive4_singleData);
	num4 = temp_MUTI * ELEC_GEAR;
	num4 += temp_SINGLE;

	temp_MUTI = Pr_Drive5_MultiData_Init - Pr_Drive5_MultiData;
	temp_SINGLE = ((Pr_Drive5_singleData_Init_HW << 16) + Pr_Drive5_singleData_Init) - ((Pr_Drive5_singleData_HW << 16) + Pr_Drive5_singleData);
	num5 = temp_MUTI * ELEC_GEAR;
	num5 += temp_SINGLE;

	temp_MUTI = Pr_Drive6_MultiData_Init - Pr_Drive6_MultiData;
	temp_SINGLE = ((Pr_Drive6_singleData_Init_HW << 16) + Pr_Drive6_singleData_Init) - ((Pr_Drive6_singleData_HW << 16) + Pr_Drive6_singleData);
	num6 = temp_MUTI * ELEC_GEAR;
	num6 += temp_SINGLE;

	if (abs(num1) < temp_PosError_Set && abs(num2) < temp_PosError_Set && abs(num3) < temp_PosError_Set && abs(num4) < temp_PosError_Set && abs(num5) < temp_PosError_Set && abs(num6) < temp_PosError_Set)
		F_Reseted = 1; //=1，位置接近，并且已经停机，则置复位到原点标志
	else
		F_Reseted = 0;
#else
	if (K_StartPoint)
		F_Reseted = 1; //=1，位置接近，并且已经停机，则置复位到原点标志
	else
		F_Reseted = 0;
#endif
}

//发送停机命令子程序
void Send_StopCMD(void)
{
	Stop_Driver(Pw_EquipmentNo1);
	Stop_Driver(Pw_EquipmentNo2);
	Stop_Driver(Pw_EquipmentNo3);
	Stop_Driver(Pw_EquipmentNo4);
	Stop_Driver(Pw_EquipmentNo5);
	Stop_Driver(Pw_EquipmentNo6);
}

//写驱动器参数函数
//输入：驱动器号driver_no，命令号CMD_No，参数地址Par_addr，参数内容Par_content
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
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 1] = 6;						   //指令长度6
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 2] = driver_no;				   //电机地址
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 3] = Modbus_CMD_No;			   //Modbus功能号03或06
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = (Par_addr & 0xFF00) >> 8; //US200或RA1伺服都可以
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = Par_addr & 0x00FF;
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 6] = (Par_content & 0xFF00) >> 8; //写1个字
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 7] = Par_content & 0x00FF;
		}
	}
}

//正常运行，赋值，发PWM波
void Normal_Run(void)
{
	//判断是否要延时
	if (arr_p1[5] > 0 && T_Driver1_delay != SClk10Ms && Driver1_delay_F == 0)
	{
		if (T_Driver1_delay != SClk10Ms)
		{
			T_Driver1_delay = SClk10Ms; //
			C_Driver1_delayCount++;
			Pr_pausetime_show = C_Driver1_delayCount;
			if (C_Driver1_delayCount > arr_p1[5]) //延时完毕，置标志位
			{
				C_Driver1_delayCount = 0;
				Pr_pausetime_show = 0;
				Driver1_delay_F = 1; //延时结束标志位
			}
		}
	}
	else if (((arr_p1[5]) == 0) || (((arr_p1[5]) > 0) && (Driver1_delay_F == 1))) //不延时或者延时结束
	{
		T_Driver1_delay = 1000;
		C_Driver1_delayCount = 0;

		//运行圈数设定
		if (Pr_RUN_Count_Set == 0 || (Pr_RUN_Count_Set > 0 && Pr_RUN_Count < Pr_RUN_Count_Set))
		{
			if ((arr_p1[6]) == 0) //停止标志位=0，表示正常执行；否则停止运行
			{
				if ((arr_p1[7] & DI2) == 0 && (arr_p1[8] & DI3) == 0) //条件1=1，并且DI2有信号，则不运行；条件2=1，并且DI3有信号，则不运行；否则，运行
				{
					//停止后才能继续发送
					if (motor1.running == 0 && motor2.running == 0 && motor3.running == 0 &&
						motor4.running == 0 && motor5.running == 0 && motor6.running == 0)
					{
						Fill_Cmd();
						Driver1_delay_F = 0;
						C_DO_delayCount = 0;
						C_DO_Open_delayCount = 0;
						F_Close_Hand = 0;						  //清标志位，允许电磁阀打开
						Pw_EquipStatus = Pw_EquipStatus & 0xFDFF; //清标志位，位置写入错误停机
					}
					else
					{
						Pw_EquipStatus = Pw_EquipStatus | 0x0200; //=512，位置写入错误停机
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
					Pw_EquipStatus = Pw_EquipStatus | 0x0040; //条件1停机
				}
				else if ((arr_p1[8] & DI3) != 0)
				{
					Pw_EquipStatus = Pw_EquipStatus | 0x0080; //条件2停机
				}
			}
		}
		else
		{
			Pw_EquipStatus = Pw_EquipStatus | 0x0100; //停机命令停机
		}
	}
}

//填充命令
void Fill_Cmd(void)
{
	u32 temp_count;
	u8 dir1, dir2, dir3, dir4, dir5, dir6;
	s32 pulse_num1, pulse_num2, pulse_num3, pulse_num4, pulse_num5, pulse_num6;
	u32 pulse_num1_P, pulse_num2_P, pulse_num3_P, pulse_num4_P, pulse_num5_P, pulse_num6_P;

	if (arr_p1[1] >= 1 && arr_p1[1] <= COM_CMD_NUM - 1) //第1个字节为命令号，在1-29之间才执行，=0则不执行
	{
		if (arr_p1[2] >= 1 && arr_p1[2] <= POS_NUM) //第2个字节为位置号，在1-15之间才执行，=0则不执行
		{
			//停止后才能继续发送
			if (motor1.running == 0 && motor2.running == 0 && motor3.running == 0 &&
				motor4.running == 0 && motor5.running == 0 && motor6.running == 0)
			{
				Pr_Driver_Previous_No = Pr_Driver_Running_No; //前一个执行指令号
				Pr_Driver_Running_No = arr_p1[1];			  //某条指令正在执行，也即表示上一条指令执行完毕
				arrp_p1_Last = arr_p1;						  //保存上一条指令的命令指针

				pulse_num1 = (arr_p1[CMD_PULSE1_HIGH] << 16) + arr_p1[CMD_PULSE1_LOW];
				pulse_num2 = (arr_p1[CMD_PULSE2_HIGH] << 16) + arr_p1[CMD_PULSE2_LOW];
				pulse_num3 = (arr_p1[CMD_PULSE3_HIGH] << 16) + arr_p1[CMD_PULSE3_LOW];
				pulse_num4 = (arr_p1[CMD_PULSE4_HIGH] << 16) + arr_p1[CMD_PULSE4_LOW];
				pulse_num5 = (arr_p1[CMD_PULSE5_HIGH] << 16) + arr_p1[CMD_PULSE5_LOW];
				pulse_num6 = (arr_p1[CMD_PULSE6_HIGH] << 16) + arr_p1[CMD_PULSE6_LOW];
				if (pulse_num1 >= 0)
				{
					dir1 = M1_CLOCKWISE;
					pulse_num1_P = pulse_num1;
				}
				else
				{
					dir1 = M1_UNCLOCKWISE;
					pulse_num1_P = -pulse_num1;
				}
				if (pulse_num2 >= 0)
				{
					dir2 = M2_CLOCKWISE;
					pulse_num2_P = pulse_num2;
				}
				else
				{
					dir2 = M2_UNCLOCKWISE;
					pulse_num2_P = -pulse_num2;
				}

				if (pulse_num3 >= 0)
				{
					dir3 = M3_CLOCKWISE;
					pulse_num3_P = pulse_num3;
				}
				else
				{
					dir3 = M3_UNCLOCKWISE;
					pulse_num3_P = pulse_num3;
				}
				if (pulse_num4 >= 0)
				{
					dir4 = M4_CLOCKWISE;
					pulse_num4_P = pulse_num4;
				}
				else
				{
					dir4 = M4_UNCLOCKWISE;
					pulse_num4_P = -pulse_num4;
				}
				if (pulse_num5 >= 0)
				{
					dir5 = M5_CLOCKWISE;
					pulse_num5_P = pulse_num5;
				}
				else
				{
					dir5 = M5_UNCLOCKWISE;
					pulse_num5_P = -pulse_num5;
				}
				if (pulse_num6 >= 0)
				{
					dir6 = M6_CLOCKWISE;
					pulse_num6_P = pulse_num6;
				}
				else
				{
					dir6 = M6_UNCLOCKWISE;
					pulse_num6_P = -pulse_num6;
				}

				// HAL_Delay(100);

				Run_Motors_sync(dir1, pulse_num1_P, Pw_Motor1_StartSpeed, arr_p1[CMD_SPEED1], arr_p1[CMD_ACC_SPEED1],
								dir2, pulse_num2_P, Pw_Motor2_StartSpeed, arr_p1[CMD_SPEED2], arr_p1[CMD_ACC_SPEED2],
								dir3, pulse_num3_P, Pw_Motor3_StartSpeed, arr_p1[CMD_SPEED3], arr_p1[CMD_ACC_SPEED3],
								dir4, pulse_num4_P, Pw_Motor4_StartSpeed, arr_p1[CMD_SPEED4], arr_p1[CMD_ACC_SPEED4],
								dir5, pulse_num5_P, Pw_Motor5_StartSpeed, arr_p1[CMD_SPEED5], arr_p1[CMD_ACC_SPEED5],
								dir6, pulse_num6_P, Pw_Motor6_StartSpeed, arr_p1[CMD_SPEED6], arr_p1[CMD_ACC_SPEED6]);

				Pw_Current_Run_Time = arr_p1[CMD_RUN_TIME]; //运行时间

				Pw_EquipStatus = Pw_EquipStatus & 0xFFDF; //清最大位置限制停机状态

				Driver1_Pos_Start_Sort = 3;
				Driver2_Pos_Start_Sort = 3;
				Driver3_Pos_Start_Sort = 3;
				Driver4_Pos_Start_Sort = 3;
				Driver5_Pos_Start_Sort = 3;
				Driver6_Pos_Start_Sort = 3;

				//指向下一条要执行的指令
				if ((arr_p1[POS_CMD_SIZE + 1]) != 0) //下一条要执行的指令
					arr_p1 += POS_CMD_SIZE;
				else
				{
					arr_p1 = &w_ParLst_Pos_CMD;
					Pr_RUN_Count++; //运行圈数+1

					temp_count = (Pw_Total_RUN_Count_HW << 16) + Pw_Total_RUN_Count;
					temp_count++; //累计运行圈数+1
					Pw_Total_RUN_Count = temp_count & 0x0000FFFF;
					Pw_Total_RUN_Count_HW = (temp_count & 0xFFFF0000) >> 16;
				}

				Pr_Send_Data_F = 0; //发送标志清0，可以发送位置
			}
		}
	}
}

/********************************************
//相对定位函数 
//num -1073741824~1073741824
//speed: 0~6000rpm
//acc_dec_time: 0~65535 加减速时间
//返回值=1，表示填充正常完成；返回值=0，表示填充错误
//----------位置1---------位置2
//1#、2#----100C----------1011
//3#~6#-----1307----------130C
*********************************************/
u8 Locate_Rle_1(u8 driver_no, s32 num, u16 speed, u16 acc_dec_time, s32 Cmd_No) //相对定位函数
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

		//开始填充
		//第1条：写位置指令
		Com_Write_p[COM_CMD_SIZE * Queue_Rear] = 12;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 1] = 15;		//指令长度15
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 2] = driver_no; //电机地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 3] = 0x10;		//写多个寄存器功能号16
		if (driver_no == 1 || driver_no == 2)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x10; //US200伺服，从P10.12地址开始
			if (*arr_Driver_Write_Sort == 0)
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //第1段位置，P10.12
			else
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x11; //第2段位置，P10.17
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x13; //RA1伺服
			if (*arr_Driver_Write_Sort == 0)
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x07; //第1段位置
			else
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //第2段位置
		}
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 6] = 0x00; //写4个字
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 7] = 0x04;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 8] = 0x08;							//写8个字节
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 9] = (num & 0x0000FF00) >> 8;		//位置高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 10] = (num & 0x000000FF);			//位置低地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 11] = (num & 0xFF000000) >> 24;		//位置高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 12] = (num & 0x00FF0000) >> 16;		//位置低地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 13] = (speed & 0xFF00) >> 8;		//速度高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 14] = (speed & 0xFF);				//速度低地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 15] = (acc_dec_time & 0xFF00) >> 8; //加减速高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 16] = (acc_dec_time & 0xFF);		//加减速低地址

		//当前发送脉冲
		//			tmp_addr=&w_ParLst_Drive[51];								//指向1#伺服电机设定值低字
		tmp_addr_pulse = &Pw_Driver1_SetValue;
		tmp_addr_pulse[(driver_no - 1) * 2] = num & 0x0000FFFF;
		tmp_addr_pulse[(driver_no - 1) * 2 + 1] = (num & 0xFFFF0000) >> 16;

		//当前运行速度
		//			tmp_addr=&w_ParLst_Drive[240];								//指向1#伺服电机运行速度
		tmp_addr_speed = &Pw_Driver1_AutoSpeed;
		tmp_addr_speed[driver_no - 1] = speed;

		tmp_addr_NeverRun = &Pr_Driver1_NeverRun;						 //指向1#伺服电机从没运行标志
		tmp_addr_ComErr = &Pr_Driver1_ComErr;							 //指向1#伺服电机通讯故障标志
		if (tmp_addr_ComErr[driver_no - 1] == 0 && num > 0 && speed > 0) //如果通讯正常，并且脉冲值及速度都大于0
			tmp_addr_NeverRun[driver_no - 1] = 0;						 //则清这台电机的从来没有运行标志

		//第2条指令：位置查询
		if (driver_no == 1 || driver_no == 2) //US200伺服
		{
			Send_Driver_CMD(driver_no, 03, 0x100C, 0x0A);
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Send_Driver_CMD(driver_no, 03, 0x1307, 0x0A);
		}

		//第3条指令：停止
		if (driver_no == 1 || driver_no == 2)
			Send_Driver_CMD(driver_no, 06, 0x3100, 1); //US200伺服，写P31.00=1
		else if (driver_no >= 3 && driver_no <= 6)
			Send_Driver_CMD(driver_no, 06, 0x8910, 1); //RA1伺服，写P8910=1

		//第4条指令：停止状态查询
		if (driver_no == 1 || driver_no == 2)
			Send_Driver_CMD(driver_no, 03, 0x3100, 1); //US200伺服，查P31.00参数状态
		else if (driver_no >= 3 && driver_no <= 6)
			Send_Driver_CMD(driver_no, 03, 0x8910, 1); //RA1伺服，查询P8910参数状态

		//第5条指令：对位置1，直接使能；对位置2，多段运动指令切换
		if (driver_no == 1 || driver_no == 2) //US200伺服
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

		//第6条指令：对位置2，使能多段位置+使能伺服
		if (driver_no == 1 || driver_no == 2) //US200伺服
		{
			if (*arr_Driver_Write_Sort == 1)
				Send_Driver_CMD(driver_no, 06, 0x3100, 0x07);
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			if (*arr_Driver_Write_Sort == 1)
				Send_Driver_CMD(driver_no, 06, 0x8910, 0x07);
		}

		//第7条指令：启动状态查询
		if (driver_no == 1 || driver_no == 2)
			Send_Driver_CMD(driver_no, 03, 0x3100, 1); //US200伺服，查P31.00参数状态
		else if (driver_no >= 3 && driver_no <= 6)
			Send_Driver_CMD(driver_no, 03, 0x8910, 1); //RA1伺服，查询P8910参数状态

		//交替写位置0和位置1
		(*arr_Driver_Write_Sort)++;
		if (*arr_Driver_Write_Sort > 1)
			*arr_Driver_Write_Sort = 0;

		return 1;
	}
	else if (Cmd_No >= 1 && Cmd_No <= COM_CMD_NUM - 1) //默认29条指令
	{
		//当前发送脉冲
		//			tmp_addr=&w_ParLst_Drive[51];						//指向1#伺服电机设定值低字
		tmp_addr_pulse = &Pw_Driver1_SetValue;
		temp_arr_p = &w_ParLst_Pos_CMD;
		tmp_addr_pulse[(driver_no - 1) * 2] = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 12];
		tmp_addr_pulse[(driver_no - 1) * 2 + 1] = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 13];
		tmp_pulse_num = (tmp_addr_pulse[(driver_no - 1) * 2 + 1] << 16) + tmp_addr_pulse[(driver_no - 1) * 2];

		//当前运行速度
		//			tmp_addr=&w_ParLst_Drive[240];						//指向1#伺服电机运行速度
		tmp_addr_speed = &Pw_Driver1_AutoSpeed;
		temp_arr_p = &w_ParLst_Pos_CMD;
		tmp_addr_speed[driver_no - 1] = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 14];
		tmp_run_speed = tmp_addr_speed[driver_no - 1];

		//加减速
		tmp_acc_time = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 15];

		tmp_addr_NeverRun = &Pr_Driver1_NeverRun;										   //指向1#伺服电机从没运行标志
		tmp_addr_ComErr = &Pr_Driver1_ComErr;											   //指向1#伺服电机通讯故障标志
		if (tmp_addr_ComErr[driver_no - 1] == 0 && tmp_pulse_num > 0 && tmp_run_speed > 0) //如果通讯正常，并且脉冲值及速度都大于0
			tmp_addr_NeverRun[driver_no - 1] = 0;										   //则清这台电机的从来没有运行标志

		Queue_Rear = Move_Cmd_Queue(driver_no);
		if (Queue_Rear == 0xFF)
			return 0;

		//开始填充
		//第1条指令：填充位置
		Com_Write_p[COM_CMD_SIZE * Queue_Rear] = 12;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 1] = 15;		//指令长度15
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 2] = driver_no; //电机地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 3] = 0x10;		//写多个寄存器功能号16
		if (driver_no == 1 || driver_no == 2)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x10; //US200伺服，从P10.12地址开始
			if (*arr_Driver_Write_Sort == 0)
			{
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //第1段位置，P10.12
			}
			else
			{
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x11; //第2段位置，P10.17
			}
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x13; //RA1伺服
			if (*arr_Driver_Write_Sort == 0)
			{
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x07; //第1段位置
			}
			else
			{
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //第2段位置
			}
		}

		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 6] = 0x00; //写4个字
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 7] = 0x04;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 8] = 0x08;								  //写8个字节
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 9] = (tmp_pulse_num & 0x0000FF00) >> 8;	  //位置高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 10] = (tmp_pulse_num & 0x000000FF);		  //位置低地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 11] = (tmp_pulse_num & 0xFF000000) >> 24; //位置高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 12] = (tmp_pulse_num & 0x00FF0000) >> 16; //位置低地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 13] = (tmp_run_speed & 0xFF00) >> 8;	  //速度高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 14] = (tmp_run_speed & 0xFF);			  //速度低地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 15] = (tmp_acc_time & 0xFF00) >> 8;		  //加减速高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 16] = (tmp_acc_time & 0xFF);			  //加减速低地址

		//第2条指令：位置查询
		if (driver_no == 1 || driver_no == 2) //US200伺服
		{
			Send_Driver_CMD(driver_no, 03, 0x100C, 0x0A);
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Send_Driver_CMD(driver_no, 03, 0x1307, 0x0A);
		}

		//第3条指令：停止
		if (driver_no == 1 || driver_no == 2)
			Send_Driver_CMD(driver_no, 06, 0x3100, 1); //US200伺服，写P31.00=1
		else if (driver_no >= 3 && driver_no <= 6)
			Send_Driver_CMD(driver_no, 06, 0x8910, 1); //RA1伺服，写P8910=1

		//第4条指令：停止状态查询
		if (driver_no == 1 || driver_no == 2)
			Send_Driver_CMD(driver_no, 03, 0x3100, 1); //US200伺服，查P31.00参数状态
		else if (driver_no >= 3 && driver_no <= 6)
			Send_Driver_CMD(driver_no, 03, 0x8910, 1); //RA1伺服，查询P8910参数状态

		//第5条指令：对位置1，直接使能；对位置2，多段运动指令切换
		if (driver_no == 1 || driver_no == 2) //US200伺服
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

		//第6条指令：对位置2，使能多段位置+使能伺服
		if (driver_no == 1 || driver_no == 2) //US200伺服
		{
			if (*arr_Driver_Write_Sort == 1)
				Send_Driver_CMD(driver_no, 06, 0x3100, 0x07);
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			if (*arr_Driver_Write_Sort == 1)
				Send_Driver_CMD(driver_no, 06, 0x8910, 0x07);
		}

		//第7条指令：启动状态查询
		if (driver_no == 1 || driver_no == 2)
			Send_Driver_CMD(driver_no, 03, 0x3100, 1); //US200伺服，查P31.00参数状态
		else if (driver_no >= 3 && driver_no <= 6)
			Send_Driver_CMD(driver_no, 03, 0x8910, 1); //RA1伺服，查询P8910参数状态

		//交替写位置0和位置1
		(*arr_Driver_Write_Sort)++;
		if (*arr_Driver_Write_Sort > 1)
			*arr_Driver_Write_Sort = 0;

		return 1;
	}

	return 0;
}

/********************************************
//只发送位置数据 
//num -1073741824~1073741824
//speed: 0~6000rpm
//acc_dec_time: 0~65535 加减速时间
//返回值=1，表示填充正常完成；返回值=0，表示填充错误
//----------位置1---------位置2
//1#、2#----100C----------1011
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

		//开始填充
		//第1条：写位置指令
		Com_Write_p[COM_CMD_SIZE * Queue_Rear] = 12;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 1] = 15;		//指令长度15
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 2] = driver_no; //电机地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 3] = 0x10;		//写多个寄存器功能号16

		if (driver_no == 1 || driver_no == 2)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x10; //US200伺服，从P10.12地址开始
			if (*arr_Driver_Write_Sort == 0)
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //第1段位置，P10.12
			else
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x11; //第2段位置，P10.17
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x13; //RA1伺服
			if (*arr_Driver_Write_Sort == 0)
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x07; //第1段位置
			else
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //第2段位置
		}

		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 6] = 0x00; //写4个字
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 7] = 0x04;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 8] = 0x08;							//写8个字节
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 9] = (num & 0x0000FF00) >> 8;		//位置高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 10] = (num & 0x000000FF);			//位置低地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 11] = (num & 0xFF000000) >> 24;		//位置高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 12] = (num & 0x00FF0000) >> 16;		//位置低地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 13] = (speed & 0xFF00) >> 8;		//速度高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 14] = (speed & 0xFF);				//速度低地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 15] = (acc_dec_time & 0xFF00) >> 8; //加减速高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 16] = (acc_dec_time & 0xFF);		//加减速低地址

		//当前发送脉冲
		//			tmp_addr=&w_ParLst_Drive[51];								//指向1#伺服电机设定值低字
		tmp_addr_pulse = &Pw_Driver1_SetValue;
		tmp_addr_pulse[(driver_no - 1) * 2] = num & 0x0000FFFF;
		tmp_addr_pulse[(driver_no - 1) * 2 + 1] = (num & 0xFFFF0000) >> 16;

		//当前运行速度
		//			tmp_addr=&w_ParLst_Drive[240];								//指向1#伺服电机运行速度
		tmp_addr_speed = &Pw_Driver1_AutoSpeed;
		tmp_addr_speed[driver_no - 1] = speed;

		tmp_addr_NeverRun = &Pr_Driver1_NeverRun;						 //指向1#伺服电机从没运行标志
		tmp_addr_ComErr = &Pr_Driver1_ComErr;							 //指向1#伺服电机通讯故障标志
		if (tmp_addr_ComErr[driver_no - 1] == 0 && num > 0 && speed > 0) //如果通讯正常，并且脉冲值及速度都大于0
			tmp_addr_NeverRun[driver_no - 1] = 0;						 //则清这台电机的从来没有运行标志

		//第2条指令：位置查询
		if (driver_no == 1 || driver_no == 2) //US200伺服
		{
			Send_Driver_CMD(driver_no, 03, 0x100C, 0x0A);
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Send_Driver_CMD(driver_no, 03, 0x1307, 0x0A);
		}

		return 1;
	}
	else if (Cmd_No >= 1 && Cmd_No <= COM_CMD_NUM - 1) //默认29条指令
	{
		//当前发送脉冲
		//			tmp_addr=&w_ParLst_Drive[51];						//指向1#伺服电机设定值低字
		tmp_addr_pulse = &Pw_Driver1_SetValue;
		temp_arr_p = &w_ParLst_Pos_CMD;
		tmp_addr_pulse[(driver_no - 1) * 2] = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 12];
		tmp_addr_pulse[(driver_no - 1) * 2 + 1] = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 13];
		tmp_pulse_num = (tmp_addr_pulse[(driver_no - 1) * 2 + 1] << 16) + tmp_addr_pulse[(driver_no - 1) * 2];

		//当前运行速度
		//			tmp_addr=&w_ParLst_Drive[240];						//指向1#伺服电机运行速度
		tmp_addr_speed = &Pw_Driver1_AutoSpeed;
		temp_arr_p = &w_ParLst_Pos_CMD;
		tmp_addr_speed[driver_no - 1] = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 14];
		tmp_run_speed = tmp_addr_speed[driver_no - 1];

		//加减速
		tmp_acc_time = temp_arr_p[(Cmd_No - 1) * POS_CMD_SIZE + (driver_no - 1) * 4 + 15];

		tmp_addr_NeverRun = &Pr_Driver1_NeverRun;										   //指向1#伺服电机从没运行标志
		tmp_addr_ComErr = &Pr_Driver1_ComErr;											   //指向1#伺服电机通讯故障标志
		if (tmp_addr_ComErr[driver_no - 1] == 0 && tmp_pulse_num > 0 && tmp_run_speed > 0) //如果通讯正常，并且脉冲值及速度都大于0
			tmp_addr_NeverRun[driver_no - 1] = 0;										   //则清这台电机的从来没有运行标志

		Queue_Rear = Move_Cmd_Queue(driver_no);
		if (Queue_Rear == 0xFF)
			return 0;

		//开始填充
		Com_Write_p[COM_CMD_SIZE * Queue_Rear] = 12;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 1] = 15;		//指令长度15
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 2] = driver_no; //电机地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 3] = 0x10;		//写多个寄存器功能号16
		if (driver_no == 1 || driver_no == 2)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x10; //US200伺服，从P10.12地址开始
			if (*arr_Driver_Write_Sort == 0)
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //第1段位置，P10.12
			else
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x11; //第2段位置，P10.17
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x13; //RA1伺服
			if (*arr_Driver_Write_Sort == 0)
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x07; //第1段位置
			else
				Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x0C; //第2段位置
		}

		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 6] = 0x00; //写4个字
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 7] = 0x04;
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 8] = 0x08;								  //写8个字节
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 9] = (tmp_pulse_num & 0x0000FF00) >> 8;	  //位置，低字，高字节地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 10] = (tmp_pulse_num & 0x000000FF);		  //位置，低字，低字节地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 11] = (tmp_pulse_num & 0xFF000000) >> 24; //位置，高字，高字节地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 12] = (tmp_pulse_num & 0x00FF0000) >> 16; //位置，高字，低字节地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 13] = (tmp_run_speed & 0xFF00) >> 8;	  //速度高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 14] = (tmp_run_speed & 0xFF);			  //速度低地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 15] = (tmp_acc_time & 0xFF00) >> 8;		  //加减速高地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 16] = (tmp_acc_time & 0xFF);			  //加减速低地址

		//第2条指令：位置查询
		if (driver_no == 1 || driver_no == 2) //US200伺服
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
//发送启动命令函数 
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

	//第1条指令：停止
	if (driver_no == 1 || driver_no == 2)
		Send_Driver_CMD(driver_no, 06, 0x3100, 1); //US200伺服，写P31.00=1
	else if (driver_no >= 3 && driver_no <= 6)
		Send_Driver_CMD(driver_no, 06, 0x8910, 1); //RA1伺服，写P8910=1

	//第2条指令：停止状态查询
	if (driver_no == 1 || driver_no == 2)
		Send_Driver_CMD(driver_no, 03, 0x3100, 1); //US200伺服，查P31.00参数状态
	else if (driver_no >= 3 && driver_no <= 6)
		Send_Driver_CMD(driver_no, 03, 0x8910, 1); //RA1伺服，查询P8910参数状态

	//第3条指令：对位置1，直接使能；对位置2，多段运动指令切换
	if (driver_no == 1 || driver_no == 2) //US200伺服
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

	//第4条指令：对位置2，使能多段位置+使能伺服
	if (driver_no == 1 || driver_no == 2) //US200伺服
	{
		if (*arr_Driver_Write_Sort == 1)
			Send_Driver_CMD(driver_no, 06, 0x3100, 0x07);
	}
	else if (driver_no >= 3 && driver_no <= 6)
	{
		if (*arr_Driver_Write_Sort == 1)
			Send_Driver_CMD(driver_no, 06, 0x8910, 0x07);
	}

	//第5条指令：启动状态查询
	if (driver_no == 1 || driver_no == 2)
		Send_Driver_CMD(driver_no, 03, 0x3100, 1); //US200伺服，查P31.00参数状态
	else if (driver_no >= 3 && driver_no <= 6)
		Send_Driver_CMD(driver_no, 03, 0x8910, 1); //RA1伺服，查询P8910参数状态

	//交替写位置0和位置1
	(*arr_Driver_Write_Sort)++;
	if (*arr_Driver_Write_Sort > 1)
		*arr_Driver_Write_Sort = 0;

	return 1;
}

//命令队列+1
//返回队尾指针表示成功；返回0xFF表示失败
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

//判断COM1命令队列是否为空
u8 Com1_Driver1_Queue_isEmpty(void)
{
	return Com1_Driver1_Queue_Front == -1;
}

u8 Com1_Driver2_Queue_isEmpty(void)
{
	return Com1_Driver2_Queue_Front == -1;
}

//判断COM2命令队列是否为空
u8 Com2_Driver3_Queue_isEmpty(void)
{
	return Com2_Driver3_Queue_Front == -1;
}

u8 Com2_Driver4_Queue_isEmpty(void)
{
	return Com2_Driver4_Queue_Front == -1;
}

//判断COM3命令队列是否为空
u8 Com3_Driver5_Queue_isEmpty(void)
{
	return Com3_Driver5_Queue_Front == -1;
}

u8 Com3_Driver6_Queue_isEmpty(void)
{
	return Com3_Driver6_Queue_Front == -1;
}

////判断COM4命令队列是否为空
//u8 Com4_Queue_isEmpty(void)
//{
//	return Com4_Queue_Front == -1;
//}

//判断COM1命令队列是否为满
u8 Com1_Driver1_Queue_isFull(void)
{
	return (Com1_Driver1_Queue_Front == (Com1_Driver1_Queue_Rear + 1) % COM_CMD_NUM);
}

u8 Com1_Driver2_Queue_isFull(void)
{
	return (Com1_Driver2_Queue_Front == (Com1_Driver2_Queue_Rear + 1) % COM_CMD_NUM);
}

//判断COM2命令队列是否为满
u8 Com2_Driver3_Queue_isFull(void)
{
	return (Com2_Driver3_Queue_Front == (Com2_Driver3_Queue_Rear + 1) % COM_CMD_NUM);
}

u8 Com2_Driver4_Queue_isFull(void)
{
	return (Com2_Driver4_Queue_Front == (Com2_Driver4_Queue_Rear + 1) % COM_CMD_NUM);
}
//判断COM3命令队列是否为满
u8 Com3_Driver5_Queue_isFull(void)
{
	return (Com3_Driver5_Queue_Front == (Com3_Driver5_Queue_Rear + 1) % COM_CMD_NUM);
}

u8 Com3_Driver6_Queue_isFull(void)
{
	return (Com3_Driver6_Queue_Front == (Com3_Driver6_Queue_Rear + 1) % COM_CMD_NUM);
}

////判断COM4命令队列是否为满
//u8 Com4_Queue_isFull(void)
//{
//	return (Com4_Queue_Front == (Com4_Queue_Rear + 1) % COM_CMD_NUM);
//}

//填充停机命令子程序
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
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 1] = 6;			//指令长度6
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 2] = driver_no; //电机地址
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 3] = 0x06;		//写单个寄存器功能号06

		if (driver_no == 1 || driver_no == 2)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x31; //US200伺服，写P31.00=1
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x00;
		}
		else if (driver_no >= 3 && driver_no <= 6)
		{
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 4] = 0x89; //RA1伺服，写P8910=1
			Com_Write_p[COM_CMD_SIZE * Queue_Rear + 5] = 0x10;
		}

		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 6] = 0x00; //写1个字
		Com_Write_p[COM_CMD_SIZE * Queue_Rear + 7] = 0x01;
	}
}

//控制电磁阀(机械手)开通、关断，进行喷漆作业
//功能：1、执行到某条指令时，可以延时再开电磁阀
//2、开通电磁阀后，可以设定持续的时间
//3、如果设定的持续时间太长，则执行到需关闭电磁阀的指令时，马上关闭
void Control_Hand(void)
{
	//判断状态
	if (HAND_STATUS)
		F_Hand_Status = 0; //0电磁阀关闭
	else
		F_Hand_Status = 1; //电磁阀打开

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
					if (C_DO_delayCount > arrp_p1_Last[9]) //延时完毕，置标志位
					{
						C_DO_delayCount = 0;
						T_DO_delay = 1000;
						DO_delay_F = 1; //延时结束标志位
					}
				}
			}
			else if (((arrp_p1_Last[9]) == 0) || (((arrp_p1_Last[9]) > 0) && (DO_delay_F == 1))) //不延时或者延时结束
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

	//DO持续时间
	if (arrp_p1_Last[11] != 0 && F_Hand_Status == 1)
	{
		if (T_DO_Open_delay != SClk10Ms)
		{
			T_DO_Open_delay = SClk10Ms; //
			C_DO_Open_delayCount++;
			if (C_DO_Open_delayCount > arrp_p1_Last[10]) //延时完毕，置标志位
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

//刹车控制
u16 T_BRAKE;
u16 C_BRAKE;

void BRAKE_Control(void)
{
	if (Pr_BRAKE_Control == 0) //=1，刹车；=0，不刹车
	{
		if (T_BRAKE != SClk10Ms)
		{
			T_BRAKE = SClk10Ms;
			C_BRAKE++;
			if (C_BRAKE > 100) //则延时1s后，开刹车，允许运转
			{
				STOP_BRAKE_SYSTEM;
			}
		}
	}
	else
		START_BRAKE_SYSTEM; //刹车制动
}

//清命令队列缓冲区
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

//记录当前位置
void Record_Current_Pos(void)
{
	if (Pw_Define_Save_Pos >= 1 && Pw_Define_Save_Pos <= POS_NUM)
	{
		Pw_Read_CurrentPos = 63; //=63，表示读6个伺服的当前位置。读完后，恢复为0
		Pw_Define_Save_Pos = 0;
	}
}

//计算位置和速度
//40*30=120个双字
//组号(0)+命令号(1)+位置号(2)+运行速度(3)+加减速时间(4)+暂停(5)+停止(6)+条件1(7)+条件2(8)+循环1(9)+循环2(10)+输出(11)
//+1#脉冲_低(12)+1#脉冲_高(13)+1#速度(14)+1#加减速(15)+2#脉冲_低(16)+2#脉冲_高(17)+2#速度(18)+2#加减速(19)
//+3#脉冲_低(20)+3#脉冲_高(21)+3#速度(22)+3#加减速(23)+4#脉冲_低(24)+4#脉冲_高(25)+4#速度(26)+4#加减速(27)
//+5#脉冲_低(28)+5#脉冲_高(29)+5#速度(30)+5#加减速(31)+6#脉冲_低(32)+6#脉冲_高(33)+6#速度(34)+6#加减速(35)
//返回值0为计算成功；1为计算失败
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
	if (temp_arr_Cmd1[2] != 1) //第1行的位置号必须为1
		temp_arr_Cmd1[2] = 1;

	//位置1的数据必须是原点，初始位置，强制赋值
	arr_P1 = &w_ParLst_PosPar;
	arr_P1[1] = Pr_Drive1_MultiData_Init;
	arr_P1[2] = Pr_Drive1_singleData_Init; //初始化位置，默认同时保存到位置1
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

	//填入默认值
	for (k = 0; k < POS_CMD_NUM; k++)
	{
		for (i = 12; i <= 35; i++)
		{
			temp_arr_Cmd1[k * POS_CMD_SIZE + i] = 0; //把所有位置的脉冲、速度等数据填0
		}
	}
	//填入默认值
	for (k = 0; k < POS_CMD_NUM; k++)
	{
		for (i = 14; i <= 35; i = i + 4)
		{
			temp_arr_Cmd1[k * POS_CMD_SIZE + i] = 200;		//因为速度不能写入0，所以默认写200
			temp_arr_Cmd1[k * POS_CMD_SIZE + i + 1] = 1000; //加减速默认写1000
		}
	}

	//计算
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

	//如果头和尾的位置号相等，则判断脉冲和伺服是否==0
	// temp_arr_Cmd1 = &w_ParLst_Pos_CMD;
	// if (temp_arr_Cmd_Last[2] == temp_arr_Cmd1[2])
	// {
	// 	//计算因为四舍五入可能产生的误差
	// 	Cal_Sum_Err(1);
	// 	Cal_Sum_Err(2);
	// 	Cal_Sum_Err(3);
	// 	Cal_Sum_Err(4);
	// 	Cal_Sum_Err(5);
	// 	Cal_Sum_Err(6);
	// }

	return 0;
}

//计算四舍五入带来的误差
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

	//如果头和尾的位置号相等，则判断脉冲和伺服是否==0
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

void sort(s32 *a, int l) //a为数组地址，l为数组长度。
{
	int i, j;
	s32 v;
	//排序主体
	for (i = 0; i < l - 1; i++)
		for (j = i + 1; j < l; j++)
		{
			if (a[i] > a[j]) //如前面的比后面的大，则交换。
			{
				v = a[i];
				a[i] = a[j];
				a[j] = v;
			}
		}
}

void sort_f(float *a, int l) //a为数组地址，l为数组长度。
{
	int i, j;
	float v;
	//排序主体
	for (i = 0; i < l - 1; i++)
		for (j = i + 1; j < l; j++)
		{
			if (a[i] > a[j]) //如前面的比后面的大，则交换。
			{
				v = a[i];
				a[i] = a[j];
				a[j] = v;
			}
		}
}

//计算一个点的脉冲和速度
/*
思路：2、计算出所有电机的脉冲数；
2、先根据每台电机的设定速度计算出每台电机的运行时间；
3、以最长的运行时间为基准，倒推其他电机的运行速度（让其他电机慢一点儿，等等这台电机，使所有电机的运行时间基本相等）
4、以修正后的运行速度，得到每台电机的实际运行时间
5、对实际运行时间排序，得到最长时间，作为本条指令的电机运行时间
*/
void Cal_One_Pos_Speed(s32 Cmd_No1, s32 Cmd_No2)
{
	s32 *temp_arr_Cmd1;
	s32 *temp_arr_Cmd2;
	s32 *arr_P1;
	s32 Pos_No1, Pos_No2;
	float num1, num2, num3, num4, num5, num6;
	float tmp_num3, tmp_num4, tmp_num5, tmp_num6;
	signed long long temp_SINGLE1, temp_SINGLE2, temp_SINGLE3, temp_SINGLE4, temp_SINGLE5, temp_SINGLE6;
	s16 temp_MUTI3, temp_MUTI4, temp_MUTI5, temp_MUTI6;
	u32 Run_Speed_Ratio;
	s32 pulse_num1, pulse_num2, pulse_num3, pulse_num4, pulse_num5, pulse_num6;

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

#if (POS_TYPE == 0)
	//	temp_MUTI1=(arr_P1[Pos_No2*POS_SIZE+1]-arr_P1[Pos_No1*POS_SIZE+1]);
	temp_SINGLE1 = ((arr_P1[Pos_No2 * POS_SIZE + 3] << 16) + arr_P1[Pos_No2 * POS_SIZE + 2] - ((arr_P1[Pos_No1 * POS_SIZE + 3] << 16) + arr_P1[Pos_No1 * POS_SIZE + 2]));
	num1 = (float)temp_SINGLE1;
	num1 *= PULSE_NUM;
	num1 /= ELEC_GEAR_US200;
	if (num1 > 0.0f)
		pulse_num1 = (s32)(num1 + 0.5f);
	else
		pulse_num1 = (s32)(num1 - 0.5f);

	//	temp_MUTI2=(arr_P1[Pos_No2*POS_SIZE+4]-arr_P1[Pos_No1*POS_SIZE+4]);
	temp_SINGLE2 = ((arr_P1[Pos_No2 * POS_SIZE + 6] << 16) + arr_P1[Pos_No2 * POS_SIZE + 5] - ((arr_P1[Pos_No1 * POS_SIZE + 6] << 16) + arr_P1[Pos_No1 * POS_SIZE + 5]));
	num2 = (float)temp_SINGLE2;
	num2 *= PULSE_NUM;
	num2 /= ELEC_GEAR_US200;
	if (num2 > 0.0f)
		pulse_num2 = (s32)(num2 + 0.5f);
	else
		pulse_num2 = (s32)(num2 - 0.5f);

	temp_MUTI3 = (arr_P1[Pos_No2 * POS_SIZE + 7] - arr_P1[Pos_No1 * POS_SIZE + 7]);
	temp_SINGLE3 = ((arr_P1[Pos_No2 * POS_SIZE + 9] << 16) + arr_P1[Pos_No2 * POS_SIZE + 8] - ((arr_P1[Pos_No1 * POS_SIZE + 9] << 16) + arr_P1[Pos_No1 * POS_SIZE + 8]));
	num3 = (float)temp_MUTI3;
	num3 = (float)num3 * PULSE_NUM;
	tmp_num3 = (float)temp_SINGLE3;
	tmp_num3 *= PULSE_NUM;
	num3 = (float)(num3 + tmp_num3 / ELEC_GEAR);

	if (num3 > 0.0f)
		pulse_num3 = (s32)(num3 + 0.5f);
	else
		pulse_num3 = (s32)(num3 - 0.5f);

	temp_MUTI4 = (arr_P1[Pos_No2 * POS_SIZE + 10] - arr_P1[Pos_No1 * POS_SIZE + 10]);
	temp_SINGLE4 = ((arr_P1[Pos_No2 * POS_SIZE + 12] << 16) + arr_P1[Pos_No2 * POS_SIZE + 11] - ((arr_P1[Pos_No1 * POS_SIZE + 12] << 16) + arr_P1[Pos_No1 * POS_SIZE + 11]));
	num4 = (float)temp_MUTI4;
	num4 *= PULSE_NUM;
	tmp_num4 = (float)temp_SINGLE4;
	tmp_num4 *= PULSE_NUM;
	num4 = (float)(num4 + tmp_num4 / ELEC_GEAR);

	if (num4 > 0.0f)
		pulse_num4 = (s32)(num4 + 0.5f);
	else
		pulse_num4 = (s32)(num4 - 0.5f);

	temp_MUTI5 = (arr_P1[Pos_No2 * POS_SIZE + 13] - arr_P1[Pos_No1 * POS_SIZE + 13]);
	temp_SINGLE5 = ((arr_P1[Pos_No2 * POS_SIZE + 15] << 16) + arr_P1[Pos_No2 * POS_SIZE + 14] - ((arr_P1[Pos_No1 * POS_SIZE + 15] << 16) + arr_P1[Pos_No1 * POS_SIZE + 14]));
	num5 = (float)temp_MUTI5;
	num5 *= PULSE_NUM;
	tmp_num5 = (float)temp_SINGLE5;
	tmp_num5 *= PULSE_NUM;
	num5 = (float)(num5 + tmp_num5 / ELEC_GEAR);

	if (num5 > 0.0f)
		pulse_num5 = (s32)(num5 + 0.5f);
	else
		pulse_num5 = (s32)(num5 - 0.5f);

	temp_MUTI6 = (arr_P1[Pos_No2 * POS_SIZE + 16] - arr_P1[Pos_No1 * POS_SIZE + 16]);
	temp_SINGLE6 = ((arr_P1[Pos_No2 * POS_SIZE + 18] << 16) + arr_P1[Pos_No2 * POS_SIZE + 17] - ((arr_P1[Pos_No1 * POS_SIZE + 18] << 16) + arr_P1[Pos_No1 * POS_SIZE + 17]));
	num6 = (float)temp_MUTI6;
	num6 *= PULSE_NUM;
	tmp_num6 = (float)temp_SINGLE6;
	tmp_num6 *= PULSE_NUM;
	num6 = (float)(num6 + tmp_num6 / ELEC_GEAR);

	if (num6 > 0.0f)
		pulse_num6 = (s32)(num6 + 0.5f);
	else
		pulse_num6 = (s32)(num6 - 0.5f);

#else
	pulse_num1 = ((arr_P1[Pos_No2 * POS_SIZE + POS_SINGLE1_HIGH] << 16) + arr_P1[Pos_No2 * POS_SIZE + POS_SINGLE1_LOW] - ((arr_P1[Pos_No1 * POS_SIZE + POS_SINGLE1_HIGH] << 16) + arr_P1[Pos_No1 * POS_SIZE + POS_SINGLE1_LOW]));
	pulse_num2 = ((arr_P1[Pos_No2 * POS_SIZE + POS_SINGLE2_HIGH] << 16) + arr_P1[Pos_No2 * POS_SIZE + POS_SINGLE2_LOW] - ((arr_P1[Pos_No1 * POS_SIZE + POS_SINGLE2_HIGH] << 16) + arr_P1[Pos_No1 * POS_SIZE + POS_SINGLE2_LOW]));
	pulse_num3 = ((arr_P1[Pos_No2 * POS_SIZE + POS_SINGLE3_HIGH] << 16) + arr_P1[Pos_No2 * POS_SIZE + POS_SINGLE3_LOW] - ((arr_P1[Pos_No1 * POS_SIZE + POS_SINGLE3_HIGH] << 16) + arr_P1[Pos_No1 * POS_SIZE + POS_SINGLE3_LOW]));
	pulse_num4 = ((arr_P1[Pos_No2 * POS_SIZE + POS_SINGLE4_HIGH] << 16) + arr_P1[Pos_No2 * POS_SIZE + POS_SINGLE4_LOW] - ((arr_P1[Pos_No1 * POS_SIZE + POS_SINGLE4_HIGH] << 16) + arr_P1[Pos_No1 * POS_SIZE + POS_SINGLE4_LOW]));
	pulse_num5 = ((arr_P1[Pos_No2 * POS_SIZE + POS_SINGLE5_HIGH] << 16) + arr_P1[Pos_No2 * POS_SIZE + POS_SINGLE5_LOW] - ((arr_P1[Pos_No1 * POS_SIZE + POS_SINGLE5_HIGH] << 16) + arr_P1[Pos_No1 * POS_SIZE + POS_SINGLE5_LOW]));
	pulse_num6 = ((arr_P1[Pos_No2 * POS_SIZE + POS_SINGLE6_HIGH] << 16) + arr_P1[Pos_No2 * POS_SIZE + POS_SINGLE6_LOW] - ((arr_P1[Pos_No1 * POS_SIZE + POS_SINGLE6_HIGH] << 16) + arr_P1[Pos_No1 * POS_SIZE + POS_SINGLE6_LOW]));
#endif
	Run_Speed_Ratio = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + CMD_RUN_SPEED_RATIO];

	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 12] = pulse_num1 & 0x0000FFFF;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 13] = (pulse_num1 & 0xFFFF0000) >> 16;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 14] = MOTOR_MAX_SPEED12 * Run_Speed_Ratio / 100;
	// temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 15] = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 4]; //加减速时间直接复制总的

	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 16] = pulse_num2 & 0x0000FFFF;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 17] = (pulse_num2 & 0xFFFF0000) >> 16;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 18] = MOTOR_MAX_SPEED12 * Run_Speed_Ratio / 100;
	// temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 19] = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 4]; //加减速时间直接复制总的

	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 20] = pulse_num3 & 0x0000FFFF;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 21] = (pulse_num3 & 0xFFFF0000) >> 16;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 22] = MOTOR_MAX_SPEED3 * Run_Speed_Ratio / 100;
	// temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 23] = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 4]; //加减速时间直接复制总的

	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 24] = pulse_num4 & 0x0000FFFF;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 25] = (pulse_num4 & 0xFFFF0000) >> 16;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 26] = MOTOR_MAX_SPEED456 * Run_Speed_Ratio / 100;
	// temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 27] = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 4]; //加减速时间直接复制总的

	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 28] = pulse_num5 & 0x0000FFFF;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 29] = (pulse_num5 & 0xFFFF0000) >> 16;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 30] = MOTOR_MAX_SPEED456 * Run_Speed_Ratio / 100;
	// temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 31] = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 4]; //加减速时间直接复制总的

	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 32] = pulse_num6 & 0x0000FFFF;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 33] = (pulse_num6 & 0xFFFF0000) >> 16;
	temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 34] = MOTOR_MAX_SPEED456 * Run_Speed_Ratio / 100;
	// temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 35] = temp_arr_Cmd2[(Cmd_No2 - 1) * POS_CMD_SIZE + 4]; //加减速时间直接复制总的
}

//运行到某一命令号对应的位置点
//输入参数：Cmd_No，命令号
void Run_to_One_Pos(s32 Cmd_No)
{
	float num1, num2, num3, num4, num5, num6;
	float tmp_num3, tmp_num4, tmp_num5, tmp_num6;
	s16 temp_MUTI3, temp_MUTI4, temp_MUTI5, temp_MUTI6;
	s32 *tmp_arr_Cmd;
	s32 *tmp_arr_Pos;
	s32 Pos_No;
	s32 acc_dec_time;
	u8 dir1, dir2, dir3, dir4, dir5, dir6;
	s32 pulse_num1, pulse_num2, pulse_num3, pulse_num4, pulse_num5, pulse_num6;
	u32 pulse_num1_P, pulse_num2_P, pulse_num3_P, pulse_num4_P, pulse_num5_P, pulse_num6_P;
	signed long long temp_SINGLE1, temp_SINGLE2, temp_SINGLE3, temp_SINGLE4, temp_SINGLE5, temp_SINGLE6;

	Pr_Send_Data_F = 0;
	tmp_arr_Cmd = &w_ParLst_Pos_CMD;
	tmp_arr_Pos = &w_ParLst_PosPar;

	if (Cmd_No >= 1 && Cmd_No <= COM_CMD_NUM - 1)
		Cmd_No--;

	Pos_No = tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_POS_NO];
	if (Pos_No >= 1 && Pos_No <= POS_NUM)
		Pos_No--;

	acc_dec_time = tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_ACC_SPEED];

#if (POS_TYPE == 0) //=0表示位置数据类型为编码器数据
	//计算1#脉冲
	//目标位置点编码器数据-当前位置点的实时编码器数据
	//	temp_MUTI1=(tmp_arr_Pos[Pos_No*POS_SIZE+1]-Pr_Drive1_MultiData);
	temp_SINGLE1 = (((tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE1_HIGH] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE1_LOW]) - ((Pr_Drive1_singleData_HW << 16) + Pr_Drive1_singleData));
	num1 = (float)temp_SINGLE1;
	num1 *= PULSE_NUM;
	num1 /= ELEC_GEAR_US200;
	if (num1 > 0.0f)
		pulse_num1 = (s32)(num1 + 0.5f);
	else
		pulse_num1 = (s32)(num1 - 0.5f);

	//计算2#脉冲
	temp_SINGLE2 = (((tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE2_HIGH] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE2_LOW]) - ((Pr_Drive2_singleData_HW << 16) + Pr_Drive2_singleData));
	num2 = (float)temp_SINGLE2;
	num2 *= PULSE_NUM;
	num2 /= ELEC_GEAR_US200;
	if (num2 > 0.0f)
		pulse_num2 = (s32)(num2 + 0.5f);
	else
		pulse_num2 = (s32)(num2 - 0.5f);

	//计算3#脉冲
	temp_MUTI3 = (tmp_arr_Pos[Pos_No * POS_SIZE + POS_MULTI3] - Pr_Drive3_MultiData);
	temp_SINGLE3 = (((tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE3_HIGH] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE3_LOW]) - ((Pr_Drive3_singleData_HW << 16) + Pr_Drive3_singleData));
	// num3 = temp_MUTI3 * PULSE_NUM + temp_SINGLE3 * PULSE_NUM / ELEC_GEAR;
	num3 = (float)temp_MUTI3;
	num3 *= PULSE_NUM;
	tmp_num3 = (float)temp_SINGLE3;
	tmp_num3 *= PULSE_NUM;
	num3 = (float)(num3 + tmp_num3 / ELEC_GEAR);

	if (num3 > 0.0f)
		pulse_num3 = (s32)(num3 + 0.5f);
	else
		pulse_num3 = (s32)(num3 - 0.5f);

	//计算4#脉冲
	temp_MUTI4 = (tmp_arr_Pos[Pos_No * POS_SIZE + POS_MULTI4] - Pr_Drive4_MultiData);
	temp_SINGLE4 = (((tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE4_HIGH] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE4_LOW]) - ((Pr_Drive4_singleData_HW << 16) + Pr_Drive4_singleData));
	// num4 = temp_MUTI4 * PULSE_NUM + temp_SINGLE4 * PULSE_NUM / ELEC_GEAR;
	num4 = (float)temp_MUTI4;
	num4 *= PULSE_NUM;
	tmp_num4 = (float)temp_SINGLE4;
	tmp_num4 *= PULSE_NUM;
	num4 = (float)(num4 + tmp_num4 / ELEC_GEAR);

	if (num4 > 0.0f)
		pulse_num4 = (s32)(num4 + 0.5f);
	else
		pulse_num4 = (s32)(num4 - 0.5f);

	//计算5#脉冲
	temp_MUTI5 = (tmp_arr_Pos[Pos_No * POS_SIZE + POS_MULTI5] - Pr_Drive5_MultiData);
	temp_SINGLE5 = (((tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE5_HIGH] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE5_LOW]) - ((Pr_Drive5_singleData_HW << 16) + Pr_Drive5_singleData));
	// num5 = temp_MUTI5 * PULSE_NUM + temp_SINGLE5 * PULSE_NUM / ELEC_GEAR;
	num5 = (float)temp_MUTI5;
	num5 *= PULSE_NUM;
	tmp_num5 = (float)temp_SINGLE5;
	tmp_num5 *= PULSE_NUM;
	num5 = (float)(num5 + tmp_num5 / ELEC_GEAR);

	if (num5 > 0.0f)
		pulse_num5 = (s32)(num5 + 0.5f);
	else
		pulse_num5 = (s32)(num5 - 0.5f);
	//计算6#脉冲
	temp_MUTI6 = (tmp_arr_Pos[Pos_No * POS_SIZE + POS_MULTI6] - Pr_Drive6_MultiData);
	temp_SINGLE6 = (((tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE6_HIGH] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE6_LOW]) - ((Pr_Drive6_singleData_HW << 16) + Pr_Drive6_singleData));
	// num6 = temp_MUTI6 * PULSE_NUM + temp_SINGLE6 * PULSE_NUM / ELEC_GEAR;
	num6 = (float)temp_MUTI6;
	num6 *= PULSE_NUM;
	tmp_num6 = (float)temp_SINGLE6;
	tmp_num6 *= PULSE_NUM;
	num6 = (float)(num6 + tmp_num6 / ELEC_GEAR);

	if (num6 > 0.0f)
		pulse_num6 = (s32)(num6 + 0.5f);
	else
		pulse_num6 = (s32)(num6 - 0.5f);
#else
	pulse_num1 = (((tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE1_HIGH] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE1_LOW]) - ((Pr_Drive1_singleData_HW << 16) + Pr_Drive1_singleData));
	pulse_num2 = (((tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE2_HIGH] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE2_LOW]) - ((Pr_Drive2_singleData_HW << 16) + Pr_Drive2_singleData));
	pulse_num3 = (((tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE3_HIGH] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE3_LOW]) - ((Pr_Drive3_singleData_HW << 16) + Pr_Drive3_singleData));
	pulse_num4 = (((tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE4_HIGH] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE4_LOW]) - ((Pr_Drive4_singleData_HW << 16) + Pr_Drive4_singleData));
	pulse_num5 = (((tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE5_HIGH] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE5_LOW]) - ((Pr_Drive5_singleData_HW << 16) + Pr_Drive5_singleData));
	pulse_num6 = (((tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE6_HIGH] << 16) + tmp_arr_Pos[Pos_No * POS_SIZE + POS_SINGLE6_LOW]) - ((Pr_Drive6_singleData_HW << 16) + Pr_Drive6_singleData));

#endif

	if (pulse_num1 >= 0)
	{
		dir1 = M1_CLOCKWISE;
		pulse_num1_P = pulse_num1;
	}
	else
	{
		dir1 = M1_UNCLOCKWISE;
		pulse_num1_P = -pulse_num1;
	}

	if (pulse_num2 >= 0)
	{
		dir2 = M2_CLOCKWISE;
		pulse_num2_P = pulse_num2;
	}
	else
	{
		dir2 = M2_UNCLOCKWISE;
		pulse_num2_P = -pulse_num2;
	}

	if (pulse_num3 >= 0)
	{
		dir3 = M3_CLOCKWISE;
		pulse_num3_P = pulse_num3;
	}
	else
	{
		dir3 = M3_UNCLOCKWISE;
		pulse_num3_P = pulse_num3;
	}

	if (pulse_num4 >= 0)
	{
		dir4 = M4_CLOCKWISE;
		pulse_num4_P = pulse_num4;
	}
	else
	{
		dir4 = M4_UNCLOCKWISE;
		pulse_num4_P = -pulse_num4;
	}

	if (pulse_num5 >= 0)
	{
		dir5 = M5_CLOCKWISE;
		pulse_num5_P = pulse_num5;
	}
	else
	{
		dir5 = M5_UNCLOCKWISE;
		pulse_num5_P = -pulse_num5;
	}

	if (pulse_num6 >= 0)
	{
		dir6 = M6_CLOCKWISE;
		pulse_num6_P = pulse_num6;
	}
	else
	{
		dir6 = M6_UNCLOCKWISE;
		pulse_num6_P = -pulse_num6;
	}

	// HAL_Delay(100);
	//停止后才能继续发送
	if (motor1.running == 0 && motor2.running == 0 && motor3.running == 0 &&
		motor4.running == 0 && motor5.running == 0 && motor6.running == 0)
	{

		Run_Motors_sync(dir1, pulse_num1_P, Pw_Motor1_StartSpeed, tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_SPEED1], tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_ACC_SPEED1],
						dir2, pulse_num2_P, Pw_Motor2_StartSpeed, tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_SPEED2], tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_ACC_SPEED2],
						dir3, pulse_num3_P, Pw_Motor3_StartSpeed, tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_SPEED3], tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_ACC_SPEED3],
						dir4, pulse_num4_P, Pw_Motor4_StartSpeed, tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_SPEED4], tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_ACC_SPEED4],
						dir5, pulse_num5_P, Pw_Motor5_StartSpeed, tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_SPEED5], tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_ACC_SPEED5],
						dir6, pulse_num6_P, Pw_Motor6_StartSpeed, tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_SPEED6], tmp_arr_Cmd[Cmd_No * POS_CMD_SIZE + CMD_ACC_SPEED6]);
	}
}

//编码器位置手动调整函数
void Pos_Manual_Adj(void)
{
	if (Pw_TouchRunStop == 1 && Pw_Pos_Adj_Cmd == 1) //触摸停机状态下，才能调整
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

//调整位置
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
			//计算单圈
			tmp_Pos = (tmp_arr_Pos[k * POS_SIZE + (driver_no - 1) * 3 + 3] << 16) + tmp_arr_Pos[k * POS_SIZE + (driver_no - 1) * 3 + 2];
			tmp_Pos += PosErr_Sing;
			if (driver_no > 2)
			{
				tmp_Pos = tmp_Pos & 0x7FFFFF; //取余，对RA1伺服，单圈最大是2^23=8388608
			}
			tmp_arr_Pos[k * POS_SIZE + (driver_no - 1) * 3 + 2] = tmp_Pos & 0x0000FFFF;			//单圈低字
			tmp_arr_Pos[k * POS_SIZE + (driver_no - 1) * 3 + 3] = (tmp_Pos & 0xFFFF0000) >> 16; //单圈高字

			//计算多圈
			tmp_arr_Pos[k * POS_SIZE + (driver_no - 1) * 3 + 1] += PosErr_Muti; //多圈
		}
		else
			break;
	}
}

//限制最大最小位置范围
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
	if (Pw_TouchRunStop == 0 && F_Starting == 0 && Pr_Driver_Previous_No > 0) //如果处于自动状态，且有电机运行时，并且不是启动过程，就判断
	{
		//!=0,有位置限制功能
		Pr_OverMaxPos_F = 0;
		if (((Pw_Limit_MaxPos & 0x01) == 0x01) && Pr_F_Drive1_Runing != 0)
		{
			if (T_Driver1_OverPos != SClk10Ms)
			{
				T_Driver1_OverPos = SClk10Ms; //
				C_Driver1_OverPos++;
				if (C_Driver1_OverPos > 2) //延时判断
				{
					C_Driver1_OverPos = 0;
					T_Driver1_OverPos = 1000;

					if (Judge_OverPos(Pw_EquipmentNo1))
					{
						Pr_OverMaxPos_F = 1; //置超出位置限制标志位
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
				if (C_Driver2_OverPos > 2) //延时判断
				{
					C_Driver2_OverPos = 0;
					T_Driver2_OverPos = 1000;

					if (Judge_OverPos(Pw_EquipmentNo2))
					{
						Pr_OverMaxPos_F = 1; //置超出位置限制标志位
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
				if (C_Driver3_OverPos > 2) //延时判断
				{
					C_Driver3_OverPos = 0;
					T_Driver3_OverPos = 1000;

					if (Judge_OverPos(Pw_EquipmentNo3))
					{
						Pr_OverMaxPos_F = 1; //置超出位置限制标志位
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
				if (C_Driver4_OverPos > 2) //延时判断
				{
					C_Driver4_OverPos = 0;
					T_Driver4_OverPos = 1000;

					if (Judge_OverPos(Pw_EquipmentNo4))
					{
						Pr_OverMaxPos_F = 1; //置超出位置限制标志位
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
				if (C_Driver5_OverPos > 2) //延时判断
				{
					C_Driver5_OverPos = 0;
					T_Driver5_OverPos = 1000;

					if (Judge_OverPos(Pw_EquipmentNo5))
					{
						Pr_OverMaxPos_F = 1; //置超出位置限制标志位
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
				if (C_Driver6_OverPos > 2) //延时判断
				{
					C_Driver6_OverPos = 0;
					T_Driver6_OverPos = 1000;

					if (Judge_OverPos(Pw_EquipmentNo6))
					{
						Pr_OverMaxPos_F = 1; //置超出位置限制标志位
					}
				}
			}
		}

		if (Pr_OverMaxPos_F)
		{
			Clear_Cmd_Queue();	 //清命令缓冲区
			Pw_TouchRunStop = 1; //停止模式
			F_SendStopCMD2 = 0;
			Pr_BRAKE_Control = 1;					  //有故障，刹车
			START_BRAKE_SYSTEM;						  //刹车
			Pw_EquipStatus = Pw_EquipStatus | 0x0020; //=32，超出位置限制停机
		}
		else
		{
			Pr_OverMaxPos_F = 0;					  //清超出位置限制标志位
			Pw_EquipStatus = Pw_EquipStatus & 0xFFDF; //清标志位
		}
	}
}

//判断某台电机是否超出位置范围
//输入：Driver_No
//输出:=1，超出范围；=0，没有超出范围
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

	//得到位置偏差
	temp_PosError_Set = (Pw_PosError_Set_HW << 16) + Pw_PosError_Set;

	//得到命令号
	Cmd_No1 = Pr_Driver_Previous_No;
	Cmd_No2 = arrp_p1_Last[1];

	//得到位置号
	temp_arr_Cmd1 = &w_ParLst_Pos_CMD;
	Pos_No1 = temp_arr_Cmd1[(Cmd_No1 - 1) * POS_CMD_SIZE + 2];
	Pos_No2 = temp_arr_Cmd1[(Cmd_No2 - 1) * POS_CMD_SIZE + 2];

	//指向当前命令号位置
	arr_Pos1 = &w_ParLst_PosPar;
	arr_Pos1 += ((Pos_No1 - 1) * POS_SIZE + (Driver_No - 1) * 3 + 1);

	//指向下一条命令号位置
	arr_Pos2 = &w_ParLst_PosPar;
	arr_Pos2 += ((Pos_No2 - 1) * POS_SIZE + (Driver_No - 1) * 3 + 1);

	//得到当前实时位置
	//先得到当前位置指针
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

	//得到当前位置
	f_Current_Pos = (float)((*P_Real_Muti_Pos) * Tmp_ELEC_Gear + (*(P_Real_Single_Pos + 1) << 16) + (*P_Real_Single_Pos));
	//得到位置1
	f_Set_Pos1 = (float)((*arr_Pos1) * Tmp_ELEC_Gear + (*(arr_Pos1 + 2) << 16) + (*(arr_Pos1 + 1)));
	//得到位置2
	f_Set_Pos2 = (float)((*arr_Pos2) * Tmp_ELEC_Gear + (*(arr_Pos2 + 2) << 16) + (*(arr_Pos2 + 1)));

	if (f_Set_Pos1 > f_Set_Pos2)
	{
		f_Tmp_Pos = f_Set_Pos1;
		f_Set_Pos1 = f_Set_Pos2;
		f_Set_Pos2 = f_Tmp_Pos;
	}

	if (f_Current_Pos < (f_Set_Pos1 - temp_PosError_Set) || f_Current_Pos > (f_Set_Pos2 + temp_PosError_Set))
		return 1; //返回1，超出范围
	else
		return 0; //返回0，没有超出范围
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
