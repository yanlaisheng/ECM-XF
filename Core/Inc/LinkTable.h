#ifndef __LINKTABLE_H
#define __LINKTABLE_H

// #include "stm32f10x_lib.h"
#include <stdlib.h>
#include "GlobalConst.h"
#include "typedef.h"

typedef struct //����һ���ṹ��
{
	u16 Cmd_Group_No;	//��š���0
	u16 Cmd_Serial_Num; //������š���1����=0����ʾ������ţ�=0����ʾ���������������������������ִ�У�
	u8 Run_Driver_No;	//Ҫ���еĵ���ţ�1-6��Ӧ1-6#�ŷ��������2
	s32 pulse_num;		//λ������������3������1073741824��
	s32 pulse_num_HW;
	u16 speed;		  //�ٶȡ���5��0-6000rpm��
	u16 acc_dec_time; //�Ӽ���ʱ�䡪��6��0-65535��

	u32 Pause_ms;		//��ͣʱ�䡪��7��=0����ʾ����ͣ����=0����ʾ��ͣ��ʱ�䣺ms��
	u8 Drive_Stop_flag; //ֹͣ���б�־����8����=1��ʾֹͣ��������У�ֹͣ��������=0��ʾ������

	u16 Drive1_Exe_flag; //ִ�б�־����9��=0����ʾ�������У���=0����ʾ����ָ���������Drive1_Exe_flag==Pr_Driver_Running_No��ִ�У�
	u16 Drive2_Exe_flag; //ִ�б�־����10��=0����ʾ�������У���=0����ʾ����ָ���������Drive2_Exe_flag==Pr_Drive2_Run_No��ִ�У�
	u16 Drive3_Exe_flag; //ִ�б�־����11��=0����ʾ�������У���=0����ʾ����ָ���������Drive3_Exe_flag==Pr_Drive3_Run_No��ִ�У�
	u16 Drive4_Exe_flag; //ִ�б�־����12��=0����ʾ�������У���=0����ʾ����ָ���������Drive4_Exe_flag==Pr_Drive4_Run_No��ִ�У�
	u16 Drive5_Exe_flag; //ִ�б�־����13��=0����ʾ�������У���=0����ʾ����ָ���������Drive5_Exe_flag==Pr_Drive5_Run_No��ִ�У�
	u16 Drive6_Exe_flag; //ִ�б�־����14��=0����ʾ�������У���=0����ʾ����ָ���������Drive6_Exe_flag==Pr_Drive6_Run_No��ִ�У�

	u8 Close_Hand_F; //�պϻ�е�������־����15
	u16 Next_Cmd_No; //��һ��Ҫִ�е������16

	u8 End_Cmd_F;	 //���������־����17
	u16 Run_Cmd_Num; //������������18
	u8 Run_Mode_F;	 //����ģʽ����19

} drive_cmd; //�ṹ��ı���Ϊdrive_cmd����ֱ���ô˱�������ṹ���������drive_cmd test;

typedef struct Drivers_cmd
{
	struct Drivers_cmd *nextDrivers_cmd; //����ָ����һ����������
	drive_cmd *drive_cmd_info;			 //�������Ϣ
} Drivers_cmdTypedef;

u8 Cmd_ListInit(void);
u8 AddCmdToList(Drivers_cmdTypedef *head_List, drive_cmd *CmdInfo);
u8 deleteCmdInfo_AccordingNum(int num);

#endif
