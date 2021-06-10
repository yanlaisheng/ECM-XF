#ifndef __LINKTABLE_H
#define __LINKTABLE_H

// #include "stm32f10x_lib.h"
#include <stdlib.h>
#include "GlobalConst.h"
#include "typedef.h"

typedef struct //定义一个结构体
{
	u16 Cmd_Group_No;	//组号——0
	u16 Cmd_Serial_Num; //命令序号——1（！=0，表示命令序号；=0，表示本条命令及后面的所有命令都跳过，不执行）
	u8 Run_Driver_No;	//要运行的电机号，1-6对应1-6#伺服电机——2
	s32 pulse_num;		//位置脉冲数——3（正负1073741824）
	s32 pulse_num_HW;
	u16 speed;		  //速度——5（0-6000rpm）
	u16 acc_dec_time; //加减速时间——6（0-65535）

	u32 Pause_ms;		//暂停时间——7（=0，表示不暂停；！=0，表示暂停的时间：ms）
	u8 Drive_Stop_flag; //停止运行标志——8，（=1表示停止本电机运行（停止发波）；=0表示正常）

	u16 Drive1_Exe_flag; //执行标志——9（=0，表示正常运行；！=0，表示本条指令必须满足Drive1_Exe_flag==Pr_Driver_Running_No才执行）
	u16 Drive2_Exe_flag; //执行标志——10（=0，表示正常运行；！=0，表示本条指令必须满足Drive2_Exe_flag==Pr_Drive2_Run_No才执行）
	u16 Drive3_Exe_flag; //执行标志——11（=0，表示正常运行；！=0，表示本条指令必须满足Drive3_Exe_flag==Pr_Drive3_Run_No才执行）
	u16 Drive4_Exe_flag; //执行标志——12（=0，表示正常运行；！=0，表示本条指令必须满足Drive4_Exe_flag==Pr_Drive4_Run_No才执行）
	u16 Drive5_Exe_flag; //执行标志——13（=0，表示正常运行；！=0，表示本条指令必须满足Drive5_Exe_flag==Pr_Drive5_Run_No才执行）
	u16 Drive6_Exe_flag; //执行标志——14（=0，表示正常运行；！=0，表示本条指令必须满足Drive6_Exe_flag==Pr_Drive6_Run_No才执行）

	u8 Close_Hand_F; //闭合机械手命令标志——15
	u16 Next_Cmd_No; //下一条要执行的命令号16

	u8 End_Cmd_F;	 //结束命令标志——17
	u16 Run_Cmd_Num; //运行条数——18
	u8 Run_Mode_F;	 //运行模式——19

} drive_cmd; //结构体的别名为drive_cmd，可直接用此别名定义结构体变量。如drive_cmd test;

typedef struct Drivers_cmd
{
	struct Drivers_cmd *nextDrivers_cmd; //用于指向下一个控制命令
	drive_cmd *drive_cmd_info;			 //具体的信息
} Drivers_cmdTypedef;

u8 Cmd_ListInit(void);
u8 AddCmdToList(Drivers_cmdTypedef *head_List, drive_cmd *CmdInfo);
u8 deleteCmdInfo_AccordingNum(int num);

#endif
