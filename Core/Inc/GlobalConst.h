/**
  ******************************************************************************
  * @file    GlobalConst.h
  * @author  ChengLei Zhou  - 周成磊
  * @version V1.27
  * @date    2014-01-03
  * @brief   全局变量声明
	******************************************************************************
	*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __GLOBALCONST_H
#define __GLOBALCONST_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f40x.h"

//设置外部铁电存储器FM25CL16的地址分配
//FM25CL64存储芯片，64Kbit，即8K字节，用于保存设定参数、位置信息及位置运行命令
/*******
 * 0-599:—————————用于保存内部参数w_ParLst[300]，300个字，每个参数2个字节，共600个字节
 * 600-1799:—————————用于保存内部参数w_ParLst_Drive[300]，300个双字，每个参数4个字节，共1200个字节
 * 1800-2999:—————————用于保存位置数据，15条，每条20个双字，共300个双字，1200个字节
 * 3000-7799:—————————用于保存位置数据，30条，每条40个双字，共1200个双字，4800个字节
 */
#define PAR_SIZE_ALL 8192 //参数区总大小
#define PAR1_SIZE 300     // NV参数数量

#define FM_FLASH_PAR1_ADDR 0                                                    //参数区1起始地址
#define FM_FLASH_PAR2_ADDR FM_FLASH_PAR1_ADDR + PAR1_SIZE * sizeof(w_ParLst[0]) //结果=0+300*2=600

#define PAR2_SIZE 300
#define w_ParLst_DrivePar w_ParLst_Drive[0]
//位置参数起始地址
#define FM_POS_ADDR FM_FLASH_PAR2_ADDR + PAR2_SIZE * sizeof(w_ParLst_DrivePar) //结果=600+300*4=1800

#define POS_SIZE 20                      //每个数据点的长度，序号+6个多圈+12个单圈编码器值+标志
#define POS_NUM 15                       //定义数据点的数量，15个
#define FLASH_POS_SIZE POS_SIZE *POS_NUM //20*15
//位置命令起始地址
#define FM_POS_CMD_ADDR FM_POS_ADDR + FLASH_POS_SIZE * sizeof(w_ParLst_DrivePar) //结果=1800+300*4=3000

#define POS_CMD_NUM 30                               //联动位置指令数量
#define POS_CMD_SIZE 40                              //联动位置指令长度
#define FLASH_POS_CMD_SIZE POS_CMD_SIZE *POS_CMD_NUM //最大位置=3000+1200*4=7800<8192

#define COM_CMD_SIZE 20 //命令指令长度
#define COM_CMD_NUM 30  //命令指令数量

#define PULSE_NUM 1000         //电机每旋转一圈的脉冲数
#define ELEC_GEAR 8388608      //电子齿轮2^23=8388608
#define ELEC_GEAR_US200 131072 //电子齿轮2^17=131072

//20*15=300个双字，位置序号+6个多圈+12个单圈编码器值+标志
//位置号(0)
//1#多圈(1)+1#单圈_低(2)+1#单圈_高(3)+2#多圈(4)+2#单圈_低(5)+2#单圈_高(6)+
//3#多圈(7)+3#单圈_低(8)+3#单圈_高(9)+4#多圈(10)+4#单圈_低(11)+4#单圈_高(12)+
//5#多圈(13)+5#单圈_低(14)+5#单圈_高(15)+6#多圈(16)+6#单圈_低(17)+6#单圈_高(18)
#define w_ParLst_PosPar w_ParLst_Drive[PAR2_SIZE]
#define POS_POS_NO 0
#define POS_MULTI1 1
#define POS_SINGLE1_LOW 2
#define POS_SINGLE1_HIGH 3
#define POS_MULTI2 4
#define POS_SINGLE2_LOW 5
#define POS_SINGLE2_HIGH 6
#define POS_MULTI3 7
#define POS_SINGLE3_LOW 8
#define POS_SINGLE3_HIGH 9
#define POS_MULTI4 10
#define POS_SINGLE4_LOW 11
#define POS_SINGLE4_HIGH 12
#define POS_MULTI5 13
#define POS_SINGLE5_LOW 14
#define POS_SINGLE5_HIGH 15
#define POS_MULTI6 16
#define POS_SINGLE6_LOW 17
#define POS_SINGLE6_HIGH 18

//40*30=1200个双字
//组号(0)+命令号(1)+位置号(2)+运行速度(3)+加减速时间(4)+暂停(5)+停止(6)+条件1(7)+条件2(8)+DO延时(9)+DO输出持续时间(10)+输出(11)
//+1#脉冲_低(12)+1#脉冲_高(13)+1#速度(14)+1#加减速(15)+2#脉冲_低(16)+2#脉冲_高(17)+2#速度(18)+2#加减速(19)
//+3#脉冲_低(20)+3#脉冲_高(21)+3#速度(22)+3#加减速(23)+4#脉冲_低(24)+4#脉冲_高(25)+4#速度(26)+4#加减速(27)
//+5#脉冲_低(28)+5#脉冲_高(29)+5#速度(30)+5#加减速(31)+6#脉冲_低(32)+6#脉冲_高(33)+6#速度(34)+6#加减速(35)
#define w_ParLst_Pos_CMD w_ParLst_Drive[PAR2_SIZE + FLASH_POS_SIZE]
#define CMD_GROUP_NO 0
#define CMD_CMD_NO 1
#define CMD_POS_NO 2
#define CMD_RUN_SPEED_RATIO 3
#define CMD_ACC_SPEED 4
#define CMD_PAUSE_TIME 5
#define CMD_HALT_TIME 6
#define CMD_CONDITION1 7
#define CMD_CONDITION2 8
#define CMD_DO_DELAY 9
#define CMD_DO_TIME 10
#define CMD_DO_OUT 11
#define CMD_PULSE1_LOW 12
#define CMD_PULSE1_HIGH 13
#define CMD_SPEED1 14
#define CMD_ACC_SPEED1 15
#define CMD_PULSE2_LOW 16
#define CMD_PULSE2_HIGH 17
#define CMD_SPEED2 18
#define CMD_ACC_SPEED2 19
#define CMD_PULSE3_LOW 20
#define CMD_PULSE3_HIGH 21
#define CMD_SPEED3 22
#define CMD_ACC_SPEED3 23
#define CMD_PULSE4_LOW 24
#define CMD_PULSE4_HIGH 25
#define CMD_SPEED4 26
#define CMD_ACC_SPEED4 27
#define CMD_PULSE5_LOW 28
#define CMD_PULSE5_HIGH 29
#define CMD_SPEED5 30
#define CMD_ACC_SPEED5 31
#define CMD_PULSE6_LOW 32
#define CMD_PULSE6_HIGH 33
#define CMD_SPEED6 34
#define CMD_ACC_SPEED6 35
#define CMD_RUN_TIME 36 //定义运行时间，在命令指令中是位置36
//最后一条命令
#define w_ParLst_LastPos_CMD w_ParLst_Drive[PAR2_SIZE + FLASH_POS_SIZE + FLASH_POS_CMD_SIZE - POS_CMD_SIZE]

//外部FLASH W25Q16地址定义
//W25Q16 FLASH存储芯片，16Mbit，即2M字节（512个扇区，每个扇区16页，每页256字节，即每个扇区4K字节）
//最大地址2*1024*1024=2097152，即0x00200000
#define FLASH_SAVE_ADDR0 0X00000000    //不能跨扇区写，所以一段保存到一个扇区，每个扇区4096个字节
#define FLASH_SAVE_POSTION1 0X00010000 //保存记录的各个位置点，格式：序号+6个关节的多圈数据及单圈数据
#define FLASH_SAVE_POS_CMD1 0X00020000 //同步联动指令，可保持5组指令
#define FLASH_SAVE_POS_CMD2 0X00030000
#define FLASH_SAVE_POS_CMD3 0X00040000
#define FLASH_SAVE_POS_CMD4 0X00050000
#define FLASH_SAVE_POS_CMD5 0X00060000

//内部Flash保存地址
//最后一个扇区，128K
#define InFLASH_SAVE_ADDR0 0x080E0000                                                            //参数区1保存在外边FLASH的起始地址
#define InFLASH_SAVE_ADDR1 InFLASH_SAVE_ADDR0 + PAR1_SIZE * sizeof(w_ParLst[0])                  //参数区2，结果=0x080E0000 +300*2=0x080E0000 +600
#define InFLASH_SAVE_POSTION1 InFLASH_SAVE_ADDR1 + PAR2_SIZE * sizeof(w_ParLst_DrivePar)         //位置数据，结果=0x080E0000 +600+300*4=0x080E0000+1800
#define InFLASH_SAVE_POS_CMD1 InFLASH_SAVE_POSTION1 + FLASH_POS_SIZE * sizeof(w_ParLst_DrivePar) //同步联动指令，结果=0x080E0000 +1800+300*4=3000

#define SECTORWORDNUM 256       // 扇区字数量
#define FLASH_ID_SIZE 128       // ID参数数量
#define PROGRAM_LEN 0xE000      // 0-0xE000 程序空间长度
#define FLASH_PAR_ADDR 0xE000   // NV参数地址
#define FLASH_ID_ADDR 0xE200    // NV 电话等参数地址
#define FLASH_FAULT_ADDR 0xE800 // 故障记录区
#define FLASH_REC_ADDR 0xEA00   // FLASH 记录
#define FLASH_REC_SIZE 64       // 一条FLASH记录的长度(字节)
#define NORCVMAXMS 5            // 20 2007.7.5
#define SECTOR_SIZE 65536       // 扇区长度
#define RDWR_SIZE 64            // 读写FLASH的长度
#define ISL1208 0xDE            // Device address for chip A--ISL1208
#define AT24C256 0xA4           // Device address for chip B
#define CS_Flash1 1             // 片选FLASH1
#define CS_FMRAM1 2             // 片选FMRAM1
#define FLASH_REC_MAX 16384     // FLASH记录最大数
#define FAULT_REC_MAX 64        //故障记录最大数

#define TXD1_MAX 255 // 最大发送数量
#define RCV1_MAX 255 // 接收缓冲区长度 //256*8.5
#define TXD2_MAX 255 // 最大发送数量
#define RCV2_MAX 255 // 接收缓冲区长度 //256*8.5
#define TXD3_MAX 255 // 最大发送数量
#define RCV3_MAX 255 // 接收缓冲区长度 //256*8.5
#define TXD4_MAX 255 // 最大发送数量
#define RCV4_MAX 255 // 接收缓冲区长度 //256*8.5
#define TXD5_MAX 255 // 最大发送数量
#define RCV5_MAX 255 // 接收缓冲区长度 //256*8.5
#define TXD6_MAX 255 // 最大发送数量
#define RCV6_MAX 255 // 接收缓冲区长度 //256*8.5

#define M1_SYNC_MASK 0x01
#define M2_SYNC_MASK 0x02
#define M3_SYNC_MASK 0x04
#define M4_SYNC_MASK 0x08
#define M5_SYNC_MASK 0x10
#define M6_SYNC_MASK 0x20
#define M6_SYNC_MASK_ALL 0x3F

// 把“位带地址＋位序号”转换别名地址宏
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
//把该地址转换成一个指针
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

//STM32F407 IO address mapping by value
#define GPIOA_ODR_Addr (GPIOA_BASE + 20) //0x4001080C
#define GPIOB_ODR_Addr (GPIOB_BASE + 20) //0x40010C0C
#define GPIOC_ODR_Addr (GPIOC_BASE + 20) //0x4001100C
#define GPIOD_ODR_Addr (GPIOD_BASE + 20) //0x4001140C
#define GPIOE_ODR_Addr (GPIOE_BASE + 20) //0x4001180C
#define GPIOF_ODR_Addr (GPIOF_BASE + 20) //0x40011A0C
#define GPIOG_ODR_Addr (GPIOG_BASE + 20) //0x40011E0C

#define GPIOA_IDR_Addr (GPIOA_BASE + 16) //0x40010808
#define GPIOB_IDR_Addr (GPIOB_BASE + 16) //0x40010C08
#define GPIOC_IDR_Addr (GPIOC_BASE + 16) //0x40011008
#define GPIOD_IDR_Addr (GPIOD_BASE + 16) //0x40011408
#define GPIOE_IDR_Addr (GPIOE_BASE + 16) //0x40011808
#define GPIOF_IDR_Addr (GPIOF_BASE + 16) //0x40011A08
#define GPIOG_IDR_Addr (GPIOG_BASE + 16) //0x40011E08

//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) //输出
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  //输入

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) //输出
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  //输入

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n) //输出
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)  //输入

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n) //输出
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)  //输入

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n) //输出
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)  //输入

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n) //输出
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)  //输入

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n) //输出
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)  //输入

/* BSP硬件定义  */
//----------------------------------------------------------------

//开关量输入
#define DI1 PDin(10)  //
#define DI2 PDin(11)  //
#define DI3 PDin(12)  //
#define DI4 PDin(13)  //
#define DI5 PFin(3)   //
#define DI6 PFin(2)   //
#define DI7 PFin(1)   //
#define DI8 PFin(0)   //
#define DI9 PEin(6)   //
#define DI10 PEin(4)  //
#define DI11 PEin(3)  //
#define DI12 PEin(2)  //
#define DI13 PGin(12) //
#define DI14 PGin(11) //
#define DI15 PGin(10) //
#define DI16 PDin(7)  //
#define DI17 PCin(4)  //
#define DI18 PCin(5)  //
#define DI19 PBin(0)  //
#define DI20 PFin(11) //
#define DI21 PEin(7)  //
#define DI22 PEin(8)  //
#define DI23 PEin(10) //
#define DI24 PEin(11) //
#define DI25 PEin(12) //
#define DI26 PEin(13) //
#define DI27 PEin(14) //
#define DI28 PEin(15) //

//继电器输出			在DSP上，通过输出DO1，DO2可以间接控制继电器 2015.5.10 必须DSP复位等都正常后
#define DO1 PAout(8) // =1 OPEN; =0 CLOSE
#define DO2 PCout(9) // =1 OPEN; =0 CLOSE
#define DO3 PCout(8) // =1 OPEN; =0 CLOSE
#define DO4 PCout(7)
#define DO5 PEout(1)
#define DO6 PEout(0)
#define DO7 PBout(9)
#define DO8 PBout(8)
#define DO9 PBout(7)
#define DO10 PFout(4)
#define DO11 PGout(5)
#define DO12 PGout(4)
#define DO13 PGout(3)
#define DO14 PFout(12)
#define DO15 PFout(13)
#define DO16 PFout(14)
#define DO17 PFout(15)
#define DO18 PGout(0)
#define DO19 PGout(1)
#define DO20 PDout(14)
#define DO21 PDout(15)
#define DO22 PGout(2)

#define DO1_IN PAin(8)
#define DO2_IN PCin(9)
#define DO3_IN PCin(8)
#define DO4_IN PCin(7)
#define DO5_IN PEin(1)
#define DO6_IN PEin(0)
#define DO7_IN PBin(9)
#define DO8_IN PBin(8)
#define DO9_IN PBin(7)
#define DO10_IN PFin(4)
#define DO11_IN PGin(5)
#define DO12_IN PGin(4)
#define DO13_IN PGin(3)
#define DO14_IN PFin(12)
#define DO15_IN PFin(13)
#define DO16_IN PFin(14)
#define DO17_IN PFin(15)
#define DO18_IN PGin(0)
#define DO19_IN PGin(1)
#define DO20_IN PDin(14)
#define DO21_IN PDin(15)
#define DO22_IN PGin(2)

#define STOP_BRAKE_SYSTEM HAL_GPIO_WritePin(GPIOA, DO1_Pin, GPIO_PIN_RESET) //停止刹车，允许运行
#define START_BRAKE_SYSTEM HAL_GPIO_WritePin(GPIOA, DO1_Pin, GPIO_PIN_SET)  //开始刹车，不允许运行
#define Brake_Status_DO1 PAin(8)                                            //刹车状态

#define OPEN_HAND HAL_GPIO_WritePin(GPIOC, DO2_Pin, GPIO_PIN_RESET) //打开机械手臂
#define CLOSE_HAND HAL_GPIO_WritePin(GPIOC, DO2_Pin, GPIO_PIN_SET)  //闭合机械手臂
#define HAND_STATUS PCin(9)                                         //0打开电磁阀；1关闭电磁阀

#define TIME_TEST_DEBUG 0 //测量函数运行时间的调试开关,=1可以print函数运行时间
#define exFLASH_EXIST 0   //外部FLASH是否存在开关,=1表示存在外部FLASH芯片;=1表示无外部FLASH，需保存在内部FLASH中
#define POS_TYPE 1        //=0表示位置数据类型为编码器数据;=1表示位置数据类型为脉冲值

#define MOTOR1_AA_VALUE ((Motor1_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor1_XiShu_1 * M_T_AA * M_T_RA) * 60) //=14*60=840
#define MOTOR2_AA_VALUE ((Motor2_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor2_XiShu_1 * M_T_AA * M_T_RA) * 60)
#define MOTOR3_AA_VALUE ((Motor3_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor3_XiShu_1 * M_T_AA * M_T_RA) * 60)
#define MOTOR4_AA_VALUE ((Motor4_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor4_XiShu_1 * M_T_AA * M_T_RA) * 60)
#define MOTOR5_AA_VALUE ((Motor5_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor5_XiShu_1 * M_T_AA * M_T_RA) * 60)
#define MOTOR6_AA_VALUE ((Motor6_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor6_XiShu_1 * M_T_AA * M_T_RA) * 60)

#define DI_STABLE_NUM 20 // 开关量稳定数目

//电机同步参数
#define TIME_COST_DELTA 32 * 20 //要调整的电机运行时间与最长运行时间的偏差值
#define FRE_AA_DELTA 1          //加加速度偏差值
#define FRE_AA_MIN 1            //最小加加速度
#define FRE_START_DELTA 0.1     //起始频率偏差值

// 定义设定参数
#define Pw_Motor1SendPulse w_ParLst[0]    // 电机1发送脉冲数
#define Pw_Motor1SendPulse_HW w_ParLst[1] // 电机1发送脉冲数高字
#define Pw_Motor1_TurnAngle w_ParLst[2]   // 电机1转动角度
#define Pw_Motor1_ACCSpeed w_ParLst[3]    // 电机1加速度
#define Pw_Motor1_FRE_AA w_ParLst[4]      // 电机1加加速度
#define Pw_Motor1_SetSpeed w_ParLst[5]    // 电机1设定速度
#define Pw_Motor1_StartSpeed w_ParLst[6]  // 电机1起始速度
#define Pw_Motor1_PULSENUM w_ParLst[7]    // 电机1每圈脉冲数
#define Pw_ModPar w_ParLst[9]             // 修改参数

#define Pw_Motor2SendPulse w_ParLst[15]    // 电机2发送脉冲数
#define Pw_Motor2SendPulse_HW w_ParLst[16] // 电机2发送脉冲数高字
#define Pw_Motor2_TurnAngle w_ParLst[17]   // 电机2转动角度
#define Pw_Motor2_ACCSpeed w_ParLst[18]    // 电机2加速度
#define Pw_Motor2_FRE_AA w_ParLst[19]      // 电机2加加速度
#define Pw_Motor2_SetSpeed w_ParLst[20]    // 电机2设定速度
#define Pw_Motor2_StartSpeed w_ParLst[21]  // 电机2起始速度
#define Pw_Motor2_PULSENUM w_ParLst[22]    // 电机2每圈脉冲数

#define Pw_Motor3SendPulse w_ParLst[25]    // 电机3发送脉冲数
#define Pw_Motor3SendPulse_HW w_ParLst[26] // 电机3发送脉冲数高字
#define Pw_Motor3_TurnAngle w_ParLst[27]   // 电机3转动角度
#define Pw_Motor3_ACCSpeed w_ParLst[28]    // 电机3加速度
#define Pw_Motor3_FRE_AA w_ParLst[29]      // 电机3加加速度
#define Pw_Motor3_SetSpeed w_ParLst[30]    // 电机3设定速度
#define Pw_Motor3_StartSpeed w_ParLst[31]  // 电机3起始速度
#define Pw_Motor3_PULSENUM w_ParLst[32]    // 电机3每圈脉冲数

#define Pw_Motor4SendPulse w_ParLst[35]    // 电机4发送脉冲数
#define Pw_Motor4SendPulse_HW w_ParLst[36] // 电机4发送脉冲数高字
#define Pw_Motor4_TurnAngle w_ParLst[37]   // 电机4转动角度
#define Pw_Motor4_ACCSpeed w_ParLst[38]    // 电机4加速度
#define Pw_Motor4_FRE_AA w_ParLst[39]      // 电机4加加速度
#define Pw_Motor4_SetSpeed w_ParLst[40]    // 电机4设定速度
#define Pw_Motor4_StartSpeed w_ParLst[41]  // 电机4起始速度
#define Pw_Motor4_PULSENUM w_ParLst[42]    // 电机4每圈脉冲数

#define Pw_Motor5SendPulse w_ParLst[45]    // 电机5发送脉冲数
#define Pw_Motor5SendPulse_HW w_ParLst[46] // 电机5发送脉冲数高字
#define Pw_Motor5_TurnAngle w_ParLst[47]   // 电机5转动角度
#define Pw_Motor5_ACCSpeed w_ParLst[48]    // 电机5加速度
#define Pw_Motor5_FRE_AA w_ParLst[49]      // 电机5加加速度
#define Pw_Motor5_SetSpeed w_ParLst[50]    // 电机5设定速度
#define Pw_Motor5_StartSpeed w_ParLst[51]  // 电机5起始速度
#define Pw_Motor5_PULSENUM w_ParLst[52]    // 电机5每圈脉冲数

#define Pw_Motor6SendPulse w_ParLst[55]    // 电机6发送脉冲数
#define Pw_Motor6SendPulse_HW w_ParLst[56] // 电机6发送脉冲数高字
#define Pw_Motor6_TurnAngle w_ParLst[57]   // 电机6转动角度
#define Pw_Motor6_ACCSpeed w_ParLst[58]    // 电机6加速度
#define Pw_Motor6_FRE_AA w_ParLst[59]      // 电机6加加速度
#define Pw_Motor6_SetSpeed w_ParLst[60]    // 电机6设定速度
#define Pw_Motor6_StartSpeed w_ParLst[61]  // 电机6起始速度
#define Pw_Motor6_PULSENUM w_ParLst[62]    // 电机6每圈脉冲数

#define Pw_Initial_F w_ParLst[65]           // 初始化标志，=0x5A，表示已经初始化
#define Pw_ParInitial w_ParLst[66]          // 参数初始化命令
#define Pos_Group_Select w_ParLst[67]       // 联动位置控制命令组选择（1-5组）
#define Pw_FRam_Read_StartAdrr w_ParLst[68] //读取FRAM起始地址
#define Pw_FRam_Read_DataLen w_ParLst[69]   //读取FRAM数据长度

#define Pw_SetSecond w_ParLst[70] // 设置秒	// 注意:地址不能随便改动,被数组使用  ZCL
#define Pw_SetMinute w_ParLst[71] // 设置分
#define Pw_SetHour w_ParLst[72]   // 设置时
#define Pw_SetDay w_ParLst[73]    // 设置日
#define Pw_SetMonth w_ParLst[74]  // 设置月
#define Pw_SetYear w_ParLst[75]   // 设置年
#define Pw_SetWeek w_ParLst[76]   // 设置星期

#define Pw_DIStableSetNum w_ParLst[115]       // 开关量稳定数目
#define Pr_Driver1_Control_OK_F w_ParLst[116] //1#伺服控制命令OK标志
#define Pr_Driver2_Control_OK_F w_ParLst[117] //2#伺服控制命令OK标志
#define Pr_Driver3_Control_OK_F w_ParLst[118] //3#伺服控制命令OK标志
#define Pr_Driver4_Control_OK_F w_ParLst[119] //4#伺服控制命令OK标志
#define Pr_Driver5_Control_OK_F w_ParLst[120] //5#伺服控制命令OK标志
#define Pr_Driver6_Control_OK_F w_ParLst[121] //6#伺服控制命令OK标志

#define Pw_SendPWM w_ParLst[122]       // 发PWM波命令
#define Pw_S_ParaInitial w_ParLst[123] // S曲线初始化命令

#define Pw_BaudRate1 w_ParLst[141] // 波特率1
#define Pw_BaudRate2 w_ParLst[142] // 波特率2
#define Pw_BaudRate3 w_ParLst[143] // 波特率3
#define Pw_BaudRate4 w_ParLst[144] // 波特率4
#define Pw_BaudRate5 w_ParLst[145] // 波特率5
#define Pw_BaudRate6 w_ParLst[146] // 波特率6

#define Pw_EquipmentNo1 w_ParLst[159] //1#伺服电机Modbus地址
#define Pw_EquipmentNo2 w_ParLst[160] //2#伺服电机Modbus地址
#define Pw_EquipmentNo3 w_ParLst[161] //3#伺服电机Modbus地址
#define Pw_EquipmentNo4 w_ParLst[162] //4#伺服电机Modbus地址
#define Pw_EquipmentNo5 w_ParLst[163] //5#伺服电机Modbus地址
#define Pw_EquipmentNo6 w_ParLst[164] //6#伺服电机Modbus地址

#define Pr_RUN_Count w_ParLst[179]          //运行指令计数
#define Pr_RUN_Count_Set w_ParLst[180]      //运行多少圈设定，初始化为0
#define Pw_Total_RUN_Count w_ParLst[181]    //累计运行圈数，出厂值为0
#define Pw_Total_RUN_Count_HW w_ParLst[182] //累计运行圈数_高字
#define Pw_ComWriteErr_Stop w_ParLst[184]   //通讯数据写入错误停机功能，=1，停机；=0，不停机

#define Pw_ComErr_Stop w_ParLst[222]  //通讯故障停机功能，=1，停机；=0，不停机
#define Pr_F_HaveFault w_ParLst[223]  //有电机故障标志
#define Pw_Limit_MaxPos w_ParLst[224] //最大位置限制功能，=1，启用；=0，不启动

#define Pr_F_ComErr w_ParLst[225]       //有通讯故障标志
#define Pr_Driver1_ComErr w_ParLst[226] //Driver1通讯故障标志
#define Pr_Driver2_ComErr w_ParLst[227] //Driver2通讯故障标志
#define Pr_Driver3_ComErr w_ParLst[228] //Driver3通讯故障标志
#define Pr_Driver4_ComErr w_ParLst[229] //Driver4通讯故障标志
#define Pr_Driver5_ComErr w_ParLst[230] //Driver5通讯故障标志
#define Pr_Driver6_ComErr w_ParLst[231] //Driver6通讯故障标志

#define Pr_BRAKE_Status w_ParLst[244]  //刹车状态
#define Pr_BRAKE_Control w_ParLst[245] //刹车控制，=1，刹车；=0，不刹车
#define Pr_F_AllStopped w_ParLst[246]  //所有电机都停止标志

#define Pr_AllRun w_ParLst[249] //所有电机都运行标志
#define w_SoftVer w_ParLst[250] // 软件版本

//软件时钟
#define SClk1Ms SoftClock[0]    // 软件时钟 1ms
#define SClk10Ms SoftClock[1]   // 软件时钟 10ms
#define SClkSecond SoftClock[2] // 软件时钟  s
#define SClkMinute SoftClock[3] // 软件时钟  m
#define SClkHour SoftClock[4]   // 软件时钟  h
#define SClkDay SoftClock[5]    // 软件时钟  d
#define SClkMonth SoftClock[6]  // 软件时钟  m
#define SClkYear SoftClock[7]   // 软件时钟  y
#define SClk0r5Ms SoftClock[8]  // 软件时钟  0.5MS	2010.8.6 定时中断中采集电流值

#define RealSecond RealClock[0] // 实时时钟
#define RealMinute RealClock[1] // 实时时钟
#define RealHour RealClock[2]   // 实时时钟
#define RealDay RealClock[3]    // 实时时钟
#define RealMonth RealClock[4]  // 实时时钟
#define RealYear RealClock[5]   // 实时时钟

//定义通讯命令缓冲区长度
#define COM_QUERY_SIZE 10 //查询指令长度
#define COM_QUERY_NUM 10  //查询指令数量

#define MOTOR_MAX_SPEED12 1500  //1#、2#电机最大转速1500r/min
#define MOTOR_MAX_SPEED3 2000   //3#电机最大转速2000r/min
#define MOTOR_MAX_SPEED456 3000 //3#电机最大转速3000r/min

// 定义设定参数
#define Pw_Driver1_Pluse w_ParLst_Drive[0]     //1#手动运行脉冲（低字）
#define Pw_Driver1_Pluse_HW w_ParLst_Drive[1]  //1#手动运行脉冲（高字）
#define Pw_Driver2_Pluse w_ParLst_Drive[2]     //2#手动运行脉冲（低字）
#define Pw_Driver2_Pluse_HW w_ParLst_Drive[3]  //2#手动运行脉冲（高字）
#define Pw_Driver3_Pluse w_ParLst_Drive[4]     //3#手动运行脉冲（低字）
#define Pw_Driver3_Pluse_HW w_ParLst_Drive[5]  //3#手动运行脉冲（高字）
#define Pw_Driver4_Pluse w_ParLst_Drive[6]     //4#手动运行脉冲（低字）
#define Pw_Driver4_Pluse_HW w_ParLst_Drive[7]  //4#手动运行脉冲（高字）
#define Pw_Driver5_Pluse w_ParLst_Drive[8]     //5#手动运行脉冲（低字）
#define Pw_Driver5_Pluse_HW w_ParLst_Drive[9]  //5#手动运行脉冲（高字）
#define Pw_Driver6_Pluse w_ParLst_Drive[10]    //6#手动运行脉冲（低字）
#define Pw_Driver6_Pluse_HW w_ParLst_Drive[11] //6#手动运行脉冲（高字）

#define Pr_Drive1_Status1 w_ParLst_Drive[12] //1#伺服电机状态字，对应伺服地址P8901
#define Pr_Drive2_Status1 w_ParLst_Drive[13] //2#伺服电机状态字
#define Pr_Drive3_Status1 w_ParLst_Drive[14] //3#伺服电机状态字
#define Pr_Drive4_Status1 w_ParLst_Drive[15] //4#伺服电机状态字
#define Pr_Drive5_Status1 w_ParLst_Drive[16] //5#伺服电机状态字
#define Pr_Drive6_Status1 w_ParLst_Drive[17] //6#伺服电机状态字

#define Pw_Driver1_R_Enable w_ParLst_Drive[21] //1#手动运行使能（反转）
#define Pw_Driver2_R_Enable w_ParLst_Drive[22] //2#手动运行使能（反转）
#define Pw_Driver3_R_Enable w_ParLst_Drive[23] //3#手动运行使能（反转）
#define Pw_Driver4_R_Enable w_ParLst_Drive[24] //4#手动运行使能（反转）
#define Pw_Driver5_R_Enable w_ParLst_Drive[25] //5#手动运行使能（反转）
#define Pw_Driver6_R_Enable w_ParLst_Drive[26] //6#手动运行使能（反转）

#define Pw_Driver1_Speed w_ParLst_Drive[27]      //1#手动运行速度
#define Pw_Driver1_AccTime w_ParLst_Drive[28]    //1#手动加减速：小，加减速慢；大，加减速快
#define Pw_Driver1_StartSpeed w_ParLst_Drive[29] //1#手动起始运行速度

#define Pw_Driver2_Speed w_ParLst_Drive[30]      //2#手动运行速度
#define Pw_Driver2_AccTime w_ParLst_Drive[31]    //2#手动加减速：小，加减速慢；大，加减速快
#define Pw_Driver2_StartSpeed w_ParLst_Drive[32] //2#手动起始运行速度

#define Pw_Driver3_Speed w_ParLst_Drive[33]      //3#手动运行速度
#define Pw_Driver3_AccTime w_ParLst_Drive[34]    //3#手动加减速：小，加减速慢；大，加减速快
#define Pw_Driver3_StartSpeed w_ParLst_Drive[35] //3#手动起始运行速度

#define Pw_Driver4_Speed w_ParLst_Drive[36]      //4#手动运行速度
#define Pw_Driver4_AccTime w_ParLst_Drive[37]    //4#手动加减速：小，加减速慢；大，加减速快
#define Pw_Driver4_StartSpeed w_ParLst_Drive[38] //4#手动起始运行速度

#define Pw_Driver5_Speed w_ParLst_Drive[39]      //5#手动运行速度
#define Pw_Driver5_AccTime w_ParLst_Drive[40]    //5#手动加减速：小，加减速慢；大，加减速快
#define Pw_Driver5_StartSpeed w_ParLst_Drive[41] //5#手动起始运行速度

#define Pw_Driver6_Speed w_ParLst_Drive[42]      //6#手动运行速度
#define Pw_Driver6_AccTime w_ParLst_Drive[43]    //6#手动加减速：小，加减速慢；大，加减速快
#define Pw_Driver6_StartSpeed w_ParLst_Drive[20] //6#手动起始运行速度

#define Pw_Driver1_Enable w_ParLst_Drive[44] //1#手动运行使能（正转）
#define Pw_Driver2_Enable w_ParLst_Drive[45] //2#手动运行使能（正转）
#define Pw_Driver3_Enable w_ParLst_Drive[46] //3#手动运行使能（正转）
#define Pw_Driver4_Enable w_ParLst_Drive[47] //4#手动运行使能（正转）
#define Pw_Driver5_Enable w_ParLst_Drive[48] //5#手动运行使能（正转）
#define Pw_Driver6_Enable w_ParLst_Drive[49] //6#手动运行使能（正转）

#define Pw_StepAutoMode w_ParLst_Drive[50] //=1，手动模式；=0，全自动模式

#define Pw_Driver1_SetValue w_ParLst_Drive[51]    //1#伺服电机设定值
#define Pw_Driver1_SetValue_HW w_ParLst_Drive[52] //1#伺服电机设定值（高字）
#define Pw_Driver2_SetValue w_ParLst_Drive[53]    //1#伺服电机设定值
#define Pw_Driver2_SetValue_HW w_ParLst_Drive[54] //1#伺服电机设定值（高字）
#define Pw_Driver3_SetValue w_ParLst_Drive[55]    //1#伺服电机设定值
#define Pw_Driver3_SetValue_HW w_ParLst_Drive[56] //1#伺服电机设定值（高字）
#define Pw_Driver4_SetValue w_ParLst_Drive[57]    //1#伺服电机设定值
#define Pw_Driver4_SetValue_HW w_ParLst_Drive[58] //1#伺服电机设定值（高字）
#define Pw_Driver5_SetValue w_ParLst_Drive[59]    //1#伺服电机设定值
#define Pw_Driver5_SetValue_HW w_ParLst_Drive[60] //1#伺服电机设定值（高字）
#define Pw_Driver6_SetValue w_ParLst_Drive[61]    //1#伺服电机设定值
#define Pw_Driver6_SetValue_HW w_ParLst_Drive[62] //1#伺服电机设定值（高字）

#define Pw_SF_Driver1_InitPos w_ParLst_Drive[63]    //伺服电机1的初始位置
#define Pw_SF_Driver1_InitPos_HW w_ParLst_Drive[64] //伺服电机1的初始位置（高字）
#define Pw_SF_Driver2_InitPos w_ParLst_Drive[65]    //伺服电机2的初始位置
#define Pw_SF_Driver2_InitPos_HW w_ParLst_Drive[66] //伺服电机2的初始位置（高字）
#define Pw_SF_Driver3_InitPos w_ParLst_Drive[67]    //伺服电机3的初始位置
#define Pw_SF_Driver3_InitPos_HW w_ParLst_Drive[67] //伺服电机3的初始位置（高字）
#define Pw_SF_Driver4_InitPos w_ParLst_Drive[69]    //伺服电机4的初始位置
#define Pw_SF_Driver4_InitPos_HW w_ParLst_Drive[70] //伺服电机4的初始位置（高字）
#define Pw_SF_Driver5_InitPos w_ParLst_Drive[71]    //伺服电机5的初始位置
#define Pw_SF_Driver5_InitPos_HW w_ParLst_Drive[72] //伺服电机5的初始位置（高字）
#define Pw_SF_Driver6_InitPos w_ParLst_Drive[73]    //伺服电机6的初始位置
#define Pw_SF_Driver6_InitPos_HW w_ParLst_Drive[74] //伺服电机6的初始位置（高字）

#define Pw_Fault_Stop w_ParLst_Drive[75]      // 发生伺服故障停机功能，=1，停机；=0，不停机，仍然运行
#define Pw_Read_CurrentPos w_ParLst_Drive[76] // 读初始位置，=1，读位置
#define Pw_EMERGENCY_STOP w_ParLst_Drive[77]  // 急停命令，=1，进行急停
#define Pw_ResetCMD w_ParLst_Drive[78]        // 复位命令，=1，进行复位
#define Pw_Stop_Reset w_ParLst_Drive[79]      // 停机复位功能，=1，停机后进行复位；=0，停机后不复位

#define Pw_TouchRunStop w_ParLst_Drive[80] // 触摸 启动/停止
#define Pw_SaveDelay w_ParLst_Drive[81]    //默认延时180s保存修改后的参数到FLASH中

#define Pr_F_Drive1_Stop w_ParLst_Drive[102] //1#电机停止标志（位置到达）
#define Pr_F_Drive2_Stop w_ParLst_Drive[103] //2#电机停止标志（位置到达）
#define Pr_F_Drive3_Stop w_ParLst_Drive[104] //3#电机停止标志（位置到达）
#define Pr_F_Drive4_Stop w_ParLst_Drive[105] //4#电机停止标志（位置到达）
#define Pr_F_Drive5_Stop w_ParLst_Drive[106] //5#电机停止标志（位置到达）
#define Pr_F_Drive6_Stop w_ParLst_Drive[107] //6#电机停止标志（位置到达）

#define Pw_ComErrCount w_ParLst_Drive[108] //通讯故障延时判断

#define Pr_Driver_Running_No w_ParLst_Drive[111]  //当前执行的命令序号
#define Pr_Driver_Previous_No w_ParLst_Drive[112] //前一个执行的命令序号
//#define	Pr_Drive3_Run_No			w_ParLst_Drive[113]			//3#电机当前执行的命令序号
//#define	Pr_Drive4_Run_No			w_ParLst_Drive[114]			//4#电机当前执行的命令序号
//#define	Pr_Drive5_Run_No			w_ParLst_Drive[115]			//5#电机当前执行的命令序号
//#define	Pr_Drive6_Run_No			w_ParLst_Drive[116]			//6#电机当前执行的命令序号

#define Pr_F_Drive1_Runing w_ParLst_Drive[117] //1#电机运行标志
#define Pr_F_Drive2_Runing w_ParLst_Drive[118] //2#电机运行标志
#define Pr_F_Drive3_Runing w_ParLst_Drive[119] //3#电机运行标志
#define Pr_F_Drive4_Runing w_ParLst_Drive[120] //4#电机运行标志
#define Pr_F_Drive5_Runing w_ParLst_Drive[121] //5#电机运行标志
#define Pr_F_Drive6_Runing w_ParLst_Drive[122] //6#电机运行标志

#define Pr_F_Driver1_Rdy w_ParLst_Drive[123] //1#伺服电机准备好标志
#define Pr_F_Driver2_Rdy w_ParLst_Drive[124] //2#伺服电机准备好标志
#define Pr_F_Driver3_Rdy w_ParLst_Drive[125] //3#伺服电机准备好标志
#define Pr_F_Driver4_Rdy w_ParLst_Drive[126] //4#伺服电机准备好标志
#define Pr_F_Driver5_Rdy w_ParLst_Drive[127] //5#伺服电机准备好标志
#define Pr_F_Driver6_Rdy w_ParLst_Drive[128] //6#伺服电机准备好标志

#define Pr_Driver1_ComCount w_ParLst_Drive[129] //1#伺服电机通讯计数
#define Pr_Driver2_ComCount w_ParLst_Drive[130] //2#伺服电机通讯计数
#define Pr_Driver3_ComCount w_ParLst_Drive[131] //3#伺服电机通讯计数
#define Pr_Driver4_ComCount w_ParLst_Drive[132] //4#伺服电机通讯计数
#define Pr_Driver5_ComCount w_ParLst_Drive[133] //5#伺服电机通讯计数
#define Pr_Driver6_ComCount w_ParLst_Drive[134] //6#伺服电机通讯计数

#define Pr_Drive1_Status w_ParLst_Drive[135] //1#伺服电机状态字，对应伺服地址P8902
#define Pr_Drive2_Status w_ParLst_Drive[136] //2#伺服电机状态字
#define Pr_Drive3_Status w_ParLst_Drive[137] //3#伺服电机状态字
#define Pr_Drive4_Status w_ParLst_Drive[138] //4#伺服电机状态字
#define Pr_Drive5_Status w_ParLst_Drive[139] //5#伺服电机状态字
#define Pr_Drive6_Status w_ParLst_Drive[140] //6#伺服电机状态字

#define Pr_F_Drvier1_Err w_ParLst_Drive[141] //1#伺服电机故障标志
#define Pr_F_Drvier2_Err w_ParLst_Drive[142] //2#伺服电机故障标志
#define Pr_F_Drvier3_Err w_ParLst_Drive[143] //3#伺服电机故障标志
#define Pr_F_Drvier4_Err w_ParLst_Drive[144] //4#伺服电机故障标志
#define Pr_F_Drvier5_Err w_ParLst_Drive[145] //5#伺服电机故障标志
#define Pr_F_Drvier6_Err w_ParLst_Drive[146] //6#伺服电机故障标志

#define Pr_Drive1_FaultNo w_ParLst_Drive[147] //1#伺服电机当前最高级别故障码，对应伺服地址P0931
#define Pr_Drive2_FaultNo w_ParLst_Drive[148] //2#伺服电机当前最高级别故障码
#define Pr_Drive3_FaultNo w_ParLst_Drive[149] //3#伺服电机当前最高级别故障码
#define Pr_Drive4_FaultNo w_ParLst_Drive[150] //4#伺服电机当前最高级别故障码
#define Pr_Drive5_FaultNo w_ParLst_Drive[151] //5#伺服电机当前最高级别故障码
#define Pr_Drive6_FaultNo w_ParLst_Drive[152] //6#伺服电机当前最高级别故障码

#define Pw_Com_Delay1 w_ParLst_Drive[153] //COM延时1
#define Pw_Com_Delay2 w_ParLst_Drive[154] //COM延时2
//#define	Pw_Com2_Delay1						w_ParLst_Drive[155]			//COM2延时1
//#define	Pw_Com2_Delay2						w_ParLst_Drive[156]			//COM2延时2
//#define	Pw_Com3_Delay1						w_ParLst_Drive[157]			//COM3延时1
//#define	Pw_Com3_Delay2						w_ParLst_Drive[158]			//COM3延时2
//#define	Pw_Com3_Delay3						w_ParLst_Drive[159]			//COM3延时3
//#define	Pw_Com4_Delay2						w_ParLst_Drive[160]			//COM4延时2

#define Pr_Drive1_singleData w_ParLst_Drive[161]    //1#伺服电机编码器单圈数据，对应伺服地址P091A
#define Pr_Drive1_singleData_HW w_ParLst_Drive[162] //1#伺服电机编码器单圈数据（高字）
#define Pr_Drive2_singleData w_ParLst_Drive[163]    //2#伺服电机编码器单圈数据，对应伺服地址P091A
#define Pr_Drive2_singleData_HW w_ParLst_Drive[164] //2#伺服电机编码器单圈数据（高字）
#define Pr_Drive3_singleData w_ParLst_Drive[165]    //3#伺服电机编码器单圈数据，对应伺服地址P091A
#define Pr_Drive3_singleData_HW w_ParLst_Drive[166] //3#伺服电机编码器单圈数据（高字）
#define Pr_Drive4_singleData w_ParLst_Drive[167]    //4#伺服电机编码器单圈数据，对应伺服地址P091A
#define Pr_Drive4_singleData_HW w_ParLst_Drive[168] //4#伺服电机编码器单圈数据（高字）
#define Pr_Drive5_singleData w_ParLst_Drive[169]    //5#伺服电机编码器单圈数据，对应伺服地址P091A
#define Pr_Drive5_singleData_HW w_ParLst_Drive[170] //5#伺服电机编码器单圈数据（高字）
#define Pr_Drive6_singleData w_ParLst_Drive[171]    //6#伺服电机编码器单圈数据，对应伺服地址P091A
#define Pr_Drive6_singleData_HW w_ParLst_Drive[172] //6#伺服电机编码器单圈数据（高字）

#define Pr_Drive1_MultiData w_ParLst_Drive[173] //1#伺服电机编码器多圈数据，对应伺服地址P091C
#define Pr_Drive2_MultiData w_ParLst_Drive[174] //2#伺服电机编码器多圈数据，对应伺服地址P091C
#define Pr_Drive3_MultiData w_ParLst_Drive[175] //3#伺服电机编码器多圈数据，对应伺服地址P091C
#define Pr_Drive4_MultiData w_ParLst_Drive[176] //4#伺服电机编码器多圈数据，对应伺服地址P091C
#define Pr_Drive5_MultiData w_ParLst_Drive[177] //5#伺服电机编码器多圈数据，对应伺服地址P091C
#define Pr_Drive6_MultiData w_ParLst_Drive[178] //6#伺服电机编码器多圈数据，对应伺服地址P091C

#define Pr_Drive1_singleData_Init w_ParLst_Drive[179]    //1#伺服电机编码器单圈初始设定值，对应伺服地址P091A
#define Pr_Drive1_singleData_Init_HW w_ParLst_Drive[180] //1#伺服电机编码器单圈初设定值（高字）
#define Pr_Drive2_singleData_Init w_ParLst_Drive[181]    //2#伺服电机编码器单圈初始设定值，对应伺服地址P091A
#define Pr_Drive2_singleData_Init_HW w_ParLst_Drive[182] //2#伺服电机编码器单圈初始设定值（高字）
#define Pr_Drive3_singleData_Init w_ParLst_Drive[183]    //3#伺服电机编码器单圈初始设定值，对应伺服地址P091A
#define Pr_Drive3_singleData_Init_HW w_ParLst_Drive[184] //3#伺服电机编码器单圈初始设定值（高字）
#define Pr_Drive4_singleData_Init w_ParLst_Drive[185]    //4#伺服电机编码器单圈初始设定值，对应伺服地址P091A
#define Pr_Drive4_singleData_Init_HW w_ParLst_Drive[186] //4#伺服电机编码器单圈初始设定值（高字）
#define Pr_Drive5_singleData_Init w_ParLst_Drive[187]    //5#伺服电机编码器单圈初始设定值，对应伺服地址P091A
#define Pr_Drive5_singleData_Init_HW w_ParLst_Drive[188] //5#伺服电机编码器单圈初始设定值（高字）
#define Pr_Drive6_singleData_Init w_ParLst_Drive[189]    //6#伺服电机编码器单圈初始设定值，对应伺服地址P091A
#define Pr_Drive6_singleData_Init_HW w_ParLst_Drive[190] //6#伺服电机编码器单圈初始设定值（高字）

#define Pr_Drive1_MultiData_Init w_ParLst_Drive[191] //1#伺服电机编码器多圈初始设定值，对应伺服地址P091C
#define Pr_Drive2_MultiData_Init w_ParLst_Drive[192] //2#伺服电机编码器多圈初始设定值，对应伺服地址P091C
#define Pr_Drive3_MultiData_Init w_ParLst_Drive[193] //3#伺服电机编码器多圈初始设定值，对应伺服地址P091C
#define Pr_Drive4_MultiData_Init w_ParLst_Drive[194] //4#伺服电机编码器多圈初始设定值，对应伺服地址P091C
#define Pr_Drive5_MultiData_Init w_ParLst_Drive[195] //5#伺服电机编码器多圈初始设定值，对应伺服地址P091C
#define Pr_Drive6_MultiData_Init w_ParLst_Drive[196] //6#伺服电机编码器多圈初始设定值，对应伺服地址P091C

#define Pr_Reset_Delay w_ParLst_Drive[197] //复位到初始位置延时时间，ms

#define Pw_PosError_Set w_ParLst_Drive[198]    //位置偏差设定
#define Pw_PosError_Set_HW w_ParLst_Drive[199] //位置偏差设定（高字）

//#define Pw_Drive1_P8910						w_ParLst_Drive[201]			//1#伺服电机P8910参数
//#define Pw_Drive2_P8910						w_ParLst_Drive[202]			//2#伺服电机P8910参数
//#define Pw_Drive3_P8910						w_ParLst_Drive[203]			//3#伺服电机P8910参数
//#define Pw_Drive4_P8910						w_ParLst_Drive[204]			//4#伺服电机P8910参数
//#define Pw_Drive5_P8910						w_ParLst_Drive[205]			//5#伺服电机P8910参数
//#define Pw_Drive6_P8910						w_ParLst_Drive[206]			//6#伺服电机P8910参数

//#define Pw_Driver_AllSavePos_Enable			w_ParLst_Drive[207]			//所有伺服电机写位置参数使能
//#define Pw_Driver1_SavePos_Enable			w_ParLst_Drive[208]			//1#伺服电机写位置参数使能
//#define Pw_Driver2_SavePos_Enable			w_ParLst_Drive[209]			//2#伺服电机写位置参数使能
//#define Pw_Driver3_SavePos_Enable			w_ParLst_Drive[210]			//3#伺服电机写位置参数使能
//#define Pw_Driver4_SavePos_Enable			w_ParLst_Drive[211]			//4#伺服电机写位置参数使能
//#define Pw_Driver5_SavePos_Enable			w_ParLst_Drive[212]			//5#伺服电机写位置参数使能
//#define Pw_Driver6_SavePos_Enable			w_ParLst_Drive[213]			//6#伺服电机写位置参数使能

#define Pw_Set_Run_Speed1 w_ParLst_Drive[214] //1#伺服电机设定最大运行速度
#define Pw_Set_Run_Speed2 w_ParLst_Drive[215] //2#伺服电机设定最大运行速度
#define Pw_Set_Run_Speed3 w_ParLst_Drive[216] //3#伺服电机设定最大运行速度
#define Pw_Set_Run_Speed4 w_ParLst_Drive[217] //4#伺服电机设定最大运行速度
#define Pw_Set_Run_Speed5 w_ParLst_Drive[218] //5#伺服电机设定最大运行速度
#define Pw_Set_Run_Speed6 w_ParLst_Drive[219] //6#伺服电机设定最大运行速度

#define Pw_Com_Delay3 w_ParLst_Drive[220]       //COM延时3
#define Pw_Com_Delay_Manual w_ParLst_Drive[221] //COM延时，手动

#define Pw_Define_Save_Pos w_ParLst_Drive[222] //定义并记录位置命令
#define Pw_Cal_Pos_CMD w_ParLst_Drive[223]     //计算位置及速度命令
#define Pw_Verify_Pos_CMD w_ParLst_Drive[224]  //校验位置命令
#define Pw_Running_Pos_CMD w_ParLst_Drive[225] //正在运行的命令号
#define Pw_Step_Pos_CMD w_ParLst_Drive[226]    //单步运行命令
#define Pw_Current_Pos_No w_ParLst_Drive[227]  //当前位置号[1-15]
#define Pw_Set_Run_Speed w_ParLst_Drive[228]   //基准运行速度
#define Pw_Read_Init_Pos w_ParLst_Drive[229]   //保存原点位置命令

#define Pw_Com1_Driver1_BufferNum w_ParLst_Drive[230] //1#命令队列中的条数
#define Pw_Com1_Driver2_BufferNum w_ParLst_Drive[231] //2#命令队列中的条数
#define Pw_Com2_Driver3_BufferNum w_ParLst_Drive[232] //3#命令队列中的条数
#define Pw_Com2_Driver4_BufferNum w_ParLst_Drive[233] //4#命令队列中的条数
#define Pw_Com3_Driver5_BufferNum w_ParLst_Drive[234] //5#命令队列中的条数
#define Pw_Com3_Driver6_BufferNum w_ParLst_Drive[235] //6#命令队列中的条数
#define Pw_StopStatus_Delay w_ParLst_Drive[236]       //停止状态检测延时
#define Pw_Acc_Delay_Ratio w_ParLst_Drive[237]        //加减速时间延时比例
#define Pw_Current_Run_Time w_ParLst_Drive[238]       //当前指令运行时间

#define Pw_Driver1_AutoSpeed w_ParLst_Drive[240] //1#当前运行速度
#define Pw_Driver2_AutoSpeed w_ParLst_Drive[241] //2#当前运行速度
#define Pw_Driver3_AutoSpeed w_ParLst_Drive[242] //3#当前运行速度
#define Pw_Driver4_AutoSpeed w_ParLst_Drive[243] //4#当前运行速度
#define Pw_Driver5_AutoSpeed w_ParLst_Drive[244] //5#当前运行速度
#define Pw_Driver6_AutoSpeed w_ParLst_Drive[245] //6#当前运行速度
#define Pw_EquipStatus w_ParLst_Drive[246]       //设备状态，低字
#define Pw_EquipStatus_HW w_ParLst_Drive[247]    //设备状态，高字
#define Pw_Run_Mode w_ParLst_Drive[248]          //运行模式，=0，喷漆模式；=1，清洗模式
#define Pw_JDQ_Addr w_ParLst_Drive[249]          //继电器板通讯地址

#define Pw_JDQ_Control w_ParLst_Drive[250]    //继电器板DO1控制字，=0断开；=1闭合
#define Pr_JDQ_Status w_ParLst_Drive[251]     //继电器板DO1状态字
#define Pr_Com4_ComCount w_ParLst_Drive[252]  //Com4通讯计数
#define Pw_Brake_Delay w_ParLst_Drive[253]    //开刹车延时
#define Pr_runtime_show w_ParLst_Drive[254]   //指令延时时间
#define Pr_pausetime_show w_ParLst_Drive[255] //指令暂停时间

#define Pr_cyclecounter_show w_ParLst_Drive[256] //程序循环计数
#define Pr_cyclecounter_HW w_ParLst_Drive[257]   //程序循环计算次数

#define Pw_Driver_Run_MinSpeed w_ParLst_Drive[258] //电机的最小运行速度，要比伺服设定的电机旋转检出阈值至少大10rpm

#define Pr_Driver1_NeverRun w_ParLst_Drive[259] //1#电机从没有运行标志
#define Pr_Driver2_NeverRun w_ParLst_Drive[260] //2#电机从没有运行标志
#define Pr_Driver3_NeverRun w_ParLst_Drive[261] //3#电机从没有运行标志
#define Pr_Driver4_NeverRun w_ParLst_Drive[262] //4#电机从没有运行标志
#define Pr_Driver5_NeverRun w_ParLst_Drive[263] //5#电机从没有运行标志
#define Pr_Driver6_NeverRun w_ParLst_Drive[264] //6#电机从没有运行标志

#define Pw_Driver1_PosErr_Muti w_ParLst_Drive[265] //1#电机编码器位置偏差，多圈
#define Pw_Driver2_PosErr_Muti w_ParLst_Drive[266] //2#电机编码器位置偏差，多圈
#define Pw_Driver3_PosErr_Muti w_ParLst_Drive[267] //3#电机编码器位置偏差，多圈
#define Pw_Driver4_PosErr_Muti w_ParLst_Drive[268] //4#电机编码器位置偏差，多圈
#define Pw_Driver5_PosErr_Muti w_ParLst_Drive[269] //5#电机编码器位置偏差，多圈
#define Pw_Driver6_PosErr_Muti w_ParLst_Drive[270] //6#电机编码器位置偏差，多圈

#define Pw_Driver1_PosErr_Sing w_ParLst_Drive[271]    //1#电机编码器位置偏差，单圈，低位
#define Pw_Driver1_PosErr_Sing_HW w_ParLst_Drive[272] //1#电机编码器位置偏差，单圈，高位
#define Pw_Driver2_PosErr_Sing w_ParLst_Drive[273]    //2#电机编码器位置偏差，单圈，低位
#define Pw_Driver2_PosErr_Sing_HW w_ParLst_Drive[274] //2#电机编码器位置偏差，单圈，高位
#define Pw_Driver3_PosErr_Sing w_ParLst_Drive[275]    //3#电机编码器位置偏差，单圈，低位
#define Pw_Driver3_PosErr_Sing_HW w_ParLst_Drive[276] //3#电机编码器位置偏差，单圈，高位
#define Pw_Driver4_PosErr_Sing w_ParLst_Drive[277]    //4#电机编码器位置偏差，单圈，低位
#define Pw_Driver4_PosErr_Sing_HW w_ParLst_Drive[278] //4#电机编码器位置偏差，单圈，高位
#define Pw_Driver5_PosErr_Sing w_ParLst_Drive[279]    //5#电机编码器位置偏差，单圈，低位
#define Pw_Driver5_PosErr_Sing_HW w_ParLst_Drive[280] //5#电机编码器位置偏差，单圈，高位
#define Pw_Driver6_PosErr_Sing w_ParLst_Drive[281]    //6#电机编码器位置偏差，单圈，低位
#define Pw_Driver6_PosErr_Sing_HW w_ParLst_Drive[282] //6#电机编码器位置偏差，单圈，高位

#define Pw_Pos_Adj_Cmd w_ParLst_Drive[283]       //电机编码器位置偏差手动调整命令
#define Pw_AllStopped_Delay w_ParLst_Drive[284]  //判断所有电机位置到达延时
#define Pw_Write_Timeout_Set w_ParLst_Drive[285] //写位置指令超时时间设定
#define Pr_Send_Data_F w_ParLst_Drive[286]       //发送数据标志
#define Pr_Driver1_Cmd_OK_F w_ParLst_Drive[287]  //1#伺服发送数据OK标志
#define Pr_Driver2_Cmd_OK_F w_ParLst_Drive[288]  //2#伺服发送数据OK标志
#define Pr_Driver3_Cmd_OK_F w_ParLst_Drive[289]  //3#伺服发送数据OK标志
#define Pr_Driver4_Cmd_OK_F w_ParLst_Drive[290]  //4#伺服发送数据OK标志

#define Pr_Driver5_Cmd_OK_F w_ParLst_Drive[291]     //5#伺服发送数据OK标志
#define Pr_Driver6_Cmd_OK_F w_ParLst_Drive[292]     //6#伺服发送数据OK标志
#define Pr_AllDriver_Cmd_OK_F w_ParLst_Drive[293]   //所有伺服发送数据OK标志
#define Pr_HaveDriver_Cmd_Err_F w_ParLst_Drive[294] //有伺服发送数据错误标志
#define Pr_OverMaxPos_F w_ParLst_Drive[295]         //超出位置范围标志

#endif
