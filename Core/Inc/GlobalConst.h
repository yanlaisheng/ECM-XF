/**
  ******************************************************************************
  * @file    GlobalConst.h
  * @author  ChengLei Zhou  - �ܳ���
  * @version V1.27
  * @date    2014-01-03
  * @brief   ȫ�ֱ�������
	******************************************************************************
	*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __GLOBALCONST_H
#define __GLOBALCONST_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f40x.h"

//�����ⲿ����洢��FM25CL16�ĵ�ַ����
//FM25CL64�洢оƬ��64Kbit����8K�ֽڣ����ڱ����趨������λ����Ϣ��λ����������
/*******
 * 0-599:���������������������ڱ����ڲ�����w_ParLst[300]��300���֣�ÿ������2���ֽڣ���600���ֽ�
 * 600-1799:���������������������ڱ����ڲ�����w_ParLst_Drive[300]��300��˫�֣�ÿ������4���ֽڣ���1200���ֽ�
 * 1800-2999:���������������������ڱ���λ�����ݣ�15����ÿ��20��˫�֣���300��˫�֣�1200���ֽ�
 * 3000-7799:���������������������ڱ���λ�����ݣ�30����ÿ��40��˫�֣���1200��˫�֣�4800���ֽ�
 */
#define PAR_SIZE_ALL 8192 //�������ܴ�С
#define PAR1_SIZE 300     // NV��������

#define FM_FLASH_PAR1_ADDR 0                                                    //������1��ʼ��ַ
#define FM_FLASH_PAR2_ADDR FM_FLASH_PAR1_ADDR + PAR1_SIZE * sizeof(w_ParLst[0]) //���=0+300*2=600

#define PAR2_SIZE 300
#define w_ParLst_DrivePar w_ParLst_Drive[0]
//λ�ò�����ʼ��ַ
#define FM_POS_ADDR FM_FLASH_PAR2_ADDR + PAR2_SIZE * sizeof(w_ParLst_DrivePar) //���=600+300*4=1800

#define POS_SIZE 20                      //ÿ�����ݵ�ĳ��ȣ����+6����Ȧ+12����Ȧ������ֵ+��־
#define POS_NUM 15                       //�������ݵ��������15��
#define FLASH_POS_SIZE POS_SIZE *POS_NUM //20*15
//λ��������ʼ��ַ
#define FM_POS_CMD_ADDR FM_POS_ADDR + FLASH_POS_SIZE * sizeof(w_ParLst_DrivePar) //���=1800+300*4=3000

#define POS_CMD_NUM 30                               //����λ��ָ������
#define POS_CMD_SIZE 40                              //����λ��ָ���
#define FLASH_POS_CMD_SIZE POS_CMD_SIZE *POS_CMD_NUM //���λ��=3000+1200*4=7800<8192

#define COM_CMD_SIZE 20 //����ָ���
#define COM_CMD_NUM 30  //����ָ������

#define PULSE_NUM 1000         //���ÿ��תһȦ��������
#define ELEC_GEAR 8388608      //���ӳ���2^23=8388608
#define ELEC_GEAR_US200 131072 //���ӳ���2^17=131072

//20*15=300��˫�֣�λ�����+6����Ȧ+12����Ȧ������ֵ+��־
//λ�ú�(0)
//1#��Ȧ(1)+1#��Ȧ_��(2)+1#��Ȧ_��(3)+2#��Ȧ(4)+2#��Ȧ_��(5)+2#��Ȧ_��(6)+
//3#��Ȧ(7)+3#��Ȧ_��(8)+3#��Ȧ_��(9)+4#��Ȧ(10)+4#��Ȧ_��(11)+4#��Ȧ_��(12)+
//5#��Ȧ(13)+5#��Ȧ_��(14)+5#��Ȧ_��(15)+6#��Ȧ(16)+6#��Ȧ_��(17)+6#��Ȧ_��(18)
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

//40*30=1200��˫��
//���(0)+�����(1)+λ�ú�(2)+�����ٶ�(3)+�Ӽ���ʱ��(4)+��ͣ(5)+ֹͣ(6)+����1(7)+����2(8)+DO��ʱ(9)+DO�������ʱ��(10)+���(11)
//+1#����_��(12)+1#����_��(13)+1#�ٶ�(14)+1#�Ӽ���(15)+2#����_��(16)+2#����_��(17)+2#�ٶ�(18)+2#�Ӽ���(19)
//+3#����_��(20)+3#����_��(21)+3#�ٶ�(22)+3#�Ӽ���(23)+4#����_��(24)+4#����_��(25)+4#�ٶ�(26)+4#�Ӽ���(27)
//+5#����_��(28)+5#����_��(29)+5#�ٶ�(30)+5#�Ӽ���(31)+6#����_��(32)+6#����_��(33)+6#�ٶ�(34)+6#�Ӽ���(35)
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
#define CMD_RUN_TIME 36 //��������ʱ�䣬������ָ������λ��36
//���һ������
#define w_ParLst_LastPos_CMD w_ParLst_Drive[PAR2_SIZE + FLASH_POS_SIZE + FLASH_POS_CMD_SIZE - POS_CMD_SIZE]

//�ⲿFLASH W25Q16��ַ����
//W25Q16 FLASH�洢оƬ��16Mbit����2M�ֽڣ�512��������ÿ������16ҳ��ÿҳ256�ֽڣ���ÿ������4K�ֽڣ�
//����ַ2*1024*1024=2097152����0x00200000
#define FLASH_SAVE_ADDR0 0X00000000    //���ܿ�����д������һ�α��浽һ��������ÿ������4096���ֽ�
#define FLASH_SAVE_POSTION1 0X00010000 //�����¼�ĸ���λ�õ㣬��ʽ�����+6���ؽڵĶ�Ȧ���ݼ���Ȧ����
#define FLASH_SAVE_POS_CMD1 0X00020000 //ͬ������ָ��ɱ���5��ָ��
#define FLASH_SAVE_POS_CMD2 0X00030000
#define FLASH_SAVE_POS_CMD3 0X00040000
#define FLASH_SAVE_POS_CMD4 0X00050000
#define FLASH_SAVE_POS_CMD5 0X00060000

//�ڲ�Flash�����ַ
//���һ��������128K
#define InFLASH_SAVE_ADDR0 0x080E0000                                                            //������1���������FLASH����ʼ��ַ
#define InFLASH_SAVE_ADDR1 InFLASH_SAVE_ADDR0 + PAR1_SIZE * sizeof(w_ParLst[0])                  //������2�����=0x080E0000 +300*2=0x080E0000 +600
#define InFLASH_SAVE_POSTION1 InFLASH_SAVE_ADDR1 + PAR2_SIZE * sizeof(w_ParLst_DrivePar)         //λ�����ݣ����=0x080E0000 +600+300*4=0x080E0000+1800
#define InFLASH_SAVE_POS_CMD1 InFLASH_SAVE_POSTION1 + FLASH_POS_SIZE * sizeof(w_ParLst_DrivePar) //ͬ������ָ����=0x080E0000 +1800+300*4=3000

#define SECTORWORDNUM 256       // ����������
#define FLASH_ID_SIZE 128       // ID��������
#define PROGRAM_LEN 0xE000      // 0-0xE000 ����ռ䳤��
#define FLASH_PAR_ADDR 0xE000   // NV������ַ
#define FLASH_ID_ADDR 0xE200    // NV �绰�Ȳ�����ַ
#define FLASH_FAULT_ADDR 0xE800 // ���ϼ�¼��
#define FLASH_REC_ADDR 0xEA00   // FLASH ��¼
#define FLASH_REC_SIZE 64       // һ��FLASH��¼�ĳ���(�ֽ�)
#define NORCVMAXMS 5            // 20 2007.7.5
#define SECTOR_SIZE 65536       // ��������
#define RDWR_SIZE 64            // ��дFLASH�ĳ���
#define ISL1208 0xDE            // Device address for chip A--ISL1208
#define AT24C256 0xA4           // Device address for chip B
#define CS_Flash1 1             // ƬѡFLASH1
#define CS_FMRAM1 2             // ƬѡFMRAM1
#define FLASH_REC_MAX 16384     // FLASH��¼�����
#define FAULT_REC_MAX 64        //���ϼ�¼�����

#define TXD1_MAX 255 // ���������
#define RCV1_MAX 255 // ���ջ��������� //256*8.5
#define TXD2_MAX 255 // ���������
#define RCV2_MAX 255 // ���ջ��������� //256*8.5
#define TXD3_MAX 255 // ���������
#define RCV3_MAX 255 // ���ջ��������� //256*8.5
#define TXD4_MAX 255 // ���������
#define RCV4_MAX 255 // ���ջ��������� //256*8.5
#define TXD5_MAX 255 // ���������
#define RCV5_MAX 255 // ���ջ��������� //256*8.5
#define TXD6_MAX 255 // ���������
#define RCV6_MAX 255 // ���ջ��������� //256*8.5

#define M1_SYNC_MASK 0x01
#define M2_SYNC_MASK 0x02
#define M3_SYNC_MASK 0x04
#define M4_SYNC_MASK 0x08
#define M5_SYNC_MASK 0x10
#define M6_SYNC_MASK 0x20
#define M6_SYNC_MASK_ALL 0x3F

// �ѡ�λ����ַ��λ��š�ת��������ַ��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
//�Ѹõ�ַת����һ��ָ��
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

//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) //���
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  //����

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) //���
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  //����

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n) //���
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)  //����

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n) //���
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)  //����

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n) //���
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)  //����

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n) //���
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)  //����

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n) //���
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)  //����

/* BSPӲ������  */
//----------------------------------------------------------------

//����������
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

//�̵������			��DSP�ϣ�ͨ�����DO1��DO2���Լ�ӿ��Ƽ̵��� 2015.5.10 ����DSP��λ�ȶ�������
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

#define STOP_BRAKE_SYSTEM HAL_GPIO_WritePin(GPIOA, DO1_Pin, GPIO_PIN_RESET) //ֹͣɲ������������
#define START_BRAKE_SYSTEM HAL_GPIO_WritePin(GPIOA, DO1_Pin, GPIO_PIN_SET)  //��ʼɲ��������������
#define Brake_Status_DO1 PAin(8)                                            //ɲ��״̬

#define OPEN_HAND HAL_GPIO_WritePin(GPIOC, DO2_Pin, GPIO_PIN_RESET) //�򿪻�е�ֱ�
#define CLOSE_HAND HAL_GPIO_WritePin(GPIOC, DO2_Pin, GPIO_PIN_SET)  //�պϻ�е�ֱ�
#define HAND_STATUS PCin(9)                                         //0�򿪵�ŷ���1�رյ�ŷ�

#define TIME_TEST_DEBUG 0 //������������ʱ��ĵ��Կ���,=1����print��������ʱ��
#define exFLASH_EXIST 0   //�ⲿFLASH�Ƿ���ڿ���,=1��ʾ�����ⲿFLASHоƬ;=1��ʾ���ⲿFLASH���豣�����ڲ�FLASH��
#define POS_TYPE 1        //=0��ʾλ����������Ϊ����������;=1��ʾλ����������Ϊ����ֵ

#define MOTOR1_AA_VALUE ((Motor1_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor1_XiShu_1 * M_T_AA * M_T_RA) * 60) //=14*60=840
#define MOTOR2_AA_VALUE ((Motor2_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor2_XiShu_1 * M_T_AA * M_T_RA) * 60)
#define MOTOR3_AA_VALUE ((Motor3_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor3_XiShu_1 * M_T_AA * M_T_RA) * 60)
#define MOTOR4_AA_VALUE ((Motor4_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor4_XiShu_1 * M_T_AA * M_T_RA) * 60)
#define MOTOR5_AA_VALUE ((Motor5_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor5_XiShu_1 * M_T_AA * M_T_RA) * 60)
#define MOTOR6_AA_VALUE ((Motor6_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor6_XiShu_1 * M_T_AA * M_T_RA) * 60)

#define DI_STABLE_NUM 20 // �������ȶ���Ŀ

//���ͬ������
#define TIME_COST_DELTA 32 * 20 //Ҫ�����ĵ������ʱ���������ʱ���ƫ��ֵ
#define FRE_AA_DELTA 1          //�Ӽ��ٶ�ƫ��ֵ
#define FRE_AA_MIN 1            //��С�Ӽ��ٶ�
#define FRE_START_DELTA 0.1     //��ʼƵ��ƫ��ֵ

// �����趨����
#define Pw_Motor1SendPulse w_ParLst[0]    // ���1����������
#define Pw_Motor1SendPulse_HW w_ParLst[1] // ���1��������������
#define Pw_Motor1_TurnAngle w_ParLst[2]   // ���1ת���Ƕ�
#define Pw_Motor1_ACCSpeed w_ParLst[3]    // ���1���ٶ�
#define Pw_Motor1_FRE_AA w_ParLst[4]      // ���1�Ӽ��ٶ�
#define Pw_Motor1_SetSpeed w_ParLst[5]    // ���1�趨�ٶ�
#define Pw_Motor1_StartSpeed w_ParLst[6]  // ���1��ʼ�ٶ�
#define Pw_Motor1_PULSENUM w_ParLst[7]    // ���1ÿȦ������
#define Pw_ModPar w_ParLst[9]             // �޸Ĳ���

#define Pw_Motor2SendPulse w_ParLst[15]    // ���2����������
#define Pw_Motor2SendPulse_HW w_ParLst[16] // ���2��������������
#define Pw_Motor2_TurnAngle w_ParLst[17]   // ���2ת���Ƕ�
#define Pw_Motor2_ACCSpeed w_ParLst[18]    // ���2���ٶ�
#define Pw_Motor2_FRE_AA w_ParLst[19]      // ���2�Ӽ��ٶ�
#define Pw_Motor2_SetSpeed w_ParLst[20]    // ���2�趨�ٶ�
#define Pw_Motor2_StartSpeed w_ParLst[21]  // ���2��ʼ�ٶ�
#define Pw_Motor2_PULSENUM w_ParLst[22]    // ���2ÿȦ������

#define Pw_Motor3SendPulse w_ParLst[25]    // ���3����������
#define Pw_Motor3SendPulse_HW w_ParLst[26] // ���3��������������
#define Pw_Motor3_TurnAngle w_ParLst[27]   // ���3ת���Ƕ�
#define Pw_Motor3_ACCSpeed w_ParLst[28]    // ���3���ٶ�
#define Pw_Motor3_FRE_AA w_ParLst[29]      // ���3�Ӽ��ٶ�
#define Pw_Motor3_SetSpeed w_ParLst[30]    // ���3�趨�ٶ�
#define Pw_Motor3_StartSpeed w_ParLst[31]  // ���3��ʼ�ٶ�
#define Pw_Motor3_PULSENUM w_ParLst[32]    // ���3ÿȦ������

#define Pw_Motor4SendPulse w_ParLst[35]    // ���4����������
#define Pw_Motor4SendPulse_HW w_ParLst[36] // ���4��������������
#define Pw_Motor4_TurnAngle w_ParLst[37]   // ���4ת���Ƕ�
#define Pw_Motor4_ACCSpeed w_ParLst[38]    // ���4���ٶ�
#define Pw_Motor4_FRE_AA w_ParLst[39]      // ���4�Ӽ��ٶ�
#define Pw_Motor4_SetSpeed w_ParLst[40]    // ���4�趨�ٶ�
#define Pw_Motor4_StartSpeed w_ParLst[41]  // ���4��ʼ�ٶ�
#define Pw_Motor4_PULSENUM w_ParLst[42]    // ���4ÿȦ������

#define Pw_Motor5SendPulse w_ParLst[45]    // ���5����������
#define Pw_Motor5SendPulse_HW w_ParLst[46] // ���5��������������
#define Pw_Motor5_TurnAngle w_ParLst[47]   // ���5ת���Ƕ�
#define Pw_Motor5_ACCSpeed w_ParLst[48]    // ���5���ٶ�
#define Pw_Motor5_FRE_AA w_ParLst[49]      // ���5�Ӽ��ٶ�
#define Pw_Motor5_SetSpeed w_ParLst[50]    // ���5�趨�ٶ�
#define Pw_Motor5_StartSpeed w_ParLst[51]  // ���5��ʼ�ٶ�
#define Pw_Motor5_PULSENUM w_ParLst[52]    // ���5ÿȦ������

#define Pw_Motor6SendPulse w_ParLst[55]    // ���6����������
#define Pw_Motor6SendPulse_HW w_ParLst[56] // ���6��������������
#define Pw_Motor6_TurnAngle w_ParLst[57]   // ���6ת���Ƕ�
#define Pw_Motor6_ACCSpeed w_ParLst[58]    // ���6���ٶ�
#define Pw_Motor6_FRE_AA w_ParLst[59]      // ���6�Ӽ��ٶ�
#define Pw_Motor6_SetSpeed w_ParLst[60]    // ���6�趨�ٶ�
#define Pw_Motor6_StartSpeed w_ParLst[61]  // ���6��ʼ�ٶ�
#define Pw_Motor6_PULSENUM w_ParLst[62]    // ���6ÿȦ������

#define Pw_Initial_F w_ParLst[65]           // ��ʼ����־��=0x5A����ʾ�Ѿ���ʼ��
#define Pw_ParInitial w_ParLst[66]          // ������ʼ������
#define Pos_Group_Select w_ParLst[67]       // ����λ�ÿ���������ѡ��1-5�飩
#define Pw_FRam_Read_StartAdrr w_ParLst[68] //��ȡFRAM��ʼ��ַ
#define Pw_FRam_Read_DataLen w_ParLst[69]   //��ȡFRAM���ݳ���

#define Pw_SetSecond w_ParLst[70] // ������	// ע��:��ַ�������Ķ�,������ʹ��  ZCL
#define Pw_SetMinute w_ParLst[71] // ���÷�
#define Pw_SetHour w_ParLst[72]   // ����ʱ
#define Pw_SetDay w_ParLst[73]    // ������
#define Pw_SetMonth w_ParLst[74]  // ������
#define Pw_SetYear w_ParLst[75]   // ������
#define Pw_SetWeek w_ParLst[76]   // ��������

#define Pw_DIStableSetNum w_ParLst[115]       // �������ȶ���Ŀ
#define Pr_Driver1_Control_OK_F w_ParLst[116] //1#�ŷ���������OK��־
#define Pr_Driver2_Control_OK_F w_ParLst[117] //2#�ŷ���������OK��־
#define Pr_Driver3_Control_OK_F w_ParLst[118] //3#�ŷ���������OK��־
#define Pr_Driver4_Control_OK_F w_ParLst[119] //4#�ŷ���������OK��־
#define Pr_Driver5_Control_OK_F w_ParLst[120] //5#�ŷ���������OK��־
#define Pr_Driver6_Control_OK_F w_ParLst[121] //6#�ŷ���������OK��־

#define Pw_SendPWM w_ParLst[122]       // ��PWM������
#define Pw_S_ParaInitial w_ParLst[123] // S���߳�ʼ������

#define Pw_BaudRate1 w_ParLst[141] // ������1
#define Pw_BaudRate2 w_ParLst[142] // ������2
#define Pw_BaudRate3 w_ParLst[143] // ������3
#define Pw_BaudRate4 w_ParLst[144] // ������4
#define Pw_BaudRate5 w_ParLst[145] // ������5
#define Pw_BaudRate6 w_ParLst[146] // ������6

#define Pw_EquipmentNo1 w_ParLst[159] //1#�ŷ����Modbus��ַ
#define Pw_EquipmentNo2 w_ParLst[160] //2#�ŷ����Modbus��ַ
#define Pw_EquipmentNo3 w_ParLst[161] //3#�ŷ����Modbus��ַ
#define Pw_EquipmentNo4 w_ParLst[162] //4#�ŷ����Modbus��ַ
#define Pw_EquipmentNo5 w_ParLst[163] //5#�ŷ����Modbus��ַ
#define Pw_EquipmentNo6 w_ParLst[164] //6#�ŷ����Modbus��ַ

#define Pr_RUN_Count w_ParLst[179]          //����ָ�����
#define Pr_RUN_Count_Set w_ParLst[180]      //���ж���Ȧ�趨����ʼ��Ϊ0
#define Pw_Total_RUN_Count w_ParLst[181]    //�ۼ�����Ȧ��������ֵΪ0
#define Pw_Total_RUN_Count_HW w_ParLst[182] //�ۼ�����Ȧ��_����
#define Pw_ComWriteErr_Stop w_ParLst[184]   //ͨѶ����д�����ͣ�����ܣ�=1��ͣ����=0����ͣ��

#define Pw_ComErr_Stop w_ParLst[222]  //ͨѶ����ͣ�����ܣ�=1��ͣ����=0����ͣ��
#define Pr_F_HaveFault w_ParLst[223]  //�е�����ϱ�־
#define Pw_Limit_MaxPos w_ParLst[224] //���λ�����ƹ��ܣ�=1�����ã�=0��������

#define Pr_F_ComErr w_ParLst[225]       //��ͨѶ���ϱ�־
#define Pr_Driver1_ComErr w_ParLst[226] //Driver1ͨѶ���ϱ�־
#define Pr_Driver2_ComErr w_ParLst[227] //Driver2ͨѶ���ϱ�־
#define Pr_Driver3_ComErr w_ParLst[228] //Driver3ͨѶ���ϱ�־
#define Pr_Driver4_ComErr w_ParLst[229] //Driver4ͨѶ���ϱ�־
#define Pr_Driver5_ComErr w_ParLst[230] //Driver5ͨѶ���ϱ�־
#define Pr_Driver6_ComErr w_ParLst[231] //Driver6ͨѶ���ϱ�־

#define Pr_BRAKE_Status w_ParLst[244]  //ɲ��״̬
#define Pr_BRAKE_Control w_ParLst[245] //ɲ�����ƣ�=1��ɲ����=0����ɲ��
#define Pr_F_AllStopped w_ParLst[246]  //���е����ֹͣ��־

#define Pr_AllRun w_ParLst[249] //���е�������б�־
#define w_SoftVer w_ParLst[250] // ����汾

//���ʱ��
#define SClk1Ms SoftClock[0]    // ���ʱ�� 1ms
#define SClk10Ms SoftClock[1]   // ���ʱ�� 10ms
#define SClkSecond SoftClock[2] // ���ʱ��  s
#define SClkMinute SoftClock[3] // ���ʱ��  m
#define SClkHour SoftClock[4]   // ���ʱ��  h
#define SClkDay SoftClock[5]    // ���ʱ��  d
#define SClkMonth SoftClock[6]  // ���ʱ��  m
#define SClkYear SoftClock[7]   // ���ʱ��  y
#define SClk0r5Ms SoftClock[8]  // ���ʱ��  0.5MS	2010.8.6 ��ʱ�ж��вɼ�����ֵ

#define RealSecond RealClock[0] // ʵʱʱ��
#define RealMinute RealClock[1] // ʵʱʱ��
#define RealHour RealClock[2]   // ʵʱʱ��
#define RealDay RealClock[3]    // ʵʱʱ��
#define RealMonth RealClock[4]  // ʵʱʱ��
#define RealYear RealClock[5]   // ʵʱʱ��

//����ͨѶ�����������
#define COM_QUERY_SIZE 10 //��ѯָ���
#define COM_QUERY_NUM 10  //��ѯָ������

#define MOTOR_MAX_SPEED12 1500  //1#��2#������ת��1500r/min
#define MOTOR_MAX_SPEED3 2000   //3#������ת��2000r/min
#define MOTOR_MAX_SPEED456 3000 //3#������ת��3000r/min

// �����趨����
#define Pw_Driver1_Pluse w_ParLst_Drive[0]     //1#�ֶ��������壨���֣�
#define Pw_Driver1_Pluse_HW w_ParLst_Drive[1]  //1#�ֶ��������壨���֣�
#define Pw_Driver2_Pluse w_ParLst_Drive[2]     //2#�ֶ��������壨���֣�
#define Pw_Driver2_Pluse_HW w_ParLst_Drive[3]  //2#�ֶ��������壨���֣�
#define Pw_Driver3_Pluse w_ParLst_Drive[4]     //3#�ֶ��������壨���֣�
#define Pw_Driver3_Pluse_HW w_ParLst_Drive[5]  //3#�ֶ��������壨���֣�
#define Pw_Driver4_Pluse w_ParLst_Drive[6]     //4#�ֶ��������壨���֣�
#define Pw_Driver4_Pluse_HW w_ParLst_Drive[7]  //4#�ֶ��������壨���֣�
#define Pw_Driver5_Pluse w_ParLst_Drive[8]     //5#�ֶ��������壨���֣�
#define Pw_Driver5_Pluse_HW w_ParLst_Drive[9]  //5#�ֶ��������壨���֣�
#define Pw_Driver6_Pluse w_ParLst_Drive[10]    //6#�ֶ��������壨���֣�
#define Pw_Driver6_Pluse_HW w_ParLst_Drive[11] //6#�ֶ��������壨���֣�

#define Pr_Drive1_Status1 w_ParLst_Drive[12] //1#�ŷ����״̬�֣���Ӧ�ŷ���ַP8901
#define Pr_Drive2_Status1 w_ParLst_Drive[13] //2#�ŷ����״̬��
#define Pr_Drive3_Status1 w_ParLst_Drive[14] //3#�ŷ����״̬��
#define Pr_Drive4_Status1 w_ParLst_Drive[15] //4#�ŷ����״̬��
#define Pr_Drive5_Status1 w_ParLst_Drive[16] //5#�ŷ����״̬��
#define Pr_Drive6_Status1 w_ParLst_Drive[17] //6#�ŷ����״̬��

#define Pw_Driver1_R_Enable w_ParLst_Drive[21] //1#�ֶ�����ʹ�ܣ���ת��
#define Pw_Driver2_R_Enable w_ParLst_Drive[22] //2#�ֶ�����ʹ�ܣ���ת��
#define Pw_Driver3_R_Enable w_ParLst_Drive[23] //3#�ֶ�����ʹ�ܣ���ת��
#define Pw_Driver4_R_Enable w_ParLst_Drive[24] //4#�ֶ�����ʹ�ܣ���ת��
#define Pw_Driver5_R_Enable w_ParLst_Drive[25] //5#�ֶ�����ʹ�ܣ���ת��
#define Pw_Driver6_R_Enable w_ParLst_Drive[26] //6#�ֶ�����ʹ�ܣ���ת��

#define Pw_Driver1_Speed w_ParLst_Drive[27]      //1#�ֶ������ٶ�
#define Pw_Driver1_AccTime w_ParLst_Drive[28]    //1#�ֶ��Ӽ��٣�С���Ӽ��������󣬼Ӽ��ٿ�
#define Pw_Driver1_StartSpeed w_ParLst_Drive[29] //1#�ֶ���ʼ�����ٶ�

#define Pw_Driver2_Speed w_ParLst_Drive[30]      //2#�ֶ������ٶ�
#define Pw_Driver2_AccTime w_ParLst_Drive[31]    //2#�ֶ��Ӽ��٣�С���Ӽ��������󣬼Ӽ��ٿ�
#define Pw_Driver2_StartSpeed w_ParLst_Drive[32] //2#�ֶ���ʼ�����ٶ�

#define Pw_Driver3_Speed w_ParLst_Drive[33]      //3#�ֶ������ٶ�
#define Pw_Driver3_AccTime w_ParLst_Drive[34]    //3#�ֶ��Ӽ��٣�С���Ӽ��������󣬼Ӽ��ٿ�
#define Pw_Driver3_StartSpeed w_ParLst_Drive[35] //3#�ֶ���ʼ�����ٶ�

#define Pw_Driver4_Speed w_ParLst_Drive[36]      //4#�ֶ������ٶ�
#define Pw_Driver4_AccTime w_ParLst_Drive[37]    //4#�ֶ��Ӽ��٣�С���Ӽ��������󣬼Ӽ��ٿ�
#define Pw_Driver4_StartSpeed w_ParLst_Drive[38] //4#�ֶ���ʼ�����ٶ�

#define Pw_Driver5_Speed w_ParLst_Drive[39]      //5#�ֶ������ٶ�
#define Pw_Driver5_AccTime w_ParLst_Drive[40]    //5#�ֶ��Ӽ��٣�С���Ӽ��������󣬼Ӽ��ٿ�
#define Pw_Driver5_StartSpeed w_ParLst_Drive[41] //5#�ֶ���ʼ�����ٶ�

#define Pw_Driver6_Speed w_ParLst_Drive[42]      //6#�ֶ������ٶ�
#define Pw_Driver6_AccTime w_ParLst_Drive[43]    //6#�ֶ��Ӽ��٣�С���Ӽ��������󣬼Ӽ��ٿ�
#define Pw_Driver6_StartSpeed w_ParLst_Drive[20] //6#�ֶ���ʼ�����ٶ�

#define Pw_Driver1_Enable w_ParLst_Drive[44] //1#�ֶ�����ʹ�ܣ���ת��
#define Pw_Driver2_Enable w_ParLst_Drive[45] //2#�ֶ�����ʹ�ܣ���ת��
#define Pw_Driver3_Enable w_ParLst_Drive[46] //3#�ֶ�����ʹ�ܣ���ת��
#define Pw_Driver4_Enable w_ParLst_Drive[47] //4#�ֶ�����ʹ�ܣ���ת��
#define Pw_Driver5_Enable w_ParLst_Drive[48] //5#�ֶ�����ʹ�ܣ���ת��
#define Pw_Driver6_Enable w_ParLst_Drive[49] //6#�ֶ�����ʹ�ܣ���ת��

#define Pw_StepAutoMode w_ParLst_Drive[50] //=1���ֶ�ģʽ��=0��ȫ�Զ�ģʽ

#define Pw_Driver1_SetValue w_ParLst_Drive[51]    //1#�ŷ�����趨ֵ
#define Pw_Driver1_SetValue_HW w_ParLst_Drive[52] //1#�ŷ�����趨ֵ�����֣�
#define Pw_Driver2_SetValue w_ParLst_Drive[53]    //1#�ŷ�����趨ֵ
#define Pw_Driver2_SetValue_HW w_ParLst_Drive[54] //1#�ŷ�����趨ֵ�����֣�
#define Pw_Driver3_SetValue w_ParLst_Drive[55]    //1#�ŷ�����趨ֵ
#define Pw_Driver3_SetValue_HW w_ParLst_Drive[56] //1#�ŷ�����趨ֵ�����֣�
#define Pw_Driver4_SetValue w_ParLst_Drive[57]    //1#�ŷ�����趨ֵ
#define Pw_Driver4_SetValue_HW w_ParLst_Drive[58] //1#�ŷ�����趨ֵ�����֣�
#define Pw_Driver5_SetValue w_ParLst_Drive[59]    //1#�ŷ�����趨ֵ
#define Pw_Driver5_SetValue_HW w_ParLst_Drive[60] //1#�ŷ�����趨ֵ�����֣�
#define Pw_Driver6_SetValue w_ParLst_Drive[61]    //1#�ŷ�����趨ֵ
#define Pw_Driver6_SetValue_HW w_ParLst_Drive[62] //1#�ŷ�����趨ֵ�����֣�

#define Pw_SF_Driver1_InitPos w_ParLst_Drive[63]    //�ŷ����1�ĳ�ʼλ��
#define Pw_SF_Driver1_InitPos_HW w_ParLst_Drive[64] //�ŷ����1�ĳ�ʼλ�ã����֣�
#define Pw_SF_Driver2_InitPos w_ParLst_Drive[65]    //�ŷ����2�ĳ�ʼλ��
#define Pw_SF_Driver2_InitPos_HW w_ParLst_Drive[66] //�ŷ����2�ĳ�ʼλ�ã����֣�
#define Pw_SF_Driver3_InitPos w_ParLst_Drive[67]    //�ŷ����3�ĳ�ʼλ��
#define Pw_SF_Driver3_InitPos_HW w_ParLst_Drive[67] //�ŷ����3�ĳ�ʼλ�ã����֣�
#define Pw_SF_Driver4_InitPos w_ParLst_Drive[69]    //�ŷ����4�ĳ�ʼλ��
#define Pw_SF_Driver4_InitPos_HW w_ParLst_Drive[70] //�ŷ����4�ĳ�ʼλ�ã����֣�
#define Pw_SF_Driver5_InitPos w_ParLst_Drive[71]    //�ŷ����5�ĳ�ʼλ��
#define Pw_SF_Driver5_InitPos_HW w_ParLst_Drive[72] //�ŷ����5�ĳ�ʼλ�ã����֣�
#define Pw_SF_Driver6_InitPos w_ParLst_Drive[73]    //�ŷ����6�ĳ�ʼλ��
#define Pw_SF_Driver6_InitPos_HW w_ParLst_Drive[74] //�ŷ����6�ĳ�ʼλ�ã����֣�

#define Pw_Fault_Stop w_ParLst_Drive[75]      // �����ŷ�����ͣ�����ܣ�=1��ͣ����=0����ͣ������Ȼ����
#define Pw_Read_CurrentPos w_ParLst_Drive[76] // ����ʼλ�ã�=1����λ��
#define Pw_EMERGENCY_STOP w_ParLst_Drive[77]  // ��ͣ���=1�����м�ͣ
#define Pw_ResetCMD w_ParLst_Drive[78]        // ��λ���=1�����и�λ
#define Pw_Stop_Reset w_ParLst_Drive[79]      // ͣ����λ���ܣ�=1��ͣ������и�λ��=0��ͣ���󲻸�λ

#define Pw_TouchRunStop w_ParLst_Drive[80] // ���� ����/ֹͣ
#define Pw_SaveDelay w_ParLst_Drive[81]    //Ĭ����ʱ180s�����޸ĺ�Ĳ�����FLASH��

#define Pr_F_Drive1_Stop w_ParLst_Drive[102] //1#���ֹͣ��־��λ�õ��
#define Pr_F_Drive2_Stop w_ParLst_Drive[103] //2#���ֹͣ��־��λ�õ��
#define Pr_F_Drive3_Stop w_ParLst_Drive[104] //3#���ֹͣ��־��λ�õ��
#define Pr_F_Drive4_Stop w_ParLst_Drive[105] //4#���ֹͣ��־��λ�õ��
#define Pr_F_Drive5_Stop w_ParLst_Drive[106] //5#���ֹͣ��־��λ�õ��
#define Pr_F_Drive6_Stop w_ParLst_Drive[107] //6#���ֹͣ��־��λ�õ��

#define Pw_ComErrCount w_ParLst_Drive[108] //ͨѶ������ʱ�ж�

#define Pr_Driver_Running_No w_ParLst_Drive[111]  //��ǰִ�е��������
#define Pr_Driver_Previous_No w_ParLst_Drive[112] //ǰһ��ִ�е��������
//#define	Pr_Drive3_Run_No			w_ParLst_Drive[113]			//3#�����ǰִ�е��������
//#define	Pr_Drive4_Run_No			w_ParLst_Drive[114]			//4#�����ǰִ�е��������
//#define	Pr_Drive5_Run_No			w_ParLst_Drive[115]			//5#�����ǰִ�е��������
//#define	Pr_Drive6_Run_No			w_ParLst_Drive[116]			//6#�����ǰִ�е��������

#define Pr_F_Drive1_Runing w_ParLst_Drive[117] //1#������б�־
#define Pr_F_Drive2_Runing w_ParLst_Drive[118] //2#������б�־
#define Pr_F_Drive3_Runing w_ParLst_Drive[119] //3#������б�־
#define Pr_F_Drive4_Runing w_ParLst_Drive[120] //4#������б�־
#define Pr_F_Drive5_Runing w_ParLst_Drive[121] //5#������б�־
#define Pr_F_Drive6_Runing w_ParLst_Drive[122] //6#������б�־

#define Pr_F_Driver1_Rdy w_ParLst_Drive[123] //1#�ŷ����׼���ñ�־
#define Pr_F_Driver2_Rdy w_ParLst_Drive[124] //2#�ŷ����׼���ñ�־
#define Pr_F_Driver3_Rdy w_ParLst_Drive[125] //3#�ŷ����׼���ñ�־
#define Pr_F_Driver4_Rdy w_ParLst_Drive[126] //4#�ŷ����׼���ñ�־
#define Pr_F_Driver5_Rdy w_ParLst_Drive[127] //5#�ŷ����׼���ñ�־
#define Pr_F_Driver6_Rdy w_ParLst_Drive[128] //6#�ŷ����׼���ñ�־

#define Pr_Driver1_ComCount w_ParLst_Drive[129] //1#�ŷ����ͨѶ����
#define Pr_Driver2_ComCount w_ParLst_Drive[130] //2#�ŷ����ͨѶ����
#define Pr_Driver3_ComCount w_ParLst_Drive[131] //3#�ŷ����ͨѶ����
#define Pr_Driver4_ComCount w_ParLst_Drive[132] //4#�ŷ����ͨѶ����
#define Pr_Driver5_ComCount w_ParLst_Drive[133] //5#�ŷ����ͨѶ����
#define Pr_Driver6_ComCount w_ParLst_Drive[134] //6#�ŷ����ͨѶ����

#define Pr_Drive1_Status w_ParLst_Drive[135] //1#�ŷ����״̬�֣���Ӧ�ŷ���ַP8902
#define Pr_Drive2_Status w_ParLst_Drive[136] //2#�ŷ����״̬��
#define Pr_Drive3_Status w_ParLst_Drive[137] //3#�ŷ����״̬��
#define Pr_Drive4_Status w_ParLst_Drive[138] //4#�ŷ����״̬��
#define Pr_Drive5_Status w_ParLst_Drive[139] //5#�ŷ����״̬��
#define Pr_Drive6_Status w_ParLst_Drive[140] //6#�ŷ����״̬��

#define Pr_F_Drvier1_Err w_ParLst_Drive[141] //1#�ŷ�������ϱ�־
#define Pr_F_Drvier2_Err w_ParLst_Drive[142] //2#�ŷ�������ϱ�־
#define Pr_F_Drvier3_Err w_ParLst_Drive[143] //3#�ŷ�������ϱ�־
#define Pr_F_Drvier4_Err w_ParLst_Drive[144] //4#�ŷ�������ϱ�־
#define Pr_F_Drvier5_Err w_ParLst_Drive[145] //5#�ŷ�������ϱ�־
#define Pr_F_Drvier6_Err w_ParLst_Drive[146] //6#�ŷ�������ϱ�־

#define Pr_Drive1_FaultNo w_ParLst_Drive[147] //1#�ŷ������ǰ��߼�������룬��Ӧ�ŷ���ַP0931
#define Pr_Drive2_FaultNo w_ParLst_Drive[148] //2#�ŷ������ǰ��߼��������
#define Pr_Drive3_FaultNo w_ParLst_Drive[149] //3#�ŷ������ǰ��߼��������
#define Pr_Drive4_FaultNo w_ParLst_Drive[150] //4#�ŷ������ǰ��߼��������
#define Pr_Drive5_FaultNo w_ParLst_Drive[151] //5#�ŷ������ǰ��߼��������
#define Pr_Drive6_FaultNo w_ParLst_Drive[152] //6#�ŷ������ǰ��߼��������

#define Pw_Com_Delay1 w_ParLst_Drive[153] //COM��ʱ1
#define Pw_Com_Delay2 w_ParLst_Drive[154] //COM��ʱ2
//#define	Pw_Com2_Delay1						w_ParLst_Drive[155]			//COM2��ʱ1
//#define	Pw_Com2_Delay2						w_ParLst_Drive[156]			//COM2��ʱ2
//#define	Pw_Com3_Delay1						w_ParLst_Drive[157]			//COM3��ʱ1
//#define	Pw_Com3_Delay2						w_ParLst_Drive[158]			//COM3��ʱ2
//#define	Pw_Com3_Delay3						w_ParLst_Drive[159]			//COM3��ʱ3
//#define	Pw_Com4_Delay2						w_ParLst_Drive[160]			//COM4��ʱ2

#define Pr_Drive1_singleData w_ParLst_Drive[161]    //1#�ŷ������������Ȧ���ݣ���Ӧ�ŷ���ַP091A
#define Pr_Drive1_singleData_HW w_ParLst_Drive[162] //1#�ŷ������������Ȧ���ݣ����֣�
#define Pr_Drive2_singleData w_ParLst_Drive[163]    //2#�ŷ������������Ȧ���ݣ���Ӧ�ŷ���ַP091A
#define Pr_Drive2_singleData_HW w_ParLst_Drive[164] //2#�ŷ������������Ȧ���ݣ����֣�
#define Pr_Drive3_singleData w_ParLst_Drive[165]    //3#�ŷ������������Ȧ���ݣ���Ӧ�ŷ���ַP091A
#define Pr_Drive3_singleData_HW w_ParLst_Drive[166] //3#�ŷ������������Ȧ���ݣ����֣�
#define Pr_Drive4_singleData w_ParLst_Drive[167]    //4#�ŷ������������Ȧ���ݣ���Ӧ�ŷ���ַP091A
#define Pr_Drive4_singleData_HW w_ParLst_Drive[168] //4#�ŷ������������Ȧ���ݣ����֣�
#define Pr_Drive5_singleData w_ParLst_Drive[169]    //5#�ŷ������������Ȧ���ݣ���Ӧ�ŷ���ַP091A
#define Pr_Drive5_singleData_HW w_ParLst_Drive[170] //5#�ŷ������������Ȧ���ݣ����֣�
#define Pr_Drive6_singleData w_ParLst_Drive[171]    //6#�ŷ������������Ȧ���ݣ���Ӧ�ŷ���ַP091A
#define Pr_Drive6_singleData_HW w_ParLst_Drive[172] //6#�ŷ������������Ȧ���ݣ����֣�

#define Pr_Drive1_MultiData w_ParLst_Drive[173] //1#�ŷ������������Ȧ���ݣ���Ӧ�ŷ���ַP091C
#define Pr_Drive2_MultiData w_ParLst_Drive[174] //2#�ŷ������������Ȧ���ݣ���Ӧ�ŷ���ַP091C
#define Pr_Drive3_MultiData w_ParLst_Drive[175] //3#�ŷ������������Ȧ���ݣ���Ӧ�ŷ���ַP091C
#define Pr_Drive4_MultiData w_ParLst_Drive[176] //4#�ŷ������������Ȧ���ݣ���Ӧ�ŷ���ַP091C
#define Pr_Drive5_MultiData w_ParLst_Drive[177] //5#�ŷ������������Ȧ���ݣ���Ӧ�ŷ���ַP091C
#define Pr_Drive6_MultiData w_ParLst_Drive[178] //6#�ŷ������������Ȧ���ݣ���Ӧ�ŷ���ַP091C

#define Pr_Drive1_singleData_Init w_ParLst_Drive[179]    //1#�ŷ������������Ȧ��ʼ�趨ֵ����Ӧ�ŷ���ַP091A
#define Pr_Drive1_singleData_Init_HW w_ParLst_Drive[180] //1#�ŷ������������Ȧ���趨ֵ�����֣�
#define Pr_Drive2_singleData_Init w_ParLst_Drive[181]    //2#�ŷ������������Ȧ��ʼ�趨ֵ����Ӧ�ŷ���ַP091A
#define Pr_Drive2_singleData_Init_HW w_ParLst_Drive[182] //2#�ŷ������������Ȧ��ʼ�趨ֵ�����֣�
#define Pr_Drive3_singleData_Init w_ParLst_Drive[183]    //3#�ŷ������������Ȧ��ʼ�趨ֵ����Ӧ�ŷ���ַP091A
#define Pr_Drive3_singleData_Init_HW w_ParLst_Drive[184] //3#�ŷ������������Ȧ��ʼ�趨ֵ�����֣�
#define Pr_Drive4_singleData_Init w_ParLst_Drive[185]    //4#�ŷ������������Ȧ��ʼ�趨ֵ����Ӧ�ŷ���ַP091A
#define Pr_Drive4_singleData_Init_HW w_ParLst_Drive[186] //4#�ŷ������������Ȧ��ʼ�趨ֵ�����֣�
#define Pr_Drive5_singleData_Init w_ParLst_Drive[187]    //5#�ŷ������������Ȧ��ʼ�趨ֵ����Ӧ�ŷ���ַP091A
#define Pr_Drive5_singleData_Init_HW w_ParLst_Drive[188] //5#�ŷ������������Ȧ��ʼ�趨ֵ�����֣�
#define Pr_Drive6_singleData_Init w_ParLst_Drive[189]    //6#�ŷ������������Ȧ��ʼ�趨ֵ����Ӧ�ŷ���ַP091A
#define Pr_Drive6_singleData_Init_HW w_ParLst_Drive[190] //6#�ŷ������������Ȧ��ʼ�趨ֵ�����֣�

#define Pr_Drive1_MultiData_Init w_ParLst_Drive[191] //1#�ŷ������������Ȧ��ʼ�趨ֵ����Ӧ�ŷ���ַP091C
#define Pr_Drive2_MultiData_Init w_ParLst_Drive[192] //2#�ŷ������������Ȧ��ʼ�趨ֵ����Ӧ�ŷ���ַP091C
#define Pr_Drive3_MultiData_Init w_ParLst_Drive[193] //3#�ŷ������������Ȧ��ʼ�趨ֵ����Ӧ�ŷ���ַP091C
#define Pr_Drive4_MultiData_Init w_ParLst_Drive[194] //4#�ŷ������������Ȧ��ʼ�趨ֵ����Ӧ�ŷ���ַP091C
#define Pr_Drive5_MultiData_Init w_ParLst_Drive[195] //5#�ŷ������������Ȧ��ʼ�趨ֵ����Ӧ�ŷ���ַP091C
#define Pr_Drive6_MultiData_Init w_ParLst_Drive[196] //6#�ŷ������������Ȧ��ʼ�趨ֵ����Ӧ�ŷ���ַP091C

#define Pr_Reset_Delay w_ParLst_Drive[197] //��λ����ʼλ����ʱʱ�䣬ms

#define Pw_PosError_Set w_ParLst_Drive[198]    //λ��ƫ���趨
#define Pw_PosError_Set_HW w_ParLst_Drive[199] //λ��ƫ���趨�����֣�

//#define Pw_Drive1_P8910						w_ParLst_Drive[201]			//1#�ŷ����P8910����
//#define Pw_Drive2_P8910						w_ParLst_Drive[202]			//2#�ŷ����P8910����
//#define Pw_Drive3_P8910						w_ParLst_Drive[203]			//3#�ŷ����P8910����
//#define Pw_Drive4_P8910						w_ParLst_Drive[204]			//4#�ŷ����P8910����
//#define Pw_Drive5_P8910						w_ParLst_Drive[205]			//5#�ŷ����P8910����
//#define Pw_Drive6_P8910						w_ParLst_Drive[206]			//6#�ŷ����P8910����

//#define Pw_Driver_AllSavePos_Enable			w_ParLst_Drive[207]			//�����ŷ����дλ�ò���ʹ��
//#define Pw_Driver1_SavePos_Enable			w_ParLst_Drive[208]			//1#�ŷ����дλ�ò���ʹ��
//#define Pw_Driver2_SavePos_Enable			w_ParLst_Drive[209]			//2#�ŷ����дλ�ò���ʹ��
//#define Pw_Driver3_SavePos_Enable			w_ParLst_Drive[210]			//3#�ŷ����дλ�ò���ʹ��
//#define Pw_Driver4_SavePos_Enable			w_ParLst_Drive[211]			//4#�ŷ����дλ�ò���ʹ��
//#define Pw_Driver5_SavePos_Enable			w_ParLst_Drive[212]			//5#�ŷ����дλ�ò���ʹ��
//#define Pw_Driver6_SavePos_Enable			w_ParLst_Drive[213]			//6#�ŷ����дλ�ò���ʹ��

#define Pw_Set_Run_Speed1 w_ParLst_Drive[214] //1#�ŷ�����趨��������ٶ�
#define Pw_Set_Run_Speed2 w_ParLst_Drive[215] //2#�ŷ�����趨��������ٶ�
#define Pw_Set_Run_Speed3 w_ParLst_Drive[216] //3#�ŷ�����趨��������ٶ�
#define Pw_Set_Run_Speed4 w_ParLst_Drive[217] //4#�ŷ�����趨��������ٶ�
#define Pw_Set_Run_Speed5 w_ParLst_Drive[218] //5#�ŷ�����趨��������ٶ�
#define Pw_Set_Run_Speed6 w_ParLst_Drive[219] //6#�ŷ�����趨��������ٶ�

#define Pw_Com_Delay3 w_ParLst_Drive[220]       //COM��ʱ3
#define Pw_Com_Delay_Manual w_ParLst_Drive[221] //COM��ʱ���ֶ�

#define Pw_Define_Save_Pos w_ParLst_Drive[222] //���岢��¼λ������
#define Pw_Cal_Pos_CMD w_ParLst_Drive[223]     //����λ�ü��ٶ�����
#define Pw_Verify_Pos_CMD w_ParLst_Drive[224]  //У��λ������
#define Pw_Running_Pos_CMD w_ParLst_Drive[225] //�������е������
#define Pw_Step_Pos_CMD w_ParLst_Drive[226]    //������������
#define Pw_Current_Pos_No w_ParLst_Drive[227]  //��ǰλ�ú�[1-15]
#define Pw_Set_Run_Speed w_ParLst_Drive[228]   //��׼�����ٶ�
#define Pw_Read_Init_Pos w_ParLst_Drive[229]   //����ԭ��λ������

#define Pw_Com1_Driver1_BufferNum w_ParLst_Drive[230] //1#��������е�����
#define Pw_Com1_Driver2_BufferNum w_ParLst_Drive[231] //2#��������е�����
#define Pw_Com2_Driver3_BufferNum w_ParLst_Drive[232] //3#��������е�����
#define Pw_Com2_Driver4_BufferNum w_ParLst_Drive[233] //4#��������е�����
#define Pw_Com3_Driver5_BufferNum w_ParLst_Drive[234] //5#��������е�����
#define Pw_Com3_Driver6_BufferNum w_ParLst_Drive[235] //6#��������е�����
#define Pw_StopStatus_Delay w_ParLst_Drive[236]       //ֹͣ״̬�����ʱ
#define Pw_Acc_Delay_Ratio w_ParLst_Drive[237]        //�Ӽ���ʱ����ʱ����
#define Pw_Current_Run_Time w_ParLst_Drive[238]       //��ǰָ������ʱ��

#define Pw_Driver1_AutoSpeed w_ParLst_Drive[240] //1#��ǰ�����ٶ�
#define Pw_Driver2_AutoSpeed w_ParLst_Drive[241] //2#��ǰ�����ٶ�
#define Pw_Driver3_AutoSpeed w_ParLst_Drive[242] //3#��ǰ�����ٶ�
#define Pw_Driver4_AutoSpeed w_ParLst_Drive[243] //4#��ǰ�����ٶ�
#define Pw_Driver5_AutoSpeed w_ParLst_Drive[244] //5#��ǰ�����ٶ�
#define Pw_Driver6_AutoSpeed w_ParLst_Drive[245] //6#��ǰ�����ٶ�
#define Pw_EquipStatus w_ParLst_Drive[246]       //�豸״̬������
#define Pw_EquipStatus_HW w_ParLst_Drive[247]    //�豸״̬������
#define Pw_Run_Mode w_ParLst_Drive[248]          //����ģʽ��=0������ģʽ��=1����ϴģʽ
#define Pw_JDQ_Addr w_ParLst_Drive[249]          //�̵�����ͨѶ��ַ

#define Pw_JDQ_Control w_ParLst_Drive[250]    //�̵�����DO1�����֣�=0�Ͽ���=1�պ�
#define Pr_JDQ_Status w_ParLst_Drive[251]     //�̵�����DO1״̬��
#define Pr_Com4_ComCount w_ParLst_Drive[252]  //Com4ͨѶ����
#define Pw_Brake_Delay w_ParLst_Drive[253]    //��ɲ����ʱ
#define Pr_runtime_show w_ParLst_Drive[254]   //ָ����ʱʱ��
#define Pr_pausetime_show w_ParLst_Drive[255] //ָ����ͣʱ��

#define Pr_cyclecounter_show w_ParLst_Drive[256] //����ѭ������
#define Pr_cyclecounter_HW w_ParLst_Drive[257]   //����ѭ���������

#define Pw_Driver_Run_MinSpeed w_ParLst_Drive[258] //�������С�����ٶȣ�Ҫ���ŷ��趨�ĵ����ת�����ֵ���ٴ�10rpm

#define Pr_Driver1_NeverRun w_ParLst_Drive[259] //1#�����û�����б�־
#define Pr_Driver2_NeverRun w_ParLst_Drive[260] //2#�����û�����б�־
#define Pr_Driver3_NeverRun w_ParLst_Drive[261] //3#�����û�����б�־
#define Pr_Driver4_NeverRun w_ParLst_Drive[262] //4#�����û�����б�־
#define Pr_Driver5_NeverRun w_ParLst_Drive[263] //5#�����û�����б�־
#define Pr_Driver6_NeverRun w_ParLst_Drive[264] //6#�����û�����б�־

#define Pw_Driver1_PosErr_Muti w_ParLst_Drive[265] //1#���������λ��ƫ���Ȧ
#define Pw_Driver2_PosErr_Muti w_ParLst_Drive[266] //2#���������λ��ƫ���Ȧ
#define Pw_Driver3_PosErr_Muti w_ParLst_Drive[267] //3#���������λ��ƫ���Ȧ
#define Pw_Driver4_PosErr_Muti w_ParLst_Drive[268] //4#���������λ��ƫ���Ȧ
#define Pw_Driver5_PosErr_Muti w_ParLst_Drive[269] //5#���������λ��ƫ���Ȧ
#define Pw_Driver6_PosErr_Muti w_ParLst_Drive[270] //6#���������λ��ƫ���Ȧ

#define Pw_Driver1_PosErr_Sing w_ParLst_Drive[271]    //1#���������λ��ƫ���Ȧ����λ
#define Pw_Driver1_PosErr_Sing_HW w_ParLst_Drive[272] //1#���������λ��ƫ���Ȧ����λ
#define Pw_Driver2_PosErr_Sing w_ParLst_Drive[273]    //2#���������λ��ƫ���Ȧ����λ
#define Pw_Driver2_PosErr_Sing_HW w_ParLst_Drive[274] //2#���������λ��ƫ���Ȧ����λ
#define Pw_Driver3_PosErr_Sing w_ParLst_Drive[275]    //3#���������λ��ƫ���Ȧ����λ
#define Pw_Driver3_PosErr_Sing_HW w_ParLst_Drive[276] //3#���������λ��ƫ���Ȧ����λ
#define Pw_Driver4_PosErr_Sing w_ParLst_Drive[277]    //4#���������λ��ƫ���Ȧ����λ
#define Pw_Driver4_PosErr_Sing_HW w_ParLst_Drive[278] //4#���������λ��ƫ���Ȧ����λ
#define Pw_Driver5_PosErr_Sing w_ParLst_Drive[279]    //5#���������λ��ƫ���Ȧ����λ
#define Pw_Driver5_PosErr_Sing_HW w_ParLst_Drive[280] //5#���������λ��ƫ���Ȧ����λ
#define Pw_Driver6_PosErr_Sing w_ParLst_Drive[281]    //6#���������λ��ƫ���Ȧ����λ
#define Pw_Driver6_PosErr_Sing_HW w_ParLst_Drive[282] //6#���������λ��ƫ���Ȧ����λ

#define Pw_Pos_Adj_Cmd w_ParLst_Drive[283]       //���������λ��ƫ���ֶ���������
#define Pw_AllStopped_Delay w_ParLst_Drive[284]  //�ж����е��λ�õ�����ʱ
#define Pw_Write_Timeout_Set w_ParLst_Drive[285] //дλ��ָ�ʱʱ���趨
#define Pr_Send_Data_F w_ParLst_Drive[286]       //�������ݱ�־
#define Pr_Driver1_Cmd_OK_F w_ParLst_Drive[287]  //1#�ŷ���������OK��־
#define Pr_Driver2_Cmd_OK_F w_ParLst_Drive[288]  //2#�ŷ���������OK��־
#define Pr_Driver3_Cmd_OK_F w_ParLst_Drive[289]  //3#�ŷ���������OK��־
#define Pr_Driver4_Cmd_OK_F w_ParLst_Drive[290]  //4#�ŷ���������OK��־

#define Pr_Driver5_Cmd_OK_F w_ParLst_Drive[291]     //5#�ŷ���������OK��־
#define Pr_Driver6_Cmd_OK_F w_ParLst_Drive[292]     //6#�ŷ���������OK��־
#define Pr_AllDriver_Cmd_OK_F w_ParLst_Drive[293]   //�����ŷ���������OK��־
#define Pr_HaveDriver_Cmd_Err_F w_ParLst_Drive[294] //���ŷ��������ݴ����־
#define Pr_OverMaxPos_F w_ParLst_Drive[295]         //����λ�÷�Χ��־

#endif
