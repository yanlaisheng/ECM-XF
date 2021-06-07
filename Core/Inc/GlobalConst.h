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

#define SECTORWORDNUM 256          // ����������
#define FLASH_PAR_SIZE 255         // NV��������
#define FLASH_ID_SIZE 128          // ID��������
#define PROGRAM_LEN 0xE000         // 0-0xE000 ����ռ䳤��
#define FLASH_PAR_ADDR 0xE000      // NV������ַ
#define FLASH_ID_ADDR 0xE200       // NV �绰�Ȳ�����ַ
#define FLASH_SETP_ADDR 0xE400     // �趨ѹ����
#define FLASH_VOICEVAL_ADDR 0xE600 // �趨������
#define FLASH_FAULT_ADDR 0xE800    // ���ϼ�¼��
#define FLASH_REC_ADDR 0xEA00      // FLASH ��¼
#define FLASH_RUNTSUM_ADDR 0xF000  // ˮ���ۼ�����ʱ�� ��¼
#define FLASH_REC_SIZE 64          // һ��FLASH��¼�ĳ���(�ֽ�)
#define NORCVMAXMS 5               // 20 2007.7.5
#define SECTOR_SIZE 65536          // ��������
#define RDWR_SIZE 64               // ��дFLASH�ĳ���
#define ISL1208 0xDE               // Device address for chip A--ISL1208
#define AT24C256 0xA4              // Device address for chip B
#define FREQMAXDEC 500             // Ƶ�����DECֵ
#define CS_Flash1 1                // ƬѡFLASH1
#define CS_Flash2 2                // ƬѡFLASH2
#define CS_FMRAM1 3                // ƬѡFMRAM1
#define FLASH_REC_MAX 16384        // FLASH��¼�����
#define FAULT_REC_MAX 64           //���ϼ�¼�����
#define FMADD_FLASH_REC_NO 512
#define FMADD_FLASH_REC_NUM 514
#define FMADD_FAULT_REC_NUM 516 //������ǰ��Ϊ�˸�SM510��Ӧ����
#define FMADD_FAULT_REC_NO 518

#define TXD1_MAX 255 // ���������
#define RCV1_MAX 255 // ���ջ��������� //256*8.5
#define TXD2_MAX 255 // ���������
#define RCV2_MAX 255 // ���ջ��������� //256*8.5
#define TXD3_MAX 255 // ���������
#define RCV3_MAX 255 // ���ջ��������� //256*8.5
#define TXD4_MAX 255 // ���������									//ZCL 2018.12.8
#define RCV4_MAX 255 // ���ջ��������� //256*8.5
#define TXD5_MAX 255 // ���������
#define RCV5_MAX 255 // ���ջ��������� //256*8.5			//ZCL 2018.12.8
#define TXD6_MAX 255 // ���������
#define RCV6_MAX 255 // ���ջ��������� //256*8.5			//ZCL 2018.12.8

// �ѡ�λ����ַ��λ��š�ת��������ַ��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
//�Ѹõ�ַת����һ��ָ��
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr (GPIOA_BASE + 12) //0x4001080C
#define GPIOB_ODR_Addr (GPIOB_BASE + 12) //0x40010C0C
#define GPIOC_ODR_Addr (GPIOC_BASE + 12) //0x4001100C
#define GPIOD_ODR_Addr (GPIOD_BASE + 12) //0x4001140C
#define GPIOE_ODR_Addr (GPIOE_BASE + 12) //0x4001180C
#define GPIOF_ODR_Addr (GPIOF_BASE + 12) //0x40011A0C
#define GPIOG_ODR_Addr (GPIOG_BASE + 12) //0x40011E0C

#define GPIOA_IDR_Addr (GPIOA_BASE + 8) //0x40010808
#define GPIOB_IDR_Addr (GPIOB_BASE + 8) //0x40010C08
#define GPIOC_IDR_Addr (GPIOC_BASE + 8) //0x40011008
#define GPIOD_IDR_Addr (GPIOD_BASE + 8) //0x40011408
#define GPIOE_IDR_Addr (GPIOE_BASE + 8) //0x40011808
#define GPIOF_IDR_Addr (GPIOF_BASE + 8) //0x40011A08
#define GPIOG_IDR_Addr (GPIOG_BASE + 8) //0x40011E08

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

#define COM1_DATA PDout(8)  //=1,��=0,��	���ǣ�Data ָʾ�� ������
#define COM0_DATA PDout(15) //=1,��=0,��

//#define		COM0_TX				PDout(9)		//�������ԣ�COM1_LED���ģʽ�����״̬���ܶ�ȡ
#define COM0_RX PDin(10) //״̬�����
//#define		COM1_TX				PDout(10)		//�������ԣ�COM1_LED���ģʽ�����״̬���ܶ�ȡ
#define COM1_RX PDin(11) //״̬�����

/* BSPӲ������  */
//----------------------------------------------------------------

//����������
//DI1 -- PC8		//DI2 -- PC7		//DI3 -- PA8		DI4 -- PC9
#define DI1 PCin(8) // 2016.2.1
#define DI2 PCin(7) //
#define DI3 PAin(8) //
#define DI4 PCin(9) //

//�̵������			��DSP�ϣ�ͨ�����DO1��DO2���Լ�ӿ��Ƽ̵��� 2015.5.10 ����DSP��λ�ȶ�������
#define DO1 PAout(11) // =1 OPEN; =0 CLOSE
#define DO2 PAout(12) // =1 OPEN; =0 CLOSE

#define Qb_Q1 PAout(11) //=1 ��,=0�պ�		//���������1  ���ӵ�DSP B4
#define Qb_Q2 PAout(12) //=1 ��,=0�պ�		//���������2  ���ӵ�DSP B5

#define Qb_Q1_IN PAin(11) //=1 ��,=0�պ�		//���������1  ���ӵ�DSP B4
#define Qb_Q2_IN PAin(12) //=1 ��,=0�պ�		//���������2  ���ӵ�DSP B5

#define STOP_BRAKE_SYSTEM GPIO_ResetBits(GPIOA, GPIO_Pin_11) //ֹͣɲ������������
#define START_BRAKE_SYSTEM GPIO_SetBits(GPIOA, GPIO_Pin_11)  //��ʼɲ��������������

#define Brake_Status_DO1 PAin(11) //ɲ��״̬

#define OPEN_HAND GPIO_ResetBits(GPIOA, GPIO_Pin_12) //�򿪻�е�ֱ�
#define CLOSE_HAND GPIO_SetBits(GPIOA, GPIO_Pin_12)  //�պϻ�е�ֱ�

//SPI NSS��
#define SPI2_NSS_L GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define SPI2_NSS_H GPIO_SetBits(GPIOB, GPIO_Pin_12)

#define SPI3_NSS_L GPIO_ResetBits(GPIOC, GPIO_Pin_6)
#define SPI3_NSS_H GPIO_SetBits(GPIOC, GPIO_Pin_6)

//ָʾ�� �ܳ��� 2016.03.10
#define LED1 PFout(8) //=1,��; =0��;
#define LED2 PBout(6)
#define LED3 PBout(5)
#define LED4 PBout(4)
#define LED5 PBout(3)
#define LED6 PAout(15) //ZCL 2018.12.8

#define LED1_IN PFin(8) //��
#define LED2_IN PBin(6)
#define LED3_IN PBin(5)
#define LED4_IN PBin(4)
#define LED5_IN PBin(3)
#define LED6_IN PAin(15) //ZCL 2018.12.8

#define HAND_STATUS PAin(12) //0�򿪵�ŷ���1�رյ�ŷ�

#define S_ACCEL 1
#define T_ACCEL 0

/* S�ͼ��ٲ��� */
#define ACCELERATED_SPEED_LENGTH 200 //������ٶȵĵ�������ʵҲ��3000��ϸ�ֲ�����˼��������������ı���ٵ�
#define FRE_MIN 500                  //��͵�����Ƶ�ʣ����������������������ٶ�
#define FRE_MAX 35000                //��ߵ�����Ƶ�ʣ������������������ʱ������ٶ�35000

// �����趨����
#define Pw_Motor1SendPulse w_ParLst[0]     // ���1����������
#define Pw_Motor1SendPulse_HW w_ParLst[1]  // ���1��������������
#define Pw_Motor1_ACCSpeed w_ParLst[2]     // ���1���ٶ�
#define Pw_Motor1_ACCSpeed_HW w_ParLst[3]  // ���1���ٶȸ���
#define Pw_Motor1_SetSpeed w_ParLst[4]     // ���1�趨�ٶ�
#define Pw_Motor1_SetSpeed_HW w_ParLst[5]  // ���1�趨�ٶȸ���
#define Pw_Motor1_PULSENUM w_ParLst[6]     // ���1ÿȦ������
#define Pw_Motor1_FRE_START w_ParLst[7]    // ���1����Ƶ��
#define Pw_Motor1_FRE_AA w_ParLst[8]       // ���1�Ӽ��ٶ�
#define Pw_ModPar w_ParLst[9]              // �޸Ĳ���
#define Pw_Motor1_STEP_PARA w_ParLst[10]   // ���1����������������
#define Pw_Motor1_maxposition w_ParLst[11] // ���1���λ��

#define Pw_Motor2SendPulse w_ParLst[16]    // ���2����������
#define Pw_Motor2SendPulse_HW w_ParLst[17] // ���2��������������
#define Pw_Motor2_ACCSpeed w_ParLst[18]    // ���2���ٶ�
#define Pw_Motor2_ACCSpeed_HW w_ParLst[19] // ���2���ٶȸ���
#define Pw_Motor2_SetSpeed w_ParLst[20]    // ���2�趨�ٶ�
#define Pw_Motor2_SetSpeed_HW w_ParLst[21] // ���2�趨�ٶȸ���
#define Pw_Motor2_PULSENUM w_ParLst[22]    // ���2ÿȦ������
#define Pw_Motor2_FRE_START w_ParLst[23]   // ���2����Ƶ��
#define Pw_Motor2_FRE_AA w_ParLst[24]      // ���2�Ӽ��ٶ�
#define Pw_Motor2_STEP_PARA w_ParLst[25]   // ���2����������������
#define Pw_Motor2_maxposition w_ParLst[26] // ���2���λ��

#define Pw_Initial_F w_ParLst[40]      // ��ʼ����־��=0x5A����ʾ�Ѿ���ʼ��
#define Pw_ParInitial w_ParLst[41]     // ������ʼ��
#define Pos_Group_Select w_ParLst[42]  // ����λ�ÿ���������ѡ��1-5�飩
#define Group_Select w_ParLst[43]      //�������ѡ��
#define Pw_Driver_PosDely w_ParLst[44] // λ��д������ʱ��250ms

#define Pw_Motor3SendPulse w_ParLst[51]    // ���3����������
#define Pw_Motor3SendPulse_HW w_ParLst[52] // ���3��������������
#define Pw_Motor3_ACCSpeed w_ParLst[53]    // ���3���ٶ�
#define Pw_Motor3_ACCSpeed_HW w_ParLst[54] // ���3���ٶȸ���
#define Pw_Motor3_SetSpeed w_ParLst[55]    // ���3�趨�ٶ�
#define Pw_Motor3_SetSpeed_HW w_ParLst[56] // ���3�趨�ٶȸ���
#define Pw_Motor3_PULSENUM w_ParLst[57]    // ���3ÿȦ������
#define Pw_Motor3_FRE_START w_ParLst[58]   // ���3����Ƶ��
#define Pw_Motor3_FRE_AA w_ParLst[59]      // ���3�Ӽ��ٶ�
#define Pw_Motor3_STEP_PARA w_ParLst[60]   // ���3����������������
#define Pw_Motor3_maxposition w_ParLst[61] // ���3���λ��

#define Pw_Motor4SendPulse w_ParLst[66]    // ���4����������
#define Pw_Motor4SendPulse_HW w_ParLst[67] // ���4��������������
#define Pw_Motor4_ACCSpeed w_ParLst[68]    // ���4���ٶ�
#define Pw_Motor4_ACCSpeed_HW w_ParLst[69] // ���4���ٶȸ���
#define Pw_Motor4_SetSpeed w_ParLst[70]    // ���4�趨�ٶ�
#define Pw_Motor4_SetSpeed_HW w_ParLst[71] // ���4�趨�ٶȸ���
#define Pw_Motor4_PULSENUM w_ParLst[72]    // ���4ÿȦ������
#define Pw_Motor4_FRE_START w_ParLst[73]   // ���4����Ƶ��
#define Pw_Motor4_FRE_AA w_ParLst[74]      // ���4�Ӽ��ٶ�
#define Pw_Motor4_STEP_PARA w_ParLst[75]   // ���4����������������
#define Pw_Motor4_maxposition w_ParLst[76] // ���4���λ��

#define Pw_Motor5SendPulse w_ParLst[81]    // ���5����������
#define Pw_Motor5SendPulse_HW w_ParLst[82] // ���5��������������
#define Pw_Motor5_ACCSpeed w_ParLst[83]    // ���5���ٶ�
#define Pw_Motor5_ACCSpeed_HW w_ParLst[84] // ���5���ٶȸ���
#define Pw_Motor5_SetSpeed w_ParLst[85]    // ���5�趨�ٶ�
#define Pw_Motor5_SetSpeed_HW w_ParLst[86] // ���5�趨�ٶȸ���
#define Pw_Motor5_PULSENUM w_ParLst[87]    // ���5ÿȦ������
#define Pw_Motor5_FRE_START w_ParLst[88]   // ���5����Ƶ��
#define Pw_Motor5_FRE_AA w_ParLst[89]      // ���5�Ӽ��ٶ�
#define Pw_Motor5_STEP_PARA w_ParLst[90]   // ���5����������������
#define Pw_Motor5_maxposition w_ParLst[91] // ���5���λ��

#define Pw_Motor6SendPulse w_ParLst[96]     // ���6����������
#define Pw_Motor6SendPulse_HW w_ParLst[97]  // ���6��������������
#define Pw_Motor6_ACCSpeed w_ParLst[98]     // ���6���ٶ�
#define Pw_Motor6_ACCSpeed_HW w_ParLst[99]  // ���6���ٶȸ���
#define Pw_Motor6_SetSpeed w_ParLst[100]    // ���6�趨�ٶ�
#define Pw_Motor6_SetSpeed_HW w_ParLst[101] // ���6�趨�ٶȸ���
#define Pw_Motor6_PULSENUM w_ParLst[102]    // ���6ÿȦ������
#define Pw_Motor6_FRE_START w_ParLst[103]   // ���6����Ƶ��
#define Pw_Motor6_FRE_AA w_ParLst[104]      // ���6�Ӽ��ٶ�
#define Pw_Motor6_STEP_PARA w_ParLst[105]   // ���6����������������
#define Pw_Motor6_maxposition w_ParLst[106] // ���6���λ��

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
//#define	Pw_SmallBadClrSec		w_ParLst[144]	// С�����ƻ������
//#define	Pw_SendToGprsEn			w_ParLst[145]	// �������͸�GPRS��������ʹ��
//#define	Pw_SendToGprsDataLen	w_ParLst[146]	// �������͸�GPRS�������ݳ��� ��
//#define	Pw_SendToGprsSec		w_ParLst[147]	// �������͸�GPRS�������ݼ��ʱ�� ��
#define Pw_ComBufType w_ParLst[148] // ͨѶ��������(ͨѶЭ������) ��ͨ=1,����=2
//#define	Pw_ASensorType			w_ParLst[149]	// ��������������
//#define	Pw_ASensorZero			w_ParLst[150]	// ������������ֵ
//#define	Pw_VASumFilter			w_ParLst[151]	// ���˲�
#define Pw_HighYCSetPEK2 w_ParLst[151]
#define Pw_BaudRate4 w_ParLst[152] // ������4
#define Pw_BaudRate5 w_ParLst[153] // ������5
#define Pw_Com5Addr w_ParLst[154]  //com5��485ͨѶ��ַ
//----
#define Pw_LLJCaiJiSelect w_ParLst[155] //�����Ʋɼ���ʽ
#define Pw_DDBCaiJiSelect w_ParLst[156]
#define Pw_DDBAddR w_ParLst[157] //���ܱ��ַ

#define Pw_EquipmentNo1 w_ParLst[159] //1#�ŷ����Modbus��ַ
#define Pw_EquipmentNo2 w_ParLst[160] //2#�ŷ����Modbus��ַ
#define Pw_EquipmentNo3 w_ParLst[161] //3#�ŷ����Modbus��ַ
#define Pw_EquipmentNo4 w_ParLst[162] //4#�ŷ����Modbus��ַ
#define Pw_EquipmentNo5 w_ParLst[163] //5#�ŷ����Modbus��ַ
#define Pw_EquipmentNo6 w_ParLst[164] //6#�ŷ����Modbus��ַ

#define Pw_Com3Add w_ParLst[165]           //com3-T3ͨѶ��ַ
#define Pw_InP_CTLVfEn w_ParLst[166]       //��ˮ����Ƶ��ʹ�� 2010.8.3 qhd
#define Pw_VfFreqDownSircle w_ParLst[167]  //ÿ��ݼ�Ƶ��ֵ
#define Pw_VfFreqDownMin w_ParLst[168]     //��С����Ƶ��
#define Pw_SetInP w_ParLst[169]            //��ˮ�趨ѹ��
#define Pw_PumpSoftStopEn w_ParLst[170]    //�豸��ͣʹ��
#define Pw_VfOnDelay w_ParLst[171]         //��Ƶת��Ƶʱ������Ƶ����ʱ  �غ���2010.8.3
#define Pw_DownFreqHex w_ParLst[172]       //ͣ��ÿ��ݼ�Ƶ��ֵ
#define Pw_AllPumpStopDelay w_ParLst[174]  //��ͣ����ʱʱ��
#define Pw_GpExitVfOnFreqHex w_ParLst[175] //��Ƶ�˳���Ƶ��Ƶ�ʸ�ֵ
#define Pw_VfAlarmDelay w_ParLst[176]      //��Ƶ������ͣ����ʱ
#define Pw_PumpRunTimEn w_ParLst[177]      //ˮ���ۼ�����ʱ���¼ʹ��
#define Pw_GpExitFreq w_ParLst[178]        //��Ƶ�˳�Ƶ��

#define Pr_RUN_Count w_ParLst[179]     //����ָ�����
#define Pr_RUN_Count_Set w_ParLst[180] //���ж���Ȧ�趨����ʼ��Ϊ0

#define Pw_Total_RUN_Count w_ParLst[181]    //�ۼ�����Ȧ��������ֵΪ0
#define Pw_Total_RUN_Count_HW w_ParLst[182] //�ۼ�����Ȧ��_����
#define Pw_Com5BufType w_ParLst[183]        //Com5ͨѶ����
#define Pw_ComWriteErr_Stop w_ParLst[184]   //ͨѶ����д�����ͣ�����ܣ�=1��ͣ����=0����ͣ��
#define Pw_HighYCSetP1 w_ParLst[185]
#define Pw_HighYCSetPEK w_ParLst[186]
#define Pw_HighYCSetPDelay w_ParLst[187]
#define Pw_PIDSlowEN w_ParLst[188]         //PID��������Ƶ��ʹ��
#define Pw_PID_UPSlowCount w_ParLst[189]   //PID�����������ڱ���
#define Pw_PID_PEKUpperlimit w_ParLst[190] //PID����ѹ������
#define Pw_PID_PEKLowerlimit w_ParLst[191] //PID����ѹ������
#define Pw_HengVfDelay w_ParLst[192]       // С������pinʱ��
#define Pw_HaveWater_MaxSupplyDelay w_ParLst[193]

#define Pw_Com3BufType w_ParLst[210]
#define Pw_Drive1_MinPos_SINGLE w_ParLst[211]    //1#�����Сλ�ã���Ȧ+��Ȧλ�ã�
#define Pw_Drive1_MinPos_SINGLE_HW w_ParLst[212] //1#�����Сλ��_���֣���Ȧ+��Ȧλ�ã�
#define Pw_Drive1_MaxPos_SINGLE w_ParLst[213]    //1#������λ�ã���Ȧ+��Ȧλ�ã�
#define Pw_Drive1_MaxPos_SINGLE_HW w_ParLst[214] //1#������λ��_���֣���Ȧ+��Ȧλ�ã�
#define Pw_Drive2_MinPos_SINGLE w_ParLst[215]    //2#�����Сλ�ã���Ȧ+��Ȧλ�ã�
#define Pw_Drive2_MinPos_SINGLE_HW w_ParLst[216] //2#�����Сλ��_���֣���Ȧ+��Ȧλ�ã�
#define Pw_Drive2_MaxPos_SINGLE w_ParLst[217]    //2#������λ�ã���Ȧ+��Ȧλ�ã�
#define Pw_Drive2_MaxPos_SINGLE_HW w_ParLst[218] //2#������λ��_���֣���Ȧ+��Ȧλ�ã�

#define Pw_Com3SendAdd w_ParLst[220] //T3���������͵�ַ 339
#define Pw_Com3Sendnum w_ParLst[221] //T3��������������32

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

#define Pw_Drive1_MinPos w_ParLst[232] //1#�����Сλ�ã���Ȧλ�ã�
#define Pw_Drive1_MaxPos w_ParLst[233] //1#������λ�ã���Ȧλ�ã�
#define Pw_Drive2_MinPos w_ParLst[234] //2#�����Сλ�ã���Ȧλ�ã�
#define Pw_Drive2_MaxPos w_ParLst[235] //2#������λ�ã���Ȧλ�ã�
#define Pw_Drive3_MinPos w_ParLst[236] //3#�����Сλ�ã���Ȧλ�ã�
#define Pw_Drive3_MaxPos w_ParLst[237] //3#������λ�ã���Ȧλ�ã�
#define Pw_Drive4_MinPos w_ParLst[238] //4#�����Сλ�ã���Ȧλ�ã�
#define Pw_Drive4_MaxPos w_ParLst[239] //4#������λ�ã���Ȧλ�ã�
#define Pw_Drive5_MinPos w_ParLst[240] //5#�����Сλ�ã���Ȧλ�ã�
#define Pw_Drive5_MaxPos w_ParLst[241] //5#������λ�ã���Ȧλ�ã�
#define Pw_Drive6_MinPos w_ParLst[242] //6#�����Сλ�ã���Ȧλ�ã�
#define Pw_Drive6_MaxPos w_ParLst[243] //6#������λ�ã���Ȧλ�ã�

#define Pr_BRAKE_Status w_ParLst[244]       //ɲ��״̬
#define Pr_BRAKE_Control w_ParLst[245]      //ɲ�����ƣ�=1��ɲ����=0����ɲ��
#define Pr_F_AllStopped w_ParLst[246]       //���е����ֹͣ��־
#define Pr_Com4_ComErr w_ParLst[247]        //Com4ͨѶ���ϱ�־
#define Pr_BRAKE_Control_Mode w_ParLst[248] //ɲ������ģʽ��=1���ֶ�����ɲ����=0���Զ�����ɲ��
#define Pr_AllRun w_ParLst[249]             //���е�������б�־

// ����ģ�����������ò������ӵ�ַ240��ʼ����ַ254����15��

// ����ֻ���������ӵ�ַ256��511��ʼ��
#define w_Uk1 w_ParLst[256]             // Ƶ�����Hexֵ
#define w_InPSensorValue w_ParLst[257]  // ��ˮ��ѹ����������ֵ
#define w_OutPSensorValue w_ParLst[258] // ��ˮ��ѹ����������ֵ
#define w_FaultRecNum w_ParLst[259]     // ���ϼ�¼���� ZCL ������ǰ��Ϊ�˸�SM510��Ӧ����
#define w_FaultRecNo w_ParLst[260]      // ���ϼ�¼��  ZCL

#define w_TSensorValue w_ParLst[261]  // �¶ȴ��������ֵ  2016.3.22
#define w_SaveBaudRate0 w_ParLst[262] // �����ʱ���ֵ0
#define w_SaveBaudRate1 w_ParLst[263] // �����ʱ���ֵ1
#define w_SaveBaudRate2 w_ParLst[264] // �����ʱ���ֵ2
#define w_SaveBaudRate3 w_ParLst[265] // �����ʱ���ֵ3
#define w_SaveBaudRate4 w_ParLst[266] // �����ʱ���ֵ4		ZCL 2018.12.8
#define w_SaveBaudRate5 w_ParLst[267] // �����ʱ���ֵ5		ZCL 2018.12.8

#define w_GongSiSelect w_ParLst[268] //  ��˾ѡ��0=�е�����1=��������
//#define	w_FlashWrRdLock			w_ParLst[267]	// Flashд������		ZCL 2018.12.8
//#define	w_FlashRecNoNumPointerNo	w_ParLst[268]	// FLASH��¼���������ָ�����
#define w_SelFaultNo w_ParLst[269] // ѡ����Ϻ�
#define w_SelRecNo w_ParLst[270]   // ѡ���¼��
#define w_PreFaultNo w_ParLst[271] // �ϴι��Ϻ�
//
#define w_TestItemSel w_ParLst[272]        // ������ѡ���趨���� (�����޸ģ������Ա���)
#define w_VvvfAlmNum w_ParLst[273]         // ��Ƶ��������
#define w_BetweenSmall w_ParLst[274]       // С�������״̬
#define w_SmallStableRunSec w_ParLst[275]  // С������ѹ��ʱ
#define w_SoftVer w_ParLst[276]            // ����汾
#define w_TimePwdStopST w_ParLst[277]      // ��ʱ����ͣ��״̬
#define w_EquipOperateStatus w_ParLst[278] // �豸����״̬
#define w_EquipOperateNum w_ParLst[279]    // �豸����״̬����
#define w_EquipAlarmStatus w_ParLst[280]   // �豸ͣ��ԭ�򣨱�����
#define w_EquipStopNum w_ParLst[281]       // �豸ͣ��������������
//
#define w_EquipAlarmLast6 w_ParLst[282]    // �豸����ͣ��ԭ����6��
#define w_EquipAlarm6YM w_ParLst[283]      // �豸����ͣ��6ʱ�䣭����
#define w_EquipAlarm6DH w_ParLst[284]      // �豸����ͣ��6ʱ�䣭��ʱ
#define w_EquipAlarm6MS w_ParLst[285]      // �豸����ͣ��6ʱ�䣭����
#define w_EquipAlarmLast5 w_ParLst[286]    // �豸����ͣ��ԭ����5��
#define w_EquipAlarm5YM w_ParLst[287]      // �豸����ͣ��5ʱ�䣭����
#define w_EquipAlarm5DH w_ParLst[288]      // �豸����ͣ��5ʱ�䣭��ʱ
#define w_EquipAlarm5MS w_ParLst[289]      // �豸����ͣ��5ʱ�䣭����
#define w_EquipAlarmLast4 w_ParLst[290]    // �豸����ͣ��ԭ����4��
#define w_EquipAlarm4YM w_ParLst[291]      // �豸����ͣ��4ʱ�䣭����
#define w_EquipAlarm4DH w_ParLst[292]      // �豸����ͣ��4ʱ�䣭��ʱ
#define w_EquipAlarm4MS w_ParLst[293]      // �豸����ͣ��4ʱ�䣭����
#define w_EquipAlarmLast3 w_ParLst[294]    // �豸����ͣ��ԭ����3��
#define w_EquipAlarm3YM w_ParLst[295]      // �豸����ͣ��3ʱ�䣭����
#define w_EquipAlarm3DH w_ParLst[296]      // �豸����ͣ��3ʱ�䣭��ʱ
#define w_EquipAlarm3MS w_ParLst[297]      // �豸����ͣ��3ʱ�䣭����
#define w_EquipAlarmLast2 w_ParLst[298]    // �豸����ͣ��ԭ����2��
#define w_EquipAlarm2YM w_ParLst[299]      // �豸����ͣ��2ʱ�䣭����
#define w_EquipAlarm2DH w_ParLst[300]      // �豸����ͣ��2ʱ�䣭��ʱ
#define w_EquipAlarm2MS w_ParLst[301]      // �豸����ͣ��2ʱ�䣭����
#define w_EquipAlarmLast1 w_ParLst[302]    // �豸����ͣ��ԭ����1��
#define w_EquipAlarm1YM w_ParLst[303]      // �豸����ͣ��1ʱ�䣭����
#define w_EquipAlarm1DH w_ParLst[304]      // �豸����ͣ��1ʱ�䣭��ʱ
#define w_EquipAlarm1MS w_ParLst[305]      // �豸����ͣ��1ʱ�䣭����
#define w_SelEquipAlarm w_ParLst[306]      // ѡ����豸����ͣ��ԭ��
#define w_SelEquipAlarm1YM w_ParLst[307]   // ѡ����豸����ͣ��ʱ�䣭����
#define w_SelEquipAlarm1DH w_ParLst[308]   // ѡ����豸����ͣ��ʱ�䣭��ʱ
#define w_SelEquipAlarm1MS w_ParLst[309]   // ѡ����豸����ͣ��ʱ�䣭����
#define w_FlashRecNo w_ParLst[310]         // FLASH��¼��
#define w_FlashRecNum w_ParLst[311]        // FLASH��¼����
#define w_TimePwdStopDays w_ParLst[312]    // ��ʱ����ͣ������
#define w_RemoteStop w_ParLst[313]         // Զ��ң�ر�Ƶֹͣ =1ͣ��
#define w_RemoteGpRun w_ParLst[314]        // Զ��ң�ع�Ƶ��ͣ .0 GP1, .1 GP2, .2 GP3, .3 GP4, .4 GP5
#define w_RemoteVfRstRelayOn w_ParLst[315] // ң�ر�Ƶ����λ�̵���ON
#define w_TouchAutoManu w_ParLst[316]      // ���� �Զ�/�ֶ�
//#define	Pw_TouchRunStop			w_ParLst[317]	// ���� ����/ֹͣ
#define w_Touch1Gp w_ParLst[318]        // ����1��Ƶ
#define w_Touch2Gp w_ParLst[319]        // ����2��Ƶ
#define w_Touch3Gp w_ParLst[320]        // ����3��Ƶ
#define w_Touch4Gp w_ParLst[321]        // ����4��Ƶ
#define w_Touch5Gp w_ParLst[322]        // ����5��Ƶ \
                                        //#define	w_RunTimeSumNo			w_ParLst[323]	// ˮ���ۼ�����ָ�����
#define w_FlashUPSetParNo w_ParLst[324] // FLASH�ϴ����� ���
#define w_FlashDWSetParNo w_ParLst[325] // FLASH���ز��� ���
#define w_FlashUPProgNo w_ParLst[326]   // FLASH�ϴ�����
#define w_FlashDWProgNo w_ParLst[327]   // FLASH���س���
#define w_FlashDWRecordNo w_ParLst[328] // FLASH������ʷ��¼
//#define	w_DisMemoryBoardRecord	w_ParLst[329]	// ��ʾFLASH �洢���¼
#define w_SaveRecNo w_ParLst[330]         // ����Flash��¼��
#define w_SaveRecNum w_ParLst[331]        // ����Flash��¼����
#define w_CpuTemperatureHex w_ParLst[332] // Cpu�¶�Hex
#define w_CpuWendu w_ParLst[333]          // Cpu�¶Ȼ���ֵ

#define w_SBaudTimeCount w_ParLst[334]   // ���ڲ����ʼ�����
#define w_SBaudThTimeCount w_ParLst[335] // ���ڲ����ʼ�������������ʼ��������
#define w_SelPar w_ParLst[336]           // ѡ�����
#define w_SelParValue w_ParLst[337]      // ѡ�������ֵ

// Զ�̼�ؼ��в�ѯ���ò��� 32��
#define w_ProcessNo w_ParLst[339] // �������
// Զ�̼�ؼ��в�ѯ���ò��� 31����
#define w_Pump12Status w_ParLst[340] // ��12״̬
#define w_Pump34Status w_ParLst[341] // ��34״̬
#define w_Pump56Status w_ParLst[342] // ��56״̬
#define w_Flag1Unit w_ParLst[343]    // ��־1��Ԫ
#define w_Flag2Unit w_ParLst[344]    // ��־2��Ԫ
#define w_Flag3Unit w_ParLst[345]    // ��־3��Ԫ
//#define	w_ResidualCL			w_ParLst[346]	//
#define w_Flag4Unit w_ParLst[346]

#define w_PIDCalcP w_ParLst[347]     // PID����ѹ��
#define w_VvvfFreq w_ParLst[348]     // ��Ƶ��Ƶ��
#define w_InPDec w_ParLst[349]       // ��ˮ��ѹ��
#define w_OutPDec w_ParLst[350]      // ��ˮ��ѹ��
#define w_InstanFlux w_ParLst[351]   // ˲ʱ����		//ZCL 2007.6.15
#define w_Pump1Current w_ParLst[352] // 1�űõ���
#define w_Pump2Current w_ParLst[353] // 2�űõ���
#define w_Pump3Current w_ParLst[354] // 3�űõ���
#define w_Pump4Current w_ParLst[355] // 4�űõ���
#define w_Pump5Current w_ParLst[356] // 5�űõ���
#define w_YeWeiDeep w_ParLst[357]    // Һλ���
#define w_SysVoltage w_ParLst[358]   // ϵͳ��ѹ		//ZCL 2007.6.15
#define w_SumFluxL w_ParLst[359]     // �ۼ���������
#define w_SumFluxH w_ParLst[360]     // �ۼ���������
#define w_DDBSumFluxL w_ParLst[361]  // �ۼƵ�������
#define w_DDBSumFluxH w_ParLst[362]  //
#define w_ZhuoDuValue w_ParLst[363]  // �Ƕ�
#define w_YuLvValue w_ParLst[364]    //
#define w_WenDuValue w_ParLst[365]   //
#define w_PHValue w_ParLst[366]      // PHֵ

//#define	w_VvvfFreq1			    w_ParLst[363]	//
#define w_YeWeiDeep2 w_ParLst[367] //2#ˮ��Һλ
#define w_NowYM w_ParLst[368]      // ����
#define w_NowDH w_ParLst[369]      // ��ʱ
#define w_NowMS w_ParLst[370]      // ����
#define w_ShiDuValue w_ParLst[371] //
#define w_ZaoYinValue w_ParLst[372]
#define w_DDF1OpenValue w_ParLst[373] // 1#�綯������
#define w_DDF2OpenValue w_ParLst[374] // 2#�綯������
//#define	w_DDF3OpenValue			w_ParLst[375]	// 1#�綯������
//#define	w_DDF4OpenValue			w_ParLst[376]	// 2#�綯������
#define w_VvvfFreq1 w_ParLst[375]
#define w_VvvfFreq2 w_ParLst[376]
#define w_VvvfFreq3 w_ParLst[377] //
#define w_VvvfFreq4 w_ParLst[378]
//#define	w_VvvfFreq5			    w_ParLst[379]
//

// 640��ʼ��Ź��̱���������Ҳ���浽FLASH��������������û���á�
#define w_WriteDate w_ParLst[642] //�����д����
#define w_Writetime w_ParLst[643] //�����дʱ��

#define w_VfNum w_ParLst[680]
#define w_GpPumpNum w_ParLst[681]
#define w_StartPump1 w_ParLst[682]
#define w_StartPump2 w_ParLst[683]
#define w_StartPump3 w_ParLst[684]
#define w_StartPump4 w_ParLst[685]
#define w_w_Uk1 w_ParLst[686]
#define w_TimeStopP w_ParLst[687]
#define w_UK_NO1 w_ParLst[688]
#define w_UK_NO2 w_ParLst[689]
#define w_VfNo w_ParLst[690]
#define w_VfNext w_ParLst[691]
#define w_NeedPidCalcP w_ParLst[692]
#define w_Alarm w_ParLst[693] // ����

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

// ����������ò������ӵ�ַ240��ʼ��
#define AIsel1 w_ParLst[TERMADDR + 0] / 100 - 1   // ģ��������ѡ��
#define AIsel2 w_ParLst[TERMADDR + 0] % 100 - 1   // ģ��������ѡ��
#define AIsel3 w_ParLst[TERMADDR + 1] / 100 - 1   // ģ��������ѡ��
#define AIsel4 w_ParLst[TERMADDR + 1] % 100 - 1   // ģ��������ѡ��
#define AIsel5 w_ParLst[TERMADDR + 2] / 100 - 1   // ģ��������ѡ��
#define AIsel6 w_ParLst[TERMADDR + 2] % 100 - 1   // ģ��������ѡ��
#define AIsel7 w_ParLst[TERMADDR + 3] / 100 - 1   // ģ��������ѡ��
#define AIsel8 w_ParLst[TERMADDR + 3] % 100 - 1   // ģ��������ѡ��
#define AIsel9 w_ParLst[TERMADDR + 4] / 100 - 1   // ģ��������ѡ��
#define AIsel10 w_ParLst[TERMADDR + 4] % 100 - 1  // ģ��������ѡ��
#define AIsel11 w_ParLst[TERMADDR + 5] / 100 - 1  // ģ��������ѡ��
#define AIsel12 w_ParLst[TERMADDR + 5] % 100 - 1  // ģ��������ѡ��
#define AIsel13 w_ParLst[TERMADDR + 6] / 100 - 1  // ģ��������ѡ��
#define AIsel14 w_ParLst[TERMADDR + 6] % 100 - 1  // ģ��������ѡ��
#define AIsel15 w_ParLst[TERMADDR + 7] / 100 - 1  // ģ��������ѡ��
#define AIsel16 w_ParLst[TERMADDR + 7] % 100 - 1  // ģ��������ѡ��
#define AIsel17 w_ParLst[TERMADDR + 8] / 100 - 1  // ģ��������ѡ��
#define AIsel18 w_ParLst[TERMADDR + 8] % 100 - 1  // ģ��������ѡ��
#define AIsel19 w_ParLst[TERMADDR + 9] / 100 - 1  // ģ��������ѡ��
#define AIsel20 w_ParLst[TERMADDR + 9] % 100 - 1  // ģ��������ѡ��
#define AIsel21 w_ParLst[TERMADDR + 10] / 100 - 1 // ģ��������ѡ��
#define AIsel22 w_ParLst[TERMADDR + 10] % 100 - 1 // ģ��������ѡ��
#define AIsel23 w_ParLst[TERMADDR + 11] / 100 - 1 // ģ��������ѡ��
#define AIsel24 w_ParLst[TERMADDR + 11] % 100 - 1 // ģ��������ѡ��
//
#define AQsel1 w_ParLst[TERMADDR + 12 + 0] / 100 - 1 // ģ�������ѡ��
#define AQsel2 w_ParLst[TERMADDR + 12 + 0] % 100 - 1 // ģ�������ѡ��
#define AQsel3 w_ParLst[TERMADDR + 12 + 1] / 100 - 1 // ģ�������ѡ��
#define AQsel4 w_ParLst[TERMADDR + 12 + 1] % 100 - 1 // ģ�������ѡ��
#define AQsel5 w_ParLst[TERMADDR + 12 + 2] / 100 - 1 // ģ�������ѡ��
#define AQsel6 w_ParLst[TERMADDR + 12 + 2] % 100 - 1 // ģ�������ѡ��

#define FLASH_CMD_SIZE CMD_LIN_NUM *CMD_LIN_SIZE //30��ָ�ÿ��ָ��20�����ֽڣ���600�����ֽڣ���2400���ֽ�
#define CMD_LIN_NUM 30                           //30��ָ��
#define CMD_LIN_SIZE 20                          //һ������ռ�õ�����

//M25P32FLASH�洢оƬ��32Mbit����8M�ֽڣ�64��������ÿ������256ҳ��ÿҳ256�ֽڣ���ÿ������64K�ֽڣ�
//����FLASH �����ַ
#define FLASH_SAVE_DRIVER_PAR 0X300000 //64��������ÿ������64K�ֽ�

#define FLASH_SAVE_ADDR1_Group1 0X020000 //64��������ÿ������64K�ֽ�
#define FLASH_SAVE_ADDR2_Group1 0X030000 //���ܿ�����д������һ�α��浽һ������
#define FLASH_SAVE_ADDR3_Group1 0X040000
#define FLASH_SAVE_ADDR4_Group1 0X050000
#define FLASH_SAVE_ADDR5_Group1 0X060000
#define FLASH_SAVE_ADDR6_Group1 0X070000

#define FLASH_SAVE_ADDR1_Group2 0X080000 //64��������ÿ������64K�ֽ�
#define FLASH_SAVE_ADDR2_Group2 0X090000 //���ܿ�����д������һ�α��浽һ������
#define FLASH_SAVE_ADDR3_Group2 0X0A0000
#define FLASH_SAVE_ADDR4_Group2 0X0B0000
#define FLASH_SAVE_ADDR5_Group2 0X0C0000
#define FLASH_SAVE_ADDR6_Group2 0X0D0000

#define FLASH_SAVE_ADDR1_Group3 0X0E0000 //64��������ÿ������64K�ֽ�
#define FLASH_SAVE_ADDR2_Group3 0X0F0000 //���ܿ�����д������һ�α��浽һ������
#define FLASH_SAVE_ADDR3_Group3 0X100000
#define FLASH_SAVE_ADDR4_Group3 0X110000
#define FLASH_SAVE_ADDR5_Group3 0X120000
#define FLASH_SAVE_ADDR6_Group3 0X130000

//����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)
//128K֮ǰΪ����code���룬128K֮��Ϊ�����趨����
#define FLASH_SAVE_ADDR0 0X08020000 //���ܿ�����д������һ�α��浽һ��������ÿ������4096���ֽ�

#define FLASH_SAVE_ADDR1 0X08021000 //���ܿ�����д������һ�α��浽һ������
#define FLASH_SAVE_ADDR2 0X08022000
#define FLASH_SAVE_ADDR3 0X08023000
#define FLASH_SAVE_ADDR4 0X08024000
#define FLASH_SAVE_ADDR5 0X08025000
#define FLASH_SAVE_ADDR6 0X08026000

#define FLASH_SAVE_ADDR1_2 0X08027000
#define FLASH_SAVE_ADDR2_2 0X08028000
#define FLASH_SAVE_ADDR3_2 0X08029000
#define FLASH_SAVE_ADDR4_2 0X0802A000
#define FLASH_SAVE_ADDR5_2 0X0802B000
#define FLASH_SAVE_ADDR6_2 0X0802C000

#define FLASH_SAVE_ADDR1_3 0X0802D000
#define FLASH_SAVE_ADDR2_3 0X0802E000
#define FLASH_SAVE_ADDR3_3 0X0802F000
#define FLASH_SAVE_ADDR4_3 0X08030000
#define FLASH_SAVE_ADDR5_3 0X08031000
#define FLASH_SAVE_ADDR6_3 0X08032000

#define FLASH_SAVE_POSTION1 0X08033000 //�����¼�ĸ���λ�õ㣬��ʽ�����+6���ؽڵĶ�Ȧ���ݼ���Ȧ����

#define FLASH_SAVE_POS_CMD1 0X08034000 //ͬ������ָ��ɱ���5��ָ��
#define FLASH_SAVE_POS_CMD2 0X08035000
#define FLASH_SAVE_POS_CMD3 0X08036000
#define FLASH_SAVE_POS_CMD4 0X08037000
#define FLASH_SAVE_POS_CMD5 0X08038000

#define START_CMD_ADDR 300
#define w_ParLst_DrivePar w_ParLst_Drive[0]
#define w_ParLst_Drive1 w_ParLst_Drive[START_CMD_ADDR]
#define w_ParLst_Drive2 w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE]
#define w_ParLst_Drive3 w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 2]
#define w_ParLst_Drive4 w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 3]
#define w_ParLst_Drive5 w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 4]
#define w_ParLst_Drive6 w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 5]

//20*15=300��˫�֣�λ�����+6����Ȧ+12����Ȧ������ֵ+��־
//λ�ú�(0)
//1#��Ȧ(1)+1#��Ȧ_��(2)+1#��Ȧ_��(3)+2#��Ȧ(4)+2#��Ȧ_��(5)+2#��Ȧ_��(6)+
//3#��Ȧ(7)+3#��Ȧ_��(8)+3#��Ȧ_��(9)+4#��Ȧ(10)+4#��Ȧ_��(11)+4#��Ȧ_��(12)+
//5#��Ȧ(13)+5#��Ȧ_��(14)+5#��Ȧ_��(15)+6#��Ȧ(16)+6#��Ȧ_��(17)+6#��Ȧ_��(18)
#define w_ParLst_PosPar w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 6]

//40*30=1200��˫��
//���(0)+�����(1)+λ�ú�(2)+�����ٶ�(3)+�Ӽ���ʱ��(4)+��ͣ(5)+ֹͣ(6)+����1(7)+����2(8)+DO��ʱ(9)+DO�������ʱ��(10)+���(11)
//+1#����_��(12)+1#����_��(13)+1#�ٶ�(14)+1#�Ӽ���(15)+2#����_��(16)+2#����_��(17)+2#�ٶ�(18)+2#�Ӽ���(19)
//+3#����_��(20)+3#����_��(21)+3#�ٶ�(22)+3#�Ӽ���(23)+4#����_��(24)+4#����_��(25)+4#�ٶ�(26)+4#�Ӽ���(27)
//+5#����_��(28)+5#����_��(29)+5#�ٶ�(30)+5#�Ӽ���(31)+6#����_��(32)+6#����_��(33)+6#�ٶ�(34)+6#�Ӽ���(35)
#define w_ParLst_Pos_CMD w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 6 + POS_SIZE * POS_NUM]

//���һ������
#define w_ParLst_LastPos_CMD w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 6 + FLASH_POS_SIZE + FLASH_POS_CMD_SIZE - POS_CMD_SIZE]

//����ͨѶ�����������
#define COM_QUERY_SIZE 10 //��ѯָ���
#define COM_QUERY_NUM 10  //��ѯָ������

#define COM_CMD_SIZE 20 //����ָ���
#define COM_CMD_NUM 30  //����ָ������

#define PULSE_NUM 5000         //���ÿ��תһȦ��������
#define ELEC_GEAR 8388608      //���ӳ���2^23=8388608
#define ELEC_GEAR_US200 131072 //���ӳ���2^17=131072

#define POS_SIZE 20 //ÿ�����ݵ�ĳ��ȣ����+6����Ȧ+12����Ȧ������ֵ+��־
#define POS_NUM 15  //�������ݵ��������15��
#define FLASH_POS_SIZE POS_SIZE *POS_NUM

#define POS_CMD_SIZE 40 //����λ��ָ���
//40*30=120��˫��
//���(0)+�����(1)+λ�ú�(2)+�����ٶ�(3)+�Ӽ���ʱ��(4)+��ͣ(5)+ֹͣ(6)+����1(7)+����2(8)+DO��ʱ(9)+DO�������ʱ��(10)+���(11)
//+1#����_��(12)+1#����_��(13)+1#�ٶ�(14)+1#�Ӽ���(15)+2#����_��(16)+2#����_��(17)+2#�ٶ�(18)+2#�Ӽ���(19)
//+3#����_��(20)+3#����_��(21)+3#�ٶ�(22)+3#�Ӽ���(23)+4#����_��(24)+4#����_��(25)+4#�ٶ�(26)+4#�Ӽ���(27)
//+5#����_��(28)+5#����_��(29)+5#�ٶ�(30)+5#�Ӽ���(31)+6#����_��(32)+6#����_��(33)+6#�ٶ�(34)+6#�Ӽ���(35)
//����ʱ��(36)+������ݱ�־(37)+�ѷ��ͱ�־(38)
#define POS_CMD_NUM 30 //����λ��ָ������
#define FLASH_POS_CMD_SIZE POS_CMD_SIZE *POS_CMD_NUM

#define MOTOR_MAX_SPEED12 1500  //1#��2#������ת��1500r/min
#define MOTOR_MAX_SPEED3 2000   //3#������ת��2000r/min
#define MOTOR_MAX_SPEED456 3000 //3#������ת��3000r/min
#define D_RUN_TIME 36           //��������ʱ�䣬������ָ������λ��36

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

#define Pw_Driver1_Speed w_ParLst_Drive[27]   //1#�ֶ������ٶ�
#define Pw_Driver1_AccTime w_ParLst_Drive[28] //1#�ֶ��Ӽ���ʱ��

#define Pw_Driver2_Speed w_ParLst_Drive[30]   //2#�ֶ������ٶ�
#define Pw_Driver2_AccTime w_ParLst_Drive[31] //2#�ֶ��Ӽ���ʱ��

#define Pw_Driver3_Speed w_ParLst_Drive[33]   //3#�ֶ������ٶ�
#define Pw_Driver3_AccTime w_ParLst_Drive[34] //3#�ֶ��Ӽ���ʱ��

#define Pw_Driver4_Speed w_ParLst_Drive[36]   //4#�ֶ������ٶ�
#define Pw_Driver4_AccTime w_ParLst_Drive[37] //4#�ֶ��Ӽ���ʱ��

#define Pw_Driver5_Speed w_ParLst_Drive[39]   //5#�ֶ������ٶ�
#define Pw_Driver5_AccTime w_ParLst_Drive[40] //5#�ֶ��Ӽ���ʱ��

#define Pw_Driver6_Speed w_ParLst_Drive[42]   //6#�ֶ������ٶ�
#define Pw_Driver6_AccTime w_ParLst_Drive[43] //6#�ֶ��Ӽ���ʱ��

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
#define Pw_SF_Driver1_InitPos_HW w_ParLst_Drive[64] //�ŷ����1�ĳ�ʼλ��
#define Pw_SF_Driver2_InitPos w_ParLst_Drive[65]    //�ŷ����2�ĳ�ʼλ��
#define Pw_SF_Driver2_InitPos_HW w_ParLst_Drive[66] //�ŷ����2�ĳ�ʼλ��
#define Pw_SF_Driver3_InitPos w_ParLst_Drive[67]    //�ŷ����3�ĳ�ʼλ��
#define Pw_SF_Driver3_InitPos_HW w_ParLst_Drive[67] //�ŷ����3�ĳ�ʼλ��
#define Pw_SF_Driver4_InitPos w_ParLst_Drive[69]    //�ŷ����4�ĳ�ʼλ��
#define Pw_SF_Driver4_InitPos_HW w_ParLst_Drive[70] //�ŷ����4�ĳ�ʼλ��
#define Pw_SF_Driver5_InitPos w_ParLst_Drive[71]    //�ŷ����5�ĳ�ʼλ��
#define Pw_SF_Driver5_InitPos_HW w_ParLst_Drive[72] //�ŷ����5�ĳ�ʼλ��
#define Pw_SF_Driver6_InitPos w_ParLst_Drive[73]    //�ŷ����6�ĳ�ʼλ��
#define Pw_SF_Driver6_InitPos_HW w_ParLst_Drive[74] //�ŷ����6�ĳ�ʼλ��

#define Pw_Fault_Stop w_ParLst_Drive[75]      // �����ŷ�����ͣ�����ܣ�=1��ͣ����=0����ͣ������Ȼ����
#define Pw_Read_CurrentPos w_ParLst_Drive[76] // ����ʼλ�ã�=1����λ��
#define Pw_EMERGENCY_STOP w_ParLst_Drive[77]  // ��ͣ���=1�����м�ͣ
#define Pw_ResetCMD w_ParLst_Drive[78]        // ��λ���=1�����и�λ
#define Pw_Stop_Reset w_ParLst_Drive[79]      // ͣ����λ���ܣ�=1��ͣ������и�λ��=0��ͣ���󲻸�λ

#define Pw_TouchRunStop w_ParLst_Drive[80] // ���� ����/ֹͣ
#define Pw_SaveDelay w_ParLst_Drive[81]    //Ĭ����ʱ180s�����޸ĺ�Ĳ�����FLASH��

//#define	Pw_Driver1_Par_Init_F		w_ParLst_Drive[82]		//1#�ŷ����������ʼ����־
//#define	Pw_Driver2_Par_Init_F		w_ParLst_Drive[83]		//2#�ŷ����������ʼ����־
//#define	Pw_Driver3_Par_Init_F		w_ParLst_Drive[84]		//3#�ŷ����������ʼ����־
//#define	Pw_Driver4_Par_Init_F		w_ParLst_Drive[85]		//4#�ŷ����������ʼ����־
//#define	Pw_Driver5_Par_Init_F		w_ParLst_Drive[86]		//5#�ŷ����������ʼ����־
//#define	Pw_Driver6_Par_Init_F		w_ParLst_Drive[87]		//6#�ŷ����������ʼ����־

//#define	Pw_Reset_MaxValue			w_ParLst_Drive[88]		//��λ����޶�ֵ
//#define	Pw_Reset_MaxValue_HW		w_ParLst_Drive[89]		//��λ����޶�ֵ�����֣�

#define Pr_current_pos1 w_ParLst_Drive[90] //1#�ŷ����λ��ָ�����������Ӧ�ŷ���ַP090A
#define Pr_current_pos1_HW w_ParLst_Drive[91]
#define Pr_current_pos2 w_ParLst_Drive[92]     //2#�����ǰ����
#define Pr_current_pos2_HW w_ParLst_Drive[93]  //2#�����ǰ����
#define Pr_current_pos3 w_ParLst_Drive[94]     //3#�����ǰ����
#define Pr_current_pos3_HW w_ParLst_Drive[95]  //3#�����ǰ����
#define Pr_current_pos4 w_ParLst_Drive[96]     //4#�����ǰ����
#define Pr_current_pos4_HW w_ParLst_Drive[97]  //4#�����ǰ����
#define Pr_current_pos5 w_ParLst_Drive[98]     //5#�����ǰ����
#define Pr_current_pos5_HW w_ParLst_Drive[99]  //5#�����ǰ����
#define Pr_current_pos6 w_ParLst_Drive[100]    //6#�����ǰ����
#define Pr_current_pos6_HW w_ParLst_Drive[101] //6#�����ǰ����

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
