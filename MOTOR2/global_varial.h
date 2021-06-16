/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBAL_VARIAL_H
#define __GLOBAL_VARIAL_H

/****************Start*******************/
#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "Macro.h"
#include "typedef.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdarg.h>
//

#define MIN(a, b) (a < b) ? (a) : (b)
#define MAX(a, b) (a > b) ? (a) : (b)
#define rt_int8_t int8_t
#define rt_int16_t int16_t
#define rt_int32_t int32_t

#define rt_uint8_t uint8_t
#define rt_uint16_t uint16_t
#define rt_uint32_t uint32_t

#define IDLE 0
#define ACCELERATING 1
#define AT_MAX 2
#define DECELERATING 3

typedef __packed struct
{
	unsigned char en;		   //ʹ��
	unsigned char dir;		   //����
	unsigned char running;	   //ת����ɱ�־
	unsigned char rstflg;	   //��λ��־
	unsigned char divnum;	   //��Ƶ��
	unsigned char speedenbale; //�Ƿ�ʹ���ٶȿ���
	unsigned char clockwise;   //˳ʱ�뷽���Ӧ��ֵ
	unsigned char id;		   //���id

	uint32_t pulsecount;
	uint16_t *Counter_Table;		//ָ������ʱ��ʱ�����������
	uint16_t *Step_Table;			//ָ������ʱ��ÿ��Ƶ�����������
	uint16_t CurrentIndex;			//��ǰ���λ��
	uint16_t TargetIndex;			//Ŀ���ٶ��ڱ���λ��
	uint16_t StartTableLength;		//�������ݱ�
	uint16_t StopTableLength;		//�������ݱ�
	uint32_t StartSteps;			//�����������
	uint32_t StopSteps;				//���ֹͣ����
	uint32_t RevetDot;				//����˶��ļ��ٵ�
	uint32_t PulsesGiven;			//����˶����ܲ���
	uint32_t PulsesHaven;			//����Ѿ����еĲ���
	uint32_t CurrentPosition;		//��ǰλ��
	uint32_t MaxPosition;			//���λ�ã�������λ����0
	uint32_t CurrentPosition_Pulse; //��ǰλ��
	uint32_t MaxPosition_Pulse;		//��ǰλ��
	uint32_t target_pos;			//Ŀ��λ��  YLS 2021.05.23 ��EtherCAT

	unsigned long long Time_Cost_Act; //ʵ����ת���ѵ�ʱ��
	unsigned long long Time_Cost_Cal; //����Ԥ����ת���ѵ�ʱ��
	TIM_TypeDef *TIMx;
} MOTOR_CONTROL_S;

#define M1_CLOCKWISE 0
#define M1_UNCLOCKWISE 1
#define M2_CLOCKWISE 0
#define M2_UNCLOCKWISE 1
#define M3_CLOCKWISE 0
#define M3_UNCLOCKWISE 1
#define M4_CLOCKWISE 0
#define M4_UNCLOCKWISE 1
#define M5_CLOCKWISE 0
#define M5_UNCLOCKWISE 1
#define M6_CLOCKWISE 0
#define M6_UNCLOCKWISE 1

#define PWM1_PreemptionPriority 1 //�׼�
#define PWM1_SubPriority 0		  //�ײ�
#define PWM2_PreemptionPriority 1 //�׼�
#define PWM2_SubPriority 1		  //�ײ�
#define PWM3_PreemptionPriority 2 //�׼�
#define PWM3_SubPriority 0		  //�ײ�
#define PWM4_PreemptionPriority 2 //�׼�
#define PWM4_SubPriority 1		  //�ײ�
/***********************END********************************************/

//���
extern MOTOR_CONTROL_S motor1;
extern MOTOR_CONTROL_S motor2;
extern MOTOR_CONTROL_S motor3;
extern MOTOR_CONTROL_S motor4;
extern MOTOR_CONTROL_S motor5;
extern MOTOR_CONTROL_S motor6;

extern uint16_t Motor1TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor1StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor2TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor2StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor3TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor3StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor4TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor4StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor5TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor5StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor6TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor6StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
//extern struct rt_ringbuffer rb_recv;

void USART1_Initial(void);
void USART1_Printf(unsigned char *q, unsigned char len);
void USART1_Printfstr(unsigned char *p);
void Initial_MotorIO(void);
void Initial_Motor(unsigned char MotorID, unsigned char StepDive, unsigned int maxposition);
void MotorRunParaInitial(void);
void Start_Motor12(unsigned char dir1, unsigned int Degree1, unsigned char dir2, unsigned int Degree2);
void Start_Motor_S(unsigned char MotorID, unsigned char dir, uint32_t Degree);
void Start_Motor_SPTA(unsigned char MotorID, unsigned char dir, uint32_t Degree, uint32_t MaxSpeed_SPTA, uint32_t AccSpeed_SPTA);
void SetSpeed(unsigned char MotorID, signed char speedindex);
void Do_Reset(unsigned char MotorID);
void Deal_Cmd(void);
void Initial_PWM_Motor1(void);
void Initial_PWM_Motor2(void);
void Initial_PWM_Motor3(void);
void Initial_PWM_Motor4(void);
void Initial_PWM_Motor5(void);
void Initial_PWM_Motor6(void);
void EXTI_Configuration(void);
void CalcMotorPeriStep_CPF(float fstart, float faa, uint16_t step_para, float taa, float tua, float tra, uint16_t MotorTimeTable[], uint16_t MotorStepTable[]);
void Run_Motor_S(unsigned char MotorID, unsigned char dir, uint32_t Degree, uint32_t StartSpeed, uint32_t MaxSpeed_S, uint32_t AccSpeed_Para);
void sort_long(unsigned long long *a, int l);
void Run_Motors_sync(u8 dir1, u32 Degree1, u16 StartSpeed1, u16 SetSpeed1, u16 AccSpeed1, u8 dir2, u32 Degree2, u16 StartSpeed2, u16 SetSpeed2, u16 AccSpeed2,
					 u8 dir3, u32 Degree3, u16 StartSpeed3, u16 SetSpeed3, u16 AccSpeed3, u8 dir4, u32 Degree4, u16 StartSpeed4, u16 SetSpeed4, u16 AccSpeed4,
					 u8 dir5, u32 Degree5, u16 StartSpeed5, u16 SetSpeed5, u16 AccSpeed5, u8 dir6, u32 Degree6, u16 StartSpeed6, u16 SetSpeed6, u16 AccSpeed6);
#endif
