//ͷ�ļ�
#include "include.h"
#include "bsp_MOTOR1.h"
#include "bsp_MOTOR2.h"
#include "bsp_MOTOR3.h"
#include "bsp_MOTOR4.h"
#include "bsp_MOTOR5.h"
#include "bsp_MOTOR6.h"
#include "GlobalConst.h"
// #include "GlobalV.h"
#include "Macro.h"

uint16_t Motor1TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor1StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor2TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor2StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor3TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor3StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor4TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor4StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor5TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor5StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor6TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor6StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};

unsigned long long Get_Time_Cost(unsigned char MotorID);
extern void MOTOR1_GPIO_Init(void);
extern void MOTOR2_GPIO_Init(void);
extern void MOTOR3_GPIO_Init(void);
extern void MOTOR4_GPIO_Init(void);
extern void MOTOR5_GPIO_Init(void);
extern void MOTOR6_GPIO_Init(void);

extern uint16_t w_ParLst[840]; // �������б���

/**************************************************************************************
 ��ʼ������Ĳ�������Ҫ��ϸ��ѡ��ʹ�õĶ�ʱ����˳ʱ�뷽��ֵ�����ID��
 **************************************************************************************/
void Initial_Motor(unsigned char MotorID, unsigned char StepDive, unsigned int maxposition)
{
	unsigned int i = 0;
	MOTOR_CONTROL_S *pmotor = NULL;
	uint16_t *MotorTimeTable;
	uint16_t *MotorStepTable;

	switch (MotorID)
	{
	case 1:
		pmotor = &motor1;
		motor1.id = 1;
		motor1.clockwise = M1_CLOCKWISE;
		motor1.TIMx = TIM2;
		MotorTimeTable = Motor1TimeTable;
		MotorStepTable = Motor1StepTable;
		break;
	case 2:
		pmotor = &motor2;
		motor2.id = 2;
		motor2.clockwise = M2_CLOCKWISE;
		motor2.TIMx = TIM4;
		MotorTimeTable = Motor2TimeTable;
		MotorStepTable = Motor2StepTable;
		break;
	case 3:
		pmotor = &motor3;
		motor3.id = 3;
		motor3.clockwise = M3_CLOCKWISE;
		motor3.TIMx = TIM8;
		MotorTimeTable = Motor3TimeTable;
		MotorStepTable = Motor3StepTable;
		break;
	case 4:
		pmotor = &motor4;
		motor4.id = 4;
		motor4.clockwise = M4_CLOCKWISE;
		motor4.TIMx = TIM3;
		MotorTimeTable = Motor4TimeTable;
		MotorStepTable = Motor4StepTable;
		break;
	case 5:
		pmotor = &motor5;
		motor5.id = 5;
		motor5.clockwise = M5_CLOCKWISE;
		motor5.TIMx = TIM1;
		MotorTimeTable = Motor5TimeTable;
		MotorStepTable = Motor5StepTable;
		break;
	case 6:
		pmotor = &motor6;
		motor6.id = 6;
		motor6.clockwise = M6_CLOCKWISE;
		motor6.TIMx = TIM5;
		MotorTimeTable = Motor6TimeTable;
		MotorStepTable = Motor6StepTable;
		break;
	default:
		break;
	}

	if (MotorID <= 6 && MotorID >= 1)
	{
		pmotor->divnum = StepDive;
		pmotor->MaxPosition = maxposition;
		pmotor->MaxPosition_Pulse = maxposition * StepDive;

		pmotor->CurrentPosition = 0;
		pmotor->CurrentPosition_Pulse = 0;
		pmotor->StartTableLength = STEP_AA + STEP_UA + STEP_RA + 1;
		pmotor->StopTableLength = STEP_AA + STEP_UA + STEP_RA;
		pmotor->Counter_Table = MotorTimeTable;
		pmotor->Step_Table = MotorStepTable;

		pmotor->CurrentIndex = 0;
		pmotor->speedenbale = 0;
		pmotor->StartSteps = 0; //�������㣬�������ۼӣ�������ǰһ�εļ���
		pmotor->StopSteps = 0;	//ͬ��
		for (i = 0; i < pmotor->StartTableLength; i++)
			pmotor->StartSteps += pmotor->Step_Table[i];
		for (i = 0; i < pmotor->StopTableLength; i++)
			pmotor->StopSteps += pmotor->Step_Table[i + pmotor->StartTableLength];

		pmotor->TIMx->ARR = pmotor->Counter_Table[0]; //��������
		// pmotor->TIMx->CCR1 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
		if (pmotor->TIMx == TIM2)
		{
			TIM2->CCR2 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
			TIM2->CNT = 0;
		}
		else if (pmotor->TIMx == TIM4)
		{
			TIM4->CCR1 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
			TIM4->CNT = 0;
		}
		else if (pmotor->TIMx == TIM8)
		{
			TIM8->CCR1 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
			TIM8->CNT = 0;
		}
		else if (pmotor->TIMx == TIM3)
		{
			TIM3->CCR4 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
			TIM3->CNT = 0;
		}
		else if (pmotor->TIMx == TIM1)
		{
			TIM1->CCR1 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
			TIM1->CNT = 0;
		}
		else if (pmotor->TIMx == TIM5)
		{
			TIM5->CCR1 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
			TIM5->CNT = 0;
		}
	}
}

/*����Эͬʹ�����㷨ԭ�����ʱ��Ԥ���������޸ĸ��㷨ʱ�ǵ�
 ����������ͬ��*/
/*����S�����߷�ת�㣬S������������ʱ���Ӽ��ٹ�������ȫ�ԳƵ�*/
unsigned long long Get_TimeCost_ReverDot_S(unsigned char MotorID)
{
	unsigned long long time_cost = 0;
	unsigned long long time_cost2 = 0;
	unsigned int pulsecnt = 0;
	int i = 0, j;
	MOTOR_CONTROL_S *pmotor = NULL;
	switch (MotorID)
	{
	case 1:
		pmotor = &motor1;
		break;
	case 2:
		pmotor = &motor2;
		break;
	case 3:
		pmotor = &motor3;
		break;
	case 4:
		pmotor = &motor4;
		break;
	case 5:
		pmotor = &motor5;
		break;
	case 6:
		pmotor = &motor6;
		break;
	default:
		return 0;
	}

	if (pmotor->PulsesGiven >= pmotor->StartSteps + pmotor->StopSteps)
	{
		for (i = 0; i < pmotor->StartTableLength; i++)
			time_cost += (pmotor->Step_Table[i] * pmotor->Counter_Table[i]);
		for (i = 0; i < pmotor->StopTableLength; i++)
			time_cost += (pmotor->Step_Table[i + pmotor->StartTableLength] * pmotor->Counter_Table[i + pmotor->StartTableLength]);
		time_cost += (pmotor->PulsesGiven - pmotor->StartSteps - pmotor->StopSteps) * pmotor->Counter_Table[pmotor->StartTableLength - 1];

		pmotor->RevetDot = pmotor->PulsesGiven - pmotor->StopSteps;
	}
	else
	{
		//���������������һƵ��142 �����ڶ�Ƶ��148����Ҫ���˶�200������ô����
		//��������Ҫ�ı�ڶ�Ƶ�ʵĲ���
		while ((pulsecnt + pmotor->Step_Table[i]) <= (pmotor->PulsesGiven >> 1))
		{
			time_cost += (pmotor->Step_Table[i] * pmotor->Counter_Table[i]);
			time_cost2 += (pmotor->Step_Table[i] * pmotor->Counter_Table[i]);
			pulsecnt += pmotor->Step_Table[i];
			i++;
		}
		time_cost += time_cost2;
		if (pmotor->Step_Table[i] < pmotor->PulsesGiven - 2 * pulsecnt)
		{
			pmotor->Step_Table[i] = pmotor->PulsesGiven - 2 * pulsecnt;
			pmotor->StartSteps = 0; //�������㣬�������ۼӣ�������ǰһ�εļ���
			pmotor->StopSteps = 0;	//ͬ��
			for (j = 0; j < pmotor->StartTableLength; j++)
				pmotor->StartSteps += pmotor->Step_Table[j];
			for (j = 0; j < pmotor->StopTableLength; j++)
				pmotor->StopSteps += pmotor->Step_Table[j + pmotor->StartTableLength];
		}
		time_cost += (pmotor->Counter_Table[i] * (pmotor->PulsesGiven - 2 * pulsecnt));
		pmotor->RevetDot = pmotor->PulsesGiven - pulsecnt;
	}
	pmotor->Time_Cost_Cal = time_cost;
	return time_cost;
}
// unsigned long long Get_Time_Cost2(unsigned char MotorID)
// {
// 	extern void TIM1_UP_IRQHandler(void);
// 	extern void TIM2_IRQHandler(void);
// 	extern void TIM3_IRQHandler(void);
// 	switch (MotorID)
// 	{
// 	case 1:
// 		while (motor1.running == 1)
// 		{
// 			TIM1_CC_IRQHandler();
// 		}
// 		return motor1.Time_Cost_Act;
// 	case 2:
// 		while (motor2.running == 1)
// 		{
// 			TIM2_IRQHandler();
// 		}
// 		return motor2.Time_Cost_Act;
// 	case 3:
// 		while (motor3.running == 1)
// 		{
// 			TIM3_IRQHandler();
// 		}
// 		return motor3.Time_Cost_Act;
// 	}
// 	return 0;
// }

/*���³�ʼ���������ʱ��ز���*/
void Motor_Reinitial(unsigned char MotorID)
{
	int i = 0;
	MOTOR_CONTROL_S *pmotor = NULL;
	switch (MotorID)
	{
	case 1:
		pmotor = &motor1;
		break;
	case 2:
		pmotor = &motor2;
		break;
	case 3:
		pmotor = &motor3;
		break;
	case 4:
		pmotor = &motor4;
		break;
	case 5:
		pmotor = &motor5;
		break;
	case 6:
		pmotor = &motor6;
		break;
	default:
		return;
	}
	pmotor->pulsecount = 0;
	pmotor->CurrentIndex = 0;
	pmotor->speedenbale = 0;
	pmotor->StartSteps = 0; //�������㣬�������ۼӣ�������ǰһ�εļ���
	pmotor->StopSteps = 0;	//ͬ��
	for (i = 0; i < pmotor->StartTableLength; i++)
		pmotor->StartSteps += pmotor->Step_Table[i];
	for (i = 0; i < pmotor->StopTableLength; i++)
		pmotor->StopSteps += pmotor->Step_Table[i + pmotor->StartTableLength];
	pmotor->TIMx->ARR = pmotor->Counter_Table[0]; //��������
	// pmotor->TIMx->CCR1 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
	if (pmotor->TIMx == TIM2)
	{
		TIM2->CCR2 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
		TIM2->CNT = 0;
	}
	else if (pmotor->TIMx == TIM4)
	{
		TIM4->CCR1 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
		TIM4->CNT = 0;
	}
	else if (pmotor->TIMx == TIM8)
	{
		TIM8->CCR1 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
		TIM8->CNT = 0;
	}
	else if (pmotor->TIMx == TIM3)
	{
		TIM3->CCR4 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
		TIM3->CNT = 0;
	}
	else if (pmotor->TIMx == TIM1)
	{
		TIM1->CCR1 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
		TIM1->CNT = 0;
	}
	else if (pmotor->TIMx == TIM5)
	{
		TIM5->CCR1 = pmotor->Counter_Table[0] >> 1; //����ռ�ձ�
		TIM5->CNT = 0;
	}
	pmotor->Time_Cost_Act = pmotor->TIMx->ARR;
	Get_TimeCost_ReverDot_S(MotorID);
}

/*����S�����߲�����ȡĳ��ʱ�̵�Ƶ��*/ //t���ǵ�ǰ���е�ʱ��
float GetFreAtTime(float fstart, float faa, float taa, float tua, float tra, float t)
{
	//���ݹ�ʽ����ӿ�ʼ������ٹ����У�tʱ�̵�ת��Ƶ��
	if (t >= 0 && t <= taa)
	{
		//�Ӽ��ٽ׶�
		return fstart + Motor1_XiShu_1 * faa * t * t; //�����߷�
	}
	else if (taa < t && t <= (taa + tua))
	{
		//�ȼ��ٽ׶�
		return fstart + Motor1_XiShu_1 * faa * taa * taa + (t - taa) * faa * taa;
	}
	else if ((taa + tua) < t && t <= (taa + tua + tra))
	{
		//�����ٽ׶�
		return fstart + Motor1_XiShu_1 * faa * taa * taa + (tua)*faa * taa + Motor1_XiShu_1 * faa * taa * tra - Motor1_XiShu_1 * faa * taa * (taa + tua + tra - t) * (taa + tua + tra - t) / (tra);
	}
	return 0;
}

/*����S�������㷨��ÿһ����ʱ�����ڼ�������*/
/*���룺
fstart:��ʼƵ��
faa:���ٶ�
step_para:���ٶȵ���
taa:�Ӽ��ٶ���
tua:�ȼ��ٶ���
tra:�����ٶ���
MotorTimeTable[]:���ʱ���
MotorStepTable[]:����������
*/
void CalcMotorPeriStep_CPF(float fstart, float faa, uint16_t step_para, float taa, float tua, float tra, uint16_t MotorTimeTable[], uint16_t MotorStepTable[])
{
	int i;
	float fi;

	for (i = 0; i < STEP_AA; i++)
	{
		fi = GetFreAtTime(fstart, faa, taa, tua, tra, taa / STEP_AA * i);
		MotorTimeTable[i] = F2TIME_PARA / fi;				  //ÿ����ʱ�����ڣ�����ط�Ҫע�⣬�������������YLS 2021.05.07
		MotorStepTable[i] = fi * (taa / STEP_AA) / step_para; //ÿ��ʱ���������еĲ���
	}
	for (i = STEP_AA; i < STEP_AA + STEP_UA; i++)
	{
		fi = GetFreAtTime(fstart, faa, taa, tua, tra, taa + (tua / STEP_UA) * (i - STEP_AA));
		MotorTimeTable[i] = F2TIME_PARA / fi;
		MotorStepTable[i] = fi * (tua / STEP_UA) / step_para;
	}
	for (i = STEP_AA + STEP_UA; i < STEP_AA + STEP_UA + STEP_RA; i++)
	{
		fi = GetFreAtTime(fstart, faa, taa, tua, tra, taa + tua + tra / STEP_RA * (i - STEP_AA - STEP_UA));
		MotorTimeTable[i] = F2TIME_PARA / fi;
		MotorStepTable[i] = fi * (tra / STEP_RA) / step_para;
	}
	fi = GetFreAtTime(fstart, faa, taa, tua, tra, taa + tua + tra);
	MotorTimeTable[STEP_AA + STEP_UA + STEP_RA] = F2TIME_PARA / fi;
	MotorStepTable[STEP_AA + STEP_UA + STEP_RA] = fi * (tra / STEP_RA) / step_para;

	for (i = STEP_AA + STEP_UA + STEP_RA + 1; i < 2 * (STEP_AA + STEP_UA + STEP_RA) + 1; i++)
	{
		MotorTimeTable[i] = MotorTimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) - i];
		MotorStepTable[i] = MotorStepTable[2 * (STEP_AA + STEP_UA + STEP_RA) - i];
	}
}

/**************************************************************************************
������в�����ʼ��*/
void MotorRunParaInitial(void)
{
	/*FIXME:�û����Ըı�ò���ʵ��S�����ߵ���������*/
	CalcMotorPeriStep_CPF(Pw_Motor1_FRE_START, Pw_Motor1_FRE_AA, Pw_Motor1_STEP_PARA, M_T_AA, M_T_UA, M_T_RA, Motor1TimeTable, Motor1StepTable);
	CalcMotorPeriStep_CPF(Pw_Motor2_FRE_START, Pw_Motor2_FRE_AA, Pw_Motor2_STEP_PARA, M_T_AA, M_T_UA, M_T_RA, Motor2TimeTable, Motor2StepTable);
	CalcMotorPeriStep_CPF(Pw_Motor3_FRE_START, Pw_Motor3_FRE_AA, Pw_Motor3_STEP_PARA, M_T_AA, M_T_UA, M_T_RA, Motor3TimeTable, Motor3StepTable);
	CalcMotorPeriStep_CPF(Pw_Motor4_FRE_START, Pw_Motor4_FRE_AA, Pw_Motor4_STEP_PARA, M_T_AA, M_T_UA, M_T_RA, Motor4TimeTable, Motor4StepTable);
	CalcMotorPeriStep_CPF(Pw_Motor5_FRE_START, Pw_Motor5_FRE_AA, Pw_Motor5_STEP_PARA, M_T_AA, M_T_UA, M_T_RA, Motor5TimeTable, Motor5StepTable);
	CalcMotorPeriStep_CPF(Pw_Motor6_FRE_START, Pw_Motor6_FRE_AA, Pw_Motor6_STEP_PARA, M_T_AA, M_T_UA, M_T_RA, Motor6TimeTable, Motor6StepTable);
}

/**************************************************************************************
�������ͬʱ����ʱ������ʱ����ٵĵ��Ҫ����ʱ�䳤�ĵ���������в���
*/
void Find_BestTimeCost(unsigned char ID, unsigned long long time_cost, unsigned char dir, unsigned int Degree)
{
	int id = ID;
	float i = 0, j = 0;
	float fi = M_FRE_START, fj = M_FRE_START;
	int cal_ij = 1;
	unsigned int PulsesGiven = 0;
	uint16_t *MotorTimeTable;
	uint16_t *MotorStepTable;
	MOTOR_CONTROL_S *pmotor = NULL;
	unsigned long long time_cost_tmp = 0;
	unsigned long long time_cost_min = 0xffffffffff;
	float i_o = 0, j_o = 0;
	float fi_o = 0, fj_o = 0;
	switch (ID)
	{
	case 1:
		pmotor = &motor1;
		MotorTimeTable = Motor1TimeTable;
		MotorStepTable = Motor1StepTable;
		break;
	case 2:
		pmotor = &motor2;
		MotorTimeTable = Motor2TimeTable;
		MotorStepTable = Motor2StepTable;
		break;
	}
	if (pmotor == NULL)
	{
		return;
	}
	j = M_FRE_AA;
	i = 0;
	while (1)
	{
		if (cal_ij)
		{
			CalcMotorPeriStep_CPF(M_FRE_START, (i + j) / 2.0, 100, M_T_AA, M_T_UA, M_T_RA, MotorTimeTable, MotorStepTable);
		}
		else
		{
			CalcMotorPeriStep_CPF((fi + fj) / 2, 0, 100, M_T_AA, M_T_UA, M_T_RA, MotorTimeTable, MotorStepTable);
		}
		pmotor->en = 1;
		pmotor->dir = dir;
		pmotor->running = 1;
		pmotor->PulsesHaven = 0;
		PulsesGiven = Degree;
		pmotor->Time_Cost_Act = 0;
		pmotor->PulsesGiven = PulsesGiven * pmotor->divnum;
		//pmotor->PulsesGiven+=300;
		Motor_Reinitial(id);
		//time_cost_tmp=Get_Time_Cost2(id);
		time_cost_tmp = pmotor->Time_Cost_Cal;
		if (time_cost_tmp < time_cost)
		{
			if (time_cost - time_cost_tmp < time_cost_min)
			{
				time_cost_min = time_cost - time_cost_tmp;
				i_o = i;
				j_o = j;
				fi_o = fi;
				fj_o = fj;
			}
		}
		else
		{
			if (time_cost_tmp - time_cost < time_cost_min)
			{
				time_cost_min = time_cost_tmp - time_cost;
				i_o = i;
				j_o = j;
				fi_o = fi;
				fj_o = fj;
			}
		}
		if (time_cost_tmp >= time_cost - 32 * 4 && time_cost_tmp <= time_cost + 32 * 4)
		{
			break;
		}
		if (cal_ij)
		{
			if (j < 0.1)
			{
				//˵����ʹ��ʹ��������������ٶȶ��޷�ͬʱֹͣ�����޸�Ϊ�����ٶȵ�����
				i_o = 0;
				j_o = 0;
				j = 0;
				i = 0;
				cal_ij = 0;
				fi = 0;
			}
		}
		if (cal_ij)
		{
			if ((i > j && i - j < 0.02 || (i < j) && j - i < 0.02))
			{
				break;
			}
			if (time_cost_tmp > time_cost)
			{
				i = (i + j) / 2.0;
			}
			else
			{
				j = (i + j) / 2.0;
			}
		}
		else
		{
			if ((fi > fj && fi - fj < 0.02 || (fi < fj) && fj - fi < 0.02))
			{
				break;
			}
			if (time_cost_tmp > time_cost)
			{
				fi = (fi + fj) / 2.0;
			}
			else
			{
				fj = (fi + fj) / 2.0;
			}
		}
	}
	CalcMotorPeriStep_CPF((fi_o + fj_o) / 2, (i_o + j_o) / 2.0, 100, M_T_AA, M_T_UA, M_T_RA, MotorTimeTable, MotorStepTable);
}

/**************************************************************************************
�����������S�����߲�������*/
void Start_Motor_S(unsigned char MotorID, unsigned char dir, uint32_t Degree)
{
	unsigned int PulsesGiven = 0;
	MOTOR_CONTROL_S *pmotor = NULL;
	if (Degree == 0)
	{
		return;
	}
	switch (MotorID)
	{
	case 1:
		pmotor = &motor1;
		if (0 == dir)
			MOTOR1_DIR_FORWARD();
		else
			MOTOR1_DIR_REVERSAL();

		pmotor->en = 1;
		pmotor->dir = dir;
		pmotor->running = 1;
		pmotor->PulsesHaven = 0;
		PulsesGiven = Degree;
		pmotor->Time_Cost_Act = 0;
		pmotor->PulsesGiven = PulsesGiven * pmotor->divnum;
		Motor_Reinitial(MotorID);
		TIM2->CR1 |= (TIM_CR1_CEN); //������ʱ��
		break;
	case 2:
		pmotor = &motor2;
		if (1 == dir)
			MOTOR2_DIR_FORWARD();
		else
			MOTOR2_DIR_REVERSAL();

		pmotor->en = 1;
		pmotor->dir = dir;
		pmotor->running = 1;
		pmotor->PulsesHaven = 0;
		PulsesGiven = Degree;
		pmotor->Time_Cost_Act = 0;
		pmotor->PulsesGiven = PulsesGiven * pmotor->divnum;
		Motor_Reinitial(MotorID);
		TIM4->CR1 |= (TIM_CR1_CEN);
		break;
	case 3:
		pmotor = &motor3;
		if (0 == dir)
			MOTOR3_DIR_FORWARD();
		else
			MOTOR3_DIR_REVERSAL();

		pmotor->en = 1;
		pmotor->dir = dir;
		pmotor->running = 1;
		pmotor->PulsesHaven = 0;
		PulsesGiven = Degree;
		pmotor->Time_Cost_Act = 0;
		pmotor->PulsesGiven = PulsesGiven * pmotor->divnum;
		Motor_Reinitial(MotorID);
		TIM8->CR1 |= (TIM_CR1_CEN);
		break;
	case 4:
		pmotor = &motor4;
		if (0 == dir)
			MOTOR4_DIR_FORWARD();
		else
			MOTOR4_DIR_REVERSAL();

		pmotor->en = 1;
		pmotor->dir = dir;
		pmotor->running = 1;
		pmotor->PulsesHaven = 0;
		PulsesGiven = Degree;
		pmotor->Time_Cost_Act = 0;
		pmotor->PulsesGiven = PulsesGiven * pmotor->divnum;
		Motor_Reinitial(MotorID);
		TIM3->CR1 |= (TIM_CR1_CEN);
		break;
	case 5:
		pmotor = &motor5;
		if (0 == dir)
			MOTOR5_DIR_FORWARD();
		else
			MOTOR5_DIR_REVERSAL();

		pmotor->en = 1;
		pmotor->dir = dir;
		pmotor->running = 1;
		pmotor->PulsesHaven = 0;
		PulsesGiven = Degree;
		pmotor->Time_Cost_Act = 0;
		pmotor->PulsesGiven = PulsesGiven * pmotor->divnum;
		Motor_Reinitial(MotorID);
		TIM1->CR1 |= (TIM_CR1_CEN);
		break;
	case 6:
		pmotor = &motor6;
		if (0 == dir)
			MOTOR6_DIR_FORWARD();
		else
			MOTOR6_DIR_REVERSAL();

		pmotor->en = 1;
		pmotor->dir = dir;
		pmotor->running = 1;
		pmotor->PulsesHaven = 0;
		PulsesGiven = Degree;
		pmotor->Time_Cost_Act = 0;
		pmotor->PulsesGiven = PulsesGiven * pmotor->divnum;
		Motor_Reinitial(MotorID);
		TIM5->CR1 |= (TIM_CR1_CEN);
		break;
	default:
		return;
	}
}

/*���¶�λ�����������еĵ�����е�ָ��λ��*/
void Reposition_Motor(unsigned char MotorID, unsigned int NewPos, uint32_t MaxSpeed_SPTA, uint32_t AccSpeed_SPTA)
{
	MOTOR_CONTROL_S *pmotor_s = NULL;
	switch (MotorID)
	{
	case 1:
		pmotor_s = &motor1;
		break;
	case 2:
		pmotor_s = &motor2;
		break;
	case 3:
		pmotor_s = &motor3;
		break;
	case 4:
		pmotor_s = &motor4;
		break;
	case 5:
		pmotor_s = &motor5;
		break;
	case 6:
		pmotor_s = &motor6;
		break;
	default:
		return;
	}
	if (pmotor_s != NULL)
	{
		if (NewPos <= pmotor_s->MaxPosition && NewPos != pmotor_s->CurrentPosition)
		{
			if (NewPos > pmotor_s->CurrentPosition)
			{
				// Start_Motor_S(MotorID, pmotor_s->clockwise, NewPos - pmotor_s->CurrentPosition);
			}
			else
			{
				// Start_Motor_S(MotorID, !pmotor_s->clockwise, pmotor_s->CurrentPosition - NewPos);
			}
			while (pmotor_s->running == 1)
				;
		}
	}
}
/*ͬʱ�������12*/
void Start_Motor12(unsigned char dir1, unsigned int Degree1, unsigned char dir2, unsigned int Degree2)
{

	unsigned int PulsesGiven = 0;
	unsigned long long time_cost = 0;
	if (Degree1 == 0 || Degree2 == 0)
	{
		return;
	}
	MotorRunParaInitial();
	Motor_Reinitial(1);
	Motor_Reinitial(2);
	if (Degree1 <= Degree2)
	{
		motor2.en = 1;
		motor2.dir = dir1;
		motor2.running = 1;
		motor2.PulsesHaven = 0;
		PulsesGiven = Degree2;
		motor2.Time_Cost_Act = 0;
		motor2.PulsesGiven = PulsesGiven * motor2.divnum;
		time_cost = Get_TimeCost_ReverDot_S(2);
		Find_BestTimeCost(1, time_cost, dir1, Degree1);
	}
	else
	{
		motor1.en = 1;
		motor1.dir = dir2;
		motor1.running = 1;
		motor1.PulsesHaven = 0;
		PulsesGiven = Degree1;
		motor1.Time_Cost_Act = 0;
		motor1.PulsesGiven = PulsesGiven * motor1.divnum;
		time_cost = Get_TimeCost_ReverDot_S(1);
		Find_BestTimeCost(2, time_cost, dir2, Degree2);
	}

	// Start_Motor_S(1, dir1, Degree1);
	// Start_Motor_S(2, dir2, Degree2);
}

/*���õ�������ٶȣ���ڲ������ٶȵȼ�*/
void SetSpeed(unsigned char MotorID, signed char speedindex)
{
	int currentindex, i;
	unsigned int destspeed;
	unsigned int stepstostop = 0;
	MOTOR_CONTROL_S *pmotor_s = NULL;
	switch (MotorID)
	{
	case 1:
		pmotor_s = &motor1;
		TIM2->CR1 &= ~(TIM_CR1_CEN); //ֹͣ���r��
		break;
	case 2:
		pmotor_s = &motor2;
		TIM4->CR1 &= ~(TIM_CR1_CEN);
		break;
	case 3:
		pmotor_s = &motor3;
		TIM8->CR1 &= ~(TIM_CR1_CEN);
		break;
	case 4:
		pmotor_s = &motor4;
		TIM3->CR1 &= ~(TIM_CR1_CEN);
		break;
	case 5:
		pmotor_s = &motor5;
		TIM1->CR1 &= ~(TIM_CR1_CEN);
		break;
	case 6:
		pmotor_s = &motor6;
		TIM5->CR1 &= ~(TIM_CR1_CEN);
		break;
	default:
		return;
	}
	if (pmotor_s != NULL)
	{
		// TIM_Cmd(pmotor_s->TIMx, DISABLE);
		if (speedindex >= 0 && speedindex <= STEP_AA + STEP_UA + STEP_RA)
		{
			//ֱ������һ�ٶ�
			currentindex = pmotor_s->CurrentIndex;
			pmotor_s->PulsesHaven = 0;
			if (pmotor_s->CurrentIndex >= pmotor_s->StartTableLength)
			{
				currentindex = pmotor_s->StartTableLength + pmotor_s->StopTableLength - pmotor_s->CurrentIndex - 1;
			}
			if (currentindex > speedindex)
			{
				//��Ҫ����
				pmotor_s->PulsesGiven = pmotor_s->PulsesHaven + pmotor_s->StopSteps - 2;
			}
			else
			{
				//��Ҫ����
				pmotor_s->PulsesGiven = 0xffffffff;
			}
			pmotor_s->CurrentIndex = currentindex;
			pmotor_s->pulsecount = pmotor_s->Step_Table[pmotor_s->CurrentIndex];
			pmotor_s->TargetIndex = speedindex;
			pmotor_s->speedenbale = 1;
			pmotor_s->running = 1;
			pmotor_s->en = 1;
		}
		else
		{
			//ֹͣ���,pmotor->CurrentIndex=currentindex-1;ֱ������һ����
			pmotor_s->speedenbale = 0;
			currentindex = pmotor_s->CurrentIndex;
			if (pmotor_s->CurrentIndex >= pmotor_s->StartTableLength)
			{
				currentindex = pmotor_s->StartTableLength + pmotor_s->StopTableLength - pmotor_s->CurrentIndex - 1;
			}
			if (1) //currentindex>=1)
			{
				for (i = 0; i < currentindex; i++)
				{
					stepstostop += pmotor_s->Step_Table[i];
				}
				/*�������index*/
				if (pmotor_s->CurrentIndex < pmotor_s->StartTableLength)
				{
					currentindex = pmotor_s->StartTableLength + pmotor_s->StopTableLength - pmotor_s->CurrentIndex - 1;
				}
				pmotor_s->CurrentIndex = currentindex;
				pmotor_s->pulsecount = pmotor_s->Step_Table[pmotor_s->CurrentIndex];
				pmotor_s->PulsesHaven = 0;
				pmotor_s->PulsesGiven = pmotor_s->PulsesHaven + stepstostop;
			}
			else
			{
				pmotor_s->PulsesGiven = pmotor_s->PulsesHaven;
			}
			if (stepstostop == 0)
			{
				//�Ѿ�ֹͣ
				return;
			}
		}
		// TIM_Cmd(pmotor_s->TIMx, ENABLE);
		switch (MotorID)
		{
		case 1:
			TIM2->CR1 |= (TIM_CR1_CEN);
			break;
		case 2:
			TIM4->CR1 |= (TIM_CR1_CEN);
			break;
		case 3:
			TIM8->CR1 |= (TIM_CR1_CEN);
			break;
		case 4:
			TIM3->CR1 |= (TIM_CR1_CEN);
			break;
		case 5:
			TIM1->CR1 |= (TIM_CR1_CEN);
			break;
		case 6:
			TIM5->CR1 |= (TIM_CR1_CEN);
			break;
		default:
			return;
		}
	}
}

/*���õ����λ�ã�������е�ָ����λ��*/
void SetPosition(unsigned char MotorID, unsigned int dest, uint32_t MaxSpeed_SPTA, uint32_t AccSpeed_SPTA)
{
	MOTOR_CONTROL_S *pmotor_s = NULL;
	switch (MotorID)
	{
	case 1:
		pmotor_s = &motor1;
		break;
	case 2:
		pmotor_s = &motor2;
		break;
	case 3:
		pmotor_s = &motor3;
		break;
	case 4:
		pmotor_s = &motor4;
		break;
	case 5:
		pmotor_s = &motor5;
		break;
	case 6:
		pmotor_s = &motor6;
		break;
	default:
		return;
	}
	if (pmotor_s != NULL)
	{
		if (dest <= pmotor_s->MaxPosition && dest != pmotor_s->CurrentPosition)
		{
			if (dest > pmotor_s->CurrentPosition)
			{
				// Start_Motor_S(MotorID, pmotor_s->clockwise, dest - pmotor_s->CurrentPosition);
			}
			else
			{
				// Start_Motor_S(MotorID, !pmotor_s->clockwise, pmotor_s->CurrentPosition - dest);
			}
			while (pmotor_s->running == 1)
				;
		}
	}
}

/*��λ���*/
void Do_Reset(unsigned char MotorID)
{
	MOTOR_CONTROL_S *pmotor_s = NULL;
	switch (MotorID)
	{
	case 1:
		pmotor_s = &motor1;
		break;
	case 2:
		pmotor_s = &motor2;
		break;
	case 3:
		pmotor_s = &motor3;
		break;
	case 4:
		pmotor_s = &motor4;
		break;
	case 5:
		pmotor_s = &motor5;
		break;
	case 6:
		pmotor_s = &motor6;
		break;
	default:
		return;
	}
	/*do reset*/
	if (pmotor_s != NULL)
	{
		pmotor_s->rstflg = 1;
		pmotor_s->running = 1;
		SetSpeed(MotorID, 16);
		while (pmotor_s->running == 1)
			;
	}
}

// void Deal_Serail_Cmd(char *buf, int len)
// {
// 	unsigned int dest = 0;
// 	/*
// 			0xAA 0x55(��ʼ��־��
// 			Data0(����ţ�1,2,3,4)
// 			Data1(0,��λ��1�������ٶȣ�2��ֹͣ�����3 ���õ�����е�ָ��λ��?
// 			Data2����data1=1ʱ�򣬸��ֽڱ�ʾ�ٶȵȼ�����data1=3ʱ�����ֽڱ�ʾλ�õĸ�8λ
// 			Data3����data1=3ʱ�����ֽڱ�ʾλ�õ���8λ
// 			Data4����data1=3ʱ�����ֽڱ�ʾλ�õĵ�8λ
// 			*/B
// 	if (buf[0] == 0xAA)
// 	{
// 		if (buf[3] == 0)
// 		{
// 			Do_Reset(buf[2]);
// 		}
// 		else if (buf[3] == 1)
// 		{
// 			SetSpeed(buf[2], buf[4]);
// 		}
// 		else if (buf[3] == 2)
// 		{
// 			SetSpeed(buf[2], -1);
// 		}
// 		else if (buf[3] == 3)
// 		{
// 			dest = (buf[4] << 16) + (buf[5] << 8) + (buf[6]);
// 			SetPosition(buf[2], dest);
// 		}
// 	}
// }

/*
void Deal_Cmd(void)
{  
	unsigned char sync[2]; 
	while(rt_ringbuffer_data_len(&rb_recv)>=7)
	{ 
		if(rt_ringbuffer_peek(&rb_recv,sync,2)==2)
		{
			if(sync[0]==0xaa&&sync[1]==0x55)
			{
				if(rt_ringbuffer_get(&rb_recv,MY_Cmd_Buff,7)==7)
				{
				//	Deal_Serail_Cmd(MY_Cmd_Buff,7); 
				}	
			}
			else if(sync[1]==0xaa)
			{
				rt_ringbuffer_getchar(&rb_recv,sync);
			}
			else
			{
				rt_ringbuffer_getchar(&rb_recv,sync);
				rt_ringbuffer_getchar(&rb_recv,sync);
			}
		}
	} 
}
  */
/*���1��PWM�����ʼ��*/
void Initial_PWM_Motor1(void)
{
	TIM_OC_InitTypeDef sConfigOC; // ��ʱ��ͨ���Ƚ����

	MOTOR1_TIM2_RCC_CLK_ENABLE();

	/* STEPMOTOR���GPIO��ʼ������ */
	MOTOR1_GPIO_Init();

	/* ��ʱ�������������� */
	htim2_MOTOR1.Instance = MOTOR1_TIM2;					  // ��ʱ�����
	htim2_MOTOR1.Init.Prescaler = MOTOR1_TIM_PRESCALER;		  // ��ʱ��Ԥ��Ƶ��
	htim2_MOTOR1.Init.CounterMode = TIM_COUNTERMODE_UP;		  // �����������ϼ���
	htim2_MOTOR1.Init.Period = 1000;						  // ��ʱ������MOTOR1_TIM_PERIOD
	htim2_MOTOR1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ
	HAL_TIM_Base_Init(&htim2_MOTOR1);

	/* ��ʱ���Ƚ�������� */
#ifdef TIM_Idle_HIGH
	//������ʱ����ߵ�ƽ
	sConfigOC.OCMode = TIM_OCMODE_PWM1;			 // �Ƚ����ģʽ��PWM2���
	sConfigOC.Pulse = 0;						 // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;	 // �������
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW; // ����ͨ���������
#else
	//������ʱ����͵�ƽ
	sConfigOC.OCMode = TIM_OCMODE_PWM2;			  // �Ƚ����ģʽ��PWM2���
	sConfigOC.Pulse = 500;						  // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;	  // �������
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; // ����ͨ���������
#endif
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;		 // ����ģʽ
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;	 // ���е�ƽTIM_OCIDLESTATE_RESET
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // ����ͨ�����е�ƽ
	HAL_TIM_PWM_ConfigChannel(&htim2_MOTOR1, &sConfigOC, MOTOR1_TIM2_CHANNEL_x);
	/* ʹ�ܱȽ����ͨ�� */
	TIM_CCxChannelCmd(MOTOR1_TIM2, MOTOR1_TIM2_CHANNEL_x, TIM_CCx_ENABLE);

	/* ���ö�ʱ���ж����ȼ���ʹ�� */
	HAL_NVIC_SetPriority(MOTOR1_TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR1_TIM2_IRQn);

	//���жϣ�����һ�����жϺ����������ж�
	// TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	// __HAL_TIM_CLEAR_FLAG(&htim2_MOTOR1, MOTOR1_TIM2_FLAG_CCx);
	__HAL_TIM_CLEAR_FLAG(&htim2_MOTOR1, TIM_IT_UPDATE);

	//ʹ��TIM1�ж�Դ
	// TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	// __HAL_TIM_ENABLE_IT(&htim2_MOTOR1, MOTOR1_TIM2_IT_CCx);
	__HAL_TIM_ENABLE_IT(&htim2_MOTOR1, TIM_IT_UPDATE);

	/* Enable the main output */
	// TIM_Cmd(TIM1, DISABLE);
	// TIM_CtrlPWMOutputs(TIM1, ENABLE); //ʹ��PWM���
	HAL_TIM_Base_Stop(&htim2_MOTOR1); // ʹ�ܶ�ʱ��
	__HAL_TIM_MOE_ENABLE(&htim2_MOTOR1);
}

void Initial_PWM_Motor2(void)
{
	TIM_OC_InitTypeDef sConfigOC; // ��ʱ��ͨ���Ƚ����

	MOTOR2_TIM4_RCC_CLK_ENABLE();

	/* STEPMOTOR���GPIO��ʼ������ */
	MOTOR2_GPIO_Init();

	/* ��ʱ�������������� */
	htim4_MOTOR2.Instance = MOTOR2_TIM4;					  // ��ʱ�����
	htim4_MOTOR2.Init.Prescaler = MOTOR2_TIM_PRESCALER;		  // ��ʱ��Ԥ��Ƶ��
	htim4_MOTOR2.Init.CounterMode = TIM_COUNTERMODE_UP;		  // �����������ϼ���
	htim4_MOTOR2.Init.Period = 1000;						  // ��ʱ������MOTOR2_TIM_PERIOD
	htim4_MOTOR2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ
	HAL_TIM_Base_Init(&htim4_MOTOR2);

	/* ��ʱ���Ƚ�������� */
#ifdef TIM_Idle_HIGH
	//������ʱ����ߵ�ƽ
	sConfigOC.OCMode = TIM_OCMODE_PWM1;			 // �Ƚ����ģʽ��PWM2���
	sConfigOC.Pulse = 0;						 // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;	 // �������
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW; // ����ͨ���������
#else
	//������ʱ����͵�ƽ
	sConfigOC.OCMode = TIM_OCMODE_PWM2;			  // �Ƚ����ģʽ��PWM2���
	sConfigOC.Pulse = 500;						  // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;	  // �������
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; // ����ͨ���������
#endif
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;		 // ����ģʽ
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;	 // ���е�ƽTIM_OCIDLESTATE_RESET
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // ����ͨ�����е�ƽ
	HAL_TIM_PWM_ConfigChannel(&htim4_MOTOR2, &sConfigOC, MOTOR2_TIM4_CHANNEL_x);
	/* ʹ�ܱȽ����ͨ�� */
	TIM_CCxChannelCmd(MOTOR2_TIM4, MOTOR2_TIM4_CHANNEL_x, TIM_CCx_ENABLE);

	/* ���ö�ʱ���ж����ȼ���ʹ�� */
	HAL_NVIC_SetPriority(MOTOR2_TIM4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR2_TIM4_IRQn);

	//���жϣ�����һ�����жϺ����������ж�
	// __HAL_TIM_CLEAR_FLAG(&htim4_MOTOR2, MOTOR2_TIM4_FLAG_CCx);
	__HAL_TIM_CLEAR_FLAG(&htim4_MOTOR2, TIM_IT_UPDATE);

	//ʹ��TIM1�ж�Դ
	// TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	// __HAL_TIM_ENABLE_IT(&htim4_MOTOR2, MOTOR2_TIM4_IT_CCx);
	__HAL_TIM_ENABLE_IT(&htim4_MOTOR2, TIM_IT_UPDATE);

	/* Enable the main output */
	// TIM_Cmd(TIM1, DISABLE);
	// TIM_CtrlPWMOutputs(TIM1, ENABLE); //ʹ��PWM���
	HAL_TIM_Base_Stop(&htim4_MOTOR2); // ʹ�ܶ�ʱ��
	__HAL_TIM_MOE_ENABLE(&htim4_MOTOR2);
}
//
void Initial_PWM_Motor3(void)
{
	TIM_OC_InitTypeDef sConfigOC; // ��ʱ��ͨ���Ƚ����

	MOTOR3_TIM8_RCC_CLK_ENABLE();

	/* STEPMOTOR���GPIO��ʼ������ */
	MOTOR3_GPIO_Init();

	/* ��ʱ�������������� */
	htim8_MOTOR3.Instance = MOTOR3_TIM8;					  // ��ʱ�����
	htim8_MOTOR3.Init.Prescaler = MOTOR3_TIM_PRESCALER;		  // ��ʱ��Ԥ��Ƶ��
	htim8_MOTOR3.Init.CounterMode = TIM_COUNTERMODE_UP;		  // �����������ϼ���
	htim8_MOTOR3.Init.Period = 1000;						  // ��ʱ������MOTOR3_TIM_PERIOD
	htim8_MOTOR3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ
	HAL_TIM_Base_Init(&htim8_MOTOR3);

	/* ��ʱ���Ƚ�������� */
#ifdef TIM_Idle_HIGH
	//������ʱ����ߵ�ƽ
	sConfigOC.OCMode = TIM_OCMODE_PWM1;			 // �Ƚ����ģʽ��PWM2���
	sConfigOC.Pulse = 0;						 // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;	 // �������
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW; // ����ͨ���������
#else
	//������ʱ����͵�ƽ
	sConfigOC.OCMode = TIM_OCMODE_PWM2;			  // �Ƚ����ģʽ��PWM2���
	sConfigOC.Pulse = 500;						  // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;	  // �������
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; // ����ͨ���������
#endif
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;		 // ����ģʽ
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;	 // ���е�ƽ
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // ����ͨ�����е�ƽ
	HAL_TIM_PWM_ConfigChannel(&htim8_MOTOR3, &sConfigOC, MOTOR3_TIM8_CHANNEL_x);
	/* ʹ�ܱȽ����ͨ�� */
	TIM_CCxChannelCmd(MOTOR3_TIM8, MOTOR3_TIM8_CHANNEL_x, TIM_CCx_ENABLE);

	/* ���ö�ʱ���ж����ȼ���ʹ�� */
	HAL_NVIC_SetPriority(MOTOR3_TIM8_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR3_TIM8_IRQn);

	//���жϣ�����һ�����жϺ����������ж�
	// TIM_ClearFlag(TIM8, TIM_FLAG_Update);
	// __HAL_TIM_CLEAR_FLAG(&htim8_MOTOR3, MOTOR3_TIM8_FLAG_CCx);
	__HAL_TIM_CLEAR_FLAG(&htim8_MOTOR3, TIM_IT_UPDATE);

	//ʹ��TIM8�ж�Դ
	// TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
	// __HAL_TIM_ENABLE_IT(&htim8_MOTOR3, MOTOR3_TIM8_IT_CCx);
	__HAL_TIM_ENABLE_IT(&htim8_MOTOR3, TIM_IT_UPDATE);

	/* Enable the main output */
	// TIM_Cmd(TIM8, DISABLE);
	// TIM_CtrlPWMOutputs(TIM8, ENABLE); //ʹ��PWM���
	HAL_TIM_Base_Stop(&htim8_MOTOR3); // ʹ�ܶ�ʱ��
	__HAL_TIM_MOE_ENABLE(&htim8_MOTOR3);
}
//
void Initial_PWM_Motor4(void)
{
	TIM_OC_InitTypeDef sConfigOC; // ��ʱ��ͨ���Ƚ����

	MOTOR4_TIM3_RCC_CLK_ENABLE();

	/* STEPMOTOR���GPIO��ʼ������ */
	MOTOR4_GPIO_Init();

	/* ��ʱ�������������� */
	htim3_MOTOR4.Instance = MOTOR4_TIM3;					  // ��ʱ�����
	htim3_MOTOR4.Init.Prescaler = MOTOR4_TIM_PRESCALER;		  // ��ʱ��Ԥ��Ƶ��
	htim3_MOTOR4.Init.CounterMode = TIM_COUNTERMODE_UP;		  // �����������ϼ���
	htim3_MOTOR4.Init.Period = 1000;						  // ��ʱ������MOTOR4_TIM_PERIOD
	htim3_MOTOR4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ
	HAL_TIM_Base_Init(&htim3_MOTOR4);

	/* ��ʱ���Ƚ�������� */
#ifdef TIM_Idle_HIGH
	//������ʱ����ߵ�ƽ
	sConfigOC.OCMode = TIM_OCMODE_PWM1;			 // �Ƚ����ģʽ��PWM2���
	sConfigOC.Pulse = 0;						 // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;	 // �������
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW; // ����ͨ���������
#else
	//������ʱ����͵�ƽ
	sConfigOC.OCMode = TIM_OCMODE_PWM2;			  // �Ƚ����ģʽ��PWM2���
	sConfigOC.Pulse = 500;						  // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;	  // �������
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; // ����ͨ���������
#endif
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;		 // ����ģʽ
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;	 // ���е�ƽ
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // ����ͨ�����е�ƽ
	HAL_TIM_PWM_ConfigChannel(&htim3_MOTOR4, &sConfigOC, MOTOR4_TIM3_CHANNEL_x);
	/* ʹ�ܱȽ����ͨ�� */
	TIM_CCxChannelCmd(MOTOR4_TIM3, MOTOR4_TIM3_CHANNEL_x, TIM_CCx_ENABLE);

	/* ���ö�ʱ���ж����ȼ���ʹ�� */
	HAL_NVIC_SetPriority(MOTOR4_TIM3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR4_TIM3_IRQn);

	//���жϣ�����һ�����жϺ����������ж�
	// TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	__HAL_TIM_CLEAR_FLAG(&htim3_MOTOR4, MOTOR4_TIM3_FLAG_CCx);
	__HAL_TIM_CLEAR_FLAG(&htim3_MOTOR4, TIM_IT_UPDATE);

	//ʹ��TIM3�ж�Դ
	// TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	// __HAL_TIM_ENABLE_IT(&htim3_MOTOR4, MOTOR4_TIM3_IT_CCx);
	__HAL_TIM_ENABLE_IT(&htim3_MOTOR4, TIM_IT_UPDATE);

	/* Enable the main output */
	// TIM_Cmd(TIM3, DISABLE);
	// TIM_CtrlPWMOutputs(TIM3, ENABLE); //ʹ��PWM���
	HAL_TIM_Base_Stop(&htim3_MOTOR4); // ʹ�ܶ�ʱ��
	__HAL_TIM_MOE_ENABLE(&htim3_MOTOR4);
}

void Initial_PWM_Motor5(void)
{
	TIM_OC_InitTypeDef sConfigOC; // ��ʱ��ͨ���Ƚ����

	MOTOR5_TIM1_RCC_CLK_ENABLE();

	/* STEPMOTOR���GPIO��ʼ������ */
	MOTOR5_GPIO_Init();

	/* ��ʱ�������������� */
	htim1_MOTOR5.Instance = MOTOR5_TIM1;					  // ��ʱ�����
	htim1_MOTOR5.Init.Prescaler = MOTOR5_TIM_PRESCALER;		  // ��ʱ��Ԥ��Ƶ��
	htim1_MOTOR5.Init.CounterMode = TIM_COUNTERMODE_UP;		  // �����������ϼ���
	htim1_MOTOR5.Init.Period = 1000;						  // ��ʱ������MOTOR5_TIM_PERIOD
	htim1_MOTOR5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ
	HAL_TIM_Base_Init(&htim1_MOTOR5);

	/* ��ʱ���Ƚ�������� */
#ifdef TIM_Idle_HIGH
	//������ʱ����ߵ�ƽ
	sConfigOC.OCMode = TIM_OCMODE_PWM1;			 // �Ƚ����ģʽ��PWM2���
	sConfigOC.Pulse = 0;						 // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;	 // �������
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW; // ����ͨ���������
#else
	//������ʱ����͵�ƽ
	sConfigOC.OCMode = TIM_OCMODE_PWM2;			  // �Ƚ����ģʽ��PWM2���
	sConfigOC.Pulse = 500;						  // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;	  // �������
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; // ����ͨ���������
#endif
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;		 // ����ģʽ
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;	 // ���е�ƽ
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // ����ͨ�����е�ƽ
	HAL_TIM_PWM_ConfigChannel(&htim1_MOTOR5, &sConfigOC, MOTOR5_TIM1_CHANNEL_x);
	/* ʹ�ܱȽ����ͨ�� */
	TIM_CCxChannelCmd(MOTOR5_TIM1, MOTOR5_TIM1_CHANNEL_x, TIM_CCx_ENABLE);

	/* ���ö�ʱ���ж����ȼ���ʹ�� */
	HAL_NVIC_SetPriority(MOTOR5_TIM1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR5_TIM1_IRQn);

	//���жϣ�����һ�����жϺ����������ж�
	// TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	// __HAL_TIM_CLEAR_FLAG(&htim1_MOTOR5, MOTOR5_TIM1_FLAG_CCx);
	__HAL_TIM_CLEAR_FLAG(&htim1_MOTOR5, TIM_IT_UPDATE);

	//ʹ��TIM1�ж�Դ
	// TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	// __HAL_TIM_ENABLE_IT(&htim1_MOTOR5, MOTOR5_TIM1_IT_CCx);
	__HAL_TIM_ENABLE_IT(&htim1_MOTOR5, TIM_IT_UPDATE);

	/* Enable the main output */
	// TIM_Cmd(TIM1, DISABLE);
	// TIM_CtrlPWMOutputs(TIM1, ENABLE); //ʹ��PWM���
	HAL_TIM_Base_Stop(&htim1_MOTOR5); // ʹ�ܶ�ʱ��
	__HAL_TIM_MOE_ENABLE(&htim1_MOTOR5);
}

void Initial_PWM_Motor6(void)
{
	TIM_OC_InitTypeDef sConfigOC; // ��ʱ��ͨ���Ƚ����

	MOTOR6_TIM5_RCC_CLK_ENABLE();

	/* STEPMOTOR���GPIO��ʼ������ */
	MOTOR6_GPIO_Init();

	/* ��ʱ�������������� */
	htim5_MOTOR6.Instance = MOTOR6_TIM5;					  // ��ʱ�����
	htim5_MOTOR6.Init.Prescaler = MOTOR6_TIM_PRESCALER;		  // ��ʱ��Ԥ��Ƶ��
	htim5_MOTOR6.Init.CounterMode = TIM_COUNTERMODE_UP;		  // �����������ϼ���
	htim5_MOTOR6.Init.Period = 1000;						  // ��ʱ������MOTOR6_TIM_PERIOD
	htim5_MOTOR6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ
	HAL_TIM_Base_Init(&htim5_MOTOR6);

	/* ��ʱ���Ƚ�������� */
#ifdef TIM_Idle_HIGH
	//������ʱ����ߵ�ƽ
	sConfigOC.OCMode = TIM_OCMODE_PWM1;			 // �Ƚ����ģʽ��PWM2���
	sConfigOC.Pulse = 0;						 // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;	 // �������
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW; // ����ͨ���������
#else
	//������ʱ����͵�ƽ
	sConfigOC.OCMode = TIM_OCMODE_PWM2;			  // �Ƚ����ģʽ��PWM2���
	sConfigOC.Pulse = 500;						  // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;	  // �������
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; // ����ͨ���������
#endif
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;		 // ����ģʽ
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;	 // ���е�ƽ
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // ����ͨ�����е�ƽ
	HAL_TIM_PWM_ConfigChannel(&htim5_MOTOR6, &sConfigOC, MOTOR6_TIM5_CHANNEL_x);
	/* ʹ�ܱȽ����ͨ�� */
	TIM_CCxChannelCmd(MOTOR6_TIM5, MOTOR6_TIM5_CHANNEL_x, TIM_CCx_ENABLE);

	/* ���ö�ʱ���ж����ȼ���ʹ�� */
	HAL_NVIC_SetPriority(MOTOR6_TIM5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR6_TIM5_IRQn);

	//���жϣ�����һ�����жϺ����������ж�
	// TIM_ClearFlag(TIM5, TIM_FLAG_Update);
	// __HAL_TIM_CLEAR_FLAG(&htim5_MOTOR6, MOTOR6_TIM5_FLAG_CCx);
	__HAL_TIM_CLEAR_FLAG(&htim5_MOTOR6, TIM_IT_UPDATE);

	//ʹ��TIM5�ж�Դ
	// TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	// __HAL_TIM_ENABLE_IT(&htim5_MOTOR6, MOTOR6_TIM5_IT_CCx);
	__HAL_TIM_ENABLE_IT(&htim5_MOTOR6, TIM_IT_UPDATE);

	/* Enable the main output */
	// TIM_Cmd(TIM5, DISABLE);
	// TIM_CtrlPWMOutputs(TIM5, ENABLE); //ʹ��PWM���
	HAL_TIM_Base_Stop(&htim5_MOTOR6); // ʹ�ܶ�ʱ��
	__HAL_TIM_MOE_ENABLE(&htim5_MOTOR6);
}

//�����������
void Run_Motor_S(unsigned char MotorID, unsigned char dir, uint32_t Degree, uint32_t MaxSpeed_S, uint32_t AccSpeed_Para)
{
	switch (MotorID)
	{
	case 1:
		Pw_Motor1_FRE_AA = (MaxSpeed_S * Pw_Motor1_PULSENUM / 60 - Pw_Motor1_FRE_START) / (Motor1_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor1_XiShu_1 * M_T_AA * M_T_RA);
		CalcMotorPeriStep_CPF(Pw_Motor1_FRE_START, Pw_Motor1_FRE_AA, AccSpeed_Para, M_T_AA, M_T_UA, M_T_RA, Motor1TimeTable, Motor1StepTable);
		Initial_Motor(1, M1DIV, MAX_POSITION);
		Start_Motor_S(1, M1_CLOCKWISE, Degree);
		break;
	case 2:
		Pw_Motor2_FRE_AA = (MaxSpeed_S * Pw_Motor2_PULSENUM / 60 - Pw_Motor2_FRE_START) / (Motor2_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor2_XiShu_1 * M_T_AA * M_T_RA);
		CalcMotorPeriStep_CPF(Pw_Motor2_FRE_START, Pw_Motor2_FRE_AA, AccSpeed_Para, M_T_AA, M_T_UA, M_T_RA, Motor2TimeTable, Motor2StepTable);
		Initial_Motor(2, M2DIV, MAX_POSITION);
		Start_Motor_S(2, M2_CLOCKWISE, Degree);
		break;
	case 3:
		Pw_Motor3_FRE_AA = (MaxSpeed_S * Pw_Motor3_PULSENUM / 60 - Pw_Motor3_FRE_START) / (Motor3_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor3_XiShu_1 * M_T_AA * M_T_RA);
		CalcMotorPeriStep_CPF(Pw_Motor3_FRE_START, Pw_Motor3_FRE_AA, AccSpeed_Para, M_T_AA, M_T_UA, M_T_RA, Motor3TimeTable, Motor3StepTable);
		Initial_Motor(3, M3DIV, MAX_POSITION);
		Start_Motor_S(3, M3_CLOCKWISE, Degree);
		break;
	case 4:
		Pw_Motor4_FRE_AA = (MaxSpeed_S * Pw_Motor4_PULSENUM / 60 - Pw_Motor4_FRE_START) / (Motor4_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor4_XiShu_1 * M_T_AA * M_T_RA);
		CalcMotorPeriStep_CPF(Pw_Motor4_FRE_START, Pw_Motor4_FRE_AA, AccSpeed_Para, M_T_AA, M_T_UA, M_T_RA, Motor4TimeTable, Motor4StepTable);
		Initial_Motor(4, M4DIV, MAX_POSITION);
		Start_Motor_S(4, M4_CLOCKWISE, Degree);
		break;
	case 5:
		Pw_Motor5_FRE_AA = (MaxSpeed_S * Pw_Motor5_PULSENUM / 60 - Pw_Motor5_FRE_START) / (Motor5_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor5_XiShu_1 * M_T_AA * M_T_RA);
		CalcMotorPeriStep_CPF(Pw_Motor5_FRE_START, Pw_Motor5_FRE_AA, AccSpeed_Para, M_T_AA, M_T_UA, M_T_RA, Motor5TimeTable, Motor5StepTable);
		Initial_Motor(5, M5DIV, MAX_POSITION);
		Start_Motor_S(5, M5_CLOCKWISE, Degree);
		break;
	case 6:
		Pw_Motor6_FRE_AA = (MaxSpeed_S * Pw_Motor6_PULSENUM / 60 - Pw_Motor6_FRE_START) / (Motor6_XiShu_1 * M_T_AA * M_T_AA + M_T_AA * M_T_UA + Motor6_XiShu_1 * M_T_AA * M_T_RA);
		CalcMotorPeriStep_CPF(Pw_Motor6_FRE_START, Pw_Motor6_FRE_AA, AccSpeed_Para, M_T_AA, M_T_UA, M_T_RA, Motor6TimeTable, Motor6StepTable);
		Initial_Motor(6, M6DIV, MAX_POSITION);
		Start_Motor_S(6, M6_CLOCKWISE, Degree);
		break;
	default:
		return;
	}
}
