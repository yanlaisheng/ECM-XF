/**
  ******************************************************************************
  * �ļ�����: bsp_STEPMOTOR.c
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-06-03
  * ��    ��: �����������������ʵ��
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "bsp_MOTOR1.h"
#include "bsp_MOTOR3.h"
#include "GlobalConst.h"
#include <math.h>
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/

TIM_HandleTypeDef htim2_MOTOR1;
speedRampData Motor1_srd = {STOP, CW, 0, 0, 0, 0, 0}; // �Ӽ������߱���
__IO int32_t Motor1_step_position = 0;                // ��ǰλ��
__IO uint8_t Motor1_MotionStatus = 0;                 //�Ƿ����˶���0��ֹͣ��1���˶�

__IO uint8_t Motor1_status = 1;
__IO int Motor1_num = 0;

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim2_MOTOR1;
/* ˽�к���ԭ�� --------------------------------------------------------------*/
//#define S_ACCEL 0
//#define T_ACCEL 1
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ���������GPIO��ʼ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void MOTOR1_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* ���Ŷ˿�ʱ��ʹ�� */
  MOTOR1_TIM2_GPIO_CLK_ENABLE();
  MOTOR1_DIR_GPIO_CLK_ENABLE();
  MOTOR1_ENA_GPIO_CLK_ENABLE();

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR1_TIM2_PUL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2; // GPIO��������TIM���ù���
  HAL_GPIO_Init(MOTOR1_TIM2_PUL_PORT, &GPIO_InitStruct);

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR1_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR1_DIR_PORT, &GPIO_InitStruct);

  /* �������ѻ�ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR1_ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR1_ENA_PORT, &GPIO_InitStruct);

  MOTOR1_DIR_FORWARD();
  MOTOR1_OUTPUT_DISABLE();
}

/**
  * ��������: ��������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void MOTOR1_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig; // ��ʱ��ʱ��
  TIM_OC_InitTypeDef sConfigOC;              // ��ʱ��ͨ���Ƚ����

  MOTOR1_TIM2_RCC_CLK_ENABLE();

  /* STEPMOTOR���GPIO��ʼ������ */
  MOTOR1_GPIO_Init();
  //
  /* ��ʱ�������������� */
  htim2_MOTOR1.Instance = MOTOR1_TIM2;                      // ��ʱ�����
  htim2_MOTOR1.Init.Prescaler = MOTOR1_TIM_PRESCALER;       // ��ʱ��Ԥ��Ƶ��
  htim2_MOTOR1.Init.CounterMode = TIM_COUNTERMODE_UP;       // �����������ϼ���
  htim2_MOTOR1.Init.Period = MOTOR1_TIM_PERIOD;             // ��ʱ������
  htim2_MOTOR1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ
  HAL_TIM_Base_Init(&htim2_MOTOR1);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htim2_MOTOR1, &sClockSourceConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;            // �Ƚ����ģʽ����ת���
  sConfigOC.Pulse = 0xFFFF;                        // ������
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;       // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;     // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;       // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;   // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // ����ͨ�����е�ƽ
  HAL_TIM_OC_ConfigChannel(&htim2_MOTOR1, &sConfigOC, MOTOR1_TIM2_CHANNEL_x);
  /* ʹ�ܱȽ����ͨ�� */
  TIM_CCxChannelCmd(MOTOR1_TIM2, MOTOR1_TIM2_CHANNEL_x, TIM_CCx_DISABLE);

  /* ���ö�ʱ���ж����ȼ���ʹ�� */
  HAL_NVIC_SetPriority(MOTOR1_TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MOTOR1_TIM2_IRQn);

  __HAL_TIM_CLEAR_FLAG(&htim2_MOTOR1, MOTOR1_TIM2_FLAG_CCx);
  /* ʹ�ܶ�ʱ���Ƚ���� */
  __HAL_TIM_ENABLE_IT(&htim2_MOTOR1, MOTOR1_TIM2_IT_CCx);
  /* Enable the main output */
  __HAL_TIM_MOE_ENABLE(&htim2_MOTOR1);
  //  HAL_TIM_Base_Start(&htim2_MOTOR1);// ʹ�ܶ�ʱ��
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
