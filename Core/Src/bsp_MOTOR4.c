/**
  ******************************************************************************
  * �ļ�����: bsp_MOTOR4.c
  * ��    ��: QINGDAO SANLI
  * ��    ��: V1.0
  * ��д����: 2021-04-19
  * ��    ��: �������������ʵ��
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "bsp_MOTOR4.h"
#include "bsp_MOTOR3.h"
#include <math.h>
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/

TIM_HandleTypeDef htim3_MOTOR4;
speedRampData MOTOR4_srd = {STOP, CW, 0, 0, 0, 0, 0}; // �Ӽ������߱���
__IO int32_t MOTOR4_step_position = 0;                // ��ǰλ��
__IO uint8_t MOTOR4_MotionStatus = 0;                 //�Ƿ����˶���0��ֹͣ��1���˶�

__IO uint8_t Motor4_status = 1;
__IO int Motor4_num = 0;

/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/
//#define S_ACCEL 1
//#define T_ACCEL 0
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ���������GPIO��ʼ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void MOTOR4_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* ���Ŷ˿�ʱ��ʹ�� */
  MOTOR4_TIM3_GPIO_CLK_ENABLE();
  MOTOR4_DIR_GPIO_CLK_ENABLE();
  MOTOR4_ENA_GPIO_CLK_ENABLE();

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR4_TIM3_PUL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; //GPIO_MODE_AF_PP
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3; // GPIO��������TIM���ù���
  HAL_GPIO_Init(MOTOR4_TIM3_PUL_PORT, &GPIO_InitStruct);

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR4_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR4_DIR_PORT, &GPIO_InitStruct);

  /* �������ѻ�ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR4_ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR4_ENA_PORT, &GPIO_InitStruct);

  MOTOR4_DIR_FORWARD();
  MOTOR4_OUTPUT_DISABLE();
}

/**
  * ��������: ��������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void MOTOR4_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig; // ��ʱ��ʱ��
  TIM_OC_InitTypeDef sConfigOC;              // ��ʱ��ͨ���Ƚ����

  MOTOR4_TIM3_RCC_CLK_ENABLE();

  /* STEPMOTOR���GPIO��ʼ������ */
  MOTOR4_GPIO_Init();
  //
  /* ��ʱ�������������� */
  htim3_MOTOR4.Instance = MOTOR4_TIM3;                      // ��ʱ�����
  htim3_MOTOR4.Init.Prescaler = MOTOR4_TIM_PRESCALER;       // ��ʱ��Ԥ��Ƶ��
  htim3_MOTOR4.Init.CounterMode = TIM_COUNTERMODE_UP;       // �����������ϼ���
  htim3_MOTOR4.Init.Period = MOTOR4_TIM_PERIOD;             // ��ʱ������
  htim3_MOTOR4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ
  HAL_TIM_Base_Init(&htim3_MOTOR4);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htim3_MOTOR4, &sClockSourceConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;            // �Ƚ����ģʽ����ת���
  sConfigOC.Pulse = 0xFFFF;                        // ������
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;       // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;     // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;       // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;   // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // ����ͨ�����е�ƽ
  HAL_TIM_OC_ConfigChannel(&htim3_MOTOR4, &sConfigOC, MOTOR4_TIM3_CHANNEL_x);
  /* ʹ�ܱȽ����ͨ�� */
  TIM_CCxChannelCmd(MOTOR4_TIM3, MOTOR4_TIM3_CHANNEL_x, TIM_CCx_DISABLE);

  /* ���ö�ʱ���ж����ȼ���ʹ�� */
  HAL_NVIC_SetPriority(MOTOR4_TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MOTOR4_TIM3_IRQn);

  __HAL_TIM_CLEAR_FLAG(&htim3_MOTOR4, MOTOR4_TIM3_FLAG_CCx);
  /* ʹ�ܶ�ʱ���Ƚ���� */
  __HAL_TIM_ENABLE_IT(&htim3_MOTOR4, MOTOR4_TIM3_IT_CCx);
  /* Enable the main output */
  __HAL_TIM_MOE_ENABLE(&htim3_MOTOR4);
  //  HAL_TIM_Base_Start(&htim3_MOTOR4);// ʹ�ܶ�ʱ��
}

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
