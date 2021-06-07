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
#include "bsp_MOTOR3.h"
#include <math.h>
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htim8_MOTOR3;
speedRampData Motor3_srd = {STOP, CW, 0, 0, 0, 0, 0}; // �Ӽ������߱���
__IO int32_t Motor3_step_position = 0;                // ��ǰλ��
__IO uint8_t Motor3_MotionStatus = 0;                 //�Ƿ����˶���0��ֹͣ��1���˶�

__IO uint8_t Motor3_status = 1;
__IO int Motor3_num = 0;

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
void MOTOR3_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* ���Ŷ˿�ʱ��ʹ�� */
  MOTOR3_TIM8_GPIO_CLK_ENABLE();
  MOTOR3_DIR_GPIO_CLK_ENABLE();
  MOTOR3_ENA_GPIO_CLK_ENABLE();

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR3_TIM8_PUL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8; // GPIO��������TIM���ù���
  HAL_GPIO_Init(MOTOR3_TIM8_PUL_PORT, &GPIO_InitStruct);

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR3_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR3_DIR_PORT, &GPIO_InitStruct);

  /* �������ѻ�ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR3_ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR3_ENA_PORT, &GPIO_InitStruct);

  MOTOR3_DIR_FORWARD();
  MOTOR3_OUTPUT_DISABLE();
}

/**
  * ��������: ��������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void MOTOR3_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig; // ��ʱ��ʱ��
  TIM_OC_InitTypeDef sConfigOC;              // ��ʱ��ͨ���Ƚ����

  MOTOR3_TIM8_RCC_CLK_ENABLE();

  /* STEPMOTOR���GPIO��ʼ������ */
  MOTOR3_GPIO_Init();
  //
  /* ��ʱ�������������� */
  htim8_MOTOR3.Instance = MOTOR3_TIM8;                      // ��ʱ�����
  htim8_MOTOR3.Init.Prescaler = MOTOR3_TIM_PRESCALER;       // ��ʱ��Ԥ��Ƶ��
  htim8_MOTOR3.Init.CounterMode = TIM_COUNTERMODE_UP;       // �����������ϼ���
  htim8_MOTOR3.Init.Period = MOTOR3_TIM_PERIOD;             // ��ʱ������
  htim8_MOTOR3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ
  HAL_TIM_Base_Init(&htim8_MOTOR3);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htim8_MOTOR3, &sClockSourceConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;            // �Ƚ����ģʽ����ת���
  sConfigOC.Pulse = 0xFFFF;                        // ������
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;       // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;     // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;       // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;   // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // ����ͨ�����е�ƽ
  HAL_TIM_OC_ConfigChannel(&htim8_MOTOR3, &sConfigOC, MOTOR3_TIM8_CHANNEL_x);
  /* ʹ�ܱȽ����ͨ�� */
  TIM_CCxChannelCmd(MOTOR3_TIM8, MOTOR3_TIM8_CHANNEL_x, TIM_CCx_DISABLE);

  /* ���ö�ʱ���ж����ȼ���ʹ�� */
  HAL_NVIC_SetPriority(MOTOR3_TIM8_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MOTOR3_TIM8_IRQn);

  __HAL_TIM_CLEAR_FLAG(&htim8_MOTOR3, MOTOR3_TIM8_FLAG_CCx);
  /* ʹ�ܶ�ʱ���Ƚ���� */
  __HAL_TIM_ENABLE_IT(&htim8_MOTOR3, MOTOR3_TIM8_IT_CCx);
  /* Enable the main output */
  __HAL_TIM_MOE_ENABLE(&htim8_MOTOR3);
  //  HAL_TIM_Base_Start(&htim8_MOTOR3);// ʹ�ܶ�ʱ��
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
