/**
  ******************************************************************************
  * �ļ�����: bsp_MOTOR6.c
  * ��    ��: QINGDAO SANLI
  * ��    ��: V1.0
  * ��д����: 2021-04-19
  * ��    ��: �������������ʵ��
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "bsp_MOTOR6.h"
#include "bsp_MOTOR3.h"
#include <math.h>
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/

TIM_HandleTypeDef htim5_MOTOR6;
speedRampData MOTOR6_srd = {STOP, CW, 0, 0, 0, 0, 0}; // �Ӽ������߱���
__IO int32_t MOTOR6_step_position = 0;                // ��ǰλ��
__IO uint8_t MOTOR6_MotionStatus = 0;                 //�Ƿ����˶���0��ֹͣ��1���˶�

__IO uint8_t Motor6_status = 1;
__IO int Motor6_num = 0;

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim5_MOTOR6;
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
void MOTOR6_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* ���Ŷ˿�ʱ��ʹ�� */
  MOTOR6_TIM5_GPIO_CLK_ENABLE();
  MOTOR6_DIR_GPIO_CLK_ENABLE();
  MOTOR6_ENA_GPIO_CLK_ENABLE();

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR6_TIM5_PUL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5; // GPIO��������TIM���ù���
  HAL_GPIO_Init(MOTOR6_TIM5_PUL_PORT, &GPIO_InitStruct);

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR6_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR6_DIR_PORT, &GPIO_InitStruct);

  /* �������ѻ�ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR6_ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR6_ENA_PORT, &GPIO_InitStruct);

  MOTOR6_DIR_FORWARD();
  MOTOR6_OUTPUT_DISABLE();
}

/**
  * ��������: ��������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void MOTOR6_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig; // ��ʱ��ʱ��
  TIM_OC_InitTypeDef sConfigOC;              // ��ʱ��ͨ���Ƚ����

  MOTOR6_TIM5_RCC_CLK_ENABLE();

  /* STEPMOTOR���GPIO��ʼ������ */
  MOTOR6_GPIO_Init();
  //
  /* ��ʱ�������������� */
  htim5_MOTOR6.Instance = MOTOR6_TIM5;                      // ��ʱ�����
  htim5_MOTOR6.Init.Prescaler = MOTOR6_TIM_PRESCALER;       // ��ʱ��Ԥ��Ƶ��
  htim5_MOTOR6.Init.CounterMode = TIM_COUNTERMODE_UP;       // �����������ϼ���
  htim5_MOTOR6.Init.Period = MOTOR6_TIM_PERIOD;             // ��ʱ������
  htim5_MOTOR6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ
  HAL_TIM_Base_Init(&htim5_MOTOR6);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htim5_MOTOR6, &sClockSourceConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;            // �Ƚ����ģʽ����ת���
  sConfigOC.Pulse = 0xFFFF;                        // ������
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;       // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;     // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;       // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;   // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // ����ͨ�����е�ƽ
  HAL_TIM_OC_ConfigChannel(&htim5_MOTOR6, &sConfigOC, MOTOR6_TIM5_CHANNEL_x);
  /* ʹ�ܱȽ����ͨ�� */
  TIM_CCxChannelCmd(MOTOR6_TIM5, MOTOR6_TIM5_CHANNEL_x, TIM_CCx_DISABLE);

  /* ���ö�ʱ���ж����ȼ���ʹ�� */
  HAL_NVIC_SetPriority(MOTOR6_TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MOTOR6_TIM5_IRQn);

  __HAL_TIM_CLEAR_FLAG(&htim5_MOTOR6, MOTOR6_TIM5_FLAG_CCx);
  /* ʹ�ܶ�ʱ���Ƚ���� */
  __HAL_TIM_ENABLE_IT(&htim5_MOTOR6, MOTOR6_TIM5_IT_CCx);
  /* Enable the main output */
  __HAL_TIM_MOE_ENABLE(&htim5_MOTOR6);
  //  HAL_TIM_Base_Start(&htim5_MOTOR6);// ʹ�ܶ�ʱ��
}

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
