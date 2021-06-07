/**
  ******************************************************************************
  * �ļ�����: bsp_PMOTOR2.c
  * ��    ��: QINGDAO SANLI
  * ��    ��: V1.0
  * ��д����: 2021-04-19
  * ��    ��: �������������ʵ��
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "bsp_MOTOR2.h"
#include "bsp_MOTOR3.h"
#include <math.h>
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/

TIM_HandleTypeDef htim4_MOTOR2;
speedRampData MOTOR2_srd = {STOP, CW, 0, 0, 0, 0, 0}; // �Ӽ������߱���
__IO int32_t MOTOR2_step_position = 0;                // ��ǰλ��
__IO uint8_t MOTOR2_MotionStatus = 0;                 //�Ƿ����˶���0��ֹͣ��1���˶�

__IO uint8_t Motor2_status = 1;
__IO int Motor2_num = 0;

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim4_MOTOR2;
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
void MOTOR2_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* ���Ŷ˿�ʱ��ʹ�� */
  MOTOR2_TIM4_GPIO_CLK_ENABLE();
  MOTOR2_DIR_GPIO_CLK_ENABLE();
  MOTOR2_ENA_GPIO_CLK_ENABLE();

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR2_TIM4_PUL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4; // GPIO��������TIM���ù���
  HAL_GPIO_Init(MOTOR2_TIM4_PUL_PORT, &GPIO_InitStruct);

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR2_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR2_DIR_PORT, &GPIO_InitStruct);

  /* �������ѻ�ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR2_ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR2_ENA_PORT, &GPIO_InitStruct);

  MOTOR2_DIR_FORWARD();
  MOTOR2_OUTPUT_DISABLE();
}

/**
  * ��������: ��������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void MOTOR2_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig; // ��ʱ��ʱ��
  TIM_OC_InitTypeDef sConfigOC;              // ��ʱ��ͨ���Ƚ����

  MOTOR2_TIM4_RCC_CLK_ENABLE();

  /* STEPMOTOR���GPIO��ʼ������ */
  MOTOR2_GPIO_Init();
  //
  /* ��ʱ�������������� */
  htim4_MOTOR2.Instance = MOTOR2_TIM4;                      // ��ʱ�����
  htim4_MOTOR2.Init.Prescaler = MOTOR2_TIM_PRESCALER;       // ��ʱ��Ԥ��Ƶ��
  htim4_MOTOR2.Init.CounterMode = TIM_COUNTERMODE_UP;       // �����������ϼ���
  htim4_MOTOR2.Init.Period = MOTOR2_TIM_PERIOD;             // ��ʱ������
  htim4_MOTOR2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ
  HAL_TIM_Base_Init(&htim4_MOTOR2);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htim4_MOTOR2, &sClockSourceConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;            // �Ƚ����ģʽ����ת���
  sConfigOC.Pulse = 0xFFFF;                        // ������
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;       // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;     // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;       // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;   // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // ����ͨ�����е�ƽ
  HAL_TIM_OC_ConfigChannel(&htim4_MOTOR2, &sConfigOC, MOTOR2_TIM4_CHANNEL_x);
  /* ʹ�ܱȽ����ͨ�� */
  TIM_CCxChannelCmd(MOTOR2_TIM4, MOTOR2_TIM4_CHANNEL_x, TIM_CCx_DISABLE);

  /* ���ö�ʱ���ж����ȼ���ʹ�� */
  HAL_NVIC_SetPriority(MOTOR2_TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MOTOR2_TIM4_IRQn);

  __HAL_TIM_CLEAR_FLAG(&htim4_MOTOR2, MOTOR2_TIM4_FLAG_CCx);
  /* ʹ�ܶ�ʱ���Ƚ���� */
  __HAL_TIM_ENABLE_IT(&htim4_MOTOR2, MOTOR2_TIM4_IT_CCx);
  /* Enable the main output */
  __HAL_TIM_MOE_ENABLE(&htim4_MOTOR2);
  //  HAL_TIM_Base_Start(&htim4_MOTOR2);// ʹ�ܶ�ʱ��
}

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
