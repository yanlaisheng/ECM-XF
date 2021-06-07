#ifndef __MOTOR5_TIM_H__
#define __MOTOR5_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
//
// #define S_ACCEL 1
// #define T_ACCEL 0

// #if S_ACCEL
// /* S�ͼ��ٲ��� */
// #define ACCELERATED_SPEED_LENGTH 200 //������ٶȵĵ�������ʵҲ��3000��ϸ�ֲ�����˼��������������ı���ٵ�
// #define FRE_MIN 500                  //��͵�����Ƶ�ʣ����������������������ٶ�
// #define FRE_MAX 35000                //��ߵ�����Ƶ�ʣ������������������ʱ������ٶ�35000

// #endif
/* �궨�� --------------------------------------------------------------------*/
#define MOTOR5_TIM1 TIM1
#define MOTOR5_TIM1_RCC_CLK_ENABLE() __HAL_RCC_TIM1_CLK_ENABLE()
#define MOTOR5_TIM1_RCC_CLK_DISABLE() __HAL_RCC_TIM1_CLK_DISABLE()
#define MOTOR5_TIM1_IT_CCx TIM_IT_CC1
#define MOTOR5_TIM1_FLAG_CCx TIM_FLAG_CC1
// #define MOTOR5_TIM1_IRQn TIM1_CC_IRQn
#define MOTOR5_TIM1_IRQn TIM1_UP_TIM10_IRQn
#define MOTOR5_TIM1_IRQHandler TIM1_CC_IRQHandler

#define MOTOR5_TIM1_CHANNEL_x TIM_CHANNEL_1
#define MOTOR5_TIM1_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE() // ���������������������
#define MOTOR5_TIM1_PUL_PORT GPIOE                                 // ��Ӧ��������PUL-��������ʹ�ù����ӷ���
#define MOTOR5_TIM1_PUL_PIN GPIO_PIN_9                             // ��PLU+ֱ�ӽӿ������VCC

#define MOTOR5_DIR_GPIO_CLK_ENABLE() __HAL_RCC_GPIOF_CLK_ENABLE() // �����ת������ƣ�������ղ���Ĭ����ת
#define MOTOR5_DIR_PORT GPIOF                                     // ��Ӧ��������DIR-��������ʹ�ù����ӷ���
#define MOTOR5_DIR_PIN GPIO_PIN_15                                // ��DIR+ֱ�ӽӿ������VCC
//#define GPIO_PIN_AF_AS_SYS GPIO_AF0_RTC_50Hz                      // ���Ų���Ϊ���ù���ʹ��

#define MOTOR5_ENA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOG_CLK_ENABLE() // ����ѻ�ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define MOTOR5_ENA_PORT GPIOG                                     // ��Ӧ��������ENA-��������ʹ�ù����ӷ���
#define MOTOR5_ENA_PIN GPIO_PIN_0                                 // ��ENA+ֱ�ӽӿ������VCC

#define MOTOR5_DIR_FORWARD() HAL_GPIO_WritePin(MOTOR5_DIR_PORT, MOTOR5_DIR_PIN, GPIO_PIN_RESET)
#define MOTOR5_DIR_REVERSAL() HAL_GPIO_WritePin(MOTOR5_DIR_PORT, MOTOR5_DIR_PIN, GPIO_PIN_SET)

#define MOTOR5_OUTPUT_ENABLE() HAL_GPIO_WritePin(MOTOR5_ENA_PORT, MOTOR5_ENA_PIN, GPIO_PIN_RESET)
#define MOTOR5_OUTPUT_DISABLE() HAL_GPIO_WritePin(MOTOR5_ENA_PORT, MOTOR5_ENA_PIN, GPIO_PIN_SET)

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��STEPMOTOR_TIMx_PRESCALER+1��
#define MOTOR5_TIM_PRESCALER 15 // �������������ϸ������Ϊ��   64  ϸ��

// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define MOTOR5_TIM_PERIOD 0xFFFF
// ����߼���ʱ���ظ������Ĵ���ֵ
#define MOTOR5_TIM_REPETITIONCOUNTER 0

#define FALSE 0
#define TRUE 1
#define CW 0  // ˳ʱ��
#define CCW 1 // ��ʱ��

#define STOP 0                                                        // �Ӽ�������״̬��ֹͣ
#define ACCEL 1                                                       // �Ӽ�������״̬�����ٽ׶�
#define DECEL 2                                                       // �Ӽ�������״̬�����ٽ׶�
#define RUN 3                                                         // �Ӽ�������״̬�����ٽ׶�
#define T1_FREQ_MOTOR5 (SystemCoreClock / (MOTOR5_TIM_PRESCALER + 1)) // Ƶ��ftֵ
#define FSPR 200                                                      //���������Ȧ����
#define MICRO_STEP 64                                                 // �������������ϸ����
#define SPR (FSPR * MICRO_STEP)                                       // ��תһȦ��Ҫ��������

// ��ѧ����
#define ALPHA ((float)(2 * 3.14159 / SPR)) // ��= 2*pi/spr
#define A_T_x10_MOTOR5 ((float)(10 * ALPHA * T1_FREQ_MOTOR5))
#define T1_FREQ_148_MOTOR5 ((float)((T1_FREQ_MOTOR5 * 0.676) / 10)) // 0.676Ϊ�������ֵ
#define A_SQ ((float)(2 * 100000 * ALPHA))
#define A_x200 ((float)(200 * ALPHA))

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim1_MOTOR5;
extern __IO uint8_t Motor5_status;
extern __IO int Motor5_num;
/* �������� ------------------------------------------------------------------*/

void MOTOR5_TIMx_Init(void);
void MOTOR5_AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif /* __MOTOR5_TIM_H__ */
/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
