#ifndef __MOTOR3_TIM_H__
#define __MOTOR3_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef struct
{
  __IO uint8_t run_state;    // �����ת״̬
  __IO uint8_t dir;          // �����ת����
  __IO int32_t step_delay;   // �¸��������ڣ�ʱ������������ʱΪ���ٶ�
  __IO uint32_t decel_start; // ��������λ��
  __IO int32_t decel_val;    // ���ٽ׶β���
  __IO int32_t min_delay;    // ��С��������(����ٶȣ������ٶ��ٶ�)
  __IO int32_t accel_count;  // �Ӽ��ٽ׶μ���ֵ
} speedRampData;

/* �궨�� --------------------------------------------------------------------*/
#define MOTOR3_TIM8 TIM8
#define MOTOR3_TIM8_RCC_CLK_ENABLE() __HAL_RCC_TIM8_CLK_ENABLE()
#define MOTOR3_TIM8_RCC_CLK_DISABLE() __HAL_RCC_TIM8_CLK_DISABLE()
#define MOTOR3_TIM8_IT_CCx TIM_IT_CC1
#define MOTOR3_TIM8_FLAG_CCx TIM_FLAG_CC1
#define MOTOR3_TIM8_IRQn TIM8_UP_TIM13_IRQn
#define MOTOR3_TIM8_IRQHandler TIM8_CC_IRQHandler

#define MOTOR3_TIM8_CHANNEL_x TIM_CHANNEL_1
#define MOTOR3_TIM8_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE() // ���������������������
#define MOTOR3_TIM8_PUL_PORT GPIOC                                 // ��Ӧ��������PUL-��������ʹ�ù����ӷ���
#define MOTOR3_TIM8_PUL_PIN GPIO_PIN_6                             // ��PLU+ֱ�ӽӿ������VCC

#define MOTOR3_DIR_GPIO_CLK_ENABLE() __HAL_RCC_GPIOG_CLK_ENABLE() // �����ת������ƣ�������ղ���Ĭ����ת
#define MOTOR3_DIR_PORT GPIOG                                     // ��Ӧ��������DIR-��������ʹ�ù����ӷ���
#define MOTOR3_DIR_PIN GPIO_PIN_5                                 // ��DIR+ֱ�ӽӿ������VCC
//#define GPIO_PIN_AF_AS_SYS GPIO_AF0_RTC_50Hz                      // ���Ų���Ϊ���ù���ʹ��

#define MOTOR3_ENA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOG_CLK_ENABLE() // ����ѻ�ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define MOTOR3_ENA_PORT GPIOG                                     // ��Ӧ��������ENA-��������ʹ�ù����ӷ���
#define MOTOR3_ENA_PIN GPIO_PIN_4                                 // ��ENA+ֱ�ӽӿ������VCC

#define MOTOR3_DIR_FORWARD() HAL_GPIO_WritePin(MOTOR3_DIR_PORT, MOTOR3_DIR_PIN, GPIO_PIN_RESET)
#define MOTOR3_DIR_REVERSAL() HAL_GPIO_WritePin(MOTOR3_DIR_PORT, MOTOR3_DIR_PIN, GPIO_PIN_SET)

#define MOTOR3_OUTPUT_ENABLE() HAL_GPIO_WritePin(MOTOR3_ENA_PORT, MOTOR3_ENA_PIN, GPIO_PIN_RESET)
#define MOTOR3_OUTPUT_DISABLE() HAL_GPIO_WritePin(MOTOR3_ENA_PORT, MOTOR3_ENA_PIN, GPIO_PIN_SET)

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��STEPMOTOR_TIMx_PRESCALER+1��
#define MOTOR3_TIM_PRESCALER 15 // �������������ϸ������Ϊ��   64  ϸ��

// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define MOTOR3_TIM_PERIOD 0xFFFF
// ����߼���ʱ���ظ������Ĵ���ֵ
#define MOTOR3_TIM_REPETITIONCOUNTER 0

#define FALSE 0
#define TRUE 1
#define CW 0  // ˳ʱ��
#define CCW 1 // ��ʱ��

#define STOP 0                                                        // �Ӽ�������״̬��ֹͣ
#define ACCEL 1                                                       // �Ӽ�������״̬�����ٽ׶�
#define DECEL 2                                                       // �Ӽ�������״̬�����ٽ׶�
#define RUN 3                                                         // �Ӽ�������״̬�����ٽ׶�
#define T1_FREQ_MOTOR3 (SystemCoreClock / (MOTOR3_TIM_PRESCALER + 1)) // Ƶ��ftֵ
#define FSPR 200                                                      //���������Ȧ����
#define MICRO_STEP 64                                                 // �������������ϸ����
#define SPR (FSPR * MICRO_STEP)                                       // ��תһȦ��Ҫ��������

// ��ѧ����
#define ALPHA ((float)(2 * 3.14159 / SPR)) // ��= 2*pi/spr
#define A_T_x10 ((float)(10 * ALPHA * T1_FREQ_MOTOR3))
#define T1_FREQ_148 ((float)((T1_FREQ_MOTOR3 * 0.676) / 10)) // 0.676Ϊ�������ֵ
#define A_SQ ((float)(2 * 100000 * ALPHA))
#define A_x200 ((float)(200 * ALPHA))

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim8_MOTOR3;
extern __IO uint8_t Motor3_status;
extern __IO int Motor3_num;
/* �������� ------------------------------------------------------------------*/

void MOTOR3_TIMx_Init(void);
void MOTOR3_AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif /* __MOTOR3_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
