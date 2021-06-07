#ifndef __MOTOR6_TIM_H__
#define __MOTOR6_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
//
//#define S_ACCEL 0
//#define T_ACCEL 1

// #if S_ACCEL
// /* S�ͼ��ٲ��� */
// #define ACCELERATED_SPEED_LENGTH 200 //������ٶȵĵ�������ʵҲ��3000��ϸ�ֲ�����˼��������������ı���ٵ�
// #define FRE_MIN 500                  //��͵�����Ƶ�ʣ����������������������ٶ�
// #define FRE_MAX 35000                //��ߵ�����Ƶ�ʣ������������������ʱ������ٶ�35000

// #endif
/* �궨�� --------------------------------------------------------------------*/
#define MOTOR6_TIM5 TIM5
#define MOTOR6_TIM5_RCC_CLK_ENABLE() __HAL_RCC_TIM5_CLK_ENABLE()
#define MOTOR6_TIM5_RCC_CLK_DISABLE() __HAL_RCC_TIM5_CLK_DISABLE()
#define MOTOR6_TIM5_IT_CCx TIM_IT_CC1
#define MOTOR6_TIM5_FLAG_CCx TIM_FLAG_CC1
#define MOTOR6_TIM5_IRQn TIM5_IRQn
#define MOTOR6_TIM5_IRQHandler TIM5_IRQHandler

#define MOTOR6_TIM5_CHANNEL_x TIM_CHANNEL_1
#define MOTOR6_TIM5_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() // ���������������������
#define MOTOR6_TIM5_PUL_PORT GPIOA                                 // ��Ӧ��������PUL-��������ʹ�ù����ӷ���
#define MOTOR6_TIM5_PUL_PIN GPIO_PIN_0                             // ��PLU+ֱ�ӽӿ������VCC

#define MOTOR6_DIR_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE() // �����ת������ƣ�������ղ���Ĭ����ת
#define MOTOR6_DIR_PORT GPIOD                                     // ��Ӧ��������DIR-��������ʹ�ù����ӷ���
#define MOTOR6_DIR_PIN GPIO_PIN_14                                // ��DIR+ֱ�ӽӿ������VCC
#define GPIO_PIN_AF_AS_SYS GPIO_AF0_RTC_50Hz                      // ���Ų���Ϊ���ù���ʹ��

#define MOTOR6_ENA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE() // ����ѻ�ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define MOTOR6_ENA_PORT GPIOD                                     // ��Ӧ��������ENA-��������ʹ�ù����ӷ���
#define MOTOR6_ENA_PIN GPIO_PIN_15                                // ��ENA+ֱ�ӽӿ������VCC

#define MOTOR6_DIR_FORWARD() HAL_GPIO_WritePin(MOTOR6_DIR_PORT, MOTOR6_DIR_PIN, GPIO_PIN_RESET)
#define MOTOR6_DIR_REVERSAL() HAL_GPIO_WritePin(MOTOR6_DIR_PORT, MOTOR6_DIR_PIN, GPIO_PIN_SET)

#define MOTOR6_OUTPUT_ENABLE() HAL_GPIO_WritePin(MOTOR6_ENA_PORT, MOTOR6_ENA_PIN, GPIO_PIN_RESET)
#define MOTOR6_OUTPUT_DISABLE() HAL_GPIO_WritePin(MOTOR6_ENA_PORT, MOTOR6_ENA_PIN, GPIO_PIN_SET)

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��STEPMOTOR_TIMx_PRESCALER+1��
#define MOTOR6_TIM_PRESCALER 7 // �������������ϸ������Ϊ��   64  ϸ��

// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define MOTOR6_TIM_PERIOD 0xFFFF
// ����߼���ʱ���ظ������Ĵ���ֵ
#define MOTOR6_TIM_REPETITIONCOUNTER 0

#define FALSE 0
#define TRUE 1
#define CW 0  // ˳ʱ��
#define CCW 1 // ��ʱ��

#define STOP 0                                                        // �Ӽ�������״̬��ֹͣ
#define ACCEL 1                                                       // �Ӽ�������״̬�����ٽ׶�
#define DECEL 2                                                       // �Ӽ�������״̬�����ٽ׶�
#define RUN 3                                                         // �Ӽ�������״̬�����ٽ׶�
#define T1_FREQ_MOTOR6 (SystemCoreClock / (MOTOR6_TIM_PRESCALER + 1)) // Ƶ��ftֵ
#define FSPR 200                                                      //���������Ȧ����
#define MICRO_STEP 64                                                 // �������������ϸ����
#define SPR (FSPR * MICRO_STEP)                                       // ��תһȦ��Ҫ��������

// ��ѧ����
#define ALPHA ((float)(2 * 3.14159 / SPR)) // ��= 2*pi/spr
#define A_T_x10_MOTOR6 ((float)(10 * ALPHA * T1_FREQ_MOTOR6))
#define T1_FREQ_148_MOTOR6 ((float)((T1_FREQ_MOTOR6 * 0.676) / 10)) // 0.676Ϊ�������ֵ
#define A_SQ ((float)(2 * 100000 * ALPHA))
#define A_x200 ((float)(200 * ALPHA))

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim5_MOTOR6;
extern __IO uint8_t Motor6_status;
extern __IO int Motor6_num;
/* �������� ------------------------------------------------------------------*/

void MOTOR6_TIMx_Init(void);
void MOTOR6_AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif /* __MOTOR6_TIM_H__ */
/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
