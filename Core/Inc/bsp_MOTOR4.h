#ifndef __MOTOR4_TIM_H__
#define __MOTOR4_TIM_H__

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
#define MOTOR4_TIM3 TIM3
#define MOTOR4_TIM3_RCC_CLK_ENABLE() __HAL_RCC_TIM3_CLK_ENABLE()
#define MOTOR4_TIM3_RCC_CLK_DISABLE() __HAL_RCC_TIM3_CLK_DISABLE()
#define MOTOR4_TIM3_IT_CCx TIM_IT_CC4
#define MOTOR4_TIM3_FLAG_CCx TIM_FLAG_CC4
#define MOTOR4_TIM3_IRQn TIM3_IRQn
#define MOTOR4_TIM3_IRQHandler TIM3_IRQHandler

#define MOTOR4_TIM3_CHANNEL_x TIM_CHANNEL_4
#define MOTOR4_TIM3_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE() // ���������������������
#define MOTOR4_TIM3_PUL_PORT GPIOB                                 // ��Ӧ��������PUL-��������ʹ�ù����ӷ���
#define MOTOR4_TIM3_PUL_PIN GPIO_PIN_1                             // ��PLU+ֱ�ӽӿ������VCC

#define MOTOR4_DIR_GPIO_CLK_ENABLE() __HAL_RCC_GPIOF_CLK_ENABLE() // �����ת������ƣ�������ղ���Ĭ����ת
#define MOTOR4_DIR_PORT GPIOF                                     // ��Ӧ��������DIR-��������ʹ�ù����ӷ���
#define MOTOR4_DIR_PIN GPIO_PIN_12                                // ��DIR+ֱ�ӽӿ������VCC
//#define GPIO_PIN_AF_AS_SYS GPIO_AF2_RTC_50Hz                      // ���Ų���Ϊ���ù���ʹ��

#define MOTOR4_ENA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOF_CLK_ENABLE() // ����ѻ�ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define MOTOR4_ENA_PORT GPIOF                                     // ��Ӧ��������ENA-��������ʹ�ù����ӷ���
#define MOTOR4_ENA_PIN GPIO_PIN_13                                // ��ENA+ֱ�ӽӿ������VCC

#define MOTOR4_DIR_FORWARD() HAL_GPIO_WritePin(MOTOR4_DIR_PORT, MOTOR4_DIR_PIN, GPIO_PIN_RESET)
#define MOTOR4_DIR_REVERSAL() HAL_GPIO_WritePin(MOTOR4_DIR_PORT, MOTOR4_DIR_PIN, GPIO_PIN_SET)

#define MOTOR4_OUTPUT_ENABLE() HAL_GPIO_WritePin(MOTOR4_ENA_PORT, MOTOR4_ENA_PIN, GPIO_PIN_RESET)
#define MOTOR4_OUTPUT_DISABLE() HAL_GPIO_WritePin(MOTOR4_ENA_PORT, MOTOR4_ENA_PIN, GPIO_PIN_SET)

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��STEPMOTOR_TIMx_PRESCALER+1��
#define MOTOR4_TIM_PRESCALER 7 // �������������ϸ������Ϊ��   64  ϸ��

// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define MOTOR4_TIM_PERIOD 0xFFFF
// ����߼���ʱ���ظ������Ĵ���ֵ
#define MOTOR4_TIM_REPETITIONCOUNTER 0

#define FALSE 0
#define TRUE 1
#define CW 0  // ˳ʱ��
#define CCW 1 // ��ʱ��

#define STOP 0                                                        // �Ӽ�������״̬��ֹͣ
#define ACCEL 1                                                       // �Ӽ�������״̬�����ٽ׶�
#define DECEL 2                                                       // �Ӽ�������״̬�����ٽ׶�
#define RUN 3                                                         // �Ӽ�������״̬�����ٽ׶�
#define T1_FREQ_MOTOR4 (SystemCoreClock / (MOTOR4_TIM_PRESCALER + 1)) // Ƶ��ftֵ
#define FSPR 200                                                      //���������Ȧ����
#define MICRO_STEP 64                                                 // �������������ϸ����
#define SPR (FSPR * MICRO_STEP)                                       // ��תһȦ��Ҫ��������

// ��ѧ����
#define ALPHA ((float)(2 * 3.14159 / SPR)) // ��= 2*pi/spr
#define A_T_x10_MOTOR4 ((float)(10 * ALPHA * T1_FREQ_MOTOR4))
#define T1_FREQ_148_MOTOR4 ((float)((T1_FREQ_MOTOR4 * 0.676) / 10)) // 0.676Ϊ�������ֵ
#define A_SQ ((float)(2 * 100000 * ALPHA))
#define A_x200 ((float)(200 * ALPHA))

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim3_MOTOR4;
extern __IO uint8_t Motor4_status;
extern __IO int Motor4_num;
/* �������� ------------------------------------------------------------------*/

void MOTOR4_TIMx_Init(void);
void MOTOR4_AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void MOTOR4_GPIO_Init(void);

#endif /* __MOTOR4_TIM_H__ */
/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
