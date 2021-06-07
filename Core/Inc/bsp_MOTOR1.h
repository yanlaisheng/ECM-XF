#ifndef __MOTOR1_TIM_H__
#define __MOTOR1_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
//
// #define S_ACCEL 1
// #define T_ACCEL 0

//#if S_ACCEL
///* S�ͼ��ٲ��� */
//#define ACCELERATED_SPEED_LENGTH 200 //������ٶȵĵ�������ʵҲ��3000��ϸ�ֲ�����˼��������������ı���ٵ�
//#define FRE_MIN 500                  //��͵�����Ƶ�ʣ����������������������ٶ�
//#define FRE_MAX 35000                //��ߵ�����Ƶ�ʣ������������������ʱ������ٶ�35000
//
//#endif
/* �궨�� --------------------------------------------------------------------*/
#define MOTOR1_TIM2 TIM2
#define MOTOR1_TIM2_RCC_CLK_ENABLE() __HAL_RCC_TIM2_CLK_ENABLE()
#define MOTOR1_TIM2_RCC_CLK_DISABLE() __HAL_RCC_TIM2_CLK_DISABLE()
#define MOTOR1_TIM2_IT_CCx TIM_IT_CC2
#define MOTOR1_TIM2_FLAG_CCx TIM_FLAG_CC2
#define MOTOR1_TIM2_IRQn TIM2_IRQn
#define MOTOR1_TIM2_IRQHandler TIM2_IRQHandler

#define MOTOR1_TIM2_CHANNEL_x TIM_CHANNEL_2
#define MOTOR1_TIM2_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() // ���������������������
#define MOTOR1_TIM2_PUL_PORT GPIOA                                 // ��Ӧ��������PUL-��������ʹ�ù����ӷ���
#define MOTOR1_TIM2_PUL_PIN GPIO_PIN_1                             // ��PLU+ֱ�ӽӿ������VCC

#define MOTOR1_DIR_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE() // �����ת������ƣ�������ղ���Ĭ����ת
#define MOTOR1_DIR_PORT GPIOE                                     // ��Ӧ��������DIR-��������ʹ�ù����ӷ���
#define MOTOR1_DIR_PIN GPIO_PIN_1                                 // ��DIR+ֱ�ӽӿ������VCC
#define GPIO_PIN_AF_AS_SYS GPIO_AF0_RTC_50Hz                      // ���Ų���Ϊ���ù���ʹ��

#define MOTOR1_ENA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE() // ����ѻ�ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define MOTOR1_ENA_PORT GPIOB                                     // ��Ӧ��������ENA-��������ʹ�ù����ӷ���
#define MOTOR1_ENA_PIN GPIO_PIN_9                                 // ��ENA+ֱ�ӽӿ������VCC

#define MOTOR1_DIR_FORWARD() HAL_GPIO_WritePin(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, GPIO_PIN_RESET)
#define MOTOR1_DIR_REVERSAL() HAL_GPIO_WritePin(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, GPIO_PIN_SET)

#define MOTOR1_OUTPUT_ENABLE() HAL_GPIO_WritePin(MOTOR1_ENA_PORT, MOTOR1_ENA_PIN, GPIO_PIN_RESET)
#define MOTOR1_OUTPUT_DISABLE() HAL_GPIO_WritePin(MOTOR1_ENA_PORT, MOTOR1_ENA_PIN, GPIO_PIN_SET)

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��STEPMOTOR_TIMx_PRESCALER+1��
#define MOTOR1_TIM_PRESCALER 7 // Ԥ��Ƶ

// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define MOTOR1_TIM_PERIOD 0xFFFF
// ����߼���ʱ���ظ������Ĵ���ֵ
#define MOTOR1_TIM_REPETITIONCOUNTER 0

#define FALSE 0
#define TRUE 1
#define CW 0  // ˳ʱ��
#define CCW 1 // ��ʱ��

#define STOP 0                                                        // �Ӽ�������״̬��ֹͣ
#define ACCEL 1                                                       // �Ӽ�������״̬�����ٽ׶�
#define DECEL 2                                                       // �Ӽ�������״̬�����ٽ׶�
#define RUN 3                                                         // �Ӽ�������״̬�����ٽ׶�
#define T1_FREQ_MOTOR1 (SystemCoreClock / (MOTOR1_TIM_PRESCALER + 1)) // Ƶ��ftֵ
#define FSPR 200                                                      //���������Ȧ����
#define MICRO_STEP 64                                                 // �������������ϸ����
#define SPR (FSPR * MICRO_STEP)                                       // ��תһȦ��Ҫ��������

// ��ѧ����
#define ALPHA ((float)(2 * 3.14159 / SPR)) // ��= 2*pi/spr
#define A_T_x10_MOTOR1 ((float)(10 * ALPHA * T1_FREQ_MOTOR1))
#define T1_FREQ_148_MOTOR1 ((float)((T1_FREQ_MOTOR1 * 0.676) / 10)) // 0.676Ϊ�������ֵ
#define A_SQ ((float)(2 * 100000 * ALPHA))
#define A_x200 ((float)(200 * ALPHA))

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim2_MOTOR1;
extern __IO uint8_t Motor1_status;
extern __IO int Motor1_num;
extern __IO uint8_t Motor1_MotionStatus;
/* �������� ------------------------------------------------------------------*/

void MOTOR1_TIMx_Init(void);
// void MOTOR1_AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif /* __MOTOR1_TIM_H__ */
/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
