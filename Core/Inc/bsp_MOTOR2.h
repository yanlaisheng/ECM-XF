#ifndef __MOTOR2_TIM_H__
#define __MOTOR2_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
//
// #define S_ACCEL 1
// #define T_ACCEL 0

//#if S_ACCEL
///* S型加速参数 */
//#define ACCELERATED_SPEED_LENGTH 200 //定义加速度的点数（其实也是3000个细分步的意思），调这个参数改变加速点
//#define FRE_MIN 500                  //最低的运行频率，调这个参数调节最低运行速度
//#define FRE_MAX 35000                //最高的运行频率，调这个参数调节匀速时的最高速度35000
//
//#endif
/* 宏定义 --------------------------------------------------------------------*/
#define MOTOR2_TIM4 TIM4
#define MOTOR2_TIM4_RCC_CLK_ENABLE() __HAL_RCC_TIM4_CLK_ENABLE()
#define MOTOR2_TIM4_RCC_CLK_DISABLE() __HAL_RCC_TIM4_CLK_DISABLE()
#define MOTOR2_TIM4_IT_CCx TIM_IT_CC1
#define MOTOR2_TIM4_FLAG_CCx TIM_FLAG_CC1
#define MOTOR2_TIM4_IRQn TIM4_IRQn
#define MOTOR2_TIM4_IRQHandler TIM4_IRQHandler

#define MOTOR2_TIM4_CHANNEL_x TIM_CHANNEL_1
#define MOTOR2_TIM4_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE() // 输出控制脉冲给电机驱动器
#define MOTOR2_TIM4_PUL_PORT GPIOB                                 // 对应驱动器的PUL-（驱动器使用共阳接法）
#define MOTOR2_TIM4_PUL_PIN GPIO_PIN_6                             // 而PLU+直接接开发板的VCC

#define MOTOR2_DIR_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE() // 电机旋转方向控制，如果悬空不接默认正转
#define MOTOR2_DIR_PORT GPIOB                                     // 对应驱动器的DIR-（驱动器使用共阳接法）
#define MOTOR2_DIR_PIN GPIO_PIN_8                                 // 而DIR+直接接开发板的VCC
#define GPIO_PIN_AF_AS_SYS GPIO_AF0_RTC_50Hz                      // 引脚不作为复用功能使用

#define MOTOR2_ENA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOF_CLK_ENABLE() // 电机脱机使能控制，如果悬空不接默认使能电机
#define MOTOR2_ENA_PORT GPIOF                                     // 对应驱动器的ENA-（驱动器使用共阳接法）
#define MOTOR2_ENA_PIN GPIO_PIN_4                                 // 而ENA+直接接开发板的VCC

#define MOTOR2_DIR_FORWARD() HAL_GPIO_WritePin(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, GPIO_PIN_RESET)
#define MOTOR2_DIR_REVERSAL() HAL_GPIO_WritePin(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, GPIO_PIN_SET)

#define MOTOR2_OUTPUT_ENABLE() HAL_GPIO_WritePin(MOTOR2_ENA_PORT, MOTOR2_ENA_PIN, GPIO_PIN_RESET)
#define MOTOR2_OUTPUT_DISABLE() HAL_GPIO_WritePin(MOTOR2_ENA_PORT, MOTOR2_ENA_PIN, GPIO_PIN_SET)

// 定义定时器预分频，定时器实际时钟频率为：168MHz/（STEPMOTOR_TIMx_PRESCALER+1）
#define MOTOR2_TIM_PRESCALER 7 // 步进电机驱动器细分设置为：   64  细分

// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define MOTOR2_TIM_PERIOD 0xFFFF
// 定义高级定时器重复计数寄存器值
#define MOTOR2_TIM_REPETITIONCOUNTER 0

#define FALSE 0
#define TRUE 1
#define CW 0  // 顺时针
#define CCW 1 // 逆时针

#define STOP 0                                                        // 加减速曲线状态：停止
#define ACCEL 1                                                       // 加减速曲线状态：加速阶段
#define DECEL 2                                                       // 加减速曲线状态：减速阶段
#define RUN 3                                                         // 加减速曲线状态：匀速阶段
#define T1_FREQ_MOTOR2 (SystemCoreClock / (MOTOR2_TIM_PRESCALER + 1)) // 频率ft值
#define FSPR 200                                                      //步进电机单圈步数
#define MICRO_STEP 64                                                 // 步进电机驱动器细分数
#define SPR (FSPR * MICRO_STEP)                                       // 旋转一圈需要的脉冲数

// 数学常数
#define ALPHA ((float)(2 * 3.14159 / SPR)) // α= 2*pi/spr
#define A_T_x10_MOTOR2 ((float)(10 * ALPHA * T1_FREQ_MOTOR2))
#define T1_FREQ_148_MOTOR2 ((float)((T1_FREQ_MOTOR2 * 0.676) / 10)) // 0.676为误差修正值
#define A_SQ ((float)(2 * 100000 * ALPHA))
#define A_x200 ((float)(200 * ALPHA))

/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim4_MOTOR2;
extern __IO uint8_t Motor2_status;
extern __IO int Motor2_num;
/* 函数声明 ------------------------------------------------------------------*/

void MOTOR2_TIMx_Init(void);
void MOTOR2_AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif /* __MOTOR2_TIM_H__ */
/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
