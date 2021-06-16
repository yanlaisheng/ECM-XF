/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

  /* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN Includes */
#include "GlobalConst.h"
#include "typedef.h"
  /* USER CODE END Includes */

  /* Exported types ------------------------------------------------------------*/
  /* USER CODE BEGIN ET */
  extern uint8_t WkPort1_RcvBuffer[RCV1_MAX]; // 接收缓冲区
  extern uint8_t WkPort2_RcvBuffer[RCV1_MAX]; // 接收缓冲区
  extern uint8_t WkPort3_RcvBuffer[RCV1_MAX]; // 接收缓冲区
  extern uint8_t WkPort4_RcvBuffer[RCV1_MAX]; // 接收缓冲区
  extern char printbuf[128];
  extern UART_HandleTypeDef huart3;

  void Driver_Control(void);                        //电机运行控制
  void Reset_Routine(void);                         //复位子程序
  void Reset_Drivers(void);                         //复位电机到初始位置
  void Stop_Driver(u8 driver_no);                   //发送停机命令
  void Control_Hand(void);                          //控制电磁阀(机械手)开通，关断
  void BRAKE_Control(void);                         //刹车控制
  void is_Reseted(void);                            //判断复位到原点子程序
  void Send_StopCMD(void);                          //发送停机命令子程序
  void PosCMD_ReadFrom_exFLASH(void);               //按组进行参数初始化
  void Driver_Save_Pos(void);                       //写入位置参数到伺服驱动器
  s16 Move_Cmd_Queue(u8 driver_no);                 //移动队列指针
  void Clear_Cmd_Queue(void);                       //清命令缓冲区
  void sort(s32 *a, int l);                         //数组排序函数
  void sort_f(float *a, int l);                     //浮点数数组排序函数
  u8 Cal_Pos_Speed(void);                           //计算脉冲函数
  void Cal_One_Pos_Speed(s32 Pos_No1, s32 Pos_No2); //计算一个位置的脉冲与速度
  void Record_Current_Pos(void);                    //记录当前位置
  void Run_to_One_Pos(s32 Cmd_No);                  //运行到某一命令号对应的位置点
  void Fill_Cmd(void);                              //填充命令
  void Cal_Sum_Err(u8 Driver_No);                   //计算四舍五入带来的误差
  void Cal_Run_to_Next_Pos(s32 Cmd_No);             //从当前位置同步运行到某一命令号对应的位置点
  void Send_Driver_CMD(u8 driver_no, u8 Modbus_CMD_No, u32 Par_addr, u32 Par_content);
  //清零位置及位置命令
  extern void ParLst_Init_Group2Zero(void);
  //清零位置
  void ParLst_Init_Pos2Zero(void);
  //编码器位置手动调整函数
  void Pos_Manual_Adj(void);
  //调整位置
  void Adj_Pos(u8 driver_no, s32 PosErr_Muti, s32 PosErr_Sing);
  //发送位置数据
  void Send_Pos_Data(void);
  //校验位置命令是否正确
  void Verify_Pos_CMD(void);
  void Limit_Max_Pos(void); //限制最大最小位置范围
  //判断某台电机是否超出位置范围
  u8 Judge_OverPos(u8 Driver_No);
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#ifndef PRINTF
#define PRINTF(str, ...)                                            \
  do                                                                \
  {                                                                 \
    int n;                                                          \
    n = sprintf(printbuf, (str), ##__VA_ARGS__);                    \
    HAL_UART_Transmit(&huart3, (uint8_t *)printbuf, n, 0xffffffff); \
  } while (0)
#endif
#ifndef GETCHAR
#define GETCHAR userGetchar
#endif

//ECAT使能
#define SPI_ECAT_CS_ON HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET) //SPI使能
#define SPI_ECAT_CS_OFF HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET)  //SPI禁止使能

  /* USER CODE END EM */

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

  /* USER CODE BEGIN EFP */
  void SystemClock_Config(void);
  void Com1_RcvProcess(void);
  void Com1_SlaveSend(void);
  void Com2_RcvProcess(void);
  void Com2_SlaveSend(void);
  void Com3_RcvProcess(void);
  void Com3_SlaveSend(void);
  void Com4_RcvProcess(void);
  void Com4_SlaveSend(void);
  void Com5_RcvProcess(void);
  void Com5_SlaveSend(void);
  void Com6_RcvProcess(void);
  void Com6_SlaveSend(void);
  void ParLimit(void);
  void test_GPIO(void);
  void Init_EtherCAT(void);
  void UART_TX(void);
  void Test_EEPROM(void);
  void Init_WK_Uart(void);
  void TEST_Send_Pwm(void);
  void Initial_PWM_Motors(void);
  void Rd2Wr_IO(void);
  void WK_Com1_RcvProcess(void);
  void WK_Com1_SlaveSend(void);
  void WK_Com2_RcvProcess(void);
  void WK_Com2_SlaveSend(void);
  void WK_Com3_RcvProcess(void);
  void WK_Com3_SlaveSend(void);
  void WK_Com4_RcvProcess(void);
  void WK_Com4_SlaveSend(void);
  void InitClock(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DI12_Pin GPIO_PIN_2
#define DI12_GPIO_Port GPIOE
#define DI11_Pin GPIO_PIN_3
#define DI11_GPIO_Port GPIOE
#define DI10_Pin GPIO_PIN_4
#define DI10_GPIO_Port GPIOE
#define DI9_Pin GPIO_PIN_6
#define DI9_GPIO_Port GPIOE
#define DI8_Pin GPIO_PIN_0
#define DI8_GPIO_Port GPIOF
#define DI7_Pin GPIO_PIN_1
#define DI7_GPIO_Port GPIOF
#define DI6_Pin GPIO_PIN_2
#define DI6_GPIO_Port GPIOF
#define DI5_Pin GPIO_PIN_3
#define DI5_GPIO_Port GPIOF
#define DO10_Pin GPIO_PIN_4
#define DO10_GPIO_Port GPIOF
#define WK2124_IRQ_Pin GPIO_PIN_8
#define WK2124_IRQ_GPIO_Port GPIOF
#define WK2124_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define WK2124_RST_Pin GPIO_PIN_1
#define WK2124_RST_GPIO_Port GPIOC
#define DI17_Pin GPIO_PIN_4
#define DI17_GPIO_Port GPIOC
#define DI18_Pin GPIO_PIN_5
#define DI18_GPIO_Port GPIOC
#define DI19_Pin GPIO_PIN_0
#define DI19_GPIO_Port GPIOB
#define DI20_Pin GPIO_PIN_11
#define DI20_GPIO_Port GPIOF
#define DO14_Pin GPIO_PIN_12
#define DO14_GPIO_Port GPIOF
#define DO15_Pin GPIO_PIN_13
#define DO15_GPIO_Port GPIOF
#define DO16_Pin GPIO_PIN_14
#define DO16_GPIO_Port GPIOF
#define DO17_Pin GPIO_PIN_15
#define DO17_GPIO_Port GPIOF
#define DO18_Pin GPIO_PIN_0
#define DO18_GPIO_Port GPIOG
#define DO19_Pin GPIO_PIN_1
#define DO19_GPIO_Port GPIOG
#define DI21_Pin GPIO_PIN_7
#define DI21_GPIO_Port GPIOE
#define DI22_Pin GPIO_PIN_8
#define DI22_GPIO_Port GPIOE
#define TIM1_GPIO_Pin GPIO_PIN_9
#define TIM1_GPIO_GPIO_Port GPIOE
#define DI23_Pin GPIO_PIN_10
#define DI23_GPIO_Port GPIOE
#define DI24_Pin GPIO_PIN_11
#define DI24_GPIO_Port GPIOE
#define DI25_Pin GPIO_PIN_12
#define DI25_GPIO_Port GPIOE
#define DI26_Pin GPIO_PIN_13
#define DI26_GPIO_Port GPIOE
#define DI27_Pin GPIO_PIN_14
#define DI27_GPIO_Port GPIOE
#define DI28_Pin GPIO_PIN_15
#define DI28_GPIO_Port GPIOE
#define FRAM_CS_Pin GPIO_PIN_12
#define FRAM_CS_GPIO_Port GPIOB
#define FLASH_CS_Pin GPIO_PIN_8
#define FLASH_CS_GPIO_Port GPIOD
#define DI1_Pin GPIO_PIN_10
#define DI1_GPIO_Port GPIOD
#define DI2_Pin GPIO_PIN_11
#define DI2_GPIO_Port GPIOD
#define DI3_Pin GPIO_PIN_12
#define DI3_GPIO_Port GPIOD
#define DI4_Pin GPIO_PIN_13
#define DI4_GPIO_Port GPIOD
#define DO20_Pin GPIO_PIN_14
#define DO20_GPIO_Port GPIOD
#define DO21_Pin GPIO_PIN_15
#define DO21_GPIO_Port GPIOD
#define DO22_Pin GPIO_PIN_2
#define DO22_GPIO_Port GPIOG
#define DO13_Pin GPIO_PIN_3
#define DO13_GPIO_Port GPIOG
#define DO12_Pin GPIO_PIN_4
#define DO12_GPIO_Port GPIOG
#define DO11_Pin GPIO_PIN_5
#define DO11_GPIO_Port GPIOG
#define DS1302_CLK_Pin GPIO_PIN_6
#define DS1302_CLK_GPIO_Port GPIOG
#define DS1302_DATA_Pin GPIO_PIN_7
#define DS1302_DATA_GPIO_Port GPIOG
#define DS1302_RST_Pin GPIO_PIN_8
#define DS1302_RST_GPIO_Port GPIOG
#define DO4_Pin GPIO_PIN_7
#define DO4_GPIO_Port GPIOC
#define DO3_Pin GPIO_PIN_8
#define DO3_GPIO_Port GPIOC
#define DO2_Pin GPIO_PIN_9
#define DO2_GPIO_Port GPIOC
#define DO1_Pin GPIO_PIN_8
#define DO1_GPIO_Port GPIOA
#define LED_H63_Pin GPIO_PIN_11
#define LED_H63_GPIO_Port GPIOA
#define LED_H70_Pin GPIO_PIN_12
#define LED_H70_GPIO_Port GPIOA
#define LED_H69_Pin GPIO_PIN_15
#define LED_H69_GPIO_Port GPIOA
#define LED_H68_Pin GPIO_PIN_0
#define LED_H68_GPIO_Port GPIOD
#define LED_H67_Pin GPIO_PIN_1
#define LED_H67_GPIO_Port GPIOD
#define LED_H66_Pin GPIO_PIN_3
#define LED_H66_GPIO_Port GPIOD
#define LED_H65_Pin GPIO_PIN_4
#define LED_H65_GPIO_Port GPIOD
#define LED_H64_Pin GPIO_PIN_5
#define LED_H64_GPIO_Port GPIOD
#define DI16_Pin GPIO_PIN_7
#define DI16_GPIO_Port GPIOD
#define DI15_Pin GPIO_PIN_10
#define DI15_GPIO_Port GPIOG
#define DI14_Pin GPIO_PIN_11
#define DI14_GPIO_Port GPIOG
#define DI13_Pin GPIO_PIN_12
#define DI13_GPIO_Port GPIOG
#define SPI3_CS_Pin GPIO_PIN_13
#define SPI3_CS_GPIO_Port GPIOG
#define BUSY_Pin GPIO_PIN_15
#define BUSY_GPIO_Port GPIOG
#define DO9_Pin GPIO_PIN_7
#define DO9_GPIO_Port GPIOB
#define DO8_Pin GPIO_PIN_8
#define DO8_GPIO_Port GPIOB
#define DO7_Pin GPIO_PIN_9
#define DO7_GPIO_Port GPIOB
#define DO6_Pin GPIO_PIN_0
#define DO6_GPIO_Port GPIOE
#define DO5_Pin GPIO_PIN_1
#define DO5_GPIO_Port GPIOE
  /* USER CODE BEGIN Private defines */

  int userGetchar(void);

  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
