/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 QINGDAO SANLI.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "stdio.h"
#include "stddef.h"
#include <stdlib.h>
#include <math.h>
#include "ecm_define.h"
#include "GlobalConst.h"
#include "GlobalV.h"
#include "GlobalV_Extern.h" // ȫ�ֱ�������
#include "typedef.h"
#include "wk2xxx.h"
#include "w25qxx.h"
#include "bsp_MOTOR3.h"
#include "bsp_MOTOR1.h"
#include "bsp_MOTOR2.h"
#include "bsp_MOTOR4.h"
#include "bsp_MOTOR5.h"
#include "bsp_MOTOR6.h"
#include "Macro.h"
#include "global_varial.h"
#include "Dowith.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef GNUC
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* GNUC */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint8_t			bNetworkMap[42] = {DRIVE,DRIVE,DRIVE,DRIVE,DRIVE,DRIVE,DRIVE,DRIVE};
uint8_t bNetworkMap[42] = {DRIVE, None, None, None, None, None, None, None};
char printbuf[128];
int GPIO_test(void);

#define NUM 1000
uint32_t send_Buf[NUM] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static int startflag = 0;
int32_t targetp;

int32_t current_Pos;
int32_t current_Pos2;
// uint16_t Master_CMD;
// uint16_t Master_state;
// uint16_t current_state;
int32_t distance; //���о���
uint8_t dir = 0;  //����
uint16_t asc;	  //���ټ���

extern uint32_t IO_Input;

struct rbuff_st
{
	uint16_t CMD;
	uint16_t Parm;
	int32_t Data1;
	int32_t Data2;
}; /* Receive Buffer */
extern struct rbuff_st rbuff[];

//Ҫд�뵽W25Q16���ַ�������
const u8 TEXT_Buffer[] = {"Explorer STM32F4 SPI TEST"};
#define SIZE sizeof(TEXT_Buffer)

int main_ini(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//�ض���printf��scanf���޸�fgetc��fputc
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

GETCHAR_PROTOTYPE
{
	int ch;
	HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
//int ECMSPIReadWrite(uint8_t *rdata, uint8_t *wdata, int rwlength)
//{
//	uint8_t *pTxBuffPtr; /*!< Pointer to SPI Tx transfer Buffer        */
//	uint8_t *pRxBuffPtr; /*!< Pointer to SPI Rx transfer Buffer        */
//	pRxBuffPtr = (uint8_t *)rdata;
//	pTxBuffPtr = (uint8_t *)wdata;
//	//���BUSY�����Ǹߵ�ƽ����һֱ�ȴ�
//	// while (HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin))
//	;
//	SPI_ECAT_CS_ON;
//	HAL_SPI_TransmitReceive(&hspi3, wdata, rdata, rwlength, 10);
//	SPI_ECAT_CS_OFF;
//	return 0;
//}

int userGetchar(void)
{
	uint8_t charbuf;
	HAL_StatusTypeDef ret;
	ret = HAL_UART_Receive(&huart3, &charbuf, 1, 0xffffffff);
	if (ret)
		return -ret;
	return charbuf;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	HAL_Delay(2000); //��ʱ
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_SPI3_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM8_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART6_UART_Init();
	MX_TIM7_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */
	//��ʼ��WK2124��չ����
	Init_WK_Uart();

	//����EEPROM
	Test_EEPROM();

	//EtherCAT��ʼ��
	// Init_EtherCAT();
	//	main_ini();
	//	GPIO_test();

	//��ʼ�������PWM
	Initial_PWM_Motors();

	ParLst_Init();	 // RAM���趨�������г�ʼ����FLASH����>RAM
	Boot_ParLst();	 // ��ʼ��STM32�趨����
	Variable_Init(); // Initialize VARIABLE
	ParLimit();		 // ��������

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		ParLimit();
		UART_TX();
		//����WK��չ����
		// TEST_WK_Uart();
		// test_GPIO();
		WK_Com1_RcvProcess();
		WK_Com1_SlaveSend();

		WK_Com2_RcvProcess();
		WK_Com2_SlaveSend();

		WK_Com3_RcvProcess();
		WK_Com3_SlaveSend();

		TEST_Send_Pwm();

		// HAL_GPIO_TogglePin(DO8_GPIO_Port, DO8_Pin);

		// HAL_Delay(200);

		// Rd2Wr_IO();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
  */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
  */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
//��ʱ1us
void PowerDelay(uint16_t nCount) //2015.9.12
{
	unsigned int i = 36;
	while (nCount--)
	{
		while (i > 0)
			i--;
		i = 36;
	}
}

void HAL_SYSTICK_Callback(void)
{
	static uint8_t time = 0;
	static uint32_t kk = 0;
	if (startflag)
	{
		//		targetp+=1000;

		time++;
		if (time > 0)
		{
			if ((rbuff[0].CMD & 0x00FF) == 0x00BF && (rbuff[0].Parm & 0x00FF) == 0x0008) //&&(rbuff[1].Parm&0xFF00)==0x0600
			{
				for (kk = 0; kk < 100; kk++)
				{
					//					tmp1=rbuff[1].Data1+80;
					if (dir == 1) //�����������λ�ü�
					{
						if (distance < 10) //���мӼ�������
							asc += 2;
						else if (distance > 40)
							asc -= 2;
						if (asc > 2000)
							asc = 2000;
						else if (asc <= 200)
							asc = 200;
						targetp += asc;
					}
					else //����淽����λ�ü�
					{
						if (distance > 40)
							asc += 2;
						else if (distance < 10)
							asc -= 2;
						if (asc > 2000)
							asc = 2000;
						else if (asc <= 200)
							asc = 200;
						targetp -= asc;
					}

					if (abs(targetp) > 10)
					{
						CMD15_CSP(1, targetp);
						SPI_exchange_polling();
						//						printf("current_Pos1= =%x\n\r", tmp1);
					}
					else
					{
						CMD00_ALL_GET_STATUS();
						SPI_exchange_polling();
						//						printf("current_Pos0= =%x\n\r", rbuff[1].Data1);
					}

					if ((rbuff[0].Data2 & 0xFF) < 20)
					{
						PowerDelay(20000);
					}
				}

				if (dir == 1)
					distance++;
				else
					distance--;
				if (distance >= 50 && dir == 1)
				{
					dir = 0;
					asc = 200;
					printf("current_Pos1(Real)= =%x\n\r", rbuff[1].Data1);
					printf("current_Pos1(targetp)= =%x\n\r", targetp);
				}
				else if (distance <= 0 && dir == 0)
				{
					dir = 1;
					asc = 200;
					printf("current_Pos2(Real)= =%x\n\r", rbuff[1].Data1);
					printf("current_Pos2(targetp)= =%x\n\r", targetp);
				}
			}
			else
			{
				CMD00_ALL_GET_STATUS();
				SPI_exchange_polling();
				//				printf("current_Pos0= =%x\n\r", rbuff[1].Data1);
			}
			time = 0;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	u8 gifr;
	// int rxlen;
	// static unsigned char dat1, dat2, dat3, dat4;
	switch (GPIO_Pin)
	{
	case GPIO_PIN_8:
		gifr = Wk2xxxReadReg(WK2XXX_PORT1, WK2XXX_GIFR); /**/
		if (gifr & WK2XXX_UT1INT)						 //�ж��Ӵ���1�Ƿ����ж�
		{
			/*���ݽ���*/
			WK_Rcv1Counter = wk_RxChars(WK2XXX_PORT1, WK_Rcv1Buffer); //һ�ν��յ����ݲ��ᳬ��256Byte
			C_WK_NoRcv1Count = 0;
		}

		gifr = Wk2xxxReadReg(WK2XXX_PORT2, WK2XXX_GIFR); /**/
		if (gifr & WK2XXX_UT2INT)						 //�ж��Ӵ���1�Ƿ����ж�
		{
			/*���ݽ���*/
			WK_Rcv2Counter = wk_RxChars(WK2XXX_PORT2, WK_Rcv2Buffer); //һ�ν��յ����ݲ��ᳬ��256Byte
			C_WK_NoRcv2Count = 0;
		}

		gifr = Wk2xxxReadReg(WK2XXX_PORT3, WK2XXX_GIFR); /**/
		if (gifr & WK2XXX_UT3INT)						 //�ж��Ӵ���1�Ƿ����ж�
		{
			/*���ݽ���*/
			WK_Rcv3Counter = wk_RxChars(WK2XXX_PORT3, WK_Rcv3Buffer); //һ�ν��յ����ݲ��ᳬ��256Byte
			C_WK_NoRcv3Count = 0;

			// while ((Wk2xxxReadReg(3, WK2XXX_FSR) & WK2XXX_RDAT))
			// {
			// 	dat3 = Wk2xxxReadReg(3, WK2XXX_FDAT);
			// 	//				printf("dat3= 0x%x\r\n",dat3);
			// 	delay_ms(1);
			// 	Wk2xxxWriteReg(3, WK2XXX_FDAT, dat3);
			// }
		}

		gifr = Wk2xxxReadReg(WK2XXX_PORT4, WK2XXX_GIFR); /**/
		if (gifr & WK2XXX_UT4INT)						 //�ж��Ӵ���1�Ƿ����ж�
		{
			/*���ݽ���*/
			WK_Rcv4Counter = wk_RxChars(WK2XXX_PORT4, WK_Rcv4Buffer); //һ�ν��յ����ݲ��ᳬ��256Byte
			C_WK_NoRcv4Count = 0;

			// while ((Wk2xxxReadReg(4, WK2XXX_FSR) & WK2XXX_RDAT)) //�жϽ���fifo���Ƿ������ݣ�ֱ����FIFO����
			// {
			// 	dat4 = Wk2xxxReadReg(4, WK2XXX_FDAT);
			// 	//				printf("dat4= 0x%x\r\n",dat4);
			// 	delay_ms(1);
			// 	Wk2xxxWriteReg(4, WK2XXX_FDAT, dat4);
			// }
		}
		break;
	case GPIO_PIN_1:
		break;
	default:
		break;

		//		gifr=WkReadGReg(WK2XXX_GIFR);
		//			delay_ms(1000);
		//		printf("over\n");
		//		printf("\n\r\n\r");
	}
}

// PWM DMA ��ɻص�����
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1)
	{
		HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	}
	else if (htim == &htim2)
	{
		HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_2);
	}
	//����
}

//����GPIO
void test_GPIO(void)
{
	if (HAL_GPIO_ReadPin(DI1_GPIO_Port, DI1_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI2_GPIO_Port, DI2_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI3_GPIO_Port, DI3_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI4_GPIO_Port, DI4_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO4_GPIO_Port, DO4_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO4_GPIO_Port, DO4_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI5_GPIO_Port, DI5_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO5_GPIO_Port, DO5_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO5_GPIO_Port, DO5_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI6_GPIO_Port, DI6_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO6_GPIO_Port, DO6_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO6_GPIO_Port, DO6_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI7_GPIO_Port, DI7_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO7_GPIO_Port, DO7_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO7_GPIO_Port, DO7_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI8_GPIO_Port, DI8_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO8_GPIO_Port, DO8_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO8_GPIO_Port, DO8_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI9_GPIO_Port, DI9_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO9_GPIO_Port, DO9_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO9_GPIO_Port, DO9_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI10_GPIO_Port, DI10_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO10_GPIO_Port, DO10_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO10_GPIO_Port, DO10_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI11_GPIO_Port, DI11_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO11_GPIO_Port, DO11_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO11_GPIO_Port, DO11_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI12_GPIO_Port, DI12_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO12_GPIO_Port, DO12_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO12_GPIO_Port, DO12_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI13_GPIO_Port, DI13_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO13_GPIO_Port, DO13_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO13_GPIO_Port, DO13_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI14_GPIO_Port, DI14_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO14_GPIO_Port, DO14_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO14_GPIO_Port, DO14_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI15_GPIO_Port, DI15_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO15_GPIO_Port, DO15_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO15_GPIO_Port, DO15_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI16_GPIO_Port, DI16_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO16_GPIO_Port, DO16_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO16_GPIO_Port, DO16_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI17_GPIO_Port, DI17_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO17_GPIO_Port, DO17_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO17_GPIO_Port, DO17_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI18_GPIO_Port, DI18_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO18_GPIO_Port, DO18_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO18_GPIO_Port, DO18_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI19_GPIO_Port, DI19_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO19_GPIO_Port, DO19_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO19_GPIO_Port, DO19_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI20_GPIO_Port, DI20_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO20_GPIO_Port, DO20_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO20_GPIO_Port, DO20_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI21_GPIO_Port, DI21_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO21_GPIO_Port, DO21_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO21_GPIO_Port, DO21_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI22_GPIO_Port, DI22_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO22_GPIO_Port, DO22_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO22_GPIO_Port, DO22_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI23_GPIO_Port, DI23_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(LED_H63_GPIO_Port, LED_H63_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_H63_GPIO_Port, LED_H63_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI24_GPIO_Port, DI24_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(LED_H64_GPIO_Port, LED_H64_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_H64_GPIO_Port, LED_H64_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI25_GPIO_Port, DI25_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(LED_H65_GPIO_Port, LED_H65_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_H65_GPIO_Port, LED_H65_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI26_GPIO_Port, DI26_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(LED_H66_GPIO_Port, LED_H66_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_H66_GPIO_Port, LED_H66_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI27_GPIO_Port, DI27_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(LED_H67_GPIO_Port, LED_H67_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_H67_GPIO_Port, LED_H67_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI28_GPIO_Port, DI28_Pin) == GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(LED_H68_GPIO_Port, LED_H68_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_H69_GPIO_Port, LED_H69_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_H70_GPIO_Port, LED_H70_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_H68_GPIO_Port, LED_H68_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_H69_GPIO_Port, LED_H69_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_H70_GPIO_Port, LED_H70_Pin, GPIO_PIN_SET);
	}
}

//�����жϻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		C_NoRcv1Count = 0;
		if (Rcv1Counter < RCV1_MAX - 2)
		{
			Rcv1Buffer[Rcv1Counter++] = Tmp_Rxd1Buffer;
		}
		//�ٴο�������
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&Tmp_Rxd1Buffer, 1);
	}
	else if (huart->Instance == USART2)
	{
		C_NoRcv2Count = 0;
		if (Rcv2Counter < RCV2_MAX - 2)
		{
			Rcv2Buffer[Rcv2Counter++] = Tmp_Rxd2Buffer;
		}
		//�ٴο�������
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&Tmp_Rxd2Buffer, 1);
	}
	else if (huart->Instance == USART3)
	{
		C_NoRcv3Count = 0;
		if (Rcv3Counter < RCV3_MAX - 2)
		{
			Rcv3Buffer[Rcv3Counter++] = Tmp_Rxd3Buffer;
		}
		//�ٴο�������
		HAL_UART_Receive_IT(&huart3, (uint8_t *)&Tmp_Rxd3Buffer, 1);
	}
	else if (huart->Instance == UART4)
	{
		C_NoRcv4Count = 0;
		if (Rcv4Counter < RCV4_MAX - 2)
		{
			Rcv4Buffer[Rcv4Counter++] = Tmp_Rxd4Buffer;
		}
		//�ٴο�������
		HAL_UART_Receive_IT(&huart4, (uint8_t *)&Tmp_Rxd4Buffer, 1);
	}
	else if (huart->Instance == UART5)
	{
		C_NoRcv5Count = 0;
		if (Rcv5Counter < RCV5_MAX - 2)
		{
			Rcv5Buffer[Rcv5Counter++] = Tmp_Rxd5Buffer;
		}
		//�ٴο�������
		HAL_UART_Receive_IT(&huart5, (uint8_t *)&Tmp_Rxd5Buffer, 1);
	}
	else if (huart->Instance == USART6)
	{
		C_NoRcv6Count = 0;
		if (Rcv6Counter < RCV6_MAX - 2)
		{
			Rcv6Buffer[Rcv6Counter++] = Tmp_Rxd6Buffer;
		}
		//�ٴο�������
		HAL_UART_Receive_IT(&huart6, (uint8_t *)&Tmp_Rxd6Buffer, 1);
	}
}

//�����жϻص�����
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//	if(Txd1Counter >= Txd1Max)
	//    {
	//		Txd1Counter=0;
	//		Txd1Max=0;
	//    }
	//	else
	//	{
	//		/* Write one byte to the transmit data register */
	//		while(huart1.gState != HAL_UART_STATE_READY);
	//		HAL_UART_Transmit_IT(&huart1, (uint8_t *)&Txd1Buffer[Txd1Counter++], 1);
	//	}
}

//����EEPROM-W25Q16
void Test_EEPROM(void)
{
	//wk2xxx��ض���
	// uint8_t gena;
	u8 datatemp[SIZE];
	u32 FLASH_SIZE;
	u8 i;
	// u16 j;
	uint8_t read_buf[10] = {0};
	uint8_t write_buf[10] = {0};
	uint32_t device_id;

	W25qxx_Init();

	while ((W25qxx_ReadID() & 0x0000FFFF) != _W25Q16) //��ⲻ��W25Q16
	{
		printf("W25Q16 Check Failed!");
		delay_ms(500);
		printf("Please Check!      ");
		delay_ms(500);
		HAL_GPIO_TogglePin(LED_H63_GPIO_Port, LED_H63_Pin); //DS0��˸
	}

	printf("W25Q16 Ready!\r\n");
	FLASH_SIZE = 64 * 1024 * 1024;
	printf("Start Write W25Q16....\r\n");
	SPI_FLASH_BufferWrite((u8 *)TEXT_Buffer, FLASH_SIZE - 100, SIZE); //�ӵ�����100����ַ����ʼ,д��SIZE���ȵ�����
	printf("W25Q16 Write Finished!\r\n");							  //��ʾ�������

	printf("Start Read W25Q16.... \r\n");
	W25qxx_ReadBytes(datatemp, FLASH_SIZE - 100, SIZE); //�ӵ�����100����ַ����ʼ,����SIZE���ֽ�
	printf("The Data Readed Is:   \r\n");				//��ʾ�������
	for (i = 0; i < sizeof(datatemp); i++)
		printf(" %d \r\n", datatemp[i]); //��ʾ�������ַ���

	//����һ�β��Զ�дW25Q16�ĳ���
	printf("W25Q64 SPI Flash Test By YLS\r\n");
	device_id = W25qxx_ReadID();
	printf("W25Q16 Device ID is 0x%04x\r\n", device_id);

	/* Ϊ����֤�����ȶ�ȡҪд���ַ�������� */
	printf("-------- read data before write -----------\r\n");
	W25qxx_ReadBytes(read_buf, 0, 10);

	for (i = 0; i < 10; i++)
	{
		printf("[0x%08x]:0x%02x\r\n", i, *(read_buf + i));
	}

	/* ���������� */
	printf("-------- erase sector 0 -----------\r\n");
	W25qxx_EraseSector(0);

	/* �ٴζ����� */
	printf("-------- read data after erase -----------\r\n");
	W25qxx_ReadBytes(read_buf, 0, 10);
	for (i = 0; i < 10; i++)
	{
		printf("[0x%08x]:0x%02x\r\n", i, *(read_buf + i));
	}

	/* д���� */
	printf("-------- write data -----------\r\n");
	for (i = 0; i < 10; i++)
	{
		write_buf[i] = i;
	}
	W25qxx_WriteBytes_Page(write_buf, 0, 10);

	/* �ٴζ����� */
	printf("-------- read data after write -----------\r\n");
	W25qxx_ReadBytes(read_buf, 0, 10);
	for (i = 0; i < 10; i++)
	{
		printf("[0x%08x]:0x%02x\r\n", i, *(read_buf + i));
	}
	printf("-------- read data end! -----------\r\n");
}

//��ʼ����չ����
void Init_WK_Uart(void)
{
	unsigned char gena;
	//    WK2XXX_RST_Init();
	WK2XXX_SPI_Init();
	//	WK2XXX_Reset_Init();
	Wk2xxxInit(WK2XXX_PORT1);
	Wk2xxxInit(WK2XXX_PORT2);
	Wk2xxxInit(WK2XXX_PORT3);
	Wk2xxxInit(WK2XXX_PORT4);
	Wk2xxxSetBaud(WK2XXX_PORT1, B57600);
	Wk2xxxSetBaud(WK2XXX_PORT2, B57600);
	Wk2xxxSetBaud(WK2XXX_PORT3, B57600);
	Wk2xxxSetBaud(WK2XXX_PORT4, B57600);
	////	���Գ���ʱ��ͨ����GENA�Ĵ�����ֵ���ж����ӿ�ͨ���Ƿ�������δ��ʼ����Ĭ��ֵ������Ϊ0x30������0x3F
	gena = Wk2xxxReadReg(WK2XXX_GPORT, WK2XXX_GENA);
	printf("gena=%x\n", gena);
}

//����WK��չ����
void TEST_WK_Uart(void)
{
	unsigned char dat1, dat2, dat3, dat4;

	if ((Wk2xxxReadReg(WK2XXX_PORT1, WK2XXX_FSR) & WK2XXX_RDAT)) //��ȡFIFO״̬���ж��Ƿ�Ϊ��
	{
		dat1 = Wk2xxxReadReg(WK2XXX_PORT1, WK2XXX_FDAT); //��FIFO����
		Wk2xxxWriteReg(WK2XXX_PORT1, WK2XXX_FDAT, dat1); //����ȡFIFO����ͨ��TX���ͳ�ȥ
	}
	else if ((Wk2xxxReadReg(WK2XXX_PORT2, WK2XXX_FSR) & WK2XXX_RDAT)) //��ȡFIFO״̬���ж��Ƿ�Ϊ��
	{
		dat2 = Wk2xxxReadReg(WK2XXX_PORT2, WK2XXX_FDAT); //��FIFO����
		Wk2xxxWriteReg(WK2XXX_PORT2, WK2XXX_FDAT, dat2); //����ȡFIFO����ͨ��TX���ͳ�ȥ
	}
	else if ((Wk2xxxReadReg(WK2XXX_PORT3, WK2XXX_FSR) & WK2XXX_RDAT)) //��ȡFIFO״̬���ж��Ƿ�Ϊ��
	{
		dat3 = Wk2xxxReadReg(WK2XXX_PORT3, WK2XXX_FDAT); //��FIFO����
		Wk2xxxWriteReg(WK2XXX_PORT3, WK2XXX_FDAT, dat3); //����ȡFIFO����ͨ��TX���ͳ�ȥ
	}
	else if ((Wk2xxxReadReg(WK2XXX_PORT4, WK2XXX_FSR) & WK2XXX_RDAT)) //��ȡFIFO״̬���ж��Ƿ�Ϊ��
	{
		dat4 = Wk2xxxReadReg(WK2XXX_PORT4, WK2XXX_FDAT); //��FIFO����
		Wk2xxxWriteReg(WK2XXX_PORT4, WK2XXX_FDAT, dat4); //����ȡFIFO����ͨ��TX���ͳ�ȥ
	}
}

//��ʼ��EtherCAT
void Init_EtherCAT(void)
{
	int i;
	/*ģʽ����*/
	SPI_ECAT_CS_OFF; //SPI-CSƬѡ

	do
	{
		printf("Set to Pre OP\n");
		CMD01_SET_STATE(STATE_PRE_OP); //�����д�վ���ý���PRE_OP״̬
		SPI_exchange_polling();
		HAL_Delay(1000);
		CMD02_SET_AXIS(0, (uint8_t *)bNetworkMap); //�趨��վ���ͣ����õ�һ����վΪDRIVE��վ
		SPI_exchange_polling();
		HAL_Delay(1000);
		CMD06_DRIVE_MODE(1, CSP_MODE, 1);
		SPI_exchange_polling();
		HAL_Delay(1000);			//yan chi da yi xie
		CMD03_SET_DC(8000, 0xffff); //�趨��������250us ,������250us������
		SPI_exchange_polling();
		HAL_Delay(200);

		CMD00_ALL_GET_STATUS();
		SPI_exchange_polling();
		HAL_Delay(100);

		printf("Set to OP\n");
		CMD01_SET_STATE(STATE_OPERATIONAL); //OP״̬
		SPI_exchange_polling();
		HAL_Delay(1000);
		printf("0 rbuff[0].CMD=%d  ", rbuff[0].CMD);
		printf("0 rbuff[0].Parm=%d  \n", rbuff[0].Parm);
	} while ((((rbuff[0].CMD & 0x00FF) != 0x00BF) && ((rbuff[0].Parm & 0x00FF) != 0x0008)));
	printf("rbuff[0].CMD=%d  ", rbuff[0].CMD);
	printf("rbuff[0].Parm=%d  ", rbuff[0].Parm);
	printf("In OP Status\n");

	while (1)
	{
		CMD00_ALL_GET_STATUS();
		SPI_exchange_polling();
		HAL_Delay(100);

		if (((rbuff[0].CMD & 0x00FF) == 0x00BF) && ((rbuff[0].Parm & 0x00FF) == 0x0008))
		{
			i++;
		}
		else
		{
			i = 0;
		}
		printf("rbuff[0].CMD=%d  ", rbuff[0].CMD);
		printf("rbuff[0].Parm=%d  ", rbuff[0].Parm);

		printf("Now is OP Status,%d  \n", i);
		if (i >= 100)
		{
			break;
		}
	}

	CMD11_SVON(1);
	SPI_exchange_polling();
	HAL_Delay(1000);
	//	while(abs(rbuff[1].Data1)<=1000)	//(rbuff[0].CMD&0x00FF)!=0x00BF &&(rbuff[0].Parm&0x00FF)!=0x0008 &&
	//	{
	//		CMD00_ALL_GET_STATUS();
	//		SPI_exchange_polling();
	////		HAL_Delay(1000);
	//	}
	targetp = rbuff[1].Data1;
	startflag = 1;
	dir = 1;
	distance = 0;
}

void UART_TX(void)
{
	Com1_RcvProcess(); // ����1���մ���
	Com1_SlaveSend();  // ����1�ӻ�����

	Com2_RcvProcess(); // ����2���մ���
	Com2_SlaveSend();  // ����2�ӻ�����

	Com3_RcvProcess(); // ����3���մ���
	Com3_SlaveSend();  // ����3�ӻ�����

	Com4_RcvProcess(); // ����4���մ���
	Com4_SlaveSend();  // ����4�ӻ�����

	Com5_RcvProcess(); // ����5���մ���
	Com5_SlaveSend();  // ����5�ӻ�����

	Com6_RcvProcess(); // ����6���մ���
	Com6_SlaveSend();  // ����6�ӻ�����
}

//���Է�PWM��
void TEST_Send_Pwm(void)
{
	if (Pw_Driver1_Enable == 1)
	{
		Pw_Driver1_Enable = 0;
		HAL_Delay(100);
		//��������
		Run_Motor_S(1, M1_CLOCKWISE, (Pw_Driver1_Pluse_HW << 16) + Pw_Driver1_Pluse, Pw_Driver1_Speed, Pw_Driver1_AccTime);
		Run_Motor_S(2, M2_CLOCKWISE, (Pw_Driver2_Pluse_HW << 16) + Pw_Driver2_Pluse, Pw_Driver2_Speed, Pw_Driver2_AccTime);
		Run_Motor_S(3, M3_CLOCKWISE, (Pw_Driver3_Pluse_HW << 16) + Pw_Driver3_Pluse, Pw_Driver3_Speed, Pw_Driver3_AccTime);
		Run_Motor_S(4, M4_CLOCKWISE, (Pw_Driver4_Pluse_HW << 16) + Pw_Driver4_Pluse, Pw_Driver4_Speed, Pw_Driver4_AccTime);
		Run_Motor_S(5, M5_CLOCKWISE, (Pw_Driver5_Pluse_HW << 16) + Pw_Driver5_Pluse, Pw_Driver5_Speed, Pw_Driver5_AccTime);
		Run_Motor_S(6, M6_CLOCKWISE, (Pw_Driver6_Pluse_HW << 16) + Pw_Driver6_Pluse, Pw_Driver6_Speed, Pw_Driver6_AccTime);
	}
}

//��ʼ�������PWM
void Initial_PWM_Motors(void)
{
	Initial_PWM_Motor1(); //��ʼ�����1��PWM
	Initial_PWM_Motor2(); //��ʼ�����2��PWM
	Initial_PWM_Motor3(); //��ʼ�����3��PWM
	Initial_PWM_Motor4(); //��ʼ�����4��PWM
	Initial_PWM_Motor5(); //��ʼ�����5��PWM
	Initial_PWM_Motor6(); //��ʼ�����6��PWM
}

//��дEtherCAT��IO��
void Rd2Wr_IO(void)
{
	CMD13_IORD(1);
	SPI_exchange_polling();

	HAL_Delay(100);
	// for (i = 1; i < DEF_MA_MAX - 1; i++) //��ȡIO����״̬
	// {
	if (bNetworkMap[0] == IO)
	{
		IO_Input = rbuff[1].Data1;
#ifndef Printf_ON
		printf("IO_Input =%x\n\r", IO_Input);
#endif
	}
	// }

	/*������վIO���*/
	//		for(i=1;i< DEF_MA_MAX-1;i++){
	//						CMD14_IOWR(i,0x55555555);
	CMD14_IOWR(1, IO_Input);
	//			CMD14_IOWR(i,IO_Input);
	//		}
	SPI_exchange_polling();

	HAL_Delay(100);
	CMD00_GET_STATUS(1);
	SPI_exchange_polling();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
