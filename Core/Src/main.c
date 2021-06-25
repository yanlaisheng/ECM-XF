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
#include "GlobalV_Extern.h" // 全局变量声明
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
int32_t distance; //运行距离
uint8_t dir = 0;  //方向
uint16_t asc;	  //加速计数

extern uint32_t IO_Input;

struct rbuff_st
{
	uint16_t CMD;
	uint16_t Parm;
	int32_t Data1;
	int32_t Data2;
}; /* Receive Buffer */
extern struct rbuff_st rbuff[];

//要写入到W25Q16的字符串数组
const u8 TEXT_Buffer[] = {"Explorer STM32F4 SPI TEST"};
#define SIZE sizeof(TEXT_Buffer)

int main_ini(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//重定向printf和scanf：修改fgetc和fputc
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */
	u32 cyc_counter = 0;
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	HAL_Delay(1000); //延时
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
	InitClock();		  // 初始化外部时钟DS1302
	Init_WK_Uart();		  //初始化WK2124扩展串口
	Initial_PWM_Motors(); //初始化电机的PWM
	// Test_EEPROM();		  //测试EEPROM

	//EtherCAT初始化
	// Init_EtherCAT();
	//	main_ini();

	ParLst_Init();	 // RAM中设定参数表列初始化，FLASH——>RAM
	Boot_ParLst();	 // 初始化STM32设定参数
	Variable_Init(); // Initialize VARIABLE
	ParLimit();		 // 参数限制

	Send_Driver_CMD(Pw_EquipmentNo3, 06, 0x8910, 0); //写驱动器3的P8910参数=0，RDY状态
	Send_Driver_CMD(Pw_EquipmentNo3, 06, 0x0029, 1); //写驱动器3的P0029参数=1，使用外部制动电阻
	Send_StopCMD();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		Boot_ParLst();			   //初始化设定参数，命令：Pw_ModPar = 2000，Pw_ParInitial = 4321
		PosCMD_SaveTo_exFLASH();   //将RAM中的位置命令到外部FLASH和FRAM，命令：Pw_ModPar == 2000，Pw_ParInitial == 8000
		PosCMD_ReadFrom_exFLASH(); //从外部FLASH按组号读取位置命令到RAM和FRAM，命令：Pw_ModPar = 2000，Pw_ParInitial = 4000
		ParLst_Init_Group2Zero();  //清零位置命令，命令：Pw_ModPar == 2000 && Pw_ParInitial == 9876
		ParLst_Init_Pos2Zero();	   //清零位置，命令：Pw_ModPar == 2000 && Pw_ParInitial == 5432
		Pos_Manual_Adj();		   //手动调整编码器位置
		ParLimit();				   // 参数限制
								   //		ReadWriteRealTime(); // 读写实时时钟 ISL1208

		EquipStatus();	  // 设备状态
		KglStatus();	  // 开关量状态
		DigitalIn();	  // DI
		Manual_Control(); // 手动控制启停

		SavePar_Prompt();	 // 定时保存设定参数到FLASH"+"状态提示
		ForceTime_SavePar(); // 定时强制保存参数

		Time_Output(); // 软件时钟输出

		UART_TX(); //串口通讯

		Driver_Control();	  //电机运行控制
		Reset_Drivers();	  //有复位命令，则复位到原点
		BRAKE_Control();	  //刹车控制
		is_Reseted();		  //判断是否在原点
		Cal_Pos_Speed();	  //计算脉冲值和速度
		Record_Current_Pos(); //记录当前位置
		Control_Hand();		  //机械手控制
		//  Limit_Max_Pos();	  //限制最大最小位置范围
		// IWDG_Feed();								// 喂狗	2013.7.3

		cyc_counter++;
		Pr_cyclecounter_show = cyc_counter & 0x0000FFFF;
		Pr_cyclecounter_HW = (cyc_counter & 0xFFFF0000) >> 16;
		TEST_Send_Pwm();
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
//延时1us
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
					if (dir == 1) //如果正方向，则位置加
					{
						if (distance < 10) //进行加减速设置
							asc += 2;
						else if (distance > 40)
							asc -= 2;
						if (asc > 2000)
							asc = 2000;
						else if (asc <= 200)
							asc = 200;
						targetp += asc;
					}
					else //如果逆方向，则位置减
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

	switch (GPIO_Pin)
	{
	case GPIO_PIN_8:
		gifr = Wk2xxxReadReg(WK2XXX_PORT1, WK2XXX_GIFR); /**/
		if (gifr & WK2XXX_UT1INT)						 //判断子串口1是否有中断
		{
			/*数据接收*/
			WK_Rcv1Counter = wk_RxChars(WK2XXX_PORT1, WK_Rcv1Buffer); //一次接收的数据不会超过256Byte
			C_WK_NoRcv1Count = 0;
		}

		gifr = Wk2xxxReadReg(WK2XXX_PORT2, WK2XXX_GIFR); /**/
		if (gifr & WK2XXX_UT2INT)						 //判断子串口1是否有中断
		{
			/*数据接收*/
			WK_Rcv2Counter = wk_RxChars(WK2XXX_PORT2, WK_Rcv2Buffer); //一次接收的数据不会超过256Byte
			C_WK_NoRcv2Count = 0;
		}

		gifr = Wk2xxxReadReg(WK2XXX_PORT3, WK2XXX_GIFR); /**/
		if (gifr & WK2XXX_UT3INT)						 //判断子串口1是否有中断
		{
			/*数据接收*/
			WK_Rcv3Counter = wk_RxChars(WK2XXX_PORT3, WK_Rcv3Buffer); //一次接收的数据不会超过256Byte
			C_WK_NoRcv3Count = 0;
		}

		gifr = Wk2xxxReadReg(WK2XXX_PORT4, WK2XXX_GIFR); /**/
		if (gifr & WK2XXX_UT4INT)						 //判断子串口1是否有中断
		{
			/*数据接收*/
			WK_Rcv4Counter = wk_RxChars(WK2XXX_PORT4, WK_Rcv4Buffer); //一次接收的数据不会超过256Byte
			C_WK_NoRcv4Count = 0;
		}
		break;

	default:
		break;
	}
}

//接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		C_NoRcv1Count = 0;
		if (Rcv1Counter < RCV1_MAX - 2)
		{
			Rcv1Buffer[Rcv1Counter++] = Tmp_Rxd1Buffer;
		}
		//再次开启接收
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&Tmp_Rxd1Buffer, 1);
	}
	else if (huart->Instance == USART2)
	{
		C_NoRcv2Count = 0;
		if (Rcv2Counter < RCV2_MAX - 2)
		{
			Rcv2Buffer[Rcv2Counter++] = Tmp_Rxd2Buffer;
		}
		//再次开启接收
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&Tmp_Rxd2Buffer, 1);
	}
	else if (huart->Instance == USART3)
	{
		C_NoRcv3Count = 0;
		if (Rcv3Counter < RCV3_MAX - 2)
		{
			Rcv3Buffer[Rcv3Counter++] = Tmp_Rxd3Buffer;
		}
		//再次开启接收
		HAL_UART_Receive_IT(&huart3, (uint8_t *)&Tmp_Rxd3Buffer, 1);
	}
	else if (huart->Instance == UART4)
	{
		C_NoRcv4Count = 0;
		if (Rcv4Counter < RCV4_MAX - 2)
		{
			Rcv4Buffer[Rcv4Counter++] = Tmp_Rxd4Buffer;
		}
		//再次开启接收
		HAL_UART_Receive_IT(&huart4, (uint8_t *)&Tmp_Rxd4Buffer, 1);
	}
	else if (huart->Instance == UART5)
	{
		C_NoRcv5Count = 0;
		if (Rcv5Counter < RCV5_MAX - 2)
		{
			Rcv5Buffer[Rcv5Counter++] = Tmp_Rxd5Buffer;
		}
		//再次开启接收
		HAL_UART_Receive_IT(&huart5, (uint8_t *)&Tmp_Rxd5Buffer, 1);
	}
	else if (huart->Instance == USART6)
	{
		C_NoRcv6Count = 0;
		if (Rcv6Counter < RCV6_MAX - 2)
		{
			Rcv6Buffer[Rcv6Counter++] = Tmp_Rxd6Buffer;
		}
		//再次开启接收
		HAL_UART_Receive_IT(&huart6, (uint8_t *)&Tmp_Rxd6Buffer, 1);
	}
}

//测试EEPROM-W25Q16
void Test_EEPROM(void)
{
	//wk2xxx相关定义
	// uint8_t gena;
	u8 datatemp[SIZE];
	u32 FLASH_SIZE;
	u8 i;
	// u16 j;
	uint8_t read_buf[10] = {0};
	uint8_t write_buf[10] = {0};
	uint32_t device_id;
	B_SelCS = CS_Flash1;

	W25qxx_Init();

	while ((W25qxx_ReadID() & 0x0000FFFF) != _W25Q16) //检测不到W25Q16
	{
		printf("W25Q16 Check Failed!");
		delay_ms(500);
		printf("Please Check!      ");
		delay_ms(500);
	}

	printf("W25Q16 Ready!\r\n");
	FLASH_SIZE = 2 * 1024 * 1024; //W25Q16，容量2MB
	printf("Start Write W25Q16....——1\r\n");
	SPI_FLASH_BufferWrite((u8 *)TEXT_Buffer, FLASH_SIZE - 100, SIZE); //从倒数第100个地址处开始,写入SIZE长度的数据
	printf("W25Q16 Write Finished!——1\r\n");						  //提示传送完成

	printf("Start Write W25Q16....——2\r\n");
	W25QXX_Write_WithErase((u8 *)TEXT_Buffer, 5, SIZE);
	printf("W25Q16 Write Finished!——2\r\n"); //提示传送完成

	printf("Start Read W25Q16.... \r\n");
	W25qxx_ReadBytes(datatemp, 5, SIZE);  //从倒数第100个地址处开始,读出SIZE个字节
	printf("The Data Readed Is:   \r\n"); //提示传送完成
	for (i = 0; i < sizeof(datatemp); i++)
		printf(" %d \r\n", datatemp[i]); //显示读到的字符串

	//另外一段测试读写W25Q16的程序
	printf("W25Q64 SPI Flash Test By YLS\r\n");
	device_id = W25qxx_ReadID();
	printf("W25Q16 Device ID is 0x%04x\r\n", device_id);

	/* 为了验证，首先读取要写入地址处的数据 */
	printf("-------- read data before write -----------\r\n");
	W25qxx_ReadBytes(read_buf, 0, 10);

	for (i = 0; i < 10; i++)
	{
		printf("[0x%08x]:0x%02x\r\n", i, *(read_buf + i));
	}

	/* 擦除该扇区 */
	printf("-------- erase sector 0 -----------\r\n");
	W25qxx_EraseSector(0);

	/* 再次读数据 */
	printf("-------- read data after erase -----------\r\n");
	W25qxx_ReadBytes(read_buf, 0, 10);
	for (i = 0; i < 10; i++)
	{
		printf("[0x%08x]:0x%02x\r\n", i, *(read_buf + i));
	}

	/* 写数据 */
	printf("-------- write data -----------\r\n");
	for (i = 0; i < 10; i++)
	{
		write_buf[i] = i;
	}
	W25qxx_WriteBytes_Page(write_buf, 0, 10);

	/* 再次读数据 */
	printf("-------- read data after write -----------\r\n");
	W25qxx_ReadBytes(read_buf, 0, 10);
	for (i = 0; i < 10; i++)
	{
		printf("[0x%08x]:0x%02x\r\n", i, *(read_buf + i));
	}
	printf("-------- read data end! -----------\r\n");

	/* 写FRAM数据 */
	printf("-------- write data to FRAM start-----------\r\n");
	// B_SelCS = CS_FMRAM1;
	SPI_FMRAM_BufferWrite((u8 *)TEXT_Buffer, 0, 10);
	printf("-------- write data to FRAM end-----------\r\n");

	/* 读FRAM数据 */
	printf("-------- read data from FRAM start-----------\r\n");
	SPI_FMRAM_BufferRead(read_buf, 0, 10);
	for (i = 0; i < 10; i++)
	{
		printf("[0x%08x]:0x%02x\r\n", i, *(read_buf + i));
	}
	printf("-------- read data from FRAM end-----------\r\n");
}

//初始化扩展串口
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
	////	调试程序时，通过读GENA寄存器的值，判断主接口通信是否正常，未初始化，默认值读出来为0x30，正常0x3F
	gena = Wk2xxxReadReg(WK2XXX_GPORT, WK2XXX_GENA);
	printf("gena=%x\n", gena);
}

//初始化EtherCAT
void Init_EtherCAT(void)
{
	int i;
	/*模式配置*/
	SPI_ECAT_CS_OFF; //SPI-CS片选

	do
	{
		printf("Set to Pre OP\n");
		CMD01_SET_STATE(STATE_PRE_OP); //将所有从站设置进入PRE_OP状态
		SPI_exchange_polling();
		HAL_Delay(1000);
		CMD02_SET_AXIS(0, (uint8_t *)bNetworkMap); //设定从站类型，配置第一个从站为DRIVE子站
		SPI_exchange_polling();
		HAL_Delay(1000);
		CMD06_DRIVE_MODE(1, CSP_MODE, 1);
		SPI_exchange_polling();
		HAL_Delay(1000);			//yan chi da yi xie
		CMD03_SET_DC(8000, 0xffff); //设定周期数据250us ,必须是250us整数倍
		SPI_exchange_polling();
		HAL_Delay(200);

		CMD00_ALL_GET_STATUS();
		SPI_exchange_polling();
		HAL_Delay(100);

		printf("Set to OP\n");
		CMD01_SET_STATE(STATE_OPERATIONAL); //OP状态
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
	Com1_RcvProcess(); // 串口1接收处理
	Com1_SlaveSend();  // 串口1从机发送

	Com2_RcvProcess(); // 串口2接收处理
	Com2_SlaveSend();  // 串口2从机发送

	Com3_RcvProcess(); // 串口3接收处理
	Com3_SlaveSend();  // 串口3从机发送

	Com4_RcvProcess(); // 串口4接收处理
	Com4_SlaveSend();  // 串口4从机发送

	Com5_RcvProcess(); // 串口5接收处理
	Com5_SlaveSend();  // 串口5从机发送

	Com6_RcvProcess(); // 串口6接收处理
	Com6_SlaveSend();  // 串口6从机发送

	WK_Com1_RcvProcess(); //WK2124扩展串口1接收处理
	WK_Com1_SlaveSend();  //WK2124扩展串口1发送处理

	WK_Com2_RcvProcess();
	WK_Com2_SlaveSend();

	WK_Com3_RcvProcess();
	WK_Com3_SlaveSend();

	WK_Com4_RcvProcess();
	WK_Com4_SlaveSend();
}

//测试发PWM波
void TEST_Send_Pwm(void)
{
	if (!Old_K_SecondPonit)
	{
		if (K_SecondPonit || Pw_Driver1_Enable) //注意： 闭合时，运行。断开时，停止运行。
		{
			Pw_Driver1_Enable = 0;
			//停止后才能继续发送
			if (motor1.running == 0 && motor2.running == 0 && motor3.running == 0 &&
				motor4.running == 0 && motor5.running == 0 && motor6.running == 0)
			{
				HAL_Delay(100);
				//发送脉冲
				Run_Motors_sync(M1_CLOCKWISE, (Pw_Motor1SendPulse_HW << 16) + Pw_Motor1SendPulse, Pw_Motor1_StartSpeed, Pw_Motor1_SetSpeed, Pw_Motor1_ACCSpeed,
								M2_CLOCKWISE, (Pw_Motor2SendPulse_HW << 16) + Pw_Motor2SendPulse, Pw_Motor2_StartSpeed, Pw_Motor2_SetSpeed, Pw_Motor2_ACCSpeed,
								M3_CLOCKWISE, (Pw_Motor3SendPulse_HW << 16) + Pw_Motor3SendPulse, Pw_Motor3_StartSpeed, Pw_Motor3_SetSpeed, Pw_Motor3_ACCSpeed,
								M4_CLOCKWISE, (Pw_Motor4SendPulse_HW << 16) + Pw_Motor4SendPulse, Pw_Motor4_StartSpeed, Pw_Motor4_SetSpeed, Pw_Motor4_ACCSpeed,
								M5_CLOCKWISE, (Pw_Motor5SendPulse_HW << 16) + Pw_Motor5SendPulse, Pw_Motor5_StartSpeed, Pw_Motor5_SetSpeed, Pw_Motor5_ACCSpeed,
								M6_CLOCKWISE, (Pw_Motor6SendPulse_HW << 16) + Pw_Motor6SendPulse, Pw_Motor6_StartSpeed, Pw_Motor6_SetSpeed, Pw_Motor6_ACCSpeed);

				// Run_Motors_sync(M1_CLOCKWISE, (Pw_Motor1SendPulse_HW << 16) + Pw_Motor1SendPulse, Pw_Motor1_StartSpeed, Pw_Motor1_SetSpeed, Pw_Motor1_ACCSpeed,
				// 				M2_CLOCKWISE, (Pw_Motor2SendPulse_HW << 16) + Pw_Motor2SendPulse, Pw_Motor2_StartSpeed, Pw_Motor2_SetSpeed, Pw_Motor2_ACCSpeed,
				// 				M3_CLOCKWISE, (Pw_Motor3SendPulse_HW << 16) + Pw_Motor3SendPulse, Pw_Motor3_StartSpeed, Pw_Motor3_SetSpeed, Pw_Motor3_ACCSpeed,
				// 				0, 0, 0, 0, 0,
				// 				M5_CLOCKWISE, (Pw_Motor5SendPulse_HW << 16) + Pw_Motor5SendPulse, Pw_Motor5_StartSpeed, Pw_Motor5_SetSpeed, Pw_Motor5_ACCSpeed,
				// 				0, 0, 0, 0, 0);

				// Start_Motor12(M1_CLOCKWISE, 5000, M2_CLOCKWISE, 10000);
				//			Run_Motor_S(1, M1_CLOCKWISE, (Pw_Motor1SendPulse_HW << 16) + Pw_Motor1SendPulse, Pw_Motor1_StartSpeed, Pw_Motor1_SetSpeed, Pw_Motor1_ACCSpeed);
				//			Run_Motor_S(2, M2_CLOCKWISE, (Pw_Motor2SendPulse_HW << 16) + Pw_Motor2SendPulse, Pw_Motor2_StartSpeed, Pw_Motor2_SetSpeed, Pw_Motor2_ACCSpeed);
				//			Run_Motor_S(3, M3_CLOCKWISE, (Pw_Motor3SendPulse_HW << 16) + Pw_Motor3SendPulse, Pw_Motor3_StartSpeed, Pw_Motor3_SetSpeed, Pw_Motor3_ACCSpeed);
				//			Run_Motor_S(4, M4_CLOCKWISE, (Pw_Motor4SendPulse_HW << 16) + Pw_Motor4SendPulse, Pw_Motor4_StartSpeed, Pw_Motor4_SetSpeed, Pw_Motor4_ACCSpeed);
				//			Run_Motor_S(5, M5_CLOCKWISE, (Pw_Motor5SendPulse_HW << 16) + Pw_Motor5SendPulse, Pw_Motor5_StartSpeed, Pw_Motor5_SetSpeed, Pw_Motor5_ACCSpeed);
				//			Run_Motor_S(6, M6_CLOCKWISE, (Pw_Motor6SendPulse_HW << 16) + Pw_Motor6SendPulse, Pw_Motor6_StartSpeed, Pw_Motor6_SetSpeed, Pw_Motor6_ACCSpeed);
			}
		}
	}
	Old_K_SecondPonit = K_SecondPonit;
}

//初始化电机的PWM
void Initial_PWM_Motors(void)
{
	Initial_PWM_Motor1(); //初始化电机1的PWM
	Initial_PWM_Motor2(); //初始化电机2的PWM
	Initial_PWM_Motor3(); //初始化电机3的PWM
	Initial_PWM_Motor4(); //初始化电机4的PWM
	Initial_PWM_Motor5(); //初始化电机5的PWM
	Initial_PWM_Motor6(); //初始化电机6的PWM
}

//读写EtherCAT的IO口
void Rd2Wr_IO(void)
{
	CMD13_IORD(1);
	SPI_exchange_polling();

	HAL_Delay(100);
	// for (i = 1; i < DEF_MA_MAX - 1; i++) //读取IO输入状态
	// {
	if (bNetworkMap[0] == IO)
	{
		IO_Input = rbuff[1].Data1;
#ifndef Printf_ON
		printf("IO_Input =%x\n\r", IO_Input);
#endif
	}
	// }

	/*控制子站IO输出*/
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

int userGetchar(void)
{
	uint8_t charbuf;
	HAL_StatusTypeDef ret;
	ret = HAL_UART_Receive(&huart3, &charbuf, 1, 0xffffffff);
	if (ret)
		return -ret;
	return charbuf;
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
