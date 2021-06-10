
#include "GlobalConst.h"
#include "GlobalV_Extern.h" // ȫ�ֱ�������
#include "ds1302.h"
#include "stm32f407xx.h"
// #include "core_cmInstr.h" //Keil library (is used for _nop()_ operation)

#define NOP() __NOP

#define DS1302_CLK_H() (GPIOG->BSRR = GPIO_PIN_6) /* DS1302_CLK */
#define DS1302_CLK_L() (GPIOG->BSRR = (GPIO_PIN_6 << 16U))

#define DS1302_RST_H() (GPIOG->BSRR = GPIO_PIN_8) /* DS1302_RST */
#define DS1302_RST_L() (GPIOG->BSRR = (GPIO_PIN_8 << 16U))

#define DS1302_OUT_H() (GPIOG->BSRR = GPIO_PIN_7) /* DS1302_DATA */
#define DS1302_OUT_L() (GPIOG->BSRR = (GPIO_PIN_7 << 16U))

#define DS1302_IN_X (GPIOG->IDR & GPIO_PIN_7)

#define Time_24_Hour 0x00 //24ʱ�ƿ���
#define Time_Start 0x00	  //��ʼ��ʱ

#define DS1302_SECOND 0x80 //DS1302���Ĵ������������
#define DS1302_MINUTE 0x82
#define DS1302_HOUR 0x84
#define DS1302_DAY 0x86
#define DS1302_MONTH 0x88
#define DS1302_WEEK 0x8A
#define DS1302_YEAR 0x8C
#define DS1302_WRITE 0x8E
#define DS1302_POWER 0x90

/* DS1302 RTC 
READ WRITE BIT 7 BIT 6 BIT 5 BIT 4 BIT 3 BIT 2 BIT 1 BIT 0 RANGE 
81h 	80h CH 10 Seconds Seconds 00�C59 
83h 	82h  10 Minutes Minutes 00�C59 
85h 	84h  12/24  0 10 - AM/PM Hour Hour 1�C12/0�C23 
87h 	86h 0 0 10 Date Date 1�C31 
89h 	88h 0 0 0 10 - Month  Month 1�C12 
8Bh 	8Ah 0 0 0 0 0 Day 1�C7 
8Dh 	8Ch 10 Year Year 00�C99 
8Fh 	8Eh WP 0 0 0 0 0 0 0 �� 
91h 	90h TCS TCS TCS TCS DS DS RS RS �� 

RAM 
C1h C0h  00-FFh 
C3h C2h  00-FFh 
C5h C4h  00-FFh 
. 
. 
FDh FCh  00-FFh 

CLOCK BURST ��BFh BEh 
RAM BURST��		FFh FEh  

Address/Command Byte :
	7		6				5		4		3		2		1		0
	1 RAM/-CK  	A4 	A3 	A2 	A1 	A0  RD/-WR
	
Bit7 Bit6 Bit5 Bit4 Bit3 Bit2 Bit1 Bit0
TCS3 TCS2 TCS1 TCS0 DS1 	DS0 ROUT1 ROUT0
---
TCS3 TCS2 TCS1 TCS0	:	1 0F 16 SELECT;  NOTE: ONLY 1010b ENABLES CHARGER
DS1 	DS0 					:	1 OF 2 SELECT
ROUT1 ROUT0					: 1 OF 3 SELECT
	
	
-----------------------------------------------����
DS1302�� ���ʱ��������
ISL1208�����ʱ��������

DS1302�� 02��Ԫ(85H  84H) Сʱ .7(12/-24 ) 	=1 12Сʱ�ƣ�=0 24Сʱ��
ISL1208��02��Ԫ Сʱ .7(MIL ) 							=1 24Сʱ�ƣ�=0 12Сʱ�� !

DS1302�� CLOCK 80H ;  RAM  C0H;
				д:ĩβΪ0;		��:ĩβΪ1
	
*/

void IWDG_Feed(void);

void DS1302_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //����GPIOCʱ��
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pins : PGPin PGPin */
	GPIO_InitStruct.Pin = DS1302_CLK_Pin | DS1302_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	//DATA������Ϊ˫���,�����ǿ�©����!!!
	//����Ϊ��©ģʽ����ģʽ�¿���ʵ�����µ�˫��IO
	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = DS1302_DATA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DS1302_DATA_GPIO_Port, &GPIO_InitStruct);
}

void DelayNOP(u32 count)
{
	while (count--)
	{
		// IWDG_Feed();
		// IWDG_Feed();
		//IAR ��
		asm("NOP");
	}
	//MDK ��
	//NOP();
}

void DS1302SendByte(u8 byte)
{
	u8 i;

	for (i = 0x01; i; i <<= 1) //i= 0000 0001 ������
	{
		DS1302_CLK_L();
		DelayNOP(10); //����ʱ

		if (byte & i)
			DS1302_OUT_H();
		else
			DS1302_OUT_L();

		DelayNOP(20); //����ʱ

		DS1302_CLK_H();
		DelayNOP(20); //����ʱ
	}
}

u8 DS1302ReceiveByte(void)
{
	u8 i, byte = 0;

	for (i = 0x01; i; i <<= 1) //i= 0000 0001 ������
	{
		DS1302_CLK_H();
		DelayNOP(20); //����ʱ

		DS1302_CLK_L();
		DelayNOP(20); //����ʱ

		if (DS1302_IN_X)
			byte |= i;
	}
	return (byte);
}

u8 Read1302(u8 add)
{
	u8 data = 0;

	DS1302_RST_L();
	DS1302_CLK_L();
	DelayNOP(5); //ZCL 2015.9.6

	DS1302_RST_H();
	DelayNOP(10);

	DS1302SendByte(add | 0x01); //����1��д��0
	data = DS1302ReceiveByte();
	DelayNOP(20);
	DS1302_RST_L();
	return (data);
}

void Write1302(u8 add, u8 data)
{
	DS1302_RST_L();
	DS1302_CLK_L();
	DelayNOP(5);

	DS1302_RST_H();
	DelayNOP(10);

	DS1302SendByte(add);
	DS1302SendByte(data);
	DelayNOP(20);
	DS1302_RST_L();
}

//��ȡʱ�亯��
void Read1302Time(u8 *time)
{
	//u8 tmp;

	time[0] = Read1302(DS1302_SECOND);
	time[1] = Read1302(DS1302_MINUTE);
	time[2] = Read1302(DS1302_HOUR);
	time[3] = Read1302(DS1302_DAY);
	time[4] = Read1302(DS1302_MONTH);
	time[5] = Read1302(DS1302_WEEK);
	time[6] = Read1302(DS1302_YEAR);
}

/*
Byte дʱ�亯��
*/
void Write1302ByteTime(u8 add, u8 data)
{
	Write1302(DS1302_WRITE, 0x00); //�ر�д����
	add = (add << 1) | 0x80;
	Write1302(add, data);
	Write1302(DS1302_WRITE, 0x80); //WP=1 ��ֹд����д������
}

/*
Burst ��ʱ�亯��,˳��Ϊ:��������ʱ����
*/
void BurstRead1302Clock(u8 *p)
{
	DS1302_RST_L();
	DS1302_CLK_L();
	DelayNOP(5);

	DS1302_RST_H();
	DelayNOP(10);

	DS1302SendByte(0xbf);			   //ͻ��ģʽ
	p[0] = DS1302ReceiveByte() & 0x7F; //��
	p[1] = DS1302ReceiveByte() & 0x7F; //��
	p[2] = DS1302ReceiveByte() & 0x7F; //ʱ
	p[3] = DS1302ReceiveByte() & 0x7F; //��
	p[4] = DS1302ReceiveByte() & 0x7F; //��
	p[5] = DS1302ReceiveByte() & 0x7F; //��
	p[6] = DS1302ReceiveByte() & 0x7F; //��
	DS1302ReceiveByte();			   //������־�ֽ�
	DelayNOP(20);
	DS1302_RST_L();
}

/*
Burst дʱ�亯��,˳��Ϊ:��������ʱ����
*/
void BurstWrite1302Clock(u8 *p)
{
	Write1302(DS1302_WRITE, 0x00); //�ر�д����
	DS1302_RST_L();
	DS1302_CLK_L();
	DelayNOP(5);

	DS1302_RST_H();
	DelayNOP(10);
	DS1302SendByte(0xbe); //ͻ��ģʽ
	DS1302SendByte(p[0]); //��
	DS1302SendByte(p[1]); //��
	DS1302SendByte(p[2]); //ʱ
	DS1302SendByte(p[3]); //��
	DS1302SendByte(p[4]); //��
	DS1302SendByte(p[5]); //�ܣ����ó���һ��û��ʹ��
	DS1302SendByte(p[6]); //��
	DS1302SendByte(0x80); //������־�ֽ�
	DelayNOP(20);
	DS1302_RST_L();
}

/*
Burst ��ȡDS1302�е�RAM
add:��ַ,��0��30,��31���ֽڵĿռ�
*p :��������ȡ������
*/
void BurstRead1302Ram(u8 *p)
{
	u8 i;

	DS1302_RST_L();
	DS1302_CLK_L();
	DelayNOP(5);

	DS1302_RST_H();
	DelayNOP(10);
	DS1302SendByte(0xff); //ͻ��ģʽ

	for (i = 0; i < 31; i++)
	{
		*p++ = DS1302ReceiveByte();
	}
	DelayNOP(20);
	DS1302_RST_L();
}

/*
Burst дDS1302�е�RAM
add:��ַ,��0��30,��31���ֽڵĿռ�
*p:Ҫд������
*/
void BurstWrite1302Ram(u8 *p)
{
	u8 i;
	Write1302(DS1302_WRITE, 0x00); //�ر�д����
	DS1302_RST_L();
	DS1302_CLK_L();
	DelayNOP(5);

	DS1302_RST_H();
	DelayNOP(20);
	DS1302SendByte(0xfe); //ͻ��ģʽ
	for (i = 0; i < 31; i++)
	{
		DS1302SendByte(*p++);
	}
	DelayNOP(20);
	DS1302_RST_L();

	Write1302(DS1302_WRITE, 0x80); //��д����
}

/*
��ȡDS1302�е�RAM
add:��ַ,��0��30,��31���ֽڵĿռ�
*p :��������ȡ������
*/
void Read1302Ram(u8 *p, u8 add, u8 cnt)
{
	u8 i, tmp;

	if (cnt > 30)
		return;
	for (i = 0; i < cnt; i++)
	{
		tmp = add + i;
		tmp = (tmp << 1) | 0xc0; //�ܳ��� ����Read1302�� ��ַ | 1����ɣ�0xc1��  (add<<1)ԭ������д��
		*p = Read1302(tmp);
		p++;
	}
}

/*
дDS1302�е�RAM
add:��ַ,��0��30,��31���ֽڵĿռ�
*p:Ҫд������
*/
// д��ĩβΪ0����C0---FC ż��
void Write1302Ram(u8 *p, u8 add, u8 cnt)
{
	u8 i, tmp;

	if (cnt > 30)
		return;
	for (i = 0; i < cnt; i++)
	{
		Write1302(DS1302_WRITE, 0x00); //WP=0 ����д���ر�д������
		tmp = add + i;
		tmp = (tmp << 1) | 0xc0; //�ܳ��� д��ʱ��0xc0    (add<<1)ԭ������д��
		Write1302(tmp, *p++);
		Write1302(DS1302_WRITE, 0x80); //WP=1 ��ֹд����д������
	}
}

void InitClock(void)
{
	/*	u8	RWBuf[7];		//I2C ��д����	

 	DS1302_Configuration();
	Read1302Ram(RWBuf,0,1);
	if(RWBuf[0]^0xa5)
	{
		RWBuf[0]=0xa5;
		Write1302Ram(RWBuf,0,1);
		Write1302(DS1302_WRITE,0x00);		//�ر�д����
		//Write1302(0x90,0x03);					//��ֹ������
		Write1302(0x90,0xA9);						//����������	 //�������/����������/2K����	
		Write1302(DS1302_HOUR,0x00);		//���ó�24Сʱ��
		Write1302(DS1302_SECOND,0x00);	//ʹ��ʱ������
		Write1302(DS1302_WRITE,0x80);		//��д����
	} */

	DS1302_Configuration();
	Write1302(DS1302_WRITE, 0x00); //�ر�д����
	//Write1302(0x90,0x03);					//��ֹ������
	Write1302(0x90, 0xA9);		   //����������	 //�������/����������/2K����
	Write1302(DS1302_WRITE, 0x80); //��д����
}

void TestDS1302(void)
{
	u8 dd0[30], dd1[30], dd2[30];
	u8 i, tt[7];
	u8 test = 3;

	//1. ����������дʱ�� OK
	if (test == 1)
	{
		tt[0] = 0x40; //��
		tt[1] = 0x12; //��
		tt[2] = 0x21; //ʱ
		tt[3] = 0x14; //��
		tt[4] = 0x05; //��
		tt[5] = 0x04; //��
		tt[6] = 0x15; //��
		BurstWrite1302Clock(tt);

		while (1)
		{
			BurstRead1302Clock(tt);
		}
	}

	//2. ����������дRAM OK
	for (i = 1; i < 30; i++)
	{
		dd1[i] = i;
		dd2[i] = 0;
	}
	if (test == 2)
	{
		BurstWrite1302Ram(dd1);
		BurstRead1302Ram(dd2);
	}

	//3. ���Ե�����дRAM OK
	if (test == 3)
	{
		Read1302Ram(dd0, 0, 30);
		Write1302Ram(dd1, 1, 30);
		BurstRead1302Ram(dd2);
		Read1302Ram(dd0, 0, 30);
	}
}

//end
