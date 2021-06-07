/***************************************************************************/
//#include "delay.h"���ļ�ΪWK2XXXϵ�д�����չоƬ���豸��������,��Ϊ�����ο�demo��ʹ���߿��Ը����������������޸ģ����ơ�

/***************************************************************************/
#include "wk2xxx.h"
#include "spi.h"
#include "usart.h"
#include "tim.h"
#include "typedef.h"

#define WK2124_RST_LOW HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
#define WK2124_RST_HIGH HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)

#define WK2124_CS_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define WK2124_CS_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

void WK2XXX_RST_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); //PC.1 �����
	WK2124_RST_HIGH;
}
void WK2XXX_Reset_Init(void)
{
	WK2124_RST_HIGH;
	WK2124_RST_LOW;
	HAL_Delay(10); //��ʱ
	WK2124_RST_HIGH;
	HAL_Delay(100); //��ʱ
}
/*************************************************************************/
//�������ܣ���ʼ��SPIƬѡ�ź�CS,����CS��Ĭ��״̬����Ϊ�ߵ�ƽ
//
//
/*************************************************************************/
void SPI_CS_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	WK2124_CS_OFF; //PA.4 �����
}
/*************************************************************************/
//�������ܣ���ʼ��SPI���ߣ�����SPI����Ϊ0ģʽ
/*************************************************************************/
void SPI_BUS_Init(void)
{

	MX_SPI1_Init(); //��ʼ��SPI
}
/*************************************************************************/
//�������ܣ�����CS�ź�Ϊ�ߵ�ƽ
/*************************************************************************/
void SPI_CS_H(void)
{
	WK2124_CS_OFF;
	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
/*************************************************************************/
//�������ܣ�����CS�ź�Ϊ�͵�ƽ
/*************************************************************************/
void SPI_CS_L(void)
{
	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	WK2124_CS_ON;
}
/*************************************************************************/
//�������ܣ���ʼ��SPI�ӿ�
/*************************************************************************/
void WK2XXX_SPI_Init(void)
{
	SPI_CS_Init();
	SPI_BUS_Init();
}

/*************************************************************************/
//�������ܣ�д�Ĵ���������ǰ���ǸüĴ�����д��ĳЩ�Ĵ��������д1�����ܻ��Զ���1������������ֲ�)
//������port:Ϊ�Ӵ��ڵ���(C0C1)
//      reg:Ϊ�Ĵ����ĵ�ַ(A3A2A1A0)
//      dat:Ϊд��Ĵ���������
//ע�⣺���Ӵ��ڱ���ͨ������£���FDATд������ݻ�ͨ��TX�������
//*************************************************************************/
void Wk2xxxWriteReg(unsigned char port, unsigned char reg, unsigned char dat)
{

	SPI_CS_L();									 //Ƭѡʹ��
	SPI1_ReadWriteByte(((port - 1) << 4) + reg); //д�����ֽ�
	SPI1_ReadWriteByte(dat);					 //д����
	SPI_CS_H();									 //Ƭѡ��Ч
}

/*************************************************************************/
//�������ܣ����Ĵ�������
//������port:Ϊ�Ӵ��ڵ���(C0C1)
//      reg:Ϊ�Ĵ����ĵ�ַ(A3A2A1A0)
//      rec_data:Ϊ��ȡ���ļĴ���ֵ
//ע�⣺���Ӵ��ڱ���ͨ������£���FDAT��ʵ���Ͼ��Ƕ�ȡuart��rx���յ�����
/*************************************************************************/
unsigned char Wk2xxxReadReg(unsigned char port, unsigned char reg)
{
	unsigned char rec_data;
	SPI_CS_L();											//Ƭѡʹ��
	SPI1_ReadWriteByte(0x40 + ((port - 1) << 4) + reg); //д�����ֽڣ���������ɼ������ֲ�
	rec_data = SPI1_ReadWriteByte(0);					//���շ��ص�����
	SPI_CS_H();											//Ƭѡ��Ч
	return rec_data;
}
/**************************** Wk2xxxWriteFifo*********************************************/
//�������ܣ��ú���ΪдFIFO������ͨ���ú���д������ݻ�ֱ�ӽ����Ӵ��ڵķ���FIFO��Ȼ��ͨ��TX���ŷ���
//������port��Ϊ�Ӵ��ڵĶ˿ں�(C0\C1)
//      *wbuf:д�����ݲ���
//      len��  д�����ݳ���
//
/*************************************************************************/
void Wk2xxxWriteFifo(unsigned char port, unsigned char *wbuf, unsigned int len)
{
	unsigned char n;
	SPI_CS_L();									  // Ƭѡ��Ч
	SPI1_ReadWriteByte(0x80 + ((port - 1) << 4)); //дFIFO����ָ��
	for (n = 0; n < len; n++)
	{
		SPI1_ReadWriteByte(*(wbuf + n));
	}
	SPI_CS_H(); //Ƭѡ��Ч
}

/**************************** Wk2xxxReadFifo*********************************************/
//�������ܣ��ú���Ϊ��FIFO������ͨ���ú�������һ�ζ����������FIFO�е����ݣ����256���ֽ�
//������port��Ϊ�Ӵ��ڵĶ˿ں�(C0\C1)
//      *rbuf:д�����ݲ���
//      len��  д�����ݳ���
//
/*************************************************************************/
void Wk2xxxReadFifo(unsigned char port, unsigned char *rbuf, unsigned int len)
{
	unsigned char n;
	SPI_CS_L();									  //Ƭѡ��Ч
	SPI1_ReadWriteByte(0xc0 + ((port - 1) << 4)); //д��fifo����ָ��
	for (n = 0; n < len; n++)
	{
		*(rbuf + n) = SPI1_ReadWriteByte(0);
	}
	SPI_CS_H(); //Ƭѡ��Ч
		//return 0;
}

/*************************************************************************/
//��������:�˺�����Ҫ��ͨ����дwk2xxx�ļĴ������ж����ӿڵ�ͨ��ʱ���Ƿ�������
//��������
//����ֵ��rv��ʾ����ֵ��0�ɹ�
/*************************************************************************/
// unsigned char Wk2xxxTest(void)
// {
//	unsigned char rec_data,rv;
////���ӿ�ΪSPI
//	rec_data=Wk2xxxReadReg(WK2XXX_GPORT,WK2XXX_GENA);
//	if(rec_data==0x30)
//		return rv;
//	else
//		{
//			rv=1;
//			return rv;
//		}
//
// }
/******************************Wk2xxxInit*******************************************/
//�������ܣ���������Ҫ���ʼ��һЩоƬ�����Ĵ�����
/*********************************************************************************/
void Wk2xxxInit(unsigned char port)
{
	unsigned char gena, grst, gier, sier, scr, lcr;
	//ʹ���Ӵ���ʱ��
	gena = Wk2xxxReadReg(WK2XXX_GPORT, WK2XXX_GENA);
	switch (port)
	{
	case 1: //ʹ���Ӵ���1��ʱ��
		gena |= WK2XXX_UT1EN;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GENA, gena);
		break;
	case 2: //ʹ���Ӵ���2��ʱ��
		gena |= WK2XXX_UT2EN;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GENA, gena);
		break;
	case 3: //ʹ���Ӵ���3��ʱ��
		gena |= WK2XXX_UT3EN;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GENA, gena);
		break;
	case 4: //ʹ���Ӵ���4��ʱ��
		gena |= WK2XXX_UT4EN;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GENA, gena);
		break;
	}
	//�����λ�Ӵ���
	grst = Wk2xxxReadReg(WK2XXX_GPORT, WK2XXX_GRST);
	switch (port)
	{
	case 1: //�����λ�Ӵ���1
		grst |= WK2XXX_UT1RST;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GRST, grst);
		break;
	case 2: //�����λ�Ӵ���2
		grst |= WK2XXX_UT2RST;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GRST, grst);
		break;
	case 3: //�����λ�Ӵ���3
		grst |= WK2XXX_UT3RST;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GRST, grst);
		break;
	case 4: //�����λ�Ӵ���4
		grst |= WK2XXX_UT4RST;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GRST, grst);
		break;
	}
	//ʹ���Ӵ����жϣ������Ӵ������жϺ��Ӵ����ڲ��Ľ����жϣ��������жϴ���
	gier = Wk2xxxReadReg(WK2XXX_GPORT, WK2XXX_GIER);
	switch (port)
	{
	case 1: //�����λ�Ӵ���1
		gier |= WK2XXX_UT1RST;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GIER, gier);
		break;
	case 2: //�����λ�Ӵ���2
		gier |= WK2XXX_UT2RST;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GIER, gier);
		break;
	case 3: //�����λ�Ӵ���3
		gier |= WK2XXX_UT3RST;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GIER, gier);
		break;
	case 4: //�����λ�Ӵ���4
		gier |= WK2XXX_UT4RST;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GIER, gier);
		break;
	}
	//ʹ���Ӵ��ڽ��մ����жϺͳ�ʱ�ж�
	sier = Wk2xxxReadReg(port, WK2XXX_SIER);
	sier |= WK2XXX_RFTRIG_IEN | WK2XXX_RXOVT_IEN;
	Wk2xxxWriteReg(port, WK2XXX_SIER, sier);
	// ��ʼ��FIFO�����ù̶��жϴ���
	Wk2xxxWriteReg(port, WK2XXX_FCR, 0XFF);
	//���������жϴ��㣬��������������Ч����ô����FCR�Ĵ����жϵĹ̶��жϴ��㽫ʧЧ
	Wk2xxxWriteReg(port, WK2XXX_SPAGE, 1);	 //�л���page1
	Wk2xxxWriteReg(port, WK2XXX_RFTL, 0X40); //���ý��մ���Ϊ64���ֽ�
	Wk2xxxWriteReg(port, WK2XXX_TFTL, 0X10); //���÷��ʹ���Ϊ16���ֽ�
	Wk2xxxWriteReg(port, WK2XXX_SPAGE, 0);	 //�л���page0
	//ʹ���Ӵ��ڵķ��ͺͽ���ʹ��
	scr = Wk2xxxReadReg(port, WK2XXX_SCR);
	scr |= WK2XXX_TXEN | WK2XXX_RXEN;
	Wk2xxxWriteReg(port, WK2XXX_SCR, scr);

	//����У��λ
	lcr = Wk2xxxReadReg(port, WK2XXX_LCR);
	lcr &= ~WK2XXX_STPL; //=0,1bit
	lcr &= ~WK2XXX_PAEN; //=0,��У��λ(8������λ)
	Wk2xxxWriteReg(port, WK2XXX_LCR, lcr);
}

/******************************Wk2xxxClose*******************************************/
//�������ܣ���������رյ�ǰ�Ӵ��ڣ��͸�λ��ʼֵ��
/*********************************************************************************/

void Wk2xxxClose(unsigned char port)
{
	unsigned char gena, grst;
	//��λ�Ӵ���
	grst = Wk2xxxReadReg(WK2XXX_GPORT, WK2XXX_GRST);
	switch (port)
	{
	case 1: //�����λ�Ӵ���1
		grst |= WK2XXX_UT1RST;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GRST, grst);
		break;
	case 2: //�����λ�Ӵ���2
		grst |= WK2XXX_UT2RST;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GRST, grst);
		break;
	case 3: //�����λ�Ӵ���3
		grst |= WK2XXX_UT3RST;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GRST, grst);
		break;
	case 4: //�����λ�Ӵ���4
		grst |= WK2XXX_UT4RST;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GRST, grst);
		break;
	}
	//�ر��Ӵ���ʱ��
	gena = Wk2xxxReadReg(WK2XXX_GPORT, WK2XXX_GENA);
	switch (port)
	{
	case 1: //ʹ���Ӵ���1��ʱ��
		gena &= ~WK2XXX_UT1EN;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GENA, gena);
		break;
	case 2: //ʹ���Ӵ���2��ʱ��
		gena &= ~WK2XXX_UT2EN;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GENA, gena);
		break;
	case 3: //ʹ���Ӵ���3��ʱ��
		gena &= ~WK2XXX_UT3EN;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GENA, gena);
		break;
	case 4: //ʹ���Ӵ���4��ʱ��
		gena &= ~WK2XXX_UT4EN;
		Wk2xxxWriteReg(WK2XXX_GPORT, WK2XXX_GENA, gena);
		break;
	}
}

/**************************Wk2xxxSetBaud*******************************************************/
//�������ܣ������Ӵ��ڲ����ʺ������˺����в����ʵ�ƥ��ֵ�Ǹ���18.432MHz�µ��ⲿ��������
// port:�Ӵ��ں�
// baud:�����ʴ�С.�����ʱ�ʾ��ʽ��
//
/**************************Wk2xxxSetBaud*******************************************************/
void Wk2xxxSetBaud(unsigned char port, int baud)
{
	unsigned char baud1, baud0, pres, scr;
	//���²�������Ӧ�ļĴ���ֵ�������ⲿʱ��Ϊ18.432MHz������¼������ã����ʹ������������Ҫ���¼���
	switch (baud)
	{
	case B600:
		baud1 = 0x07;
		baud0 = 0x7f;
		pres = 0;
		break;
	case B1200:
		baud1 = 0x3;
		baud0 = 0x3F;
		pres = 0;
		break;
	case B2400:
		baud1 = 0x1;
		baud0 = 0xdf;
		pres = 0;
		break;
	case B4800:
		baud1 = 0x00;
		baud0 = 0xef;
		pres = 0;
		break;
	case B9600:
		baud1 = 0x00;
		baud0 = 0x77;
		pres = 0;
		break;
	case B19200:
		baud1 = 0x00;
		baud0 = 0x3b;
		pres = 0;
		break;
	case B38400:
		baud1 = 0x00;
		baud0 = 0x1d;
		pres = 0;
		break;

	case B76800:
		baud1 = 0x00;
		baud0 = 0x0e;
		pres = 0;
		break;

	case B1800:
		baud1 = 0x02;
		baud0 = 0x7f;
		pres = 0;
		break;
	case B3600:
		baud1 = 0x01;
		baud0 = 0x3f;
		pres = 0;
		break;
	case B7200:
		baud1 = 0x00;
		baud0 = 0x9f;
		pres = 0;
		break;
	case B14400:
		baud1 = 0x00;
		baud0 = 0x4f;
		pres = 0;
		break;
	case B28800:
		baud1 = 0x00;
		baud0 = 0x27;
		pres = 0;
		break;
	case B57600:
		baud1 = 0x00;
		baud0 = 0x13;
		pres = 0;
		break;
	case B115200:
		baud1 = 0x00;
		baud0 = 0x09;
		pres = 0;
		break;
	case B230400:
		baud1 = 0x00;
		baud0 = 0x04;
		pres = 0;
		break;
	default:
		baud1 = 0x00;
		baud0 = 0x00;
		pres = 0;
	}
	//�ص��Ӵ����շ�ʹ��
	scr = Wk2xxxReadReg(port, WK2XXX_SCR);
	Wk2xxxWriteReg(port, WK2XXX_SCR, 0);
	//���ò�������ؼĴ���
	Wk2xxxWriteReg(port, WK2XXX_SPAGE, 1); //�л���page1
	Wk2xxxWriteReg(port, WK2XXX_BAUD1, baud1);
	Wk2xxxWriteReg(port, WK2XXX_BAUD0, baud0);
	Wk2xxxWriteReg(port, WK2XXX_PRES, pres);
	Wk2xxxWriteReg(port, WK2XXX_SPAGE, 0); //�л���page0
	//ʹ���Ӵ����շ�ʹ��
	Wk2xxxWriteReg(port, WK2XXX_SCR, scr);
}
/*****************************Wk2xxxSendBuf****************************************/
//������Ϊ�Ӵ��ڷ������ݵĺ������������ݵ��Ӵ��ڵ�FIFO.Ȼ��ͨ���ٷ���
//����˵����port���Ӵ��ڶ˿ں�
//          *sendbuf:��Ҫ���͵�����buf
//          len����Ҫ�������ݵĳ���
// ��������ֵ��ʵ�ʳɹ����͵�����
//˵�������ô˺���ֻ�ǰ�����д���Ӵ��ڵķ���FIFO��Ȼ���ٷ��͡�1������ȷ���Ӵ��ڵķ���FIFO�ж������ݣ����ݾ��������
//ȷ��д��FIFO���ݵĸ�����
/*********************************************************************/
unsigned int Wk2xxxSendBuf(unsigned char port, unsigned char *sendbuf, unsigned int len)
{
	unsigned int ret, tfcnt, sendlen;
	unsigned char fsr;

	fsr = Wk2xxxReadReg(port, WK2XXX_FSR);
	if (~fsr & WK2XXX_TFULL) //�Ӵ��ڷ���FIFOδ��
	{

		tfcnt = Wk2xxxReadReg(port, WK2XXX_TFCNT); //���Ӵ��ڷ���fifo�����ݸ���
		sendlen = 256 - tfcnt;					   //FIFO��д�������ֽ���

		if (sendlen < len)
		{
			ret = sendlen;
			Wk2xxxWriteFifo(port, sendbuf, sendlen);
		}
		else
		{
			Wk2xxxWriteFifo(port, sendbuf, len);
			ret = len;
		}
	}

	return ret;
}

/*****************************Wk2xxxGetBuf****************************************/
//������Ϊ�Ӵ��ڽ������ݺ���
//����˵����port���Ӵ��ڶ˿ں�
//          *getbuf:���յ�������buf
// ��������ֵ��ʵ�ʽ��յ������ݸ���
/*********************************************************************/
unsigned int Wk2xxxGetBuf(unsigned char port, unsigned char *getbuf)
{
	unsigned int ret = 0, rfcnt;
	unsigned char fsr;
	fsr = Wk2xxxReadReg(port, WK2XXX_FSR);
	if (fsr & WK2XXX_RDAT) //�Ӵ��ڽ���FIFOδ��
	{
		rfcnt = Wk2xxxReadReg(port, WK2XXX_RFCNT); //���Ӵ��ڷ���fifo�����ݸ���
		if (rfcnt == 0)							   //��RFCNT�Ĵ���Ϊ0��ʱ�������������������256������0�����ʱ��ͨ��FSR���жϣ����FSR��ʾ����FIFO��Ϊ�գ���Ϊ256���ֽ�
		{
			rfcnt = 256;
		}
		Wk2xxxReadFifo(port, getbuf, rfcnt);
		ret = rfcnt;
	}
	return ret;
}

/**************************WK_RxChars*******************************************/
//��������:��ȡ�Ӵ���fifo�е�����
// port:�˿ں�
// recbuf:���յ�������
// ����ֵ���������ݵĳ���
/**************************WK_RxChars********************************************/
int wk_RxChars(u8 port, u8 *recbuf)
{
	u8 fsr = 0, rfcnt = 0, rfcnt2 = 0, sifr = 0;
	int len = 0;
	sifr = Wk2xxxReadReg(port, WK2XXX_SIFR);

	if ((sifr & WK2XXX_RFTRIG_INT) || (sifr & WK2XXX_RXOVT_INT)) //�н����жϺͽ��ճ�ʱ�ж�
	{
		fsr = Wk2xxxReadReg(port, WK2XXX_FSR);
		rfcnt = Wk2xxxReadReg(port, WK2XXX_RFCNT);
		rfcnt2 = Wk2xxxReadReg(port, WK2XXX_RFCNT);
		//printf("rfcnt=0x%x.\n",rfcnt);
		/*�ж�fifo�����ݸ���*/
		if (fsr & WK2XXX_RDAT)
		{
			if (!(rfcnt2 >= rfcnt))
			{
				rfcnt = rfcnt2;
			}
			len = (rfcnt == 0) ? 256 : rfcnt;
		}
#if 1
		Wk2xxxReadFifo(port, recbuf, len);
#else
		for (n = 0; n < len; n++)
			*(recbuf + n) = WkReadSReg(port, WK2XXX_FDAT);
#endif
		return len;
	}
	else
	{
		len = 0;
		return len;
	}
}

/**************************WK_TxChars*******************************************/
//��������:ͨ���Ӵ��ڷ��͹̶���������
// port:�˿ں�
// len:���η��ͳ��Ȳ�����256
//
/**************************WK_TxChars********************************************/
int wk_TxChars(u8 port, int len, u8 *sendbuf)
{

#if 1
	Wk2xxxWriteFifo(port, sendbuf, len); //ͨ��fifo��ʽ��������
#else
	int num = len;
	for (num = 0; num < len; num++)
	{
		WkWriteSReg(port, WK2XXX_FDAT, *(sendbuf + num));
	}
#endif
	return 0;
}
