/** 
  ******************************************************************************
  * @file    com4_232.c
  * @author  YLS
  * @version V1.00
  * @date    2021-04-03
  * @brief   
	******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "com4_232.h"
#include "GlobalV_Extern.h"			// ȫ�ֱ�������
#include "GlobalConst.h"
#include <stdio.h>					
//#include "CRCdata.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
extern	UART_HandleTypeDef huart4;


/* Private function prototypes -----------------------------------------------*/
extern	void PowerDelay(uint16_t nCount);


/* Private functions ---------------------------------------------------------*/

//���մ������ У�����
void Com4_RcvProcess(void)
{
	uint8_t k,s,i=0;       						// ��ʱ����
    uint16_t j;
	//��Ϊ����,ָ������ʱ�䵽��,�Ϳ��Դ�����յ����ַ�����	
	// ��û�յ������ַ���ʱ�䳬���趨ʱ�����ԶԽ��ջ�����д�����
	// **********************************rcv_counter<>0,�յ��ַ����ܴ���
	if ( Rcv4Counter>0 &&  T_NoRcv4Count!=SClk1Ms )
	{								// ���մ������
		T_NoRcv4Count=SClk1Ms;				// 
		C_NoRcv4Count++;
		if ( C_NoRcv4Count>NORCVMAXMS )				// 	
		{
			/* Disable the UART Parity Error Interrupt and RXNE interrupt*/
			BakRcv4Count=Rcv4Counter;		// �� Rcv4Counter ����
			C_NoRcv4Count=0;				// ��û�н��ռ�����
			//
			if(BakRcv4Count<=RCV4_MAX)		// ���ճ�����ȷ,��������.
			{
				// �ӵ�ַ��⣭���յ�����λ����ѯָ��  ���ı�ͨѶ
				if( Rcv4Buffer[0]==Pw_EquipmentNo4 )		
				{	
					j=CRC16( Rcv4Buffer, BakRcv4Count-2);		// CRC У��
					k=j>>8;
					s=j;
					if ( k==Rcv4Buffer[BakRcv4Count-2] 
						&& s==Rcv4Buffer[BakRcv4Count-1] )
					{											// CRCУ����ȷ
						if ( Rcv4Buffer[1]==3 )		// 03��ȡ���ּĴ���
						{
							B_Com4Cmd03=1;
							j=Rcv4Buffer[2];
							w_Com4RegAddr=(j<<8)+Rcv4Buffer[3];
						}
						else if ( Rcv4Buffer[1]==16 )	// 16Ԥ�ö�Ĵ���
						{
//							C_ForceSavPar=0;		// ǿ�Ʊ������������=0							
							B_Com4Cmd16=1;
							j=Rcv4Buffer[2];
							w_Com4RegAddr=(j<<8)+Rcv4Buffer[3];
						}
						else if ( Rcv4Buffer[1]==1 )		// 01��ȡ��Ȧ״̬
						{
							B_Com4Cmd01=1;
						}
						else if ( Rcv4Buffer[1]==6 )		// 06Ԥ�õ��Ĵ���
						{
//							C_ForceSavPar=0;		// ǿ�Ʊ������������=0							
							B_Com4Cmd06=1;
							j=Rcv4Buffer[2];
							w_Com4RegAddr=(j<<8)+Rcv4Buffer[3];
						}
						
						else
							i=1;
					}
					else
						i=2;
				}
			}
			else
				i=4;
//			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
			HAL_UART_Receive_IT(&huart4, (uint8_t *)&Tmp_Rxd4Buffer, 1);
			Rcv4Counter=0;					// ׼���´ν��յ����濪ʼ
		}
	}
	if(i>0)
	{
		for(j=0;j<20;j++)
		{
			Rcv4Buffer[j]=0;
		}
//		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	
		HAL_UART_Receive_IT(&huart4, (uint8_t *)&Tmp_Rxd4Buffer, 1);		
	}	
}



void Com4_SlaveSend(void)			// ����1�ӻ�����  
{
	uint16_t	m,n; 
	uint8_t		j=0,k;
	uint16_t	 *p_wRead;
	s32	 		*p_wRead2;
	uint8_t	 	*p_bMove;
	uint8_t	 	*p_bGen;
	uint16_t	 *p_wTarget;			// ָ��Ŀ���ַ�����xdata zcl
	s32	 		*p_wTarget2;

	//
	if ( B_Com4Cmd03 )		// ��ȡ���ּĴ���
	{
		Txd4Buffer[0]=Rcv4Buffer[0];	// �豸�ӵ�ַPw_EquipmentNo
		Txd4Buffer[1]=Rcv4Buffer[1];	// ������
		Txd4Buffer[2]=Rcv4Buffer[5]*2;	// Rcv4Buffer[5]=���� ��
		//
		if ( w_Com4RegAddr<0x800 && Pw_ComBufType==1 )	// �����ѯ Pw_ComBufType==1 2016.4.21
		{
			p_wRead=w_ParLst;			// PAR��
			p_bMove=Txd4Buffer;
			//
			for ( k=0;k<Rcv4Buffer[5] ;k++ )	// ����ѯ����
			{
				m=*(p_wRead+w_Com4RegAddr+k);	
				*(p_bMove+3+k*2)=m>>8;
				*(p_bMove+3+k*2+1)=m;
			}
		}
		else if ( w_Com4RegAddr>=5000 && Pw_ComBufType==1 )		// ��ȡ����趨������������ַ��5000
		{
			p_wRead2=w_ParLst_Drive;			// PAR��
			p_bMove=Txd4Buffer;
			//
			for ( k=0;k<Rcv4Buffer[5] ;k++ )	// ����ѯ����
			{
				m=*(p_wRead2+w_Com4RegAddr-5000+k);		//������ַ��5000
				*(p_bMove+3+k*2)=m>>8;
				*(p_bMove+3+k*2+1)=m;
			}
		}
		//
		w_Txd4ChkSum=CRC16( Txd4Buffer, Txd4Buffer[2]+3 );
		Txd4Buffer[Txd4Buffer[2]+3]=w_Txd4ChkSum>>8;			// /256
		Txd4Buffer[Txd4Buffer[2]+4]=w_Txd4ChkSum;				// ��λ�ֽ�
		Txd4Max=Txd4Buffer[2]+5;
		//
		B_Com4Cmd03=0;
		Txd4Counter=0;
		HAL_UART_Transmit(&huart4, (u8 *)&Txd4Buffer, Txd4Max,0xffff);
		HAL_GPIO_TogglePin(LED_H66_GPIO_Port,LED_H66_Pin);
	}
	//
	else if ( B_Com4Cmd16 || B_Com4Cmd06 )					// 16Ԥ�ö�Ĵ���
	{
		if ( w_Com4RegAddr<=6000 ) //YLS 2020.06.23����������Ϳ����޸Ĳ���
		{
			j=1;
		}
		// ��Ҫ����ſ����޸ĵĲ���
		else if ( Pw_ModPar==2000 )	// ��Ҫ�Ȱ�Pw_ModPar �޸ĳɹ涨ֵ�������޸ĵĲ���
		{
			j=1;
		}

		// �޸Ĳ�����Ԫ
		if ( j )					
		{
			if ( w_Com4RegAddr>=45 && w_Com4RegAddr<=51 )			// �޸�ʱ��
			{
				w_ModRealTimeNo=w_Com4RegAddr-45;
				F_ModRealTime=1;
			}	
			//
			if ( B_Com4Cmd06 )		// Ԥ�õ���
			{
				if ( w_Com4RegAddr<0x800 )
				{
					m=Rcv4Buffer[4];
					w_ParLst[w_Com4RegAddr]=(m<<8)+Rcv4Buffer[5];
				}
				//-------------------------
				else if ( w_Com4RegAddr>=5000)			// �޸��ŷ��������
				{
					m=Rcv4Buffer[4];
					w_ParLst_Drive[w_Com4RegAddr-5000]=(m<<8)+Rcv4Buffer[5];	//��ַ-5000
				}
				//-------------------------
			}
			else if ( B_Com4Cmd16 )	// Ԥ�ö��
			{
				if ( Rcv4Buffer[5]<100 )
				{
					if ( w_Com4RegAddr<0x800 )
					{
						p_bGen=Rcv4Buffer;
						p_wTarget=w_ParLst;
						for ( k=0;k<Rcv4Buffer[5] ;k++ )		// Rcv4Buffer[5]=����
						{	
							m = *(p_bGen+7+k*2);
							n = *(p_bGen+7+k*2+1);
							*(p_wTarget+w_Com4RegAddr+k)= (m<<8) + n;	
						}	
					}
					else if ( w_Com4RegAddr>=5000)			// �޸��ŷ��������
					{
						p_bGen=Rcv4Buffer;
						p_wTarget2=w_ParLst_Drive;
						for ( k=0;k<Rcv4Buffer[5] ;k++ )		// Rcv4Buffer[5]=����
						{	
							m = *(p_bGen+7+k*2);
							n = *(p_bGen+7+k*2+1);
							*(p_wTarget2+w_Com4RegAddr-5000+k)= (m<<8) + n;	
						}
					}						
				}
			}
		}


		// -------------------------
		// ��������
		Txd4Buffer[0]=2;				// �豸�ӵ�ַ
		Txd4Buffer[1]=Rcv4Buffer[1];	// ������
		Txd4Buffer[2]=Rcv4Buffer[2];	// ��ʼ��ַ��λ�ֽ�
		Txd4Buffer[3]=Rcv4Buffer[3];	// ��ʼ��ַ��λ�ֽ�
		Txd4Buffer[4]=Rcv4Buffer[4];	// �Ĵ���������λ
		Txd4Buffer[5]=Rcv4Buffer[5];	// �Ĵ���������λ	
		if ( j==0 )							// ������ܱ�����Ԥ�ã�����FFFF zcl
		{
			Txd4Buffer[4]=0xff;				// �Ĵ���������λ��Ԥ������
			Txd4Buffer[5]=0xff;				// �Ĵ���������λ��Ԥ������
		}
		w_Txd4ChkSum=CRC16( Txd4Buffer, 6);
		Txd4Buffer[6]=w_Txd4ChkSum>>8;		// /256
		Txd4Buffer[7]=w_Txd4ChkSum;				// ��λ�ֽ�
		Txd4Max=8;
		
		B_Com4Cmd16=0;
		B_Com4Cmd06=0;
		Txd4Counter=0;
		HAL_UART_Transmit(&huart4, (u8 *)&Txd4Buffer, Txd4Max,0xffff);
//		HAL_GPIO_TogglePin(LED_H64_GPIO_Port,LED_H64_Pin);
	}// 06��16Ԥ�üĴ��� ����
}







/******************* (C) COPYRIGHT 2021 SANLI *****END OF FILE****/
