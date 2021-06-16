/** 
  ******************************************************************************
  * @file    com3_232.c
  * @author  YLS
  * @version V1.00
  * @date    2021-04-03
  * @brief   
	******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "com3_232.h"
#include "GlobalV_Extern.h"			// ȫ�ֱ�������
#include "GlobalConst.h"
#include <stdio.h>					
//#include "CRCdata.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
extern	UART_HandleTypeDef huart3;


/* Private function prototypes -----------------------------------------------*/
extern	void PowerDelay(uint16_t nCount);


/* Private functions ---------------------------------------------------------*/

//���մ������ У�����
void Com3_RcvProcess(void)
{
	uint8_t k,s,i=0;       						// ��ʱ����
    uint16_t j;
	//��Ϊ����,ָ������ʱ�䵽��,�Ϳ��Դ�����յ����ַ�����	
	// ��û�յ������ַ���ʱ�䳬���趨ʱ�����ԶԽ��ջ�����д�����
	// **********************************rcv_counter<>0,�յ��ַ����ܴ���
	if ( Rcv3Counter>0 &&  T_NoRcv3Count!=SClk1Ms )
	{								// ���մ������
		T_NoRcv3Count=SClk1Ms;				// 
		C_NoRcv3Count++;
		if ( C_NoRcv3Count>NORCVMAXMS )				// 	
		{
			/* Disable the UART Parity Error Interrupt and RXNE interrupt*/
//			CLEAR_BIT(huart1->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
			BakRcv3Count=Rcv3Counter;		// �� Rcv3Counter ����
			C_NoRcv3Count=0;				// ��û�н��ռ�����
			//
			if(BakRcv3Count<=RCV3_MAX)		// ���ճ�����ȷ,��������.
			{
				// �ӵ�ַ��⣭���յ�����λ����ѯָ��  ���ı�ͨѶ
				if( Rcv3Buffer[0]==Pw_EquipmentNo3 )		
				{	
					j=CRC16( Rcv3Buffer, BakRcv3Count-2);		// CRC У��
					k=j>>8;
					s=j;
					if ( k==Rcv3Buffer[BakRcv3Count-2] 
						&& s==Rcv3Buffer[BakRcv3Count-1] )
					{											// CRCУ����ȷ
						if ( Rcv3Buffer[1]==3 )		// 03��ȡ���ּĴ���
						{
							B_Com3Cmd03=1;
							j=Rcv3Buffer[2];
							w_Com3RegAddr=(j<<8)+Rcv3Buffer[3];
						}
						else if ( Rcv3Buffer[1]==16 )	// 16Ԥ�ö�Ĵ���
						{
//							C_ForceSavPar=0;		// ǿ�Ʊ������������=0							
							B_Com3Cmd16=1;
							j=Rcv3Buffer[2];
							w_Com3RegAddr=(j<<8)+Rcv3Buffer[3];
						}
						else if ( Rcv3Buffer[1]==1 )		// 01��ȡ��Ȧ״̬
						{
							B_Com3Cmd01=1;
						}
						else if ( Rcv3Buffer[1]==6 )		// 06Ԥ�õ��Ĵ���
						{
//							C_ForceSavPar=0;		// ǿ�Ʊ������������=0							
							B_Com3Cmd06=1;
							j=Rcv3Buffer[2];
							w_Com3RegAddr=(j<<8)+Rcv3Buffer[3];
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
			HAL_UART_Receive_IT(&huart3, (uint8_t *)&Tmp_Rxd3Buffer, 1);
			Rcv3Counter=0;					// ׼���´ν��յ����濪ʼ
		}
	}
	if(i>0)
	{
		for(j=0;j<20;j++)
		{
			Rcv3Buffer[j]=0;
		}
//		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	
		HAL_UART_Receive_IT(&huart3, (uint8_t *)&Tmp_Rxd3Buffer, 1);		
	}	
}



void Com3_SlaveSend(void)			// ����1�ӻ�����  
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
	if ( B_Com3Cmd03 )		// ��ȡ���ּĴ���
	{
		Txd3Buffer[0]=Rcv3Buffer[0];	// �豸�ӵ�ַPw_EquipmentNo
		Txd3Buffer[1]=Rcv3Buffer[1];	// ������
		Txd3Buffer[2]=Rcv3Buffer[5]*2;	// Rcv3Buffer[5]=���� ��
		//
		if ( w_Com3RegAddr<0x800 )	// �����ѯ
		{
			p_wRead=w_ParLst;			// PAR��
			p_bMove=Txd3Buffer;
			//
			for ( k=0;k<Rcv3Buffer[5] ;k++ )	// ����ѯ����
			{
				m=*(p_wRead+w_Com3RegAddr+k);	
				*(p_bMove+3+k*2)=m>>8;
				*(p_bMove+3+k*2+1)=m;
			}
		}
		else if ( w_Com3RegAddr>=5000)		// ��ȡ����趨������������ַ��5000
		{
			p_wRead2=w_ParLst_Drive;			// PAR��
			p_bMove=Txd3Buffer;
			//
			for ( k=0;k<Rcv3Buffer[5] ;k++ )	// ����ѯ����
			{
				m=*(p_wRead2+w_Com3RegAddr-5000+k);		//������ַ��5000
				*(p_bMove+3+k*2)=m>>8;
				*(p_bMove+3+k*2+1)=m;
			}
		}
		//
		w_Txd3ChkSum=CRC16( Txd3Buffer, Txd3Buffer[2]+3 );
		Txd3Buffer[Txd3Buffer[2]+3]=w_Txd3ChkSum>>8;			// /256
		Txd3Buffer[Txd3Buffer[2]+4]=w_Txd3ChkSum;				// ��λ�ֽ�
		Txd3Max=Txd3Buffer[2]+5;
		//
		B_Com3Cmd03=0;
		Txd3Counter=0;
		HAL_UART_Transmit(&huart3, (u8 *)&Txd3Buffer, Txd3Max,0xffff);
		HAL_GPIO_TogglePin(LED_H66_GPIO_Port,LED_H66_Pin);
	}
	//
	else if ( B_Com3Cmd16 || B_Com3Cmd06 )					// 16Ԥ�ö�Ĵ���
	{
		if ( w_Com3RegAddr<=6000 ) //YLS 2020.06.23����������Ϳ����޸Ĳ���
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
			if ( w_Com3RegAddr>=45 && w_Com3RegAddr<=51 )			// �޸�ʱ��
			{
				w_ModRealTimeNo=w_Com3RegAddr-45;
				F_ModRealTime=1;
			}	
			//
			if ( B_Com3Cmd06 )		// Ԥ�õ���
			{
				if ( w_Com3RegAddr<0x800 )
				{
					m=Rcv3Buffer[4];
					w_ParLst[w_Com3RegAddr]=(m<<8)+Rcv3Buffer[5];
				}
				//-------------------------
				else if ( w_Com3RegAddr>=5000)			// �޸��ŷ��������
				{
					m=Rcv3Buffer[4];
					w_ParLst_Drive[w_Com3RegAddr-5000]=(m<<8)+Rcv3Buffer[5];	//��ַ-5000
				}
				//-------------------------
			}
			else if ( B_Com3Cmd16 )	// Ԥ�ö��
			{
				if ( Rcv3Buffer[5]<100 )
				{
					if ( w_Com3RegAddr<0x800 )
					{
						p_bGen=Rcv3Buffer;
						p_wTarget=w_ParLst;
						for ( k=0;k<Rcv3Buffer[5] ;k++ )		// Rcv3Buffer[5]=����
						{	
							m = *(p_bGen+7+k*2);
							n = *(p_bGen+7+k*2+1);
							*(p_wTarget+w_Com3RegAddr+k)= (m<<8) + n;	
						}	
					}
					else if ( w_Com3RegAddr>=5000)			// �޸��ŷ��������
					{
						p_bGen=Rcv3Buffer;
						p_wTarget2=w_ParLst_Drive;
						for ( k=0;k<Rcv3Buffer[5] ;k++ )		// Rcv3Buffer[5]=����
						{	
							m = *(p_bGen+7+k*2);
							n = *(p_bGen+7+k*2+1);
							*(p_wTarget2+w_Com3RegAddr-5000+k)= (m<<8) + n;	
						}
					}						
				}
			}
		}


		// -------------------------
		// ��������
		Txd3Buffer[0]=2;				// �豸�ӵ�ַ
		Txd3Buffer[1]=Rcv3Buffer[1];	// ������
		Txd3Buffer[2]=Rcv3Buffer[2];	// ��ʼ��ַ��λ�ֽ�
		Txd3Buffer[3]=Rcv3Buffer[3];	// ��ʼ��ַ��λ�ֽ�
		Txd3Buffer[4]=Rcv3Buffer[4];	// �Ĵ���������λ
		Txd3Buffer[5]=Rcv3Buffer[5];	// �Ĵ���������λ	
		if ( j==0 )							// ������ܱ�����Ԥ�ã�����FFFF zcl
		{
			Txd3Buffer[4]=0xff;				// �Ĵ���������λ��Ԥ������
			Txd3Buffer[5]=0xff;				// �Ĵ���������λ��Ԥ������
		}
		w_Txd3ChkSum=CRC16( Txd3Buffer, 6);
		Txd3Buffer[6]=w_Txd3ChkSum>>8;		// /256
		Txd3Buffer[7]=w_Txd3ChkSum;				// ��λ�ֽ�
		Txd3Max=8;
		
		B_Com3Cmd16=0;
		B_Com3Cmd06=0;
		Txd3Counter=0;
		HAL_UART_Transmit(&huart3, (u8 *)&Txd3Buffer, Txd3Max,0xffff);
//		HAL_GPIO_TogglePin(LED_H64_GPIO_Port,LED_H64_Pin);
	}// 06��16Ԥ�üĴ��� ����
}







/******************* (C) COPYRIGHT 2021 SANLI *****END OF FILE****/
