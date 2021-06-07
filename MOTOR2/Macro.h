#ifndef __MACRO_H
#define __MACRO_H

#define M1DIV 1 //������1��ϸ����	16	 32
#define M2DIV 1 //������2��ϸ����		32
#define M3DIV 1 //������3��ϸ����		32
#define M4DIV 1 //������4��ϸ����		64
#define M5DIV 1 //������5��ϸ����		64
#define M6DIV 1 //������6��ϸ����		64

//���Ҫ������ʼ��ת��Ϊ60rpm����1��ת1Ȧ�����跢10000������ת1Ȧ������ʼת��Ϊ1���ӷ�10000������
#define M_FRE_START 10000 //���������Ƶ��	  8000
#define M_FRE_AA 10000    //���Ƶ�ʵļӼ��ٶ�		 4500
#define M_T_AA 2          //���Ƶ�ʵļӼ���ʱ��2
#define M_T_UA 5          //���Ƶ�ʵ��ȼ���ʱ��	   6
#define M_T_RA 2          //���Ƶ�ʵļ�����ʱ�� 2

#define Motor1_XiShu_1 0.5 //ϵ��1
#define Motor2_XiShu_1 0.5 //ϵ��1
#define Motor3_XiShu_1 0.5 //ϵ��1
#define Motor4_XiShu_1 0.5 //ϵ��1
#define Motor5_XiShu_1 0.5 //ϵ��1
#define Motor6_XiShu_1 0.5 //ϵ��1
#define PULSENUM 10000     //ÿȦ������

//������S�Ͳ���
#define F2TIME_PARA 10500000 //��Ƶ��ֵת��Ϊ��ʱ���Ĵ���ֵ��ת������
#define STEP_PARA 500        //����ʱ��ת��������������
#define STEP_AA 31           //�Ӽ��ٽ׶Σ���ɢ������
#define STEP_UA 31           //�ȼ��ٽ׶Σ���ɢ������
#define STEP_RA 31           //�����ٽ׶Σ���ɢ������

#define MAX_POSITION 8388608 //���λ��
#define TIM_Idle_HIGH 1      //��ʱ��������ʱ����ߵ�ƽ

#endif
