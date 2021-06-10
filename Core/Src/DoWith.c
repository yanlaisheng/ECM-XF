/**
  ******************************************************************************
  * @file    DoWith.c
  * @author  ChengLei Zhou  - �ܳ���
  * @version V1.27
  * @date    2014-01-03
  * @brief   �����������⣬���������,ģ���������⣬ģ�������,������������
	******************************************************************************

	******************************************************************************	
	*/

/* Includes ------------------------------------------------------------------*/
#include "GlobalConst.h"
#include "GlobalV_Extern.h" // ȫ�ֱ�������
#include "DoWith.h"
//#include "ds1302.h"
#include "com1_232.h"
#include "typedef.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t FilterNo; // �������
//
uint8_t T_BootParLst; // ��ʼ����������
uint16_t C_BootParLst;
uint8_t T_BootParLst2; // ��ʼ����������
uint16_t C_BootParLst2;

uint8_t S_BootParLst;
uint8_t T_SavePar; // �������
uint16_t C_SavePar;
uint8_t S_SavePar;

uint8_t T_ReadRealTime; // ��ʵʱʱ��
uint16_t C_ReadRealTime;
uint8_t T_ModRealTime; // �޸ģ�д��ʵʱʱ��
uint16_t C_ModRealTime;
uint8_t S_ModRealTime;
uint8_t T_TimeWriteRec; // ��ʱд��¼
uint16_t C_TimeWriteRec;
//
uint8_t T_ForceSavPar;	// ǿ�Ʊ������
uint8_t B_ForceSavPar;	// ��־
uint16_t C_ForceSavPar; // ʹ�ô����ϴ�ʹ��pdata

uint16_t w_DIStableCounter; // ����������PE�ȶ�������

uint8_t T_DoWith; // ����
uint8_t C_bDoWith;
uint8_t T_VADelay; // ��ѹ������ʾ��ʱ,Ϊ���ȶ���� 2007.11.1
uint16_t C_VADelay;

uint8_t F_ReadMemoryBoardNoNum; // ������չ�洢���¼��ź�����
uint32_t PmupRunTimeH;

uint8_t S_KglInPrompt;	// ����������״̬��ʾ ZCL 2015.9.23
uint8_t S_KglOutPrompt; // ���������״̬��ʾ
uint8_t T_KglInPrompt;
uint16_t C_KglInPrompt;
uint8_t T_KglOutPrompt;
uint16_t C_KglOutPrompt;

uint8_t F_KglInPrompt;
uint8_t F_KglOutPrompt;
uint16_t C_SaveDelay;
uint8_t T_SaveDelay;

uint16_t w_DI1StableCounter; // �����������ȶ�������
uint16_t w_DI2StableCounter; // �����������ȶ�������
uint16_t w_DI3StableCounter; // �����������ȶ�������
uint16_t w_DI4StableCounter; // �����������ȶ�������

uint8_t B_DI1;
uint8_t B_DI2;
uint8_t B_DI3;
uint8_t B_DI4;

uint8_t Old_Run_Mode;		  //����ԭ��������ģʽ
uint8_t Old_Pos_Group_Select; //����ԭ�������

//-----------------------------------------------------------------------------------
//EXTERN Global VARIABLES - Bit VARIABLES   �ⲿ������λ��������
//-----------------------------------------------------------------------------------

//
extern uint8_t F_Com2SendNext;		  // ���ڷ�����һ����־
extern uint8_t F_SlaveNoRcvMasterCMD; // �ӻ�û�н��յ���������
extern uint32_t Driver1_Cmd_Group1[];
extern uint32_t Driver2_Cmd_Group1[];
extern uint32_t Driver3_Cmd_Group1[];
extern uint32_t Driver4_Cmd_Group1[];
extern uint32_t Driver5_Cmd_Group1[];
extern uint32_t Driver6_Cmd_Group1[];

extern sc32 Pos_Init[];
extern sc32 Pos_Cmd_Group1[];
extern sc32 Pos_Cmd_Group2[];
extern sc32 Pos_Cmd_Group3[];
extern sc32 Pos_Cmd_Group4[];
extern sc32 Pos_Cmd_Group5[];

extern s16 Com1_Driver1_Queue_Rear;	 //��������е�������������βָ��
extern s16 Com1_Driver2_Queue_Rear;	 //��������е�������������βָ��
extern s16 Com1_Driver1_Queue_Front; //��������еĵ�ǰҪ���ӵģ���ͷָ��
extern s16 Com1_Driver2_Queue_Front; //��������еĵ�ǰҪ���ӵģ���ͷָ��

extern s16 Com2_Driver3_Queue_Rear;	 //��������е�������������βָ��
extern s16 Com2_Driver4_Queue_Rear;	 //��������е�������������βָ��
extern s16 Com2_Driver3_Queue_Front; //��������еĵ�ǰҪ���ӵģ���ͷָ��
extern s16 Com2_Driver4_Queue_Front; //��������еĵ�ǰҪ���ӵģ���ͷָ��

extern s16 Com3_Driver5_Queue_Rear;
extern s16 Com3_Driver6_Queue_Rear;
extern s16 Com3_Driver5_Queue_Front;
extern s16 Com3_Driver6_Queue_Front;

extern s32 *arr_p1;
extern s32 *arrp_p1_Last; //������һ��ָ�������ָ��

extern uint16_t Rcv1Counter; // ���ռ�����//
extern uint16_t Txd1Counter; // ���ͼ�����//
extern uint16_t Rcv2Counter; // ���ռ�����//
extern uint16_t Txd2Counter; // ���ͼ�����//
extern uint16_t Rcv3Counter; // ���ռ�����//
extern uint16_t Txd3Counter; // ���ͼ�����//
extern uint16_t Rcv4Counter; // ���ռ�����//
extern uint16_t Txd4Counter; // ���ͼ�����//

extern uint8_t F_Starting;
extern uint8_t F_Stoping;
extern uint8_t F_AskReset;
extern uint8_t F_Reseting;
extern uint8_t F_AskForceReset;
extern uint8_t F_ForceReseting;
extern uint8_t F_TouchForceReSet;
extern uint8_t F_PowerOnRun;
extern uint8_t F_StepMode;
extern uint8_t F_SendStopCMD2;

extern uint8_t F_Encoder_Read; //�����������Ѷ�ȡ��־
extern uint8_t F_AllRdy;	   //���е����Ready��־
extern uint8_t F_AllRun;	   //���е������RUN��־

extern uint16_t Driver1_RcvCount; //���ռ���
extern uint16_t Driver2_RcvCount; //���ռ���
extern uint16_t Driver3_RcvCount; //���ռ���
extern uint16_t Driver4_RcvCount; //���ռ���
extern uint16_t Driver5_RcvCount; //���ռ���
extern uint16_t Driver6_RcvCount; //���ռ���
extern uint16_t Com4_RcvCount;

extern uint16_t C_Driver1_Pos1_Delay;
extern uint16_t C_Driver1_Pos2_Delay;
extern uint16_t C_Driver2_Pos1_Delay;
extern uint16_t C_Driver2_Pos2_Delay;

extern uint32_t T_Driver1_delay;	  //��ʱ��ʱ��
extern uint32_t C_Driver1_delayCount; //��ʱ����
extern uint32_t T_Driver2_delay;
extern uint32_t C_Driver2_delayCount;
extern uint32_t T_Driver3_delay;
extern uint32_t C_Driver3_delayCount;
extern uint32_t T_Driver4_delay;
extern uint32_t C_Driver4_delayCount;
extern uint32_t T_Driver5_delay;
extern uint32_t C_Driver5_delayCount;
extern uint32_t T_Driver6_delay;
extern uint32_t C_Driver6_delayCount;

extern uint8_t Driver1_Write_Sort; //������1дλ�ö�˳��0д��1�Σ�1д��2��
extern uint8_t Driver2_Write_Sort;
extern uint8_t Driver3_Write_Sort;
extern uint8_t Driver4_Write_Sort;
extern uint8_t Driver5_Write_Sort;
extern uint8_t Driver6_Write_Sort;

extern uint8_t K_StopRun; // ����ֹͣ
uint8_t Old_K_StopRun;

extern uint8_t F_Driver1_notBrake; //1#�ŷ�ɲ���ź�
extern uint8_t F_Driver2_notBrake;
extern uint8_t F_Driver3_notBrake;
extern uint8_t F_Driver4_notBrake;
extern uint8_t F_Driver5_notBrake;
extern uint8_t F_Driver6_notBrake;
extern uint8_t F_Driver_All_notBrake;

/* Private function prototypes -----------------------------------------------*/
void I2C_EE_ByteWrite(uint8_t *pBuffer, uint8_t WriteAddr);
void I2C_EE_BufferRead(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
void ParArrayRead_DWord(uint32_t *p_Top, uc32 *p_Base, uint16_t w_ReadSize);

uint16_t CRC16(uint8_t *pCrcData, uint8_t CrcDataLen);

//uint16_t STMFLASH_ReadHalfWord(uint32_t faddr);		  //��������
uint32_t STMFLASH_ReadWord(uint32_t faddr);
void STMFLASH_WriteLenByte(uint32_t WriteAddr, uint32_t DataToWrite, uint16_t Len); //ָ����ַ��ʼд��ָ�����ȵ�����
uint32_t STMFLASH_ReadLenByte(uint32_t ReadAddr, uint16_t Len);						//ָ����ַ��ʼ��ȡָ����������
void STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint16_t NumToWrite);	//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint16_t NumToRead);		//��ָ����ַ��ʼ����ָ�����ȵ�����

//������в�����ʼ��
void ParLst_Init_Group(void);
//����λ�ü�λ������
void ParLst_Init_Group2Zero(void);
//����λ��
void ParLst_Init_Pos2Zero(void);

uc16 w_ParBootLst[FLASH_PAR_SIZE] = {
	1500, 1500, 1500, 100, 50,	 // 0-4		0=�ŷ����1��ʼλ��,1=�ŷ����2��ʼλ��,2=�ŷ����3��ʼλ��
								 //			3=��ˮ����ѹ��,4=��ˮͣ��ѹ��
	10, 1, 60, 5, 0,			 // 5-9		5=��ˮ������ʱ,6=�������ѡ��
								 //			7=��Ƶת��Ƶʱ�� 8=��Ƶ�˳�ʱ��	9=�޸Ĳ���
	120, 2, 2, 240, 5,			 // 10-14	10=��СƵ�� 11=�豸ͨѶ��ַ 12=ˮ������
								 //			13=�������� 14=��Ƶ����λ����
	1, 0, 0, 0, 0,				 // 15-19	15=���� ����/ֹͣ 16=���� ��λ 17=ǿ�Ƹ�λ
								 //			18=ʹ��Ѳ�칦�� 19=������� Pw_EnablePlayV
	6, 0, 0x5A, 0, 1,			 // 20-24	20=Pw_BetweenPlaySec 21=��Ƶ��ͨѶ��ַ 22=��ʼ����־
								 //			23=������ʼ�� 24=����λ�ÿ���������ѡ��1-5�飩
	1, 1000, 10000, 0, 5000,	 // 25-29	25=�������ѡ�� 26=λ��д������ʱ  27=1#�ֶ������ٶ�
								 //			28=1#�ֶ����з��� 29=2#�ֶ���������
	10000, 0, 5000, 10000, 0,	 // 30-34	30-2#�ֶ������ٶ�	31=2#�ֶ����з���
								 //			32=3#�ֶ���������
								 //			33=3#�ֶ������ٶ� 34=3#�ֶ����з���
	5000, 10000, 0, 1500, 1,	 // 35-39	35=4#�ֶ��������� 36=4#�ֶ������ٶ� 37=4#�ֶ����з���
								 //			38=1#�ŷ����λ�� 39=1#�ŷ�����ٶ�
	20, 1500, 1, 20, 0,			 // 40-44	40=1#�ŷ������ʱ 41=2#�ŷ����λ��
								 //			42=2#�ŷ�����ٶ�
								 //			43=2#�ŷ������ʱ 44=1#�ֶ�����ʹ��
	0, 0, 0, 0, 0,				 // 45-49	45=2#�ֶ�����ʹ�� 46=3#�ֶ�����ʹ�� 47=4#�ֶ�����ʹ��
								 //			48=1#�ŷ�����ֶ�����ʹ�� 49= 2#�ŷ�����ֶ�����ʹ��
								 //
	1, 2, 800, 1500, 1,			 // 50-54	50=�ֶ�ģʽ��1�����Զ�ģʽ��0�� 51=���� 52=PID_Kc
								 //			53=3#�ŷ����λ�� 54=3#�ŷ�����ٶ�
	20, 1, 0, 0, 0,				 // 55-59	55=3#�ŷ������ʱ 56= 3#�ŷ�����ֶ�����ʹ��  /57=SM510ģ��1��ַ
								 //			58=ģ����Ƶ���� /59=SM510ģ��2��ַ
	30, 500, 2500, 500, 2500,	 // 60-64	60=����ˮѹ������ʱ 61=1#�ŷ������Сλ�� 62=1#�ŷ�������λ��
								 //			63=2#�ŷ������Сλ�� 64=2#�ŷ�������λ��
	500, 2500, 630, 830, 400,	 // 65-69	65=3#�ŷ������Сλ�� 66=3#�ŷ�������λ��  67=��ʱ1����
								 //			68=��ʱ1ֹͣ 69=��ʱ1�趨ѹ��  66��84 6�鶨ʱʱ��
	1100, 1300, 400, 1700, 2200, // 70-74	70=��ʱ2���� 71=��ʱ2ֹͣ 72=��ʱ2�趨ѹ��
								 //			73=��ʱ3���� 74=��ʱ3ֹͣ
	400, 0, 0, 0, 0,			 // 75-79	75=��ʱ3�趨ѹ�� 76=��ʱ4���� 77=��ʱ4ֹͣ
								 //			78=��ʱ4�趨ѹ�� 79=��ʱ5����
	0, 0, 0, 0, 0,				 // 80-84	80=��ʱ5ֹͣ 81=��ʱ5�趨ѹ�� 82=��ʱ6����
								 //			83=��ʱ6ֹͣ 84=��ʱ6�趨ѹ��
	0, 0, 0, 0, 0,				 // 85-89	85=����ͣ������ 86=����ͣ������ 87=��ˮ��ѹ����������ֵ
								 //			88=��ˮ��ѹ����������ֵ 89=С��������ѹ��1ʹ��
	400, 3, 3, 0, 30,			 // 90-94	90-93����
								 //			93=�������˱�ʹ�� 94=��ˮ�ڴ��ڳ�ˮ��ѹ����ʱ
	30, 1, 1, 1, 1,				 // 95-99	95=���ˮ������ʱ 96=��ˮ�ڴ����趨ѹ��ʹ��
								 //			97=��ˮ�ڴ��ڳ�ˮ��ѹ��ʹ��
								 //			98=���ˮ����ʹ�� 99=��ʱ����ʹ��
								 //
	0, 0, 0, 0, 0,				 // 100-104	100=     101=1#�����ǰ����  102=2#�����ǰ����
								 //			103=3#�����ǰ���� 104=4#�����ǰ����
	20, 0, 0, 0, 0,				 // 105=�������ȶ���Ŀ	106=2#�ŷ������ǰλ��
								 //			107=3#�ŷ������ǰλ��
								 //			108=    109=
	0, 0, 0, 0, 0,				 // 110-114	110=  111=1#�����ǰִ�е�������� 112=2#�����ǰִ�е��������
								 //			113=3#�����ǰִ�е�������� 114=4#�����ǰִ�е��������
	0, 0, 0, 0, 0,				 // 115-119	115=1#�ŷ������ǰִ�е�������� 116=1#�ŷ���������OK��־
								 //			117=2#�ŷ���������OK��־
								 //			118=3#�ŷ���������OK��־ 119=4#�ŷ���������OK��־
	0, 0, 0, 0, 0,				 // 120-124	120=5#�ŷ���������OK��־ 121=6#�ŷ���������OK��־ 122=�����ӻ�ѡ��
								 //			123=1#�ŷ�����ֶ�����ʹ�� 124= 2#�ŷ�����ֶ�����ʹ��
	200, 250, 0, 2400, 2400,	 // 125-129	125=�½�PID_Ts 126=�½�PID_Ti 127=0 0:0���㽻��
								 //							//			128=2400(�޶���) 129=2400(�޶���)
	15, 12, 2000, 300, 5000,	 // 130-134	130=��Ƶת��Ƶƫ�� 131=��Ƶ�˳�ƫ�� 132=2000 ������ֵ
								 //			133=��С����300 134=������� 5000

	3, 7, 180, 1, 70,			  // 135-139	135=��Сѹ��300 136=���ѹ��500 137=������ѹʹ��0
								  //			138=��Ƶת��Ƶʹ�� 139= Cpu����ЧӦ�¶�
	5, 57600, 57600, 57600, 1800, // 140-144	140=ѹ��������������ʱ �� 141=����1������ 142=��2������  ZCL 2018.12.8
								  //			143=��3������ 144= С�����ƻ������
	0, 17, 600, 1, 1,			  // 145-149	145= 146= 147=
								  //			148= 149=
								  //
	50, 20, 57600, 57600, 2,	  // 150-154	150= 151= 152=
								  //			153= 154=
	1, 1, 0, 0, 1,				  // 155-159	155= 156= 157=
								  //			158= 159=1#�ŷ����Modbus��ַ
	2, 3, 4, 5, 6,				  // 160-164	160=2#�ŷ����Modbus��ַ 161=3#�ŷ����Modbus��ַ 162=4#�ŷ����Modbus��ַ
								  //			163=5#�ŷ����Modbus��ַ 164=6#�ŷ����Modbus��ַ
	2, 1, 50, 250, 60,			  // 165-169	165= 166=  166=
								  //			168=  169=
	1, 500, 50, 8760, 5,		  // 170-174	170= 171= 172=
								  //			173= 174=
	400, 5, 1, 120, 0,			  // 175-179	175= 176= 177=
								  //			178= 179=����ָ�����
	0, 0, 0, 1, 1,				  // 180-184	180=���ж���Ȧ�趨 181=�ۼ�����Ȧ�� 182=�ۼ�����Ȧ��_����
								  //			183=Com5ͨѶ���� 184=ͨѶ����д�����ͣ�����ܣ�=1��ͣ����=0����ͣ��
	100, 50, 10, 1, 3,			  // 185-189	185= 186= 187=
								  //			188= 189=
	200, 50, 300, 1800, 0,		  // 190-194	190= 191= 192=
								  //			193= 194=
	2, 10, 10, 0, 0,			  // 195-199	195= 196= 197=
								  //			198= 199=
								  //
	10, 200, 20, 0, 400,		  // 200-204	200= 201= 202=
								  //			203= 204=
	0, 1, 1000, 1, 4,			  // 205-209	205= 206= 207=��ѯ���ʱ�䣬ms
								  //			208= 209=
	1, 1, 1, 1, 1,				  // 210-214	210= 211=COM1������ѯʹ�� 212=COM2������ѯʹ��
								  //			213=COM3������ѯʹ�� 214=COM4������ѯʹ��
	10, 10, 0, 0, 0,			  // 215-219	215= 216= 217=
								  //			218= 219=
	339, 0, 1, 0, 1,			  // 220-224	220= 221= 222=ͨѶ����ͣ�����ܣ�=1��ͣ����=0����ͣ��
								  //			223= 224=���λ�����ƹ��ܣ�=1�����ã�=0��������
	15, 0, 0, 0, 0,				  // 225-229	225= 226=1#ͨѶ���ϱ�־ 227=2#ͨѶ���ϱ�־
								  //			228=3#ͨѶ���ϱ�־ 229=4#ͨѶ���ϱ�־
	0, 0, 10, 10, 10,			  // 230-234	230=5#ͨѶ���ϱ�־ 231=6#ͨѶ���ϱ�־ 232=1#�����Сλ��
								  //			233=1#������λ�� 234=2#�����Сλ��
	10, 10, 10, 10, 10,			  // 235-239	235=2#������λ�� 236=3#�����Сλ�� 237=3#������λ��
								  //			238=4#�����Сλ�� 239=4#������λ��
								  //---------------------------
	10, 10, 10, 10, 0,			  // 240-244	240=5#�����Сλ�� 241=5#������λ�� 242=6#�����Сλ��
								  //			243=6#������λ�� 244=ɲ��״̬
	1, 1, 0, 0, 0,				  // 245-249	245=ɲ�����ƣ�=1��ɲ����=0����ɲ��  246=���е����ֹͣ��־ 247=Com4ͨѶ���ϱ�־
								  //			248=ɲ������ģʽ��=1���ֶ�����ɲ����=0���Զ�����ɲ�� 249=
	0, 0, 0, 0, 0,				  // 250-254	250=  251= 252=
								  //			253= 254=

};	//0,   1,    2,    3,    4,	//��������
	//5,   6,    7,    8,    9,	//��������

//��ַ+5000���ʶ�Ӧ
uc32 w_ParBootLst_Drive[START_CMD_ADDR] = {
	10000, 0, 10000, 0, 10000, // 0-4		0=1#�ֶ��������壨���֣�,1=1#�ֶ��������壨���֣�,2=2#�ֶ��������壨���֣�
							   //			3=2#�ֶ��������壨���֣�,4=3#�ֶ��������壨���֣�
	0, 10000, 0, 10000, 0,	   // 5-9		5=3#�ֶ��������壨���֣�,6=4#�ֶ��������壨���֣�
							   //			7=4#�ֶ��������壨���֣�  8=5#�ֶ��������壨���֣� 	9=5#�ֶ��������壨���֣�
	10000, 0, 0, 0, 0,		   // 10-14	9=6#�ֶ��������壨���֣� 	10=6#�ֶ��������壨���֣�  12=
							   //			13=  14=
	0, 0, 0, 0, 0,			   // 15-19	15=  16=  17=6#�ŷ����״̬��
							   //			18= =1����ʼ������  19=
	0, 0, 0, 0, 0,			   // 20-24	20=  21=1#�ֶ�����ʹ�ܣ���ת��  22=1#�ֶ�����ʹ�ܣ���ת��
							   //			23=1#�ֶ�����ʹ�ܣ���ת��  24=1#�ֶ�����ʹ�ܣ���ת��
	0, 0, 200, 1000, 0,		   // 25-29	25=1#�ֶ�����ʹ�ܣ���ת��  26=1#�ֶ�����ʹ�ܣ���ת��   27=1#�ֶ������ٶ�
							   //			28=1#�ֶ��Ӽ���ʱ�� 29=
	200, 1000, 0, 200, 1000,   // 30-34	30-2#�ֶ������ٶ�	31=2#�ֶ��Ӽ���ʱ��
							   //			32=  33=3#�ֶ������ٶ� 34=3#�ֶ��Ӽ���ʱ��
	0, 200, 1000, 0, 200,	   // 35-39	35=  36=4#�ֶ������ٶ� 37=4#�ֶ��Ӽ���ʱ��
							   //			38=  39=5#�ֶ������ٶ�
	1000, 0, 200, 1000, 0,	   // 40-44	40=5#�ֶ��Ӽ���ʱ�� 41=
							   //			42=6#�ֶ������ٶ� 43=6#�ֶ��Ӽ���ʱ�� 44=1#�ֶ�����ʹ��
	0, 0, 0, 0, 0,			   // 45-49	45=2#�ֶ�����ʹ�� 46=3#�ֶ�����ʹ�� 47=4#�ֶ�����ʹ��
							   //			48=5#�ֶ�����ʹ�� 49= 6#�ֶ�����ʹ��
	1, 0, 0, 0, 0,			   // 50-54	50=�ֶ�ģʽ��1�����Զ�ģʽ��0�� 51=  52=
							   //			53=  54=
	0, 0, 0, 0, 0,			   // 55-59	55=  56=    /57=
							   //			58=  /59=
	0, 0, 0, 0, 0,			   // 60-64	60=  61=  62=
							   //			63=�ŷ����1�ĳ�ʼλ�� 64=
	0, 0, 0, 0, 0,			   // 65-69	65=�ŷ����2�ĳ�ʼλ�� 66=   67=�ŷ����3�ĳ�ʼλ��
							   //			68=  69=�ŷ����4�ĳ�ʼλ��
	0, 0, 0, 0, 0,			   // 70-74	70=  71=�ŷ����5�ĳ�ʼλ�� 72=�ŷ����5�ĳ�ʼλ��
							   //			73=�ŷ����6�ĳ�ʼλ�� 74=
	1, 0, 0, 0, 0,			   // 75-79	75=��������ͣ������  76=����ǰλ��  77=��ͣ���=1�����м�ͣ
							   //			78=��λ���=1�����и�λ  79=ͣ����λ����
	1, 180, 0, 0, 0,		   // 80-84	80=���� ����/ֹͣ 81=Ĭ����ʱ180s�����޸ĺ�Ĳ�����FLASH�� 82=1#�ŷ����������ʼ����־
							   //			83=2#�ŷ����������ʼ����־ 84=3#�ŷ����������ʼ����־
	0, 0, 0, 0x3880, 0x01,	   // 85-89	85=4#�ŷ����������ʼ����־ 86=5#�ŷ����������ʼ����־ 87=6#�ŷ����������ʼ����־
							   //			88=��λ����޶�ֵ  89=
	0, 0, 0, 0, 0,			   // 90-94	90-1#�ŷ����λ��ָ������� 91= 92-2#�ŷ����λ��ָ�������
							   //			93= 94=3#�ŷ����λ��ָ�������
	0, 0, 0, 0, 0,			   // 95-99	95= 96=4#�ŷ����λ��ָ�������
							   //			97= 98=5#�ŷ����λ��ָ������� 99=
	0, 0, 0, 0, 0,			   // 100-104	100=6#�ŷ����λ��ָ�������     101=   102=
							   //			103=  104=
	0, 0, 0, 100, 0,		   // 105-  	106=
							   //			107= 			108=ͨѶ������ʱ�ж� 10ms   109=
	0, 0, 0, 0, 0,			   // 110-114	110=  111=1#�����ǰִ�е�������� 112=2#�����ǰִ�е��������
							   //			113=3#�����ǰִ�е�������� 114=4#�����ǰִ�е��������
	0, 0, 0, 0, 0,			   // 115-119	115=5#�����ǰִ�е�������� 116=6#�����ǰִ�е��������
							   //			117=1#������б�־
							   //			118=2#������б�־ 119=3#������б�־
	0, 0, 0, 0, 0,			   // 120-124	120=4#������б�־ 121=5#�ŷ�������б�־ 122=6#�ŷ�������б�־
							   //			123=1#�ŷ����׼���ñ�־ 124= 2#�ŷ����׼���ñ�־
	0, 0, 0, 0, 0,			   // 125-129	125=3#�ŷ����׼���ñ�־ 126=4#�ŷ����׼���ñ�־ 127=5#�ŷ����׼���ñ�־
							   //			128=6#�ŷ����׼���ñ�־ 129=1#�ŷ����ͨѶ����
	0, 0, 0, 0, 0,			   // 130-134	130=2#�ŷ����ͨѶ����  131=3#�ŷ����ͨѶ����  132=4#�ŷ����ͨѶ����
							   //			133=5#�ŷ����ͨѶ����  134=6#�ŷ����ͨѶ����
	0, 0, 0, 0, 0,			   // 135-139	135=1#�ŷ����״̬�� 136=2#�ŷ����״̬�� 137=3#�ŷ����״̬��
							   //			138=4#�ŷ����״̬�� 139=5#�ŷ����״̬��
	0, 0, 0, 0, 0,			   // 140-144	140=6#�ŷ����״̬�� 141=  142=
							   //			143=  144=
	0, 0, 0, 0, 0,			   // 145-149	145=  146=  147=1#�ŷ������ǰ��߼��������
							   //			148=2#�ŷ������ǰ��߼�������� 149=3#�ŷ������ǰ��߼��������
	0, 0, 0, 2, 1,			   // 150-154	150=4#�ŷ������ǰ��߼�������� 151=5#�ŷ������ǰ��߼�������� 152=6#�ŷ������ǰ��߼��������
							   //			153=COM��ʱ1  154=COM��ʱ2
	0, 0, 0, 0, 0,			   // 155-159	155=   156=   157=
							   //			158=   159=
	0, 0, 0, 0, 0,			   // 160-164	160=  161=1#�ŷ������������Ȧ����  162=2#�ŷ������������Ȧ���ݣ����֣�
							   //			163=2#�ŷ������������Ȧ����  164=3#�ŷ������������Ȧ���ݣ����֣�
	0, 0, 0, 0, 0,			   // 165-169	165=3#�ŷ������������Ȧ����  166=4#�ŷ������������Ȧ���ݣ����֣�  167=4#�ŷ������������Ȧ����
							   //			168=1#�ŷ������������Ȧ���ݣ����֣�  169=5#�ŷ������������Ȧ����
	0, 0, 0, 0, 0,			   // 170-174	170=5#�ŷ������������Ȧ���ݣ����֣�    171=6#�ŷ������������Ȧ����  172=6#�ŷ������������Ȧ���ݣ����֣�
							   //			173=1#�ŷ������������Ȧ����  174=2#�ŷ������������Ȧ����
	0, 0, 0, 0, 0,			   // 175-179	175=3#�ŷ������������Ȧ����  176=4#�ŷ������������Ȧ����  177=5#�ŷ������������Ȧ����
							   //			178=6#�ŷ������������Ȧ����  179=1#�ŷ������������Ȧ��ʼ�趨ֵ

	0, 0, 0, 0, 0, // 180-184	180=1#�ŷ������������Ȧ���趨ֵ�����֣�    181=2#�ŷ������������Ȧ��ʼ�趨ֵ  182=2#�ŷ������������Ȧ���趨ֵ�����֣�
				   //			183=3#�ŷ������������Ȧ��ʼ�趨ֵ  184=3#�ŷ������������Ȧ���趨ֵ�����֣�
	0, 0, 0, 0, 0, // 185-189	185=4#�ŷ������������Ȧ��ʼ�趨ֵ  186=4#�ŷ������������Ȧ���趨ֵ�����֣�  187=5#�ŷ������������Ȧ��ʼ�趨ֵ
				   //			188=5#�ŷ������������Ȧ���趨ֵ�����֣�  189=6#�ŷ������������Ȧ��ʼ�趨ֵ

	0, 0, 0, 0, 0,				  // 190-194	190=6#�ŷ������������Ȧ���趨ֵ�����֣�    191=1#�ŷ������������Ȧ��ʼ�趨ֵ  192=2#�ŷ������������Ȧ��ʼ�趨ֵ
								  //			193=3#�ŷ������������Ȧ��ʼ�趨ֵ  194=4#�ŷ������������Ȧ��ʼ�趨ֵ
	0, 0, 200, 5000, 0,			  // 195-199	195=5#�ŷ������������Ȧ��ʼ�趨ֵ  196=6#�ŷ������������Ȧ��ʼ�趨ֵ  197= ��λ����ʼλ����ʱʱ�䣬10ms
								  //			198=λ��ƫ���趨  199=λ��ƫ���趨�����֣�
	0, 0, 0, 0, 0,				  // 200-204	200=   201=1#�ŷ����P8910����  202=2#�ŷ����P8910����
								  //			203=3#�ŷ����P8910����  204=4#�ŷ����P8910����
	0, 0, 0, 0, 0,				  // 205-209	205=5#�ŷ����P8910����  206=6#�ŷ����P8910����  207= �����ŷ����дλ�ò���ʹ��
								  //			208=1#�ŷ����дλ�ò���ʹ��  209=2#�ŷ����дλ�ò���ʹ��
	0, 0, 0, 0, 1000,			  // 210-214	210=3#�ŷ����дλ�ò���ʹ��   211=4#�ŷ����дλ�ò���ʹ��  212=5#�ŷ����дλ�ò���ʹ��
								  //			213=6#�ŷ����дλ�ò���ʹ��  214=1#�ŷ�����趨��������ٶ�
	1000, 1000, 1000, 1000, 1000, // 215-219	215=2#�ŷ�����趨��������ٶ�  216=3#�ŷ�����趨��������ٶ�  217=4#�ŷ�����趨��������ٶ�
								  //			218=5#�ŷ�����趨��������ٶ�  219=6#�ŷ�����趨��������ٶ�
	5, 6, 0, 0, 0,				  // 220-224	220=COM1��ʱ3  10ms 221=COM��ʱ���ֶ� 10ms     222=���岢����λ������
								  //			223=����λ�ü��ٶ�����   224=У��λ������
	0, 0, 0, 1000, 0,			  // 225-229	225=�������е������   226=�ֶ���������   227=��ǰλ�ú�[1-15]
								  //			228=��׼�����ٶ�   229=1����ԭ��λ������
	0, 0, 0, 0, 0,				  // 230-234	230=1#��������е�����   231=2#��������е�����   232=3#��������е�����
								  //			233=4#��������е�����   234=5#��������е�����
	0, 1000, 0, 0, 0,			  // 235-239	235=6#��������е�����   236=ֹͣ״̬�����ʱ   237=�Ӽ���ʱ����ʱ����
								  //			238=��ǰָ������ʱ��   239=�豸״̬
	0, 0, 0, 0, 0,				  // 240-244	240=1#��ǰ�����ٶ�   241=2#��ǰ�����ٶ�   242=3#��ǰ�����ٶ�
								  //			243=4#��ǰ�����ٶ�   244=5#��ǰ�����ٶ�
	0, 0, 0, 0, 3,				  // 245-249	245=6#��ǰ�����ٶ�   246=�豸״̬������    247=�豸״̬������
								  //			248=����ģʽ��=0������ģʽ��=1����ϴģʽ   249=�̵�����ͨѶ��ַ
	0, 0, 0, 300, 0,			  // 250-254	250=�̵�����DO1������    251=�̵�����DO1״̬��    252=Com4ͨѶ����
								  //			253=��ɲ����ʱ    254=
	0, 0, 20, 40, 1,			  // 255-259	255=    256=     257=����ѭ���������
								  //			258=�������С�����ٶ�    259= 1#�����û�����б�־
	1, 1, 1, 1, 1,				  // 260-264	260=2#�����û�����б�־    261=3#�����û�����б�־    262=4#�����û�����б�־
								  //			263=5#�����û�����б�־    264= 6#�����û�����б�־
	0, 0, 0, 0, 0,				  // 265-269	265=1#���������λ��ƫ���Ȧ    266=2#���������λ��ƫ���Ȧ     267=3#���������λ��ƫ���Ȧ
								  //			268=4#���������λ��ƫ���Ȧ     269=5#���������λ��ƫ���Ȧ
	0, 0, 0, 0, 0,				  // 270-274	270=6#���������λ��ƫ���Ȧ    271=1#���������λ��ƫ���Ȧ����λ    272=1#���������λ��ƫ���Ȧ����λ
								  //			273=2#���������λ��ƫ���Ȧ����λ     274=2#���������λ��ƫ���Ȧ����λ
	0, 0, 0, 0, 0,				  // 275-279	275=3#���������λ��ƫ���Ȧ����λ     276=3#���������λ��ƫ���Ȧ����λ      277=4#���������λ��ƫ���Ȧ����λ
								  //			278=4#���������λ��ƫ���Ȧ����λ      279=5#���������λ��ƫ���Ȧ����λ
	0, 0, 0, 0, 50,				  // 280-284	280=5#���������λ��ƫ���Ȧ����λ    281=6#���������λ��ƫ���Ȧ����λ    282=6#���������λ��ƫ���Ȧ����λ
								  //			283=���������λ��ƫ���ֶ���������     284=�ж����е��λ�õ�����ʱ
	2, 0, 0, 0, 0,				  // 285-289	285=дλ��ָ�ʱʱ���趨 s    286=�������ݱ�־      287=1#�ŷ���������OK��־
								  //			288=2#�ŷ���������OK��־      289=3#�ŷ���������OK��־
	0, 0, 0, 0, 0,				  // 290-294	290=4#�ŷ���������OK��־    291=5#�ŷ���������OK��־   292=6#�ŷ���������OK��־
								  //			293=�����ŷ���������OK��־     294=���ŷ��������ݴ����־
	0, 0, 0, 0, 0,				  // 295-299	295=����λ�÷�Χ��־    296=      297=
								  //			298=      299=
};								  //0,   1,    2,    3,    4,	//��������
								  //5,   6,    7,    8,    9,	//��������
/* Private functions ---------------------------------------------------------*/
//
void ReadWriteRealTime(void) // ��дʵʱʱ�� ISL1208
{
	uint8_t i;
	uint16_t k;
	uint8_t RWBuf[7]; //I2C ��д����
	// ��ʱ��
	if (T_ReadRealTime != SClk10Ms && !F_ModRealTime)
	{
		T_ReadRealTime = SClk10Ms; //
		C_ReadRealTime++;
		if (C_ReadRealTime > 20)
		{
			T_ReadRealTime = 100; //
			C_ReadRealTime = 0;
			//Read1302Time(RWBuf);	// Read address 0x00 on ISL1208 ��Ӱ�촮���ٶ�

			//			BurstRead1302Clock(RWBuf);

			//RWBuf[2] &=0x7f;				// Сʱ
			for (i = 0; i < 7; i++) // 16����ת10���Ƹ���ʾ���޸�ʱ10����ת16����
			{
				RWBuf[i] = RWBuf[i] / 16 * 10 + (RWBuf[i] % 16);
			}
			//
			for (i = 0; i < 5; i++) // 7---19  ��ʱ��7���Ĵ�������
			{
				w_ParLst[i + 45] = RWBuf[i];
				RealClock[i] = RWBuf[i];
			}
			// Pw_SetWeek = RWBuf[5];		  // ����
			// Pw_SetYear = RWBuf[6] + 2000; // �꣫2000
			// RealClock[6] = RWBuf[5];	  //����
			// RealClock[5] = RWBuf[6];	  //��
		}
	}

	// �޸�ʱ�ӵľ��崦�����
	if (F_ModRealTime)
	{
		if (T_ModRealTime != SClk10Ms)
		{
			T_ModRealTime = SClk10Ms; //
			C_ModRealTime++;
			if (C_ModRealTime > 10 && S_ModRealTime == 0)
			{
				RWBuf[0] = 0x90;
				//I2C_EE_ByteWrite(RWBuf, 0x07);	// ����дʱ��
				S_ModRealTime = 1;
			}
			if (C_ModRealTime > 20 && S_ModRealTime == 1)
			{
				k = w_ParLst[45 + w_ModRealTimeNo]; // k Ϊ�޸�����
				//
				if (w_ModRealTimeNo == 5) // ��
				{
					k = k - 2000;
					w_ModRealTimeNo++; //�ܳ��� 2015.5.14 ����DS1302
				}
				else if (w_ModRealTimeNo == 6) //��
				{
					w_ModRealTimeNo = w_ModRealTimeNo - 1;
				}
				else if (w_ModRealTimeNo == 2)
				{
					//k |=0x80;  ISL1208 .7=1Ϊ24Сʱ��
				}

				k = k / 10 * 16 + (k % 10); // �޸�ʱ16����ת10����

				RWBuf[0] = k;
				//				Write1302ByteTime(w_ModRealTimeNo,RWBuf[0]);		// �޸�ʱ��
				S_ModRealTime = 2;
			}
			if (C_ModRealTime > 30 && S_ModRealTime == 2)
			{
				RWBuf[0] = 0x80;
				//I2C_EE_ByteWrite(RWBuf, 0x07);	// ��ֹд
				S_ModRealTime = 0;
				T_ModRealTime = 100; //
				C_ModRealTime = 0;
				F_ModRealTime = 0;
			}
		}
	}
}

void Variable_Init(void) //	������ʼ��
{
	int i;
	s32 *arrp;

	w_SoftVer = 100;	// ����汾�� VERSION
	w_WriteDate = 2021; //�����д����
	w_Writetime = 0516; //�����дʱ��
						//	w_GongSiSelect=0;               //  ��˾ѡ��0=��de����1=��������

	F_ManualRunStop = 1;

	F_Starting = 0;
	F_Stoping = 0;
	F_AskReset = 0;
	F_Reseting = 0;
	F_AskForceReset = 0;
	F_ForceReseting = 0;
	F_TouchForceReSet = 0;

	F_PowerOnRun = 1;
	F_AskStop = 1;
	Pw_StepAutoMode = 0; //�ϵ磬=0���Զ�״̬
	F_StepMode = 0;

	Pw_TouchRunStop = 1; //�ϵ磬=1��ֹͣ
	F_TouchRunStop = 1;

	arr_p1 = &w_ParLst_Pos_CMD; //ָ��λ���������ͷ
	arrp_p1_Last = arr_p1;
	Pr_Driver_Running_No = 0;
	Pr_Driver_Previous_No = 0; //ǰһ��ִ��ָ���

	Pr_F_Drive1_Runing = 0;
	Pr_F_Drive2_Runing = 0;
	Pr_F_Drive3_Runing = 0;
	Pr_F_Drive4_Runing = 0;
	Pr_F_Drive5_Runing = 0;
	Pr_F_Drive6_Runing = 0;

	F_Encoder_Read = 0; //=0����������ʼ���ݻ�δ��ȡ
	Pw_ResetCMD = 0;
	Pw_Read_CurrentPos = 0; //=1,����ǰλ��
	Pw_Read_Init_Pos = 0;	//=1,��ԭ��λ��

	Pr_Drive1_Status = 0;
	Pr_Drive2_Status = 0;
	Pr_Drive3_Status = 0;
	Pr_Drive4_Status = 0;
	Pr_Drive5_Status = 0;
	Pr_Drive6_Status = 0;

	Pr_Driver1_ComCount = 0;
	Pr_Driver2_ComCount = 0;
	Pr_Driver3_ComCount = 0;
	Pr_Driver4_ComCount = 0;
	Pr_Driver5_ComCount = 0;
	Pr_Driver6_ComCount = 0;

	Pr_Drive1_FaultNo = 0;
	Pr_Drive2_FaultNo = 0;
	Pr_Drive3_FaultNo = 0;
	Pr_Drive4_FaultNo = 0;
	Pr_Drive5_FaultNo = 0;
	Pr_Drive6_FaultNo = 0;

	F_AllRdy = 0;
	Pr_F_AllStopped = 1; //�ϵ�Ĭ�����е����ֹͣ
	Pr_F_HaveFault = 0;
	Pw_Step_Pos_CMD = 0; //��������ģʽ=0

	Pr_BRAKE_Control = 1; //�ϵ�Ĭ��ɲ��
						  //	START_BRAKE_SYSTEM;				//ɲ��

	T_Driver1_delay = 0;
	C_Driver1_delayCount = 0;
	T_Driver2_delay = 0;
	C_Driver2_delayCount = 0;
	T_Driver3_delay = 0;
	C_Driver3_delayCount = 0;
	T_Driver4_delay = 0;
	C_Driver4_delayCount = 0;
	T_Driver5_delay = 0;
	C_Driver5_delayCount = 0;
	T_Driver6_delay = 0;
	C_Driver6_delayCount = 0;

	Pw_EquipStatus = 0; //�豸״̬

	Driver1_Write_Sort = 0; //������1дλ�ö�˳��0д��1�Σ�1д��2��
	Driver2_Write_Sort = 0;
	Driver3_Write_Sort = 0;
	Driver4_Write_Sort = 0;
	Driver5_Write_Sort = 0;
	Driver6_Write_Sort = 0;

	Pw_Run_Mode = 0; //=0��Ĭ��Ϊ����ģʽ
	F_SendStopCMD2 = 0;

	arrp = &w_ParLst_PosPar;
	for (i = 1; i <= 15; i++)
	{
		arrp[POS_SIZE * (i - 1)] = i; //λ�������е�1���ּ�Ϊλ����ţ���15��λ��,1-15
	}

	arrp = &w_ParLst_Pos_CMD;
	for (i = 1; i <= POS_CMD_NUM; i++)
	{
		arrp[POS_CMD_SIZE * (i - 1)] = i; //���������е�1���ּ�Ϊ��ţ���30������,1-30
	}

	Pr_Driver1_NeverRun = 1;
	Pr_Driver2_NeverRun = 1;
	Pr_Driver3_NeverRun = 1;
	Pr_Driver4_NeverRun = 1;
	Pr_Driver5_NeverRun = 1;
	Pr_Driver6_NeverRun = 1;

	Pr_Driver1_Control_OK_F = 0;
	Pr_Driver2_Control_OK_F = 0;
	Pr_Driver3_Control_OK_F = 0;
	Pr_Driver4_Control_OK_F = 0;
	Pr_Driver5_Control_OK_F = 0;
	Pr_Driver6_Control_OK_F = 0;

	Pr_AllDriver_Cmd_OK_F = 0;
	Pr_HaveDriver_Cmd_Err_F = 0;
	Pr_OverMaxPos_F = 0;

	C_NoRcv1Count = 0;
	C_NoRcv2Count = 0;
	C_NoRcv3Count = 0;
	C_NoRcv4Count = 0;
	C_NoRcv5Count = 0;
	C_NoRcv6Count = 0;
}

// RAM�в������� ��ʼ�� (����)
void ParLst_Init(void)
{
	//	//������洢��FM25L16�ж����趨��������¼�Ų�����ˮ���ۼ����в���
	//	B_SelCS=CS_FMRAM1;
	//	SPI_FMRAM_BufferRead((uint8_t *)(&w_ParLst[0]), 0, FLASH_PAR_SIZE*2);			// ��FMRAM�ĵ�ַ0��ʼ�����趨������255����
	//	SPI_FMRAM_BufferRead((uint8_t *)(&w_ParLst[512]), 1024, FLASH_ID_SIZE*2);		// �����绰����Ȳ������ӵ�ַ1024��ʼ����128����

	//	//��FLASH ROM�ж�ȡ����Ĳ�����д��w_ParLst_Drive�У����򣺡���>
	//	STMFLASH_Read(FLASH_SAVE_ADDR0,(uint32_t*)&w_ParLst_Drive,START_CMD_ADDR);
	//
	//	//��ȡλ����Ϣ
	//	STMFLASH_Read(FLASH_SAVE_POSTION1,(uint32_t*)&w_ParLst_PosPar,FLASH_POS_SIZE);
	//
	//	//��ȡλ�ÿ���ָ��
	//	//��FLASH_SAVE_POS_CMD1��������д�뵽w_ParLst_Pos_CMD��		���򣺡���>
	//	if(Pos_Group_Select>5 || Pos_Group_Select<1)
	//	{
	//		Pos_Group_Select=1;
	//		Old_Pos_Group_Select=1;
	//	}
	//	STMFLASH_Read(FLASH_SAVE_POS_CMD1+0X00001000*(Pos_Group_Select-1),(uint32_t*)&w_ParLst_Pos_CMD,FLASH_POS_CMD_SIZE);
}

////��ָ����ַ��ʼ����ָ�����ȵ�����
////ReadAddr:��ʼ��ַ
////pBuffer:����ָ��
////NumToWrite:����(16λ)��
//void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint16_t NumToRead)
//{
//	uint16_t i;
//	for(i=0;i<NumToRead;i++)
//	{
//		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ2���ֽ�.
//		ReadAddr+=4;//ƫ��2���ֽ�.
//	}
//}

void ParArrayRead(uint16_t *p_Top, uc16 *p_Base, uint16_t w_ReadSize)
{
	uint16_t i;
	for (i = 0; i < w_ReadSize; i++)
	{
		*p_Top++ = *p_Base++; // zcl ��ַ�Զ���2��
	}
}

void ParArrayRead_Word(uint32_t *p_Top, uc32 *p_Base, uint w_ReadSize)
{
	uint16_t i;
	for (i = 0; i < w_ReadSize; i++)
	{
		*p_Top++ = *p_Base++; // zcl ��ַ�Զ���4��
	}
}

//void ParArrayRead_DWord(uint32_t *p_Top,uc32 *p_Base, uint16_t w_ReadSize)
//{
//	uint16_t i;
//	for (i=0;i<w_ReadSize ;i++ )
//	{
//		*p_Top++ = *p_Base++;	// zcl ��ַ�Զ���4��
//	}
//}

//�ָ��趨����Ϊ����ֵ
//ֻ��STMFLASH_Read���������Ǵ�������д������>
//�����ģ���STMFLASH_Write��ParArrayRead�����Ǵ�������д��<����
void Boot_ParLst(void) // �ָ��趨����Ϊ����ֵ
{
	uint8_t i;
	uint8_t tmp_Group_Pos;

	i = 0;

	//���!=0x5A������г�ʼ��
	if (Pw_Initial_F != 0x5A)
	{
		Pw_ModPar = 2000;
		Pw_ParInitial = 4321;
		S_BootParLst = 0;
		Pw_Initial_F = 0x5A;
	}
	// Pw_ModPar=2000,�����޸Ĳ���// Pw_ParInitial==4321 ��ʼ������
	if (Pw_ModPar == 2000 && Pw_ParInitial == 4321 && S_BootParLst == 0)
	{
		i = 1;
	}

	if (i == 1)
	{
		//��ʼ��ϵͳ����
		//RAM<������������
		//�����������
		if (Pos_Group_Select > 5 || Pos_Group_Select < 1)
			Pos_Group_Select = 1;

		tmp_Group_Pos = Pos_Group_Select;
		ParArrayRead(w_ParLst, w_ParBootLst, FLASH_PAR_SIZE); // �ӳ��������ж�����ʼ������,255����
		Pos_Group_Select = tmp_Group_Pos;

		//��ʼ��������������������FLASH��
		//FLASH<������������
		// STMFLASH_Write(FLASH_SAVE_ADDR0, (uint32_t *)w_ParBootLst_Drive, START_CMD_ADDR); //�ѳ��������е�ֵд�뵽FLASH��
		//RAM<������������
		ParArrayRead_Word((uint32_t *)&w_ParLst_Drive, (uint32_t *)&w_ParBootLst_Drive, START_CMD_ADDR); // ������ʼ������

		arr_p1 = &w_ParLst_Pos_CMD;
		arrp_p1_Last = arr_p1;
		//		arr_p1[37]=0;											//���ͱ�־��0�����Է���λ��

		// �ݲ������������Pw_ModPar==5000�����Ա��������FLASH
		Pw_ModPar = 2000;
		Pw_ParInitial = 4321;
		S_BootParLst = 1;
	}

	//
	if (T_BootParLst != SClk10Ms && S_BootParLst != 0) // ����MD304L��ʾ״̬�����Ի���� ZCL
	{
		T_BootParLst = SClk10Ms; // ����Pw_ParInitial=4321
		C_BootParLst++;
		if (C_BootParLst > 150 && S_BootParLst == 1)
		{
			S_BootParLst = 2;
			Pw_ParInitial = 6000;
		}
		else if (C_BootParLst > 300 && S_BootParLst == 2)
		{
			T_BootParLst = 100;
			C_BootParLst = 0;
			S_BootParLst = 0;
			Pw_ParInitial = 0;
		}
	}
}

//ֻ������г�ʼ��λ������ӱ�����FLASH�е�ĳ�����е��뵽RAM��
//��λ����Ϣ����ʼ��
void ParLst_Init_Group(void)
{
	if(Pw_ModPar==2000 && Pw_ParInitial==4000)
		{
			//�����������
			if(Pos_Group_Select>5 || Pos_Group_Select<1)
				Pos_Group_Select=1;
			//FLASH����>RAM
			STMFLASH_Read(FLASH_SAVE_POS_CMD1+0X00001000*(Pos_Group_Select-1),(uint32_t*)&w_ParLst_Pos_CMD,FLASH_POS_CMD_SIZE);
	
			Pw_ModPar=0;
			Pw_ParInitial=0;
		}
}

//����λ������
void ParLst_Init_Group2Zero(void)
{
	if(Pw_ModPar==2000 && Pw_ParInitial==9876)
	{
		//�����������
		if(Pos_Group_Select>5 || Pos_Group_Select<1)
			Pos_Group_Select=1;
		//����λ������
		//��Driver1_Cmd_Group1�ж�����д�뵽FLASH_SAVE_ADDR1��FLASH��		
		// if(Pos_Group_Select==1)	
		// 	STMFLASH_Write(FLASH_SAVE_POS_CMD1,(u32*)Pos_Cmd_Group1,FLASH_POS_CMD_SIZE);		
		// else if(Pos_Group_Select==2)
		// 	STMFLASH_Write(FLASH_SAVE_POS_CMD2,(u32*)Pos_Cmd_Group2,FLASH_POS_CMD_SIZE);
		// else if(Pos_Group_Select==3)
		// 	STMFLASH_Write(FLASH_SAVE_POS_CMD3,(u32*)Pos_Cmd_Group3,FLASH_POS_CMD_SIZE);	
		// else if(Pos_Group_Select==4)
		// 	STMFLASH_Write(FLASH_SAVE_POS_CMD4,(u32*)Pos_Cmd_Group4,FLASH_POS_CMD_SIZE);
		// else if(Pos_Group_Select==5)
		// 	STMFLASH_Write(FLASH_SAVE_POS_CMD5,(u32*)Pos_Cmd_Group5,FLASH_POS_CMD_SIZE);	

		//FLASH����>RAM����FLASH�ж��������λ���������RAM��
//		ParArrayRead_Word((u32*)&w_ParLst_Pos_CMD,(u32*)Pos_Cmd_Group1,FLASH_POS_CMD_SIZE);
		STMFLASH_Read(FLASH_SAVE_POS_CMD1+0X00001000*(Pos_Group_Select-1),(u32*)&w_ParLst_Pos_CMD,FLASH_POS_CMD_SIZE);		
		
		Pw_ModPar=0;
		Pw_ParInitial=0;
	}	
}

//����λ��
void ParLst_Init_Pos2Zero(void)
{
		if(Pw_ModPar==2000 && Pw_ParInitial==5432)
		{
			//����λ��
			//FLASH<������������
			// STMFLASH_Write(FLASH_SAVE_POSTION1,(uint32_t*)Pos_Init,FLASH_POS_SIZE);
			//RAM<�����������飬��ʼ��λ�ã�����
			ParArrayRead_Word((uint32_t*)&w_ParLst_PosPar,(uint32_t*)Pos_Init,FLASH_POS_SIZE);
	
			Pw_ModPar=0;
			Pw_ParInitial=0;
		}
}

void SavePar_Prompt(void) // �������+״̬��ʾ
{
	//	if ( B_ForceSavPar==1 && S_SavePar==0 )		// ǿ�Ʊ������
	//	{
	//		Pw_ModPar=0;		// ��ֹ�� Pw_ModPar==5000 ���浽FMRAM
	//		B_SelCS=CS_FMRAM1;
	//		SPI_FMRAM_BufferWrite((uint8_t *)(&w_ParLst[0]), 0, FLASH_PAR_SIZE*2);
	//		SPI_FMRAM_BufferWrite((uint8_t *)(&w_ParLst[512]), 1024, FLASH_ID_SIZE*2);		//FLASH_ID_SIZE 128
	//		Pw_ModPar=18;
	//		// �ټ��޸Ķ������� ZCL
	//		S_SavePar=1;
	//		B_ForceSavPar=0;

	//		//д�룬����<����
	//		STMFLASH_Write(FLASH_SAVE_ADDR0,(uint32_t*)&w_ParLst_Drive,START_CMD_ADDR);		//�����������ֵ���浽FLASH��

	//		//����λ����Ϣ
	//		STMFLASH_Write(FLASH_SAVE_POSTION1,(uint32_t*)&w_ParLst_PosPar,FLASH_POS_SIZE);
	//
	//		//�����������
	//		if(Pos_Group_Select>5 || Pos_Group_Select<1)
	//			Pos_Group_Select=1;
	//		//����λ��ָ��
	//		//��w_ParLst_Pos_CMD���浽FLASH_SAVE_POS_CMD1��		����<����
	//		STMFLASH_Write(FLASH_SAVE_POS_CMD1+0X00001000*(Pos_Group_Select-1),(uint32_t*)&w_ParLst_Pos_CMD,FLASH_POS_CMD_SIZE);
	//	}
	//
	//	if ( Pw_ModPar==5000 && S_SavePar==0 )
	//	{
	//		Pw_ModPar=0;		// ��ֹ�� Pw_ModPar==5000 ���浽FMRAM
	//		B_SelCS=CS_FMRAM1;
	//		SPI_FMRAM_BufferWrite((uint8_t *)(&w_ParLst[0]), 0, FLASH_PAR_SIZE*2);			//FLASH_PAR_SIZE=255��255*2���ֽڣ���255����
	//		SPI_FMRAM_BufferWrite((uint8_t *)(&w_ParLst[512]), 1024, FLASH_ID_SIZE*2);		//FLASH_ID_SIZE 128
	//		Pw_ModPar=5000;
	//		// �ټ��޸Ķ������� ZCL
	//		S_SavePar=1;

	//		//��RAM�еĲ����趨ֵ���浽FLASH��
	//		STMFLASH_Write(FLASH_SAVE_ADDR0,(uint32_t*)&w_ParLst_Drive,START_CMD_ADDR);

	//		//����λ����Ϣ��FALSH
	//		STMFLASH_Write(FLASH_SAVE_POSTION1,(uint32_t*)&w_ParLst_PosPar,FLASH_POS_SIZE);
	//
	//		//�����������
	//		if(Pos_Group_Select>5 || Pos_Group_Select<1)
	//			Pos_Group_Select=1;
	//		//����λ��ָ�FALSH
	//		//��w_ParLst_Pos_CMD���浽FLASH_SAVE_POS_CMD1��		����<����
	//		STMFLASH_Write(FLASH_SAVE_POS_CMD1+0X00001000*(Pos_Group_Select-1),(uint32_t*)&w_ParLst_Pos_CMD,FLASH_POS_CMD_SIZE);
	//	}

	//	//
	//	if( T_SavePar!=SClk10Ms && S_SavePar!=0 )		// ����MD304L��ʾ״̬�����Ի���� ZCL
	//	{
	//		T_SavePar=SClk10Ms;			// Pw_ModPar=50
	//		C_SavePar++;
	//		if ( C_SavePar>150 && S_SavePar==1 )
	//		{
	//			S_SavePar=2;
	//			Pw_ModPar=6000;
	//		}
	//		else if ( C_SavePar>300 && S_SavePar==2 )
	//		{
	//			T_SavePar=100;
	//			C_SavePar=0;
	//			S_SavePar=0;
	//			Pw_ModPar=0;
	//		}
	//	}
}

// Pw_ModPar=2000,�涨ʱ���ڣ�Ĭ��90s��û���޸ĺͱ������,�򱣴����һ��
void ForceSavePar(void)
{
	//	//ͣ��״̬�£������Զ�����
	//	if(Pw_TouchRunStop==1 && Pr_F_AllStopped!=0)
	//	{
	//		if ( Pw_ModPar==2000 && T_ForceSavPar!=SClkSecond)
	//		{
	//			T_ForceSavPar=SClkSecond;
	//			C_ForceSavPar++;
	//			if ( C_ForceSavPar>90 )		//90s
	//			{
	//				T_ForceSavPar=100;
	//				C_ForceSavPar=0;
	//				B_ForceSavPar=1;
	//			}
	//		}
	//	}
}

uint16_t T_ComErr;

void EquipStatus(void) // �豸״̬
{
}

void KglStatus(void) // ������״̬
{
	if (F_KglOutPrompt && T_KglOutPrompt != SClk10Ms)
	{
		T_KglOutPrompt = SClk10Ms; // ��ʱ���	F_AlarmStopPrompt��־
		C_KglOutPrompt++;
		if (C_KglOutPrompt > 100)
		{
			T_KglOutPrompt = 100;
			C_KglOutPrompt = 0;
			F_KglOutPrompt = 0;
		}
	}
}

void FilterDI(void) // ���˿��������� 2016.4.12
{
	//DI1
	if (!B_DI1)
	{
		if (DI1)
		{
			if (w_DI1StableCounter++ > Pw_DIStableSetNum) // ��ѭ����������
			{
				w_DI1StableCounter = 0;
				DInb[0] = DInb[0] | 0x01;
				B_DI1 = 1;
			}
		}
		else if (w_DI1StableCounter > 0)
			w_DI1StableCounter--;
	}

	else
	{
		if (!DI1)
		{
			if (w_DI1StableCounter++ > Pw_DIStableSetNum) // ��ѭ����������
			{
				w_DI1StableCounter = 0;
				DInb[0] = DInb[0] & 0xFE;
				B_DI1 = 0;
			}
		}
		else if (w_DI1StableCounter > 0)
			w_DI1StableCounter--;
	}
	//DI2
	if (!B_DI2)
	{
		if (DI2)
		{
			if (w_DI2StableCounter++ > Pw_DIStableSetNum) // ��ѭ����������
			{
				w_DI2StableCounter = 0;
				DInb[0] = DInb[0] | 0x02;
				B_DI2 = 1;
			}
		}
		else if (w_DI2StableCounter > 0)
			w_DI2StableCounter--;
	}

	else
	{
		if (!DI2)
		{
			if (w_DI2StableCounter++ > Pw_DIStableSetNum) // ��ѭ����������
			{
				w_DI2StableCounter = 0;
				DInb[0] = DInb[0] & 0xFD;
				B_DI2 = 0;
			}
		}
		else if (w_DI2StableCounter > 0)
			w_DI2StableCounter--;
	}
	//DI3
	if (!B_DI3)
	{
		if (DI3)
		{
			if (w_DI3StableCounter++ > Pw_DIStableSetNum) // ��ѭ����������
			{
				w_DI3StableCounter = 0;
				DInb[0] = DInb[0] | 0x04;
				B_DI3 = 1;
			}
		}
		else if (w_DI3StableCounter > 0)
			w_DI3StableCounter--;
	}

	else
	{
		if (!DI3)
		{
			if (w_DI3StableCounter++ > Pw_DIStableSetNum) // ��ѭ����������
			{
				w_DI3StableCounter = 0;
				DInb[0] = DInb[0] & 0xFB;
				B_DI3 = 0;
			}
		}
		else if (w_DI3StableCounter > 0)
			w_DI3StableCounter--;
	}
	//DI4
	if (!B_DI4)
	{
		if (DI4)
		{
			if (w_DI4StableCounter++ > Pw_DIStableSetNum) // ��ѭ����������
			{
				w_DI4StableCounter = 0;
				DInb[0] = DInb[0] | 0x08;
				B_DI4 = 1;
			}
		}
		else if (w_DI4StableCounter > 0)
			w_DI4StableCounter--;
	}

	else
	{
		if (!DI4)
		{
			if (w_DI4StableCounter++ > Pw_DIStableSetNum) // ��ѭ����������
			{
				w_DI4StableCounter = 0;
				DInb[0] = DInb[0] & 0xF7;
				B_DI4 = 0;
			}
		}
		else if (w_DI4StableCounter > 0)
			w_DI4StableCounter--;
	}
}

//�������������ֵ�Ӻ��� ZCL
void DOConfigValue(uint8_t DOValue, uint8_t DO_BitNo) // DO_BitNo:DO λ��
{
	//DOValue: Ҫ����ļ̵���ֵ��DO_BitNo:DO λ�ţ�ָ���ĸ��̵������
	uint32_t BBDIValue;
	uint8_t j;
	BBDIValue = (uint32_t)(DOutb[(DO_BitNo - 1) / 8]);
	j = (DO_BitNo - 1) % 8;
	if (DOValue)
		MEM_ADDR(BITBAND((uint32_t)&BBDIValue, j)) = 1;
	else
		MEM_ADDR(BITBAND((uint32_t)&BBDIValue, j)) = 0;

	DOutb[(DO_BitNo - 1) / 8] = BBDIValue;
}

void ParLimit(void) // ��������
{
	if (Pw_ComBufType != 1 && Pw_ComBufType != 2) // ͨѶЭ������ 2007.11.1
	{
		Pw_ComBufType = 1;
	}

	//����������
	if (Pw_BaudRate1 != 2400 && Pw_BaudRate1 != 4800 && Pw_BaudRate1 != 9600 && Pw_BaudRate1 != 19200 && Pw_BaudRate1 != 38400 && Pw_BaudRate1 != 57600)
	{
		Pw_BaudRate1 = 57600; // ����1������
	}
	if (Pw_BaudRate2 != 2400 && Pw_BaudRate2 != 4800 && Pw_BaudRate2 != 9600 && Pw_BaudRate2 != 19200 && Pw_BaudRate2 != 38400 && Pw_BaudRate2 != 57600)
	{
		Pw_BaudRate2 = 57600; // ����2������
	}
	if (Pw_BaudRate3 != 2400 && Pw_BaudRate3 != 4800 && Pw_BaudRate3 != 9600 && Pw_BaudRate3 != 19200 && Pw_BaudRate3 != 38400 && Pw_BaudRate3 != 57600)
	{
		Pw_BaudRate3 = 57600; // ����3������
	}
	if (Pw_BaudRate4 != 2400 && Pw_BaudRate4 != 4800 && Pw_BaudRate4 != 9600 && Pw_BaudRate4 != 19200 && Pw_BaudRate4 != 38400 && Pw_BaudRate4 != 57600)
	{
		Pw_BaudRate4 = 57600; // ����4������
	}
	if (Pw_BaudRate5 != 2400 && Pw_BaudRate5 != 4800 && Pw_BaudRate5 != 9600 && Pw_BaudRate5 != 19200 && Pw_BaudRate5 != 38400 && Pw_BaudRate5 != 57600)
	{
		Pw_BaudRate5 = 57600; // ����5������
	}

	if (Pr_Driver1_ComCount > 65535)
	{
		Pr_Driver1_ComCount = 0;
	}

	if (Pr_Driver2_ComCount > 65535)
	{
		Pr_Driver2_ComCount = 0;
	}

	if (Pr_Driver3_ComCount > 65535)
	{
		Pr_Driver3_ComCount = 0;
	}

	if (Pr_Driver4_ComCount > 65535)
	{
		Pr_Driver4_ComCount = 0;
	}

	if (Pr_Driver5_ComCount > 65535)
	{
		Pr_Driver5_ComCount = 0;
	}

	if (Pr_Driver6_ComCount > 65535)
	{
		Pr_Driver6_ComCount = 0;
	}

	//�����������
	if (Pos_Group_Select > 5 || Pos_Group_Select < 1)
		Pos_Group_Select = 1;

	//���Ƶ�ǰλ�ú�
	if (Pw_Current_Pos_No < 1)
		Pw_Current_Pos_No = 1;

	if (Pw_Current_Pos_No > 15)
		Pw_Current_Pos_No = 15;

	//���Ƶ�ǰ�����
	if (Pw_Running_Pos_CMD < 1)
		Pw_Running_Pos_CMD = 1;

	if (Pw_Running_Pos_CMD > COM_CMD_NUM)
		Pw_Running_Pos_CMD = COM_CMD_NUM;

	//����ͨѶ����ʱʱ��
	if (Pw_Write_Timeout_Set < 1)
		Pw_Write_Timeout_Set = 1;

	if (Pw_Write_Timeout_Set > 500)
		Pw_Write_Timeout_Set = 500;

	if (Pw_EquipmentNo1 == 0)
		Pw_EquipmentNo1 = 1;

	if (Pw_EquipmentNo2 == 0)
		Pw_EquipmentNo2 = 1;

	if (Pw_EquipmentNo3 == 0)
		Pw_EquipmentNo3 = 1;

	if (Pw_EquipmentNo4 == 0)
		Pw_EquipmentNo4 = 4;

	if (Pw_EquipmentNo5 == 0)
		Pw_EquipmentNo5 = 5;

	if (Pw_EquipmentNo6 == 0)
		Pw_EquipmentNo6 = 6;

	//--------1# Start--------------------------------------------------------
	if (Pw_Driver1_Pluse_HW == 0 && Pw_Driver1_Pluse == 0)
		Pw_Driver1_Pluse = 10000; //���͵�������

	if (Pw_Driver1_AccTime == 0)
		Pw_Driver1_AccTime = 500; //���ٶ�

	if (Pw_Driver1_Speed == 0)
		Pw_Driver1_Speed = 1000; //�趨�ٶ�

	if (Pw_Motor1_PULSENUM == 0)
		Pw_Motor1_PULSENUM = 10000; //ÿȦ������

	if (Pw_Motor1_FRE_START == 0)
		Pw_Motor1_FRE_START = 10000; //��ʼƵ��

	if (Pw_Motor1_FRE_AA == 0)
		Pw_Motor1_FRE_AA = 10000; //�Ӽ��ٶ�

	if (Pw_Motor1_STEP_PARA == 0)
		Pw_Motor1_STEP_PARA = 500; //����������������

	if (Pw_Motor1_maxposition == 0)
		Pw_Motor1_maxposition = 8388608; //���λ��

	//--------2# Start--------------------------------------------------------
	if (Pw_Driver2_Pluse_HW == 0 && Pw_Driver2_Pluse == 0)
		Pw_Driver2_Pluse = 10000; //���͵�������

	if (Pw_Driver2_AccTime == 0)
		Pw_Driver2_AccTime = 100; //���ٶ�

	if (Pw_Driver2_Speed == 0)
		Pw_Driver2_Speed = 1000; //�趨�ٶ�

	if (Pw_Motor2_PULSENUM == 0)
		Pw_Motor2_PULSENUM = 10000; //ÿȦ������

	if (Pw_Motor2_FRE_START == 0)
		Pw_Motor2_FRE_START = 10000; //��ʼƵ��

	if (Pw_Motor2_FRE_AA == 0)
		Pw_Motor2_FRE_AA = 10000; //�Ӽ��ٶ�

	if (Pw_Motor2_STEP_PARA == 0)
		Pw_Motor2_STEP_PARA = 500; //����������������

	if (Pw_Motor2_maxposition == 0)
		Pw_Motor2_maxposition = 8388608; //���λ��

	//--------3# Start--------------------------------------------------------
	if (Pw_Driver3_Pluse_HW == 0 && Pw_Driver3_Pluse == 0)
		Pw_Driver3_Pluse = 10000; //���͵�������

	if (Pw_Driver3_AccTime == 0)
		Pw_Driver3_AccTime = 100; //���ٶ�

	if (Pw_Driver3_Speed == 0)
		Pw_Driver3_Speed = 1000; //�趨�ٶ�

	if (Pw_Motor3_PULSENUM == 0)
		Pw_Motor3_PULSENUM = 10000; //ÿȦ������

	if (Pw_Motor3_FRE_START == 0)
		Pw_Motor3_FRE_START = 10000; //��ʼƵ��

	if (Pw_Motor3_FRE_AA == 0)
		Pw_Motor3_FRE_AA = 10000; //�Ӽ��ٶ�

	if (Pw_Motor3_STEP_PARA == 0)
		Pw_Motor3_STEP_PARA = 500; //����������������

	if (Pw_Motor3_maxposition == 0)
		Pw_Motor3_maxposition = 8388608; //���λ��

	//--------4# Start--------------------------------------------------------
	if (Pw_Driver4_Pluse_HW == 0 && Pw_Driver4_Pluse == 0)
		Pw_Driver4_Pluse = 10000; //���͵�������

	if (Pw_Driver4_AccTime == 0)
		Pw_Driver4_AccTime = 100; //���ٶ�

	if (Pw_Driver4_Speed == 0)
		Pw_Driver4_Speed = 1000; //�趨�ٶ�

	if (Pw_Motor4_PULSENUM == 0)
		Pw_Motor4_PULSENUM = 10000; //ÿȦ������

	if (Pw_Motor4_FRE_START == 0)
		Pw_Motor4_FRE_START = 10000; //��ʼƵ��

	if (Pw_Motor4_FRE_AA == 0)
		Pw_Motor4_FRE_AA = 10000; //�Ӽ��ٶ�

	if (Pw_Motor4_STEP_PARA == 0)
		Pw_Motor4_STEP_PARA = 500; //����������������

	if (Pw_Motor4_maxposition == 0)
		Pw_Motor4_maxposition = 8388608; //���λ��

	//--------5# Start--------------------------------------------------------
	if (Pw_Driver5_Pluse_HW == 0 && Pw_Driver5_Pluse == 0)
		Pw_Driver5_Pluse = 10000; //���͵�������

	if (Pw_Driver5_AccTime == 0)
		Pw_Driver5_AccTime = 100; //���ٶ�

	if (Pw_Driver5_Speed == 0)
		Pw_Driver5_Speed = 1000; //�趨�ٶ�

	if (Pw_Motor5_PULSENUM == 0)
		Pw_Motor5_PULSENUM = 10000; //ÿȦ������

	if (Pw_Motor5_FRE_START == 0)
		Pw_Motor5_FRE_START = 10000; //��ʼƵ��

	if (Pw_Motor5_FRE_AA == 0)
		Pw_Motor5_FRE_AA = 10000; //�Ӽ��ٶ�

	if (Pw_Motor5_STEP_PARA == 0)
		Pw_Motor5_STEP_PARA = 500; //����������������

	if (Pw_Motor5_maxposition == 0)
		Pw_Motor5_maxposition = 8388608; //���λ��

	//--------6# Start--------------------------------------------------------
	if (Pw_Driver6_Pluse_HW == 0 && Pw_Driver6_Pluse == 0)
		Pw_Driver6_Pluse = 10000; //���͵�������

	if (Pw_Driver6_AccTime == 0)
		Pw_Driver6_AccTime = 100; //���ٶ�

	if (Pw_Driver6_Speed == 0)
		Pw_Driver6_Speed = 1000; //�趨�ٶ�

	if (Pw_Motor6_PULSENUM == 0)
		Pw_Motor6_PULSENUM = 10000; //ÿȦ������

	if (Pw_Motor6_FRE_START == 0)
		Pw_Motor6_FRE_START = 10000; //��ʼƵ��

	if (Pw_Motor6_FRE_AA == 0)
		Pw_Motor6_FRE_AA = 10000; //�Ӽ��ٶ�

	if (Pw_Motor6_STEP_PARA == 0)
		Pw_Motor6_STEP_PARA = 500; //����������������

	if (Pw_Motor6_maxposition == 0)
		Pw_Motor6_maxposition = 8388608; //���λ��
}

void Time_Output(void) // ���ʱ�����	 2008.10.21
{
	//���ʱ�ӣ�����ʵʱ��
	if (SClkSecond >= 60) // ��
	{
		SClkSecond = 0;
		SClkMinute++;
		//w_SetMinute=SClkMinute;
		if (SClkMinute >= 60) // ��
		{
			SClkMinute = 0;
			SClkHour++;
			//w_SetHour=SClkHour;
			if (SClkHour >= 24) // ʱ
			{
				SClkHour = 0;
				SClkDay++;
				//w_SetDay=SClkDay;
				if (SClkDay > 30) // ��
				{
					SClkDay = 1;
					SClkMonth++;
					//w_SetMonth=SClkMonth;
					if (SClkMonth > 12) // ��
					{
						SClkMonth = 1;
						SClkYear++;
					}
				}
			}
		}
	}
}

//��������������ֵ�Ӻ���
uint8_t DIConfigValue(uint8_t DI_BitNo) // DI_BitNo:DI λ��,����������ֵ����
{
	uint32_t BBDIValue;
	uint8_t j;
	BBDIValue = (uint32_t)(DInb[(DI_BitNo - 1) / 8]);
	j = (DI_BitNo - 1) % 8;
	if (MEM_ADDR(BITBAND((uint32_t)&BBDIValue, j)))
		return 1;
	else
		return 0;
}

void DigitalIn(void)
{
	// �����������⣻ =1,���ź�(�̽ӵأ��������1����MCU��Ƭ��)��=0,���źš�
	FilterDI();

	K_StopRun = DIConfigValue(1); // ����ֹͣ					//ע�⣺ �պ�ʱ�����С��Ͽ�ʱ��ֹͣ���С�
}



//ͨ�����������������ͣ
void Manual_Control(void)
{
	if(!Old_K_StopRun)
	{
		if(K_StopRun)												//ע�⣺ �պ�ʱ�����С��Ͽ�ʱ��ֹͣ���С�
		{
			if(Pw_StepAutoMode==0 && F_ForceReseting==0 && Pw_TouchRunStop==1)			//Pw_StepAutoMode=1���ֶ�ģʽ��Ĭ��ֵ����=0��ȫ�Զ�ģʽ
			{
				Pw_TouchRunStop=0;									//Pw_TouchRunStop=0������;=1ֹͣ
			}
			else if ( Pw_StepAutoMode==0 && Pw_TouchRunStop==0 && F_AskStop==0 && F_Starting==0 && F_Stoping==0 && F_ForceReseting==0)
			{
				Pw_TouchRunStop=1;
			}		
		}
	}
	Old_K_StopRun=K_StopRun;
}