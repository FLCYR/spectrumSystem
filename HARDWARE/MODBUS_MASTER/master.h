#ifndef __MASTER_H
#define __MASTER_H
#include "sys.h"
#include "shutter.h"
#include "sht20.h"
#include "motor.h"
#include "fitting_algorithm.h"
#define READ_COIL     01
#define READ_DI       02
#define READ_HLD_REG  03
#define READ_AI       04
#define SET_COIL      05
#define SET_HLD_REG   06
#define WRITE_COIL    15
#define WRITE_HLD_REG 16


extern u8 timeFlag;
extern u8 isNeedReadRatios;


#define RS485_TX_EN PAout(8)

#define HI(n) ((n)>>8)
#define LOW(n) ((n)&0xff)

#define	GET_BIT(x, bit)	((x & (1 << bit))>>bit)



#define SCREEN_USART			USART1
#define SCREEN_USART_PORT		GPIOA
#define SCREEN_USART_PIN_TX		GPIO_Pin_9
#define SCREEN_USART_PIN_RX		GPIO_Pin_10
#define	SCREEN_USART_IRQN		USART1_IRQn



#define SP_MAX_SIZE			5000	//������������



void RS485_Init(void);
void Timer4_Init(u16 arr);

//void Modbus_RegMap(void);

//д�Ĵ���ӳ��
void Modbus_RegMap(u16 regStartAddr,u16*buf,u16 len);
	
void Modbus_01_Solve(void);
void Modbus_02_Solve(void);
void Modbus_03_Solve(void);
void Modbus_05_Solve(void);
void Modbus_06_Solve(void);
void Modbus_15_Solve(void);
void Modbus_16_Solve(void);
	
void Timer3_Init(void);
void RS485_TX_Service(void);
void Master_Service(u8 SlaverAddr,u8 order,u16 addr,u16 value);
void RS485_RX_Service(void);
void modbus_rtu(void);
void RS485_SendByte(u8 data);
void RS485_SendData(u8 *buf,u16 len);
void USART1_IRQHandler(void);

//�ָ���������
void Restore_Factory_Setting(void);

//�ϵ��ʼ�����ֲ���
void Init_All_Argument(void);
//��ʱ��2 �ж�
//��ʾ�����ǽ������
void TIM2_IRQHandler(void);
//TIM4 ��ʱ���ж�
//ѭ�����ͼ����Ļ��ť״̬
void TIM4_IRQHandler(void) ;


//ѭ�����
void Check_SPData(void);




//��λ�������ӿ�
//���ôӻ�վ�� ������EEPROM�� λ�� 0x0000
void Set_Slave_Addr(void);
void Send_Slave_Addr(void); //���ʹӻ���ַ

//���ù�����ƽ������
void Set_SP_Average_Time(void);
void Get_SP_Average_Time(void); //��ȡƽ������
//����ģʽ
void Set_SP_Pulse_Mode(void);
//����ߵ�ʱ��
void Set_SP_Pulse_Time(void);
void Get_SP_Pulse_Time(void);	//��ȡ����ߵ�ʱ��
//���û���ʱ��
void Set_SP_IntegTime(void);
//��ȡ����ʱ��
void Get_SP_IntegTime(void);



//д���û����õ�ϵ��
void Write_User_Ratios_To_Rom(u8*buf);

//��ʼ��ʱ��ȡ����
void Init_Get_Wave(void);

//��ȡROM�еĹ���
void Read_220_275_Spectrum(u16 addr,double*data_220,double*data_275);

void Write_Ratio_To_Rom(void); //д��ϵ��
void Send_Ratio_To_USART(void);	//����ϵ������λ��

void Set_Calc_Wave(void);		//����������������ʲ���
void Read_Calc_Wave(void);	//��ȡĳ�����ʵļ��㲨��
void Write_Material_Wave_To_Rom(u8 type,u16 A1,u16 A2);	//д��ĳ�����ʼ��㲨��

void Reset_Factory_Setting(void);	//�ָ���������

void Set_Clear_Period(void); //�����������

void Get_Sample_Spectrum(void);	//��ȡ��Ʒ���ײ����͸���λ��

void Get_Dark_Spectrum(void);		//��ȡ������

void Get_Reference_Spectrum(void);		//��ȡ�αȹ���

void Get_White_Spectrum(void);	//��ȡ�׹���

//д�����  EEPROM ��0x1000��
void Write_DarkSp_To_Rom(void);   //д�밵����

void Write_ReferenceSp_To_Rom(void);  //д��αȹ���

void Write_WhiteSp_To_Rom(void);		//д��׹���


void Clear_Device(void);	//����豸
void Send_Light_Mode(void);	//���͹��������ģʽ����λ��
void Send_Temp_Humidity(void);	//��ʪ�ȸ���λ��

//����EEPROM ��ŵĹ��� �� USART1 ����

void Send_DarkSp_To_USART(void);	//���Ͱ�����

void Send_ReferenceSp_To_USART(void); //���Ͳο�����

void Send_WhiteSp_To_USART(void); 	//���Ͱ׹���

void Send_WaveCount_To_USART(void);	//���Ͳ�������

void Send_WaveData_To_USART(void);	//���Ͳ�������

void CalculateWaveLength(void);		//���㲨��ֵ



//��Ļ����

void setAlarmState(int temp1,int temp2,int humidity1,int humidity2);	//���ø��ֱ���״̬

void  Calc_Query_Btn(void); //ȡ��ȷ����ť
void  Process_Ratios_From_User(void);	//�����û���ϵ��
void  Send_Order_Read_Ratios(void);		//���������ȡ�û�ϵ��
void  Read_User_Set_Ratios(void);		//��ȡ�û�д���ϵ��

void  Send_All_Info_To_Screen(void);
void  Send_Time_To_Screen(void);		//д�뵱ǰ����ֵ���ۼƹ���ʱ������һ�β���ʱ��
void  Send_Temp_Humidity_To_Screen(void);	//������ʪ�ȵ���Ļ
void  Check_Btn_State(void);		//�����Ļ��ť״̬
//void  Check_Calibration_State(void);	//���У׼��ť״̬
void  Analysis_Btn_State(void);		//������ť״̬
void  Send_Value_To_USER(void);	//���û��趨��ϵ������
void Switch_Rubber_Brush(void);		//������ˢ
void Detector_Diagnosis(void);		//̽ͷ���
void Detctor_Calibration(void);		//̽ͷУ׼

void Select_Concentration(u8 index,u16 switchAddr,u16 valueAddr);




void Start_Darw(void);		//��ʼ��ͼ
void Stop_Draw(void);		//ֹͣ��ͼ

//void Send_Zero_Concentration_To_Screen(void);	//����Ũ��0����Ļ
//void Send_One_Concentration_To_Screen(void);	//����Ũ��1


void Reset_All_Setting(void);			//������������
void Get_All_Setting(void);				//��ȡ��������
void Set_All_Setting(void);				//����Ļ������Ӧ��




//-------------------------------------
//����ԭʼ��Ũ��ֵ
//void calculateConcentration(void);
//void getCalibratedData(void);		// ��ȡ�궨���Ũ��ֵ x :ԭʼŨ��ֵ

double calculateConcentration(void);

void CalculateValue(void);
double getCalibratedData(double x);

//�������
void calculateCalibrationRatios(void);

//��ȡ����
//��ȡ��λ�����͹����Ĳ���
//��������ʱ
void getRatios(u8 type,u16 startAddr);
//��������
void getRatios_Sample(u16 startAddr);
void getAllInitRatios(void);	//��ȡ�������ʵļ���ϵ��
#endif









