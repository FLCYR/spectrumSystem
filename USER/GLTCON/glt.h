#ifndef GLT_H_
#define GLT_H_
#include "usart.h"
#include "crc.h"
#include "master.h"
#include "delay.h"
#include "timer.h"
#include "i2c_ee.h"



//extern u8 spArr[4096];
//extern u16 size;



//USART2�ж�  ���������ݽӿ�
void USART2_IRQHandler(void);
//���û���ʱ��  us
void SP_Set_IntegTime(u32 time);
//��ȡ����ʱ�� us
void SP_Get_IntegTime(void);

//��ȡ����������
void SP_Get_SPData(void);

//����믵�����ߵ�ʱ��
void SP_Set_Pulse_Time(u8 *bufTime);
void SP_Get_Pulse_Time(void);	//��ȡ믵�����ʱ��
//��믵� ������ģʽ
void SP_Open_Light(u8 mode);
//��ȡģʽ
void SP_Get_Light_Mode(void);
//��ȡ����ֵ
void SP_Get_WaveLength(void);


//��λ
void SP_Reset(void);


//����ƽ������

void SP_Set_Average(u16 count);
void SP_Get_Average(void);	//��ȡƽ������
#endif

