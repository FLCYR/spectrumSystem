#include "motor.h"
#include "delay.h"

u8 change_brush=0;
//��ʼ��PB5��PE5Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void Dis_check_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PA,PD�˿�ʱ��
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;				 //LED0-->PA.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //��������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				 //LED0-->PA.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8
 GPIO_ResetBits(GPIOA,GPIO_Pin_7);						 //PA.8 �����	
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |  GPIO_Pin_1 |  GPIO_Pin_2;				 //LED0-->PA.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8
 GPIO_ResetBits(GPIOB,GPIO_Pin_0 |  GPIO_Pin_1 |  GPIO_Pin_2);						 //PA.8 �����		
}
 
void Motor_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PA,PD�˿�ʱ��
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12;				 //LED0-->PA.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8
 GPIO_SetBits(GPIOA,GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12);						 //PA.8 �����
	
 GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_1;				 //LED0-->PA.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //��������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8
// //GPIO_ResetBits(GPIOA,GPIO_Pin_2);		
//	
}

void Motor_move(char dir)
{
	change_brush=1;
	if((dir == 0)&&(change_brush == 1))	//������ˢ��ˢƬ��λ
	{
		change_brush = 0;
		Discs1 = 1;Discs2 = 1;Discs3 = 1;
		MIN1 = 1;
		MIN2 = 0;	
		while(HallB == 0);
		delay_ms(10);
		MIN1 = 0;
		MIN2 = 0;
		delay_ms(5);
		MIN1 = 0;
		MIN2 = 1;
		while(HallA == 0);
		delay_ms(50);
		MIN1 = 0;
		MIN2 = 0;
	}
	else if((dir == 1)&&(change_brush == 1))		//������ˢƬ,ˢƬ����
	{
		Discs1 = 1;Discs2 = 1;Discs3 = 1;
		MIN1 = 0;
		MIN2 = 1;	
		while(HallC == 0);
		delay_ms(50);
		MIN1 = 0;
		MIN2 = 0;						
	}
	
	else if(dir == 2)	//���
	{		
		Discs1 = 1;Discs2 = 1;Discs3 = 1;
		MIN1 = 1;
		MIN2 = 0;
		while(HallB == 0)
			;
		delay_ms(10);
		MIN1=0;
		MIN2=0;
		delay_ms(5);
		MIN1 = 0;
		MIN2 = 1;
		while(HallA == 0)
			;
		delay_ms(150);
		MIN1 = 0;
		MIN2 = 0;
		
		
		
//		Discs1 = 1;Discs2 = 1;Discs3 = 1;
//		MIN1 = 1;
//		MIN2 = 0;
//		while(HallB == 0)
//			;
////		delay_ms(10);
////		MIN1=0;
////		MIN2=0;
////		delay_ms(5);
//		MIN1 = 0;
//		MIN2 = 1;
//		while(HallA == 0)
//			;
//		delay_ms(250);
//		MIN1 = 0;
//		MIN2 = 0;
	}
}

char dischek(void)
{
	char dis;
	if(HallA == 0)
	{
		dis = 1;
	}
	if(HallB == 0)
	{
		dis = 2;
	}
	return dis;
}

