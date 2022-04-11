#include "shutter.h"
#include "sys.h"

void Shutter_Init(void)
{
	 GPIO_InitTypeDef  GPIO_InitStructure;
		
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PA,PD�˿�ʱ��
		
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				 //LED0-->PA.8 �˿�����
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8
	// GPIO_SetBits(GPIOB,GPIO_Pin_10);						 //PA.8 �����
		
	 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8
}


void Set_shutter(u8 state)
{
	shutter = state;
}

