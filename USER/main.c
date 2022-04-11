#include "led.h"
#include "delay.h"
#include "sys.h"
#include "key.h"
#include "usart.h"
#include "timer.h"
#include "master.h"
#include "glt.h"
#include "iwdg.h"


/*
 Modbus������
	 
	0x01: ����Ȧ�Ĵ���
	0x02: ����ɢ����Ĵ���
	0x03: �����ּĴ���
	0x04: ������Ĵ���
	0x05: д������Ȧ�Ĵ���
	0x06: д�������ּĴ���
	0x0f: д�����Ȧ�Ĵ���
	0x10: д������ּĴ���
*/



 int main(void)
 {	
	
	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//NVIC���ȼ����
    Shutter_Init();//��Ƭ��ʼ��
	delay_init();	    	 //��ʱ������ʼ��
	SP_USART_Config();	//�����Ǵ�������
	RS485_Init();		//����λ��485��������
	SHT_IIC_Init();		//I2c
	GENERAL_TIM_Init(60);	//��ʱ��
	AT24CXX_Init();			//rom����
	SHT2x_Init();	//��ʪ�ȴ�������ʼ��
	Motor_Init();	//ˢ��
	Dis_check_Init();	//ˢ��
	Timer4_Init(15000);	//��ʱ��4��ʼ��
	RS485_TX_EN=0;		//485����ʹ�ܶ�
	
	
	 

	delay_ms(300);	//�ȴ���ʼ�����
	//Init_All_Argument();		
	
	Init_All_Argument();//��ʼ�����ֲ���
	//(128/40)*1000=3s
	IWDG_Config(IWDG_Prescaler_128,2000); //���ÿ��Ź� 6sһ��ι��
	
	
	
	while(1)
	{	
		
		RS485_RX_Service();	//������յ�����
		
		if(timeFlag)	//������Ϣ����Ļ ��ʱ����
		{	
			timeFlag=0;
			Send_All_Info_To_Screen();	//��ʱ������ҳ���ݸ���Ļ
			delay_ms(50);
			Check_Btn_State();		//�����Ļ�Ƿ��°�ť
		}
		IWDG_Feed();//ι��
	}
}
