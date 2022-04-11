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
 Modbus功能码
	 
	0x01: 读线圈寄存器
	0x02: 读离散输入寄存器
	0x03: 读保持寄存器
	0x04: 读输入寄存器
	0x05: 写单个线圈寄存器
	0x06: 写单个保持寄存器
	0x0f: 写多个线圈寄存器
	0x10: 写多个保持寄存器
*/



 int main(void)
 {	
	
	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//NVIC优先级组别
    Shutter_Init();//挡片初始化
	delay_init();	    	 //延时函数初始化
	SP_USART_Config();	//光谱仪串口设置
	RS485_Init();		//与上位机485串口设置
	SHT_IIC_Init();		//I2c
	GENERAL_TIM_Init(60);	//定时器
	AT24CXX_Init();			//rom设置
	SHT2x_Init();	//温湿度传感器初始化
	Motor_Init();	//刷子
	Dis_check_Init();	//刷子
	Timer4_Init(15000);	//定时器4初始化
	RS485_TX_EN=0;		//485发送使能端
	
	
	 

	delay_ms(300);	//等待初始化完成
	//Init_All_Argument();		
	
	Init_All_Argument();//初始化各种参数
	//(128/40)*1000=3s
	IWDG_Config(IWDG_Prescaler_128,2000); //配置看门狗 6s一次喂狗
	
	
	
	while(1)
	{	
		
		RS485_RX_Service();	//处理接收到命令
		
		if(timeFlag)	//发送信息给屏幕 定时处理
		{	
			timeFlag=0;
			Send_All_Info_To_Screen();	//定时发送首页数据给屏幕
			delay_ms(50);
			Check_Btn_State();		//检查屏幕是否按下按钮
		}
		IWDG_Feed();//喂狗
	}
}
