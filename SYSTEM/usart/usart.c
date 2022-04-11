#include "usart.h"

//配置SP_USART
void SP_USART_Config(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(SP_USART_GPIO_CLOCK,ENABLE);//GPIO时钟
	RCC_APB1PeriphClockCmd(SP_USART_CLOCK,ENABLE);	//串口时钟
	
	
	GPIO_InitStructure.GPIO_Pin=SP_USART_PIN_TX;//PA2 TX
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(SP_USART_PORT,&GPIO_InitStructure); //发送引脚
	
	
	
	
	GPIO_InitStructure.GPIO_Pin=SP_USART_PIN_RX;//PA3（RX）输入上拉
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;   //修改原GPIO_Mode_IPU（输入上拉）->GPIO_Mode_IN_FLOATING(浮空输入)/////////////////////////////////////////////
	GPIO_Init(SP_USART_PORT,&GPIO_InitStructure);//接收引脚
	
	
	
	USART_DeInit(SP_USART);//复位串口SP_USART
	USART_InitStructure.USART_BaudRate=SP_BAUDRATE;  //115200
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//收发模式
	USART_InitStructure.USART_Parity=USART_Parity_No;
	
	USART_Init(SP_USART,&USART_InitStructure);
	
	USART_ClearITPendingBit(SP_USART,USART_IT_RXNE);
	USART_ITConfig(SP_USART,USART_IT_RXNE,ENABLE);//使能SP_USART接收中断
	
	NVIC_InitStructure.NVIC_IRQChannel=SP_USART_IRQN;	//中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(SP_USART,ENABLE);//使能串口2
}

//发送n个字节数据
//buff:发送区首地址
//len：发送的字节数
void SP_SendData(u8 *buf,u8 len)
{ 
        //RS485_TX_EN=1;//切换为发送模式
        while(len--)
        {
                while(USART_GetFlagStatus(SP_USART,USART_FLAG_TXE)==RESET);//等待发送区为空
                USART_SendData(SP_USART,*(buf++));
        }
        while(USART_GetFlagStatus(SP_USART,USART_FLAG_TC)==RESET);//等待发送完成
}







