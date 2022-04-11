#include "usart.h"

//����SP_USART
void SP_USART_Config(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(SP_USART_GPIO_CLOCK,ENABLE);//GPIOʱ��
	RCC_APB1PeriphClockCmd(SP_USART_CLOCK,ENABLE);	//����ʱ��
	
	
	GPIO_InitStructure.GPIO_Pin=SP_USART_PIN_TX;//PA2 TX
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(SP_USART_PORT,&GPIO_InitStructure); //��������
	
	
	
	
	GPIO_InitStructure.GPIO_Pin=SP_USART_PIN_RX;//PA3��RX����������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;   //�޸�ԭGPIO_Mode_IPU������������->GPIO_Mode_IN_FLOATING(��������)/////////////////////////////////////////////
	GPIO_Init(SP_USART_PORT,&GPIO_InitStructure);//��������
	
	
	
	USART_DeInit(SP_USART);//��λ����SP_USART
	USART_InitStructure.USART_BaudRate=SP_BAUDRATE;  //115200
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//�շ�ģʽ
	USART_InitStructure.USART_Parity=USART_Parity_No;
	
	USART_Init(SP_USART,&USART_InitStructure);
	
	USART_ClearITPendingBit(SP_USART,USART_IT_RXNE);
	USART_ITConfig(SP_USART,USART_IT_RXNE,ENABLE);//ʹ��SP_USART�����ж�
	
	NVIC_InitStructure.NVIC_IRQChannel=SP_USART_IRQN;	//�жϺ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(SP_USART,ENABLE);//ʹ�ܴ���2
}

//����n���ֽ�����
//buff:�������׵�ַ
//len�����͵��ֽ���
void SP_SendData(u8 *buf,u8 len)
{ 
        //RS485_TX_EN=1;//�л�Ϊ����ģʽ
        while(len--)
        {
                while(USART_GetFlagStatus(SP_USART,USART_FLAG_TXE)==RESET);//�ȴ�������Ϊ��
                USART_SendData(SP_USART,*(buf++));
        }
        while(USART_GetFlagStatus(SP_USART,USART_FLAG_TC)==RESET);//�ȴ��������
}







