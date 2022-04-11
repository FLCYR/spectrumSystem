#ifndef _SP_USART_H
#define _SP_USART_H

#include "stm32f10x.h"

#define SP_USART				USART2
#define SP_USART_PORT			GPIOA
#define SP_USART_PIN_TX			GPIO_Pin_2
#define SP_USART_PIN_RX			GPIO_Pin_3
#define SP_USART_CLOCK			RCC_APB1Periph_USART2
#define SP_USART_GPIO_CLOCK		RCC_APB2Periph_GPIOA
#define SP_USART_IRQN			USART2_IRQn
#define SP_BAUDRATE				115200



//����SP_USART
void SP_USART_Config(void);
//����n���ֽ�����
//buff:�������׵�ַ
//len�����͵��ֽ���
void SP_SendData(u8 *buf,u8 len);
#endif




