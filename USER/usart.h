#ifndef _SP_USART_H
#define _SP_USART_H

#include "stm32f10x.h"

#define SP_USART			GPIOA
#define SP_USART_PIN_TX		GPIO_Pin_2
#define SP_USART_PIN_RX		GPIO_Pin_3

//≈‰÷√SP_USART
void SP_USART_Config(void);

#endif

