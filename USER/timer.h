#ifndef TIMER_H
#define TIMER_H

#include "stm32f10x.h"


#define            GENERAL_TIM                   TIM2
#define            GENERAL_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            GENERAL_TIM_CLK               RCC_APB1Periph_TIM2
#define            GENERAL_TIM_Prescaler         7199
#define            GENERAL_TIM_IRQ               TIM2_IRQn
#define            GENERAL_TIM_IRQHandler        TIM2_IRQHandler



void GENERAL_TIM_Init(u16 per);	  

void TIM4_IRQHandler(void);   //TIM4ÖÐ¶Ï

#endif





