#ifndef _TIMER_H
#define _TIMER_H

#include "stm32f10x.h"


#define            GENERAL_TIM                   TIM2
#define            GENERAL_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            GENERAL_TIM_CLK               RCC_APB1Periph_TIM2
#define            GENERAL_TIM_Prescaler         7199
#define            GENERAL_TIM_IRQ               TIM2_IRQn
#define            GENERAL_TIM_IRQHandler        TIM2_IRQHandler



//#define            MEAN_TIM                   TIM5
//#define            MEAN_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
//#define            MEAN_TIM_CLK               RCC_APB1Periph_TIM2
//#define            MEAN_TIM_Prescaler         7199
//#define            MEAN_TIM_IRQ               TIM5_IRQn
//#define            MEAN_TIM_IRQHandler        TIM5_IRQHandler

void GENERAL_TIM_Init(u16 per);	  

//void MEANl_TIM_Init(u16 per);
#endif





