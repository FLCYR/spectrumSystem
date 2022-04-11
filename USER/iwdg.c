#include "iwdg.h"

void IWDG_Config(u8 prv,u16 rlv)
{	
	//pr 和 RLR可写 使能
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	//预分频值
	IWDG_SetPrescaler(prv);
	
	//重载值
	IWDG_SetReload(rlv);
	
	IWDG_Enable();	//使能
}
//喂狗
void IWDG_Feed(void)
{
	
	//重装计数值
	IWDG_ReloadCounter();
	
}

