#include "iwdg.h"

void IWDG_Config(u8 prv,u16 rlv)
{	
	//pr �� RLR��д ʹ��
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	//Ԥ��Ƶֵ
	IWDG_SetPrescaler(prv);
	
	//����ֵ
	IWDG_SetReload(rlv);
	
	IWDG_Enable();	//ʹ��
}
//ι��
void IWDG_Feed(void)
{
	
	//��װ����ֵ
	IWDG_ReloadCounter();
	
}

