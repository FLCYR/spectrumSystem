#ifndef __SPI2_H
#define __SPI2_H

#include "sys.h"


void SPI2_Init(void);			 //��ʼ��SPI��
void SPI2_SetSpeed(u8 SpeedSet); //����SPI�ٶ�   
u16 SPI2_ReadWriteByte(u16 TxData);//SPI���߶�дһ���ֽ�

#endif
