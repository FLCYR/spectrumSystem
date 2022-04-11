#ifndef __MYIIC_H
#define __MYIIC_H

#include "stm32f10x.h"

   	   		   
//IO��������
#define SDA_IN()  {GPIOB->CRL&=0XFFF0FFFF;GPIOB->CRL|=8<<16;}
#define SDA_OUT() {GPIOB->CRL&=0XFFF0FFFF;GPIOB->CRL|=3<<16;}

//IO��������	 
#define IIC_SCL    PBout(5) //SCL
#define IIC_SDA    PBout(4) //SDA	 
#define READ_SDA   PBin(4)  //����SDA 	



//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

u8 IIC_Read_Byte_2(void);
#endif
















