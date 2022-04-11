#include "glt.h"






///////////////////////////
//ָ����10ms
//////////////////////////


/*
void USART2_IRQHandler(void)
{		

        u8 res;
    
        if(USART_GetITStatus(SP_USART,USART_IT_RXNE)!=RESET)
        {
                
                res=USART_ReceiveData(SP_USART); //�����յ����ֽڣ�ͬʱ��ر�־�Զ����
                
				
                if(size<SP_MAX_SIZE)
                {		
                        spArr[size]=res;
                        size++;
                        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//�����ʱ������ж�
						TIM_SetCounter(TIM2,0);//�����յ�һ���µ��ֽڣ�����ʱ��3��λΪ0�����¼�ʱ
                        TIM_Cmd(TIM2,ENABLE);//��ʼ��ʱ
                }
				else
				{
					RS485_SendByte('M');
				}
        }
}
*/

//���û���ʱ��
void SP_Set_IntegTime(u32 time)
{
	
	u8 buf[7]={0x69};
	u16 crc;
	
	buf[1]=time>>24;
	buf[2]=(time>>16)&0xff;
	buf[3]=(time>>8)&0xff;
	buf[4]=time&0xff;
	crc=CRC_Compute(buf,5);
	buf[5]=crc>>8;
	buf[6]=crc&0xff;
	SP_SendData(buf,7);
	delay_ms(10);
	
}
//��ѯ����ʱ��


void SP_Get_IntegTime(void)
{
	
	u8 buf[4]={0x3f,0x69,0x6e,0xd0};
	SP_SendData(buf,4);
	delay_ms(10);
}

//��ȡ����������
void SP_Get_SPData(void)
{
	u8 buf[3]={0x53,0x7d,0xff};
	SP_SendData(buf,3);
	delay_ms(10);
}

//����믵Ƹߵ�����ʱ��
//��λ10ns
void SP_Set_Pulse_Time(u8 *bufTime)
{
	
	u8 buf[11]={0x30};
	u8 i;
	u16 crc;
	for(i=0;i<8;i++)
	{
		buf[i+1]=bufTime[i];
	}
	
	crc=CRC_Compute(buf,9);
	
	buf[9]=crc>>8;
	buf[10]=crc&0xff;
	
	SP_SendData(buf,11);
	delay_ms(10);
}

void SP_Get_Pulse_Time(void)	//��ȡ믵�����ʱ��
{
	u8 buf[4]={0x3f,0x30,0x54,0x10};
	
	SP_SendData(buf,4);
	delay_ms(10);
}

//��믵� ������ģʽ
void SP_Open_Light(u8 mode)
{	
	u16 crc;
	u8 buf[4]={0x31};
	//00000001		0x01�������ģʽ
	//10000001		0x81	�������
	//00000000		0x0		�ر�
	buf[1]=mode;
	crc=CRC_Compute(buf,2);
	buf[2]=crc>>8;
	buf[3]=crc&0xff;
	SP_SendData(buf,4);
	delay_ms(10);
}
//��ȡģʽ
void SP_Get_Light_Mode(void)
{
	u8 buf[4]={0x3f,0x31,0x94,0xd1};
	SP_SendData(buf,4);
	delay_ms(10);
}
void SP_Reset(void)
{
	
	u8 buf[3]={0x52,0xbd,0x3f};
	SP_SendData(buf,3);
	delay_ms(10);
}

//��ȡ����ֵ

void SP_Get_WaveLength(void)
{
	
	
	u8 buf[4]={0x3f,0x53,0x7d,0x50};
	SP_SendData(buf,4);
	delay_ms(10);
}

void SP_Get_Average(void)	//��ȡƽ������
{
	u8 buf[4]={0x3f,0x41,0x70,0xd0};
	SP_SendData(buf,4);
	delay_ms(10);
}
void SP_Set_Average(u16 count)
{
	
	u16 crc;
	u8 buf[5]={0x41};
	buf[1]=count>>8;
	buf[2]=count&0xff;
	crc=CRC_Compute(buf,3);
	buf[3]=crc>>8;
	buf[4]=crc&0xff;
	SP_SendData(buf,5);
	delay_ms(10);
	
}






