#include "sht20.h"
#include "delay.h"
#include "usart.h"


//��ʼ��IIC
void SHT_IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	IIC_SCL1=1;
	IIC_SDA1=1;
	
	IIC_SCL2=1;
	IIC_SDA2=1;

}
//����IIC��ʼ�ź�
void SHT1_IIC_Start(void)
{
	SDA_OUT1();     //sda�����
	IIC_SDA1=1;	  	  
	IIC_SCL1=1;
	delay_us(4);
 	IIC_SDA1=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL1=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	delay_us(4);
}

void SHT2_IIC_Start(void)
{
	SDA_OUT2();     //sda�����
	IIC_SDA2=1;	  	  
	IIC_SCL2=1;
	delay_us(4);
 	IIC_SDA2=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL2=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	delay_us(4);
}
//����IICֹͣ�ź�
void SHT1_IIC_Stop(void)
{
	SDA_OUT1();//sda�����
	IIC_SCL1=0;
	IIC_SDA1=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL1=1; 
	delay_us(1);
	IIC_SDA1=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//����IICֹͣ�ź�
void SHT2_IIC_Stop(void)
{
	SDA_OUT2();//sda�����
	IIC_SCL2=0;
	IIC_SDA2=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL2=1; 
	delay_us(1);
	IIC_SDA2=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 SHT1_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN1();      //SDA����Ϊ����  
	IIC_SDA1=1;delay_us(1);	   
	IIC_SCL1=1;delay_us(1);		
	while(READ_SDA1)
	{
		ucErrTime++;
	//	delay_us(1);
		if(ucErrTime>250)
		{
			SHT1_IIC_Stop();
			return 1;
		}
	}
	IIC_SCL1=0;//ʱ�����0 	   
	return 0;  
} 

u8 SHT2_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN2();      //SDA����Ϊ����  
	IIC_SDA2=1;delay_us(1);	   
	IIC_SCL2=1;delay_us(1);	 
	while(READ_SDA2)
	{
		ucErrTime++;
	//	delay_us(1);
		if(ucErrTime>250)
		{
			SHT2_IIC_Stop();
			return 1;
		}
	}
	IIC_SCL2=0;//ʱ�����0 	   
	return 0;  
}
//����ACKӦ��
void SHT1_IIC_Ack(void)
{
	IIC_SCL1=0;
	SDA_OUT1();
	IIC_SDA1=0;
	delay_us(20);
	IIC_SCL1=1;
	delay_us(2);
	IIC_SCL1=0;
}

void SHT2_IIC_Ack(void)
{
	IIC_SCL2=0;
	SDA_OUT2();
	IIC_SDA2=0;
	delay_us(20);
	IIC_SCL2=1;
	delay_us(2);
	IIC_SCL2=0;
}
//������ACKӦ��		    
void SHT1_IIC_NAck(void)
{
	IIC_SCL1=0;
	SDA_OUT1();
	IIC_SDA1=1;
	delay_us(5);
	IIC_SCL1=1;
	delay_us(5);
	IIC_SCL1=0;
}	
void SHT2_IIC_NAck(void)
{
	IIC_SCL2=0;
	SDA_OUT2();
	IIC_SDA2=1;
	delay_us(5);
	IIC_SCL2=1;
	delay_us(5);
	IIC_SCL2=0;
}
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void SHT1_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT1(); 	    
    IIC_SCL1=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA1=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(5);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL1=1;
		delay_us(5); 
		IIC_SCL1=0;	
		delay_us(5);
    }	 
} 	  

void SHT2_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT2(); 	    
    IIC_SCL2=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA2=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(5);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL2=1;
		delay_us(5); 
		IIC_SCL2=0;	
		delay_us(5);
    }	 
} 	 
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 SHT1_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN1();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
		IIC_SCL1=0; 
		delay_us(5);
		IIC_SCL1=1;
		receive<<=1;
		if(READ_SDA1)
			receive++;   
		delay_us(5); 
    }					 
    if (!ack)
        SHT1_IIC_NAck();//����nACK
    else
        SHT1_IIC_Ack(); //����ACK   
    return receive;
}

u8 SHT2_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN2();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
    IIC_SCL2=0; 
    delay_us(5);
		IIC_SCL2=1;
        receive<<=1;
        if(READ_SDA2)receive++;   
		delay_us(5); 
    }					 
    if (!ack)
        SHT2_IIC_NAck();//����nACK
    else
        SHT2_IIC_Ack(); //����ACK   
    return receive;
}



//SHT2x_data SHT20;

u8 SHT2x_Init(void)
{
	u8 err;
	SHT_IIC_Init();
	err = SHT2x_SoftReset();
	return err;
}

u8 SHT2x_SoftReset(void)   //SHT20�����λ
{
	u8 err=0;
	SHT1_IIC_Start();
	SHT1_IIC_Send_Byte(0x80);
	err = SHT1_IIC_Wait_Ack();
	SHT1_IIC_Send_Byte(0xFE);
	err = SHT1_IIC_Wait_Ack();
	SHT1_IIC_Stop();
	
	SHT2_IIC_Start();
	SHT2_IIC_Send_Byte(0x80);
	err = SHT2_IIC_Wait_Ack();
	SHT2_IIC_Send_Byte(0xFE);
	err = SHT1_IIC_Wait_Ack();
	SHT2_IIC_Stop();
	return err;
}

float SHT2x_GetTempPoll1(void)
{
    float TEMP;
    u8 ack, tmp1, tmp2;
    u16 ST;
    u16 i=0;	
    SHT1_IIC_Start();				//����IIC��ʼ�ź�
    SHT1_IIC_Send_Byte(I2C_ADR_W);			//IIC����һ���ֽ� 
	ack = SHT1_IIC_Wait_Ack();
	SHT1_IIC_Send_Byte(TRIG_TEMP_MEASUREMENT_POLL);
	ack = SHT1_IIC_Wait_Ack();
    do 
	{
		delay_ms(100);               
		SHT1_IIC_Start();				//����IIC��ʼ�ź�
		SHT1_IIC_Send_Byte(I2C_ADR_R);	
		i++;
		ack = SHT1_IIC_Wait_Ack();
		if(i==1000)
			break;
    } while(ack!=0);
	
    tmp1 = SHT1_IIC_Read_Byte(1);
    tmp2 = SHT1_IIC_Read_Byte(1);
		SHT1_IIC_Read_Byte(0);
    SHT1_IIC_Stop();
    
    ST = (tmp1 << 8) | (tmp2 << 0);
	
    ST &= ~0x0003;
    TEMP = ((float)ST * 0.00268127) - 46.85;

    return (TEMP);	  
}

float SHT2x_GetTempPoll2(void)
{
    float TEMP;
    u8 ack, tmp1, tmp2;
    u16 ST;
    u16 i=0;
    SHT2_IIC_Start();				//����IIC��ʼ�ź�
    SHT2_IIC_Send_Byte(I2C_ADR_W);			//IIC����һ���ֽ� 
		ack = SHT2_IIC_Wait_Ack();	
		SHT2_IIC_Send_Byte(TRIG_TEMP_MEASUREMENT_POLL);
		ack = SHT2_IIC_Wait_Ack();

    do {
        delay_ms(100);               
        SHT2_IIC_Start();				//����IIC��ʼ�ź�
        SHT2_IIC_Send_Byte(I2C_ADR_R);	
			  i++;
			  ack = SHT2_IIC_Wait_Ack();
			  if(i==1000)
				  break;
    } while(ack!=0);
    tmp1 = SHT2_IIC_Read_Byte(1);
    tmp2 = SHT2_IIC_Read_Byte(1);
		SHT2_IIC_Read_Byte(0);
    SHT2_IIC_Stop();
    
    ST = (tmp1 << 8) | (tmp2 << 0);
	
    ST &= ~0x0003;
    TEMP = ((float)ST * 0.00268127) - 46.85;

    return (TEMP);	  
}

float SHT2x_GetHumiPoll1(void)
{
    float HUMI;
    u8 ack, tmp1, tmp2;
    u16 SRH;
		u16 i=0;
    
    SHT1_IIC_Start();				//����IIC��ʼ�ź�
    SHT1_IIC_Send_Byte(I2C_ADR_W);			//IIC����һ���ֽ� 
	ack = SHT1_IIC_Wait_Ack();	
	SHT1_IIC_Send_Byte(TRIG_HUMI_MEASUREMENT_POLL);
	ack = SHT1_IIC_Wait_Ack();    
    do {
		delay_ms(100);               
        SHT1_IIC_Start();				//����IIC��ʼ�ź�
        SHT1_IIC_Send_Byte(I2C_ADR_R);	
		i++;
		ack = SHT1_IIC_Wait_Ack();
		if(i==100)
			break;
    } while(ack!=0);
    
    tmp1 = SHT1_IIC_Read_Byte(1);
   
    tmp2 = SHT1_IIC_Read_Byte(1);
    SHT1_IIC_Read_Byte(0);
    SHT1_IIC_Stop();
    
    SRH = (tmp1 << 8) | (tmp2 << 0);
    SRH &= ~0x0003;
    HUMI = ((float)SRH * 0.00190735) - 6;  

    return (HUMI);
}

float SHT2x_GetHumiPoll2(void)
{
    float HUMI;
    u8 ack, tmp1, tmp2;
    u16 SRH;
		u16 i=0;
    
    SHT2_IIC_Start();				//����IIC��ʼ�ź�
    SHT2_IIC_Send_Byte(I2C_ADR_W);			//IIC����һ���ֽ� 
		ack = SHT2_IIC_Wait_Ack();	
		SHT2_IIC_Send_Byte(TRIG_HUMI_MEASUREMENT_POLL);
		ack = SHT2_IIC_Wait_Ack();    
    do {
       delay_ms(100);               
        SHT2_IIC_Start();				//����IIC��ʼ�ź�
        SHT2_IIC_Send_Byte(I2C_ADR_R);	
			  i++;
			  ack = SHT2_IIC_Wait_Ack();
			  if(i==100)break;
    } while(ack!=0);
    
    tmp1 = SHT2_IIC_Read_Byte(1);
   
    tmp2 = SHT2_IIC_Read_Byte(1);
    SHT2_IIC_Read_Byte(0);
    SHT2_IIC_Stop();
    
    SRH = (tmp1 << 8) | (tmp2 << 0);
    SRH &= ~0x0003;
    HUMI = ((float)SRH * 0.00190735) - 6;  
    return (HUMI);
}




