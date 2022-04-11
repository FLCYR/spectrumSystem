#include "i2c_ee.h"
#include "new_i2c_ee.h"
#include "delay.h"

//��ʼ��IIC�ӿ�
void AT24CXX_Init(void)
{
	IIC_Init();
}
//��AT24CXXָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
	u8 temp=0;		  	    																 
    IIC_Start();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(0XA0);	   //����д����
		IIC_Wait_Ack();
		IIC_Send_Byte(ReadAddr>>8);//���͸ߵ�ַ
		//IIC_Wait_Ack();		 
	}
	else 
		IIC_Send_Byte(0XA0+((ReadAddr/256)<<1));   //����������ַ0XA0,д���� 	 

	IIC_Wait_Ack(); 
    IIC_Send_Byte(ReadAddr%256);   //���͵͵�ַ
	IIC_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte(0XA1);           //�������ģʽ			   
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();//����һ��ֹͣ����	   
	return temp;
}
//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{				   	  	    																 
   IIC_Start(); 
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(0XA0);	    //����д����
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr>>8);//���͸ߵ�ַ
 	}else
	{
		IIC_Send_Byte(0XA0+((WriteAddr/256)<<1));   //����������ַ0XA0,д���� 
	}	 
	IIC_Wait_Ack();	   
    IIC_Send_Byte(WriteAddr%256);   //���͵͵�ַ
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(DataToWrite);     //�����ֽ�							   
	IIC_Wait_Ack();  		    	   
    IIC_Stop();//����һ��ֹͣ���� 
	delay_ms(5);
}
//��AT24CXX�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//��AT24CXX�����ָ����ַ��ʼ��������ΪLen������
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len)
{  	
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}
//���AT24CXX�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(255);//����ÿ�ο�����дAT24CXX			   
	if(temp==0X55)return 0;		   
	else//�ų���һ�γ�ʼ�������
	{
		AT24CXX_WriteOneByte(255,0X55);
	    temp=AT24CXX_ReadOneByte(255);	  
		if(temp==0X55)
			return 0;
	}
	return 1;											  
}

//��AT24CXX�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
/*
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  */
//��AT24CXX�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
/*
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}*/
 
uint8_t AT24CXX_Write(u16 _usAddress,u8 *_pWriteBuf,u16 _usSize)
{
	uint16_t i,m;
	uint16_t usAddr;
	
	/* 
		д����EEPROM�������������������ȡ�ܶ��ֽڣ�ÿ��д����ֻ����ͬһ��page��
		����24xx02��page size = 8
		�򵥵Ĵ�����Ϊ�����ֽ�д����ģʽ��ûд1���ֽڣ������͵�ַ
		Ϊ���������д��Ч��: ����������page wirte������
	*/
	
	
	
	usAddr = _usAddress;	
	for (i = 0; i < _usSize; i++)
	{
		/* �����͵�1���ֽڻ���ҳ���׵�ַʱ����Ҫ���·��������źź͵�ַ */
		if ((i == 0) || (usAddr & (EEPROM_PAGE_SIZE - 1)) == 0)
		{
			/*����0������ֹͣ�źţ������ڲ�д������*/
			//i2c_Stop();
			IIC_Stop();
			/* ͨ���������Ӧ��ķ�ʽ���ж��ڲ�д�����Ƿ����, һ��С�� 10ms 			
				CLKƵ��Ϊ200KHzʱ����ѯ����Ϊ30������
			*/
			for (m = 0; m < 1000; m++)
			{				
				/* ��1��������I2C���������ź� */
				//i2c_Start();
				IIC_Start();
				/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
				//i2c_SendByte(EEPROM_DEV_ADDR | EEPROM_I2C_WR);	/* �˴���дָ�� */
				IIC_Send_Byte(EEPROM_DEV_ADDR | EEPROM_I2C_WR);
				/* ��3��������һ��ʱ�ӣ��ж������Ƿ���ȷӦ�� */
				if (IIC_Wait_Ack()==0)//i2c_WaitAck() == 0)
				{
					break;
				}
			}
			if (m  == 1000)
			{
				goto cmd_fail;	/* EEPROM����д��ʱ */
			}
		
			/* ��4���������ֽڵ�ַ��24C02ֻ��256�ֽڣ����1���ֽھ͹��ˣ������24C04���ϣ���ô�˴���Ҫ���������ַ */
			//i2c_SendByte((uint8_t)usAddr);
			IIC_Send_Byte(usAddr>>8);
			IIC_Wait_Ack();
			IIC_Send_Byte(usAddr&0xff);
			//IIC_Wait_Ack();
			/* ��5�����ȴ�ACK */
			if (IIC_Wait_Ack()!=0)//i2c_WaitAck() != 0)
			{
				goto cmd_fail;	/* EEPROM������Ӧ�� */
			}
		}
	
		/* ��6������ʼд������ */
		//i2c_SendByte(_pWriteBuf[i]);
		IIC_Send_Byte(_pWriteBuf[i]);
		/* ��7��������ACK */
		if (IIC_Wait_Ack()!=0)//i2c_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM������Ӧ�� */
		}

		usAddr++;	/* ��ַ��1 */		
	}
	
	/* ����ִ�гɹ�������I2C����ֹͣ�ź� */
	//i2c_Stop();
	IIC_Stop();
	return 1;

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	//i2c_Stop();
	IIC_Stop();
	return 0;
}




uint8_t AT24CXX_Read(u16 _usAddress,u8 *_pReadBuf, u16 _usSize)
{
	uint16_t i;
	
	/* ���ô���EEPROM�漴��ȡָ�����У�������ȡ�����ֽ� */
	
	/* ��1��������I2C���������ź� */
	IIC_Start();
	
	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	IIC_Send_Byte(EEPROM_DEV_ADDR | EEPROM_I2C_WR);	/* �˴���дָ�� */
	 
	/* ��3�����ȴ�ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��4���������ֽڵ�ַ��24C02ֻ��256�ֽڣ����1���ֽھ͹��ˣ������24C04���ϣ���ô�˴���Ҫ���������ַ */
	IIC_Send_Byte(_usAddress>>8);	//�ߵ�ַ
	IIC_Wait_Ack();	
	IIC_Send_Byte(_usAddress&0xff);		//�͵�ַ
	/* ��5�����ȴ�ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}
	
	/* ��6������������I2C���ߡ�ǰ��Ĵ����Ŀ����EEPROM���͵�ַ�����濪ʼ��ȡ���� */
	IIC_Start();
	
	/* ��7������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	IIC_Send_Byte(EEPROM_DEV_ADDR | EEPROM_I2C_RD);	/* �˴��Ƕ�ָ�� */
	
	/* ��8��������ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}	
	
	/* ��9����ѭ����ȡ���� */
	for (i = 0; i < _usSize; i++)
	{
		_pReadBuf[i] = IIC_Read_Byte_2();	/* ��1���ֽ� */
		
		/* ÿ����1���ֽں���Ҫ����Ack�� ���һ���ֽڲ���ҪAck����Nack */
		if (i != _usSize - 1)
		{
			IIC_Ack();	/* �м��ֽڶ����CPU����ACK�ź�(����SDA = 0) */
		}
		else
		{
			IIC_NAck();	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
		}
	}
	/* ����I2C����ֹͣ�ź� */
	IIC_Stop();
	return 1;	/* ִ�гɹ� */

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	IIC_Stop();
	return 0;
}




