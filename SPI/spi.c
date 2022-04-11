#include "spi.h"
#include "delay.h"

void SPI_GPIO_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

//    //CS
//    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
//    GPIO_InitStruct.GPIO_Pin=SPI_CS_PIN;
//    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
//    GPIO_Init(SPI_CS_PORT, &GPIO_InitStruct);
    //SCK
    GPIO_InitStruct.GPIO_Pin=SPI_SCK_PIN;
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_SCK_PORT,&GPIO_InitStruct);
    //MISO
    GPIO_InitStruct.GPIO_Pin=SPI_MISO_PIN;
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    GPIO_Init(SPI_MISO_PORT,&GPIO_InitStruct);
    //MOSI
    GPIO_InitStruct.GPIO_Pin=SPI_MOSI_PIN;
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_Init(SPI_MOSI_PORT,&GPIO_InitStruct);  

//    SPI_CS_HIGH;
    SPI_SCK_LOW;
}
//

u16 SPI_RW(u16 data)            //¶ÁÐ´Êý¾ÝÊ±Ðò
{
    u8 i;
		u16 temp;
		temp = data;
    SPI_SCK_LOW;     //???????
    for(i=0;i<16;i++)
    {   
			if(data&0x8000)  //?????
			{
					SPI_MOSI_HIGH;
			}
			else
			{
					SPI_MOSI_LOW;
			}
			delay_us(1);
			SPI_SCK_HIGH;
			
			delay_us(1);
			data <<= 1;
			
			delay_us(1);
			if(SPI_MISO_READ)   //??????????
			{
				data|=0x0001;    
				delay_us(1);					
			}
			SPI_SCK_LOW;     //????????????
    }
		
		for(i = 0; i < 16; i++)
		{
			if(temp & 0x8000)
			{
				SPI_MOSI_HIGH;
			}
			else
			{
				SPI_MOSI_LOW;
			}
			delay_us(1);
			SPI_SCK_HIGH;
			
			delay_us(1);
			temp <<= 1;
			
			delay_us(1);
			if(SPI_MISO_READ)   //??????????
			{
				temp |= 0x0001;    
				delay_us(1);					
			}
			SPI_SCK_LOW;     //????????????
		}
		SPI_MOSI_LOW;
		
    return data;         //????????
}
//

/*//u8 SPI_RW(u8 data)
//{
//    u8 i;
//    SPI_SCK_LOW;     //???????
//    for(i=0;i<8;i++)
//    {   
//        if((data&0x80)==0x80)  //?????
//        {
//            SPI_MOSI_HIGH;
//        }
//        else
//        {
//            SPI_MOSI_LOW;
//        }

//        SPI_SCK_HIGH;  //??????,??????,????????

//        data<<=1;

//        if(SPI_MISO_READ)   //??????????
//        {
//            data|=0x01;     
//        }
//        SPI_SCK_LOW;     //????????????
//    }

//    return data;         //????????
//}*/
//


u8 SPI_Moni_Write_Reg(u8 Reg,u8 data) 
{
	u8 states;
	
//	SPI_CS_LOW;  //??CSN??
	states=SPI_RW(Reg);  //????????,????Cn?,??????
	SPI_RW(data);       // ??????
//	SPI_CS_HIGH;
	
	return states;
}
//

u16 SPI_Moni_Read_Reg(u16 Reg)
{
    u16 data;

//    SPI_CS_LOW;
    SPI_RW(Reg);   //?????????
    data=SPI_RW(0); //????????0,???????????
//    SPI_CS_HIGH;

    return data;
}
//

u8 SPI_Moni_Write_Buf(u8 Reg,u8 *Buf,u8 len)
{
    u8 states;

//    SPI_CS_LOW;
    states=SPI_RW(Reg);
    while(len>0)
    {
        SPI_RW(*Buf);
        Buf++;
        len--;
    }
//    SPI_CS_HIGH;

    return states;
}
//

u8 SPI_Moni_Read_Buf(u8 Reg,u8 *Buf,u8 len)
{
    u8 states;

//    SPI_CS_LOW;
    states=SPI_RW(Reg);
    while(len>0)
    {
        *Buf=SPI_RW(0);
        Buf++;
        len--;
    }
//    SPI_CS_HIGH;

    return states;
}
//

u16 spi_start(u16 add)
{
	u16 spi;
	spi = (int)SPI_Moni_Read_Reg(add); 
//			SPI_Moni_Read_Reg(0); 
//			spidata = SPI2_ReadWriteByte(0x420a);
//			SPI2_ReadWriteByte(0);
	SPI_SCK_LOW;
	return spi;
}

