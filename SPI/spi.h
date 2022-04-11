#ifndef __SPI_H
#define __SPI_H	 
#include "sys.h"


//#define SPI_CS_PORT     GPIOB
//#define SPI_CS_PIN      GPIO_Pin_11
//#define SPI_CS_LOW      (SPI_CS_PORT->BRR |=SPI_CS_PIN)
//#define SPI_CS_HIGH     (SPI_CS_PORT->BSRR |=SPI_CS_PIN)

#define SPI_SCK_PORT    GPIOB
#define SPI_SCK_PIN     GPIO_Pin_13
#define SPI_SCK_LOW     (SPI_SCK_PORT->BRR |=SPI_SCK_PIN)
#define SPI_SCK_HIGH    (SPI_SCK_PORT->BSRR|=SPI_SCK_PIN)

#define SPI_MISO_PORT    GPIOB
#define SPI_MISO_PIN     GPIO_Pin_14
#define SPI_MISO_LOW     (SPI_MISO_PORT->BRR |=SPI_MISO_PIN)
#define SPI_MISO_HIGH    (SPI_MISO_PORT->BSRR|=SPI_MISO_PIN)
#define SPI_MISO_READ    (SPI_MISO_PORT->IDR &SPI_MISO_PIN)

#define SPI_MOSI_PORT    GPIOB
#define SPI_MOSI_PIN     GPIO_Pin_15
#define SPI_MOSI_LOW     (SPI_MOSI_PORT->BRR |=SPI_MOSI_PIN)
#define SPI_MOSI_HIGH    (SPI_MOSI_PORT->BSRR|=SPI_MOSI_PIN)

void SPI_GPIO_Config(void);
u16 spi_start(u16 add);

extern u32 background;

u16 neg_convert_pos(u16 num);       //数值转化


u16 SPI_RW(u16 data);            //读写数据时序

u8 SPI_Moni_Write_Reg(u8 Reg,u8 data) ;
u16 SPI_Moni_Read_Reg(u16 Reg);
u8 SPI_Moni_Write_Buf(u8 Reg,u8 *Buf,u8 len);
u8 SPI_Moni_Read_Buf(u8 Reg,u8 *Buf,u8 len);


#endif
