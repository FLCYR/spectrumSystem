#ifndef __IIC_H
#define __IIC_H	 
#include "sys.h"

//IO方向设置
#define SDA_IN1()  {GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(u32)8<<28;}
#define SDA_OUT1() {GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(u32)3<<28;}

#define SDA_IN2()  {GPIOB->CRH&=0XFF0FFFFF;GPIOB->CRH|=(u32)8<<20;}
#define SDA_OUT2() {GPIOB->CRH&=0XFF0FFFFF;GPIOB->CRH|=(u32)3<<20;}

//IO操作函数	 
#define IIC_SCL1    PBout(14) //SCL
#define IIC_SDA1    PBout(15) //SDA	 
#define READ_SDA1   PBin(15)  //输入SDA 
//IO操作函数	 
#define IIC_SCL2    PBout(12) //SCL
#define IIC_SDA2    PBout(13) //SDA	 
#define READ_SDA2   PBin(13)  //输入SDA 

//IIC所有操作函数
void SHT_IIC_Init(void);                //初始化IIC的IO口	

void SHT1_IIC_Start(void);
void SHT2_IIC_Start(void);

void SHT2_IIC_Stop(void);
void SHT2_IIC_Stop(void);

u8 SHT1_IIC_Wait_Ack(void);
u8 SHT2_IIC_Wait_Ack(void);

void SHT1_IIC_Ack(void);
void SHT2_IIC_Ack(void);

void SHT1_IIC_NAck(void);
void SHT2_IIC_NAck(void);

void SHT1_IIC_Send_Byte(u8 txd);
void SHT2_IIC_Send_Byte(u8 txd);
	  			
u8 SHT1_IIC_Read_Byte(unsigned char ack);
u8 SHT2_IIC_Read_Byte(unsigned char ack);


void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  




#ifdef __cplusplus
 extern "C" {
#endif

#define  I2C_ADR_W 0x80  //SHT2X IIC 读地址
#define  I2C_ADR_R 0x81  //SHT2X IIC 写地址

typedef enum {
    TRIG_TEMP_MEASUREMENT_HM   = 0xE3, // 触发 温度 测量 ，保持主机   命令
    TRIG_HUMI_MEASUREMENT_HM   = 0xE5, // 触发 湿度 测量 ，保持主机   命令
    TRIG_TEMP_MEASUREMENT_POLL = 0xF3, // 触发 温度 测量 ，非保持主机 命令
    TRIG_HUMI_MEASUREMENT_POLL = 0xF5, // 触发 湿度 测量 ，非保持主机 命令
    USER_REG_W                 = 0xE6, // 写寄存器命令
    USER_REG_R                 = 0xE7, // 读寄存器命令
    SOFT_RESET                 = 0xFE  // 软件复位命令
} SHT2xCommand;

typedef enum {
    SHT2x_RES_12_14BIT         = 0x00, //RH=12bit, T=14bit 这是默认的值   我们不用修改
    SHT2x_RES_8_12BIT          = 0x01, //RH= 8bit, T=12bit
    SHT2x_RES_10_13BIT         = 0x80, //RH=10bit, T=13bit
    SHT2x_RES_11_11BIT         = 0x81, //RH=11bit, T=11bit
    SHT2x_RES_MASK             = 0x81  //Mask for res. bits (7,0) in user reg.
} SHT2xResolution;

typedef enum {
    SHT2x_HEATER_ON            = 0x04, //heater on
    SHT2x_HEATER_OFF           = 0x00, //heater off
    SHT2x_HEATER_MASK          = 0x04  //Mask for Heater bit(2) in user reg.
} SHT2xHeater;

typedef struct{
    float TEMP_POLL;
    float HUMI_POLL;    
} SHT2x_data;

extern SHT2x_data SHT20;

u8 SHT2x_Init(void); 
u8    SHT2x_SoftReset(void);  //SHT20软件复位
float SHT2x_GetTempPoll1(void);//获取SHT20 温度
float SHT2x_GetHumiPoll1(void);//获取SHT20 湿度

float SHT2x_GetTempPoll2(void);//获取SHT20 温度
float SHT2x_GetHumiPoll2(void);//获取SHT20 湿度
#ifdef __cplusplus
}
#endif




#endif
