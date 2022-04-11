#ifndef GLT_H_
#define GLT_H_
#include "usart.h"
#include "crc.h"
#include "master.h"
#include "delay.h"
#include "timer.h"
#include "i2c_ee.h"



//extern u8 spArr[4096];
//extern u16 size;



//USART2中断  光谱仪数据接口
void USART2_IRQHandler(void);
//设置积分时间  us
void SP_Set_IntegTime(u32 time);
//获取积分时间 us
void SP_Get_IntegTime(void);

//获取光谱仪数据
void SP_Get_SPData(void);

//配置氙灯脉冲高低时间
void SP_Set_Pulse_Time(u8 *bufTime);
void SP_Get_Pulse_Time(void);	//获取氙灯脉冲时间
//打开氙灯 并配置模式
void SP_Open_Light(u8 mode);
//获取模式
void SP_Get_Light_Mode(void);
//获取波长值
void SP_Get_WaveLength(void);


//复位
void SP_Reset(void);


//设置平均次数

void SP_Set_Average(u16 count);
void SP_Get_Average(void);	//获取平均次数
#endif

