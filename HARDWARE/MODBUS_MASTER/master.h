#ifndef __MASTER_H
#define __MASTER_H
#include "sys.h"
#include "shutter.h"
#include "sht20.h"
#include "motor.h"
#include "fitting_algorithm.h"
#define READ_COIL     01
#define READ_DI       02
#define READ_HLD_REG  03
#define READ_AI       04
#define SET_COIL      05
#define SET_HLD_REG   06
#define WRITE_COIL    15
#define WRITE_HLD_REG 16


extern u8 timeFlag;
extern u8 isNeedReadRatios;


#define RS485_TX_EN PAout(8)

#define HI(n) ((n)>>8)
#define LOW(n) ((n)&0xff)

#define	GET_BIT(x, bit)	((x & (1 << bit))>>bit)



#define SCREEN_USART			USART1
#define SCREEN_USART_PORT		GPIOA
#define SCREEN_USART_PIN_TX		GPIO_Pin_9
#define SCREEN_USART_PIN_RX		GPIO_Pin_10
#define	SCREEN_USART_IRQN		USART1_IRQn



#define SP_MAX_SIZE			5000	//波长返回数量



void RS485_Init(void);
void Timer4_Init(u16 arr);

//void Modbus_RegMap(void);

//写寄存器映射
void Modbus_RegMap(u16 regStartAddr,u16*buf,u16 len);
	
void Modbus_01_Solve(void);
void Modbus_02_Solve(void);
void Modbus_03_Solve(void);
void Modbus_05_Solve(void);
void Modbus_06_Solve(void);
void Modbus_15_Solve(void);
void Modbus_16_Solve(void);
	
void Timer3_Init(void);
void RS485_TX_Service(void);
void Master_Service(u8 SlaverAddr,u8 order,u16 addr,u16 value);
void RS485_RX_Service(void);
void modbus_rtu(void);
void RS485_SendByte(u8 data);
void RS485_SendData(u8 *buf,u16 len);
void USART1_IRQHandler(void);

//恢复出厂设置
void Restore_Factory_Setting(void);

//上电初始化各种参数
void Init_All_Argument(void);
//定时器2 中断
//表示光谱仪接收完毕
void TIM2_IRQHandler(void);
//TIM4 定时器中断
//循环发送检测屏幕按钮状态
void TIM4_IRQHandler(void) ;


//循环检测
void Check_SPData(void);




//上位机交互接口
//设置从机站点 保存在EEPROM中 位置 0x0000
void Set_Slave_Addr(void);
void Send_Slave_Addr(void); //发送从机地址

//设置光谱仪平均次数
void Set_SP_Average_Time(void);
void Get_SP_Average_Time(void); //获取平均次数
//脉冲模式
void Set_SP_Pulse_Mode(void);
//脉冲高低时间
void Set_SP_Pulse_Time(void);
void Get_SP_Pulse_Time(void);	//获取脉冲高低时间
//设置积分时间
void Set_SP_IntegTime(void);
//获取积分时间
void Get_SP_IntegTime(void);



//写入用户设置的系数
void Write_User_Ratios_To_Rom(u8*buf);

//初始化时获取波长
void Init_Get_Wave(void);

//读取ROM中的光谱
void Read_220_275_Spectrum(u16 addr,double*data_220,double*data_275);

void Write_Ratio_To_Rom(void); //写入系数
void Send_Ratio_To_USART(void);	//发送系数到上位机

void Set_Calc_Wave(void);		//设置用来计算的物质波长
void Read_Calc_Wave(void);	//读取某种物质的计算波长
void Write_Material_Wave_To_Rom(u8 type,u16 A1,u16 A2);	//写入某种物质计算波长

void Reset_Factory_Setting(void);	//恢复出厂设置

void Set_Clear_Period(void); //设置清洁周期

void Get_Sample_Spectrum(void);	//获取样品光谱并发送给上位机

void Get_Dark_Spectrum(void);		//获取暗光谱

void Get_Reference_Spectrum(void);		//获取参比光谱

void Get_White_Spectrum(void);	//获取白光谱

//写入光谱  EEPROM 从0x1000起
void Write_DarkSp_To_Rom(void);   //写入暗光谱

void Write_ReferenceSp_To_Rom(void);  //写入参比光谱

void Write_WhiteSp_To_Rom(void);		//写入白光谱


void Clear_Device(void);	//清洁设备
void Send_Light_Mode(void);	//发送光谱仪清洁模式给上位机
void Send_Temp_Humidity(void);	//温湿度给上位机

//发送EEPROM 存放的光谱 到 USART1 串口

void Send_DarkSp_To_USART(void);	//发送暗光谱

void Send_ReferenceSp_To_USART(void); //发送参考光谱

void Send_WhiteSp_To_USART(void); 	//发送白光谱

void Send_WaveCount_To_USART(void);	//发送波长数量

void Send_WaveData_To_USART(void);	//发送波长数据

void CalculateWaveLength(void);		//计算波长值



//屏幕交互

void setAlarmState(int temp1,int temp2,int humidity1,int humidity2);	//设置各种报警状态

void  Calc_Query_Btn(void); //取消确定按钮
void  Process_Ratios_From_User(void);	//处理用户的系数
void  Send_Order_Read_Ratios(void);		//发送命令读取用户系数
void  Read_User_Set_Ratios(void);		//读取用户写入的系数

void  Send_All_Info_To_Screen(void);
void  Send_Time_To_Screen(void);		//写入当前测量值，累计工作时长，下一次测量时间
void  Send_Temp_Humidity_To_Screen(void);	//发送温湿度到屏幕
void  Check_Btn_State(void);		//检测屏幕按钮状态
//void  Check_Calibration_State(void);	//检测校准按钮状态
void  Analysis_Btn_State(void);		//分析按钮状态
void  Send_Value_To_USER(void);	//用用户设定的系数计算
void Switch_Rubber_Brush(void);		//更换橡胶刷
void Detector_Diagnosis(void);		//探头诊断
void Detctor_Calibration(void);		//探头校准

void Select_Concentration(u8 index,u16 switchAddr,u16 valueAddr);




void Start_Darw(void);		//开始画图
void Stop_Draw(void);		//停止画图

//void Send_Zero_Concentration_To_Screen(void);	//发送浓度0到屏幕
//void Send_One_Concentration_To_Screen(void);	//发送浓度1


void Reset_All_Setting(void);			//重置所有设置
void Get_All_Setting(void);				//获取所有设置
void Set_All_Setting(void);				//将屏幕的设置应用




//-------------------------------------
//计算原始的浓度值
//void calculateConcentration(void);
//void getCalibratedData(void);		// 获取标定后的浓度值 x :原始浓度值

double calculateConcentration(void);

void CalculateValue(void);
double getCalibratedData(double x);

//计算参数
void calculateCalibrationRatios(void);

//获取参数
//获取上位机发送过来的参数
//多种物质时
void getRatios(u8 type,u16 startAddr);
//单种物质
void getRatios_Sample(u16 startAddr);
void getAllInitRatios(void);	//获取所有物质的计算系数
#endif









