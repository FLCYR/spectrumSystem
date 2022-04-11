//	主机服务程序    //

#include"master.h"
#include"usart.h"
#include"delay.h"
#include "led.h"
#include "crc.h"
#include "i2c_ee.h"
#include "glt.h"
#include "shutter.h"
#include "addrMap.h"
#include "math.h"

u8 isNeedReadRatios=1;
u8 isNeedProcess=1;  //是否需要处理返回的数据
u8 timeFlag=0;		//定时器中断产生标志
u8 sendCompleteFlag=1; //RS485发送完成标志
u8 isCalcQueryBtn=0;
u8 spArr[5000]={1,2,3,4,5,6,6,7}; //接收光谱仪缓冲区
u16 size=0;



u32 RS485_Baudrate=9600;//通讯波特率
u8 RS485_Parity=0;//0无校验；1奇校验；2偶校验
u16 RS485_Frame_Distance=4;//数据帧最小间隔（ms),超过此时间则认为是下一帧

u8 RS485_RX_BUFF[512];//接收缓冲区2048字节
u16 RS485_RX_CNT=0;//接收计数器
u8 RS485_RxFlag=0;//接收一帧结束标记

u8 RS485_TX_BUFF[128];//发送缓冲区
u16 RS485_TX_CNT=0;//发送计数器
u8 RS485_TxFlag=0;//发送一帧结束标记


/////////////////////////////////////////////////////////////////////////////////////
//主机命令区
u8   SlaverAddr=0x01;    //默认从机地址
u8   Fuction=0;      // 功能码	//0x10
u16  StartAddr=0;    //起始地址
u16  ValueOrLenth=1;  //数据or长度
//////////////////////////////////////////////////////////////////////////////////////////

u8 TX_RX_SET=0; //发送，接受命令切换。 0 发送模式 1接受模式
u8 ComErr=8; //0代表通讯正常
             //1代表CRC错误
			// 2代表功能码错误 
//-------------------------------------------------			
//屏幕相关变量
//查询按钮状态命令

u16 MeansurePeriodArr[]={15,20,30,60,120,180,240,300,600,1200,1800};
u32 ClearPeriodArr[]={60,120,180,240,300,600,1200,1800,3600,7200,10800,21600,43200};

double totalTime=0;	//运行的总时长 秒
	
double curValue=0;	//当前测量值  
double originValue=0; //原始浓度值

u16 nextMeansureTime=60;  //距离下一次测量时间
u16 setNextMeansureTime=60;	//用户设置的下一次测量时间

u32 clearPeriod=60; //清洁周期
u32	setClearPeriod=60;	//用户设置的清洁周期

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Master寄存器和单片机寄存器的映射关系

u8 spectrumData[3000]; //保存光谱数据

u16 spSize=4;			//光谱数组数据个数
u8 isSendSpectrum=0; //是否发送光谱数据

float waveLengthData[1026]; //波长数据
u8 waveLengthBuf[4200];		//要发送的波长数据	
u8 isGetWaveLength=0;	//是否读取了波长
u8 isWaveFlag=0;		//是否是波长数据
u8 isWaveSendFlag=0;		//是否发送了波长数据
u8 isGetSPIntegTime=0;		//是否获取光谱仪积分时间
u8 isGetPulseTime=0;		//是否获取脉冲时间
u8 isGetLightMode=0;
u8 isGetAverageTime=0;
u16 waveSize=0;		//当前波长数组个数
u16	waveSendLen=0;	//发送波长数据数组的长度

u8 spReadyNext=0;	//光谱仪是否可以进行下一条命令发送

u8 SP_RX_COMPLETE=0;	//光谱仪数据接收完成

u8 spMode=0x01;			//光谱仪氙灯的模式 00 关 01 连续  0x81单次



//五点标定
u8 caliBrationFlag=0;	//是否发送检测标定的命令
double xConcentration[8];
double yConcentration[8]={0,0.5,1.0,2.0,5.0,10.0,20.0,25.0};
u8     conValueFlag[8]={0};//选择哪个数值进行校准
u8 alarmTempFlag;	//温度警报
u8 alarmHumidityFlag=1;	//湿度警报
u8 isReSetAlarm=1;


//是否为参考光谱
u8 refFlag=0;
//是否为样品光谱
u8 sampleFlag=0;
//是否开始画图
u8 isDraw=0;
//浓度计算参数

//系数
double SP_a=10;
double SP_b=10;
double SP_c=100;
double SP_d=1000;

//10组系数	每组三个
//二阶拟合 y=ax^2+bx+c
double spRatios[30];


//五种物质
//No3-N		220  275
//COD		254  576
//BOD		254	 576
//TOC		254	 576
//TURB		660	 0  
u16 Sample_Data_Wave[20]={220,275,254,576,254,576,254,576,660,0};
u16 CurRef_Data_Wave[20]={220,275,254,576,254,576,254,576,660,0};

double CurSample_Data[10];	
double CurRef_Data[10]; 

double concentration[5];	//五种物质的浓度
double calibrationValue[5];	//经过校准的浓度

//标定好的光谱数据
double Ref_Data[10];	//参考
double White_Data[10];	//白光谱
double Dark_Data[10];	//暗光谱


//当前样本光谱
double Sample_220_Data=0;
double Sample_275_Data=0;
//当前参考光谱
double Cur_Reference_220_Data=0;
double Cur_Reference_275_Data=0;


//标定好的白光谱
double White_220_Data=0;
double White_275_Data=0;

//标定好的参考光谱
double Reference_220_Data=0;
double Reference_275_Data=0;

//标定好的暗光谱
double Dark_220_Data=0;
double Dark_275_Data=0;

//光谱系数
double ratio_220=0;
double ratio_275=0;



//光谱仪 接收完成指令 触发中断  TIM2
void TIM2_IRQHandler(void)
{
	 if(TIM_GetITStatus(GENERAL_TIM,TIM_IT_Update)!=RESET)
     {	
		
		TIM_ClearITPendingBit(GENERAL_TIM,TIM_IT_Update);//清除中断标志
		TIM_Cmd(GENERAL_TIM,DISABLE);//停止定时器
		SP_RX_COMPLETE=1;	//接收完成
		Check_SPData();
		spReadyNext=1;
		size=0;
     }
}

//
void USART2_IRQHandler(void)
{		

        u8 res;
    
        if(USART_GetITStatus(SP_USART,USART_IT_RXNE)!=RESET)
        {
                
                res=USART_ReceiveData(SP_USART); //读接收到的字节，同时相关标志自动清除
                
			
                if(size<SP_MAX_SIZE)
                {		
                        spArr[size]=res;
                        size++;
                        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//清除定时器溢出中断
						TIM_SetCounter(TIM2,0);//当接收到一个新的字节，将定时器3复位为0，重新计时
                        TIM_Cmd(TIM2,ENABLE);//开始计时
                }
        }
}



//处理接收到的光谱数据
void Check_SPData(void)
{
	u16 i;
	u16 crc;
	
	if(SP_RX_COMPLETE)
	{	
		
		if(spArr[0]==0x06)
		{
			if(isSendSpectrum)
			{	
				isSendSpectrum=0;
				for(i=9;i<size-6;i++)  //只获取有效数据
				{	
					spectrumData[spSize]=spArr[i];
					spSize++;
				}
				
				if(refFlag)
				{
					refFlag=0;
					for(i=0;i<10;i++)
					{	
						refFlag=0;
						Cur_Reference_220_Data=Fitting_WaveLength(&spectrumData[4],waveLengthData,waveSize,220);
						Cur_Reference_275_Data=Fitting_WaveLength(&spectrumData[4],waveLengthData,waveSize,275);
						//CurRef_Data[i]=Fitting_WaveLength(&spectrumData[4],waveLengthData,waveSize,CurRef_Data_Wave[i]);
					}
					
				}
				else if(sampleFlag)
				{	
					sampleFlag=0;
					for(i=0;i<10;i++)
					{	
						sampleFlag=0;
						Sample_220_Data=Fitting_WaveLength(&spectrumData[4],waveLengthData,waveSize,220);
						Sample_275_Data=Fitting_WaveLength(&spectrumData[4],waveLengthData,waveSize,275);
				
						//CurSample_Data[i]=Fitting_WaveLength(&spectrumData[4],waveLengthData,waveSize,Sample_Data_Wave[i]);
					}
				}
				else
				{
					crc=CRC_Compute(spectrumData,spSize);
					spectrumData[spSize]=crc&0xff;
					spSize++;
					spectrumData[spSize]=crc>>8;
					spSize++;
					RS485_SendData(spectrumData,spSize);
				}
			}
			
			if(isWaveFlag)
			{	
				
				if(size<SP_MAX_SIZE)
				{
					
					CalculateWaveLength();
				}
				isWaveFlag=0;
				isGetWaveLength=1;
			}
			//发送积分时间
			else if(isGetSPIntegTime)
			{	
				isGetSPIntegTime=0;
				spectrumData[spSize++]=spArr[3];
				spectrumData[spSize++]=spArr[4];
				spectrumData[spSize++]=spArr[1];
				spectrumData[spSize++]=spArr[2];
				crc=CRC_Compute(spectrumData,spSize);
				spectrumData[spSize]=crc&0xff;
				spSize++;
				spectrumData[spSize]=crc>>8;
				spSize++;
				RS485_SendData(spectrumData,spSize);
			}
			else if(isGetPulseTime)
			{
				isGetPulseTime=0;
				spectrumData[spSize++]=spArr[3];
				spectrumData[spSize++]=spArr[4];
				spectrumData[spSize++]=spArr[1];
				spectrumData[spSize++]=spArr[2];
				
				spectrumData[spSize++]=spArr[7];
				spectrumData[spSize++]=spArr[8];
				spectrumData[spSize++]=spArr[5];
				spectrumData[spSize++]=spArr[6];
				
				crc=CRC_Compute(spectrumData,spSize);
				spectrumData[spSize]=crc&0xff;
				spSize++;
				spectrumData[spSize]=crc>>8;
				spSize++;
				RS485_SendData(spectrumData,spSize);
			}
			
			else if(isGetAverageTime)
			{
				isGetAverageTime=0;
				spectrumData[spSize++]=spArr[1];
				spectrumData[spSize++]=spArr[2];
	
				crc=CRC_Compute(spectrumData,spSize);
				spectrumData[spSize]=crc&0xff;
				spSize++;
				spectrumData[spSize]=crc>>8;
				spSize++;
				RS485_SendData(spectrumData,spSize);
			}
			else if(isGetLightMode)
			{	
				isGetLightMode=0;
				spMode=spArr[1];
			}
		}
		
		
		size=0;
		SP_RX_COMPLETE=0; //处理完成可以重新接收SP串口数据
		
	}
	
}


//上位机交互
//写入位置
void Write_Ratio_To_Rom(void)//写入系数
{	
	//写系数进去
	u8 buf[8]={SlaverAddr,0x10,0x00,0x8b,0x00,0x0b};
	u16 crc;
	u16 addr;
	u16 type=(RS485_RX_BUFF[8]<<8)|RS485_RX_BUFF[9];
	if(type<0x0A)
	{	
		addr=SP_RATIO_ADDR+SP_RATIO_LEN*type;
		AT24CXX_Write(addr,&RS485_RX_BUFF[8],SP_RATIO_LEN);	//写入ROM
		delay_ms(10);
		//addr=SP_RATIO_FACTORY_ADDR+SP_RATIO_LEN*type;	//存放到ROM中 用于恢复出厂设置
		//wAT24CXX_Write(addr,&RS485_RX_BUFF[8],SP_RATIO_LEN);
		crc=CRC_Compute(buf,6);
		buf[6]=crc&0xff;
		buf[7]=crc>>8;
		delay_ms(5);
		getRatios(type,addr);	//获取系数	//赋值给对应系数
		RS485_SendData(buf,8);
	}
	
	
}

//读取ROM对应的系数到上位机
void Send_Ratio_To_USART(void)	//发送系数到上位机
{	
	u16 addr;
	u16 crc;
	u8 buf[28]={SlaverAddr,0x06,0x00,0x96};
	u16 type=(RS485_RX_BUFF[4]<<8)|RS485_RX_BUFF[5];
	if(type<0x0A)
	{
		addr=SP_RATIO_ADDR+SP_RATIO_LEN*type;
		AT24CXX_Read(addr,&buf[4],SP_RATIO_LEN);
		buf[4]=type>>8;;
		buf[5]=type&0xff;
		crc=CRC_Compute(buf,26);
		buf[26]=crc&0xff;
		buf[27]=crc>>8;
		RS485_SendData(buf,28);
	}
}

//恢复出厂设置
//......................
void Reset_Factory_Setting(void)	
{
	
}
//设置从机地址
void Set_Slave_Addr(void)
{
	
	SlaverAddr=RS485_RX_BUFF[5];
	AT24CXX_WriteOneByte(SP_SLAVE_ADDR,SlaverAddr); //保存好站点
	RS485_RX_BUFF[0]=SlaverAddr;
	RS485_SendData(RS485_RX_BUFF,8); //返回值
}

//发送从机地址

void Send_Slave_Addr(void)
{	
	u8 buf[10]={0xff,0x66,0xaa,0xbb,0xcc,0xdd};
	u16 crc;
	buf[6]=0x00;
	buf[7]=AT24CXX_ReadOneByte(SP_SLAVE_ADDR);
	crc=CRC_Compute(buf,3);
	buf[8]=crc&0xff;
	buf[9]=crc>>8;
	SlaverAddr=buf[7];
	RS485_SendData(buf,10);
}

//设置清洁周期
//保存在EEPROM      0x0001~0x0002
//分钟表示
void Set_Clear_Period(void)
{
	
	
	//设置清洁周期
	//并写入EEPROM
	clearPeriod=RS485_RX_BUFF[4];
	clearPeriod<<=8;
	clearPeriod|=RS485_RX_BUFF[5];
	
	AT24CXX_WriteOneByte(SP_CLEAR_PERIOD_ADDR,clearPeriod>>8);
	AT24CXX_WriteOneByte(SP_CLEAR_PERIOD_ADDR+1,clearPeriod&0xff);
	RS485_SendData(RS485_RX_BUFF,8);
	
}

//脉冲模式
void Set_SP_Pulse_Mode()
{
	spMode=RS485_RX_BUFF[5];
	SP_Open_Light(spMode);
	RS485_SendData(RS485_RX_BUFF,8);
}
//设置脉冲高低时间
void Set_SP_Pulse_Time()
{		
	
	u16 crc;
	u8 buf[8]={SlaverAddr,0x10,0x00,0x9e,0x00,0x04};
	u8 timeBuf[8];
	timeBuf[0]=RS485_RX_BUFF[9];
	timeBuf[1]=RS485_RX_BUFF[10];
	timeBuf[2]=RS485_RX_BUFF[7];
	timeBuf[3]=RS485_RX_BUFF[8];
	
	timeBuf[4]=RS485_RX_BUFF[13];
	timeBuf[5]=RS485_RX_BUFF[14];
	timeBuf[6]=RS485_RX_BUFF[11];
	timeBuf[7]=RS485_RX_BUFF[12];
	
	SP_Set_Pulse_Time(timeBuf);
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
}

void Get_SP_Pulse_Time(void)	//获取脉冲高低时间
{	
	isGetPulseTime=1;
	spSize=3;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x03;
	spectrumData[2]=0x08;
	SP_Get_Pulse_Time();
}
//积分时间
void Set_SP_IntegTime()
{	
	u32 spTime;
	u16 crc;
	u8 buf[8]={SlaverAddr,0x10,0x00,0x9c,0x00,0x02};
	spTime=(RS485_RX_BUFF[9]<<8)|RS485_RX_BUFF[10];
	spTime=(spTime<<8)|RS485_RX_BUFF[7];
	spTime=(spTime<<8)|RS485_RX_BUFF[8];
	SP_Set_IntegTime(spTime);
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
	//错误处理.....????
	
	
}

//获取积分时间
void Get_SP_IntegTime(void)
{	
	isGetSPIntegTime=1;
	spSize=3;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x03;
	spectrumData[2]=0x04;
	SP_Get_IntegTime();
	
}

//设置光谱仪平均次数
void Set_SP_Average_Time()
{
	SP_Set_Average((RS485_RX_BUFF[4]<<8)|RS485_RX_BUFF[5]);
	RS485_SendData(RS485_RX_BUFF,8);
}
void Get_SP_Average_Time(void) //获取平均次数
{
	isGetAverageTime=1;
	spSize=3;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x03;
	spectrumData[2]=0x0f;
	SP_Get_Average();
}

//获取样品光谱
//并发送给上位机
void Get_Sample_Spectrum(void)
{	
	spReadyNext=0;
	spSize=4;
	
	isSendSpectrum=0;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x05;
	spectrumData[2]=0x00;
	spectrumData[3]=0x87;
	//开灯
	Set_shutter(0);
	SP_Open_Light(spMode);
	delay_ms(200);
	while(!spReadyNext)	//等待上一个返回
		;
	isSendSpectrum=1;
	SP_Get_SPData();
	
}

//获取暗光谱

void Get_Dark_Spectrum(void)
{	
	
	//关灯....0x00关灯
	spReadyNext=0;
	spSize=4;
	Set_shutter(0);	 //挡片
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x05;
	spectrumData[2]=0x00;
	spectrumData[3]=0x88;
	isSendSpectrum=0;
	SP_Open_Light(0x00);
	delay_ms(10);
	//等待可以采集下一次
	while(!spReadyNext)
		;
	isSendSpectrum=1;
	SP_Get_SPData();
	
}


//获取参比光谱
void Get_Reference_Spectrum(void)
{	
	
	//Get_Dark_Spectrum();
	//打开挡片采集参比光谱
	Set_shutter(1);
	delay_ms(20);
	isSendSpectrum=0;
	spSize=4;
	spReadyNext=0;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x05;
	spectrumData[2]=0x00;
	spectrumData[3]=0x89;
	isSendSpectrum=0;
	//shutter=1;
	
	SP_Open_Light(spMode);//单次
	delay_ms(200);
	while(!spReadyNext)	//等待上一条命令返回 可以执行下一条命令
		;
	
	isSendSpectrum=1;
	SP_Get_SPData();
}

//获取白光谱
void Get_White_Spectrum(void)
{	
	//复位挡片采集白光谱

	Set_shutter(0);
	delay_ms(20);
	
	spReadyNext=0;
	isSendSpectrum=0;
	spSize=4;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x05;
	spectrumData[2]=0x00;
	spectrumData[3]=0x9a;
	SP_Open_Light(spMode);	//开灯
	delay_ms(200);
	while(!spReadyNext)		//设置超时时间....
		;
	isSendSpectrum=1;
	SP_Get_SPData();
}

//ROM 从0x1000开始
//写入暗光谱
void Write_DarkSp_To_Rom()
{	
	

	AT24CXX_Write(SP_DARK_SPECTRUM_ADDR,&spectrumData[4],SP_DARK_SPECTRUM_LEN);
	//AT24CXX_Page_Write(SP_DARK_SPECTRUM_ADDR,&spectrumData[4],SP_DARK_SPECTRUM_LEN);
	RS485_SendData(RS485_RX_BUFF,8);
	//RS485_SendByte('Q');
}

//写入参比光谱
//地址: SP_REFENCE_SPECTRUM
void Write_ReferenceSp_To_Rom()
{
	
	AT24CXX_Write(SP_REFENCE_SPECTRUM_ADDR,&spectrumData[4],SP_REFENCE_SPECTRUM_LEN);
	RS485_SendData(RS485_RX_BUFF,8);
}	

//白光谱
//标明是何种物质根据物质种类进行拓展
//开始地址:	SP_WHITE_SPECTRUM_ADDR
void Write_WhiteSp_To_Rom()
{	
	
	AT24CXX_Write(SP_WHITE_SPECTRUM_ADDR,&spectrumData[4],SP_WHITE_SPECTRUM_LEN);
	RS485_SendData(RS485_RX_BUFF,8);
}	

//发送标定好的暗光谱到上位机
void Send_DarkSp_To_USART(void)
{	
	u16 crc;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x06;
	spectrumData[2]=0x00;
	spectrumData[3]=0x97;
	
	AT24CXX_Read(SP_DARK_SPECTRUM_ADDR,&spectrumData[4],SP_DARK_SPECTRUM_LEN);
	
	crc=CRC_Compute(spectrumData,SP_DARK_SPECTRUM_LEN+4);
	spectrumData[SP_DARK_SPECTRUM_LEN+4]=crc&0xff;	//低8位
	spectrumData[SP_DARK_SPECTRUM_LEN+5]=crc>>8;	//高8位
	RS485_SendData(spectrumData,SP_DARK_SPECTRUM_LEN+6);
}
//发送标定好的参考光谱
void Send_ReferenceSp_To_USART(void)
{
	
	u16 crc;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x06;
	spectrumData[2]=0x00;
	spectrumData[3]=0x98;
	
	AT24CXX_Read(SP_REFENCE_SPECTRUM_ADDR,&spectrumData[4],SP_REFENCE_SPECTRUM_LEN);
	
	crc=CRC_Compute(spectrumData,SP_REFENCE_SPECTRUM_LEN+4);
	spectrumData[SP_REFENCE_SPECTRUM_LEN+4]=crc&0xff;	//低8位
	spectrumData[SP_REFENCE_SPECTRUM_LEN+5]=crc>>8;	//高8位
	RS485_SendData(spectrumData,SP_REFENCE_SPECTRUM_LEN+6);
}

//发送白光谱
void Send_WhiteSp_To_USART(void)
{	
	u16 crc;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x06;
	spectrumData[2]=0x00;
	spectrumData[3]=0x99;
	AT24CXX_Read(SP_WHITE_SPECTRUM_ADDR,&spectrumData[4],SP_WHITE_SPECTRUM_LEN);
	crc=CRC_Compute(spectrumData,SP_WHITE_SPECTRUM_LEN+4);
	spectrumData[SP_WHITE_SPECTRUM_LEN+4]=crc&0xff;
	spectrumData[SP_WHITE_SPECTRUM_LEN+5]=crc>>8;
	RS485_SendData(spectrumData,SP_WHITE_SPECTRUM_LEN+6);
	
}

//发送波长数量
void Send_WaveCount_To_USART(void)
{	
	u16 crc;
	u16 waveCount=0;	//波长数量
	u8 buf[7]={SlaverAddr,0x03,0x02};
	spReadyNext=0;
	if(!isGetWaveLength)
	{
		isGetWaveLength=0;
		isWaveFlag=1;
		SP_Get_WaveLength();
		while(!spReadyNext)
			;
	}
	waveCount=waveSize;
	buf[3]=waveCount>>8;
	buf[4]=waveCount&0xff;
	crc=CRC_Compute(buf,5);
	
	buf[5]=crc&0xff;
	buf[6]=crc>>8;
	RS485_SendData(buf,7);
}
//获取波长数据
void Send_WaveData_To_USART(void)
{
	
	

	u16 crc=0;
	spReadyNext=0;
	u32 data=1;
	u16 len=4;
	if(!isGetWaveLength)
	{
		isGetWaveLength=0;
		isWaveFlag=1;
		SP_Get_WaveLength();
		while(!spReadyNext)
				;
	}
	
	if(isWaveSendFlag)
	{	
		waveLengthBuf[0]=SlaverAddr;
		RS485_SendData(waveLengthBuf,waveSendLen);
		return;
	}
	
	waveLengthBuf[0]=SlaverAddr;
	waveLengthBuf[1]=0x06;
	waveLengthBuf[2]=0x00;
	waveLengthBuf[3]=0x9a;
	
	for(int i=0;i<waveSize;i++)
	{
		
		data=(u32)(waveLengthData[i]*10000);
		waveLengthBuf[len++]=(data>>8)&0xff;
		waveLengthBuf[len++]=data&0xff;
		waveLengthBuf[len++]=(data>>24)&0xff;		
		waveLengthBuf[len++]=(data>>16)&0xff;
		
	}
	crc=CRC_Compute(waveLengthBuf,len);
	waveLengthBuf[len+1]=crc&0xff;
	waveLengthBuf[len+2]=crc>>8;
	waveSendLen=len+2;
	RS485_SendData(waveLengthBuf,len+2);
	isWaveSendFlag=1;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//初始化USART1
void RS485_Init(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        NVIC_InitTypeDef NVIC_InitStructure;
	
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE); //串口1
        
        GPIO_InitStructure.GPIO_Pin=SCREEN_USART_PIN_TX;//PA9（TX）复用推挽输出
        GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
        GPIO_Init(SCREEN_USART_PORT,&GPIO_InitStructure);
      
        
        GPIO_InitStructure.GPIO_Pin=SCREEN_USART_PIN_RX;//PA10（RX）输入上拉
        GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;   //修改原GPIO_Mode_IPU（输入上拉）->GPIO_Mode_IN_FLOATING(浮空输入)/////////////////////////////////////////////
        GPIO_Init(SCREEN_USART_PORT,&GPIO_InitStructure);
        
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;//修改PG9（RE/DE）通用推挽输出->PD7（RE/DE）通用推挽输出//////////////////////////////////////////////////////////////////////
        GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
        GPIO_Init(GPIOA,&GPIO_InitStructure);
        GPIO_ResetBits(GPIOA,GPIO_Pin_8);//默认接收状态
        
        USART_DeInit(SCREEN_USART);//复位串口1
        USART_InitStructure.USART_BaudRate=RS485_Baudrate;
        USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
        USART_InitStructure.USART_WordLength=USART_WordLength_8b;
        USART_InitStructure.USART_StopBits=USART_StopBits_1;
        USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//收发模式
        switch(RS485_Parity)
        {
                case 0:USART_InitStructure.USART_Parity=USART_Parity_No;break;//无校验
                case 1:USART_InitStructure.USART_Parity=USART_Parity_Odd;break;//奇校验
                case 2:USART_InitStructure.USART_Parity=USART_Parity_Even;break;//偶校验
        }
        USART_Init(SCREEN_USART,&USART_InitStructure);
        
        NVIC_InitStructure.NVIC_IRQChannel=SCREEN_USART_IRQN;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
        NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
        NVIC_Init(&NVIC_InitStructure);
		
		USART_ClearITPendingBit(SCREEN_USART,USART_IT_RXNE);
        USART_ITConfig(SCREEN_USART,USART_IT_RXNE,ENABLE);//使能串口1接收中
		
		USART_Cmd(SCREEN_USART,ENABLE);//使能串口1
		
        Timer3_Init();//定时器7初始化，用于监视空闲时间
        
}





void USART1_IRQHandler(void)//串口1中断服务程序
{		
	
		u8 res;
       if(USART_GetITStatus(SCREEN_USART,USART_IT_RXNE)!=RESET)
		//if (USART_GetFlagStatus(SCREEN_USART, USART_FLAG_ORE) != RESET )
        {		
				
                res=USART_ReceiveData(SCREEN_USART); //读接收到的字节，同时相关标志自动清除
				RS485_RX_BUFF[RS485_RX_CNT]=res;
				RS485_RX_CNT++;
				TIM_ClearITPendingBit(TIM3,TIM_IT_Update);//清除定时器溢出中断
				TIM_SetCounter(TIM3,0);//当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
				TIM_Cmd(TIM3,ENABLE);//开始计时
              
        }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//定时器4初始化
//
void Timer4_Init(u16 arr)	   	//TIM4使能
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能	
	TIM_TimeBaseStructure.TIM_Period = arr-1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler = 7199; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4,TIM_IT_Update, ENABLE );//TIM3 允许更新中断
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM4, ENABLE);  //使能TIMx外设	
	
}

void Timer4_disable (void)					   //TIM4失能
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, DISABLE); //时钟失能
	TIM_ITConfig(TIM4, TIM_IT_Update | TIM_IT_Trigger,DISABLE );
	TIM_Cmd(TIM4, DISABLE);  //失能TIMx外设
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
void TIM4_IRQHandler(void)   //TIM4中断
{	
	
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) //检查指定的TIM中断发生与否:TIM 中断源 
	{	
		timeFlag=1;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	
		//清除TIMx的中断待处理位:TIM 中断源 
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
//定时器3初始化---功能：判断从机返回的数据是否接受完成

void Timer3_Init(void)
{
        TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
        NVIC_InitTypeDef NVIC_InitStructure;

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //TIM3时钟使能 

        //TIM3初始化设置
        TIM_TimeBaseStructure.TIM_Period = RS485_Frame_Distance*10; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
        TIM_TimeBaseStructure.TIM_Prescaler =7199; //设置用来作为TIMx时钟频率除数的预分频值 设置计数频率为10kHz
        TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
		
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
        TIM_ITConfig( TIM3, TIM_IT_Update, ENABLE );//TIM3 允许更新中断

        //TIM3中断分组配置
        NVIC_InitStructure.NVIC_IRQChannel =TIM3_IRQn;  //TIM3中断
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
        NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器   
		TIM_Cmd(TIM3,DISABLE);
}



/////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////

//用定时器3判断接收空闲时间，当空闲时间大于指定时间，认为一帧结束
//定时器3中断服务程序         
void TIM3_IRQHandler(void)
{                                                                   
        if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)
        {		
				
                TIM_ClearITPendingBit(TIM3,TIM_IT_Update);//清除中断标志
                TIM_Cmd(TIM3,DISABLE);//停止定时器
                RS485_TX_EN=1;//停止接收，切换为发送状态
                RS485_RxFlag=1;//置位帧结束标记
        }
}

//////////////////////////////////////////////////////////////////////////////
//发送n个字节数据
//buff:发送区首地址
//len：发送的字节数
void RS485_SendData(u8 *buf,u16 len)
{ 		
		
		
        RS485_TX_EN=1;//切换为发送模式
        while(len--)
        {	
                while(USART_GetFlagStatus(SCREEN_USART,USART_FLAG_TXE)==RESET);//等待发送区为空
            
				USART_SendData(SCREEN_USART,*(buf++));
				
        }
        while(USART_GetFlagStatus(SCREEN_USART,USART_FLAG_TC)==RESET);//等待发送完成
		TX_RX_SET=1; //发送命令完成
		RS485_TX_EN=0;	//切换接收状态
}


//发送单个字节

void RS485_SendByte(u8 data)
{
	RS485_TX_EN=1;//切换为发送模式
	while(USART_GetFlagStatus(SCREEN_USART,USART_FLAG_TXE)==RESET)
		;
	USART_SendData(SCREEN_USART,data);
	while(USART_GetFlagStatus(SCREEN_USART,USART_FLAG_TC)==RESET);//等待发送完成
	RS485_TX_EN=0;	//切换接收状态
}


/////////////////////////////////////////////////////////////////////////////////////
//RS485服务程序

void RS485_RX_Service(void)
{
		
        if(RS485_RxFlag==1)
        {		
                if(RS485_RX_BUFF[0]==SlaverAddr)//地址正确
                {			
                        if((RS485_RX_BUFF[1]==0x01)||(RS485_RX_BUFF[1]==0x02)||(RS485_RX_BUFF[1]==0x03)||(RS485_RX_BUFF[1]==0x05)||(RS485_RX_BUFF[1]==0x06)||(RS485_RX_BUFF[1]==0x04)||(RS485_RX_BUFF[1]==16))//功能码正确
						{			
									
                                      // calCRC=CRC_Compute(RS485_RX_BUFF,RS485_RX_CNT-2);//计算所接收数据的CRC
                                      // recCRC=RS485_RX_BUFF[RS485_RX_CNT-1]|(((u16)RS485_RX_BUFF[RS485_RX_CNT-2])<<8);//接收到的CRC(低字节在前，高字节在后)
                                        /*-----------------------------*/
										if(1)//CRC校验正确	//不进行校验
                                        {		
											
											
											    
                                                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                switch(RS485_RX_BUFF[1])//根据不同的功能码进行处理
                                                {		
													
														
                                                        case 0x02://读输入开关量
                                                        {
                                                                Modbus_02_Solve();
                                                                break;
                                                        }
                                                        
                                                        case 0x01://读输出开关量
                                                        {		
																
															 
													
                                                                Modbus_01_Solve();
                                                                break;
                                                        }
                                                                
                                                        case 0x05://写单个输出开关量
                                                        {
                                                                Modbus_05_Solve();
                                                                break;
                                                        }
                                                                
                                                        case 15://写多个输出开关量
                                                        {
                                                                //Modbus_15_Solve();
                                                                break;
                                                        }
                                                
                                                        case 0x03: //读多个寄存器
                                                        {																
																Modbus_03_Solve();
																
                                                                break;
                                                        }
                                                                
                                                        case 0x06: //写单个寄存器
                                                        {
                                                                Modbus_06_Solve();
																
                                                                break;
                                                        }
                                                                
                                                        case 0x10: //写多个寄存器
                                                        {
                                                                Modbus_16_Solve();
                                                                break;
                                                        }
														default:
															
														break;
                                                                                        
                                                }
										}
                                        else//CRC校验错误
                                        {
												  ComErr=14;

                                        }
											
						}
                        else//功能码错误
                        {
							if((RS485_RX_BUFF[1]==0x81)||(RS485_RX_BUFF[1]==0x82)||(RS485_RX_BUFF[1]==0x83)||(RS485_RX_BUFF[1]==0x85)||(RS485_RX_BUFF[1]==0x86)||(RS485_RX_BUFF[1]==0x8F)||(RS485_RX_BUFF[1]==0x90))
							{
								switch(RS485_RX_BUFF[2])
								{
									case 0x01:
												ComErr=11;
												break;
									case 0x02:
												ComErr=12;
												break;
									case 0x03:
												ComErr=13;
												break;
									case 0x04:
												ComErr=14;
												break;
									
								}
								TX_RX_SET=0; //命令完成	
							}
							
							
                        }
          }
		  //发送从机地址
		  if(RS485_RX_BUFF[0] == 0xff && RS485_RX_BUFF[1]==0x66&&RS485_RX_BUFF[2]==0xaa && RS485_RX_BUFF[3]==0xbb)
		  {		
			  TIM_Cmd(TIM4, DISABLE);
			  timeFlag=0;
			  Send_Slave_Addr();
			  
		  }
                //ComErr=0;                
			RS485_RxFlag=0;//复位帧结束标志
			RS485_RX_CNT=0;//接收计数器清零
			TX_RX_SET=0; //命令完成
			RS485_TX_EN=0;//切换接收状态
			
        }
}



//Modbus功能码01处理程序 ///////////////////////////////////////////////////
//读输出开关量
void Modbus_01_Solve(void)   
{	
	
	
	switch(RS485_RX_BUFF[2])
	{	
		
		case 0x03:		//全部按钮状态
			Analysis_Btn_State();	//解析按钮状态
			break;	
		case 0x14:		//报警状态
			break;
		case 0x00:
			Send_Light_Mode();
			break;
	}
	//TX_RX_SET=0; //命令完成
	
		
}

//Modbus功能码02处理程序 //////////////////////////////////////////////////////
//读输入开关量
void Modbus_02_Solve(void)   
{
		
}



//Modbus功能码03处理程序//////////////////////////////////////////////////////////////////////////////////
//读保持寄存器
void Modbus_03_Solve(void)
{		
		
	
		switch(RS485_RX_BUFF[2])
		{
			case 0x0a:			//屏幕发送全部设置数据
				Set_All_Setting();
				break;
			case 0x0c:
				Read_User_Set_Ratios();	//读取用户在屏幕设置的参数
				break;
		}
		
		switch(RS485_RX_BUFF[3])
		{		
			case 0x9b:
				Send_WaveCount_To_USART();	//读取波长个数
				break;
			case 0x9c:
				Get_SP_IntegTime(); //获取光谱仪积分时间
				break;
			case 0x9d:
				Get_SP_Pulse_Time();	//获取光谱仪脉冲时间
				break;
			case 0x9e:
				Get_SP_Average_Time();	//获取平均次数
				break;
			case 0x10:
				Send_Temp_Humidity();	//发送温湿度给上位机
				break;
			case 0x0e:
				Read_Calc_Wave();		//读取要用来计算的波长值
				break;
		}
		
}

//Modbus功能码05处理程序   ///////////////////////////////////////////////////////
//写单个输出开关量
void Modbus_05_Solve(void)
{	
	
	
	switch(RS485_RX_BUFF[3])
	{	
		case 0x86:
			//恢复出厂设置
			Restore_Factory_Setting();
			break;
		case 0x87:
			Get_Sample_Spectrum();	//采集样品光谱 发给上位机
			break; 
		case 0x88:
			Get_Dark_Spectrum();	//采集暗光谱 并给上位机
			break;
		case 0x89:
			Get_Reference_Spectrum();	//采集参考光谱
			break;
		case 0x9a:
			Get_White_Spectrum();	//白光谱
			break;	
	}
	
}

//Modbus功能码06处理程序   //////////////////////////////////////////////////////////////////////////////////已验证程序OK
//写单个保持寄存器
void Modbus_06_Solve(void)
{		
		
		switch(RS485_RX_BUFF[3])
		{
			
			case 0x86:
				Set_Slave_Addr();	//设置站点
				break;
			case 0x87:
				Set_Clear_Period();	//设置清洁周期
				break;
			case 0x88:
				Write_DarkSp_To_Rom();	//写入暗光谱
				break;
			case 0x89:
				Write_ReferenceSp_To_Rom();	//写入参考光谱
				break;
			case 0x8a:
				Write_WhiteSp_To_Rom();		//写入白光谱
				break;
			case 0x96:					//读取系数
				Send_Ratio_To_USART();
				break;
			case 0x97:					//读取ROM暗光谱
				Send_DarkSp_To_USART();
				break;
			case 0x98:					//读取ROM参比光谱
				Send_ReferenceSp_To_USART();
				break;
			case 0x99:
				Send_WhiteSp_To_USART();	//读取ROM白光谱
				break;
			case 0x9a:
				Send_WaveData_To_USART();	//读取波长数据
				break;
			case 0x9c:					//设置平均次数
				Set_SP_Average_Time();
				break;
			case 0x9d:
				Set_SP_Pulse_Mode();	//设置脉冲模式
				break;
			case 0x9e:
				Clear_Device();
				break;
			default:
				break;
		}
	
		
}
//Modbus功能码15处理程序   //////////////////////////////////////////////////////程序已验证OK
//写多个输出开关量
void Modbus_15_Solve(void)
{
        u16 i;//数据返回校验用
        i=(((u16)RS485_RX_BUFF[4])<<8)|RS485_RX_BUFF[5];//获取寄存器数量
         if(i==ValueOrLenth)
		{
			ComErr=0;
		}
         else
		{
			ComErr=15;
		}
		TX_RX_SET=0; //命令完成   
}

//Modbus功能码16处理程序 /////////////////////////////////////////////////////////////////////////////////////////////////已验证程序OK
//写多个保持寄存器
void Modbus_16_Solve(void)
{		
	
	switch(RS485_RX_BUFF[3])
	{	
		//写入系数到ROM
		case 0x8b:
			Write_Ratio_To_Rom();
			break;
		case 0x9c:	//设置积分时间
			Set_SP_IntegTime();
			break;
		//设置脉冲高低时间
		case 0x9e:
			Set_SP_Pulse_Time();
			break;
		case 0x9f:
			Set_Calc_Wave();
			break;
		
	}
}

//-------------------------

//读取用户写入的系数
void Read_User_Set_Ratios(void)		
{
	int32_t ratios[3];
	int16_t temp;
	u8 i=0;
	u8 index=2;
	u8 buf[22]={0x00,0x00};
	for(i=0;i<3;i++)
	{
		ratios[i]=(RS485_RX_BUFF[index+i*4+3]<<8)|RS485_RX_BUFF[index+i*4+4];
		temp=(RS485_RX_BUFF[index+i*4+1]<<8)|RS485_RX_BUFF[index+i*4+2];
		ratios[i]=(ratios[i]<<16)|temp;
	}
	

	
	SP_a=0;
	SP_b=(double)ratios[0];
	SP_c=(double)ratios[1];
	SP_d=(double)ratios[2];
	
	SP_b/=10000;
	SP_c/=10000;
	SP_d/=10000;
	
	Write_User_Ratios_To_Rom(buf);
	
}


//屏幕交互指令
//写入。。。。
//void  Send_All_Info_To_Screen(void)
//{
//	u16 crc;
//	double originValue=0; //原始浓度值
//	u32 result=0;
//	u16 temp_1,temp_2;
//	u16 humidity_1,humidity_2;
//	
//	totalTime+=1.5;	//时间加一
//	nextMeansureTime-=1;
//	clearPeriod--;
//	
//	
//	if(clearPeriod <=0 )
//	{	
//		clearPeriod=setClearPeriod;
//		Motor_move(2);
//	}
//	if(nextMeansureTime<=0)
//	{	
//		calculateConcentration();	//获取原始浓度值
//		originValue=curValue;
//		//curValue=getCalibratedData(curValue);	//获取校准后的浓度值
//		nextMeansureTime=setNextMeansureTime;
//	}
//	
//	//温度1 湿度1
//	temp_1=(int)(SHT2x_GetTempPoll1()*10);
//	humidity_1=(int)(SHT2x_GetHumiPoll1()*10);
//	//温度2  湿度2
//	temp_2=(int)(SHT2x_GetTempPoll2()*10);
//	humidity_2=(int)(SHT2x_GetHumiPoll2()*10);
//	
//	result=(u32)(originValue*100);	//原始浓度值
//	u32 curRes=(u32)(curValue*100);	//当前测量值
//	u32 total=(u32)(totalTime*100);	//总测量时间
//	total/=3600;			//转换为小时
//	
//	
//	RS485_TX_BUFF[0]=SlaverAddr;
//	RS485_TX_BUFF[1]=0x10;
//	RS485_TX_BUFF[2]=0x00;
//	RS485_TX_BUFF[3]=0x00;
//	RS485_TX_BUFF[4]=0x00;
//	RS485_TX_BUFF[5]=0x0b;
//	RS485_TX_BUFF[6]=0x16;
//	
//	//当前测量值
//	RS485_TX_BUFF[7]=(curRes>>8)&0xff;
//	RS485_TX_BUFF[8]=curRes&0xff;
//	RS485_TX_BUFF[9]=curRes>>24;
//	RS485_TX_BUFF[10]=(curRes>>16)&0xff;
//	
//	//累计工作时长 单位时
//	RS485_TX_BUFF[11]=(total>>8)&0xff;
//	RS485_TX_BUFF[12]=total&0xff;
//	RS485_TX_BUFF[13]=total>>24;
//	RS485_TX_BUFF[14]=(total>>16)&0xff;
//	
//	RS485_TX_BUFF[15]=nextMeansureTime>>8;
//	RS485_TX_BUFF[16]=nextMeansureTime&0xff;
//	
//	//温度1，湿度1
//	RS485_TX_BUFF[17]=temp_1>>8;
//	RS485_TX_BUFF[18]=temp_1&0xff;
//	
//	RS485_TX_BUFF[19]=humidity_1>>8;
//	RS485_TX_BUFF[20]=humidity_1&0xff;
//	//温度2，湿度2
//	RS485_TX_BUFF[21]=temp_2>>8;
//	RS485_TX_BUFF[22]=temp_2&0xff;
//	
//	RS485_TX_BUFF[23]=humidity_2>>8;
//	RS485_TX_BUFF[24]=humidity_2&0xff;
//	
//	
//	//原始浓度值

//	RS485_TX_BUFF[25]=(result>>8)&0xff;
//	RS485_TX_BUFF[26]=result&0xff;
//	RS485_TX_BUFF[27]=result>>24;
//	RS485_TX_BUFF[28]=(result>>16)&0xff;
//	
//	crc=CRC_Compute(RS485_TX_BUFF,29);
//	RS485_TX_BUFF[29]=crc&0xff;
//	RS485_TX_BUFF[30]=crc>>8;
//	
//	//设置各种报警状态
//	setAlarmState(temp_1,temp_2,humidity_1,humidity_2);
//	
//	isNeedProcess=0;
//	RS485_SendData(RS485_TX_BUFF,31);
//	
//}



//屏幕交互指令
//写入 首页的全部数值
void  Send_All_Info_To_Screen(void)
{
	u16 crc;

	u32 result=0;
	u16 temp_1,temp_2;
	u16 humidity_1,humidity_2;
	
	totalTime+=1.5;	//时间加一v
	nextMeansureTime-=1;
	clearPeriod--;
	
	
	if(clearPeriod <=0 )
	{	
		clearPeriod=setClearPeriod;
		Motor_move(2);
	}
	if(nextMeansureTime<=0)
	{	
		curValue=calculateConcentration();	//获取原始浓度值
		originValue=curValue;
		curValue=getCalibratedData(curValue);	//获取校准后的浓度值
		//curValue*=1.12;
		nextMeansureTime=setNextMeansureTime;
	}
	
	//温度1 湿度1
	temp_1=(int)(SHT2x_GetTempPoll1()*10);
	humidity_1=(int)(SHT2x_GetHumiPoll1()*10);
	//温度2  湿度2
	temp_2=(int)(SHT2x_GetTempPoll2()*10);
	humidity_2=(int)(SHT2x_GetHumiPoll2()*10);
	
	result=(u32)(originValue*1e4);	//原始浓度值 //保留四位小数
					
	u32 curRes=(u32)(curValue*100);	//当前测量值
	u32 total=(u32)(totalTime*100);	//总测量时间
	total/=3600;			//转换为小时
	

	RS485_TX_BUFF[0]=SlaverAddr;
	RS485_TX_BUFF[1]=0x10;
	RS485_TX_BUFF[2]=0x00;
	RS485_TX_BUFF[3]=0x00;
	RS485_TX_BUFF[4]=0x00;
	RS485_TX_BUFF[5]=0x0b;
	RS485_TX_BUFF[6]=0x16;
	
	//当前测量值
	RS485_TX_BUFF[7]=(curRes>>8)&0xff;
	RS485_TX_BUFF[8]=curRes&0xff;
	RS485_TX_BUFF[9]=curRes>>24;
	RS485_TX_BUFF[10]=(curRes>>16)&0xff;
	
	//累计工作时长 单位时
	RS485_TX_BUFF[11]=(total>>8)&0xff;
	RS485_TX_BUFF[12]=total&0xff;
	RS485_TX_BUFF[13]=total>>24;
	RS485_TX_BUFF[14]=(total>>16)&0xff;
	
	RS485_TX_BUFF[15]=nextMeansureTime>>8;
	RS485_TX_BUFF[16]=nextMeansureTime&0xff;
	
	//温度1，湿度1
	RS485_TX_BUFF[17]=temp_1>>8;
	RS485_TX_BUFF[18]=temp_1&0xff;
	
	RS485_TX_BUFF[19]=humidity_1>>8;
	RS485_TX_BUFF[20]=humidity_1&0xff;
	//温度2，湿度2
	RS485_TX_BUFF[21]=temp_2>>8;
	RS485_TX_BUFF[22]=temp_2&0xff;
	
	RS485_TX_BUFF[23]=humidity_2>>8;
	RS485_TX_BUFF[24]=humidity_2&0xff;
	
	
	//原始浓度值

	RS485_TX_BUFF[25]=(result>>8)&0xff;
	RS485_TX_BUFF[26]=result&0xff;
	RS485_TX_BUFF[27]=result>>24;
	RS485_TX_BUFF[28]=(result>>16)&0xff;
	
	crc=CRC_Compute(RS485_TX_BUFF,29);
	RS485_TX_BUFF[29]=crc&0xff;
	RS485_TX_BUFF[30]=crc>>8;
	
	
	//设置各种报警状态
	setAlarmState(temp_1,temp_2,humidity_1,humidity_2);
	
	
	RS485_SendData(RS485_TX_BUFF,31);
	
}

//获取校准后的浓度值
double getCalibratedData(double x)
{
	
	
	double result=0;
	//y=a+bx^2+cx
	
	result=SP_b+(SP_c*x*x)+SP_d*x;
	//SP_b*(x*x)+SP_c*x+SP_d;
	return result;
}

//单种物质
//计算原始浓度值
double calculateConcentration()
{	
	if(!isGetWaveLength)
	{
		isWaveFlag=1;
		spReadyNext=0;
		SP_Get_WaveLength();
		while(!spReadyNext)
			;
	}
	sampleFlag=1;
	Get_Sample_Spectrum();
	while(sampleFlag)
		;
	refFlag=1;
	Get_Reference_Spectrum();
	while(refFlag)
		;
	double pw_220,pw_275;
	double A220=0.0,A275=0.0;
	if(Reference_220_Data<=0||Reference_275_Data<=0)
	{
		ratio_220=1;
		ratio_275=1;
	}
	else
	{
		ratio_220=Cur_Reference_220_Data/Reference_220_Data;
		ratio_275=Cur_Reference_275_Data/Reference_275_Data;
	}
	pw_220=ratio_220*White_220_Data;
	pw_275=ratio_275*White_275_Data;
	
	if(pw_220>Dark_220_Data && Sample_220_Data>Dark_220_Data)
	{
		A220=log10((pw_220-Dark_220_Data)/(Sample_220_Data-Dark_220_Data));
	}
	if(pw_275>Dark_275_Data && Sample_275_Data>Dark_275_Data)
	{
		A275=log10((pw_275-Dark_275_Data)/(Sample_275_Data-Dark_275_Data));
	}
	//220  60		w:9098		d:2962		r:29567
	//275  142		w:8284		d:2942		r:45793
	
	
	return (A220-2*A275);
}


//读取按钮的指令
//写入当前测量值，累计工作时长，下一次测量时间
//void Send_Time_To_Screen()
//{	
//	
//	u16 crc;
//	//测试
//	totalTime+=1.5;	//时间加一
//	nextMeansureTime-=1;
//	clearPeriod--;
//	if(clearPeriod <=0 )
//	{	
//		clearPeriod=setClearPeriod;
//		Motor_move(2);
//	}
//	if(nextMeansureTime<=0)
//	{	
//		curValue=calculateConcentration();	//获取原始浓度值
//		curValue=getCalibratedData(curValue);	//获取校准后的浓度值
//		nextMeansureTime=setNextMeansureTime;
//	}
//	
//	u32 curRes=(u32)(curValue*100);	//当前测量值
//	u32 total=(u32)(totalTime*100);	//总测量时间
//	total/=3600;			//转换为小时
//	

//	RS485_TX_BUFF[0]=SlaverAddr;
//	RS485_TX_BUFF[1]=0x10;
//	RS485_TX_BUFF[2]=0x00;
//	RS485_TX_BUFF[3]=0x00;
//	RS485_TX_BUFF[4]=0x00;
//	RS485_TX_BUFF[5]=0x05;
//	RS485_TX_BUFF[6]=0x0A;
//	
//	//当前测量值
//	RS485_TX_BUFF[7]=(curRes>>8)&0xff;
//	RS485_TX_BUFF[8]=curRes&0xff;
//	RS485_TX_BUFF[9]=curRes>>24;
//	RS485_TX_BUFF[10]=(curRes>>16)&0xff;
//	
//	//累计工作时长 单位时
//	RS485_TX_BUFF[11]=(total>>8)&0xff;
//	RS485_TX_BUFF[12]=total&0xff;
//	RS485_TX_BUFF[13]=total>>24;
//	RS485_TX_BUFF[14]=(total>>16)&0xff;
//	
//	RS485_TX_BUFF[15]=nextMeansureTime>>8;
//	RS485_TX_BUFF[16]=nextMeansureTime&0xff;
//	
//	crc=CRC_Compute(RS485_TX_BUFF,17);
//	
//	RS485_TX_BUFF[17]=crc&0xff;
//	RS485_TX_BUFF[18]=crc>>8;
//	RS485_SendData(RS485_TX_BUFF,19);
//	
//}
//.....................................
//发送温湿度
void Send_Temp_Humidity_To_Screen()
{	
	
	
	u16 crc;
	u16 temp_1,temp_2;
	u16 humidity_1,humidity_2;
	//温度1 湿度1
	temp_1=(int)(SHT2x_GetTempPoll1()*10);
	humidity_1=(int)(SHT2x_GetHumiPoll1()*10);
	//温度2  湿度2
	temp_2=(int)(SHT2x_GetTempPoll2()*10);
	humidity_2=(int)(SHT2x_GetHumiPoll2()*10);
	
	//设置各种报警状态
	//setAlarmState(temp_1,temp_2,humidity_1,humidity_2);
	
	
	RS485_TX_BUFF[0]=SlaverAddr;
	RS485_TX_BUFF[1]=0x10;
	RS485_TX_BUFF[2]=0x00;
	RS485_TX_BUFF[3]=0x06;
	RS485_TX_BUFF[4]=0x00;
	RS485_TX_BUFF[5]=0x04;
	RS485_TX_BUFF[6]=0x08;
	
	RS485_TX_BUFF[7]=temp_1>>8;
	RS485_TX_BUFF[8]=temp_1&0xff;
	
	RS485_TX_BUFF[9]=humidity_1>>8;
	RS485_TX_BUFF[10]=humidity_1&0xff;
	
	RS485_TX_BUFF[11]=temp_2>>8;
	RS485_TX_BUFF[12]=temp_2&0xff;
	
	RS485_TX_BUFF[13]=humidity_2>>8;
	RS485_TX_BUFF[14]=humidity_2&0xff;
	
	crc=CRC_Compute(RS485_TX_BUFF,15);
	RS485_TX_BUFF[15]=crc&0xff;
	RS485_TX_BUFF[16]=crc>>8;
	
	
	RS485_SendData(RS485_TX_BUFF,17);
	
}

void Calc_Query_Btn(void) //取消设置的确定按钮
{
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x10,0x00,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
}

//获取按钮全部状态
void  Check_Btn_State(void)
{	
	u16 crc;
	u8 btnStateOrder[8]={SlaverAddr,0x01,0x00,0x00,0x00,0x11};
	crc=CRC_Compute(btnStateOrder,6);
	btnStateOrder[6]=crc&0xff;
	btnStateOrder[7]=crc>>8;
	RS485_SendData(btnStateOrder,8);
	

}
//处理用户标定的系数
void  Process_Ratios_From_User(void)	
{	
	u8 i;
	u8 num=0;
	double xTemp[8];
	double yTemp[8];
	u8 buf[22]={0x00,0x00};
	for(i=0;i<8;i++)
	{
		if(conValueFlag[i])
		{
			xTemp[i]=xConcentration[i];
			yTemp[i]=yConcentration[i];
			conValueFlag[i]=0;
			num++;
		}
		
	}
	//计算系数
	if(num>0)
	{
		
		SP_a=0;
		Fit2(xTemp,yTemp,num,&SP_b,&SP_c,&SP_d);
		Write_User_Ratios_To_Rom(buf);
	}
}

//解析按钮状态
void Analysis_Btn_State()
{	
	
	u8 i=0;
	u16 crc;
	u8 buf[8]={SlaverAddr,0x03,0x00,0x2a,0x00,0x06};
	u8 buf2[8]={SlaverAddr,0x05,0x00,0x11,0x00,0x00};
	
	
	for(i=0;i<8;i++)	//浓度校准
	{
		if(GET_BIT(RS485_RX_BUFF[4],i))
		{
			Select_Concentration(i,i+8,40+i*2);
			break;
		}
	}
//	//屏幕一校准按钮按下
	if(GET_BIT(RS485_RX_BUFF[5],0))
	{	
		
		Process_Ratios_From_User();
		//delay_ms(50);
		Calc_Query_Btn();
		
		
	}
	//屏幕二校准按钮按下
	else if(GET_BIT(RS485_RX_BUFF[5],1))
	{	
		
			
		if(isCalcQueryBtn)
		{	
			isCalcQueryBtn=0;
			crc=CRC_Compute(buf2,6);
			buf2[6]=crc&0xff;
			buf2[7]=crc>>8;
			RS485_SendData(buf2,8);
			//delay_ms(50);
			
		}
		else
		{	
			
			isCalcQueryBtn=1;
			crc=CRC_Compute(buf,6);
			buf[6]=crc&0xff;
			buf[7]=crc>>8;
			RS485_SendData(buf,8);
			//delay_ms(50);
		}
	}
	
	if(GET_BIT(RS485_RX_BUFF[3],0))
	{
		//更换橡胶刷
		Switch_Rubber_Brush();
	}
	else if(GET_BIT(RS485_RX_BUFF[3],1))
	{
		//探头自诊断
		Detector_Diagnosis();
		//Motor_move(2);
		
	}
	
	else if(GET_BIT(RS485_RX_BUFF[3],6))	//设置确定按钮
	{	
		
		Get_All_Setting();
		
		
	}
	else if(GET_BIT(RS485_RX_BUFF[3],7))	//重置所有设置
	{
		Reset_All_Setting();
	}

}

//void  Send_Value_To_USER(void)	//发送原始浓度
//{	
//	double content=0;
//	u32 temp=0;
//	u16 crc;
//	u8 conBuf[13]={SlaverAddr,0x10,0x00,0x28,0x00,0x02,0x04};
//	content=calculateConcentration();
//	temp=(u32)(content*10000);
//	conBuf[7]=(temp>>8)&0xff;		//赋值
//	conBuf[8]=temp&0xff;
//	conBuf[9]=temp>>24;
//	conBuf[10]=(temp>>16)&0xff;
//	crc=CRC_Compute(conBuf,11);
//	conBuf[11]=crc&0xff;
//	conBuf[12]=crc>>8;
//	isNeedProcess=0;
//	RS485_SendData(conBuf,13);	//发送原始值
//	delay_ms(20);
//}


//发送命令读取用户系数

void  Send_Order_Read_Ratios(void)		
{
	u16 crc;
	u8 buf[8]={SlaverAddr,0x03,0x00,0x2a,0x00,0x06};
	u8 buf2[8]={SlaverAddr,0x05,0x00,0x11,0x00,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	
	crc=CRC_Compute(buf2,6);
	buf2[6]=crc&0xff;
	buf2[7]=crc>>8;
	isNeedProcess=0;
	
	
	RS485_SendData(buf,8);
	delay_ms(50);

	
//	if(isCalcQueryBtn)
//	{
//		RS485_SendData(buf2,8);
//		delay_ms(50);
//		isCalcQueryBtn=0;
//	}
//	else
//	{	
//		isCalcQueryBtn=1;
//		RS485_SendData(buf,8);
//		delay_ms(200);
//	}
	
	
}

//更换橡胶刷
void Switch_Rubber_Brush()
{	
	//复位橡胶刷状态
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x00,0x00,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
	Motor_move(1);	//1更换电刷 2清洁
}

//探头诊断
void Detector_Diagnosis(void)
{	
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x01,0x00,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
	//Motor_move(0);	//刷片复位
}
//探头校准
void Detctor_Calibration(void)
{	
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x02,0x00,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
}

//根据选择浓度选择
void Select_Concentration(u8 index,u16 switchAddr,u16 valueAddr)
{
	u16 crc;
	u32 temp;
	double content=0.0;
	u8 conBuf[13]={SlaverAddr,0x10,0x00,0x00,0x00,0x02,0x04};
	u8 buf[8]={SlaverAddr,0x05,0x00,0x00,0x00,0x00};	//置零
	
	
	buf[2]=switchAddr>>8;
	buf[3]=switchAddr&0xff;
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	
	content=calculateConcentration();	//计算溶液物质浓度
	xConcentration[index]=content;
	conValueFlag[index]=1;
	//保留四位小数
	temp=(u32)(content*1e4);			//发送浓度到屏幕

	//temp=25000;
	conBuf[2]=valueAddr>>8;				//写入屏幕的线圈地址
	conBuf[3]=valueAddr&0xff;
	
	conBuf[7]=(temp>>8)&0xff;		//赋值
	conBuf[8]=temp&0xff;
	conBuf[9]=temp>>24;
	conBuf[10]=(temp>>16)&0xff;
	
	crc=CRC_Compute(conBuf,11);		//计算CRC
	conBuf[11]=crc&0xff;
	conBuf[12]=crc>>8;
	
	RS485_SendData(conBuf,13);	//发送浓度值
	delay_ms(50);
	RS485_SendData(buf,8);
	delay_ms(50);
}



 void Reset_All_Setting(void)			//重置所有设置
{	
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x07,0x00,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
}




void Get_All_Setting(void)				//获取所有设置
{	
	u16 crc;
	u8 temp[8]={SlaverAddr,0x05,0x00,0x06,0x00,0x00};
	crc=CRC_Compute(temp,6);
	temp[6]=crc&0xff;
	temp[7]=crc>>8;
	RS485_SendData(temp,8);
	
	delay_ms(60);
	u8 buf[8]={SlaverAddr,0x03,0x00,0x0c,0x00,0x05};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
	//delay_ms(50);
	
}

//应用所有设置
void Set_All_Setting(void)
{	
	
	
	u16 crc;
	u8 buf[19]={SlaverAddr,0x10,0x00,0x0a,0x00,0x05,0x0a};

	setNextMeansureTime=MeansurePeriodArr[RS485_RX_BUFF[8]];
	setClearPeriod=ClearPeriodArr[RS485_RX_BUFF[10]];
	
	nextMeansureTime=setNextMeansureTime;
	clearPeriod=setClearPeriod;
	
	buf[7]=RS485_RX_BUFF[3];
	buf[8]=RS485_RX_BUFF[4];
	buf[9]=RS485_RX_BUFF[5];
	buf[10]=RS485_RX_BUFF[6];
	buf[11]=RS485_RX_BUFF[7];
	buf[12]=RS485_RX_BUFF[8];
	buf[13]=RS485_RX_BUFF[9];
	buf[14]=RS485_RX_BUFF[10];
	buf[15]=RS485_RX_BUFF[11];
	buf[16]=RS485_RX_BUFF[12];
	crc=CRC_Compute(buf,17);
	buf[17]=crc&0xff;
	buf[18]=crc>>8;
	//RS485_SendData(buf,19);

}

void Start_Darw()		//开始画图
{
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x0a,0xff,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
}
//停止画图
void Stop_Draw()
{
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x0a,0x00,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	isDraw=1;
	RS485_SendData(buf,8);
}


//--------------------------
//计算浓度
void CalculateValue(void)
{
	u8 i=0;
	u8 num=0;
	
	double pw_1,pw_2;
	double A1=0.0,A2=0.0;
	//光谱系数
	double ratio_1=0;
	double ratio_2=0;

	for(i=0;i<5;i++)
	{	
		A1=0;
		A2=0;
		if(Ref_Data[num]<=0||Ref_Data[num+1]<=0)
		{
			ratio_1=1;
			ratio_2=1;
		}
		else
		{
			ratio_1=CurRef_Data[num]/Ref_Data[num];
			ratio_2=CurRef_Data[num+1]/Ref_Data[num+1];
		}
		pw_1=ratio_1*White_Data[num];
		pw_2=ratio_2*White_Data[num+1];
		
		if(pw_1>Dark_Data[num]&&CurSample_Data[num]>Dark_Data[num])
		{
			A1=log10((pw_1-Dark_Data[num])/(CurSample_Data[num]-Dark_Data[num]));
		}
		if(pw_2>Dark_Data[num+1] &&CurSample_Data[num+1]>Dark_Data[num+1])
		{
			A2=log10((pw_2-Dark_Data[num+1])/(CurSample_Data[num+1]-Dark_Data[num+1]));
		}
		concentration[i]=A1-2*A2;
		num+=2;
	}
}


//计算原始浓度值
//void calculateConcentration()
//{	
//	if(!isGetWaveLength)
//	{
//		isWaveFlag=1;
//		spReadyNext=0;
//		SP_Get_WaveLength();
//		while(!spReadyNext)
//			;
//	}
//	sampleFlag=1;
//	Get_Sample_Spectrum();
//	while(sampleFlag)
//		;
//	refFlag=1;
//	Get_Reference_Spectrum();
//	while(refFlag)
//		;
//	
//	CalculateValue();	//计算浓度值
//	
//}
//计算标定好的浓度值
//void getCalibratedData()
//{
//	u8 i=0;
//	//y=ax^2+bx+c
//	for(i=0;i<5;i++)
//	{
//		calibrationValue[i]=concentration[i]*(spRatios[i*3]*spRatios[i*3])+spRatios[i*3+1]*concentration[i]+spRatios[i*3+2];
//	}
//	
//}

//下位机计算对应的系数（a b c d) 并储存在ROM中
void calculateCalibrationRatios(void)
{

}



//获取标定好的参数
void getRatios_Sample(u16 startAddr)
{	
	u8 buf[SP_RATIO_LEN];
	u32 ratios[4];
	u8 index=2;
	u16 temp;
	u8 i;
	AT24CXX_Read(startAddr,buf,SP_RATIO_LEN);
	
	for(i=0;i<4;i++)
	{
		ratios[i]=(buf[index+i*5+3]<<8)|buf[index+i*5+4];
		temp=(buf[index+i*5+1]<<8)|buf[index+i*5+2];
		ratios[i]=(ratios[i]<<16)|temp;
	}
	//SP_a=ratios[0]/10000.0;
	SP_a=0;
	SP_b=ratios[1]/10000.0;
	SP_c=ratios[2]/10000.0;
	SP_d=ratios[3]/10000.0;
	
	if(!buf[index])
		SP_a*=-1;
	if(!buf[index+5])
		SP_b*=-1;
	if(!buf[index+10])
		SP_c*=-1;
	if(!buf[index+15])
		SP_d*=-1;
}


//多种物质时使用
////获取标定好的参数
void getRatios(u8 type,u16 startAddr)
{	
	u8 buf[SP_RATIO_LEN];
	u32 ratios[4];
	u8 index=2;
	u16 temp;
	u8 i;
	AT24CXX_Read(startAddr,buf,SP_RATIO_LEN);
	
	for(i=0;i<4;i++)
	{
		ratios[i]=(buf[index+i*5+3]<<8)|buf[index+i*5+4];
		temp=(buf[index+i*5+1]<<8)|buf[index+i*5+2];
		ratios[i]=(ratios[i]<<16)|temp;
	}
	//SP_a=ratios[0]/10000.0;
	SP_b=ratios[1]/10000.0;
	SP_c=ratios[2]/10000.0;
	SP_d=ratios[3]/10000.0;
	
	//if(!buf[index])
		//SP_a*=-1;
	
	if(!buf[index+5])
		SP_b*=-1;
	if(!buf[index+10])
		SP_c*=-1;
	if(!buf[index+15])
		SP_d*=-1;
	
	index=type*3;
	spRatios[index]=SP_b;
	spRatios[index+1]=SP_c;
	spRatios[index+2]=SP_d;
}


//设置报警状态
void setAlarmState(int temp1,int temp2,int humidity1,int humidity2)
{
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x18,0xff,0x00};

	//湿度过大警报
	if(!alarmHumidityFlag&&(humidity1 >=700 || humidity2 >=700 ))
	{	
		crc=CRC_Compute(buf,6);
		buf[6]=crc&0xff;
		buf[7]=crc>>8;
		alarmHumidityFlag=1;
		RS485_SendData(buf,8);
		delay_ms(20);
		isReSetAlarm=0;
	}
	if(humidity1 < 700 && humidity2 < 700)
	{	
		if(!isReSetAlarm)
		{
			buf[4]=0x00;
			crc=CRC_Compute(buf,6);
			buf[6]=crc&0xff;
			buf[7]=crc>>8;
			alarmHumidityFlag=0;
			RS485_SendData(buf,8);
			delay_ms(20);
			isReSetAlarm=1;
			
		}
		
	}
	
}
 

//上电初始化各种参数
void Init_All_Argument()
{
	u8 buf[2];
	Init_Get_Wave();
	//读取清洁周期
	AT24CXX_Read(SP_CLEAR_PERIOD_ADDR,buf,SP_CLEAR_PERIOD_LEN);
	clearPeriod=(buf[0]>>8) | buf[1];
	setClearPeriod=clearPeriod;
	
	//获取标定好的参比光路
	Read_220_275_Spectrum(SP_REFENCE_SPECTRUM_ADDR,&Reference_220_Data,&Reference_275_Data);
	//白光谱
	Read_220_275_Spectrum(SP_WHITE_SPECTRUM_ADDR,&White_220_Data,&White_275_Data);
	//暗光谱
	Read_220_275_Spectrum(SP_DARK_SPECTRUM_ADDR,&Dark_220_Data,&Dark_275_Data);
	
	//获取单种物质的系数
	getRatios_Sample(SP_RATIO_ADDR);
	//读取系数
	//getAllInitRatios();	//
	
	
}


//恢复出厂设置
//恢复系数
//恢复清洁周期
void Restore_Factory_Setting()
{	
//	u8 buf[2];
//	//获取出厂设置的系数参数
//	getRatios(SP_RATIO_FACTORY_ADDR);
//	//获取清洁周期
//	AT24CXX_Read(SP_CLEAR_PERIOD_ADDR,buf,SP_CLEAR_PERIOD_LEN);
//	clearPeriod=(buf[0]>>8) | buf[1];
//	setClearPeriod=clearPeriod;
//	RS485_SendData(RS485_RX_BUFF,8);
}

void CalculateWaveLength(void)
{	
	float wave=102.78;
	u8 temp[4];
	u16 i;
	waveSize=0;
	//u16 count=4101;
	i=9;	//从
	if(size<SP_MAX_SIZE)
	{
		while(waveSize<1024)
		{	
			temp[3]=spArr[i];
			temp[2]=spArr[i+1];
			temp[1]=spArr[i+2];
			temp[0]=spArr[i+3];
			memcpy(&wave,temp,4);
			waveLengthData[waveSize]=wave;
			waveSize++;
			i+=4;
		}
	}
}


//初始化时获取波长
void Init_Get_Wave(void)
{
	spReadyNext=0;
	if(!isGetWaveLength)
	{
		isGetWaveLength=0;
		isWaveFlag=1;
		SP_Get_WaveLength();
		while(!spReadyNext)
			;
	}
	
}





//读取标定好的光谱
void Read_220_275_Spectrum(u16 addr,double*data_220,double*data_275)
{	
	u16 i=0;
	float x1=0,x2=0;
	u16 y1=0,y2=0;
	u8 buf[2];
	//获取标定好的光谱
	//220
	//275
	for(i=0;i<waveSize;i++)
	{
		if(waveLengthData[i]>220)
		{
			x2=waveLengthData[i];
			AT24CXX_Read(addr+i*2,buf,2);
			break;
		}
	
	}
	y2=(buf[0]<<8)|buf[1];
	if(i>0)
	{
		x1=waveLengthData[i-1];
		AT24CXX_Read(addr+(i-1)*2,buf,2);
		y1=(buf[0]<<8)|buf[1];	
	}

	*data_220=((220.0-x1)*(y2-y1))/(x2-x1)+y1;
	
	for(i=0;i<waveSize;i++)
	{
		if(waveLengthData[i]>275)
		{
			x2=waveLengthData[i];
			AT24CXX_Read(addr+i*2,buf,2);
			break;
		}
	
	}
	y2=(buf[0]<<8)|buf[1];
	if(i>0)
	{
		x1=waveLengthData[i-1];
		AT24CXX_Read(addr+(i-1)*2,buf,2);
		y1=(buf[0]<<8)|buf[1];	
	}

	*data_275=((275.0-x1)*(y2-y1))/(x2-x1)+y1;
}



void Clear_Device(void)	//清洁设备
{	
	u16 crc;
	u8 buf[8]={SlaverAddr,0x06,0x00,0x9e,0x00,0x01};
	Motor_move(2);	//清洁
	crc=CRC_Compute(buf,7);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
	
}

void Send_Temp_Humidity(void)	//温湿度给上位机
{	
	
	u8 buf[15]={SlaverAddr,0x03,0x0a};
	u16 crc;
	u16 temp_1,temp_2;
	u16 humidity_1,humidity_2;
	//温度1 湿度1
	temp_1=(int)(SHT2x_GetTempPoll1()*10);
	humidity_1=(int)(SHT2x_GetHumiPoll1()*10);
	//温度2  湿度2
	temp_2=(int)(SHT2x_GetTempPoll2()*10);
	humidity_2=(int)(SHT2x_GetHumiPoll2()*10);
	
	
	buf[3]=temp_1>>8;
	buf[4]=temp_1&0xff;
	
	buf[5]=humidity_1>>8;
	buf[6]=humidity_1&0xff;
	
	buf[7]=temp_2>>8;
	buf[8]=temp_2&0xff;
	
	buf[9]=humidity_2>>8;
	buf[10]=humidity_2&0xff;
	
	buf[11]=0x00;
	buf[12]=0x01;
	crc=CRC_Compute(buf,13);
	buf[13]=crc&0xff;
	buf[14]=crc>>8;

	RS485_SendData(buf,15);
}

void Send_Light_Mode(void)
{	
	u16 crc;
	u8 buf[6]={SlaverAddr,01,01};
	SP_Get_Light_Mode();
	delay_ms(20);
	buf[3]=spMode;
	crc=CRC_Compute(buf,4);
	buf[4]=crc&0xff;
	buf[5]=crc>>8;
	RS485_SendData(buf,6);
}

//写入系数
//TO DO .............
void Write_User_Ratios_To_Rom(u8*buf)
{	
	
	u32 temp;
	buf[2]=SP_a<0?SP_a*=-1,0:1;
	buf[7]=SP_b<0?SP_b*=-1,0:1;
	buf[12]=SP_c<0?SP_c*=-1,0:1;
	buf[17]=SP_d<0?SP_d*=-1,0:1;
	
	temp=(u32)(SP_a*10000);
	
	buf[3]=(temp>>8)&0xff;
	buf[4]=temp&0xff;
	buf[5]=temp>>24;
	buf[6]=(temp>>16)&0xff;
	
	temp=(u32)(SP_b*10000);
	
	buf[8]=(temp>>8)&0xff;
	buf[9]=temp&0xff;
	buf[10]=temp>>24;
	buf[11]=(temp>>16)&0xff;
	
	temp=(u32)(SP_c*10000);
	
	buf[13]=(temp>>8)&0xff;
	buf[14]=temp&0xff;
	buf[15]=temp>>24;
	buf[16]=(temp>>16)&0xff;
	
	temp=(u32)(SP_d*10000);
	
	buf[18]=(temp>>8)&0xff;
	buf[19]=temp&0xff;
	buf[20]=temp>>24;
	buf[21]=(temp>>16)&0xff;
	
	AT24CXX_Write(SP_RATIO_ADDR,buf,SP_RATIO_LEN);//写入屏幕标定的参数
	
}
//设置用来计算的物质波长
void Set_Calc_Wave(void)
{
	u8 type;
	u16 A1,A2,crc;
	u8 buf[8]={SlaverAddr,0x10,0x00,0x9f,0x00,0x06};
	
	type=(RS485_RX_BUFF[7]<<8)|RS485_RX_BUFF[8];
	if(type<0x0A)
	{
		A1=(RS485_RX_BUFF[9]<<8)| RS485_RX_BUFF[10];
		A2=(RS485_RX_BUFF[11]<<8)| RS485_RX_BUFF[12];
		Sample_Data_Wave[type*2]=A1;
		Sample_Data_Wave[type*2+1]=A2;
		CurRef_Data_Wave[type*2]=A1;
		CurRef_Data_Wave[type*2+1]=A2;
		Write_Material_Wave_To_Rom(type,A1,A2);
	}
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	
	RS485_SendData(buf,8);
	
	
}
//写入某种物质计算波长
void Write_Material_Wave_To_Rom(u8 type,u16 A1,u16 A2)	
{	
	u8 buf[SP_MATERIAL_LEN]={0x00};
	u16 addr=type*SP_MATERIAL_LEN+SP_MATERIAL_ADDR;
	buf[1]=type;
	buf[2]=A1>>8;
	buf[3]=A1&0xff;
	
	buf[4]=A2>>8;
	buf[5]=A2&0xff;
	
	AT24CXX_Write(addr,buf,SP_MATERIAL_LEN);
}
//读取某种物质的计算波长
void Read_Calc_Wave(void)
{
	u8 buf[11]={SlaverAddr,0x03,0x06};
	u16 addr,crc;
	u8 type;
	type=RS485_RX_BUFF[5];
	if(type<0x0A)
	{
		
		addr=SP_MATERIAL_LEN*type+SP_MATERIAL_ADDR;
		AT24CXX_Read(addr,&buf[3],SP_MATERIAL_LEN);
		buf[3]=type>>8;
		buf[4]=type&0xff;
	}
	
	crc=CRC_Compute(buf,9);
	buf[9]=crc&0xff;
	buf[10]=crc>>8;
	
	RS485_SendData(buf,11);
}

//获取所有物质的计算系数
//void getAllInitRatios(void)
//{
//	u8 i=0;
//	u16 addr;
//	for(i=0;i<5;i++)
//	{	
//		addr=SP_RATIO_ADDR+SP_RATIO_LEN*i;
//		getRatios(i,addr);
//	}
//}
