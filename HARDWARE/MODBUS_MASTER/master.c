//	�����������    //

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
u8 isNeedProcess=1;  //�Ƿ���Ҫ�����ص�����
u8 timeFlag=0;		//��ʱ���жϲ�����־
u8 sendCompleteFlag=1; //RS485������ɱ�־
u8 isCalcQueryBtn=0;
u8 spArr[5000]={1,2,3,4,5,6,6,7}; //���չ����ǻ�����
u16 size=0;



u32 RS485_Baudrate=9600;//ͨѶ������
u8 RS485_Parity=0;//0��У�飻1��У�飻2żУ��
u16 RS485_Frame_Distance=4;//����֡��С�����ms),������ʱ������Ϊ����һ֡

u8 RS485_RX_BUFF[512];//���ջ�����2048�ֽ�
u16 RS485_RX_CNT=0;//���ռ�����
u8 RS485_RxFlag=0;//����һ֡�������

u8 RS485_TX_BUFF[128];//���ͻ�����
u16 RS485_TX_CNT=0;//���ͼ�����
u8 RS485_TxFlag=0;//����һ֡�������


/////////////////////////////////////////////////////////////////////////////////////
//����������
u8   SlaverAddr=0x01;    //Ĭ�ϴӻ���ַ
u8   Fuction=0;      // ������	//0x10
u16  StartAddr=0;    //��ʼ��ַ
u16  ValueOrLenth=1;  //����or����
//////////////////////////////////////////////////////////////////////////////////////////

u8 TX_RX_SET=0; //���ͣ����������л��� 0 ����ģʽ 1����ģʽ
u8 ComErr=8; //0����ͨѶ����
             //1����CRC����
			// 2����������� 
//-------------------------------------------------			
//��Ļ��ر���
//��ѯ��ť״̬����

u16 MeansurePeriodArr[]={15,20,30,60,120,180,240,300,600,1200,1800};
u32 ClearPeriodArr[]={60,120,180,240,300,600,1200,1800,3600,7200,10800,21600,43200};

double totalTime=0;	//���е���ʱ�� ��
	
double curValue=0;	//��ǰ����ֵ  
double originValue=0; //ԭʼŨ��ֵ

u16 nextMeansureTime=60;  //������һ�β���ʱ��
u16 setNextMeansureTime=60;	//�û����õ���һ�β���ʱ��

u32 clearPeriod=60; //�������
u32	setClearPeriod=60;	//�û����õ��������

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Master�Ĵ����͵�Ƭ���Ĵ�����ӳ���ϵ

u8 spectrumData[3000]; //�����������

u16 spSize=4;			//�����������ݸ���
u8 isSendSpectrum=0; //�Ƿ��͹�������

float waveLengthData[1026]; //��������
u8 waveLengthBuf[4200];		//Ҫ���͵Ĳ�������	
u8 isGetWaveLength=0;	//�Ƿ��ȡ�˲���
u8 isWaveFlag=0;		//�Ƿ��ǲ�������
u8 isWaveSendFlag=0;		//�Ƿ����˲�������
u8 isGetSPIntegTime=0;		//�Ƿ��ȡ�����ǻ���ʱ��
u8 isGetPulseTime=0;		//�Ƿ��ȡ����ʱ��
u8 isGetLightMode=0;
u8 isGetAverageTime=0;
u16 waveSize=0;		//��ǰ�����������
u16	waveSendLen=0;	//���Ͳ�����������ĳ���

u8 spReadyNext=0;	//�������Ƿ���Խ�����һ�������

u8 SP_RX_COMPLETE=0;	//���������ݽ������

u8 spMode=0x01;			//������믵Ƶ�ģʽ 00 �� 01 ����  0x81����



//���궨
u8 caliBrationFlag=0;	//�Ƿ��ͼ��궨������
double xConcentration[8];
double yConcentration[8]={0,0.5,1.0,2.0,5.0,10.0,20.0,25.0};
u8     conValueFlag[8]={0};//ѡ���ĸ���ֵ����У׼
u8 alarmTempFlag;	//�¶Ⱦ���
u8 alarmHumidityFlag=1;	//ʪ�Ⱦ���
u8 isReSetAlarm=1;


//�Ƿ�Ϊ�ο�����
u8 refFlag=0;
//�Ƿ�Ϊ��Ʒ����
u8 sampleFlag=0;
//�Ƿ�ʼ��ͼ
u8 isDraw=0;
//Ũ�ȼ������

//ϵ��
double SP_a=10;
double SP_b=10;
double SP_c=100;
double SP_d=1000;

//10��ϵ��	ÿ������
//������� y=ax^2+bx+c
double spRatios[30];


//��������
//No3-N		220  275
//COD		254  576
//BOD		254	 576
//TOC		254	 576
//TURB		660	 0  
u16 Sample_Data_Wave[20]={220,275,254,576,254,576,254,576,660,0};
u16 CurRef_Data_Wave[20]={220,275,254,576,254,576,254,576,660,0};

double CurSample_Data[10];	
double CurRef_Data[10]; 

double concentration[5];	//�������ʵ�Ũ��
double calibrationValue[5];	//����У׼��Ũ��

//�궨�õĹ�������
double Ref_Data[10];	//�ο�
double White_Data[10];	//�׹���
double Dark_Data[10];	//������


//��ǰ��������
double Sample_220_Data=0;
double Sample_275_Data=0;
//��ǰ�ο�����
double Cur_Reference_220_Data=0;
double Cur_Reference_275_Data=0;


//�궨�õİ׹���
double White_220_Data=0;
double White_275_Data=0;

//�궨�õĲο�����
double Reference_220_Data=0;
double Reference_275_Data=0;

//�궨�õİ�����
double Dark_220_Data=0;
double Dark_275_Data=0;

//����ϵ��
double ratio_220=0;
double ratio_275=0;



//������ �������ָ�� �����ж�  TIM2
void TIM2_IRQHandler(void)
{
	 if(TIM_GetITStatus(GENERAL_TIM,TIM_IT_Update)!=RESET)
     {	
		
		TIM_ClearITPendingBit(GENERAL_TIM,TIM_IT_Update);//����жϱ�־
		TIM_Cmd(GENERAL_TIM,DISABLE);//ֹͣ��ʱ��
		SP_RX_COMPLETE=1;	//�������
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
                
                res=USART_ReceiveData(SP_USART); //�����յ����ֽڣ�ͬʱ��ر�־�Զ����
                
			
                if(size<SP_MAX_SIZE)
                {		
                        spArr[size]=res;
                        size++;
                        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//�����ʱ������ж�
						TIM_SetCounter(TIM2,0);//�����յ�һ���µ��ֽڣ�����ʱ��3��λΪ0�����¼�ʱ
                        TIM_Cmd(TIM2,ENABLE);//��ʼ��ʱ
                }
        }
}



//������յ��Ĺ�������
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
				for(i=9;i<size-6;i++)  //ֻ��ȡ��Ч����
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
			//���ͻ���ʱ��
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
		SP_RX_COMPLETE=0; //������ɿ������½���SP��������
		
	}
	
}


//��λ������
//д��λ��
void Write_Ratio_To_Rom(void)//д��ϵ��
{	
	//дϵ����ȥ
	u8 buf[8]={SlaverAddr,0x10,0x00,0x8b,0x00,0x0b};
	u16 crc;
	u16 addr;
	u16 type=(RS485_RX_BUFF[8]<<8)|RS485_RX_BUFF[9];
	if(type<0x0A)
	{	
		addr=SP_RATIO_ADDR+SP_RATIO_LEN*type;
		AT24CXX_Write(addr,&RS485_RX_BUFF[8],SP_RATIO_LEN);	//д��ROM
		delay_ms(10);
		//addr=SP_RATIO_FACTORY_ADDR+SP_RATIO_LEN*type;	//��ŵ�ROM�� ���ڻָ���������
		//wAT24CXX_Write(addr,&RS485_RX_BUFF[8],SP_RATIO_LEN);
		crc=CRC_Compute(buf,6);
		buf[6]=crc&0xff;
		buf[7]=crc>>8;
		delay_ms(5);
		getRatios(type,addr);	//��ȡϵ��	//��ֵ����Ӧϵ��
		RS485_SendData(buf,8);
	}
	
	
}

//��ȡROM��Ӧ��ϵ������λ��
void Send_Ratio_To_USART(void)	//����ϵ������λ��
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

//�ָ���������
//......................
void Reset_Factory_Setting(void)	
{
	
}
//���ôӻ���ַ
void Set_Slave_Addr(void)
{
	
	SlaverAddr=RS485_RX_BUFF[5];
	AT24CXX_WriteOneByte(SP_SLAVE_ADDR,SlaverAddr); //�����վ��
	RS485_RX_BUFF[0]=SlaverAddr;
	RS485_SendData(RS485_RX_BUFF,8); //����ֵ
}

//���ʹӻ���ַ

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

//�����������
//������EEPROM      0x0001~0x0002
//���ӱ�ʾ
void Set_Clear_Period(void)
{
	
	
	//�����������
	//��д��EEPROM
	clearPeriod=RS485_RX_BUFF[4];
	clearPeriod<<=8;
	clearPeriod|=RS485_RX_BUFF[5];
	
	AT24CXX_WriteOneByte(SP_CLEAR_PERIOD_ADDR,clearPeriod>>8);
	AT24CXX_WriteOneByte(SP_CLEAR_PERIOD_ADDR+1,clearPeriod&0xff);
	RS485_SendData(RS485_RX_BUFF,8);
	
}

//����ģʽ
void Set_SP_Pulse_Mode()
{
	spMode=RS485_RX_BUFF[5];
	SP_Open_Light(spMode);
	RS485_SendData(RS485_RX_BUFF,8);
}
//��������ߵ�ʱ��
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

void Get_SP_Pulse_Time(void)	//��ȡ����ߵ�ʱ��
{	
	isGetPulseTime=1;
	spSize=3;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x03;
	spectrumData[2]=0x08;
	SP_Get_Pulse_Time();
}
//����ʱ��
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
	//������.....????
	
	
}

//��ȡ����ʱ��
void Get_SP_IntegTime(void)
{	
	isGetSPIntegTime=1;
	spSize=3;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x03;
	spectrumData[2]=0x04;
	SP_Get_IntegTime();
	
}

//���ù�����ƽ������
void Set_SP_Average_Time()
{
	SP_Set_Average((RS485_RX_BUFF[4]<<8)|RS485_RX_BUFF[5]);
	RS485_SendData(RS485_RX_BUFF,8);
}
void Get_SP_Average_Time(void) //��ȡƽ������
{
	isGetAverageTime=1;
	spSize=3;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x03;
	spectrumData[2]=0x0f;
	SP_Get_Average();
}

//��ȡ��Ʒ����
//�����͸���λ��
void Get_Sample_Spectrum(void)
{	
	spReadyNext=0;
	spSize=4;
	
	isSendSpectrum=0;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x05;
	spectrumData[2]=0x00;
	spectrumData[3]=0x87;
	//����
	Set_shutter(0);
	SP_Open_Light(spMode);
	delay_ms(200);
	while(!spReadyNext)	//�ȴ���һ������
		;
	isSendSpectrum=1;
	SP_Get_SPData();
	
}

//��ȡ������

void Get_Dark_Spectrum(void)
{	
	
	//�ص�....0x00�ص�
	spReadyNext=0;
	spSize=4;
	Set_shutter(0);	 //��Ƭ
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x05;
	spectrumData[2]=0x00;
	spectrumData[3]=0x88;
	isSendSpectrum=0;
	SP_Open_Light(0x00);
	delay_ms(10);
	//�ȴ����Բɼ���һ��
	while(!spReadyNext)
		;
	isSendSpectrum=1;
	SP_Get_SPData();
	
}


//��ȡ�αȹ���
void Get_Reference_Spectrum(void)
{	
	
	//Get_Dark_Spectrum();
	//�򿪵�Ƭ�ɼ��αȹ���
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
	
	SP_Open_Light(spMode);//����
	delay_ms(200);
	while(!spReadyNext)	//�ȴ���һ������� ����ִ����һ������
		;
	
	isSendSpectrum=1;
	SP_Get_SPData();
}

//��ȡ�׹���
void Get_White_Spectrum(void)
{	
	//��λ��Ƭ�ɼ��׹���

	Set_shutter(0);
	delay_ms(20);
	
	spReadyNext=0;
	isSendSpectrum=0;
	spSize=4;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x05;
	spectrumData[2]=0x00;
	spectrumData[3]=0x9a;
	SP_Open_Light(spMode);	//����
	delay_ms(200);
	while(!spReadyNext)		//���ó�ʱʱ��....
		;
	isSendSpectrum=1;
	SP_Get_SPData();
}

//ROM ��0x1000��ʼ
//д�밵����
void Write_DarkSp_To_Rom()
{	
	

	AT24CXX_Write(SP_DARK_SPECTRUM_ADDR,&spectrumData[4],SP_DARK_SPECTRUM_LEN);
	//AT24CXX_Page_Write(SP_DARK_SPECTRUM_ADDR,&spectrumData[4],SP_DARK_SPECTRUM_LEN);
	RS485_SendData(RS485_RX_BUFF,8);
	//RS485_SendByte('Q');
}

//д��αȹ���
//��ַ: SP_REFENCE_SPECTRUM
void Write_ReferenceSp_To_Rom()
{
	
	AT24CXX_Write(SP_REFENCE_SPECTRUM_ADDR,&spectrumData[4],SP_REFENCE_SPECTRUM_LEN);
	RS485_SendData(RS485_RX_BUFF,8);
}	

//�׹���
//�����Ǻ������ʸ����������������չ
//��ʼ��ַ:	SP_WHITE_SPECTRUM_ADDR
void Write_WhiteSp_To_Rom()
{	
	
	AT24CXX_Write(SP_WHITE_SPECTRUM_ADDR,&spectrumData[4],SP_WHITE_SPECTRUM_LEN);
	RS485_SendData(RS485_RX_BUFF,8);
}	

//���ͱ궨�õİ����׵���λ��
void Send_DarkSp_To_USART(void)
{	
	u16 crc;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x06;
	spectrumData[2]=0x00;
	spectrumData[3]=0x97;
	
	AT24CXX_Read(SP_DARK_SPECTRUM_ADDR,&spectrumData[4],SP_DARK_SPECTRUM_LEN);
	
	crc=CRC_Compute(spectrumData,SP_DARK_SPECTRUM_LEN+4);
	spectrumData[SP_DARK_SPECTRUM_LEN+4]=crc&0xff;	//��8λ
	spectrumData[SP_DARK_SPECTRUM_LEN+5]=crc>>8;	//��8λ
	RS485_SendData(spectrumData,SP_DARK_SPECTRUM_LEN+6);
}
//���ͱ궨�õĲο�����
void Send_ReferenceSp_To_USART(void)
{
	
	u16 crc;
	spectrumData[0]=SlaverAddr;
	spectrumData[1]=0x06;
	spectrumData[2]=0x00;
	spectrumData[3]=0x98;
	
	AT24CXX_Read(SP_REFENCE_SPECTRUM_ADDR,&spectrumData[4],SP_REFENCE_SPECTRUM_LEN);
	
	crc=CRC_Compute(spectrumData,SP_REFENCE_SPECTRUM_LEN+4);
	spectrumData[SP_REFENCE_SPECTRUM_LEN+4]=crc&0xff;	//��8λ
	spectrumData[SP_REFENCE_SPECTRUM_LEN+5]=crc>>8;	//��8λ
	RS485_SendData(spectrumData,SP_REFENCE_SPECTRUM_LEN+6);
}

//���Ͱ׹���
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

//���Ͳ�������
void Send_WaveCount_To_USART(void)
{	
	u16 crc;
	u16 waveCount=0;	//��������
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
//��ȡ��������
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


//��ʼ��USART1
void RS485_Init(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        NVIC_InitTypeDef NVIC_InitStructure;
	
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE); //����1
        
        GPIO_InitStructure.GPIO_Pin=SCREEN_USART_PIN_TX;//PA9��TX�������������
        GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
        GPIO_Init(SCREEN_USART_PORT,&GPIO_InitStructure);
      
        
        GPIO_InitStructure.GPIO_Pin=SCREEN_USART_PIN_RX;//PA10��RX����������
        GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;   //�޸�ԭGPIO_Mode_IPU������������->GPIO_Mode_IN_FLOATING(��������)/////////////////////////////////////////////
        GPIO_Init(SCREEN_USART_PORT,&GPIO_InitStructure);
        
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;//�޸�PG9��RE/DE��ͨ���������->PD7��RE/DE��ͨ���������//////////////////////////////////////////////////////////////////////
        GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
        GPIO_Init(GPIOA,&GPIO_InitStructure);
        GPIO_ResetBits(GPIOA,GPIO_Pin_8);//Ĭ�Ͻ���״̬
        
        USART_DeInit(SCREEN_USART);//��λ����1
        USART_InitStructure.USART_BaudRate=RS485_Baudrate;
        USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
        USART_InitStructure.USART_WordLength=USART_WordLength_8b;
        USART_InitStructure.USART_StopBits=USART_StopBits_1;
        USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//�շ�ģʽ
        switch(RS485_Parity)
        {
                case 0:USART_InitStructure.USART_Parity=USART_Parity_No;break;//��У��
                case 1:USART_InitStructure.USART_Parity=USART_Parity_Odd;break;//��У��
                case 2:USART_InitStructure.USART_Parity=USART_Parity_Even;break;//żУ��
        }
        USART_Init(SCREEN_USART,&USART_InitStructure);
        
        NVIC_InitStructure.NVIC_IRQChannel=SCREEN_USART_IRQN;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
        NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
        NVIC_Init(&NVIC_InitStructure);
		
		USART_ClearITPendingBit(SCREEN_USART,USART_IT_RXNE);
        USART_ITConfig(SCREEN_USART,USART_IT_RXNE,ENABLE);//ʹ�ܴ���1������
		
		USART_Cmd(SCREEN_USART,ENABLE);//ʹ�ܴ���1
		
        Timer3_Init();//��ʱ��7��ʼ�������ڼ��ӿ���ʱ��
        
}





void USART1_IRQHandler(void)//����1�жϷ������
{		
	
		u8 res;
       if(USART_GetITStatus(SCREEN_USART,USART_IT_RXNE)!=RESET)
		//if (USART_GetFlagStatus(SCREEN_USART, USART_FLAG_ORE) != RESET )
        {		
				
                res=USART_ReceiveData(SCREEN_USART); //�����յ����ֽڣ�ͬʱ��ر�־�Զ����
				RS485_RX_BUFF[RS485_RX_CNT]=res;
				RS485_RX_CNT++;
				TIM_ClearITPendingBit(TIM3,TIM_IT_Update);//�����ʱ������ж�
				TIM_SetCounter(TIM3,0);//�����յ�һ���µ��ֽڣ�����ʱ��7��λΪ0�����¼�ʱ���൱��ι����
				TIM_Cmd(TIM3,ENABLE);//��ʼ��ʱ
              
        }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//��ʱ��4��ʼ��
//
void Timer4_Init(u16 arr)	   	//TIM4ʹ��
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʱ��ʹ��	
	TIM_TimeBaseStructure.TIM_Period = arr-1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler = 7199; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4,TIM_IT_Update, ENABLE );//TIM3 ��������ж�
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIMx����	
	
}

void Timer4_disable (void)					   //TIM4ʧ��
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, DISABLE); //ʱ��ʧ��
	TIM_ITConfig(TIM4, TIM_IT_Update | TIM_IT_Trigger,DISABLE );
	TIM_Cmd(TIM4, DISABLE);  //ʧ��TIMx����
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
void TIM4_IRQHandler(void)   //TIM4�ж�
{	
	
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{	
		timeFlag=1;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	
		//���TIMx���жϴ�����λ:TIM �ж�Դ 
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
//��ʱ��3��ʼ��---���ܣ��жϴӻ����ص������Ƿ�������

void Timer3_Init(void)
{
        TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
        NVIC_InitTypeDef NVIC_InitStructure;

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //TIM3ʱ��ʹ�� 

        //TIM3��ʼ������
        TIM_TimeBaseStructure.TIM_Period = RS485_Frame_Distance*10; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
        TIM_TimeBaseStructure.TIM_Prescaler =7199; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ ���ü���Ƶ��Ϊ10kHz
        TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
		
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
        TIM_ITConfig( TIM3, TIM_IT_Update, ENABLE );//TIM3 ��������ж�

        //TIM3�жϷ�������
        NVIC_InitStructure.NVIC_IRQChannel =TIM3_IRQn;  //TIM3�ж�
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
        NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���   
		TIM_Cmd(TIM3,DISABLE);
}



/////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////

//�ö�ʱ��3�жϽ��տ���ʱ�䣬������ʱ�����ָ��ʱ�䣬��Ϊһ֡����
//��ʱ��3�жϷ������         
void TIM3_IRQHandler(void)
{                                                                   
        if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)
        {		
				
                TIM_ClearITPendingBit(TIM3,TIM_IT_Update);//����жϱ�־
                TIM_Cmd(TIM3,DISABLE);//ֹͣ��ʱ��
                RS485_TX_EN=1;//ֹͣ���գ��л�Ϊ����״̬
                RS485_RxFlag=1;//��λ֡�������
        }
}

//////////////////////////////////////////////////////////////////////////////
//����n���ֽ�����
//buff:�������׵�ַ
//len�����͵��ֽ���
void RS485_SendData(u8 *buf,u16 len)
{ 		
		
		
        RS485_TX_EN=1;//�л�Ϊ����ģʽ
        while(len--)
        {	
                while(USART_GetFlagStatus(SCREEN_USART,USART_FLAG_TXE)==RESET);//�ȴ�������Ϊ��
            
				USART_SendData(SCREEN_USART,*(buf++));
				
        }
        while(USART_GetFlagStatus(SCREEN_USART,USART_FLAG_TC)==RESET);//�ȴ��������
		TX_RX_SET=1; //�����������
		RS485_TX_EN=0;	//�л�����״̬
}


//���͵����ֽ�

void RS485_SendByte(u8 data)
{
	RS485_TX_EN=1;//�л�Ϊ����ģʽ
	while(USART_GetFlagStatus(SCREEN_USART,USART_FLAG_TXE)==RESET)
		;
	USART_SendData(SCREEN_USART,data);
	while(USART_GetFlagStatus(SCREEN_USART,USART_FLAG_TC)==RESET);//�ȴ��������
	RS485_TX_EN=0;	//�л�����״̬
}


/////////////////////////////////////////////////////////////////////////////////////
//RS485�������

void RS485_RX_Service(void)
{
		
        if(RS485_RxFlag==1)
        {		
                if(RS485_RX_BUFF[0]==SlaverAddr)//��ַ��ȷ
                {			
                        if((RS485_RX_BUFF[1]==0x01)||(RS485_RX_BUFF[1]==0x02)||(RS485_RX_BUFF[1]==0x03)||(RS485_RX_BUFF[1]==0x05)||(RS485_RX_BUFF[1]==0x06)||(RS485_RX_BUFF[1]==0x04)||(RS485_RX_BUFF[1]==16))//��������ȷ
						{			
									
                                      // calCRC=CRC_Compute(RS485_RX_BUFF,RS485_RX_CNT-2);//�������������ݵ�CRC
                                      // recCRC=RS485_RX_BUFF[RS485_RX_CNT-1]|(((u16)RS485_RX_BUFF[RS485_RX_CNT-2])<<8);//���յ���CRC(���ֽ���ǰ�����ֽ��ں�)
                                        /*-----------------------------*/
										if(1)//CRCУ����ȷ	//������У��
                                        {		
											
											
											    
                                                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                switch(RS485_RX_BUFF[1])//���ݲ�ͬ�Ĺ�������д���
                                                {		
													
														
                                                        case 0x02://�����뿪����
                                                        {
                                                                Modbus_02_Solve();
                                                                break;
                                                        }
                                                        
                                                        case 0x01://�����������
                                                        {		
																
															 
													
                                                                Modbus_01_Solve();
                                                                break;
                                                        }
                                                                
                                                        case 0x05://д�������������
                                                        {
                                                                Modbus_05_Solve();
                                                                break;
                                                        }
                                                                
                                                        case 15://д������������
                                                        {
                                                                //Modbus_15_Solve();
                                                                break;
                                                        }
                                                
                                                        case 0x03: //������Ĵ���
                                                        {																
																Modbus_03_Solve();
																
                                                                break;
                                                        }
                                                                
                                                        case 0x06: //д�����Ĵ���
                                                        {
                                                                Modbus_06_Solve();
																
                                                                break;
                                                        }
                                                                
                                                        case 0x10: //д����Ĵ���
                                                        {
                                                                Modbus_16_Solve();
                                                                break;
                                                        }
														default:
															
														break;
                                                                                        
                                                }
										}
                                        else//CRCУ�����
                                        {
												  ComErr=14;

                                        }
											
						}
                        else//���������
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
								TX_RX_SET=0; //�������	
							}
							
							
                        }
          }
		  //���ʹӻ���ַ
		  if(RS485_RX_BUFF[0] == 0xff && RS485_RX_BUFF[1]==0x66&&RS485_RX_BUFF[2]==0xaa && RS485_RX_BUFF[3]==0xbb)
		  {		
			  TIM_Cmd(TIM4, DISABLE);
			  timeFlag=0;
			  Send_Slave_Addr();
			  
		  }
                //ComErr=0;                
			RS485_RxFlag=0;//��λ֡������־
			RS485_RX_CNT=0;//���ռ���������
			TX_RX_SET=0; //�������
			RS485_TX_EN=0;//�л�����״̬
			
        }
}



//Modbus������01������� ///////////////////////////////////////////////////
//�����������
void Modbus_01_Solve(void)   
{	
	
	
	switch(RS485_RX_BUFF[2])
	{	
		
		case 0x03:		//ȫ����ť״̬
			Analysis_Btn_State();	//������ť״̬
			break;	
		case 0x14:		//����״̬
			break;
		case 0x00:
			Send_Light_Mode();
			break;
	}
	//TX_RX_SET=0; //�������
	
		
}

//Modbus������02������� //////////////////////////////////////////////////////
//�����뿪����
void Modbus_02_Solve(void)   
{
		
}



//Modbus������03�������//////////////////////////////////////////////////////////////////////////////////
//�����ּĴ���
void Modbus_03_Solve(void)
{		
		
	
		switch(RS485_RX_BUFF[2])
		{
			case 0x0a:			//��Ļ����ȫ����������
				Set_All_Setting();
				break;
			case 0x0c:
				Read_User_Set_Ratios();	//��ȡ�û�����Ļ���õĲ���
				break;
		}
		
		switch(RS485_RX_BUFF[3])
		{		
			case 0x9b:
				Send_WaveCount_To_USART();	//��ȡ��������
				break;
			case 0x9c:
				Get_SP_IntegTime(); //��ȡ�����ǻ���ʱ��
				break;
			case 0x9d:
				Get_SP_Pulse_Time();	//��ȡ����������ʱ��
				break;
			case 0x9e:
				Get_SP_Average_Time();	//��ȡƽ������
				break;
			case 0x10:
				Send_Temp_Humidity();	//������ʪ�ȸ���λ��
				break;
			case 0x0e:
				Read_Calc_Wave();		//��ȡҪ��������Ĳ���ֵ
				break;
		}
		
}

//Modbus������05�������   ///////////////////////////////////////////////////////
//д�������������
void Modbus_05_Solve(void)
{	
	
	
	switch(RS485_RX_BUFF[3])
	{	
		case 0x86:
			//�ָ���������
			Restore_Factory_Setting();
			break;
		case 0x87:
			Get_Sample_Spectrum();	//�ɼ���Ʒ���� ������λ��
			break; 
		case 0x88:
			Get_Dark_Spectrum();	//�ɼ������� ������λ��
			break;
		case 0x89:
			Get_Reference_Spectrum();	//�ɼ��ο�����
			break;
		case 0x9a:
			Get_White_Spectrum();	//�׹���
			break;	
	}
	
}

//Modbus������06�������   //////////////////////////////////////////////////////////////////////////////////����֤����OK
//д�������ּĴ���
void Modbus_06_Solve(void)
{		
		
		switch(RS485_RX_BUFF[3])
		{
			
			case 0x86:
				Set_Slave_Addr();	//����վ��
				break;
			case 0x87:
				Set_Clear_Period();	//�����������
				break;
			case 0x88:
				Write_DarkSp_To_Rom();	//д�밵����
				break;
			case 0x89:
				Write_ReferenceSp_To_Rom();	//д��ο�����
				break;
			case 0x8a:
				Write_WhiteSp_To_Rom();		//д��׹���
				break;
			case 0x96:					//��ȡϵ��
				Send_Ratio_To_USART();
				break;
			case 0x97:					//��ȡROM������
				Send_DarkSp_To_USART();
				break;
			case 0x98:					//��ȡROM�αȹ���
				Send_ReferenceSp_To_USART();
				break;
			case 0x99:
				Send_WhiteSp_To_USART();	//��ȡROM�׹���
				break;
			case 0x9a:
				Send_WaveData_To_USART();	//��ȡ��������
				break;
			case 0x9c:					//����ƽ������
				Set_SP_Average_Time();
				break;
			case 0x9d:
				Set_SP_Pulse_Mode();	//��������ģʽ
				break;
			case 0x9e:
				Clear_Device();
				break;
			default:
				break;
		}
	
		
}
//Modbus������15�������   //////////////////////////////////////////////////////��������֤OK
//д������������
void Modbus_15_Solve(void)
{
        u16 i;//���ݷ���У����
        i=(((u16)RS485_RX_BUFF[4])<<8)|RS485_RX_BUFF[5];//��ȡ�Ĵ�������
         if(i==ValueOrLenth)
		{
			ComErr=0;
		}
         else
		{
			ComErr=15;
		}
		TX_RX_SET=0; //�������   
}

//Modbus������16������� /////////////////////////////////////////////////////////////////////////////////////////////////����֤����OK
//д������ּĴ���
void Modbus_16_Solve(void)
{		
	
	switch(RS485_RX_BUFF[3])
	{	
		//д��ϵ����ROM
		case 0x8b:
			Write_Ratio_To_Rom();
			break;
		case 0x9c:	//���û���ʱ��
			Set_SP_IntegTime();
			break;
		//��������ߵ�ʱ��
		case 0x9e:
			Set_SP_Pulse_Time();
			break;
		case 0x9f:
			Set_Calc_Wave();
			break;
		
	}
}

//-------------------------

//��ȡ�û�д���ϵ��
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


//��Ļ����ָ��
//д�롣������
//void  Send_All_Info_To_Screen(void)
//{
//	u16 crc;
//	double originValue=0; //ԭʼŨ��ֵ
//	u32 result=0;
//	u16 temp_1,temp_2;
//	u16 humidity_1,humidity_2;
//	
//	totalTime+=1.5;	//ʱ���һ
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
//		calculateConcentration();	//��ȡԭʼŨ��ֵ
//		originValue=curValue;
//		//curValue=getCalibratedData(curValue);	//��ȡУ׼���Ũ��ֵ
//		nextMeansureTime=setNextMeansureTime;
//	}
//	
//	//�¶�1 ʪ��1
//	temp_1=(int)(SHT2x_GetTempPoll1()*10);
//	humidity_1=(int)(SHT2x_GetHumiPoll1()*10);
//	//�¶�2  ʪ��2
//	temp_2=(int)(SHT2x_GetTempPoll2()*10);
//	humidity_2=(int)(SHT2x_GetHumiPoll2()*10);
//	
//	result=(u32)(originValue*100);	//ԭʼŨ��ֵ
//	u32 curRes=(u32)(curValue*100);	//��ǰ����ֵ
//	u32 total=(u32)(totalTime*100);	//�ܲ���ʱ��
//	total/=3600;			//ת��ΪСʱ
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
//	//��ǰ����ֵ
//	RS485_TX_BUFF[7]=(curRes>>8)&0xff;
//	RS485_TX_BUFF[8]=curRes&0xff;
//	RS485_TX_BUFF[9]=curRes>>24;
//	RS485_TX_BUFF[10]=(curRes>>16)&0xff;
//	
//	//�ۼƹ���ʱ�� ��λʱ
//	RS485_TX_BUFF[11]=(total>>8)&0xff;
//	RS485_TX_BUFF[12]=total&0xff;
//	RS485_TX_BUFF[13]=total>>24;
//	RS485_TX_BUFF[14]=(total>>16)&0xff;
//	
//	RS485_TX_BUFF[15]=nextMeansureTime>>8;
//	RS485_TX_BUFF[16]=nextMeansureTime&0xff;
//	
//	//�¶�1��ʪ��1
//	RS485_TX_BUFF[17]=temp_1>>8;
//	RS485_TX_BUFF[18]=temp_1&0xff;
//	
//	RS485_TX_BUFF[19]=humidity_1>>8;
//	RS485_TX_BUFF[20]=humidity_1&0xff;
//	//�¶�2��ʪ��2
//	RS485_TX_BUFF[21]=temp_2>>8;
//	RS485_TX_BUFF[22]=temp_2&0xff;
//	
//	RS485_TX_BUFF[23]=humidity_2>>8;
//	RS485_TX_BUFF[24]=humidity_2&0xff;
//	
//	
//	//ԭʼŨ��ֵ

//	RS485_TX_BUFF[25]=(result>>8)&0xff;
//	RS485_TX_BUFF[26]=result&0xff;
//	RS485_TX_BUFF[27]=result>>24;
//	RS485_TX_BUFF[28]=(result>>16)&0xff;
//	
//	crc=CRC_Compute(RS485_TX_BUFF,29);
//	RS485_TX_BUFF[29]=crc&0xff;
//	RS485_TX_BUFF[30]=crc>>8;
//	
//	//���ø��ֱ���״̬
//	setAlarmState(temp_1,temp_2,humidity_1,humidity_2);
//	
//	isNeedProcess=0;
//	RS485_SendData(RS485_TX_BUFF,31);
//	
//}



//��Ļ����ָ��
//д�� ��ҳ��ȫ����ֵ
void  Send_All_Info_To_Screen(void)
{
	u16 crc;

	u32 result=0;
	u16 temp_1,temp_2;
	u16 humidity_1,humidity_2;
	
	totalTime+=1.5;	//ʱ���һv
	nextMeansureTime-=1;
	clearPeriod--;
	
	
	if(clearPeriod <=0 )
	{	
		clearPeriod=setClearPeriod;
		Motor_move(2);
	}
	if(nextMeansureTime<=0)
	{	
		curValue=calculateConcentration();	//��ȡԭʼŨ��ֵ
		originValue=curValue;
		curValue=getCalibratedData(curValue);	//��ȡУ׼���Ũ��ֵ
		//curValue*=1.12;
		nextMeansureTime=setNextMeansureTime;
	}
	
	//�¶�1 ʪ��1
	temp_1=(int)(SHT2x_GetTempPoll1()*10);
	humidity_1=(int)(SHT2x_GetHumiPoll1()*10);
	//�¶�2  ʪ��2
	temp_2=(int)(SHT2x_GetTempPoll2()*10);
	humidity_2=(int)(SHT2x_GetHumiPoll2()*10);
	
	result=(u32)(originValue*1e4);	//ԭʼŨ��ֵ //������λС��
					
	u32 curRes=(u32)(curValue*100);	//��ǰ����ֵ
	u32 total=(u32)(totalTime*100);	//�ܲ���ʱ��
	total/=3600;			//ת��ΪСʱ
	

	RS485_TX_BUFF[0]=SlaverAddr;
	RS485_TX_BUFF[1]=0x10;
	RS485_TX_BUFF[2]=0x00;
	RS485_TX_BUFF[3]=0x00;
	RS485_TX_BUFF[4]=0x00;
	RS485_TX_BUFF[5]=0x0b;
	RS485_TX_BUFF[6]=0x16;
	
	//��ǰ����ֵ
	RS485_TX_BUFF[7]=(curRes>>8)&0xff;
	RS485_TX_BUFF[8]=curRes&0xff;
	RS485_TX_BUFF[9]=curRes>>24;
	RS485_TX_BUFF[10]=(curRes>>16)&0xff;
	
	//�ۼƹ���ʱ�� ��λʱ
	RS485_TX_BUFF[11]=(total>>8)&0xff;
	RS485_TX_BUFF[12]=total&0xff;
	RS485_TX_BUFF[13]=total>>24;
	RS485_TX_BUFF[14]=(total>>16)&0xff;
	
	RS485_TX_BUFF[15]=nextMeansureTime>>8;
	RS485_TX_BUFF[16]=nextMeansureTime&0xff;
	
	//�¶�1��ʪ��1
	RS485_TX_BUFF[17]=temp_1>>8;
	RS485_TX_BUFF[18]=temp_1&0xff;
	
	RS485_TX_BUFF[19]=humidity_1>>8;
	RS485_TX_BUFF[20]=humidity_1&0xff;
	//�¶�2��ʪ��2
	RS485_TX_BUFF[21]=temp_2>>8;
	RS485_TX_BUFF[22]=temp_2&0xff;
	
	RS485_TX_BUFF[23]=humidity_2>>8;
	RS485_TX_BUFF[24]=humidity_2&0xff;
	
	
	//ԭʼŨ��ֵ

	RS485_TX_BUFF[25]=(result>>8)&0xff;
	RS485_TX_BUFF[26]=result&0xff;
	RS485_TX_BUFF[27]=result>>24;
	RS485_TX_BUFF[28]=(result>>16)&0xff;
	
	crc=CRC_Compute(RS485_TX_BUFF,29);
	RS485_TX_BUFF[29]=crc&0xff;
	RS485_TX_BUFF[30]=crc>>8;
	
	
	//���ø��ֱ���״̬
	setAlarmState(temp_1,temp_2,humidity_1,humidity_2);
	
	
	RS485_SendData(RS485_TX_BUFF,31);
	
}

//��ȡУ׼���Ũ��ֵ
double getCalibratedData(double x)
{
	
	
	double result=0;
	//y=a+bx^2+cx
	
	result=SP_b+(SP_c*x*x)+SP_d*x;
	//SP_b*(x*x)+SP_c*x+SP_d;
	return result;
}

//��������
//����ԭʼŨ��ֵ
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


//��ȡ��ť��ָ��
//д�뵱ǰ����ֵ���ۼƹ���ʱ������һ�β���ʱ��
//void Send_Time_To_Screen()
//{	
//	
//	u16 crc;
//	//����
//	totalTime+=1.5;	//ʱ���һ
//	nextMeansureTime-=1;
//	clearPeriod--;
//	if(clearPeriod <=0 )
//	{	
//		clearPeriod=setClearPeriod;
//		Motor_move(2);
//	}
//	if(nextMeansureTime<=0)
//	{	
//		curValue=calculateConcentration();	//��ȡԭʼŨ��ֵ
//		curValue=getCalibratedData(curValue);	//��ȡУ׼���Ũ��ֵ
//		nextMeansureTime=setNextMeansureTime;
//	}
//	
//	u32 curRes=(u32)(curValue*100);	//��ǰ����ֵ
//	u32 total=(u32)(totalTime*100);	//�ܲ���ʱ��
//	total/=3600;			//ת��ΪСʱ
//	

//	RS485_TX_BUFF[0]=SlaverAddr;
//	RS485_TX_BUFF[1]=0x10;
//	RS485_TX_BUFF[2]=0x00;
//	RS485_TX_BUFF[3]=0x00;
//	RS485_TX_BUFF[4]=0x00;
//	RS485_TX_BUFF[5]=0x05;
//	RS485_TX_BUFF[6]=0x0A;
//	
//	//��ǰ����ֵ
//	RS485_TX_BUFF[7]=(curRes>>8)&0xff;
//	RS485_TX_BUFF[8]=curRes&0xff;
//	RS485_TX_BUFF[9]=curRes>>24;
//	RS485_TX_BUFF[10]=(curRes>>16)&0xff;
//	
//	//�ۼƹ���ʱ�� ��λʱ
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
//������ʪ��
void Send_Temp_Humidity_To_Screen()
{	
	
	
	u16 crc;
	u16 temp_1,temp_2;
	u16 humidity_1,humidity_2;
	//�¶�1 ʪ��1
	temp_1=(int)(SHT2x_GetTempPoll1()*10);
	humidity_1=(int)(SHT2x_GetHumiPoll1()*10);
	//�¶�2  ʪ��2
	temp_2=(int)(SHT2x_GetTempPoll2()*10);
	humidity_2=(int)(SHT2x_GetHumiPoll2()*10);
	
	//���ø��ֱ���״̬
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

void Calc_Query_Btn(void) //ȡ�����õ�ȷ����ť
{
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x10,0x00,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
}

//��ȡ��ťȫ��״̬
void  Check_Btn_State(void)
{	
	u16 crc;
	u8 btnStateOrder[8]={SlaverAddr,0x01,0x00,0x00,0x00,0x11};
	crc=CRC_Compute(btnStateOrder,6);
	btnStateOrder[6]=crc&0xff;
	btnStateOrder[7]=crc>>8;
	RS485_SendData(btnStateOrder,8);
	

}
//�����û��궨��ϵ��
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
	//����ϵ��
	if(num>0)
	{
		
		SP_a=0;
		Fit2(xTemp,yTemp,num,&SP_b,&SP_c,&SP_d);
		Write_User_Ratios_To_Rom(buf);
	}
}

//������ť״̬
void Analysis_Btn_State()
{	
	
	u8 i=0;
	u16 crc;
	u8 buf[8]={SlaverAddr,0x03,0x00,0x2a,0x00,0x06};
	u8 buf2[8]={SlaverAddr,0x05,0x00,0x11,0x00,0x00};
	
	
	for(i=0;i<8;i++)	//Ũ��У׼
	{
		if(GET_BIT(RS485_RX_BUFF[4],i))
		{
			Select_Concentration(i,i+8,40+i*2);
			break;
		}
	}
//	//��ĻһУ׼��ť����
	if(GET_BIT(RS485_RX_BUFF[5],0))
	{	
		
		Process_Ratios_From_User();
		//delay_ms(50);
		Calc_Query_Btn();
		
		
	}
	//��Ļ��У׼��ť����
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
		//������ˢ
		Switch_Rubber_Brush();
	}
	else if(GET_BIT(RS485_RX_BUFF[3],1))
	{
		//̽ͷ�����
		Detector_Diagnosis();
		//Motor_move(2);
		
	}
	
	else if(GET_BIT(RS485_RX_BUFF[3],6))	//����ȷ����ť
	{	
		
		Get_All_Setting();
		
		
	}
	else if(GET_BIT(RS485_RX_BUFF[3],7))	//������������
	{
		Reset_All_Setting();
	}

}

//void  Send_Value_To_USER(void)	//����ԭʼŨ��
//{	
//	double content=0;
//	u32 temp=0;
//	u16 crc;
//	u8 conBuf[13]={SlaverAddr,0x10,0x00,0x28,0x00,0x02,0x04};
//	content=calculateConcentration();
//	temp=(u32)(content*10000);
//	conBuf[7]=(temp>>8)&0xff;		//��ֵ
//	conBuf[8]=temp&0xff;
//	conBuf[9]=temp>>24;
//	conBuf[10]=(temp>>16)&0xff;
//	crc=CRC_Compute(conBuf,11);
//	conBuf[11]=crc&0xff;
//	conBuf[12]=crc>>8;
//	isNeedProcess=0;
//	RS485_SendData(conBuf,13);	//����ԭʼֵ
//	delay_ms(20);
//}


//���������ȡ�û�ϵ��

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

//������ˢ
void Switch_Rubber_Brush()
{	
	//��λ��ˢ״̬
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x00,0x00,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
	Motor_move(1);	//1������ˢ 2���
}

//̽ͷ���
void Detector_Diagnosis(void)
{	
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x01,0x00,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
	//Motor_move(0);	//ˢƬ��λ
}
//̽ͷУ׼
void Detctor_Calibration(void)
{	
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x02,0x00,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
}

//����ѡ��Ũ��ѡ��
void Select_Concentration(u8 index,u16 switchAddr,u16 valueAddr)
{
	u16 crc;
	u32 temp;
	double content=0.0;
	u8 conBuf[13]={SlaverAddr,0x10,0x00,0x00,0x00,0x02,0x04};
	u8 buf[8]={SlaverAddr,0x05,0x00,0x00,0x00,0x00};	//����
	
	
	buf[2]=switchAddr>>8;
	buf[3]=switchAddr&0xff;
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	
	content=calculateConcentration();	//������Һ����Ũ��
	xConcentration[index]=content;
	conValueFlag[index]=1;
	//������λС��
	temp=(u32)(content*1e4);			//����Ũ�ȵ���Ļ

	//temp=25000;
	conBuf[2]=valueAddr>>8;				//д����Ļ����Ȧ��ַ
	conBuf[3]=valueAddr&0xff;
	
	conBuf[7]=(temp>>8)&0xff;		//��ֵ
	conBuf[8]=temp&0xff;
	conBuf[9]=temp>>24;
	conBuf[10]=(temp>>16)&0xff;
	
	crc=CRC_Compute(conBuf,11);		//����CRC
	conBuf[11]=crc&0xff;
	conBuf[12]=crc>>8;
	
	RS485_SendData(conBuf,13);	//����Ũ��ֵ
	delay_ms(50);
	RS485_SendData(buf,8);
	delay_ms(50);
}



 void Reset_All_Setting(void)			//������������
{	
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x07,0x00,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
}




void Get_All_Setting(void)				//��ȡ��������
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

//Ӧ����������
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

void Start_Darw()		//��ʼ��ͼ
{
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x0a,0xff,0x00};
	crc=CRC_Compute(buf,6);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
}
//ֹͣ��ͼ
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
//����Ũ��
void CalculateValue(void)
{
	u8 i=0;
	u8 num=0;
	
	double pw_1,pw_2;
	double A1=0.0,A2=0.0;
	//����ϵ��
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


//����ԭʼŨ��ֵ
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
//	CalculateValue();	//����Ũ��ֵ
//	
//}
//����궨�õ�Ũ��ֵ
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

//��λ�������Ӧ��ϵ����a b c d) ��������ROM��
void calculateCalibrationRatios(void)
{

}



//��ȡ�궨�õĲ���
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


//��������ʱʹ��
////��ȡ�궨�õĲ���
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


//���ñ���״̬
void setAlarmState(int temp1,int temp2,int humidity1,int humidity2)
{
	u16 crc;
	u8 buf[8]={SlaverAddr,0x05,0x00,0x18,0xff,0x00};

	//ʪ�ȹ��󾯱�
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
 

//�ϵ��ʼ�����ֲ���
void Init_All_Argument()
{
	u8 buf[2];
	Init_Get_Wave();
	//��ȡ�������
	AT24CXX_Read(SP_CLEAR_PERIOD_ADDR,buf,SP_CLEAR_PERIOD_LEN);
	clearPeriod=(buf[0]>>8) | buf[1];
	setClearPeriod=clearPeriod;
	
	//��ȡ�궨�õĲαȹ�·
	Read_220_275_Spectrum(SP_REFENCE_SPECTRUM_ADDR,&Reference_220_Data,&Reference_275_Data);
	//�׹���
	Read_220_275_Spectrum(SP_WHITE_SPECTRUM_ADDR,&White_220_Data,&White_275_Data);
	//������
	Read_220_275_Spectrum(SP_DARK_SPECTRUM_ADDR,&Dark_220_Data,&Dark_275_Data);
	
	//��ȡ�������ʵ�ϵ��
	getRatios_Sample(SP_RATIO_ADDR);
	//��ȡϵ��
	//getAllInitRatios();	//
	
	
}


//�ָ���������
//�ָ�ϵ��
//�ָ��������
void Restore_Factory_Setting()
{	
//	u8 buf[2];
//	//��ȡ�������õ�ϵ������
//	getRatios(SP_RATIO_FACTORY_ADDR);
//	//��ȡ�������
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
	i=9;	//��
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


//��ʼ��ʱ��ȡ����
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





//��ȡ�궨�õĹ���
void Read_220_275_Spectrum(u16 addr,double*data_220,double*data_275)
{	
	u16 i=0;
	float x1=0,x2=0;
	u16 y1=0,y2=0;
	u8 buf[2];
	//��ȡ�궨�õĹ���
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



void Clear_Device(void)	//����豸
{	
	u16 crc;
	u8 buf[8]={SlaverAddr,0x06,0x00,0x9e,0x00,0x01};
	Motor_move(2);	//���
	crc=CRC_Compute(buf,7);
	buf[6]=crc&0xff;
	buf[7]=crc>>8;
	RS485_SendData(buf,8);
	
}

void Send_Temp_Humidity(void)	//��ʪ�ȸ���λ��
{	
	
	u8 buf[15]={SlaverAddr,0x03,0x0a};
	u16 crc;
	u16 temp_1,temp_2;
	u16 humidity_1,humidity_2;
	//�¶�1 ʪ��1
	temp_1=(int)(SHT2x_GetTempPoll1()*10);
	humidity_1=(int)(SHT2x_GetHumiPoll1()*10);
	//�¶�2  ʪ��2
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

//д��ϵ��
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
	
	AT24CXX_Write(SP_RATIO_ADDR,buf,SP_RATIO_LEN);//д����Ļ�궨�Ĳ���
	
}
//����������������ʲ���
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
//д��ĳ�����ʼ��㲨��
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
//��ȡĳ�����ʵļ��㲨��
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

//��ȡ�������ʵļ���ϵ��
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
