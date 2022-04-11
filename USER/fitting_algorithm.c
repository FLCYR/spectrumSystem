
#include "fitting_algorithm.h"

//获取浓度值
double Fitting_WaveLength(u8*data,float*waveData,u16 length,float fitWave)
{
	

	float x1,x2;
	u16 y1=1,y2=1;
	u16 i=0;
	
	for(i=0;i<length;i++)
	{
		if(waveData[i]>fitWave)
		{
			x2=waveData[i];
			y2=(data[i*2+4]<<8)|data[i*2+5];
			break;
		}
		
	}
	
	if(i>0)
	{
		x1=waveData[i-1];
		y1=(data[(i-1)*2+4]<<8)|data[(i-1)*2+5];
			
	}

	//y=(x-x1)(y2-y1)/(x2-x1)+y1
	return ((fitWave-x1)*(y2-y1))/(x2-x1)+y1;
	
}
//得到当前浓度值
void Fitting_Concentration()
{
	
}

//三阶拟合
void Fit3(double *x, double *y, int num, double *a, double *b, double *c, double *d)
{	
	int i;
    double sum_x = 0;
    double sum_y = 0;
    double sum_x2 = 0;
    double sum_x3 = 0;
    double sum_x4 = 0;
    double sum_x5 = 0;
    double sum_x6 = 0;
    double sum_xy = 0;
    double sum_x2y = 0;
    double sum_x3y = 0;
 
    for (i= 0; i < num; ++i)
    {	
		
        sum_x += x[i];
        sum_y += y[i];
        sum_x2 += x[i] * x[i];
        sum_x3 += x[i] * x[i] * x[i];
        sum_x4 += x[i] * x[i] * x[i] * x[i];
        sum_x5 += x[i] * x[i] * x[i] * x[i] * x[i];
        sum_x6 += x[i] * x[i] * x[i] * x[i] * x[i] * x[i];
        sum_xy += x[i] * y[i];
        sum_x2y += x[i] * x[i] * y[i];
        sum_x3y += x[i] * x[i] * x[i] * y[i];
    }
 
    sum_x /= num;
    sum_y /= num;
    sum_x2 /= num;
    sum_x3 /= num;
    sum_x4 /= num;
    sum_x5 /= num;
    sum_x6 /= num;
    sum_xy /= num;
    sum_x2y /= num;
    sum_x3y /= num;
 
    double m1 = (sum_x3 * sum_x6 - sum_x4 * sum_x5) / (sum_x4 * sum_x6 - sum_x5 * sum_x5);
    double n1 = (sum_x2 * sum_x6 - sum_x3 * sum_x5) / (sum_x4 * sum_x6 - sum_x5 * sum_x5);
    double k1 = (sum_x2y * sum_x6 - sum_x3y * sum_x5) / (sum_x4 * sum_x6 - sum_x5 * sum_x5);
 
    double m2 = (sum_x2 * sum_x6 - sum_x4 * sum_x4) / (sum_x3 * sum_x6 - sum_x5 * sum_x4);
    double n2 = (sum_x * sum_x6 - sum_x3 * sum_x4) / (sum_x3 * sum_x6 - sum_x5 * sum_x4);
    double k2 = (sum_xy * sum_x6 - sum_x3y * sum_x4) / (sum_x3 * sum_x6 - sum_x5 * sum_x4);
 
    double m3 = (sum_x * sum_x6 - sum_x4 * sum_x3) / (sum_x2 * sum_x6 - sum_x5 * sum_x3);
    double n3 = (sum_x6 - sum_x3 * sum_x3) / (sum_x2 * sum_x6 - sum_x5 * sum_x3);
    double k3 = (sum_y * sum_x6 - sum_x3y * sum_x3) / (sum_x2 * sum_x6 - sum_x5 * sum_x3);
 
    *d = ((k3 - k1) / (m3 - m1) - (k2 - k1) / (m2 - m1)) / ((n3 - n1) / (m3 - m1) - (n2 - n1) / (m2 - m1));
    *c = (k2 - k1) / (m2 - m1) - (n2 - n1) / (m2 - m1) * (*d);
    *b = k1 - m1 * (*c) - n1 * (*d);
    *a = sum_x3y / sum_x6 - sum_x5 / sum_x6 * (*b) - sum_x4 / sum_x6 * (*c) - sum_x3 / sum_x6 * (*d);
	
}
//二阶拟合
void Fit2(double*x, double*y, int num, double *a, double *b, double *c)
{
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_x2 = 0.0f;
    float sum_x3 = 0.0f;
    float sum_x4 = 0.0f;
    float sum_xy = 0.0f;
    float sum_x2y = 0.0f;
 
    for (int i = 0; i < num; ++i)
    {
        sum_x += x[i];
        sum_y += y[i];
        sum_x2 += x[i] * x[i];
        sum_x3 += x[i] * x[i] * x[i];
        sum_x4 += x[i] * x[i] * x[i] * x[i];
        sum_xy += x[i] * y[i];
        sum_x2y += x[i] * x[i] * y[i];
    }
 
    sum_x /= num;
    sum_y /= num;
    sum_x2 /= num;
    sum_x3 /= num;
    sum_x4 /= num;
    sum_xy /= num;
    sum_x2y /= num;
 
    *b = ((sum_x * sum_y - sum_xy) / (sum_x3 - sum_x2 * sum_x) - (sum_x2 * sum_y - sum_x2y) / (sum_x4 - sum_x2 * sum_x2)) /
         ((sum_x3 - sum_x2 * sum_x) / (sum_x4 - sum_x2 * sum_x2) - (sum_x2 - sum_x * sum_x) / (sum_x3 - sum_x2 * sum_x));
    *a = (sum_x2y - sum_x2 * sum_y - (sum_x3 - sum_x * sum_x2) * (*b)) / (sum_x4 - sum_x2 * sum_x2);
    *c = sum_y - sum_x2 * (*a) - sum_x * (*b);
}

