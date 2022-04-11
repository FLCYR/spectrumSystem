
#ifndef FITTING_ALGORITHM_H
#define	FITTING_ALGORITHM_H

#include "stm32f10x.h"
#include "string.h"
//拟合波长
double Fitting_WaveLength(u8*data,float * waveData,u16 length,float fitWave);
//拟合浓度
void Fitting_Concentration(void);

//三阶拟合
void Fit3(double *x, double *y, int num, double *a, double *b, double *c, double *d);
/**
 * 说明：2阶拟合函数
 * x：x坐标值
 * y：y坐标值
 * num：数据个数
 * a,b,c：返回拟合系数
 */
 
void Fit2(double*x, double*y, int num, double *a, double *b, double *c);
#endif
