
#ifndef FITTING_ALGORITHM_H
#define	FITTING_ALGORITHM_H

#include "stm32f10x.h"
#include "string.h"
//��ϲ���
double Fitting_WaveLength(u8*data,float * waveData,u16 length,float fitWave);
//���Ũ��
void Fitting_Concentration(void);

//�������
void Fit3(double *x, double *y, int num, double *a, double *b, double *c, double *d);
/**
 * ˵����2����Ϻ���
 * x��x����ֵ
 * y��y����ֵ
 * num�����ݸ���
 * a,b,c���������ϵ��
 */
 
void Fit2(double*x, double*y, int num, double *a, double *b, double *c);
#endif
