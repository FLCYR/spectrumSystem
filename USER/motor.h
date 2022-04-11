#ifndef __MOTOR_H
#define __MOTOR_H	 
#include "sys.h"

////////////////////////////////////////////////////////////////////////////////// 
#define Discs3	 PAout(7)	// PD2	
#define Discs2   PBout(0)	// PD2	
#define Discs1   PBout(1)	// PD2	
#define sense    PBout(2)	// PD2	

#define RS485_En    	PAout(8)	// PD2	

#define HallA           PAin(4)	//	
#define HallB           PAin(5)	//	
#define HallC           PAin(6)	//	

#define MIN2   			PAout(12)	//	
#define MIN1    		PAout(11)	//	

//#define LED1    		PCout(13)	//	

void Motor_Init(void);
void Motor_move(char dir);
void Dis_check_Init(void);
extern u8  change_brush;
		 				    
#endif
