#ifndef __SHUTTER_H
#define __SHUTTER_H
#include "sys.h"

#define shutter           PBout(10)

void Shutter_Init(void);
void Set_shutter(u8 state);

#endif




