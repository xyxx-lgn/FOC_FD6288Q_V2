#ifndef __MY_CORDIC_H_
#define __MY_CORDIC_H_

#include "cordic.h"

#define Q31 0x80000000
#define RADIAN_Q31_f 683565275.6f

void cordic_config(void);
void Calculate_Float_Sin(float angle,float *sin);
void Calculate_Float_Sin_Cos(float angle,float *sin, float *cos);
	
#endif
