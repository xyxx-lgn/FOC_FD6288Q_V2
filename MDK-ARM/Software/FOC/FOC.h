#ifndef __FOC_H
#define __FOC_H		


//头文件包含
#include "ALL_H.h"

#define MOTOR_VDD 24.0f

float ElectAngle_Limit(float Angle,uint8_t pole);
void Clark_Park(AdcValue *adcvalue,Encoder_Struct *encoder_str,PID *pid,SVPWM_Struct *svpwm_str);
void SVPWM(float Uq,float Ud,float Angle,SVPWM_Struct *svpwm_str);
void SVPWM_Zero(float Uq,float Ud,float elect_angle,SVPWM_Struct *svpwm_str);
void SVPWM_Zero1(float *Uq, float *Ud, float *angle_el);

#endif
