#ifndef __PID_H
#define __PID_H		


//头文件包含
#include "ALL_H.h"



void PID_I_Control(PID *pid);
void PID_Speed_Control(PID *pid,Encoder_Struct *encoder_str);
void PID_Position_Control(PID *pid,Encoder_Struct *encoder_str);
void Planner_Speed_Init(Simple_Position_Speed_Planner * planner,float target_speed,float max_accel,float current_speed);
float Planner_Speed_Update(Simple_Position_Speed_Planner * planner,float dt);
void Planner_Position_Init(Simple_Position_Speed_Planner * planner,float target_angle,float max_accel,float current_speed,float curise_speed);
float Planner_Position_Update(Simple_Position_Speed_Planner * planner,float current_angle,float dt);
#endif
