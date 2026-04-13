#ifndef __ALL_H
#define __ALL_H		


//头文件包含
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "gpio.h"
#include "tim.h"
#include "math.h"
#include "adc.h"
#include "arm_math.h"         //DSP库头文件
#include "my_cordic.h"        //自定义cordic库文件
#include "Filter.h"           //Fir滤波器
#include "usart.h"

#include "Start.h"
#include "My_Can.h"

#define Limit(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define sqrt3 1.7320508f
#define one_sqrt3 0.57735027f
#define M_PI 3.14159265358979323846f
#define _2PI 6.283185307f
//变量集合

typedef struct
{  

	float supply_Udc;           //输入电压值最大值
	float supply_IMax;          //电流最大值
	float motor_gear;           //电机减速比
	float motor_phaseL;         //电机相电感
	float motor_phaseR;         //电机相电阻
	float motor_sampleR;        //电机采样电阻
	uint8_t motor_pole;         //电机极对数
	float Ia_Sample;            //A相电流采样值
	float Ib_Sample;
	float Ic_Sample;
	float Udc_Sample;           //母线电压采样值
	float Ia_offect;            //电压偏置
	float Ib_offect;
	float Ic_offect;
	float Ia;
	float Ib;
	float Ic;
	float Udc;
	uint16_t Iadc_count;        //ADC校准计次变量
	float Gain_I;               //电流增益倍数 Gain*Rs=50*0.02=1.0
}AdcValue;


typedef struct
{
	uint8_t Erro_flag;          //错误化标志位，0代表正常，1代表电压异常,2代表电流超限
	uint8_t Adc_Adjust_flag;    //Adc校准标志位，0代表未校准，1代表校准完成，2代表过流
	uint8_t Zero_flag;          //编码器零点校准标志位,0代表未校准，1代表校准完成  
	uint8_t Encoder_flag;       //编码器模式 
	uint8_t Mode_flag;          //电机控制模式选择
	uint8_t CANFD_Tx_flag;      //CANFD成功发送标志位，1代表成功发送，0代表发送失败
	uint16_t Erro_counts;       //错误计次
}AllFlag;

typedef struct   
{
	uint8_t motordir;              //电机方向 //(-1为逆时针正向旋转，1为顺时针反向旋转)
	float r_s_speed;           //开环设置的转速，单位：圈/秒(指的是机械角度)
	uint16_t Encoder;
	float Encoder_raw;      //编码器原始值
	float Encoder_old_raw;  //上一次编码器原始值
	int Encoder_raw_erro; //编码器误差值
	float Encoder_raw_sum;  //编码器原始值和
	float Shaft_Angle; 		   //机械角度
	float Elect_Angle;		   //电角度
	float Encoder_Mode1_Angle; //模式1临时角度变量
	float Encoder_Mode3_Angle; //模式1临时角度变量
	float Return_Rad;          //真实返回角度,归一化到[0-2PI]
	float Return_Angle;        //真实返回角度(输出给SVPWM),归一化到[-PI,PI]
	float Zero_Angle_cal;      //零点偏移角度计算中间变量
	float Zero_Angle;          //零点偏移角度
	uint16_t zero_count;       //零点校准计次
	int rotation_counts;       //电机轴旋转圈数计次
	float last_shaft_angle;    //上一次电机轴角度，用于判断圈数
	float shaft_total_angle;   //电机轴累计角度
	float output_shaft_angle;  //输出轴绝对角度
}Encoder_Struct;               //编码器方向校准和零点校准

typedef struct    
{
	float Udc;                 //母线电压                          
	float U1,U2,U3;            //(经典SVPWM)U1,U2,U3是用来扇区判断的---(在零序注入)为三相电压
	float Ualpha,Ubeta;        //克拉克变换中间变量
	uint8_t A,B,C,N;       	   //配合U1,U2,U3判断扇区
	uint8_t Sector;            //扇区号
	float X,Y,Z;               //方便计算T4，T6时间,临时变量
	float T4,T6;         	   //T4，T6调制时间
	float T4_temp,T6_temp;     //当T4+T6>Ts时的临时变量
	uint16_t Ts;               //PWM的Pulse最大值
	float Ta,Tb,Tc;            //三相切换点时间                           
	uint16_t PWMA,PWMB,PWMC;   //PWMABC的Pulse值
	float sin_dsp,cos_dsp;     //计算完的正余弦值
}SVPWM_Struct;


typedef struct
{
	float max_accel_speed;            //设定加速度
	float current_plan_speed;         //当前规划速度
	//速度规划
	float target_speed;               //目标速度
	uint8_t plan_speed_isactive;      //速度梯形规划器标志
	//位置规划 
	float target_angle;               //目标位置
	float curise_speed;               //巡航速度
	uint8_t plan_position_isactive;   //位置梯形规划器标志
}Simple_Position_Speed_Planner;  //速度和位置规划的结构体，用梯形加速来实现


typedef struct
{
	//下面为电流环参数
	float Kp_i;         //iq的Kp值    
	float Ki_i;         //iq的Ki值    
	
	float Iq_aim;        //iq目标值
	float Iq_current;    //iq实际值
	float Iq_current_OK;
	float erro_iq;       //PI控制器里面的误差，等于目标值-实际值
	float erro_iq_sum;   //Ki的积分项
	float Uq;            //输出的Uq值
	
	float Ki_SumMax;     //KI的积分限幅最大值
	float Ialfa;         //电流环帕克变换
	float Ibeta;
	float Vmax;          //电流环输出最大电压矢量
	float Iqd_MAX;       //Iq,Id设定最大值
	

	float Id_aim;        //id目标值
	float Id_current;    //id实际值
	float erro_id;       //PI控制器里面的误差，等于目标值-实际值
	float erro_id_sum;   //Ki的积分项
	float Ud;            //输出的Ud值
	
	//更换后的增量式电流环PID
	float Kp_Current;
	float Ki_Current;
	float prevError;     // 上次误差
    float prevOutput;    // 上次输出
	
	
	//下面为速度环参数
	float Kp_speed;      //速度环Kp值
	float Ki_speed;      //速度环Ki值
	
	float target_speed;  //目标速度设定，随指令更改
	float Speed_aim;     //目标速度设定，放在PID运算里面，单位：转/分
	float Speed_last;    //实际速度
	float Speed_now;     //实际速度
	float erro_speed;    //速度误差
	float erro_speed_sum;//速度积分误差
	float speed_out;     //PI输出结果，输出为电流环Iq_aim
	
	float speed_out_max; //速度环输出Iq限幅值
	float speed_max;     //速度设定限幅
	uint8_t speed_count; //速度环计次
	
	//下面为位置环
	float Kp_position;        //位置环Kp值
	float Ki_position;        //位置环Ki值 
	
	float Position_aim;       //目标位置，取值[0,360]
	float erro_positon;       //位置误差=Position_aim-Shaft_Angle
	float erro_position_sum;  //位置积分误差 
	float position_out;       //位置环PI输出，输出为速度环的Speed_aim  
	uint8_t position_count;   //位置环计次
	
	float position_out_max;   //位置环的输出限幅
	
	Simple_Position_Speed_Planner simple_plan; //规划器
}PID;

typedef struct
{
	uint16_t CAN_ID;
	uint8_t command_type;
	uint32_t target_value;
	uint8_t direction;
	float Control_Target;
}CANFD_Message;    //CAN通信消息结构体




#endif
