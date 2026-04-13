#include "FOC.h"



extern SVPWM_Struct svpwm_str;  //svpwm结构体


/********************
电角度限制：使角度处于0-360
********************/
float ElectAngle_Limit(float Angle,uint8_t pole)
{
	Angle *= pole;
	float a =  fmod(Angle,360);    //取余运算用于归一化
	return a>0?a:(a+360);
	/*************
	fmod函数余数的符号和被除数相同。
	例如：当Angle为-PI/2时，fmod(Angle,2*PI)会返回一个负数，因此通过在结果加上2*PI来确保角度值处于0-2PI
	*************/
}

/********************
克拉克变换和帕克变换  
	用于电流环
********************/
void Clark_Park(AdcValue *adcvalue,Encoder_Struct *encoder_str,PID *pid,SVPWM_Struct *svpwm_str)
{
	/*克拉克变换
		Ialfa = Ia
		Ibeta = (Ia+2Ib)/sqrt(3)
	*/
	pid->Ialfa = adcvalue->Ia;
	pid->Ibeta = (adcvalue->Ia+2*adcvalue->Ib)/sqrt3;
	
	//只在此处计算一次正余弦
	Calculate_Float_Sin_Cos(encoder_str->Return_Angle,&svpwm_str->sin_dsp,&svpwm_str->cos_dsp);  //Cordic库计算三角
	
	/*帕克变换
		Id = Ialfa*cos+Ibeta*sin
		Iq = -Ialfa*sin+Ibeta*cos
	*/
//	pid->Id_current = pid->Ialfa*arm_cos_f32(encoder_str->Return_Angle)+pid->Ibeta*arm_sin_f32(encoder_str->Return_Angle);
//	pid->Iq_current = -pid->Ialfa*arm_sin_f32(encoder_str->Return_Angle)+pid->Ibeta*arm_cos_f32(encoder_str->Return_Angle);
	arm_park_f32(pid->Ialfa,pid->Ibeta,&pid->Id_current,&pid->Iq_current_OK,svpwm_str->sin_dsp,svpwm_str->cos_dsp);
	Filter_DSP_Fir(&pid->Iq_current_OK,&pid->Iq_current);
}


//经典SVPWM(未作任何优化)
void SVPWM(float Uq,float Ud,float Angle,SVPWM_Struct *svpwm_str)
{
	float Ualpha=0,Ubeta=0;
	float gain = (float)sqrt3/svpwm_str->Udc*svpwm_str->Ts;
	float Umax_n = 0.577350269f * svpwm_str->Ts;   // =Ts/sqrt3，对应线性圆半径
	//标幺后，再根据载波定标  标幺：sqrt3*Uref/Udc  ,因为SVPWM里面Uref最大值为Udc/sqrt3
	Uq = Uq*gain;
	Ud = Ud*gain;
	
	//圆形限幅
    float mag = sqrtf(Uq*Uq + Ud*Ud);
    if(mag > Umax_n)
    {
        float scale = Umax_n / mag;
        Uq *= scale;
        Ud *= scale;
    }
	
	//帕克逆变换
	Ualpha = Ud*arm_cos_f32(Angle)-Uq*arm_sin_f32(Angle);
	Ubeta  = Ud*arm_sin_f32(Angle)+Uq*arm_cos_f32(Angle);
//	
	
	//通过U1，2，3和ABCN来判断扇区Sector
	svpwm_str->U1 = Ubeta; 								
	svpwm_str->U2 = sqrt3*Ualpha-Ubeta;
	svpwm_str->U3 = -(sqrt3*Ualpha)-Ubeta;	//通过U1，2，3正负来确定ABC值
	
	//确定ABC值{0,1},N值
	if(svpwm_str->U1>0) svpwm_str->A=1;
	else svpwm_str->A=0;
	if(svpwm_str->U2>0) svpwm_str->B=1;
	else svpwm_str->B=0;
	if(svpwm_str->U3>0) svpwm_str->C=1;
	else svpwm_str->C=0;
	svpwm_str->N = (svpwm_str->C<<2)+(svpwm_str->B<<1)+svpwm_str->A;  //N=C*4+B*2+A
	
	
	//算出XYZ值，方便将调制时间
	svpwm_str->X = Ubeta;
	svpwm_str->Y = 0.5f*(sqrt3*Ualpha+Ubeta);       //标幺后不需要了乘以sqrt3*Ts/Udc！！！！！！
	svpwm_str->Z = 0.5f*(-sqrt3*Ualpha+Ubeta);      //sqrt3*Ts/Udc相同的项没有乘进去，标准的XYZ还需乘以sqrt3*Ts/Udc
	
	//通过N值查表来确定扇区
	switch(svpwm_str->N)
	{
		case 1:
			svpwm_str->T4 = svpwm_str->Z;
			svpwm_str->T6 = svpwm_str->Y;
			svpwm_str->Sector = 2;
			break;
		case 2:
			svpwm_str->T4 = svpwm_str->Y;
			svpwm_str->T6 = -svpwm_str->X;
			svpwm_str->Sector = 6;
			break;
		case 3:
			svpwm_str->T4 = -svpwm_str->Z;
			svpwm_str->T6 = svpwm_str->X;
			svpwm_str->Sector = 1;
			break;
		case 4:
			svpwm_str->T4 = -svpwm_str->X;
			svpwm_str->T6 = svpwm_str->Z;
			svpwm_str->Sector = 4;
			break;
		case 5:
			svpwm_str->T4 = svpwm_str->X;
			svpwm_str->T6 = -svpwm_str->Y;
			svpwm_str->Sector = 3;
			break;
		case 6:
			svpwm_str->T4 = -svpwm_str->Y;
			svpwm_str->T6 = -svpwm_str->Z;
			svpwm_str->Sector = 5;
			break;
		default:             //出现错误，即出现扇区为0的情况
			break;
	}
	
	//过调制判断，当T4+T6>Ts值过调制，就需要缩小比例
	/*
		T4=T4/(T4+T6) * Ts
		T6=T6/(T4+T6) * Ts
	*/
	if((svpwm_str->T4+svpwm_str->T6) > svpwm_str->Ts)
	{
		svpwm_str->T4_temp = svpwm_str->Ts*svpwm_str->T4/(svpwm_str->T4+svpwm_str->T6);
		svpwm_str->T6_temp = svpwm_str->Ts*svpwm_str->T6/(svpwm_str->T4+svpwm_str->T6);
		svpwm_str->T4 = svpwm_str->T4_temp;
		svpwm_str->T6 = svpwm_str->T6_temp;
	}
	
	
	//计算出三相切换点时间
	/*
		Ta=(Ts-T4-T6)/4
		Tb=Ta+T4/2
		Tc=Tb+T6/2
	*/
	svpwm_str->Ta = (svpwm_str->Ts-svpwm_str->T4-svpwm_str->T6)/4.0f;
	svpwm_str->Tb = svpwm_str->Ta+svpwm_str->T4/2.0f;
	svpwm_str->Tc = svpwm_str->Tb+svpwm_str->T6/2.0f;
	
	//根据扇区设置三相比较器的值
	switch(svpwm_str->N)
	{
		case 1:
			svpwm_str->PWMA = (uint16_t)svpwm_str->Tb;
			svpwm_str->PWMB = (uint16_t)svpwm_str->Ta;
			svpwm_str->PWMC = (uint16_t)svpwm_str->Tc;
			break;
		case 2:
			svpwm_str->PWMA = (uint16_t)svpwm_str->Ta;
			svpwm_str->PWMB = (uint16_t)svpwm_str->Tc;
			svpwm_str->PWMC = (uint16_t)svpwm_str->Tb;
			break;
		case 3:
			svpwm_str->PWMA = (uint16_t)svpwm_str->Ta;
			svpwm_str->PWMB = (uint16_t)svpwm_str->Tb;
			svpwm_str->PWMC = (uint16_t)svpwm_str->Tc;
			break;
		case 4:
			svpwm_str->PWMA = (uint16_t)svpwm_str->Tc;
			svpwm_str->PWMB = (uint16_t)svpwm_str->Tb;
			svpwm_str->PWMC = (uint16_t)svpwm_str->Ta;
			break;
		case 5:
			svpwm_str->PWMA = (uint16_t)svpwm_str->Tc;
			svpwm_str->PWMB = (uint16_t)svpwm_str->Ta;
			svpwm_str->PWMC = (uint16_t)svpwm_str->Tb;
			break;
		case 6:
			svpwm_str->PWMA = (uint16_t)svpwm_str->Tb;
			svpwm_str->PWMB = (uint16_t)svpwm_str->Tc;
			svpwm_str->PWMC = (uint16_t)svpwm_str->Ta;
			break;
		default:
			break;
	}
	svpwm_str->PWMA = Limit(svpwm_str->PWMA,1,svpwm_str->Ts-50);
	svpwm_str->PWMB = Limit(svpwm_str->PWMB,1,svpwm_str->Ts-50);
	svpwm_str->PWMC = Limit(svpwm_str->PWMC,1,svpwm_str->Ts-50);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,svpwm_str->PWMA);   //Duty=set/svpwm_str->Ts
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,svpwm_str->PWMB);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,svpwm_str->PWMC);
	
}


//零序注入SVPWM波
void SVPWM_Zero(float Uq,float Ud,float elect_angle,SVPWM_Struct *svpwm_str)
{
	float Ualpha,Ubeta;
	
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET); 
	
//	arm_sin_cos_f32(elect_angle,&sin_dsp,&cos_dsp);        //DSP库计算三角，在电流环帕克变换处计算一次即可
//	Calculate_Float_Sin_Cos(elect_angle,&svpwm_str->sin_dsp,&svpwm_str->cos_dsp);  //Cordic库计算三角
	//反帕克变换
	arm_inv_park_f32(Ud,Uq,&Ualpha,&Ubeta,svpwm_str->sin_dsp,svpwm_str->cos_dsp);
	
	//归一化
	Ualpha /= svpwm_str->Udc;
	Ubeta /= svpwm_str->Udc;
	
	
	//SPWM电压
	float Ua = Ualpha;
	float Ub = -0.5f*Ualpha+0.8660254f*Ubeta;
	float Uc = -0.5f*Ualpha-0.8660254f*Ubeta;
	
	//零序分量计算 
	float UolMax = (Ua > Ub) ? ((Ua > Uc) ? Ua : Uc) : (Ub > Uc) ? Ub : Uc;
	float UolMin = (Ua < Ub) ? ((Ua < Uc) ? Ua : Uc) : (Ub < Uc) ? Ub : Uc;
	float UolZero = -0.5f*(UolMax + UolMin);
	
	//占空比计算
	svpwm_str->Ta = 2453.7375f*(Ua + UolZero) + 2125.0f;
	svpwm_str->Tb = 2453.7375f*(Ub + UolZero) + 2125.0f;
	svpwm_str->Tc = 2453.7375f*(Uc + UolZero) + 2125.0f;
	
	
	
	TIM1->CCR1 = (uint32_t)svpwm_str->Tc; //Duty=set/svpwm_str->Ts
	TIM1->CCR2 = (uint32_t)svpwm_str->Tb;
	TIM1->CCR3 = (uint32_t)svpwm_str->Ta;
	
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
}



//零序注入SVPWM波
void SVPWM_Zero1(float *Uq, float *Ud, float *angle_el)
{
	// 变量定义
	float m;						// 调制比
	float Ualpha,Ubeta;				// 静止坐标系电压
	float Valpha_ST,Vbeta_ST;		// 极坐标系电压
	float sin_result, cos_result;	// DSP正余弦计算结果
	
	// 电角度标准化[0, 2PI]  floorf(*angle_el * 0.159155f) = angle_el/2PI,向下取整
	float angle_el_mod = *angle_el - floorf(*angle_el * 0.159155f) * _2PI;  
	*angle_el = angle_el_mod >= 0 ? angle_el_mod : (angle_el_mod + _2PI);
	
	// 电角度标准化[-PI, PI]
	if (*angle_el > 3.141592f) *angle_el -= 6.283185f;
	else if (*angle_el < -3.141592f) *angle_el += 6.283185f;
	
	// 帕克逆变换（dq -> αβ）
	Calculate_Float_Sin_Cos( *angle_el, &sin_result, &cos_result);
	arm_inv_park_f32( *Ud, *Uq, &Ualpha, &Ubeta, sin_result, cos_result);

	// 极坐标变换（计算调制比）
	arm_sqrt_f32(Ualpha * Ualpha + Ubeta * Ubeta, &m);	
	arm_inv_park_f32( 0.0f, m/MOTOR_VDD, &Vbeta_ST, &Valpha_ST, sin_result, cos_result);

	// 电压参考值计算
	float Vref1 = Valpha_ST;	
	float Vref2 = - 0.5f * Valpha_ST + 0.866025f * Vbeta_ST;
	float Vref3 = - 0.5f * Valpha_ST - 0.866025f * Vbeta_ST;
	
	// 零序分量计算
    float volMax = (Vref1 > Vref2) ? ((Vref1 > Vref3) ? Vref1 : Vref3) : ((Vref2 > Vref3) ? Vref2 : Vref3);
    float volMin = (Vref1 < Vref2) ? ((Vref1 < Vref3) ? Vref1 : Vref3) : ((Vref2 < Vref3) ? Vref2 : Vref3);
    float volZero = -0.5f * (volMax + volMin);

	// 三相PWM占空比计算(已预乘 TIM_ARR:8500)
    float Ta = 4907.475f * (Vref1 + volZero) + 4250;
    float Tb = 4907.475f * (Vref2 + volZero) + 4250;
    float Tc = 4907.475f * (Vref3 + volZero) + 4250;
	
	// 三相PWM占空比赋值
	TIM1->CCR1 = (uint32_t)Ta;
	TIM1->CCR2 = (uint32_t)Tb;
	TIM1->CCR3 = (uint32_t)Tc;
}
