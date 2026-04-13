#include "PID.h"

extern PID pid_m1;                     //PID参数结构体
extern Encoder_Struct encoder_str;     //编码器结构体




void PID_I_Control(PID *pid)           //电流环控制(PI控制器)
{
	/*
		BEMF 电压 E = ω·Ke
		电机额定 3000 rpm
		KV = 170 rpm/V（机械）
		pole = 7
		Ke_rms = 1 / (KV × √3 × √2) × 60/(2π)
			   = 1 / (170 × 1.732 × 1.414) × 9.549
               ≈ 0.00229 V·s/rad（相电压有效值）

		Ke_peak = Ke_rms × √2
                = 0.00229 × 1.414
                ≈ 0.00324 V·s/rad
	*/
//	float omega_e = pid->Speed_now*0.733038f;   //机械->电  pid->Speed_now*7*2.0f*PI/60.0f，变成rad/s  0.733038
//	float BEMF = omega_e*0.00324f;                      //峰值
	
	//电流环PI控制
	//Iq
	pid->Iq_aim = Limit(pid->Iq_aim,-pid->Iqd_MAX,pid->Iqd_MAX);   //输入电流Iq，Id限幅
	pid->Id_aim = Limit(pid->Id_aim,-pid->Iqd_MAX,pid->Iqd_MAX);
	
	pid->erro_iq = pid->Iq_aim-pid->Iq_current;                                //电流iq误差
	if (pid->Uq >= pid->Ki_SumMax && pid->erro_iq > 0) {}           // 停止积分
    else if (pid->Uq <= -pid->Ki_SumMax && pid->erro_iq < 0) {}     // 停止积分
	else pid->erro_iq_sum = pid->erro_iq_sum+pid->Ki_i*pid->erro_iq;               //积分累加
	pid->erro_iq_sum = Limit(pid->erro_iq_sum,-pid->Ki_SumMax,pid->Ki_SumMax); //积分限幅
	pid->Uq = pid->Kp_i*pid->erro_iq+pid->erro_iq_sum;                        //电压Uq计算
	pid->Uq = Limit(pid->Uq,-pid->Ki_SumMax,pid->Ki_SumMax);                   //电压Uq限幅
//	pid->Uq += BEMF;                                                           //补偿
	//Id
	pid->erro_id = pid->Id_aim-pid->Id_current;                                //电流id误差
	if (pid->Ud >= pid->Ki_SumMax && pid->erro_id > 0) {}           // 停止积分
    else if (pid->Ud <= -pid->Ki_SumMax && pid->erro_id < 0) {}     // 停止积分
	else pid->erro_id_sum = pid->erro_id_sum+pid->Ki_i*pid->erro_id;               //积分累加
	pid->erro_id_sum = Limit(pid->erro_id_sum,-pid->Ki_SumMax,pid->Ki_SumMax); //积分限幅
	pid->Ud = pid->Kp_i*pid->erro_id+pid->erro_id_sum;                        //电压Ud计算
	pid->Ud = Limit(pid->Ud,-pid->Ki_SumMax,pid->Ki_SumMax);                   //电压Ud限幅	
}


void PID_Current_Control(PID *pid) //电流环控制(PI控制器)
{
	
}

void PID_Speed_Control(PID *pid,Encoder_Struct *encoder_str)
{

	

	
	if(pid->speed_count++>9)
	{
		encoder_str->Encoder_raw_erro = (encoder_str->Encoder_raw-encoder_str->Encoder_old_raw);
		//16384/2，判断正反转过零点
		if(encoder_str->Encoder_raw_erro>8192)          //反转过零点
			encoder_str->Encoder_raw_erro = encoder_str->Encoder_raw-encoder_str->Encoder_old_raw-16384;
		else if(encoder_str->Encoder_raw_erro<-8192)    //正转过零点  
			encoder_str->Encoder_raw_erro = 16384-encoder_str->Encoder_old_raw+encoder_str->Encoder_raw;
		
		encoder_str->Encoder_raw_sum += (float)encoder_str->Encoder_raw_erro;
		
		encoder_str->Encoder_old_raw = encoder_str->Encoder_raw;               //保留上一次编码器数值
		
		pid->Speed_now = encoder_str->Encoder_raw_sum*7.3242187f;  //(Encoder_raw_sum/16384)*(2000*60) 为转/分
		encoder_str->Encoder_raw_sum = 0;
		pid->speed_count = 0;
		
		pid->Speed_aim = Limit(pid->Speed_aim,-pid->speed_max,pid->speed_max);   //输入速度限幅
		pid->erro_speed = pid->Speed_aim-pid->Speed_now;
		 /*保存上一次积分值，用于抗饱和回退 */
        float erro_sum_last = pid->erro_speed_sum;
		
		pid->erro_speed_sum = pid->erro_speed_sum+pid->Ki_speed*pid->erro_speed;
		pid->erro_speed_sum = Limit(pid->erro_speed_sum,-pid->speed_out_max,pid->speed_out_max); //积分限幅
		
		pid->speed_out = pid->Kp_speed*pid->erro_speed+pid->erro_speed_sum;
		pid->speed_out =Limit(pid->speed_out,-pid->speed_out_max,pid->speed_out_max);            //输出Iq限幅
		
        /* 抗积分饱和：若输出被限幅，则撤销本次积分累加 */
        if (pid->speed_out <= -pid->speed_out_max ||
            pid->speed_out >=  pid->speed_out_max)
        {
            pid->erro_speed_sum = erro_sum_last;   // 退回上一次积分
        }
	}
	

//		//速度PI环
//		pid->Speed_aim = Limit(pid->Speed_aim,-pid->speed_max,pid->speed_max);   //输入速度限幅
//		pid->erro_speed = pid->Speed_aim-pid->Speed_now;
//		pid->erro_speed_sum = pid->erro_speed_sum+pid->Ki_speed*pid->erro_speed;
//		pid->erro_speed_sum = Limit(pid->erro_speed_sum,-pid->speed_out_max,pid->speed_out_max); //积分限幅
//		pid->speed_out = pid->Kp_speed*pid->erro_speed+pid->erro_speed_sum;
//		pid->speed_out =Limit(pid->speed_out,-pid->speed_out_max,pid->speed_out_max);            //输出Iq限幅

//	pid->Speed_show = alpha*pid->Speed_now+(1-alpha)*pid->Speed_last;
//	pid->Speed_last = pid->Speed_show;
//	encoder_str->Encoder_raw_sum = 0;

	//速度环执行周期为电流环的5~10倍,我这里10倍，即频率为1K
//	if(pid->speed_count++>9)
//	{
//		pid->Speed_now = (float)encoder_str->Encoder_raw_sum*3.662109f;  //(Encoder_raw_sum/16384)*(1000*60) 为转/分
//		pid->Speed_show = alpha*pid->Speed_now+(1-alpha)*pid->Speed_last;
//		pid->Speed_last = pid->Speed_show;
//		encoder_str->Encoder_raw_sum = 0;
//		pid->speed_count = 0;
//		//速度PI环
//		pid->Speed_aim = Limit(pid->Speed_aim,-pid->speed_max,pid->speed_max);   //输入速度限幅
//		pid->erro_speed = pid->Speed_aim-pid->Speed_now;
//		pid->erro_speed_sum = pid->erro_speed_sum+pid->Ki_speed*pid->erro_speed;
//		pid->erro_speed_sum = Limit(pid->erro_speed_sum,-pid->speed_out_max,pid->speed_out_max); //积分限幅
//		pid->speed_out = pid->Kp_speed*pid->erro_speed+pid->erro_speed_sum;
//		pid->speed_out =Limit(pid->speed_out,-pid->speed_out_max,pid->speed_out_max);            //输出Iq限幅
//	}
}

void PID_Position_Control(PID *pid,Encoder_Struct *encoder_str)
{
	if(pid->position_count++>9)
	{
		
		// 将电机轴角度转换为输出轴角度
        float current_output_angle = encoder_str->output_shaft_angle;
		
		//角度限制 0-360°
		pid->Position_aim = fmod(pid->Position_aim,360);
		if(pid->Position_aim<0) pid->Position_aim+=360;
		
		//最短圆弧误差（±180°）
		pid->erro_positon = pid->Position_aim-current_output_angle;                                //角度误差
		if (pid->erro_positon >  180.0f) pid->erro_positon -= 360.0f;
        else if (pid->erro_positon < -180.0f) pid->erro_positon += 360.0f;
		
		pid->erro_position_sum = pid->erro_position_sum+pid->Ki_position*pid->erro_positon;               //积分累加
		pid->erro_position_sum = Limit(pid->erro_position_sum,-pid->position_out_max,pid->position_out_max); //积分限幅
		//计算输出
		pid->position_out = pid->Kp_position*pid->erro_positon+pid->erro_position_sum;                        //输出速度计算
		pid->position_out = Limit(pid->position_out,-pid->position_out_max,pid->position_out_max);                   //输出速度限幅	
	}
}


//速度规划器初始化
void Planner_Speed_Init(Simple_Position_Speed_Planner * planner,float target_speed,float max_accel,float current_speed)
{
	planner->target_speed = target_speed;              //设定速度
	planner->max_accel_speed = max_accel;              //最大加速度
	planner->current_plan_speed = current_speed;       //规划器初始速度
	planner->plan_speed_isactive = 1;                  //规划器启动
}

//速度规划器更新
float Planner_Speed_Update(Simple_Position_Speed_Planner * planner,float dt)
{
	if(!planner->plan_speed_isactive) return 0.0f;
	
	//计算剩余速度差值
	float remaining_speed = planner->target_speed - planner->current_plan_speed;
	float abs_remain_speed = fabs(remaining_speed);
	int8_t direction =  (remaining_speed>0) ? 1 : -1;
	
	//计算加速或者减速所需时间
	
	if(abs_remain_speed>30)
		planner->current_plan_speed += direction*planner->max_accel_speed*dt;
	else
		planner->current_plan_speed = planner->target_speed;
	
	return planner->current_plan_speed;
}

//位置规划器初始化
void Planner_Position_Init(Simple_Position_Speed_Planner * planner,float target_angle,float max_accel,float current_speed,float curise_speed)
{
	planner->target_angle = target_angle;                //设定目标角度
	planner->curise_speed = curise_speed;                //设定巡航速度
	planner->current_plan_speed = current_speed;         //规划器初始速度
	planner->max_accel_speed = max_accel;                //最大加速度
	planner->plan_position_isactive = 1;                 //规划器启动
}


//位置规划器更新
float Planner_Position_Update(Simple_Position_Speed_Planner * planner,float current_angle,float dt)
{
	if(!planner->plan_position_isactive) return 0.0f;
	
	//计算剩余角度差值
	float remaining_angle = planner->target_angle - current_angle;
	if (remaining_angle >  180.0f) remaining_angle -= 360.0f;
	else if (remaining_angle < -180.0f) remaining_angle += 360.0f;
	
	float abs_remaining_angle = fabsf(remaining_angle);
	int8_t direction = (remaining_angle>0) ? 1 : -1;
	
	//计算减速所需时间距离
	float decel_time = fabs(planner->current_plan_speed) / planner->max_accel_speed;
	float decel_distance = 0.5f*planner->max_accel_speed*decel_time*decel_time*(60/360);
	
	//运动速度设定
	if(abs_remaining_angle>decel_distance+0.5f)
	{	//加速段和匀速段
		if(fabs(planner->current_plan_speed)<fabs(planner->curise_speed)-30)
			planner->current_plan_speed += direction*planner->max_accel_speed*dt;
		else
			planner->current_plan_speed = direction*fabsf(planner->curise_speed);		
	}
	else
	{
		if(abs_remaining_angle>2.0f)
			planner->current_plan_speed -= direction*sqrt(2.0f*planner->max_accel_speed*abs_remaining_angle*(60/360));
		else
			planner->current_plan_speed = 0.0f;
	}
		
	return planner->current_plan_speed;
}
