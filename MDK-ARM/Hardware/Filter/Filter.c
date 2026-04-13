#include "Filter.h"
#include "arm_math.h"
#include "stdlib.h"

/*********************** FIR ***********************/
/** 采样频率：20kHz 通带频率：1000Hz 阻带频率：2000Hz **/
#define FIR_LENGTH 1                         									/* FIR滤波器输入输出数据的长度 */
#define FIR_NUMTAPS_LENGTH 19                 									/* FIR滤波器的系数个数 */
#define FIR_PSTATE_LENGTH (FIR_LENGTH + FIR_NUMTAPS_LENGTH - 1) /* FIR滤波器状态变量的长度 */
 
extern uint16_t ADC1InjectDate[4];     //注入组采样数组
extern PID pid_m1;                     //PID参数结构体
 
arm_fir_instance_f32  fir_Ia,fir_Ib,fir_Ic,fir_Speed;												/* FIR实例化结构体 */

float32_t FIR_InputBufer[FIR_LENGTH] = {0};					/* 输入数据缓冲区，长度为 FIR_LENGTH */
float32_t FIR_OutputBufer[FIR_LENGTH] = {0};				/* 输出数据缓冲区，长度为 FIR_LENGTH */

uint16_t fir_numTaps = FIR_NUMTAPS_LENGTH;					/* FIR滤波器系数个数 */
uint32_t fir_blockSize = FIR_LENGTH;								/* 块处理大小 */

float32_t fir_pStateIa[FIR_PSTATE_LENGTH] = {0.0f};		/* FIR滤波器状态变量暂存数组：状态数组的大小为 fir_numTaps + fir_blockSize - 1 */
float32_t fir_pStateIb[FIR_PSTATE_LENGTH] = {0.0f};		/* FIR滤波器状态变量暂存数组：状态数组的大小为 fir_numTaps + fir_blockSize - 1 */
float32_t fir_pStateIc[FIR_PSTATE_LENGTH] = {0.0f};		/* FIR滤波器状态变量暂存数组：状态数组的大小为 fir_numTaps + fir_blockSize - 1 */
float32_t fir_pStateSpeed[FIR_PSTATE_LENGTH] = {0.0f};	/* FIR滤波器状态变量暂存数组：状态数组的大小为 fir_numTaps + fir_blockSize - 1 */

const float32_t fir_pCoeffs[FIR_NUMTAPS_LENGTH] = {	/* FIR滤波器系数数组，长度为 FIR_NUMTAPS_LENGTH */                                                      
  -0.00035615152f,  0.017694939f,  0.027213134f,  0.044123626f,
   0.064001396f,    0.085008130f,  0.104900170f,  0.121301759f,
   0.132117123f,    0.135889322f,  0.132117123f,  0.121301759f,
   0.104900170f,    0.085008130f,  0.064001396f,  0.044123626f,
   0.027213134f,    0.017694939f, -0.00035615152f
};


/* 上电调用一次即可 */
void FIR_DSP_Init(void)
{
    arm_fir_init_f32(&fir_Ia,  fir_numTaps, fir_pCoeffs, fir_pStateIa,  fir_blockSize);
//    arm_fir_init_f32(&fir_Ib,  fir_numTaps, fir_pCoeffs, fir_pStateIb,  fir_blockSize);
//    arm_fir_init_f32(&fir_Ic,  fir_numTaps, fir_pCoeffs, fir_pStateIc,  fir_blockSize);
//    arm_fir_init_f32(&fir_Speed, fir_numTaps, fir_pCoeffs, fir_pStateSpeed, fir_blockSize);
}

void Filter_DSP_Fir(float * ArrInput,float * ArrOutPut)
{
	//1.临时储存
    float f[4];                          // 临时缓冲
    f[0] = ArrInput[0];           // 先转成 float
//    f[1] = (float)ArrInput[1];
//    f[2] = (float)ArrInput[2];
//    f[3] = pid_m1.Speed_now;
    /* 2 直接滤波（blockSize=1） */
    arm_fir_f32(&fir_Ia,  &f[0],  &ArrOutPut[0],  1);
//    arm_fir_f32(&fir_Ib,  &f[1],  &ArrOutPut[1],  1);
//    arm_fir_f32(&fir_Ic,  &f[2],  &ArrOutPut[2],  1);
//    arm_fir_f32(&fir_Speed, &f[3],  &ArrOutPut[3], 1);

}

