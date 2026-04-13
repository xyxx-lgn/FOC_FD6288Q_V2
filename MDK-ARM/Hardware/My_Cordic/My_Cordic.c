#include "my_cordic.h"

CORDIC_ConfigTypeDef sCordicConfig;

void cordic_config(void)
{
	sCordicConfig.Function         = CORDIC_FUNCTION_SINE;     /* sine function */
  	sCordicConfig.Precision        = CORDIC_PRECISION_6CYCLES; /* max precision for q1.31 sine */
 	sCordicConfig.Scale            = CORDIC_SCALE_0;           /* no scale */
 	sCordicConfig.NbWrite          = CORDIC_NBWRITE_1;         /* One input data: angle. Second input data (modulus) is 1 after cordic reset */
 	sCordicConfig.NbRead           = CORDIC_NBREAD_2;          /* One output data: sine cosine */
 	sCordicConfig.InSize           = CORDIC_INSIZE_32BITS;     /* q1.31 format for input data */
  	sCordicConfig.OutSize          = CORDIC_OUTSIZE_32BITS;    /* q1.31 format for output data */
	
	HAL_CORDIC_Configure(&hcordic, &sCordicConfig);
}

void Calculate_Float_Sin(float angle,float *sin)
{
	/* Q31,two write, one read, sine calculate, 6 precision */
	CORDIC->CSR = 0x00100061;
	/* Write data into WDATA */
	CORDIC->WDATA = (int32_t )(angle * RADIAN_Q31_f);
	/* Modulus is m=1 */
	CORDIC->WDATA = 0x7FFFFFFF;
	/* Get sin value in float */
	*sin = ((int32_t)CORDIC->RDATA)*1.0f/Q31;
}

void Calculate_Float_Sin_Cos(float angle,float *sin, float *cos)     //angle->[-PI,PI]
{
	/* Q31,two write, two read, sine calculate, 6 precision */
	CORDIC->CSR = 0x00180061;
	/* Write data into WDATA */
	CORDIC->WDATA = (int32_t )(angle * RADIAN_Q31_f);
	/* Modulus is m=1 */
	CORDIC->WDATA = 0x7FFFFFFF;
	/* Get sin value in float */
	*sin = ((int32_t)CORDIC->RDATA)*1.0f/Q31;
	/* Get cos value in float */
	*cos = ((int32_t)CORDIC->RDATA)*1.0f/Q31;
}
