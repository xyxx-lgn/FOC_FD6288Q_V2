#ifndef __MT6701_H
#define __MT6701_H		


//头文件包含
#include "spi.h"
#include "ALL_H.h"


#define MT6701_CS_ON()      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define MT6701_CS_OFF()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
 
static void MT6701_Read(uint8_t *p);
uint16_t MT6701_ReadRaw(void);   //14位原始数据
#endif
