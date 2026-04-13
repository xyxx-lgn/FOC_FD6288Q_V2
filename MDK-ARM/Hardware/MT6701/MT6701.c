#include "MT6701.h"


static void MT6701_Read(uint8_t *p)
{
    MT6701_CS_ON();
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t[3]){0xFF,0xFF,0xFF}, p, 3, 100); // 0=不超时 ，超时时间过短会导致数据抖动异常
    MT6701_CS_OFF();
}


uint16_t MT6701_ReadRaw(void)   //14位原始数据
{
    uint8_t data[3];
    uint16_t angle_u16;

    MT6701_Read(data);
    angle_u16 = (uint16_t)(data[1] >> 2);    
    angle_u16 |= ((uint16_t)data[0] << 6);                //该操作后得到14位原始角度数据
	return angle_u16;
}

