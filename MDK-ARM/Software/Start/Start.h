#ifndef __Start_H
#define __Start_H		


//头文件包含
#include "ALL_H.h"


//UART1
extern uint8_t rx_buffer[100];//接收数组
extern uint8_t rx_len; //接收到的数据长度



extern uint16_t raw;

void Enable_IT(void);

#endif
