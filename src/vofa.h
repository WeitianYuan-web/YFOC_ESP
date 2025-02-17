#include <Arduino.h>


#ifndef VOFA_H
#define VOFA_H

// 修改为实际使用的通道数
#define VOFA_CH_COUNT 10

void vofa(float data[VOFA_CH_COUNT]);

#endif