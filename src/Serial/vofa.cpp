#include <Arduino.h>
#include "vofa.h"

void vofa(float data[VOFA_CH_COUNT]) {
    // 发送数据
    Serial.write((char *)data, sizeof(float) * VOFA_CH_COUNT); 
    
    // 发送帧尾
    unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};
    Serial.write(tail, 4);
}