#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h> 

#define _2PI 6.28318530718f



// AS5600 相关
double Sensor_AS5600::getSensorAngle() {
  uint8_t angle_reg_msb = 0x0C;

  byte readArray[2];
  uint16_t readValue = 0;

  wire->beginTransmission(0x36);
  wire->write(angle_reg_msb);
  wire->endTransmission(false);


  wire->requestFrom(0x36, (size_t)2);
  for (byte i=0; i < 2; i++) {
    readArray[i] = wire->read();
  }
  int _bit_resolution=12;
  int _bits_used_msb=11-7;
  float cpr = pow(2, _bit_resolution);
  int lsb_used = _bit_resolution - _bits_used_msb;

  uint8_t lsb_mask = (uint8_t)((1 << lsb_used) - 1);
  uint8_t msb_mask = (uint8_t)((1 << _bits_used_msb) - 1);
  
  readValue = ( readArray[1] &  lsb_mask );
  readValue += ( ( readArray[0] & msb_mask ) << lsb_used );
  return (readValue/ (float)cpr) * _2PI; 

}

//AS5600 相关

//=========角度处理相关=============
Sensor_AS5600::Sensor_AS5600(int Mot_Num) {
   _Mot_Num=Mot_Num;  //使得 Mot_Num 可以统一在该文件调用
   
}
void Sensor_AS5600::Sensor_init(TwoWire* _wire) {
    wire=_wire;
    wire->begin();   //电机Sensor
    delay(50);
    getSensorAngle(); 
    delayMicroseconds(1);
    vel_angle_prev = getSensorAngle(); 
    vel_angle_prev_ts = micros();
    delay(1);
    getSensorAngle(); 
    delayMicroseconds(1);
    angle_prev = getSensorAngle(); 
    angle_prev_ts = micros();
}
void Sensor_AS5600::Sensor_update() {
    float val = getSensorAngle();
    angle_prev_ts = micros();
    float d_angle = val - angle_prev;
    // 圈数检测
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
}

float Sensor_AS5600::getMechanicalAngle() {
    return angle_prev;
}

float Sensor_AS5600::getAngle(){
    return (float)full_rotations * _2PI + angle_prev;
}

float Sensor_AS5600::getVelocity() {
    // 获取当前时间戳（确保每次调用都更新）
    uint32_t now_us = micros();
    
    // 计算时间差（秒），限制在0.1ms-0.1s之间
    float Ts = constrain((now_us - vel_angle_prev_ts) * 1e-6f, 0.0001f, 0.1f);
    
    // 计算总角度差（处理多圈旋转）
    int32_t rotation_diff = full_rotations - vel_full_rotations;
    float angle_diff = rotation_diff * _2PI + (angle_prev - vel_angle_prev);
    
    // 角度差合理性检查（防止传感器读数异常）
    if(fabs(angle_diff) > 10*_2PI){ // 最大允许10圈/秒（约600RPM）
        angle_diff = 0; // 视为无效数据
        Ts = 0.001f;    // 重置时间间隔
    }
    
    // 自动处理角度环绕（优化版）
    angle_diff = fmod(angle_diff + PI, _2PI) - PI;
    
    // 计算原始速度
    float raw_vel = angle_diff / Ts;
        // 在返回前更新调试参数
    debug_raw_vel = raw_vel;
    // 更新记录值（原子操作）
    noInterrupts();
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = now_us;
    interrupts();
    
    // 改进滤波：一阶低通滤波
    static float y1 = 0.0f;
    const float cutoff_freq = 10.0f; // 截止频率10Hz
    const float RC = 1.0f/(2*PI*cutoff_freq);
    const float alpha = Ts / (Ts + RC);
    

    y1 = (1 - alpha) * y1 + alpha * raw_vel;
    // 零速死区处理
    if(fabs(y1) < 0.02f) y1 = 0;
    
    return y1;
}
