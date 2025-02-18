#pragma once
#include <Arduino.h>
#include "InlineCurrent.h"
#include "AS5600.h"
#include <math.h>
#include "freertos/semphr.h" // 加入 FreeRTOS 互斥量支持


// 电机参数宏定义
#define Motor_max_velocity 60.0f    //rad/s
#define Motor_max_current 0.9f      //A
#define Motor_resistance 16.0f      // 电机相电阻
#define Motor_max_voltage 12.6f     //V
#define Motor_KV 10.0f              //KV值
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define DIRECTION_FORWARD -1

// 电机参数结构体
struct MotorParameters {
    float voltage_power_supply = Motor_max_voltage;
    float voltage_limit = 12.6f;
    float shaft_angle = 0.0f;
    float open_loop_timestamp = 0.0f;
    float zero_electric_angle = 0.0f;
    float electrical_angle = 0.0f;
    int direction = DIRECTION_FORWARD;
    float open_loop_voltage_ratio = 0.7f;
    float max_openloop_speed = 10.0f;
};

// 相电压结构体
struct PhaseVoltage {
    float Ualpha = 0.0f;
    float Ubeta  = 0.0f;
    float Ua = 0.0f;
    float Ub = 0.0f;
    float Uc = 0.0f;
    float dc_a = 0.0f;
    float dc_b = 0.0f;
    float dc_c = 0.0f;
};

#define NOT_SET -1

// 修改后的传感器管理类，增加双缓冲缓存实现两阶段更新
class SensorManager {
public:
    SensorManager() : as5600(0), current_sense(nullptr) {
        sensorMutex = xSemaphoreCreateRecursiveMutex();
    }
    
    // 初始化传感器（包括I2C初始化、编码器和电流检测传感器）
    void init(int pinA, int pinB, int pinC = NOT_SET,
              TwoWire* encoder_wire = &Wire,
              uint8_t encoder_addr = 0x36,
              int i2cSda = 21,
              int i2cScl = 22,
              uint32_t clock = 400000) {
        encoder_wire->begin(i2cSda, i2cScl, clock);
        as5600.Sensor_init(encoder_wire, encoder_addr);
        
        current_sense = new CurrSense(pinA, pinB, pinC);
        current_sense->init();
    }
    
    // 更新所有传感器数据，采用两阶段更新方案
    void update() {
        // 第一阶段：无锁调用硬件接口更新数据（耗时操作，不宜持锁）
        as5600.Sensor_update();
        current_sense->getPhaseCurrents();
        
        // 第二阶段：加锁后快速将数据复制到内部缓存中
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        sensorCache.position = as5600.getAngle();
        sensorCache.velocity = as5600.getVelocity();
        sensorCache.mech_angle = as5600.getMechanicalAngle();
        
        currentCache.current_a = current_sense->current_a;
        currentCache.current_b = current_sense->current_b;
        currentCache.current_c = current_sense->current_c;
        xSemaphoreGiveRecursive(sensorMutex);
    }
    
    // 以下接口在短临界区内快速获取已缓存的数据
    float getAngle() {
        float angle;
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        angle = sensorCache.position;
        xSemaphoreGiveRecursive(sensorMutex);
        return angle;
    }
    
    float getVelocity() {
        float vel;
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        vel = sensorCache.velocity;
        xSemaphoreGiveRecursive(sensorMutex);
        return vel;
    }
    
    float getMechanicalAngle() {
        float angle;
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        angle = sensorCache.mech_angle;
        xSemaphoreGiveRecursive(sensorMutex);
        return angle;
    }
    
    float getCurrentA() {
        float curr;
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        curr = currentCache.current_a;
        xSemaphoreGiveRecursive(sensorMutex);
        return curr;
    }
    
    float getCurrentB() {
        float curr;
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        curr = currentCache.current_b;
        xSemaphoreGiveRecursive(sensorMutex);
        return curr;
    }
    
    float getCurrentC() {
        float curr;
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        curr = currentCache.current_c;
        xSemaphoreGiveRecursive(sensorMutex);
        return curr;
    }
    
    Sensor_AS5600 as5600;      // 作为实例成员：角度传感器
    CurrSense* current_sense;  // 电流传感器指针

private:
    SemaphoreHandle_t sensorMutex; // 用于数据缓存的保护

    // 内部缓存结构，存储从传感器获取到的数据
    struct SensorCacheData {
        float position = 0;
        float velocity = 0;
        float mech_angle = 0;
    } sensorCache;
    
    struct CurrentCacheData {
        float current_a = 0;
        float current_b = 0;
        float current_c = 0;
    } currentCache;
};

// 修改 MotorDriver 为实例类
class MotorDriver {
public:
    MotorDriver() : ledcChannelA(0), ledcChannelB(0), ledcChannelC(0) {}
    // 将接口函数改为普通成员函数
    void hardwareInit(int pinA, int pinB, int pinC, 
                      int channelA, int channelB, int channelC);
    void setPwm(float Ua, float Ub, float Uc);
    void setPhaseVoltage(float Uq, float Ud, float angle_el);
    float velocityOpenloop(float target_velocity);
    float electricalAngle(float shaft_angle, int pole_pairs);
    float normalizeAngle(float angle);
    float computeIq(float Ia, float Ib, float electrical_angle);

    // 实例成员
    MotorParameters params;
    PhaseVoltage phase;
    SensorManager sensor;
    int ledcChannelA, ledcChannelB, ledcChannelC;

private:
    // 约束宏实现
    float _constrain(float amt, float low, float high) {
        return (amt < low) ? low : ((amt > high) ? high : amt);
    }
}; 