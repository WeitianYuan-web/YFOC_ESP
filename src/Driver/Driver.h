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

// 修改后的传感器管理模块，去除static，转换为实例成员
class SensorManager {
public:
    SensorManager() : as5600(0), current_sense(nullptr) {
        sensorMutex = xSemaphoreCreateRecursiveMutex();
    }

    // 初始化函数，设置I2C总线、编码器和电流传感器
    void init(int pinA, int pinB, int pinC = NOT_SET,
              TwoWire* encoder_wire = &Wire,
              uint8_t encoder_addr = 0x36) {
        // I2C总线初始化（可更换默认引脚）
        encoder_wire->begin(21, 22, 400000);
        as5600.Sensor_init(encoder_wire, encoder_addr);
        
        // 初始化电流传感器
        current_sense = new CurrSense(pinA, pinB, pinC);
        current_sense->init();
    }

    // 示例：获取编码器角度（其余接口以类似方式改为非静态成员函数）
    float getAngle(uint8_t encoder_id = 0) { 
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        float val = as5600.getAngle();
        xSemaphoreGiveRecursive(sensorMutex);
        return val;
    }

    // 更新所有传感器数据（需周期性调用）
    void update() {
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        // 更新角度传感器
        as5600.Sensor_update();
        // 更新电流传感器
        current_sense->getPhaseCurrents();
        xSemaphoreGiveRecursive(sensorMutex);
    }

    float getMechanicalAngle() {
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        float val = as5600.getMechanicalAngle();
        xSemaphoreGiveRecursive(sensorMutex);
        return val;
    }
    float getVelocity() {
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        float val = as5600.getVelocity();
        xSemaphoreGiveRecursive(sensorMutex);
        return val;
    }

    // 电流获取接口
    float getCurrentA() { 
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        float val = current_sense->current_a; 
        xSemaphoreGiveRecursive(sensorMutex);
        return val;
    }
    float getCurrentB() { 
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        float val = current_sense->current_b; 
        xSemaphoreGiveRecursive(sensorMutex);
        return val;
    }
    float getCurrentC() { 
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        float val = current_sense->current_c; 
        xSemaphoreGiveRecursive(sensorMutex);
        return val;
    }
    Sensor_AS5600 as5600;         // 作为实例成员 
    CurrSense* current_sense;
private:

    SemaphoreHandle_t sensorMutex;
};

// 修改 MotorDriver 为实例类
class MotorDriver {
public:
    MotorDriver() {}
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

private:
    // 约束宏实现
    float _constrain(float amt, float low, float high) {
        return (amt < low) ? low : ((amt > high) ? high : amt);
    }
}; 