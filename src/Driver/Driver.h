#pragma once
#include <Arduino.h>
#include "InlineCurrent.h"
#include "AS5600.h"
#include <math.h>

// PWM输出引脚定义
const int pwmA = 32;
const int pwmB = 33;
const int pwmC = 25;

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

// 新增传感器管理模块
class SensorManager {
public:
    // 修改初始化函数签名
    static void init(int pinA, int pinB, int pinC = NOT_SET, 
                    TwoWire* encoder_wire = &Wire, 
                    uint8_t encoder_addr = 0x36) {  // 新增编码器参数
        // 编码器初始化
        encoder_wire->begin(21, 22, 400000); // 保持默认引脚但允许更换总线
        as5600.Sensor_init(encoder_wire, encoder_addr);
        
        // 电流传感器初始化
        current_sense = new CurrSense(pinA, pinB, pinC);
        current_sense->init();
    }

    
    // 修改获取角度方法
    static float getAngle(uint8_t encoder_id = 0) { 
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        float val = as5600.getAngle();
        xSemaphoreGiveRecursive(sensorMutex);
        return val;
    }

    // 更新所有传感器数据（需周期性调用）
    static void update() {
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        // 更新角度传感器
        as5600.Sensor_update();
        // 更新电流传感器
        current_sense->getPhaseCurrents();
        xSemaphoreGiveRecursive(sensorMutex);
    }

    static float getMechanicalAngle() {
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        float val = as5600.getMechanicalAngle();
        xSemaphoreGiveRecursive(sensorMutex);
        return val;
    }
    static float getVelocity() {
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        float val = as5600.getVelocity();
        xSemaphoreGiveRecursive(sensorMutex);
        return val;
    }

    // 电流获取接口
    static float getCurrentA() { 
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        float val = current_sense->current_a; 
        xSemaphoreGiveRecursive(sensorMutex);
        return val;
    }
    static float getCurrentB() { 
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        float val = current_sense->current_b; 
        xSemaphoreGiveRecursive(sensorMutex);
        return val;
    }
    static float getCurrentC() { 
        xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
        float val = current_sense->current_c; 
        xSemaphoreGiveRecursive(sensorMutex);
        return val;
    }

private:
    static Sensor_AS5600 as5600;         // 主编码器
    static CurrSense* current_sense;
    static SemaphoreHandle_t sensorMutex;
};

// 在Driver类中集成传感器管理
class MotorDriver {
public:
    // 新增传感器访问接口
    static SensorManager sensor;
    
    // 初始化函数
    static void hardwareInit();
    
    // 基础驱动函数
    static void setPwm(float Ua, float Ub, float Uc);
    static void setPhaseVoltage(float Uq, float Ud, float angle_el);
    static float velocityOpenloop(float target_velocity);
    
    // 电角度计算
    static float electricalAngle(float shaft_angle, int pole_pairs);
    static float normalizeAngle(float angle);
    static float computeIq(float Ia, float Ib, float electrical_angle);
    
    // 全局实例
    static MotorParameters params;
    static PhaseVoltage phase;

private:
    // 约束宏实现
    static float _constrain(float amt, float low, float high) {
        return (amt < low) ? low : ((amt > high) ? high : amt);
    }
}; 