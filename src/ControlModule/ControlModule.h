#pragma once
#include <Arduino.h>
#include "Driver/Driver.h"
#include "freertos/semphr.h"

#define Motor_pole_pairs 7

// 前向声明
struct MotorParameters;
struct PhaseVoltage;

// 控制参数结构体
struct ControlParams {
    float zero_electric_angle;  // 零电角
    float target_pos;  // 位置环目标值（rad）
    float target_vel;  // 速度环目标值（rad/s）
    float target_cur;  // 电流环目标值（A）
    float max_current; // 最大电流限制（A）
    float pos_vel_limit; // 位置环速度限制（rad/s）
    float voltage_limit = Motor_max_voltage; // 电压限制（V）
};

// 核心控制模块结构体
struct ControlModule {
    ControlModule(int mot_num = 0) : encoder(mot_num) {} // 构造函数

    // 模式枚举
    enum Mode { 
        POSITION = 1,
        VELOCITY = 2,
        OPEN_LOOP = 3,
        CURRENT = 4,
        CURRENT_POSITION = 5,
        CURRENT_VELOCITY = 6,
        CASCADE_POS_VEL_CUR = 7
    } current_mode = POSITION;
    
    // 传感器成员
    Sensor_AS5600 encoder;
    CurrSense* current_sensor;
    
    // 新增校准时使用的零电角存储
    float zero_electric_angle = 0.0f;

    // 初始化传感器
    void initSensors(TwoWire* encoder_wire = &Wire, 
                     uint8_t encoder_addr = 0x36,
                     int current_pinA = 39,
                     int current_pinB = 36,
                     int current_pinC = -1);
    
    // 控制接口（全部改为非静态成员函数）
    bool set_control_mode(Mode mode, ControlParams params);
    float positionClosedLoop(float target_pos, float dt);
    float velocityClosedLoop(float target_vel, float dt);
    float openLoopControl(float target_velocity);
    float currentClosedLoop(float target_current, float dt, float electrical_angle, float Kp, float Ki);
    float PositionCurrentClosedLoop(float target_position, float dt);
    float VelocityCurrentClosedLoop(float target_velocity, float dt);
    float PositionVelocityCurrentLoop(float target_position, float dt);
    void calibrateZeroPoint();
    float getElectricalAngle();
    float getNormalizedElectricalAngle();
    void setPhaseVoltage(float Uq, float Ud, float angle_el);
    void updateSensorData();

    // 内部数据（传感器、电流、目标、PID、滤波、限流等）
    struct SensorData {
        float position;
        float velocity;
        float mech_angle;
    } sensor;
    
    struct CurrentData {
        float Ia;
        float Ib;
        float Ic;
        float Iq;
        float Iq_filtered;
    } current;
    
    struct ControlTarget {
        float position;
        float openloop_velocity;
        float velocity;
        float current;
    } target;

    struct PIDParameters {
        struct Basic {
            struct Position {
                float Kp = 10.0f;
                float Ki = 0.001f;
                float Kd = 0.001f;
                float integral = 0;
                float prev_error = 0;
            } position;
            struct {
                float Kp = 0.65f;
                float Ki = 0.1f;
                float Kd = 0.0005f;
                float integral = 0;
                float prev_error = 0;
            } velocity;
            struct {
                float Kp = 5.0f;
                float Ki = 500.0f;
                float Kd = 0.0f;
                float integral = 0;
                float prev_error = 0;
            } current;
        } basic;
        struct Advanced {
            struct PositionCurrent {
                float pos_Kp = 2.0f;
                float pos_Ki = 0.002f;
                float pos_Kd = 0.02f;
                float pos_prev_error = 0;
                float cur_Kp = 3.0f;
                float cur_Ki = 500.0f;
                float pos_integral = 0;
            } position_current;
            struct VelocityCurrent {
                float vel_Kp = 0.1f;
                float vel_Ki = 0.1f;
                float vel_Kd = 0.0002f;
                float vel_prev_error = 0;
                float cur_Kp = 4.0f;
                float cur_Ki = 300.0f;
                float vel_integral = 0;
            } velocity_current;
            struct Cascade {
                float pos_Kp = 5.0f;
                float pos_Ki = 0.001f;
                float pos_Kd = 0.0015f;
                float vel_Kp = 0.1f;
                float vel_Ki = 0.01f;
                float vel_Kd = 0.00015f;
                float cur_Kp = 5.0f;
                float cur_Ki = 500.0f;
                float pos_integral = 0;
                float vel_integral = 0;
                float pos_prev_error = 0;
                float vel_prev_error = 0;
                float cur_integral = 0;
            } cascade;
        } advanced;
    } pid;
    
    struct FilterParams {
        float k_ff = 0.09f;
        float k_ff_velocity_current = 0.01f;
        float k_ff_cascade = 0.4f;
        float alpha = 0.2f;
    } filter;

    struct CurrentLimits {
        float max_current = 0.9f;
        float min_current = 0.001f;
        struct {
            float pos_vel_limit = Motor_max_voltage;
        } cascade_limits;
    } limits;

private:
    // 将使用 this 的辅助函数声明为非静态成员函数
    float applyCurrentLimit(float target);
    bool isCurrentValid(float current);
};