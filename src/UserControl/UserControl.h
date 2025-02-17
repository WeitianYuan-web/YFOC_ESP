#pragma once
#include "ControlModule/ControlModule.h"

/* 新增结构体，用于返回电机完整数据 */
struct MotorData {
    float position;
    float velocity;
    float mech_angle;
    float current_Iq;
};

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

class UserControl {
public:
    // 构造函数：初始化 motor 编号及 last_us 时间戳
    UserControl() : motor(0), last_us(micros()) { }
    explicit UserControl(int mot_num) : motor(mot_num), last_us(micros()) { }
    
    // 初始化封装：初始化传感器、校准零点，并初始化时间戳
    void begin(int mot_num = 0,
               TwoWire* encoder_wire = &Wire,
               uint8_t encoder_addr = 0x36,
               int current_pinA = 39,
               int current_pinB = 36,
               int current_pinC = -1,
               int pinA = 14,
               int pinB = 27,
               int pinC = 26,
               int channelA = 0,
               int channelB = 1,
               int channelC = 2) {
        // 根据传入电机编号构造控制器对象
        motor = ControlModule(mot_num);
        // 初始化传感器：传入编码器接口、地址以及电流检测引脚
        motor.initSensors(encoder_wire, encoder_addr, current_pinA, current_pinB, current_pinC);
        // 初始化硬件：传入 PWM 引脚和 LEDC 通道参数
        motor.hardwareInit(pinA, pinB, pinC, channelA, channelB, channelC);
        // 校准零点
        motor.calibrateZeroPoint();
        // 初始化时间戳，供 dt 计算使用
        last_us = micros();
    }
    
    // 位置控制模式
    void positionMode(float target_pos) {   
        uint32_t dt_us = updateDt();
        if (dt_us == 0) dt_us = 1;  // 零值保护
        float dt = dt_us * 1e-6f;
        ControlParams params;
        params.target_pos = target_pos;
        motor.set_control_mode(ControlModule::POSITION, params);
        float Uq = motor.positionClosedLoop(target_pos, dt);
        motor.setPhaseVoltage(Uq, 0, motor.getElectricalAngle());
    }
    
    // 速度控制模式
    void velocityMode(float target_rad_s) {
        uint32_t dt_us = updateDt();
        if (dt_us == 0) dt_us = 1;
        float dt = dt_us * 1e-6f;
        ControlParams params;
        params.target_vel = target_rad_s;
        motor.set_control_mode(ControlModule::VELOCITY, params);
        float Uq = motor.velocityClosedLoop(target_rad_s, dt);
        motor.setPhaseVoltage(Uq, 0, motor.getElectricalAngle());
    }
    
    // 电流控制模式
    void currentMode(float target_amps) {
        uint32_t dt_us = updateDt();
        if (dt_us == 0) dt_us = 1;
        float dt = dt_us * 1e-6f;
        ControlParams params;
        params.target_cur = target_amps;
        motor.set_control_mode(ControlModule::CURRENT, params);
        float Uq = motor.currentClosedLoop(target_amps, dt, motor.getElectricalAngle(),
                                             motor.pid.basic.current.Kp,
                                             motor.pid.basic.current.Ki);
        motor.setPhaseVoltage(Uq, 0, motor.getElectricalAngle());
    }
    
    // 级联控制模式
    void cascadePosition(float target_pos, float max_vel, float max_current) {
        uint32_t dt_us = updateDt();
        if (dt_us == 0) dt_us = 1;
        float dt = dt_us * 1e-6f;
        ControlParams params;
        params.target_pos = target_pos;
        params.pos_vel_limit = max_vel;
        params.max_current = max_current;
        motor.set_control_mode(ControlModule::CASCADE_POS_VEL_CUR, params);
        float Uq = motor.PositionVelocityCurrentLoop(target_pos, dt);
        motor.setPhaseVoltage(Uq, 0, motor.getElectricalAngle());
    }
    
    // 更新控制模式，可支持多种模式
    void update(ControlModule::Mode mode, float target_pos, float target_vel, float target_cur, float max_vel, float max_current) {
        motor.updateSensorData();
        ControlParams params;
        params.target_pos = target_pos;
        params.target_vel = target_vel;
        params.target_cur = target_cur;
        params.pos_vel_limit = max_vel;
        params.max_current = max_current;
        motor.set_control_mode(mode, params);
        float Uq = calculateOutput(mode, target_pos, target_vel, target_cur);
        applyVoltage(Uq);
    }
    
    // 数据获取接口：获取电机传感器数据（位置、速度、机械角度）以及电流 Iq 数据
    MotorData getSensorData() {
        motor.updateSensorData();
        MotorData data;
        data.position = motor.sensor.position;
        data.velocity = motor.sensor.velocity;
        data.mech_angle = motor.sensor.mech_angle;
        data.current_Iq = motor.current.Iq;  // 返回电流 Iq 数据
        return data;
    }
    
    // 单独提供获取当前位置、速度、电流的接口
    float currentPosition() { return motor.sensor.position; }
    float currentVelocity() { return motor.sensor.velocity; }
    float currentCurrent() { return motor.current.Iq; }
    
private:
    ControlModule motor;    // 作为实例成员，支持多个控制器
    uint32_t last_us;       // 上一次更新时间戳
    
    // 根据模式计算控制输出
    float calculateOutput(ControlModule::Mode mode, float target_pos, float target_vel, float target_cur) {
        uint32_t dt_us = updateDt();
        if (dt_us == 0) dt_us = 1;
        float dt = dt_us * 1e-6f;
        switch (mode) {
            case ControlModule::POSITION:
                return motor.positionClosedLoop(target_pos, dt);
            case ControlModule::VELOCITY:
                return motor.velocityClosedLoop(target_vel, dt);
            case ControlModule::OPEN_LOOP:
                return motor.openLoopControl(target_vel);
            case ControlModule::CURRENT:
                return motor.currentClosedLoop(target_cur, dt, motor.sensor.mech_angle,
                                               motor.pid.basic.current.Kp,
                                               motor.pid.basic.current.Ki);
            case ControlModule::CURRENT_POSITION:
                return motor.PositionCurrentClosedLoop(target_pos, dt);
            case ControlModule::CURRENT_VELOCITY:
                return motor.VelocityCurrentClosedLoop(target_vel, dt);
            case ControlModule::CASCADE_POS_VEL_CUR:
                return motor.PositionVelocityCurrentLoop(target_pos, dt);
            default:
                return 0;
        }
    }
    
    // 应用电压
    void applyVoltage(float Uq) {
        motor.setPhaseVoltage(Uq, 0, motor.sensor.mech_angle);
    }
    
    // 更新 dt，并返回 dt（微秒），采用简单保护防止 dt 为 0
    uint32_t updateDt() {
        uint32_t now_us = micros();
        uint32_t dt_us = now_us - last_us;
        last_us = now_us;
        return dt_us;
    }
}; 