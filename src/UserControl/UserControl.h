#pragma once
#include "ControlModule/ControlModule.h"

/* 新增结构体，用于返回电机完整数据 */
struct MotorData {
    float position;
    float velocity;
    float mech_angle;
    float current_Iq;
    float electrical_angle;
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
               int current_pinC = 1,
               int pinA = 14,
               int pinB = 27,
               int pinC = 26,
               int direction = 1,
               int channelA = 0,
               int channelB = 1,
               int channelC = 2) {
        // 根据传入电机编号构造控制器对象
        motor = ControlModule(mot_num);
        // 初始化传感器：传入编码器接口、地址以及电流检测引脚
        // 初始化硬件：传入 PWM 引脚和 LEDC 通道参数
        motor.hardwareInit(pinA, pinB, pinC, direction, channelA, channelB, channelC);
        motor.initSensors(encoder_wire, encoder_addr, current_pinA, current_pinB, current_pinC);
        delay(100);
        // 校准零点
        motor.calibrateZeroPoint();
        // 初始化时间戳，供 dt 计算使用
        last_us = micros();
    }
    
    // 统一控制更新接口，更新传感器数据，并根据不同模式调用对应的闭环函数
    void update(ControlModule::Mode mode, float target_pos, float target_vel, float target_cur,
                float max_vel, float max_current,uint32_t dt, float ramp_factor = 1.0f) {

        // 只调用一次 dt 计算
        if (dt == 0) dt = 1;

        float Uq = 0;
        // 根据不同模式调用对应闭环控制函数
        switch (mode) {
            case ControlModule::POSITION:
                Uq = motor.positionClosedLoop(target_pos, dt);
                break;
            case ControlModule::VELOCITY:
                Uq = motor.velocityClosedLoop(target_vel, dt);
                break;
            case ControlModule::OPEN_LOOP:
                Uq = motor.openLoopControl(target_vel);
                break;
            case ControlModule::CURRENT:
                Uq = motor.currentClosedLoop(target_cur, dt, motor.getNormalizedElectricalAngle(),
                                             motor.pid.basic.current.Kp,
                                             motor.pid.basic.current.Ki);
                break;
            case ControlModule::CURRENT_POSITION:
                Uq = motor.PositionCurrentClosedLoop(target_pos, dt);
                break;
            case ControlModule::CURRENT_VELOCITY:
                Uq = motor.VelocityCurrentClosedLoop(target_vel, dt);
                break;
            case ControlModule::CASCADE_POS_VEL_CUR:
                Uq = motor.PositionVelocityCurrentLoop(target_pos, dt);
                break;
            case ControlModule::SINGLE_POSITION:
                Uq = motor.positionClosedLoop_single(target_pos, dt);
                break;
            default:
                Uq = 0;
                break;
        }
        applyVoltage(Uq * ramp_factor);
    }
    
    // 单独的模式控制接口，内部调用统一的 update() 接口
    void positionMode(float target_pos, uint32_t dt) {   
        update(ControlModule::POSITION, target_pos, 0, 0, 10.0f, 1.0f, dt);
    }
    
    void velocityMode(float target_rad_s, uint32_t dt) {
        update(ControlModule::VELOCITY, 0, target_rad_s, 0, 10.0f, 1.0f, dt);
    }
    
    void currentMode(float target_amps, uint32_t dt) {
        update(ControlModule::CURRENT, 0, 0, target_amps, 10.0f, 1.0f, dt);
    }
    void initPID (){
        motor.initPID();
    }
    // 数据获取接口：获取电机传感器数据（位置、速度、机械角度）以及电流 Iq 数据
    MotorData update_getSensorData() {
                // 更新传感器数据
        motor.updateSensorData();
        MotorData data;
        data.position = motor.sensor.position;
        data.velocity = motor.sensor.velocity;
        data.mech_angle = motor.sensor.mech_angle;
        data.current_Iq = motor.current.Iq;  // 返回电流 Iq 数据
        motor.electrical_angle = motor.getNormalizedElectricalAngle();
        data.electrical_angle = motor.electrical_angle;
        return data;
    }

    // 单独提供获取当前位置、速度、电流的接口
    float currentPosition() { return motor.sensor.position; }
    float currentVelocity() { return motor.sensor.velocity; }
    float currentMechanicalAngle() { return motor.sensor.mech_angle; }
    float currentIq() { return motor.current.Iq; }
    float currentElectricalAngle() { return motor.electrical_angle; }

        // 更新 dt（微秒）
    uint32_t updateDt() {
        uint32_t now_us = micros();
        uint32_t dt_us = now_us - last_us;
        last_us = now_us;
        return dt_us * 1e-6f;
    }
    
    
private:
    ControlModule motor;    // 作为实例成员，支持多个控制器
    uint32_t last_us;       // 上一次更新时间戳
    

    // 应用输出电压到相电压（这里调用 ControlModule 中的电机驱动接口）
    void applyVoltage(float Uq) {
        motor.setPhaseVoltage(Uq, 0, motor.electrical_angle);
    }
}; 