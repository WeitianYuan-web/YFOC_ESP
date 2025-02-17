#pragma once
#include "ControlModule/ControlModule.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))


class UserControl {
public:
    // 初始化封装
    static void begin(int mot_num = 0) {
        motor = ControlModule(mot_num);
        motor.initSensors();
        motor.calibrateZeroPoint();
    }

    // 模式封装方法
    static void positionMode(float target_pos) {
        ControlParams params;
        params.target_pos = target_pos;
        ControlModule::set_control_mode(POSITION, params);
    }

    static void velocityMode(float target_rad_s, float max_current = 1.0f) {
        ControlParams p{0, target_rad_s, 0, max_current};
        ControlModule::set_control_mode(VELOCITY, p);
    }

    static void currentMode(float target_amps) {
        ControlParams p{0, 0, target_amps, fabs(target_amps)*1.2f};
        ControlModule::set_control_mode(CURRENT, p);
    }

    // 级联控制模式
    static void cascadePosition(float target_rad, float max_vel = 30.0f, float max_current = 2.0f) {
        ControlParams p{target_rad, 0, 0, max_current};
        p.pos_vel_limit = max_vel;
        ControlModule::set_control_mode(CASCADE_POS_VEL_CUR, p);
    }

    // 更新方法
    static void update() {
        motor.updateSensorData();
        float Uq = calculateOutput();
        applyVoltage(Uq);
    }

    // 数据获取
    static float currentPosition() { return motor.sensor.position; }
    static float currentVelocity() { return motor.sensor.velocity; }
    static float currentCurrent() { return motor.computeIq(); }

private:
    static ControlModule motor;

    static float calculateOutput() {
        switch(motor.current_mode) {
            case ControlModule::POSITION: 
                return motor.positionClosedLoop(User_control_params.target_pos, 0.001f);
            case ControlModule::CASCADE_POS_VEL_CUR:
                return motor.PositionVelocityCurrentLoop(User_control_params.target_pos, 0.001f);
            // 其他模式处理...
            default: return 0;
        }
    }

    static void applyVoltage(float Uq) {
        MotorDriver::setPhaseVoltage(
            _constrain(Uq, -User_control_params.voltage_limit, User_control_params.voltage_limit),
            0,
            motor.sensor.mech_angle
        );
    }
}; 