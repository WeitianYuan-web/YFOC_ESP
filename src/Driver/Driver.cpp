#include "Driver.h"
#include "freertos/semphr.h"
#include "Wire.h"


void MotorDriver::hardwareInit(int pinA, int pinB, int pinC, 
                               int channelA, int channelB, int channelC) {
    // 保存通道信息（如果有需要后续使用，可增加成员变量保存）
    
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    pinMode(pinC, OUTPUT);
    
    ledcSetup(channelA, 20000, 10);
    ledcSetup(channelB, 20000, 10);
    ledcSetup(channelC, 20000, 10);
    
    ledcAttachPin(pinA, channelA);
    ledcAttachPin(pinB, channelB);
    ledcAttachPin(pinC, channelC);
}

void MotorDriver::setPwm(float Ua, float Ub, float Uc) {
    phase.dc_a = _constrain(Ua / params.voltage_power_supply, 0.0f, 1.0f);
    phase.dc_b = _constrain(Ub / params.voltage_power_supply, 0.0f, 1.0f);
    phase.dc_c = _constrain(Uc / params.voltage_power_supply, 0.0f, 1.0f);
    phase.Ua = Ua;
    phase.Ub = Ub;
    phase.Uc = Uc;  
    ledcWrite(0, phase.dc_a * 1023);
    ledcWrite(1, phase.dc_b * 1023);
    ledcWrite(2, phase.dc_c * 1023);
}

void MotorDriver::setPhaseVoltage(float Uq, float Ud, float angle_el) {
    Uq = _constrain(Uq, -params.voltage_limit/2, params.voltage_limit/2);
    Ud = _constrain(Ud, -params.voltage_limit/2, params.voltage_limit/2);
    
    phase.Ualpha = -Uq * sin(angle_el);
    phase.Ubeta = Uq * cos(angle_el);
    
    float v_a = phase.Ualpha;
    float v_b = -0.5f * phase.Ualpha + (sqrt(3) / 2.0f) * phase.Ubeta;
    float v_c = -0.5f * phase.Ualpha - (sqrt(3) / 2.0f) * phase.Ubeta;
    
    float v_max = max(max(v_a, v_b), v_c);
    float v_min = min(min(v_a, v_b), v_c);
    float v_offset = -(v_max + v_min) / 2.0f;
    
    float Ua_out = ((v_a + v_offset) / params.voltage_power_supply + 0.5f) * params.voltage_power_supply;
    float Ub_out = ((v_b + v_offset) / params.voltage_power_supply + 0.5f) * params.voltage_power_supply;
    float Uc_out = ((v_c + v_offset) / params.voltage_power_supply + 0.5f) * params.voltage_power_supply;
    
    setPwm(Ua_out, Ub_out, Uc_out);
}

float MotorDriver::velocityOpenloop(float target_velocity) {
    if(target_velocity == 0) {
        setPwm(0, 0, 0);
        return 0;
    }
    unsigned long now_us = micros();
    float Ts = (now_us - params.open_loop_timestamp) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    int direction = (target_velocity >= 0) ? 1 : -1;
    float abs_speed = fabs(target_velocity);
    float electrical_speed = _constrain(abs_speed * 7, 0, 30.0f) * direction;
    
    params.shaft_angle = normalizeAngle(params.shaft_angle + electrical_speed * Ts);
    
    float voltage_ratio = 0.3f + 0.7f * _constrain(abs_speed/10.0f, 0.0f, 1.0f);
    float Uq = params.voltage_power_supply * params.open_loop_voltage_ratio * voltage_ratio;
    
    Uq *= direction;
    if(fabs(Uq) < 3.0f && target_velocity != 0) {
        Uq = (direction > 0) ? 3.0f : -3.0f;
    }  
    
    params.open_loop_timestamp = now_us;
    return Uq;
}

float MotorDriver::electricalAngle(float shaft_angle, int pole_pairs) {
    return (shaft_angle * pole_pairs * params.direction);
}

float MotorDriver::normalizeAngle(float angle) {
    float a = fmod(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
} 

// 计算 Iq 的函数，输入参数为 Ia、Ib 和电角度 electrical_angle，返回计算得到的 q 轴电流 Iq
float MotorDriver::computeIq(float Ia, float Ib, float electrical_angle) {
    // 依据 Clarke 变换，取 Ialpha = Ia
    float Ialpha = Ia;
    /*  
       标准Clarke变换（假设平衡系统，且 Ia + Ib + Ic = 0）：
         Ibeta = (Ib - Ic) / sqrt(3)
              = (Ib - (-(Ia+Ib))) / sqrt(3)

              = (Ia + 2*Ib) / sqrt(3)
    */
    float Ibeta = (Ia + 2.0f * Ib) / sqrtf(3.0f);

    // 利用 Park 变换计算 q 轴电流
    float Iq = Ibeta * cos(electrical_angle) - Ialpha * sin(electrical_angle);
    return Iq;
}
