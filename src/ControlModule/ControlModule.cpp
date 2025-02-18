#include "ControlModule.h"
#include <math.h>
#include "freertos/semphr.h"
#include "Driver/Driver.h"



// 辅助函数声明
float _constrain(float amt, float low, float high);
float computeIq(float Ia, float Ib, float electrical_angle);
void setPhaseVoltage(float Uq, float Ud, float angle_el);

// ====================== 核心控制函数实现 ======================
bool ControlModule::set_control_mode(Mode mode, ControlParams params) {
    switch(mode) {
        case POSITION:  // 纯位置环
            this->current_mode = POSITION;
            this->target.position = params.target_pos;
            break;
            
        case VELOCITY:  // 纯速度环
            this->current_mode = VELOCITY;
            this->target.velocity = params.target_vel;
            break;
            
        case OPEN_LOOP:  // 开环模式
            this->current_mode = OPEN_LOOP;
            this->target.openloop_velocity = params.target_vel;
            break;
            
        case CURRENT:  // 纯电流环
            this->current_mode = CURRENT;
            this->target.current = params.target_cur;
            break;
            
        case CURRENT_POSITION:  // 电流+位置双环
            this->current_mode = CURRENT_POSITION;
            this->target.position = params.target_pos;
            this->limits.max_current = params.max_current;
            break;
            
        case CURRENT_VELOCITY:  // 电流+速度双环
            this->current_mode = CURRENT_VELOCITY;
            this->target.velocity = params.target_vel;
            this->limits.max_current = params.max_current;
            break;
            
        case CASCADE_POS_VEL_CUR:  // 三环级联
            this->current_mode = CASCADE_POS_VEL_CUR;
            this->target.position = params.target_pos;
            this->limits.cascade_limits.pos_vel_limit = params.pos_vel_limit;
            this->limits.max_current = params.max_current;
            break;
            
        default:
            return false;
    }
    return true;
}

float ControlModule::positionClosedLoop(float target_pos, float dt) {
    // 使用 atan2 计算最小角差，确保误差在 [-PI, PI] 范围内
    float raw_error = target_pos - this->sensor.position;
    float error = atan2(sin(raw_error), cos(raw_error));

    // 当误差在 ±90° 内时更新积分项，否则清零积分项以防止积分风up
    if (fabs(error) < 1.57f) {
        this->pid.basic.position.integral += error * dt;
        this->pid.basic.position.integral = _constrain(this->pid.basic.position.integral, -100.0f, 100.0f);
    } else {
        this->pid.basic.position.integral = 0.0f;
    }

    // PID微分部分
    float derivative = (error - this->pid.basic.position.prev_error) / dt;
    this->pid.basic.position.prev_error = error;

    float control_output = this->pid.basic.position.Kp * error 
         + this->pid.basic.position.Ki * this->pid.basic.position.integral
         + this->pid.basic.position.Kd * derivative;
    return control_output;
}

float ControlModule::velocityClosedLoop(float target_vel, float dt) {
   // 防止 dt 非法（例如首次调用或者系统抖动导致 dt 为 0）
    if (dt <= 0) return 0.0f;
    
    // 计算速度误差：目标速度 - 实际速度（单位：rad/s）
    float error = target_vel - this->sensor.velocity;
    
    // 积分项累加（同时限幅防止积分饱和）
    this->pid.basic.velocity.integral += error * dt;
    this->pid.basic.velocity.integral = _constrain(this->pid.basic.velocity.integral, -500.0f, 500.0f);
    
    // 计算微分项（当前误差与上一误差的变化率）
    float derivative = (error - this->pid.basic.velocity.prev_error) / dt;
    this->pid.basic.velocity.prev_error = error;
    
    // 计算前馈补偿：直接利用目标速度生成控制量
    float feedforward = this->filter.k_ff * target_vel;
    
    // PID 控制输出 + 前馈补偿
    float control_output = this->pid.basic.velocity.Kp * error 
                           + this->pid.basic.velocity.Ki * this->pid.basic.velocity.integral 
                           + this->pid.basic.velocity.Kd * derivative
                           + feedforward;

    
    // 返回最终控制输出，后续在 MotorControlTask 中会对该输出进行限幅处理
    return control_output;
}

float ControlModule::currentClosedLoop(float target_current, float dt, float electrical_angle, float Kp ,float Ki) {
 // 获取测量的 q 轴电流，并进行低通滤波
    float measured_Iq = this->motorDriver.computeIq(this->current.Ia, this->current.Ib, electrical_angle);
    this->current.Iq_filtered = this->filter.alpha * measured_Iq 
                             + (1.0f - this->filter.alpha) * this->current.Iq_filtered;
    
    target_current = applyCurrentLimit(target_current);
    // 计算电流环误差
    float current_error = target_current - this->current.Iq_filtered;
    

    // 更新电流环积分（使用 cascade 模块中的积分变量）
    this->pid.advanced.cascade.cur_integral += current_error * dt;
    this->pid.advanced.cascade.cur_integral = _constrain(this->pid.advanced.cascade.cur_integral, -100.0f, 100.0f);
    
    // 计算电流环 PID 输出
    float control_output = Kp * current_error 
                           + Ki * this->pid.advanced.cascade.cur_integral;
    
    // 积分抗饱和策略：当输出异常时修正积分项
    if ((!isCurrentValid(control_output)) && (current_error * control_output > 0)) {
        this->pid.advanced.cascade.cur_integral = 0.998f * this->pid.advanced.cascade.cur_integral;
    }
    

    return control_output;
}
float ControlModule::openLoopControl(float target_velocity) {
    return this->motorDriver.velocityOpenloop(target_velocity); // 调用开环速度函数
}


// ====================== 电流相关函数 ======================
float ControlModule::applyCurrentLimit(float target) {
    float abs_target = fabs(target);
    float limited = _constrain(target, -this->limits.max_current, this->limits.max_current);
    if (abs_target < this->limits.min_current) {
        return 0.0f;
    }
    return limited;
}

bool ControlModule::isCurrentValid(float current) {
    float abs_current = fabs(current);
    return (abs_current >= this->limits.min_current &&
            abs_current <= this->limits.max_current);
}

// ====================== 高级控制模式实现 ======================
float ControlModule::PositionCurrentClosedLoop(float target_position, float dt) {
        // 使用 atan2 计算最小角差，确保误差在 [-PI, PI] 范围内
    float raw_error = target_position - this->sensor.position;
    float position_error = atan2(sin(raw_error), cos(raw_error));

      // 当误差在 ±90° 内时更新积分项，否则清零积分项以防止积分风up
    if (fabs(position_error) < 1.57f) {
        this->pid.advanced.position_current.pos_integral += position_error * dt;
        this->pid.advanced.position_current.pos_integral = _constrain(this->pid.advanced.position_current.pos_integral, -50.0f, 50.0f);
    } else {
        this->pid.advanced.position_current.pos_integral = 0.0f;
    }

    // 微分项计算
    float pos_derivative = (position_error - this->pid.advanced.position_current.pos_prev_error) / dt;
    this->pid.advanced.position_current.pos_prev_error = position_error;
    
    

    float target_current = this->pid.advanced.position_current.pos_Kp * position_error 
                         + this->pid.advanced.position_current.pos_Ki * this->pid.advanced.position_current.pos_integral
                         + this->pid.advanced.position_current.pos_Kd * pos_derivative;
    
    // 应用电流限制（包含最小值和最大值）
    target_current = applyCurrentLimit(target_current);
   
    // 电流环跟踪（此处继续使用原有电流闭环逻辑）
    return currentClosedLoop(target_current, dt, this->motorDriver.electricalAngle(this->sensor.mech_angle, Motor_pole_pairs), 
                            this->pid.advanced.position_current.cur_Kp, 
                            this->pid.advanced.position_current.cur_Ki);
}

float ControlModule::VelocityCurrentClosedLoop(float target_velocity, float dt) {
      // 防止 dt 非法（例如首次调用或者系统抖动导致 dt 为 0）
    if (dt <= 0) return 0.0f;
    float velocity_error = target_velocity - this->sensor.velocity;
    
    // 微分项计算
    float vel_derivative = (velocity_error - this->pid.advanced.velocity_current.vel_prev_error) / dt;
    this->pid.advanced.velocity_current.vel_prev_error = velocity_error;
    
    // PID计算
    this->pid.advanced.velocity_current.vel_integral += velocity_error * dt;
    // 计算前馈补偿：直接利用目标速度生成控制量
    float feedforward = this->filter.k_ff_velocity_current * target_velocity;

    float target_current = this->pid.advanced.velocity_current.vel_Kp * velocity_error 
                         + this->pid.advanced.velocity_current.vel_Ki * this->pid.advanced.velocity_current.vel_integral
                         + this->pid.advanced.velocity_current.vel_Kd * vel_derivative
                         + feedforward;
    // 对目标电流进行限幅
    target_current = applyCurrentLimit(target_current);
  
    // 电流环跟踪
    return currentClosedLoop(target_current, dt, this->motorDriver.electricalAngle(this->sensor.mech_angle, Motor_pole_pairs), 
                            this->pid.advanced.velocity_current.cur_Kp, 
                            this->pid.advanced.velocity_current.cur_Ki);
}

float ControlModule::PositionVelocityCurrentLoop(float target_position, float dt) {
    // 使用主编码器位置作为反馈
    float raw_error = target_position - this->sensor.position;
    float position_error = atan2(sin(raw_error), cos(raw_error));
    
    // 计算位置环微分，并更新历史误差
    float pos_derivative = (position_error - this->pid.advanced.cascade.pos_prev_error) / dt;
    this->pid.advanced.cascade.pos_prev_error = position_error;
    
    // 计算位置环 P 和 D 分量
    float p_pos = this->pid.advanced.cascade.pos_Kp * position_error;
    float d_pos = this->pid.advanced.cascade.pos_Kd * pos_derivative;
    float pd_pos = p_pos + d_pos;
    
    // 条件积分：当 PD 输出未饱和时积分，否则采用泄漏防止积分风up
    if (fabs(pd_pos) < this->limits.cascade_limits.pos_vel_limit) {
        this->pid.advanced.cascade.pos_integral += position_error * dt;
        this->pid.advanced.cascade.pos_integral = _constrain(this->pid.advanced.cascade.pos_integral, -100.0f, 100.0f);
    } else {
        // 积分泄漏
        this->pid.advanced.cascade.pos_integral *= 0.98f;
    }
    
    // 位置环 PID 输出（目标速度），并限幅
    float target_velocity = p_pos + this->pid.advanced.cascade.pos_Ki * this->pid.advanced.cascade.pos_integral + d_pos;
    target_velocity = _constrain(target_velocity, -this->limits.cascade_limits.pos_vel_limit, this->limits.cascade_limits.pos_vel_limit);
    
    
    // --- 速度环 ---
    float velocity_error = target_velocity - this->sensor.velocity;
    float vel_derivative = (velocity_error - this->pid.advanced.cascade.vel_prev_error) / dt;
    this->pid.advanced.cascade.vel_prev_error = velocity_error;
    
    float p_vel = this->pid.advanced.cascade.vel_Kp * velocity_error;
    float d_vel = this->pid.advanced.cascade.vel_Kd * vel_derivative;
    float pd_vel = p_vel + d_vel;
    
    if (fabs(pd_vel) < 100.0f) {
        this->pid.advanced.cascade.vel_integral += velocity_error * dt;
        this->pid.advanced.cascade.vel_integral = _constrain(this->pid.advanced.cascade.vel_integral, -80.0f, 80.0f);
    } else {

        this->pid.advanced.cascade.vel_integral *= 0.98f;
    }
    
    // 加入前馈补偿（利用目标速度及电机参数进行比例调节）
    float feedforward = this->filter.k_ff_cascade * target_velocity * this->limits.cascade_limits.pos_vel_limit / Motor_max_velocity;
    
    // 速度环 PID 输出转为目标电流，并限幅
    float target_current = p_vel + this->pid.advanced.cascade.vel_Ki * this->pid.advanced.cascade.vel_integral + d_vel + feedforward;
    target_current = applyCurrentLimit(target_current);
    
    
    // --- 内嵌电流环 ---
    // 获取测量的 q 轴电流（用 AS5600 回传数据计算得到），并经过低通滤波
    float measured_Iq = this->motorDriver.computeIq(this->current.Ia, this->current.Ib, this->motorDriver.electricalAngle(this->sensor.mech_angle, Motor_pole_pairs));
    this->current.Iq_filtered = this->filter.alpha * measured_Iq + (1.0f - this->filter.alpha) * this->current.Iq_filtered;
    
    // 电流环误差
    float current_error = target_current - this->current.Iq_filtered;
    

    // 更新电流环积分累积（注意：需在 cascade 结构中添加成员 cur_integral，初始值为 0）
    this->pid.advanced.cascade.cur_integral += current_error * dt;
    this->pid.advanced.cascade.cur_integral = _constrain(this->pid.advanced.cascade.cur_integral, -50.0f, 50.0f);
    
    // 计算电流环 PID 输出
    float current_output = this->pid.advanced.cascade.cur_Kp * current_error 
                           + this->pid.advanced.cascade.cur_Ki * this->pid.advanced.cascade.cur_integral;
    
    // 当前环抗积分饱和：当输出异常时减小积分
    if ((!isCurrentValid(current_output)) && (current_error * current_output > 0)) {
        this->pid.advanced.cascade.cur_integral = 0.998f * this->pid.advanced.cascade.cur_integral;
    }
    
    return current_output;
}

float ControlModule::getElectricalAngle() {
    return this->motorDriver.electricalAngle(this->sensor.mech_angle, Motor_pole_pairs);
}

float ControlModule::getNormalizedElectricalAngle() {
    return this->motorDriver.normalizeAngle(getElectricalAngle());
}

void ControlModule::setPhaseVoltage(float Uq, float Ud, float angle_el) {
    this->motorDriver.setPhaseVoltage(Uq, Ud, angle_el);
}

// ====================== 工具函数 ======================
float _constrain(float amt, float low, float high) {
    return (amt < low) ? low : ((amt > high) ? high : amt);
}

// 更新传感器数据，采用 SensorManager 内部的两阶段更新机制
void ControlModule::updateSensorData() {
    // 调用 SensorManager 的更新函数
    this->sensorManager.update();
    
    // 复制缓存数据到 ControlModule 内部数据结构
    this->sensor.position   = sensorManager.getAngle();
    this->sensor.velocity   = sensorManager.getVelocity();
    this->sensor.mech_angle = sensorManager.getMechanicalAngle();
    
    this->current.Ia = sensorManager.getCurrentA();
    this->current.Ib = sensorManager.getCurrentB();
    this->current.Ic = sensorManager.getCurrentC(); // 如果需要使用第三路电流数据
    this->current.Iq = motorDriver.computeIq(this->current.Ia, this->current.Ib, this->motorDriver.electricalAngle(this->sensor.mech_angle, Motor_pole_pairs));
}


void ControlModule::initPID() {
    this->pid.basic.position.integral = 0;
    this->pid.basic.position.prev_error = 0;
    this->pid.basic.velocity.integral = 0;
    this->pid.basic.velocity.prev_error = 0;
    this->pid.basic.current.integral = 0;
    this->pid.basic.current.prev_error = 0;   
}

void ControlModule::calibrateZeroPoint() {
    // 1. 用最大电流对电机通道进行校准注入
    this->motorDriver.setPhaseVoltage(this->limits.max_current, 0, _3PI_2);
    delay(200);  // 等待电机稳定

    // 2. 更新传感器数据
    this->sensorManager.update();

    // 3. 使用传感器的机械角度计算电气角
    float measuredElectricalAngle = this->motorDriver.electricalAngle(
        this->motorDriver.sensor.getMechanicalAngle(),
        Motor_pole_pairs
    );

    // 4. 保存校准得到的电气角
    this->zero_electric_angle = measuredElectricalAngle;
    this->motorDriver.params.zero_electric_angle = measuredElectricalAngle;

    // 5. 校准完成后关闭电机相电压
    this->motorDriver.setPhaseVoltage(0, 0, _3PI_2);
}

void ControlModule::hardwareInit(int pinA, int pinB, int pinC, 
                                 int channelA, int channelB, int channelC) {
    // 调用内嵌 MotorDriver 的硬件初始化接口
    motorDriver.hardwareInit(pinA, pinB, pinC, channelA, channelB, channelC);
}

void ControlModule::initSensors(TwoWire* encoder_wire, uint8_t encoder_addr, 
                                int current_pinA, int current_pinB, int current_pinC) {
    sensorManager.init(current_pinA, current_pinB, current_pinC, encoder_wire, encoder_addr);
}


