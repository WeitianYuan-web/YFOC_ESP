#include "ControlModule.h"
#include <math.h>
#include "freertos/semphr.h"
#include "Driver/Driver.h"



// 辅助函数声明
float _constrain(float amt, float low, float high);
float computeIq(float Ia, float Ib, float electrical_angle);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
static inline float pidCompute(float error, float dt, float Kp, float Ki, float Kd, float &integral, float &prev_error, float integral_limit);



float ControlModule::positionClosedLoop(float target_pos, float dt) {
    // 直接使用目标位置和当前位置的差值作为误差
    float error = target_pos - this->sensor.position;
    
    // 当误差在合理范围内时更新积分项
    if (fabs(error) < 3.14f) {  // 使用 π 作为阈值
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

float ControlModule::positionClosedLoop_single(float target_pos, float dt) {
    // 直接使用目标位置和当前位置的差值作为误差
    float raw_error = target_pos - this->sensor.position;
    float error = atan2(sin(raw_error), cos(raw_error));
    
    // 当误差在合理范围内时更新积分项
    if (fabs(error) < 3.14f) {  // 使用 π 作为阈值
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
    if (dt <= 0) return 0.0f;
    float error = target_vel - this->sensor.velocity;
    
    float control_output = pidCompute(error, dt, 
                                      this->pid.basic.velocity.Kp, 
                                      this->pid.basic.velocity.Ki, 
                                      this->pid.basic.velocity.Kd, 
                                      this->pid.basic.velocity.integral, 
                                      this->pid.basic.velocity.prev_error, 
                                      500.0f);  // 积分限幅值同样需调试确认
                                      
    // 添加前馈补偿
    float feedforward = this->filter.k_ff * target_vel;
    return control_output + feedforward;
}

float ControlModule::currentClosedLoop(float target_current, float dt, float electrical_angle, float Kp, float Ki) {
    // 获取并低通滤波 q 轴电流
    float measured_Iq = this->motorDriver.computeIq(this->current.Ia, this->current.Ib, electrical_angle);
    this->current.Iq_filtered = this->filter.alpha * measured_Iq + (1.0f - this->filter.alpha) * this->current.Iq_filtered;
    
    target_current = applyCurrentLimit(target_current);
    float error = target_current - this->current.Iq_filtered;
    
    // 累计积分（直接累加，不使用通用函数，因为目前只用了 PI 控制）
    this->pid.advanced.cascade.cur_integral += error * dt;
    if (this->pid.advanced.cascade.cur_integral > 100.0f)
        this->pid.advanced.cascade.cur_integral = 100.0f;
    else if (this->pid.advanced.cascade.cur_integral < -100.0f)
        this->pid.advanced.cascade.cur_integral = -100.0f;
    
    float control_output = Kp * error + Ki * this->pid.advanced.cascade.cur_integral;
    
    // 积分抗饱和策略
    if ((!isCurrentValid(control_output)) && (error * control_output > 0)) {
        this->pid.advanced.cascade.cur_integral *= 0.998f;
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
    return currentClosedLoop(target_current, dt, this->electrical_angle, 
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
    return currentClosedLoop(target_current, dt, this->electrical_angle, 
                            this->pid.advanced.velocity_current.cur_Kp, 
                            this->pid.advanced.velocity_current.cur_Ki);
}

float ControlModule::PositionVelocityCurrentLoop(float target_position, float dt) {
    // ----- 位置环 -----
    float raw_error = target_position - this->sensor.position;
    float pos_error = atan2(sin(raw_error), cos(raw_error));
    
    // 此处积分和微分使用统一 PID 函数（注意积分限幅根据需求调整）
    float target_velocity = pidCompute(pos_error, dt, 
                                       this->pid.advanced.cascade.pos_Kp, 
                                       this->pid.advanced.cascade.pos_Ki, 
                                       this->pid.advanced.cascade.pos_Kd,
                                       this->pid.advanced.cascade.pos_integral,
                                       this->pid.advanced.cascade.pos_prev_error,
                                       this->limits.cascade_limits.pos_vel_limit);
    // 限幅目标速度
    target_velocity = _constrain(target_velocity, -this->limits.cascade_limits.pos_vel_limit, this->limits.cascade_limits.pos_vel_limit);
    
    // ----- 速度环 -----
    float vel_error = target_velocity - this->sensor.velocity;
    float target_current = pidCompute(vel_error, dt, 
                                      this->pid.advanced.cascade.vel_Kp, 
                                      this->pid.advanced.cascade.vel_Ki, 
                                      this->pid.advanced.cascade.vel_Kd,
                                      this->pid.advanced.cascade.vel_integral,
                                      this->pid.advanced.cascade.vel_prev_error,
                                      80.0f);
    // 添加前馈补偿（根据实际进行调整）
    float feedforward = this->filter.k_ff_cascade * target_velocity;
    target_current += feedforward;
    
    // 对目标电流进行限幅，最后由电流闭环跟踪
    target_current = applyCurrentLimit(target_current);
    
    return currentClosedLoop(target_current, dt, 
                             this->electrical_angle, 
                             this->pid.advanced.cascade.cur_Kp, 
                             this->pid.advanced.cascade.cur_Ki);
}

float ControlModule::getElectricalAngle() {
    return this->motorDriver.electricalAngle(this->sensor.mech_angle, Motor_pole_pairs) - this->zero_electric_angle;
}

float ControlModule::getNormalizedElectricalAngle() {
    float electrical_angle = getElectricalAngle();  
    this->electrical_angle = this->motorDriver.normalizeAngle(electrical_angle);
    return this->electrical_angle;
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
    this->motorDriver.setPhaseVoltage(this->limits.max_current/2, 0, _3PI_2);
    delay(200);  // 等待电机稳定

    // 2. 更新传感器数据
    this->sensorManager.update();

    // 3. 使用传感器的机械角度计算电气角
    float measuredElectricalAngle = this->motorDriver.electricalAngle(
        this->sensorManager.getMechanicalAngle(),
        Motor_pole_pairs
    );

    // 4. 保存校准得到的电气角
   this->zero_electric_angle = measuredElectricalAngle * this->motorDriver.params.direction;
   this->motorDriver.params.zero_electric_angle = measuredElectricalAngle * this->motorDriver.params.direction;

    // 5. 校准完成后关闭电机相电压
    this->motorDriver.setPhaseVoltage(0, 0, _3PI_2);
}

void ControlModule::hardwareInit(int pinA, int pinB, int pinC, int direction,
                                 int channelA, int channelB, int channelC) {
    // 调用内嵌 MotorDriver 的硬件初始化接口
    motorDriver.hardwareInit(pinA, pinB, pinC, direction, channelA, channelB, channelC);
}

void ControlModule::initSensors(TwoWire* encoder_wire, uint8_t encoder_addr, 
                                int current_pinA, int current_pinB, int current_pinC) {
    sensorManager.SensorManager_init(current_pinA, current_pinB, current_pinC, encoder_wire, encoder_addr);
}

// 通用 PID 计算函数
// error: 当前误差，dt: 时间间隔
// Kp, Ki, Kd: PID 参数
// integral: 积分项引用，需要在函数外保存、更新
// prev_error: 上一次的误差（用于计算微分项）的引用
// integral_limit: 积分限幅值
static inline float pidCompute(float error, float dt, float Kp, float Ki, float Kd, float &integral, float &prev_error, float integral_limit) {
    // 更新积分项并限幅
    integral += error * dt;
    if (integral > integral_limit) integral = integral_limit;
    else if (integral < -integral_limit) integral = -integral_limit;
    
    // 计算微分项
    float derivative = (error - prev_error) / dt;
    prev_error = error;
    
    return Kp * error + Ki * integral + Kd * derivative;
}


