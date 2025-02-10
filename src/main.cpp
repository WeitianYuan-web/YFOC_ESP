#include <Arduino.h>
#include "InlineCurrent.h"
#include "vofa.h"
#include "AS5600.h"
#include "Wire.h"
#include <math.h>
#include "freertos/semphr.h"   // 添加 FreeRTOS 互斥锁支持

// PWM输出引脚定义
const int pwmA = 32;
const int pwmB = 33;
const int pwmC = 25;

#define Motor_max_velocity 60.0f    //rad/s
#define Motor_max_current 0.9f      //A
#define Motor_resistance 16.0f //电机相电阻
#define Motor_max_voltage 12.6f     //V
#define Motor_KV 10.0f      //KV值

// 常量定义
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define DIRECTION_FORWARD -1


// 宏定义约束函数
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// 定义存储电源参数与电机状态的结构体
struct MotorParameters {
    float voltage_power_supply = Motor_max_voltage;  // 供电电压
    float voltage_limit = 12.6f;         // 最大输出电压限制
    float shaft_angle = 0.0f;            // 机械轴角
    float open_loop_timestamp = 0.0f;    // 开环更新时间戳
    float zero_electric_angle = 0.0f;    // 电机零电角

    float electrical_angle = 0.0f;        // 电角度
    int direction = DIRECTION_FORWARD; // 电机旋转方向系数
    float open_loop_voltage_ratio = 0.7f; // 开环电压比例系数
    float max_openloop_speed = 10.0f; // 最大开环速度（rad/s）

};

// 定义保存相电压数据的结构体
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


// 全局实例
MotorParameters motorParams;
PhaseVoltage phaseVolt;

// 全局对象
CurrSense current_sense(0);     // 电流检测对象（电机0的配置）
Sensor_AS5600 sensor(0);        // AS5600对象（电机0的配置）



// 前置函数声明
float _electricalAngle(float shaft_angle, int pole_pairs);
float _normalizeAngle(float angle);
void setPwm(float Ua, float Ub, float Uc);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
float velocityOpenloop(float target_velocity);
float computeIq(float Ia, float Ib, float electrical_angle);
float applyCurrentLimit(float target);
bool isCurrentValid(float current);

// FreeRTOS任务函数声明
void MotorControlTask(void *pvParameters);
void SensorUpdateTask(void *pvParameters);
void DataPrintTask(void *pvParameters);
void SerialReadTask(void *pvParameters);


// ====================== 修改后的控制模块结构体 ======================
struct ControlModule {
  enum Mode { 
    POSITION = 1,          // 纯位置环
    VELOCITY = 2,          // 纯速度环
    OPEN_LOOP = 3,         // 开环模式
    CURRENT = 4,           // 纯电流环
    CURRENT_POSITION = 5,  // 电流+位置双环 (新增)
    CURRENT_VELOCITY = 6,  // 电流+速度双环 (新增)
    CASCADE_POS_VEL_CUR =7 // 位置-速度-电流三环 (新增)
  } current_mode = POSITION;
  
  // 传感器接口
  struct {
    float position;      // 累计位置（rad）
    float velocity;      // 转速（rad/s）
    float mech_angle;    // 机械角度（rad）
  } sensor;
  
  struct {
    float Ia;
    float Ib;
    float Ic;
    float Iq;
    float Iq_filtered;
  } current;
  

  // 控制目标说明
  struct {
    float position;      // 位置模式目标（rad）
    float openloop_velocity;      // 开环模式目标（rad/s）
    float velocity;      // 速度模式目标（rad/s）
    float voltage;       // 预留字段
    float current;       // 电流闭环模式目标（目标Iq）

  } target;

  // PID参数分组
  struct {
    // 基础模式参数
    struct {
      // 原有位置环参数
      struct {
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

    // 新增模式专用参数
    struct {
      // 电流+位置双环参数
      struct {
        float pos_Kp = 2.0f;   // 位置环比例
        float pos_Ki = 0.002f;  // 位置环积分
        float pos_Kd = 0.02f;   // 新增位置环微分
        float pos_prev_error = 0; // 新增误差记录
        float cur_Kp = 3.0f;   // 电流环比例
        float cur_Ki = 500.0f; // 电流环积分
        float pos_integral = 0;
      } position_current;
      
      // 电流+速度双环参数
      struct {
        float vel_Kp = 0.1f;   // 速度环比例
        float vel_Ki = 0.1f;  // 速度环积分 
        float vel_Kd = 0.0002f;  // 新增速度环微分
        float vel_prev_error = 0;
        float cur_Kp = 4.0f;
        float cur_Ki = 300.0f;
        float vel_integral = 0;
      } velocity_current;
      
      // 三环级联参数
      struct {
        float pos_Kp = 5.0f;
        float pos_Ki = 0.001f;
        float pos_Kd = 0.0015f;   // 位置环微分
        float vel_Kp = 0.1f;
        float vel_Ki = 0.01f;
        float vel_Kd = 0.00015f;  // 速度环微分
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

  // 其他参数
  struct {
    float k_ff = 0.09f;  // 速度前馈增益
    float k_ff_velocity_current = 0.01f;  // 速度前馈增益
    float k_ff_cascade = 0.4f;  // 三环级联前馈增益
    float alpha = 0.2f;  // 电流低通滤波系数 (0 < alpha < 1, 越小滤波越强)
  } filter;


  // 电流限制参数
  struct {
    float max_current = 0.9f;  // 最大电流限制
    float min_current = 0.001f;  // 新增：最小有效电流（绝对值）
    struct {
      float pos_vel_limit = Motor_max_velocity;  // 三环位置环速度限制
    } cascade_limits;
  } limits;

};

ControlModule ctrl; // 全局控制模块实例

// 在全局范围新增两个互斥锁
SemaphoreHandle_t sensorMutex = xSemaphoreCreateRecursiveMutex();  // 传感器数据专用

// ====================== 独立控制算法函数 ======================
// 位置闭环计算
float positionClosedLoop(float target_pos, float dt) {
    // 使用 atan2 计算最小角差，确保误差在 [-PI, PI] 范围内
    float raw_error = target_pos - ctrl.sensor.position;
    float error = atan2(sin(raw_error), cos(raw_error));

    // 当误差在 ±90° 内时更新积分项，否则清零积分项以防止积分风up
    if (fabs(error) < 1.57f) {
        ctrl.pid.basic.position.integral += error * dt;
        ctrl.pid.basic.position.integral = _constrain(ctrl.pid.basic.position.integral, -100.0f, 100.0f);
    } else {
        ctrl.pid.basic.position.integral = 0.0f;
    }




    // PID微分部分
    float derivative = (error - ctrl.pid.basic.position.prev_error) / dt;
    ctrl.pid.basic.position.prev_error = error;


    float control_output = ctrl.pid.basic.position.Kp * error 
         + ctrl.pid.basic.position.Ki * ctrl.pid.basic.position.integral
         + ctrl.pid.basic.position.Kd * derivative;
    return control_output;



}

// 改进后的速度闭环计算（使用 PID + 前馈控制）
float velocityClosedLoop(float target_vel, float dt) {
    // 防止 dt 非法（例如首次调用或者系统抖动导致 dt 为 0）
    if (dt <= 0) return 0.0f;
    
    // 计算速度误差：目标速度 - 实际速度（单位：rad/s）
    float error = target_vel - ctrl.sensor.velocity;
    
    // 积分项累加（同时限幅防止积分饱和）
    ctrl.pid.basic.velocity.integral += error * dt;
    ctrl.pid.basic.velocity.integral = _constrain(ctrl.pid.basic.velocity.integral, -500.0f, 500.0f);
    
    // 计算微分项（当前误差与上一误差的变化率）
    float derivative = (error - ctrl.pid.basic.velocity.prev_error) / dt;
    ctrl.pid.basic.velocity.prev_error = error;
    
    // 计算前馈补偿：直接利用目标速度生成控制量
    float feedforward = ctrl.filter.k_ff * target_vel;
    
    // PID 控制输出 + 前馈补偿
    float control_output = ctrl.pid.basic.velocity.Kp * error 
                           + ctrl.pid.basic.velocity.Ki * ctrl.pid.basic.velocity.integral 
                           + ctrl.pid.basic.velocity.Kd * derivative
                           + feedforward;

    
    // 返回最终控制输出，后续在 MotorControlTask 中会对该输出进行限幅处理
    return control_output;
}

// 修改后的开环控制函数
float openLoopControl(float target_velocity) {
    // 使用固定12V供电电压，第三个参数作为目标速度
    const float voltage = 12.0f; // 固定供电电压
    //target_velocity = _constrain(target_velocity, -motorParams.max_openloop_speed, motorParams.max_openloop_speed);
    return velocityOpenloop(target_velocity); // 调用开环速度函数
}

// 电流闭环控制函数（使用 PI 控制，并先计算 Iq）
// 请确保传入 electrical_angle 为当前电机电角（单位：rad）
float currentClosedLoop(float target_current, float dt, float electrical_angle, float Kp ,float Ki) {
    // 获取测量的 q 轴电流，并进行低通滤波
    float measured_Iq = computeIq(ctrl.current.Ia, ctrl.current.Ib, electrical_angle);
    ctrl.current.Iq_filtered = ctrl.filter.alpha * measured_Iq 
                             + (1.0f - ctrl.filter.alpha) * ctrl.current.Iq_filtered;
    
    target_current = applyCurrentLimit(target_current);
    // 计算电流环误差
    float current_error = target_current - ctrl.current.Iq_filtered;
    

    // 更新电流环积分（使用 cascade 模块中的积分变量）
    ctrl.pid.advanced.cascade.cur_integral += current_error * dt;
    ctrl.pid.advanced.cascade.cur_integral = _constrain(ctrl.pid.advanced.cascade.cur_integral, -100.0f, 100.0f);
    
    // 计算电流环 PID 输出
    float control_output = Kp * current_error 
                           + Ki * ctrl.pid.advanced.cascade.cur_integral;
    
    // 积分抗饱和策略：当输出异常时修正积分项
    if ((!isCurrentValid(control_output)) && (current_error * control_output > 0)) {
        ctrl.pid.advanced.cascade.cur_integral = 0.99 * ctrl.pid.advanced.cascade.cur_integral;
    }
    

    return control_output;
}

// 计算 Iq 的函数，输入参数为 Ia、Ib 和电角度 electrical_angle，返回计算得到的 q 轴电流 Iq
float computeIq(float Ia, float Ib, float electrical_angle) {
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

// ================== 电流+位置双环控制 ==================
float positionCurrentClosedLoop(float target_position, float dt) {
        // 使用 atan2 计算最小角差，确保误差在 [-PI, PI] 范围内
    float raw_error = target_position - ctrl.sensor.position;
    float position_error = atan2(sin(raw_error), cos(raw_error));

      // 当误差在 ±90° 内时更新积分项，否则清零积分项以防止积分风up
    if (fabs(position_error) < 1.57f) {
        ctrl.pid.advanced.position_current.pos_integral += position_error * dt;
        ctrl.pid.advanced.position_current.pos_integral = _constrain(ctrl.pid.advanced.position_current.pos_integral, -50.0f, 50.0f);
    } else {
        ctrl.pid.advanced.position_current.pos_integral = 0.0f;
    }

    // 微分项计算
    float pos_derivative = (position_error - ctrl.pid.advanced.position_current.pos_prev_error) / dt;
    ctrl.pid.advanced.position_current.pos_prev_error = position_error;
    
    

    float target_current = ctrl.pid.advanced.position_current.pos_Kp * position_error 
                         + ctrl.pid.advanced.position_current.pos_Ki * ctrl.pid.advanced.position_current.pos_integral
                         + ctrl.pid.advanced.position_current.pos_Kd * pos_derivative;
    
    // 应用电流限制（包含最小值和最大值）
    target_current = applyCurrentLimit(target_current);
   
    // 电流环跟踪（此处继续使用原有电流闭环逻辑）
    return currentClosedLoop(target_current, dt, motorParams.electrical_angle, 
                            ctrl.pid.advanced.position_current.cur_Kp, 
                            ctrl.pid.advanced.position_current.cur_Ki);
}

// ================== 电流+速度双环控制 ==================
float velocityCurrentClosedLoop(float target_velocity, float dt) {
      // 防止 dt 非法（例如首次调用或者系统抖动导致 dt 为 0）
    if (dt <= 0) return 0.0f;
    float velocity_error = target_velocity - ctrl.sensor.velocity;
    
    // 微分项计算
    float vel_derivative = (velocity_error - ctrl.pid.advanced.velocity_current.vel_prev_error) / dt;
    ctrl.pid.advanced.velocity_current.vel_prev_error = velocity_error;
    
    // PID计算
    ctrl.pid.advanced.velocity_current.vel_integral += velocity_error * dt;
    // 计算前馈补偿：直接利用目标速度生成控制量
    float feedforward = ctrl.filter.k_ff_velocity_current * target_velocity;

    float target_current = ctrl.pid.advanced.velocity_current.vel_Kp * velocity_error 
                         + ctrl.pid.advanced.velocity_current.vel_Ki * ctrl.pid.advanced.velocity_current.vel_integral
                         + ctrl.pid.advanced.velocity_current.vel_Kd * vel_derivative
                         + feedforward;
    // 对目标电流进行限幅
    target_current = applyCurrentLimit(target_current);
  
    // 电流环跟踪
    return currentClosedLoop(target_current, dt, motorParams.electrical_angle, 
                            ctrl.pid.advanced.velocity_current.cur_Kp, 
                            ctrl.pid.advanced.velocity_current.cur_Ki);
}



// ================== 位置-速度-电流三环控制 ==================
float cascadePositionVelocityCurrentLoop(float target_position, float dt) {
    // --- 位置环 ---
    // 计算位置误差，确保误差在 [-PI, PI] 范围内
    float raw_error = target_position - ctrl.sensor.position;
    float position_error = atan2(sin(raw_error), cos(raw_error));
    
    // 计算位置环微分，并更新历史误差
    float pos_derivative = (position_error - ctrl.pid.advanced.cascade.pos_prev_error) / dt;
    ctrl.pid.advanced.cascade.pos_prev_error = position_error;
    
    // 计算位置环 P 和 D 分量
    float p_pos = ctrl.pid.advanced.cascade.pos_Kp * position_error;
    float d_pos = ctrl.pid.advanced.cascade.pos_Kd * pos_derivative;
    float pd_pos = p_pos + d_pos;
    
    // 条件积分：当 PD 输出未饱和时积分，否则采用泄漏防止积分风up
    if (fabs(pd_pos) < ctrl.limits.cascade_limits.pos_vel_limit) {
        ctrl.pid.advanced.cascade.pos_integral += position_error * dt;
        ctrl.pid.advanced.cascade.pos_integral = _constrain(ctrl.pid.advanced.cascade.pos_integral, -100.0f, 100.0f);
    } else {
        // 积分泄漏
        ctrl.pid.advanced.cascade.pos_integral *= 0.98f;
    }
    
    // 位置环 PID 输出（目标速度），并限幅
    float target_velocity = p_pos + ctrl.pid.advanced.cascade.pos_Ki * ctrl.pid.advanced.cascade.pos_integral + d_pos;
    target_velocity = _constrain(target_velocity, -ctrl.limits.cascade_limits.pos_vel_limit, ctrl.limits.cascade_limits.pos_vel_limit);
    
    
    // --- 速度环 ---
    float velocity_error = target_velocity - ctrl.sensor.velocity;
    float vel_derivative = (velocity_error - ctrl.pid.advanced.cascade.vel_prev_error) / dt;
    ctrl.pid.advanced.cascade.vel_prev_error = velocity_error;
    
    float p_vel = ctrl.pid.advanced.cascade.vel_Kp * velocity_error;
    float d_vel = ctrl.pid.advanced.cascade.vel_Kd * vel_derivative;
    float pd_vel = p_vel + d_vel;
    
    if (fabs(pd_vel) < 100.0f) {
        ctrl.pid.advanced.cascade.vel_integral += velocity_error * dt;
        ctrl.pid.advanced.cascade.vel_integral = _constrain(ctrl.pid.advanced.cascade.vel_integral, -80.0f, 80.0f);
    } else {

        ctrl.pid.advanced.cascade.vel_integral *= 0.98f;
    }
    
    // 加入前馈补偿（利用目标速度及电机参数进行比例调节）
    float feedforward = ctrl.filter.k_ff_cascade * target_velocity * ctrl.limits.cascade_limits.pos_vel_limit / Motor_max_velocity;
    
    // 速度环 PID 输出转为目标电流，并限幅
    float target_current = p_vel + ctrl.pid.advanced.cascade.vel_Ki * ctrl.pid.advanced.cascade.vel_integral + d_vel + feedforward;
    target_current = applyCurrentLimit(target_current);
    
    
    // --- 内嵌电流环 ---
    // 获取测量的 q 轴电流（用 AS5600 回传数据计算得到），并经过低通滤波
    float measured_Iq = computeIq(ctrl.current.Ia, ctrl.current.Ib, motorParams.electrical_angle);
    ctrl.current.Iq_filtered = ctrl.filter.alpha * measured_Iq + (1.0f - ctrl.filter.alpha) * ctrl.current.Iq_filtered;
    
    // 电流环误差
    float current_error = target_current - ctrl.current.Iq_filtered;
    

    // 更新电流环积分累积（注意：需在 cascade 结构中添加成员 cur_integral，初始值为 0）
    ctrl.pid.advanced.cascade.cur_integral += current_error * dt;
    ctrl.pid.advanced.cascade.cur_integral = _constrain(ctrl.pid.advanced.cascade.cur_integral, -50.0f, 50.0f);
    
    // 计算电流环 PID 输出
    float current_output = ctrl.pid.advanced.cascade.cur_Kp * current_error 
                           + ctrl.pid.advanced.cascade.cur_Ki * ctrl.pid.advanced.cascade.cur_integral;
    
    // 当前环抗积分饱和：当输出异常时减小积分
    if ((!isCurrentValid(current_output)) && (current_error * current_output > 0)) {
        ctrl.pid.advanced.cascade.cur_integral = 0.99f * ctrl.pid.advanced.cascade.cur_integral;
    }
    
    return current_output;
}


// 声明全局递归互斥锁
SemaphoreHandle_t globalMutex = NULL;

void setup() {
  Serial.begin(115200);
  delay(10);

  // 创建全局递归互斥锁
  globalMutex = xSemaphoreCreateRecursiveMutex();
  if (globalMutex == NULL) {
    Serial.println("创建全局互斥锁失败！");
    while (1);
  }

  // PWM初始化
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  ledcSetup(0, 20000, 10);  // PWM通道0, 频率20000Hz, 10位分辨率
  ledcSetup(1, 20000, 10);  // PWM通道1, 频率20000Hz, 10位分辨率
  ledcSetup(2, 20000, 10);  // PWM通道2, 频率20000Hz, 10位分辨率
  ledcAttachPin(pwmA, 0);
  ledcAttachPin(pwmB, 1);
  ledcAttachPin(pwmC, 2);
  Serial.println("完成PWM初始化设置");
  
  // 板载信号灯初始化
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  delay(10);

  // 电流检测初始化
  current_sense.init();
  Serial.println("电流检测初始化完成");
  delay(10);

  // I2C初始化
  Wire.begin(19, 18, 400000);

  // AS5600初始化
  sensor.Sensor_init(&Wire);
  Serial.println("AS5600初始化完成");
  delay(10);

  setPhaseVoltage(motorParams.voltage_power_supply/2, 0, _3PI_2);
  delay(200);
  // 校准流程
  sensor.Sensor_update();
  motorParams.zero_electric_angle = _electricalAngle(sensor.getMechanicalAngle(), 7);
  

  delay(200);
  setPhaseVoltage(0, 0, _3PI_2);



  // 创建FreeRTOS任务
  xTaskCreatePinnedToCore(MotorControlTask, "MotorControlTask", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(DataPrintTask, "DataPrintTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(SerialReadTask, "SerialReadTask", 2048, NULL, 1, NULL, 1);

}

void loop() {
  // 空的 loop 函数，任务由 FreeRTOS 调度
}

// 计算电角度：机械角度乘以极对数
float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs * DIRECTION_FORWARD);
}

// 将角度归一化到 [0, 2PI]
float _normalizeAngle(float angle) {
  float a = fmod(angle, 2 * PI);
  return a >= 0 ? a : (a + 2 * PI);
}

// 设置PWM信号到控制器输出
void setPwm(float Ua, float Ub, float Uc) {
  phaseVolt.dc_a = _constrain(Ua / motorParams.voltage_power_supply, 0.0f, 1.0f);
  phaseVolt.dc_b = _constrain(Ub / motorParams.voltage_power_supply, 0.0f, 1.0f);
  phaseVolt.dc_c = _constrain(Uc / motorParams.voltage_power_supply, 0.0f, 1.0f);
  phaseVolt.Ua = Ua;
  phaseVolt.Ub = Ub;
  phaseVolt.Uc = Uc;  
  ledcWrite(0, phaseVolt.dc_a * 1023);
  ledcWrite(1, phaseVolt.dc_b * 1023);
  ledcWrite(2, phaseVolt.dc_c * 1023);
}

// 根据 Uq、Ud 与电角度设定相电压（目前 Ud 固定为 0）
void setPhaseVoltage(float Uq, float Ud, float angle_el) {
    Uq = _constrain(Uq, -motorParams.voltage_limit/2, motorParams.voltage_limit/2);
    Ud = _constrain(Ud, -motorParams.voltage_limit/2, motorParams.voltage_limit/2);
    

    // 计算 Ualpha 与 Ubeta（与 SPWM 相同）
    phaseVolt.Ualpha = -Uq * sin(angle_el);
    phaseVolt.Ubeta = Uq * cos(angle_el);
    
    // 逆 Clarke 变换（未加入零序分量）
    float v_a = phaseVolt.Ualpha;
    float v_b = -0.5f * phaseVolt.Ualpha + (sqrt(3) / 2.0f) * phaseVolt.Ubeta;
    float v_c = -0.5f * phaseVolt.Ualpha - (sqrt(3) / 2.0f) * phaseVolt.Ubeta;
    
    // 计算三相中最大值和最小值
    float v_max = max(max(v_a, v_b), v_c);
    float v_min = min(min(v_a, v_b), v_c);
    // 计算零序分量（公共模式电压）
    float v_offset = -(v_max + v_min) / 2.0f;
    
    // 注入零序分量后重新计算三相电压
    v_a += v_offset;
    v_b += v_offset;
    v_c += v_offset;
    
    // 将电压转换成 PWM 输出。计算公式为：duty = (v_phase/V_dc + 0.5)，再乘以 V_dc 得到传给 setPwm 的值
    float Ua_out = (v_a / motorParams.voltage_power_supply + 0.5f) * motorParams.voltage_power_supply;
    float Ub_out = (v_b / motorParams.voltage_power_supply + 0.5f) * motorParams.voltage_power_supply;
    float Uc_out = (v_c / motorParams.voltage_power_supply + 0.5f) * motorParams.voltage_power_supply;
    
    setPwm(Ua_out, Ub_out, Uc_out);

}

// 修正后的开环速度控制函数
float velocityOpenloop(float target_velocity) {
  unsigned long now_us = micros();
  float Ts = (now_us - motorParams.open_loop_timestamp) * 1e-6f;
  if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // 处理方向
  int direction = (target_velocity >= 0) ? 1 : -1;
  float abs_speed = fabs(target_velocity);
  
  // 电角度速度计算
  float electrical_speed = abs_speed * 7; // 7为极对数
  electrical_speed = _constrain(electrical_speed, 0, 30.0f) * direction;
  
  // 更新角度
  motorParams.shaft_angle = _normalizeAngle(motorParams.shaft_angle + electrical_speed * Ts);
  
  // 动态电压计算
  float voltage_ratio = 0.3f + 0.7f * _constrain(abs_speed/10.0f, 0.0f, 1.0f);
  float Uq = motorParams.voltage_power_supply * motorParams.open_loop_voltage_ratio * voltage_ratio;
  
  // 死区补偿和方向处理
  Uq *= direction;
  if(fabs(Uq) < 3.0f && target_velocity != 0) {
      Uq = (direction > 0) ? 3.0f : -3.0f;
  }  
  
  motorParams.open_loop_timestamp = now_us;
  return Uq;
}

// ====================== 修改后的电机控制任务 ======================
void MotorControlTask(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t last_us = micros();

  // 定义启动状态机
  enum StartupState { WAIT_STABLE, RAMP_UP, RUNNING };
  StartupState startup_state = WAIT_STABLE;
  
  // 定义各个启动阶段的持续时间
  const TickType_t wait_duration = pdMS_TO_TICKS(500); // 等待稳定阶段
  const TickType_t ramp_duration = pdMS_TO_TICKS(500); // 渐进过渡阶段
  
  ctrl.current_mode = ControlModule::POSITION;
  // 启动阶段时间记录
  TickType_t startup_start_time = xTaskGetTickCount();

  for (;;) {
    uint32_t now_us = micros();
    float dt = (now_us - last_us) * 1e-6f;
    last_us = now_us;

    // 更新传感器数据（使用 sensorMutex）
    xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
      sensor.Sensor_update();
      ctrl.sensor.position = sensor.getAngle();
      ctrl.sensor.mech_angle = sensor.getMechanicalAngle();
      ctrl.sensor.velocity = sensor.getVelocity();
    xSemaphoreGiveRecursive(sensorMutex);

  
      current_sense.getPhaseCurrents();
      ctrl.current.Ia = current_sense.current_a;
      ctrl.current.Ib = current_sense.current_b;
      ctrl.current.Ic = current_sense.current_c;
   
    // 计算当前电角度（包含零电角偏置，并归一化至 [0, 2π]）
    float electrical_angle = motorParams.direction * _electricalAngle(ctrl.sensor.mech_angle, 7) + motorParams.zero_electric_angle;
    electrical_angle = _normalizeAngle(electrical_angle);
    motorParams.electrical_angle = electrical_angle;
    ctrl.current.Iq = computeIq(ctrl.current.Ia, ctrl.current.Ib, electrical_angle);


    // 状态机处理启动阶段逻辑
    switch (startup_state)
    {
    case WAIT_STABLE: {
        if ((xTaskGetTickCount() - startup_start_time) < wait_duration) {
            // 等待阶段：输出0电压稳定磁场
            setPhaseVoltage(0, 0, electrical_angle);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
            continue;
        } else {
            // 等待阶段结束：初始化PID参数和目标位置，并切换到渐进阶段
        xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
          ctrl.target.position = ctrl.sensor.mech_angle;
          ctrl.pid.basic.position.integral = 0;
          ctrl.pid.basic.position.prev_error = 0;
          ctrl.pid.basic.velocity.integral = 0;
          ctrl.pid.basic.velocity.prev_error = 0;
          ctrl.pid.basic.current.integral = 0;
          ctrl.pid.basic.current.prev_error = 0;
        xSemaphoreGiveRecursive(globalMutex);

        

        startup_state = RAMP_UP;
            startup_start_time = xTaskGetTickCount(); // 重置计时用于渐进阶段
        }
      }
      break;
    case RAMP_UP: {
        TickType_t elapsed = xTaskGetTickCount() - startup_start_time;
        float ramp_factor = (elapsed < ramp_duration) ? (elapsed / (float)ramp_duration) : 1.0f;
        
        // 根据不同模式计算闭环输出
        float Uq = 0.0f;
        ControlModule::Mode currentMode = ctrl.current_mode;
        float target_value = (currentMode == ControlModule::POSITION) ? ctrl.target.position : ctrl.target.velocity;
        switch(currentMode) {
          case ControlModule::POSITION:
            Uq = positionClosedLoop(target_value, dt);
            break;
          case ControlModule::VELOCITY:
            Uq = velocityClosedLoop(target_value, dt);
            break;
          case ControlModule::OPEN_LOOP:
            Uq = openLoopControl(target_value);
            break;
          case ControlModule::CURRENT:
            Uq = currentClosedLoop(ctrl.target.current, dt, electrical_angle, ctrl.pid.basic.current.Kp, ctrl.pid.basic.current.Ki);
            break;
          case ControlModule::CURRENT_POSITION:
            Uq = positionCurrentClosedLoop(ctrl.target.position, dt);
            break;
          case ControlModule::CURRENT_VELOCITY:
            Uq = velocityCurrentClosedLoop(ctrl.target.velocity, dt);
            break;
          case ControlModule::CASCADE_POS_VEL_CUR:
            Uq = cascadePositionVelocityCurrentLoop(ctrl.target.position, dt);
            break;
        }
        
        // 渐进输出：控制输出与 ramp_factor 相乘，实现从0到正常闭环输出的平滑过渡
        Uq *= ramp_factor;
        setPhaseVoltage(Uq, 0, electrical_angle);
        
        // 当 ramp_factor 达到1后，切换至正常闭环控制
        if (ramp_factor >= 1.0f) {
            startup_state = RUNNING;
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
        continue;
      }
      break;
    case RUNNING:
      // 正常闭环控制，无需额外处理
      break;
    }
    
    // 正常运行状态下，根据当前模式计算 Uq
    ControlModule::Mode currentMode = ctrl.current_mode;
    float Uq = 0.0f;
    switch(currentMode) {
      case ControlModule::POSITION:
        Uq = positionClosedLoop(ctrl.target.position, dt);
        break;
      case ControlModule::VELOCITY:
        Uq = velocityClosedLoop(ctrl.target.velocity, dt);

        break;
      case ControlModule::OPEN_LOOP:
        Uq = openLoopControl(ctrl.target.openloop_velocity);
        break;
      case ControlModule::CURRENT:
        Uq = currentClosedLoop(ctrl.target.current, dt, electrical_angle, ctrl.pid.basic.current.Kp, ctrl.pid.basic.current.Ki);
        break;
      case ControlModule::CURRENT_POSITION:
        Uq = positionCurrentClosedLoop(ctrl.target.position, dt);
        break;
      case ControlModule::CURRENT_VELOCITY:
        Uq = velocityCurrentClosedLoop(ctrl.target.velocity, dt);
        break;
      case ControlModule::CASCADE_POS_VEL_CUR:
        Uq = cascadePositionVelocityCurrentLoop(ctrl.target.position, dt);
        break;
    }
    
    setPhaseVoltage(Uq, 0, electrical_angle);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}


// 数据打印任务：集中打印电流与编码器数据，方便调试
void DataPrintTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    float data[VOFA_CH_COUNT]; // 确保数组大小足够
    
    // 第一部分：电流相关数据
      data[0] = ctrl.current.Iq;       
      data[1] = phaseVolt.Ua;          
      data[2] = phaseVolt.Ub;           
      data[3] = phaseVolt.Uc;          

    // 第二部分：传感器数据
    xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
      data[4] = sensor.getVelocity();               
      data[5] = sensor.getAngle() * 180.0f / PI;     
      data[6] = sensor.getMechanicalAngle() * 180.0f / PI; 
    xSemaphoreGiveRecursive(sensorMutex);

    // 第三部分：其他数据（假设后续通道有其他用途）
    data[7] = ctrl.current_mode;
    data[8] = ctrl.target.position * 180.0f / PI;
    data[9] = ctrl.target.velocity;
    data[10] = ctrl.target.current;


    vofa(data);  // 发送数据
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setControlMode(ControlModule::Mode new_mode) {
  // 进入临界区
  xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
  // 切换前重置积分项
  ctrl.pid.basic.position.integral = 0;
  ctrl.pid.basic.velocity.integral = 0;
  ctrl.pid.basic.current.integral = 0;
  ctrl.pid.advanced.position_current.pos_integral = 0;
  ctrl.pid.advanced.velocity_current.vel_integral = 0;
  ctrl.pid.advanced.cascade.pos_integral = 0;
  ctrl.pid.advanced.cascade.vel_integral = 0;
  ctrl.current_mode = new_mode;
  xSemaphoreGiveRecursive(globalMutex); // 离开临界区


}


// 修改后的串口读取任务：SerialReadTask()，根据逗号分割数据
void SerialReadTask(void *pvParameters) {
  (void) pvParameters;
  String inputBuffer = "";
  while (true) {
    while (Serial.available() > 0) {
      char inChar = Serial.read();
      if (inChar == '\n') {
        inputBuffer.trim();
        if (inputBuffer.length() > 0) {
          // 如果接收到模式命令，则切换模式
          if (inputBuffer.startsWith("mode:")) {
            int mode = inputBuffer.substring(5).toInt();
            xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
              if(mode == 1) setControlMode(ControlModule::POSITION);
              else if(mode == 2) setControlMode(ControlModule::VELOCITY);
              else if(mode == 3) setControlMode(ControlModule::OPEN_LOOP);
              else if(mode == 4) setControlMode(ControlModule::CURRENT);
              else if(mode == 5) setControlMode(ControlModule::CURRENT_POSITION);
              else if(mode == 6) setControlMode(ControlModule::CURRENT_VELOCITY);
              else if(mode == 7) setControlMode(ControlModule::CASCADE_POS_VEL_CUR);
            xSemaphoreGiveRecursive(globalMutex);
          }
          else { // 根据逗号分割数据，更新控制目标
            int firstDelim = inputBuffer.indexOf(',');
            int secondDelim = inputBuffer.indexOf(',', firstDelim + 1);
            int thirdDelim = inputBuffer.indexOf(',', secondDelim + 1);
            if (firstDelim != -1 && secondDelim != -1 && thirdDelim != -1) {
                String token1 = inputBuffer.substring(0, firstDelim);
                String token2 = inputBuffer.substring(firstDelim + 1, secondDelim);
                String token3 = inputBuffer.substring(secondDelim + 1, thirdDelim);
                String token4 = inputBuffer.substring(thirdDelim + 1);
                float val1 = token1.toFloat();
                float val2 = token2.toFloat();
                float val3 = token3.toFloat();
                float val4 = token4.toFloat();

                xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
                  switch(ctrl.current_mode) {
                    case ControlModule::POSITION:
                    case ControlModule::CASCADE_POS_VEL_CUR:
                    case ControlModule::CURRENT_POSITION:
                      ctrl.target.position = val1;
                      break;
                    case ControlModule::VELOCITY:
                    case ControlModule::CURRENT_VELOCITY:
                      ctrl.target.velocity = val2;
                      break;
                    case ControlModule::OPEN_LOOP:
                      ctrl.target.openloop_velocity = val3;
                      break;
                    case ControlModule::CURRENT:
                      ctrl.target.current = val4;
                      break;

                  }
                xSemaphoreGiveRecursive(globalMutex);
            }
          }
        }
        inputBuffer = "";
      } else if (inChar != '\r') {
        inputBuffer += inChar;
      }
    }
    if (inputBuffer.startsWith("MAX_CUR:")) {
      float max_cur = inputBuffer.substring(7).toFloat();
      xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
        ctrl.limits.max_current = max_cur;
      xSemaphoreGiveRecursive(globalMutex);
    }
    if (inputBuffer.startsWith("MIN_CUR:")) {
      float min_cur = inputBuffer.substring(7).toFloat();
      xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
        ctrl.limits.min_current = min_cur;
      xSemaphoreGiveRecursive(globalMutex);
    }
    if (inputBuffer.startsWith("CASCADE_VEL_LIMIT:")) {
      float vel_limit = inputBuffer.substring(18).toFloat();
      xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
        ctrl.limits.cascade_limits.pos_vel_limit = fabs(vel_limit); // 确保为正值
      xSemaphoreGiveRecursive(globalMutex);

    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// 通用电流限制函数
float applyCurrentLimit(float target) {
  float abs_target = fabs(target);
  
  // 先应用最大值限制
  float limited = _constrain(target, -ctrl.limits.max_current, ctrl.limits.max_current );
  
  // 再应用最小值限制（死区）
  if (abs_target < ctrl.limits.min_current) {
    return 0.0f; // 当目标电流小于最小值时，输出0
  }
  return limited ;
}

// 新增电流有效性判断
bool isCurrentValid(float current) {
  float abs_current = fabs(current);
  return (abs_current >= ctrl.limits.min_current && 
          abs_current <= ctrl.limits.max_current );
}

