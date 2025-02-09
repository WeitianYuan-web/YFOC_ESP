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

// 宏定义约束函数
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// 定义存储电源参数与电机状态的结构体
struct MotorParameters {
    float voltage_power_supply = 12.6f;  // 供电电压
    float voltage_limit = 12.6f;         // 最大输出电压限制
    float shaft_angle = 0.0f;            // 机械轴角
    float open_loop_timestamp = 0.0f;    // 开环更新时间戳
    float zero_electric_angle = 0.0f;    // 电机零电角
    float electrical_angle = 0.0f;        // 电角度
    int direction = -1; // 电机旋转方向系数
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

// 常量定义
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define DIRECTION_FORWARD -1


// 前置函数声明
float _electricalAngle(float shaft_angle, int pole_pairs);
float _normalizeAngle(float angle);
void setPwm(float Ua, float Ub, float Uc);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
float velocityOpenloop(float target_velocity);
float computeIq(float Ia, float Ib, float electrical_angle);

// FreeRTOS任务函数声明
void MotorControlTask(void *pvParameters);
void SensorUpdateTask(void *pvParameters);
void DataPrintTask(void *pvParameters);
void SerialReadTask(void *pvParameters);

// ====================== 新增控制模块结构体 ======================
struct ControlModule {
  enum Mode { 
    POSITION = 1,  // 位置模式
    VELOCITY = 2,  // 速度模式 
    OPEN_LOOP = 3, // 开环模式
    CURRENT = 4    // 电流闭环模式
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

    float velocity;      // 速度模式/开环模式目标（rad/s）
    float voltage;       // 预留字段
    float current;       // 电流闭环模式目标（目标Iq）
  } target;

  // PID参数：位置控制、速度控制和新增电流闭环控制参数
  struct {
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
    
    // 新增电流闭环参数，通常电流闭环使用 PI 控制（可根据需要调整Kp和Ki）
    struct {
      float Kp = 5.0f;    // 电流闭环比例增益
      float Ki = 500.0f;   // 电流闭环积分增益
      float Kd = 0.0f;    // 通常电流闭环只采用 PI 控制，D 项置0
      float integral = 0;
      float prev_error = 0;
    } current;
  } pid;

  // 其他参数
  struct {
    float k_ff = 0.09f;  // 速度前馈增益
    float alpha = 0.2f;  // 电流低通滤波系数 (0 < alpha < 1, 越小滤波越强)
  } filter;

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
        ctrl.pid.position.integral += error * dt;
        ctrl.pid.position.integral = _constrain(ctrl.pid.position.integral, -100.0f, 100.0f);
    } else {
        ctrl.pid.position.integral = 0.0f;
    }



    // PID微分部分
    float derivative = (error - ctrl.pid.position.prev_error) / dt;
    ctrl.pid.position.prev_error = error;

    float control_output = ctrl.pid.position.Kp * error 
         + ctrl.pid.position.Ki * ctrl.pid.position.integral
         + ctrl.pid.position.Kd * derivative;
    return control_output;


}

// 改进后的速度闭环计算（使用 PID + 前馈控制）
float velocityClosedLoop(float target_vel, float dt) {
    // 防止 dt 非法（例如首次调用或者系统抖动导致 dt 为 0）
    if (dt <= 0) return 0.0f;
    
    // 计算速度误差：目标速度 - 实际速度（单位：rad/s）
    float error = target_vel - ctrl.sensor.velocity;
    
    // 积分项累加（同时限幅防止积分饱和）
    ctrl.pid.velocity.integral += error * dt;
    ctrl.pid.velocity.integral = _constrain(ctrl.pid.velocity.integral, -500.0f, 500.0f);
    
    // 计算微分项（当前误差与上一误差的变化率）
    float derivative = (error - ctrl.pid.velocity.prev_error) / dt;
    ctrl.pid.velocity.prev_error = error;
    
    // 计算前馈补偿：直接利用目标速度生成控制量
    float feedforward = ctrl.filter.k_ff * target_vel;
    
    // PID 控制输出 + 前馈补偿

    float control_output = ctrl.pid.velocity.Kp * error 
                           + ctrl.pid.velocity.Ki * ctrl.pid.velocity.integral 
                           + ctrl.pid.velocity.Kd * derivative
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
float currentClosedLoop(float target_current, float dt, float electrical_angle) {
    // 计算原始 Iq
    float measured_Iq = computeIq(ctrl.current.Ia , ctrl.current.Ib, electrical_angle);
    

    // 对 Iq 进行低通滤波
    ctrl.current.Iq_filtered = ctrl.filter.alpha * measured_Iq 
                             + (1 - ctrl.filter.alpha) * ctrl.current.Iq_filtered;
    
    // 使用滤波后的 Iq 进行闭环控制
    float error = target_current - ctrl.current.Iq_filtered;

    ctrl.pid.current.integral += error * dt;
   
    ctrl.pid.current.prev_error = error;
    

    float control_output = ctrl.pid.current.Kp * error + ctrl.pid.current.Ki * ctrl.pid.current.integral;
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
  ctrl.target.current = 0.1f;
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
          ctrl.pid.position.integral = 0;
          ctrl.pid.position.prev_error = 0;
          ctrl.pid.velocity.integral = 0;
          ctrl.pid.velocity.prev_error = 0;
          ctrl.pid.current.integral = 0;
          ctrl.pid.current.prev_error = 0;
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
            Uq = currentClosedLoop(ctrl.target.current, dt, electrical_angle);
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
        Uq = currentClosedLoop(ctrl.target.current, dt, electrical_angle);
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
    data[8] = ctrl.target.position;
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
  ctrl.pid.position.integral = 0;
  ctrl.pid.velocity.integral = 0;
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
                      ctrl.target.position = val1;
                      break;
                    case ControlModule::VELOCITY:
                      ctrl.target.velocity = val2;
                      break;
                    case ControlModule::OPEN_LOOP:
                      ctrl.target.velocity = val3;
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
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

