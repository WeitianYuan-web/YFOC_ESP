#include <Arduino.h>
#include "InlineCurrent.h"
#include "vofa.h"
#include "AS5600.h"
#include "Wire.h"
#include <math.h>
#include "freertos/semphr.h"   // 添加 FreeRTOS 互斥锁支持
#include "ControlModule/ControlModule.h"


// 宏定义约束函数
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

void MotorControlTask(void *pvParameters);
void DataPrintTask(void *pvParameters);
void SerialReadTask(void *pvParameters);
void UserAppTask(void *pvParameters);

ControlModule motor(0); // 电机编号
// 全局控制参数
ControlParams User_control_params;

// 在全局范围新增两个互斥锁
SemaphoreHandle_t sensorMutex = xSemaphoreCreateRecursiveMutex();  // 传感器数据专用

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

  // 板载信号灯初始化
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  delay(10);

  motor.initSensors(); // 使用默认参数

  motor.calibrateZeroPoint();

  // 创建FreeRTOS任务
  xTaskCreatePinnedToCore(MotorControlTask, "MotorControlTask", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(DataPrintTask, "DataPrintTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(SerialReadTask, "SerialReadTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(UserAppTask, "UserAppTask", 4096, NULL, 1, NULL, 1);

}

void loop() {
  // 空的 loop 函数，任务由 FreeRTOS 调度
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
  
  motor.current_mode = ControlModule::POSITION;
  // 启动阶段时间记录
  TickType_t startup_start_time = xTaskGetTickCount();

  for (;;) {
    uint32_t now_us = micros();
    float dt = (now_us - last_us) * 1e-6f;
    last_us = now_us;

    // 更新传感器数据
    motor.updateSensorData();

  
    // 计算当前电角度（包含零电角偏置，并归一化至 [0, 2π]）
    float electrical_angle = motor.getNormalizedElectricalAngle();



    // 状态机处理启动阶段逻辑
    switch (startup_state)
    {
    case WAIT_STABLE: {
        if ((xTaskGetTickCount() - startup_start_time) < wait_duration) {
            // 等待阶段：输出0电压稳定磁场
            motor.openLoopControl(0);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
            continue;
        } else {
            // 等待阶段结束：初始化PID参数和目标位置，并切换到渐进阶段
        xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
          motor.target.position = motor.sensor.mech_angle;
          User_control_params.target_pos = motor.sensor.mech_angle;
          motor.pid.basic.position.integral = 0;
          motor.pid.basic.position.prev_error = 0;
          motor.pid.basic.velocity.integral = 0;
          motor.pid.basic.velocity.prev_error = 0;
          motor.pid.basic.current.integral = 0;
          motor.pid.basic.current.prev_error = 0;
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
        ControlModule::Mode currentMode = motor.current_mode;
        switch(currentMode) {
          case ControlModule::POSITION:
            Uq = motor.positionClosedLoop(motor.target.position, dt);
            break;
          case ControlModule::VELOCITY:
            Uq = motor.velocityClosedLoop(motor.target.velocity, dt);
            break;
          case ControlModule::OPEN_LOOP:
            Uq = motor.openLoopControl(motor.target.openloop_velocity);
            break;
          case ControlModule::CURRENT:
            Uq = motor.currentClosedLoop(motor.target.current, dt, electrical_angle, motor.pid.basic.current.Kp, motor.pid.basic.current.Ki);
            break;
          case ControlModule::CURRENT_POSITION:
            Uq = motor.PositionCurrentClosedLoop(motor.target.position, dt);
            break;
          case ControlModule::CURRENT_VELOCITY:
            Uq = motor.VelocityCurrentClosedLoop(motor.target.velocity, dt);
            break;
          case ControlModule::CASCADE_POS_VEL_CUR:
            Uq = motor.PositionVelocityCurrentLoop(motor.target.position, dt);
            break;
        }
        
        // 渐进输出：控制输出与 ramp_factor 相乘，实现从0到正常闭环输出的平滑过渡
        Uq *= ramp_factor;
        motor.setPhaseVoltage(Uq, 0, electrical_angle);
        
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
    ControlModule::Mode currentMode = motor.current_mode;
    float Uq = 0.0f;
    switch(currentMode) {
      case ControlModule::POSITION:
        Uq = motor.positionClosedLoop(motor.target.position, dt);
        break;
      case ControlModule::VELOCITY:
        Uq = motor.velocityClosedLoop(motor.target.velocity, dt);
        break;
      case ControlModule::OPEN_LOOP:
        Uq = motor.openLoopControl(motor.target.openloop_velocity);
        break;
      case ControlModule::CURRENT:
        Uq = motor.currentClosedLoop(motor.target.current, dt, electrical_angle, motor.pid.basic.current.Kp, motor.pid.basic.current.Ki);
        break;
      case ControlModule::CURRENT_POSITION:
        Uq = motor.PositionCurrentClosedLoop(motor.target.position, dt);
        break;
      case ControlModule::CURRENT_VELOCITY:
        Uq = motor.VelocityCurrentClosedLoop(motor.target.velocity, dt);
        break;
      case ControlModule::CASCADE_POS_VEL_CUR:
        Uq = motor.PositionVelocityCurrentLoop(motor.target.position, dt);
        break;
    }
    
    motor.setPhaseVoltage(Uq, 0, electrical_angle);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}


// 数据打印任务：集中打印电流与编码器数据，方便调试
void DataPrintTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    float data[VOFA_CH_COUNT]; // 确保数组大小足够
    
    // 第一部分：电流相关数据
      data[0] = motor.current.Iq;         

    // 第二部分：传感器数据
    xSemaphoreTakeRecursive(sensorMutex, portMAX_DELAY);
      data[1] = motor.sensor.velocity;               
      data[2] = motor.sensor.position * 180.0f / PI;     
      data[3] = motor.sensor.mech_angle * 180.0f / PI; 
    xSemaphoreGiveRecursive(sensorMutex);

    xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
    // 第三部分：其他数据（假设后续通道有其他用途）
    data[4] = motor.current_mode;
    data[5] = User_control_params.target_pos * 180.0f / PI;
    data[6] = User_control_params.target_vel;
    data[7] = User_control_params.target_cur;
    data[8] = User_control_params.max_current;
    data[9] = User_control_params.pos_vel_limit;
    xSemaphoreGiveRecursive(globalMutex);

    vofa(data);  // 发送数据
    vTaskDelay(pdMS_TO_TICKS(10));
  }
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
          // 模式切换命令
          if (inputBuffer.startsWith("mode:")) {
            int new_mode = inputBuffer.substring(5).toInt();
            xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
              motor.current_mode = static_cast<ControlModule::Mode>(new_mode);
            xSemaphoreGiveRecursive(globalMutex);
          }
          // 参数设置命令（格式：pos,vel,cur,limit）
          else if (inputBuffer.startsWith("param:")) {
            String paramStr = inputBuffer.substring(6);
            int delim1 = paramStr.indexOf(',');
            int delim2 = paramStr.indexOf(',', delim1+1);
            int delim3 = paramStr.indexOf(',', delim2+1);
            int delim4 = paramStr.indexOf(',', delim3+1);

            if (delim1 != -1 && delim2 != -1 && delim3 != -1 && delim4 != -1) {
              xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
                User_control_params.target_pos = paramStr.substring(0, delim1).toFloat();
                User_control_params.target_vel = paramStr.substring(delim1+1, delim2).toFloat();
                User_control_params.target_cur = paramStr.substring(delim2+1, delim3).toFloat();
                User_control_params.max_current = paramStr.substring(delim3+1, delim4).toFloat();
                User_control_params.pos_vel_limit = paramStr.substring(delim4+1).toFloat();
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

// 新增用户自定义任务
void UserAppTask(void *pvParameters) {
    (void) pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
            // 创建临时参数副本
    ControlModule::Mode current_mode;
    current_mode = ControlModule::POSITION;
    ControlParams current_params;
    current_params.target_vel = 10.0f;
    current_params.target_cur = 0.5f;
    current_params.max_current = 0.9f;
    for (;;) {

        // 唯一执行设置的地方
          motor.set_control_mode(current_mode, current_params);
          
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

