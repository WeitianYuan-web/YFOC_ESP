#include <Arduino.h>
#include "UserControl/UserControl.h"
#include "vofa.h"
#include "Wire.h"
#include <math.h>
#include "freertos/semphr.h"
#include "Serial/SerialRead.h"

// -------------- 宏定义 --------------
// 定义 USE_DUAL_MOTOR 为1启用双电机模式，设置为0则仅使用一个电机
#define USE_DUAL_MOTOR 1

// -------------- 全局变量 --------------
SemaphoreHandle_t globalMutex = NULL;

#if USE_DUAL_MOTOR
// 双电机模式：实例化两个 UserControl 对象和对应的模式、参数
UserControl userControl0;    // 电机0
UserControl userControl1;    // 电机1

ControlModule::Mode current_mode0 = ControlModule::POSITION;
ControlModule::Mode current_mode1 = ControlModule::POSITION;

ControlParams User_control_params0;
ControlParams User_control_params1;
#else
// 单电机模式：只实例化 motor0
UserControl userControl0;    // 电机0
ControlModule::Mode current_mode0 = ControlModule::POSITION;
ControlParams User_control_params0;
#endif

// FreeRTOS任务函数声明
void ControlTask(void *pvParameters);
void DataPrintTask(void *pvParameters);
void SerialReadTask(void *pvParameters);

void setup() {
  Serial.begin(115200);
  delay(10);

  // 创建全局互斥锁
  globalMutex = xSemaphoreCreateRecursiveMutex();
  if (globalMutex == NULL) {
    Serial.println("创建全局互斥锁失败！");
    while (1);
  }

  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);


#if USE_DUAL_MOTOR
  Wire.begin(19, 18, 800000);
  Wire1.begin(23, 5, 800000);

  // 初始化双电机
  userControl0.begin(0, &Wire, 0x36, 39, 36, -1, 32, 33, 25, 0, 1, 2);
  userControl1.begin(1, &Wire1, 0x36, 39, 36, -1, 32, 33, 25, 0, 1, 2);

  // 初始化参数：以电机0当前读取的位置为基准（两个电机可共用或分别初始化）
  xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
    MotorData data0 = userControl0.getSensorData();
    User_control_params0.target_pos = data0.position;
    User_control_params0.target_vel = 0;
    User_control_params0.target_cur = 0;
    User_control_params0.max_current = 1.0f;    
    User_control_params0.pos_vel_limit = 10.0f;
    
    // 电机1也以电机0的当前位置信息初始化（根据需要可以修改）
    User_control_params1.target_pos = data0.position;
    User_control_params1.target_vel = 0;
    User_control_params1.target_cur = 0;
    User_control_params1.max_current = 1.0f;    
    User_control_params1.pos_vel_limit = 10.0f;
  xSemaphoreGiveRecursive(globalMutex);
#else
  Wire.begin(19, 18, 800000);

  // 单电机模式：只初始化 motor0
  userControl0.begin(0, &Wire, 0x36, 39, 36, -1, 32, 33, 25, 0, 1, 2);
  
  xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
    MotorData data0 = userControl0.getSensorData();
    User_control_params0.target_pos = data0.position;
    User_control_params0.target_vel = 0;
    User_control_params0.target_cur = 0;
    User_control_params0.max_current = 1.0f;    
    User_control_params0.pos_vel_limit = 10.0f;
  xSemaphoreGiveRecursive(globalMutex);
#endif

  // 创建FreeRTOS任务
  xTaskCreatePinnedToCore(ControlTask,    "ControlTask",    8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(DataPrintTask,    "DataPrintTask",  2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(SerialReadTask,   "SerialReadTask", 2048, NULL, 1, NULL, 1);
}

void loop() {
  // 空的loop，所有任务由FreeRTOS调度
}


// --------------------------------------------------------------------------
// ControlTask: 根据全局模式与参数调用UserControl接口进行闭环控制
// 这里保持单/双电机模式的处理一致，具体是否控制两台电机由宏控制
void ControlTask(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  // 启动状态机相关定义略（与之前代码类似）
  enum StartupState { WAIT_STABLE, RAMP_UP, RUNNING };
  StartupState startup_state = WAIT_STABLE;
  const TickType_t wait_duration = pdMS_TO_TICKS(500);
  const TickType_t ramp_duration = pdMS_TO_TICKS(500);
  TickType_t state_start_time = xTaskGetTickCount();

  for (;;) {
    TickType_t now = xTaskGetTickCount();
    switch(startup_state) {
      case WAIT_STABLE:
        if ((now - state_start_time) < wait_duration) {
          // 启动等待阶段：使用OPEN_LOOP输出0电压来稳定
#if USE_DUAL_MOTOR
          userControl0.update(ControlModule::OPEN_LOOP, 0, 0, 0, 0, 0);
          userControl1.update(ControlModule::OPEN_LOOP, 0, 0, 0, 0, 0);
#else
          userControl0.update(ControlModule::OPEN_LOOP, 0, 0, 0, 0, 0);
#endif
        } else {
#if USE_DUAL_MOTOR
          MotorData data = userControl0.getSensorData();
          xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
            User_control_params0.target_pos = data.position;
            User_control_params1.target_pos = data.position;
          xSemaphoreGiveRecursive(globalMutex);
#else
          MotorData data = userControl0.getSensorData();
          xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
            User_control_params0.target_pos = data.position;
          xSemaphoreGiveRecursive(globalMutex);
#endif
          startup_state = RAMP_UP;
          state_start_time = now;
        }
        break;
      
      case RAMP_UP:
      {
          TickType_t elapsed = now - state_start_time;
          float ramp_factor = (elapsed < ramp_duration) ? (elapsed / (float)ramp_duration) : 1.0f;
#if USE_DUAL_MOTOR
          xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
            ControlModule::Mode mode0 = current_mode0;
            ControlModule::Mode mode1 = current_mode1;
            ControlParams params0 = User_control_params0;
            ControlParams params1 = User_control_params1;
          xSemaphoreGiveRecursive(globalMutex);
          userControl0.update(mode0, params0.target_pos, params0.target_vel, params0.target_cur,
                             params0.pos_vel_limit, params0.max_current, ramp_factor);
          userControl1.update(mode1, params1.target_pos, params1.target_vel, params1.target_cur,
                             params1.pos_vel_limit, params1.max_current, ramp_factor);
#else
          xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
            ControlModule::Mode mode = current_mode0;
            ControlParams params = User_control_params0;
          xSemaphoreGiveRecursive(globalMutex);
          userControl0.update(mode, params.target_pos, params.target_vel, params.target_cur,
                              params.pos_vel_limit, params.max_current, ramp_factor);
#endif
          if (ramp_factor >= 1.0f) {
            startup_state = RUNNING;
          }
      }
      break;
      
      case RUNNING:
#if USE_DUAL_MOTOR
          xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
            ControlModule::Mode mode0 = current_mode0;
            ControlModule::Mode mode1 = current_mode1;
            ControlParams params0 = User_control_params0;
            ControlParams params1 = User_control_params1;
          xSemaphoreGiveRecursive(globalMutex);
          userControl0.update(mode0, params0.target_pos, params0.target_vel, params0.target_cur,
                             params0.pos_vel_limit, params0.max_current);
          userControl1.update(mode1, params1.target_pos, params1.target_vel, params1.target_cur,
                             params1.pos_vel_limit, params1.max_current);
#else
          xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
            ControlModule::Mode mode = current_mode0;
            ControlParams params = User_control_params0;
          xSemaphoreGiveRecursive(globalMutex);
          userControl0.update(mode, params.target_pos, params.target_vel, params.target_cur,
                              params.pos_vel_limit, params.max_current);
#endif
        break;
    }
    
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
  }
}


// --------------------------------------------------------------------------
// DataPrintTask: 采集并发送电机数据，依据宏选择单/双电机模式
void DataPrintTask(void *pvParameters) {
  (void) pvParameters;
#if USE_DUAL_MOTOR
  // 每个电机输出10个数据，构造包含两个电机数据的数组，共20个浮点数，其中第一个元素为电机编号
  for (;;) {
    float dataArray[20];

    // 电机0数据
    MotorData data0 = userControl0.getSensorData();
    dataArray[0]  = 0;  // 电机编号
    dataArray[1]  = data0.current_Iq;
    dataArray[2]  = data0.velocity;
    dataArray[3]  = data0.position * 180.0f / PI;
    dataArray[4]  = data0.mech_angle * 180.0f / PI;
    xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
      dataArray[5]  = static_cast<float>(current_mode0);
      dataArray[6]  = User_control_params0.target_pos * 180.0f / PI;
      dataArray[7]  = User_control_params0.target_vel;
      dataArray[8]  = User_control_params0.target_cur;
      dataArray[9]  = User_control_params0.max_current;
    xSemaphoreGiveRecursive(globalMutex);

    // 电机1数据
    MotorData data1 = userControl1.getSensorData();
    dataArray[10] = 1;  // 电机编号
    dataArray[11] = data1.current_Iq;
    dataArray[12] = data1.velocity;
    dataArray[13] = data1.position * 180.0f / PI;
    dataArray[14] = data1.mech_angle * 180.0f / PI;
    xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
      dataArray[15] = static_cast<float>(current_mode1);
      dataArray[16] = User_control_params1.target_pos * 180.0f / PI;
      dataArray[17] = User_control_params1.target_vel;
      dataArray[18] = User_control_params1.target_cur;
      dataArray[19] = User_control_params1.max_current;
    xSemaphoreGiveRecursive(globalMutex);

    vofa(dataArray);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
#else
  // 单电机模式：只采集电机0的数据，数组包含10个数据
  for (;;) {
    float dataArray[10];
    MotorData data0 = userControl0.getSensorData();
    dataArray[0]  = 0;  // 电机编号（单电机时默认为0）
    dataArray[1]  = data0.current_Iq;
    dataArray[2]  = data0.velocity;
    dataArray[3]  = data0.position * 180.0f / PI;
    dataArray[4]  = data0.mech_angle * 180.0f / PI;
    xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
      dataArray[5]  = static_cast<float>(current_mode0);
      dataArray[6]  = User_control_params0.target_pos * 180.0f / PI;
      dataArray[7]  = User_control_params0.target_vel;
      dataArray[8]  = User_control_params0.target_cur;
      dataArray[9]  = User_control_params0.max_current;
    xSemaphoreGiveRecursive(globalMutex);
    
    vofa(dataArray);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
#endif
}


// --------------------------------------------------------------------------
// SerialReadTask: 使用 SerialReader 解析串口指令，并根据宏选择更新单/双电机的控制模式与参数
void SerialReadTask(void *pvParameters) {
  (void) pvParameters;
  SerialReader serialReader;
  
  for (;;) {
    SerialCommand cmd = serialReader.readCommand();
#if USE_DUAL_MOTOR
    if (cmd.commandType == SERIAL_COMMAND_MODE) {
      if (cmd.motor_id == 0) {
        xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
          current_mode0 = static_cast<ControlModule::Mode>(cmd.mode);
        xSemaphoreGiveRecursive(globalMutex);
      } else if (cmd.motor_id == 1) {
        xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
          current_mode1 = static_cast<ControlModule::Mode>(cmd.mode);
        xSemaphoreGiveRecursive(globalMutex);
      }
    }
    else if (cmd.commandType == SERIAL_COMMAND_PARAM) {
      if (cmd.motor_id == 0) {
        xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
          User_control_params0.target_pos  = cmd.target_pos;
          User_control_params0.target_vel  = cmd.target_vel;
          User_control_params0.target_cur  = cmd.target_cur;
          User_control_params0.max_current = cmd.max_current;
          User_control_params0.pos_vel_limit = cmd.pos_vel_limit;
        xSemaphoreGiveRecursive(globalMutex);
      } else if (cmd.motor_id == 1) {
        xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
          User_control_params1.target_pos  = cmd.target_pos;
          User_control_params1.target_vel  = cmd.target_vel;
          User_control_params1.target_cur  = cmd.target_cur;
          User_control_params1.max_current = cmd.max_current;
          User_control_params1.pos_vel_limit = cmd.pos_vel_limit;
        xSemaphoreGiveRecursive(globalMutex);
      }
    }
#else
    // 单电机模式：忽略指令中的 motor_id，默认为电机0
    if (cmd.commandType == SERIAL_COMMAND_MODE) {
      xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
        current_mode0 = static_cast<ControlModule::Mode>(cmd.mode);
      xSemaphoreGiveRecursive(globalMutex);
    }
    else if (cmd.commandType == SERIAL_COMMAND_PARAM) {
      xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
        User_control_params0.target_pos  = cmd.target_pos;
        User_control_params0.target_vel  = cmd.target_vel;
        User_control_params0.target_cur  = cmd.target_cur;
        User_control_params0.max_current = cmd.max_current;
        User_control_params0.pos_vel_limit = cmd.pos_vel_limit;
      xSemaphoreGiveRecursive(globalMutex);
    }
#endif
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}