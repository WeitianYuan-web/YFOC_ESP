#include <Arduino.h>
#include "UserControl/UserControl.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "Serial/vofa.h"

UserControl userControl0;    // 电机0
ControlModule::Mode current_mode0 = ControlModule::SINGLE_POSITION;
ControlParams User_control_params0;
UserControl userControl1;    // 电机0
ControlModule::Mode current_mode1 = ControlModule::SINGLE_POSITION;
ControlParams User_control_params1;

MotorData data0;
MotorData data1;

// 添加FreeRTOS相关变量
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t dataTaskHandle = NULL;
SemaphoreHandle_t xMutex = xSemaphoreCreateRecursiveMutex();

// 电机控制任务
void MotorControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;) {
        uint32_t dt = userControl0.updateDt();
        xSemaphoreTakeRecursive(xMutex, portMAX_DELAY);
        userControl0.update_getSensorData();
        userControl1.update_getSensorData();
        xSemaphoreGiveRecursive(xMutex);
        userControl0.update(current_mode0, data0.position, 0, 0,
                           User_control_params0.pos_vel_limit, User_control_params0.max_current, dt);
        userControl1.update(current_mode1, data0.position, 0, 0,
                           User_control_params0.pos_vel_limit, User_control_params0.max_current, dt);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));  // 1ms延时
    }
}

// 数据发送任务
void DataSendTask(void *pvParameters) {
   for (;;) {
    float data[VOFA_CH_COUNT]; // 初始化数组
    xSemaphoreTakeRecursive(xMutex, portMAX_DELAY);
    
    // 第一部分：电流相关数据
    data[0] = userControl0.currentPosition();    // 建议使用真实数据
    data[1] = userControl0.currentIq();
    data[2] = userControl0.currentVelocity();           
    data[3] = userControl1.currentPosition();          
    data[4] = userControl1.currentIq();               
    data[5] = userControl1.currentVelocity();     
    vofa(data);  // 发送数据
    xSemaphoreGiveRecursive(xMutex);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);

    // 创建全局递归互斥锁
  xMutex = xSemaphoreCreateRecursiveMutex();
  if (xMutex == NULL) {
    Serial.println("创建全局互斥锁失败！");
    while (1);
  }else{
    Serial.println("互斥锁创建成功");
  }

  pinMode(12,OUTPUT);
  digitalWrite(12,HIGH);

  Wire.begin(19, 18, 800000);
  Wire1.begin(23, 5, 800000);

  // 单电机模式：只初始化 motor0
  userControl0.begin(0, &Wire, 0x36, 39, 36, -1, 32, 33, 25, 1, 0, 1, 2);
  userControl1.begin(0, &Wire1, 0x36, 35, 34, -1, 27, 26, 14, 1, 3, 4, 5);
  userControl0.update_getSensorData();
  userControl1.update_getSensorData();
  data0.position = userControl0.currentMechanicalAngle();
  data1.position = userControl1.currentMechanicalAngle();
 
  // 创建任务
  xTaskCreatePinnedToCore(
      MotorControlTask,   // 任务函数
      "MotorCtrl",        // 任务名称
      8192,               // 堆栈大小
      NULL,               // 参数
      3,                  // 优先级（0-24，数字越大优先级越高）
      &motorTaskHandle,   // 任务句柄
      1);                 // 运行在核心1（避免与无线模块冲突）

  xTaskCreatePinnedToCore(
      DataSendTask,
      "DataSend",
      1024,
      NULL,
      2,                  // 提升到与电机任务相同优先级
      &dataTaskHandle,
      0);
}

void loop() {
    // FreeRTOS接管任务调度，保持空循环
    vTaskDelete(NULL);
}