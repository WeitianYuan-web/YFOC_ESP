#include <Arduino.h>
#include "ControlModule/ControlModule.h"
#include "Wire.h"

// 实例化两个 ControlModule 对象
ControlModule motor1(0);
ControlModule motor2(1);

void setup() {
    Serial.begin(115200);
    Wire.begin(19, 18, 400000);
    Wire1.begin(23, 5, 400000);

    // 对 motor1 使用 PWM 引脚 32,33,25，并分配 LEDc 通道 0,1,2
    motor1.hardwareInit(32, 33, 25, 0, 1, 2);
    // 对 motor2 使用 PWM 引脚 26,27,14，并分配 LEDc 通道 3,4,5
    motor2.hardwareInit(26, 27, 14, 3, 4, 5);
    
    // 初始化传感器（根据需要传入不同参数）
    motor1.initSensors(&Wire, 0x36, 39, 36, -1);
    motor2.initSensors(&Wire1, 0x36, 39, 36, -1);
    
    // 其它初始化操作……
}

void loop() {
    // 定时更新各自的传感器数据和控制输出
    motor1.updateSensorData();
    motor2.updateSensorData();
    Serial.println(motor1.sensor.position);
    Serial.println(motor2.sensor.position);
    delay(100);
}