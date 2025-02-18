#include <Arduino.h>
#include "UserControl/UserControl.h"

// 实例化 UserControl 对象，用于控制编号为 0 的电机
UserControl motorControl(0);

void setup() {
    Serial.begin(115200);
    delay(10);
    
    Wire.begin(19, 18, 400000);
    // 使用传入必要参数进行初始化：
    // mot_num, 编码器使用 I2C 接口, 编码器地址,
    // 当前检测引脚 (current_pinA, current_pinB, current_pinC), 
    // PWM 输出引脚 (pinA, pinB, pinC),
    // LEDC 通道号 (channelA, channelB, channelC)
    motorControl.begin(0,          // 电机编号
                       &Wire,      // 编码器所用 TwoWire 接口
                       0x36,       // 编码器 I2C 地址
                       39, 36, -1, // 电流检测引脚（当某引脚未使用时传 -1）
                       32, 33, 25, // PWM 输出引脚
                       0, 1, 2);   // LEDC 通道
}

void loop() {
    // 示例：调用位置控制模式，将目标位置设置为 1.57 弧度
    motorControl.positionMode(1.57f);
    
    // 通过 UserControl 获取传感器数据（含位置、速度、机械角度和电流 Iq）
    MotorData data = motorControl.getSensorData();
    
    // 将数据通过串口打印出来，便于调试和监控
    Serial.print("位置: ");
    Serial.print(data.position);
    Serial.print(" 弧度, 速度: ");
    Serial.print(data.velocity);
    Serial.print(" rad/s, 机械角度: ");
    Serial.print(data.mech_angle);
    Serial.print(" 弧度, 电流_Iq: ");
    Serial.print(data.current_Iq);
    Serial.println(" A");
    
    delay(100);  // 控制周期，可根据实际需求调整
}