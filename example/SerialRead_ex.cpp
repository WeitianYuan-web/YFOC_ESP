#include <Arduino.h>
#include "Serial/SerialRead.h"

SerialReader serialReader;

void setup() {
  Serial.begin(115200);
  // 其他初始化...
}

void loop() {
  // 调用 readCommand() 检查是否有有效的串口指令
  SerialCommand cmd = serialReader.readCommand();
  if (cmd.commandType == SERIAL_COMMAND_MODE) {
    // 处理模式切换，例如设置 current_mode = cmd.mode
    Serial.print("收到模式切换指令: ");
    Serial.println(cmd.mode);
    // ... 更新全局变量或进行其他处理
  }
  else if (cmd.commandType == SERIAL_COMMAND_PARAM) {
    // 处理参数设置命令
    Serial.println("收到参数设置指令:");
    Serial.print("target_pos: "); Serial.println(cmd.target_pos);
    Serial.print("target_vel: "); Serial.println(cmd.target_vel);
    Serial.print("target_cur: "); Serial.println(cmd.target_cur);
    Serial.print("max_current: "); Serial.println(cmd.max_current);
    Serial.print("pos_vel_limit: "); Serial.println(cmd.pos_vel_limit);
    // ... 更新全局参数或进行其他处理
  }

  // 注意：为了避免 busy loop ，可以适当延时
  delay(10);
}