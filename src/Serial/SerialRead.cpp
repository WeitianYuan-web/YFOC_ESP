#include "SerialRead.h"

SerialReader::SerialReader() : inputBuffer("") {
}

SerialCommand SerialReader::readCommand() {
    SerialCommand cmd;
    // 默认没有可用指令
    cmd.commandType = SERIAL_COMMAND_NONE;
    // 如果电机编号未解析，则设置默认为0
    cmd.motor_id = 0; 

    // 轮询读取串口数据
    while (Serial.available() > 0) {
        char inChar = Serial.read();
        if (inChar == '\n') {  // 遇到换行符表示一条完整命令结束
            inputBuffer.trim();
            if (inputBuffer.length() > 0) {
                // 先检查是否包含 motor_id 信息（例如 "id:0,"）
                if (inputBuffer.startsWith("id:")) {
                    int commaIndex = inputBuffer.indexOf(',');
                    if (commaIndex != -1) {
                        String idStr = inputBuffer.substring(3, commaIndex);
                        cmd.motor_id = idStr.toInt();
                        // 去掉 motor_id 部分，剩下的命令内容
                        inputBuffer = inputBuffer.substring(commaIndex + 1);
                    }
                }
                
                // 模式切换指令，例如："mode:1"
                if (inputBuffer.startsWith("mode:")) {
                    String modeStr = inputBuffer.substring(5);
                    cmd.commandType = SERIAL_COMMAND_MODE;
                    cmd.mode = modeStr.toInt();
                }
                // 参数设置指令，格式："param:target_pos,target_vel,target_cur,max_current,pos_vel_limit"
                else if (inputBuffer.startsWith("param:")) {
                    String paramStr = inputBuffer.substring(6);
                    int delim1 = paramStr.indexOf(',');
                    int delim2 = paramStr.indexOf(',', delim1 + 1);
                    int delim3 = paramStr.indexOf(',', delim2 + 1);
                    int delim4 = paramStr.indexOf(',', delim3 + 1);
                    if (delim1 != -1 && delim2 != -1 && delim3 != -1 && delim4 != -1) {
                        cmd.commandType = SERIAL_COMMAND_PARAM;
                        cmd.target_pos = paramStr.substring(0, delim1).toFloat();
                        cmd.target_vel = paramStr.substring(delim1 + 1, delim2).toFloat();
                        cmd.target_cur = paramStr.substring(delim2 + 1, delim3).toFloat();
                        cmd.max_current = paramStr.substring(delim3 + 1, delim4).toFloat();
                        cmd.pos_vel_limit = paramStr.substring(delim4 + 1).toFloat();
                    }
                }
            }
            // 处理完后清空缓存准备下一条指令
            inputBuffer = "";
            break; // 此处只返回一条完整的命令（如果有的话）
        }
        else if (inChar != '\r') {  // 忽略'\r'
            inputBuffer += inChar;
        }
    }
    return cmd;
}
