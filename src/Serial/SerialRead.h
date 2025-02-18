#ifndef SERIAL_READ_H
#define SERIAL_READ_H

#include <Arduino.h>

// 定义串口指令的类型
enum SerialCommandType {
    SERIAL_COMMAND_NONE = 0,   // 当前无可用指令
    SERIAL_COMMAND_MODE,       // 模式切换命令
    SERIAL_COMMAND_PARAM       // 参数设置命令
};

// 定义返回的指令结构体
struct SerialCommand {
    SerialCommandType commandType;
    int motor_id;          // 新增字段：目标电机编号（例如 0 或 1）
    int mode;              // 当 commandType == SERIAL_COMMAND_MODE 时有效
    float target_pos;      // 当 commandType == SERIAL_COMMAND_PARAM 时有效
    float target_vel;
    float target_cur;
    float max_current;
    float pos_vel_limit;
};

class SerialReader {
public:
    SerialReader();
    /**
     * 非阻塞式检查串口数据，若已读取完整命令（以'\n'结束），则解析并返回对应指令结构体，
     * 否则返回commandType为 SERIAL_COMMAND_NONE 的结构体。
     */
    SerialCommand readCommand();

private:
    String inputBuffer;
};

#endif // SERIAL_READ_H
