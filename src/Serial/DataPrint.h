

void dataprint (){
    for (;;) {
    MotorData data = userControl.getSensorData();
    float dataArray[10];
    // 电流数据
    dataArray[0] = data.current_Iq;
    // 传感器数据
    dataArray[1] = data.velocity;
    dataArray[2] = data.position * 180.0f / PI;
    dataArray[3] = data.mech_angle * 180.0f / PI;
    
    // 其他数据（模式及全局参数）
    //xSemaphoreTakeRecursive(globalMutex, portMAX_DELAY);
      dataArray[4] = static_cast<float>(current_mode);
      dataArray[5] = User_control_params.target_pos * 180.0f / PI;
      dataArray[6] = User_control_params.target_vel;
      dataArray[7] = User_control_params.target_cur;
      dataArray[8] = User_control_params.max_current;
      dataArray[9] = User_control_params.pos_vel_limit;
    //xSemaphoreGiveRecursive(globalMutex);

    vofa(dataArray);  // 发送数据
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}