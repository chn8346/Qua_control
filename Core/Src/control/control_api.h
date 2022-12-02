//
// Created by fanchunhui on 2022/11/9.
//

#ifndef QUA_CONTROL_CONTROL_API_H
#define QUA_CONTROL_CONTROL_API_H

#define CONTROL_MODE_PID  0
#define CONTROL_MODE_ADRC 1

#define CONTROL_MODE    CONTROL_MODE_PID

// 使用PID控制器
#if CONTROL_MODE == CONTROL_MODE_PID
#include "PID/pid_api.h"   // 导入PID API

// 控制器
class controller{
public:
    // 写入pid参数进行初始化
    void control_init(uint8_t controller_type,
                      float* pos_x_pid_parameter_,
                      float* pos_y_pid_parameter_,
                      float* pos_z_pid_parameter_,
                      float* pitch_pid_parameter_,
                      float* roll_pid_parameter_,
                      float* yaw_pid_parameter_);

    // 控制器根据观测值进行解算
    void control_GO(float* estimate_pos_angle);

    // 更改目标
    void change_target(float* target_pos_angle);
    void change_target(uint8_t index, float target_pos_angle);

    // 算法类型识别
    const uint8_t controller = CONTROLLER_TYPE_PID;
private:
    pid_API pid_api;

};

#elif CONTROL_MODE == CONTROL_MODE_ADRC // 使用ADRC控制器

int x // 如果选择错误，这里会报错

#endif

#endif //QUA_CONTROL_CONTROL_API_H
