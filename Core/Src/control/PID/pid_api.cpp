//
// Created by fanchunhui on 2022/11/13.
//

#include "pid_api.h"

// 这里使用宏定义，减少编译的代码量
#if CONTROL_PID_MODEL == CONTROL_PID_MODEL_DIV_FLAT   // 使用微分平坦

    int y;

#elif CONTROL_PID_MODEL == CONTROL_PID_MODEL_CLASSIC  // 使用经典模型
// 还没开工，先写微分平坦的内容

    iny z;

#elif CONTROL_PID_MODEL == CONTROL_PID_MODEL_STAY // 角度控制

// 3-D(pitch\roll\yaw)角度控制器
pid_base pitch_controller, roll_controller, yaw_controller;

// 姿态控制器
// qs : 当前状态
// target : 目标状态
// allocate : 旋翼分配
void angle_control(qua_state* qs, qua_state * target, float allocate[4])
{
    // 状态
    float* angle      = qs->angle;      // 角度状态
    float* angle_rate = qs->angle_rate; // 角速度
    float* angle_acc  = qs->angle_acc;  // 角加速度

    // 目标
    float* target_a   = target->angle;  // 目标角度
    float* target_ar  = target->angle_rate; // 目标角速度限制
    float* target_ac  = target->angle_acc;  // 目标角加速度限制

    // 分配初始化模长 = 1
    allocate[0] = 0.5; allocate[1] = 0.5; allocate[2] = 0.5; allocate[3] = 0.5;

    // 给定目标
    pitch_controller.target_give(target_a[0]);
    roll_controller.target_give(target_a[1]);
    yaw_controller.target_give(target_a[2]);

    // PID参数给定
    pitch_controller.pid_para_change(P, I, D);
    roll_controller.pid_para_change(P, I, D);
    yaw_controller.pid_para_change(P, I, D);

    // 更新
    pitch_controller.pid_update(angle[0]);
    roll_controller.pid_update(angle[1]);
    yaw_controller.pid_update(angle[2]);
}

#endif
