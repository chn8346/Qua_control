//
// Created by fanchunhui on 2022/11/13.
//

#ifndef QUA_CONTROL_PID_API_H
#define QUA_CONTROL_PID_API_H

#include "pid_base.h"

// PID参数
#define P   0.1
#define I   0.2
#define D   0.04

// 模型选择，模型直接决定了控制方案的不同
#define CONTROL_PID_MODEL_DIV_FLAT      1       // 使用微分平坦模型
#define CONTROL_PID_MODEL_CLASSIC       2       // 使用经典控制模型
#define CONTROL_PID_MODEL_STAY          3       // 角度环稳定

#define CONTROL_PID_MODEL       CONTROL_PID_MODEL_STAY

// 这里使用宏定义，减少编译的代码量
#if CONTROL_PID_MODEL == CONTROL_PID_MODEL_DIV_FLAT   // 使用微分平坦

    int yl;

#elif CONTROL_PID_MODEL == CONTROL_PID_MODEL_CLASSIC  // 使用经典模型


    // 还没开工，先写微分平坦的内容
    int stat;

#elif CONTROL_PID_MODEL == CONTROL_PID_MODEL_STAY   // 角度环稳定测试

    // 姿态控制器
    //
    // qs : 当前状态
    // target : 目标状态
    // allocate : 旋翼分配
    //
    void angle_control(qua_state* qs, qua_state * target, float allocate[4]);

#endif

#endif //QUA_CONTROL_PID_API_H
