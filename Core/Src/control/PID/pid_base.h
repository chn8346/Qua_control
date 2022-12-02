//
// Created by fanchunhui on 2022/11/13.
//

#ifndef QUA_CONTROL_PID_BASE_H
#define QUA_CONTROL_PID_BASE_H

#include "../../base_head.h"

class pid_base{
public:
    // pid参数
    float P = 1;
    float I = 0;
    float D = 0;

    // 限幅(正负限幅)
    float P_limit = 2;
    float I_limit = 10;
    float D_limit = 1;

    // 限幅标志位,表示被限幅次数
    int p_limit_flag = 0;
    int i_limit_flag = 0;
    int d_limit_flag = 0;

    void reset_();                      // 重置参数
    void target_give(float target);     // 给定量进入
    void pid_update(float estimate);    // 估计量进入，并进行pid参数计算
    void pid_get(float* output);         // 给出具体输出，方便后续环节进行控制，接下来的控制内容交给pid_api
    void pid_para_change(const float* pid);    // PID 参数更换
    void pid_para_change(float p, float i, float d);    // PID 参数更换(3 float 变量版本重载)

private:
    char target_get = 0;    // 获取给定值后为1
    char estimate_get = 0;  // 获取估计值之后为1
    float err = 0;          // 误差
    float target_get_value = 0; // 目标值
    float estimate_value = 0;     // 估计值

    // PID积累量
    float Int_value = 0;  // 积分积累量
    float D_value = 0;
    float P_value = 0;
    float last_state = 0; // 微分--上次参数

    // 输出值
    float out_put_value= 0;

    void limit_process();   // 限幅

};


#endif //QUA_CONTROL_PID_BASE_H
