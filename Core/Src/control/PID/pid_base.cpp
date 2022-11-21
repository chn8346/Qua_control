//
// Created by fanchunhui on 2022/11/13.
//

#include "pid_base.h"

void pid_base::target_give(float target) {
    this->target_get = 1;
    this->target_get_value = target;
}

// PID 处理
void pid_base::pid_update(float estimate) {
    this->estimate_value = estimate;
    this->estimate_get = 1;

    // 有目标值才继续进行计算
    if (this->target_get) {
        this->err = this->target_get_value - this->estimate_value;

        // P update
        this->P_value   = this->err                       *  this->P;

        // I update
        this->Int_value = this->Int_value + this->err     *  this->I;

        // D update
        this->D_value   = (this->err - this->last_state)  *  this->D;
        this->last_state = this->err;

        // 限幅计算
        this->limit_process();

        // output update
        this->out_put_value = this->P_value
                            + this->Int_value
                            + this->D_value;

    }

    this->estimate_get = 0;
}

void pid_base::limit_process() {
    // P 限幅
    float px =  this->P * this->err - this->P_limit;     // 正 ...
    float px_ = this->P * this->err + this->P_limit;    // 负限幅位置检测
    // 正限幅
    if(px > 0)
    {
        this->p_limit_flag = this->p_limit_flag + 1;
        this->Int_value = this->P_limit;
    }
    // 负限幅
    else if(px_ < 0)
    {
        this->p_limit_flag = this->p_limit_flag + 1;
        this->Int_value = -this->P_limit;
    }


    // I 限幅
    px =  this->Int_value - this->I_limit;
    px_ = this->Int_value + this->I_limit;
    // 正限幅
    if(px > 0)
    {
        this->i_limit_flag = this->i_limit_flag + 1;
        this->Int_value = this->I_limit;
    }
    // 负限幅
    else if(px_ < 0)
    {
        this->i_limit_flag = this->i_limit_flag + 1;
        this->Int_value = -this->I_limit;
    }


    // D 限幅
    px =  (this->err - this->last_state) * this->D - this->D_limit;
    px_ = (this->err - this->last_state) * this->D + this->D_limit;
    // 正限幅
    if(px > 0)
    {
        this->d_limit_flag = this->d_limit_flag + 1;
        this->D_value = this->D_limit;
    }
    // 负限幅
    else if(px_ < 0)
    {
        this->d_limit_flag = this->d_limit_flag + 1;
        this->D_value = -this->D_limit;
    }

}

void pid_base::pid_get(float* output) {
    *output = this->out_put_value;
}

void pid_base::reset_() {
    this->target_get = 0;    // 获取给定值后为1
    this->estimate_get = 0;  // 获取估计值之后为1
    this->err = {0};          // 误差

    // PID积累量
    this->Int_value = 0;              // 积分积累量
    this->last_state = {0}; // 微分--上次参数

    // 输出值
    this->out_put_value={0};

    this->p_limit_flag = 0;
    this->i_limit_flag = 0;
    this->d_limit_flag = 0;
}

void pid_base::pid_para_change(float p, float i, float d) {
    this->P = p;
    this->I = i;
    this->D = d;
}
