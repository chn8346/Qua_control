//
// Created by fanchunhui on 2022/11/9.
//

#include "control_api.h"

// 使用PID控制器
#if CONTROL_MODE == CONTROL_MODE_PID
void controller::control_init(uint8_t controller_type, float *pos_x_pid_parameter_, float *pos_y_pid_parameter_,
                              float *pos_z_pid_parameter_, float *pitch_pid_parameter_, float *roll_pid_parameter_,
                              float *yaw_pid_parameter_) {
    // 正确的类型才能使用
    if(controller_type == CONTROLLER_TYPE_PID)
    {
        float target_zero[6] = {0,0,0,0,0,0};
        this->pid_api.init_PID_API(pos_x_pid_parameter_,
                                   pos_y_pid_parameter_,
                                   pos_z_pid_parameter_,
                                   pitch_pid_parameter_,
                                   roll_pid_parameter_,
                                   yaw_pid_parameter_,
                                   target_zero);
    }
}

void controller::control_GO(float *estimate_pos_angle) {

}

void controller::change_target(float *target_pos_angle) {
    this->pid_api.INPUT_target(target_pos_angle);
}

void controller::change_target(uint8_t index, float target_pos_angle) {
    this->pid_api.INPUT_target(index, target_pos_angle);
}


#elif CONTROL_MODE == CONTROL_MODE_ADRC // 使用ADRC控制器


#endif