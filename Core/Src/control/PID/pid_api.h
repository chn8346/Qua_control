//
// Created by fanchunhui on 2022/11/13.
//

#ifndef QUA_CONTROL_PID_API_H
#define QUA_CONTROL_PID_API_H

#include "pid_base.h"

// PID参数
#define P_Default   0.1
#define I_Default   0.2
#define D_Default   0.04

// 模型选择，模型直接决定了控制方案的不同
#define CONTROL_PID_MODEL_DIV_FLAT      1       // 使用微分平坦模型
#define CONTROL_PID_MODEL_CLASSIC       2       // 使用经典控制模型
#define CONTROL_PID_MODEL_STAY          3       // 角度环稳定

#define CONTROL_PID_MODEL       CONTROL_PID_MODEL_CLASSIC

// 这里使用宏定义，确定算法使用的类型
#if CONTROL_PID_MODEL == CONTROL_PID_MODEL_DIV_FLAT   // 使用微分平坦


#elif CONTROL_PID_MODEL == CONTROL_PID_MODEL_CLASSIC  // 使用经典模型

// PID API 的数据索引
#define PID_API_INDEX_POSX   0
#define PID_API_INDEX_POSY   1
#define PID_API_INDEX_POSZ   2
#define PID_API_INDEX_PITCH  3
#define PID_API_INDEX_ROLL   4
#define PID_API_INDEX_YAW    5

// 手动模式还是自动模式
#define AUTO_PILOT_ON        0x00
#define AUTO_PILOT_OFF       0x0F

class pid_API{
    public:
    // 对控制器进行初始化，确定参数和目标
    void init_PID_API(float* pos_x_pid_parameter,
                      float* pos_y_pid_parameter,
                      float* pos_z_pid_parameter,
                      float* pitch_pid_parameter,
                      float* roll_pid_parameter,
                      float* yaw_pid_parameter,
                      float* target_pos_n_angle);

    // 写入目标参数
    void INPUT_target(float pos_angle_target[6]);
    void INPUT_target(int PID_API_INDEX, float value);    // 重载 针对单个目标修改

    // 写入状态估计值
    void INPUT_estimate(float pos_angle_esti[6]);

    // 总控制函数
    void PROCESS_update();

    // 获取输出值（油门、力和力矩）
    void get_OUTPUT(float Qua_out[4] ,float force_moment_out[6]);

    // 设置操作模式
    void set_Auto_pilot(){this->auto_pilot_mode=AUTO_PILOT_ON;}      // 启动自动飞行
    void clear_Auto_pilot(){this->auto_pilot_mode=AUTO_PILOT_OFF;}    // 关闭自动飞行

    private:
    void traject_gen_control();  // 轨迹生成
    void position_control();     // 位置控制(后续会直接控制姿态)
    void altitude_control();    // 姿态和高度控制

    // 单信号控制器
    pid_base x_pid;
    pid_base y_pid;
    pid_base z_pid;
    pid_base pitch_pid;
    pid_base roll_pid;
    pid_base yaw_pid;

    // 临时变量成员，存放目标值和估计值
    float target_pos_angle[6];
    float estimate_pos_angle[6];

    // target 和 estimate 采集情况
    uint8_t get_data_condition = 0;  // 0：啥也没有  1：有target  2:有estimate

    // 手动模式还是自动飞行模式
    uint8_t auto_pilot_mode = AUTO_PILOT_OFF;

};

// 通过遥控器百分比,生成需要的目标值
void PID_API_remote_control_gen_target(float* ppm_phased, float * target);

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
