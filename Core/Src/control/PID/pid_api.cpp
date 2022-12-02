//
// Created by fanchunhui on 2022/11/13.
//

#include "pid_api.h"

// 这里使用宏定义，减少编译的代码量
#if CONTROL_PID_MODEL == CONTROL_PID_MODEL_DIV_FLAT   // 使用微分平坦

    int y

#elif CONTROL_PID_MODEL == CONTROL_PID_MODEL_CLASSIC  // 使用经典模型

void pid_API::init_PID_API(float* pos_x_pid_parameter,
                           float* pos_y_pid_parameter,
                           float* pos_z_pid_parameter,
                           float* pitch_pid_parameter,
                           float* roll_pid_parameter,
                           float* yaw_pid_parameter,
                           float* target_pos_n_angle) {
    // 修改PID参数
    this->x_pid.pid_para_change(pos_x_pid_parameter);
    this->y_pid.pid_para_change(pos_y_pid_parameter);
    this->z_pid.pid_para_change(pos_z_pid_parameter);
    this->pitch_pid.pid_para_change(pitch_pid_parameter);
    this->roll_pid.pid_para_change(roll_pid_parameter);
    this->yaw_pid.pid_para_change(yaw_pid_parameter);

    // 目标值保存到类内成员
    this->target_pos_angle[PID_API_INDEX_POSX]  = target_pos_n_angle[PID_API_INDEX_POSX];
    this->target_pos_angle[PID_API_INDEX_POSY]  = target_pos_n_angle[PID_API_INDEX_POSY];
    this->target_pos_angle[PID_API_INDEX_POSZ]  = target_pos_n_angle[PID_API_INDEX_POSZ];
    this->target_pos_angle[PID_API_INDEX_PITCH] = target_pos_n_angle[PID_API_INDEX_PITCH];
    this->target_pos_angle[PID_API_INDEX_ROLL]  = target_pos_n_angle[PID_API_INDEX_ROLL];
    this->target_pos_angle[PID_API_INDEX_YAW]   = target_pos_n_angle[PID_API_INDEX_YAW];

    // 修改目标值
    this->x_pid.target_give(this->target_pos_angle[PID_API_INDEX_POSX]);
    this->y_pid.target_give(this->target_pos_angle[PID_API_INDEX_POSY]);
    this->z_pid.target_give(this->target_pos_angle[PID_API_INDEX_POSZ]);
    this->pitch_pid.target_give(this->target_pos_angle[PID_API_INDEX_PITCH]);
    this->roll_pid.target_give(this->target_pos_angle[PID_API_INDEX_ROLL]);
    this->yaw_pid.target_give(this->target_pos_angle[PID_API_INDEX_YAW]);

    // 更改数据修改情况
    this->get_data_condition = 1;
}

void pid_API::INPUT_target(float pos_angle_target[6]) {
    // TODO 【后续需要补充】 角度限幅


    // 类内临时数值 赋值
    this->target_pos_angle[PID_API_INDEX_POSX] = pos_angle_target[PID_API_INDEX_POSX];
    this->target_pos_angle[PID_API_INDEX_POSY] = pos_angle_target[PID_API_INDEX_POSY];
    this->target_pos_angle[PID_API_INDEX_POSZ] = pos_angle_target[PID_API_INDEX_POSZ];
    this->target_pos_angle[PID_API_INDEX_PITCH] = pos_angle_target[PID_API_INDEX_PITCH];
    this->target_pos_angle[PID_API_INDEX_ROLL] = pos_angle_target[PID_API_INDEX_ROLL];
    this->target_pos_angle[PID_API_INDEX_YAW] = pos_angle_target[PID_API_INDEX_YAW];

    // 目标导入
    this->z_pid.target_give(this->target_pos_angle[PID_API_INDEX_POSZ]);
    this->yaw_pid.target_give(this->target_pos_angle[PID_API_INDEX_YAW]);
    this->roll_pid.target_give(this->target_pos_angle[PID_API_INDEX_ROLL]);
    this->pitch_pid.target_give(this->target_pos_angle[PID_API_INDEX_PITCH]);

    // 更改数据修改指示
    this->get_data_condition = this->get_data_condition | 0x01;
}

void pid_API::INPUT_target(int PID_API_INDEX, float value) {
    this->target_pos_angle[PID_API_INDEX] = value;

    // 更改数据修改情况
    this->get_data_condition = this->get_data_condition | 0x01;
}

void pid_API::INPUT_estimate(float pos_angle_esti[6]) {

    this->estimate_pos_angle[0] = pos_angle_esti[0];
    this->estimate_pos_angle[1] = pos_angle_esti[1];
    this->estimate_pos_angle[2] = pos_angle_esti[2];
    this->estimate_pos_angle[3] = pos_angle_esti[3];
    this->estimate_pos_angle[4] = pos_angle_esti[4];
    this->estimate_pos_angle[5] = pos_angle_esti[5];

    // 更改数据修改情况
    this->get_data_condition = this->get_data_condition | 0x02;
}

// 进行转速相关计算，两个参数都是输出值
void pid_API::get_OUTPUT(float Qua_out[4] ,float force_moment_out[6]) {
    // 获取输入  [force、pitch、roll、yaw],并返到 force_moment_out，6位只用4位
    this->z_pid.pid_get(force_moment_out+PID_API_INDEX_POSZ);
    this->pitch_pid.pid_get(force_moment_out+PID_API_INDEX_PITCH);
    this->roll_pid.pid_get(force_moment_out+PID_API_INDEX_ROLL);
    this->yaw_pid.pid_get(force_moment_out+PID_API_INDEX_YAW);
    float* u;
    u = force_moment_out;

    float force = u[PID_API_INDEX_POSZ]+(float)Qua_MASS*(float)9.8; // 加上重力，防止下落
    float pitch = u[PID_API_INDEX_PITCH];
    float roll  = u[PID_API_INDEX_ROLL];
    float yaw   = u[PID_API_INDEX_YAW];
    float p = pitch;
    float r = roll;
    float y = yaw;

#define shrink_K   ((float)0.75)
    // 力矩进行约束计算 -- 保持转速平方为正数
    if(force/2/Qua_Kf*shrink_K < (abs(yaw)/2/Qua_Km + (abs(roll) + abs(pitch))/Qua_Kf/Qua_DIAMETER)) {
        y   = yaw  *(force/2/Qua_Kf*shrink_K/(abs(yaw)/2/Qua_Km + (abs(roll)+abs(pitch))/Qua_Kf/Qua_DIAMETER));
        r   = roll *(force/2/Qua_Kf*shrink_K/(abs(yaw)/2/Qua_Km + (abs(roll)+abs(pitch))/Qua_Kf/Qua_DIAMETER));
        p   = pitch*(force/2/Qua_Kf*shrink_K/(abs(yaw)/2/Qua_Km + (abs(roll)+abs(pitch))/Qua_Kf/Qua_DIAMETER));
    }

    // 求解四旋翼转速
    Qua_out[0] = force/(float)4.0/Qua_Kf + y/(float)4.0/Qua_Km -  r/(float)2.0/Qua_Kf/Qua_DIAMETER;
    Qua_out[1] = force/(float)4.0/Qua_Kf - y/(float)4.0/Qua_Km + p/(float)2.0/Qua_Kf/Qua_DIAMETER;
    Qua_out[2] = force/(float)4.0/Qua_Kf + y/(float)4.0/Qua_Km +  r/(float)2.0/Qua_Kf/Qua_DIAMETER;
    Qua_out[3] = force/(float)4.0/Qua_Kf - y/(float)4.0/Qua_Km - p/(float)2.0/Qua_Kf/Qua_DIAMETER;

    // 油门比例进行等幅缩小(和油门gu)
    float sum = Qua_out[0] + Qua_out[1] + Qua_out[2] + Qua_out[3];
    float K = force/Qua_MASS/9.8*Qua_TAKE_OFF_P*4/(sum);
    // 开方得到转速数值
    Qua_out[0] = sqrt(Qua_out[0]*K);
    Qua_out[1] = sqrt(Qua_out[1]*K);
    Qua_out[2] = sqrt(Qua_out[2]*K);
    Qua_out[3] = sqrt(Qua_out[3]*K);

    // 限幅
    // 不大于1
    Qua_out[0] = Qua_out[0] > (float)1.0 ? (float)1.0 : Qua_out[0];
    Qua_out[1] = Qua_out[1] > (float)1.0 ? (float)1.0 : Qua_out[1];
    Qua_out[2] = Qua_out[2] > (float)1.0 ? (float)1.0 : Qua_out[2];
    Qua_out[3] = Qua_out[3] > (float)1.0 ? (float)1.0 : Qua_out[3];

    // 不小于0.1
    Qua_out[0] = Qua_out[0] < (float)0.1 ? (float)0.1 : Qua_out[0];
    Qua_out[1] = Qua_out[1] < (float)0.1 ? (float)0.1 : Qua_out[1];
    Qua_out[2] = Qua_out[2] < (float)0.1 ? (float)0.1 : Qua_out[2];
    Qua_out[3] = Qua_out[3] < (float)0.1 ? (float)0.1 : Qua_out[3];

}

// 总控制函数
void pid_API::PROCESS_update() {
    // 如果自动飞行开启，则进行下方操作
    if(this->auto_pilot_mode == AUTO_PILOT_ON) {
        // 轨迹生成
        this->traject_gen_control();

        // 位置控制
        this->position_control();
    }

    // 根据指定的高度和姿态要求，对姿态和高度进行控制
    this->altitude_control();

    // 修改数据状态
    this->get_data_condition = this->get_data_condition & 0xFD;
}

// 姿态控制器 -- 只完成解算，具体的动力分配交给output完成
void pid_API::altitude_control() {
    // 高度误差解算
    this->z_pid.pid_update(this->estimate_pos_angle[PID_API_INDEX_POSZ]);

    // pitch误差
    this->pitch_pid.pid_update(this->estimate_pos_angle[PID_API_INDEX_PITCH]);

    // roll误差
    this->roll_pid.pid_update(this->estimate_pos_angle[PID_API_INDEX_ROLL]);

    // yaw误差
    this->yaw_pid.pid_update(this->estimate_pos_angle[PID_API_INDEX_YAW]);

}

// 位置控制器
void pid_API::position_control() {

}

// 轨迹生成
void pid_API::traject_gen_control() {

}


#define PID_API_remote_gen_target_POSZ_LIMIT  1.0
#define PID_API_remote_gen_target_YAW_LIMIT   0.01

void PID_API_remote_control_gen_target(float* ppm_phased, float * target){
    // 高度 -- 高于起飞油门时刻，target是正的，否则是负的
    target[PID_API_INDEX_POSZ] = ppm_phased[PPM_CH_FORCE]*(float)PID_API_remote_gen_target_POSZ_LIMIT;

    // pitch -- 位置形式
    target[PID_API_INDEX_PITCH] = ppm_phased[PPM_CH_PITCH]*(float)PITCH_LIMIT_RAD;

    // roll -- 位置形式
    target[PID_API_INDEX_ROLL] = ppm_phased[PPM_CH_ROLL]*(float)ROLL_LIMIT_RAD;

    // yaw -- yaw是积分形式
    target[PID_API_INDEX_YAW] =
            target[PID_API_INDEX_YAW] + ppm_phased[PPM_CH_YAW]*(float)PID_API_remote_gen_target_YAW_LIMIT;

}

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
