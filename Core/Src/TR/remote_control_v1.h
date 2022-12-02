//
// Created by fanchunhui on 2022/11/26.
//

#ifndef QUA_CONTROL_REMOTE_CONTROL_V1_H
#define QUA_CONTROL_REMOTE_CONTROL_V1_H

#include "../base_head.h"

#define REMOTE_CONTROL_MID_PPM_us       1350    // 整数--- 遥控器的中位值
#define REMOTE_CONTROL_DEATH_PPM_us     20      // 整数--- 遥控器的死区半径
#define REMOTE_CONTROL_LOW_PPM_us       900     // 整数--- 遥控器的pwm最小值
#define REMOTE_CONTROL_TOP_PPM_us       1800    // 整数--- 遥控器的pwm最大值
#define REMOTE_CONTROL_DIAMETER_PPM_us  900     // 整数--- 遥控器的pwm直径
#define REMOTE_CONTROL_RADIUS_PPM_us    450     // 整数--- 遥控器的pwm半径
#define REMOTE_CONTROL_FRAME_LEN_PPM_us 2000    // 整数--- 单帧的最大值

#define REMOTE_CONTROL_MID_PPM_usf          (float)1350.0    // 浮点--- 遥控器的中位值
#define REMOTE_CONTROL_DEATH_PPM_usf        (float)20.0      // 浮点--- 遥控器的死区半径
#define REMOTE_CONTROL_LOW_PPM_usf          (float)900.0     // 浮点--- 遥控器的pwm最小值
#define REMOTE_CONTROL_TOP_PPM_usf          (float)1800.0    // 浮点--- 遥控器的pwm最大值
#define REMOTE_CONTROL_DIAMETER_PPM_usf     (float)900.0     // 浮点--- 遥控器的pwm直径
#define REMOTE_CONTROL_RADIUS_PPM_usf       (float)450.0     // 浮点--- 遥控器的pwm半径
#define REMOTE_CONTROL_FRAME_LEN_PPM_usf    (float)2000.0    // 浮点--- 单帧的最大值


// 遥控器解析
// 1 由于PWM容易漏数据跑偏，这个程序的作用就是找到信号的头部
// 2 数据没有百分比信息，从这里对数据进行整理
void remote_control_phase(uint32_t* ppm_data, float* ppm_phased);

// 获取信息头的位置
uint16_t REMOTE_CONTROL_get_zero_index(uint32_t* data, uint16_t len);

// 遥控器所有数据中找最大值
void remote_max_in_list(uint32_t* data, uint16_t len);

#endif //QUA_CONTROL_REMOTE_CONTROL_V1_H
