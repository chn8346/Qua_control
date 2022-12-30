//
// Created by fanchunhui on 2022/11/9.
//

// app included here

#ifndef QUA_CONTROL_HEAD_H
#define QUA_CONTROL_HEAD_H

// 工具
#include "toolkit/pid_control.hpp" // TODO 【待删除】 测试库，用处不大，后续可以删掉
#include "toolkit/pos_velo_estimate_testv1.h" // 位置和速度测试库

// 状态观测器/滤波器
#include "estimate/Mahony/MahonyAHRS.h"
#include "estimate/Wick/MadgwickAHRS.h"
#include "estimate/EKF/EKF_update.h"

// 控制器
#include "control/control_api.h"    // 最上层飞控API
#include "control/PID/pid_base.h"   // pid底层
#include "control/PID/pid_api.h"    // pid和其他控制器的联合API
#include "control/ADRC/track_differentiator.h"  // 追踪微分器
#include "control/Filter_digital/digital_filter.h" // 数字滤波器
#include "control/PWM/PWM_generator.h"  // pwm生成器
#include "control/procession_operate/landing_process.h" // 过程控制： 降落流程
#include "control/procession_operate/test_hove.h"   // 控制过程： 测试悬停性能
#include "control/procession_operate/hove_alt_offset.h" // 在起飞阶段对

// 数学库
#include "math/matrix.h"      // 矩阵计算库
#include "math/pre_process.h" // 用于起飞前的传感器校准
#include "math/crc16.h"       // crc16校验库
#include "math/quat.h"

// 传感器接口/驱动
#include "sensor_socket/sensor_calibrate.h" // 校准库
#include "sensor_socket/spi/spi.h"
#include "sensor_socket/i2c/hard_i2c.h"
#include "sensor_socket/sensor_socket.h"

// 通信
#include "TR/data_feedback.h"  // 暂未解析
#include "TR/NGroundStation.h" // 无名地面站通信
#include "TR/usart_plus.h"     // uart封装

#include "sensor_socket/i2c/hard_i2c.h" // 硬件I2C工具包
#include "sensor_socket/spi/spi.h"      // spi工具包
#include "TR/remote_control_v1.h"   // 遥控器解析

#endif // QUA_CONTROL_HEAD_H
