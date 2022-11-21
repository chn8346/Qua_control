//
// Created by fanchunhui on 2022/11/9.
//

// app included here

#ifndef QUA_CONTROL_HEAD_H
#define QUA_CONTROL_HEAD_H

// 工具
#include "toolkit/pid_control.hpp" // TODO 【待删除】 测试库，用处不大，后续可以删掉

// 状态观测器/滤波器
#include "estimate/Mahony/MahonyAHRS.h"
#include "estimate/Wick/MadgwickAHRS.h"
#include "estimate/EKF/EKF_update.h"

// 控制器
#include "control/control_api.h"    // 最上层飞控API
#include "control/PID/pid_base.h"   // pid底层
#include "control/PID/pid_api.h"    // pid和其他控制器的联合API
#include "control/ADRC/track_differentiator.h"  // 追踪微分器

// 数学库
#include "math/matrix.h"      // 矩阵计算库
#include "math/pre_process.h" // 用于起飞前的传感器校准
#include "math/crc16.h"       // crc16校验库

// 传感器接口/驱动
#include "sensor_socket/sensor_calibrate.h" // 校准库
#if 1

// IMU
#include "sensor_socket/gyro_acc/ICM20602.h"
#include "sensor_socket/gyro_acc/MPU6050.h"

// 磁力计
#include "sensor_socket/mag/Mag3110.h"
#include "sensor_socket/mag/IST8310.h"
#include "sensor_socket/mag/HMC5883.h"

// 气压计
#include "sensor_socket/pressure/SPL06.h"
#include "sensor_socket/pressure/BMP180.h"

// GPS
#include "sensor_socket/GPS/BN880.h"

#else
#include "sensor_socket/sensor_socket.h"
#endif

// 通信
#include "TR/data_feedback.h"  // 暂未解析
#include "TR/NGroundStation.h" // 无名地面站通信
#include "TR/usart_plus.h"     // uart封装

#include "sensor_socket/i2c/hard_i2c.h" // 硬件I2C工具包
#include "sensor_socket/spi/spi.h"      // spi工具包

#endif // QUA_CONTROL_HEAD_H
