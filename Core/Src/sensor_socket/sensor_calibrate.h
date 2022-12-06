//
// Created by fanchunhui on 2022/11/13.
//

#ifndef QUA_CONTROL_SENSOR_CALIBRATE_H
#define QUA_CONTROL_SENSOR_CALIBRATE_H

// 传感器校准库


#include "../base_head.h"
#include "sensor_socket.h"

// calibrate parameter 中的参数位置
#define CLB_ACCX    0
#define CLB_ACCY    1
#define CLB_ACCZ    2
#define CLB_GX      3
#define CLB_GY      4
#define CLB_GZ      5
#define CLB_MAGX    6
#define CLB_MAGY    7
#define CLB_MAGZ    8
#define CLB_PSR     9   // 气压计

// 返回的数据类型
typedef struct {
    float mid[10];
    float angle_gain_rad[3]; // 角速度转换乘积
    float mag_limit[6];  // 磁力计的极值，奇数是下界，偶数是上界
    float mag_half_range[3]; // 磁力计的半数值范围
    float gravity_value;
}calibrate_parameter;

uint8_t str_len_cbl(const char * x);
void calibrate_get_i2c_spi(I2C_HandleTypeDef* i2c1,
                           I2C_HandleTypeDef* i2c2,
                           SPI_HandleTypeDef* spi1,
                           SPI_HandleTypeDef* spi2);
void calibrate_at_init(UART_HandleTypeDef* uart);
void calibrate_at_init();       // 不加参数，自动使用经验参数启动
void psr_calibrate_at_flying();
void calibrate_return_data(float * origin_acc, float * origin_angle, float * origin_mag, float q1, float q2, float q3, float q4);
void uart_transmit_with_next_sensor_clb(UART_HandleTypeDef* uart, uint8_t* data, uint8_t len);

// 根据重力和加速度信息对重力和拉力进行分离，获得更准确的状态信息
void gravity_altitude_uni_estimate(const float* gravity_norm, float* altitude_or_q, float yaw);

// 对Q-X构型进行二次数据加工
void calibrate_QX(float * origin_acc, float * origin_angle, float * origin_mag);

// TODO 【急】 基于姿态对磁力计数据进行补偿

// 用于快速init使用的经验数值

#define CBL_MPU6050_ACC_GYRO_BIAS       {550.0, 1500.0, -2900.0, -64.8, 294.0, -32.0}
#define CBL_MPU6050_ANGLE_RATE_GAIN     {0.00012, 0.00014, 0.00012}
#define CBL_MPU6050_GRAVITY_VALUE       176.0

#define CBL_MAG3110_BIAS                {27800.0, 103.0, -2005.0}
#define CBL_MAG3110_HALF_RANGE          {45.0, 310.0, 375.0}

#define CBL_HMC5883_BIAS                {15.0000, 477.5000, -109.5000}
#define CBL_HMC5883_HALF_RANGE          {365.0000, 105.5000, 391.5000}

#endif //QUA_CONTROL_SENSOR_CALIBRATE_H
