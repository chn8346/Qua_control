//
// Created by fanchunhui on 2022/11/9.
//

#ifndef QUA_CONTROL_SENSOR_SOCKET_H
#define QUA_CONTROL_SENSOR_SOCKET_H

#include "../base_head.h"

#include "gyro_acc/MPU6050.h"
#include "gyro_acc/ICM20602.h"

#include "mag/Mag3110.h"
#include "mag/IST8310.h"
#include "mag/HMC5883.h"

#include "pressure/SPL06.h"
#include "pressure/BMP180.h"

#include "GPS/BN880.h"

#include "ultra_sonic/Ultra_sonic.h"

/*
 *      可选传感器参数,后续定义会进行选择
 * */
#define SENSOR_SELECT_IMU_MU6050        0x00
#define SENSOR_SELECT_IMU_ICM20602      0x01

#define SENSOR_SELECT_GPS_BN880         0x10

#define SENSOR_SELECT_MAG_HMC5883       0x20
#define SENSOR_SELECT_MAG_IST8310       0x21
#define SENSOR_SELECT_MAG_MAG3110       0x22

#define SENSOR_SELECT_PSR_BMP180        0x30
#define SENSOR_SELECT_PSR_SPL06         0x31

#define SENSOR_SELECT_SONIC_US1         0x40

#define SENSOR_SELECT_NONE              0xFF    // 表示此选项不选择传感器



/*
 *      实际使用的传感器参数
 * */

#define SENSOR_SELECT_IMU               SENSOR_SELECT_IMU_MU6050
#define SENSOR_SELECT_GPS               SENSOR_SELECT_GPS_BN880
#define SENSOR_SELECT_MAG               SENSOR_SELECT_MAG_HMC5883
#define SENSOR_SELECT_PSR               SENSOR_SELECT_PSR_BMP180
#define SENSOR_SELECT_SONIC             SENSOR_SELECT_SONIC_US1

/*
 *      函数接口定义
 * */

void IMU_init(I2C_HandleTypeDef* i2c);
void IMU_read_data(uint8_t* data_int, float* data_f);
// void IMU_write_data(uint8_t reg, uint8_t* data_int);

void MAG_init(I2C_HandleTypeDef* i2c);
void MAG_read_data(uint8_t* data_int, float* data_f);

void GPS_init(I2C_HandleTypeDef* i2c);
void GPS_read_data(uint8_t* data_int, float* data_f);

void PSR_init(I2C_HandleTypeDef* i2c);
void PSR_read_data(uint8_t* data_int, float* data_f);

void SNC_init(I2C_HandleTypeDef* i2c);
void SNC_read_data(uint8_t* data_int, float* data_f);


#endif //QUA_CONTROL_SENSOR_SOCKET_H
