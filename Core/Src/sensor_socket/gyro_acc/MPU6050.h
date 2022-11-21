//
// Created by fanchunhui on 2022/11/9.
//

#ifndef QUA_CONTROL_MPU6050_H
#define QUA_CONTROL_MPU6050_H

#include "../../base_head.h"

#define imu_addr (0x68<<1)
#define imu2_addr (0x69<<1)
#define MPU6050_RA_SMPLRT_DIV 			0x19
#define MPU6050_RA_CONFIG				0x1a
#define MPU6050_RA_ACCEL_CONFIG			0x1c
#define MPU6050_RA_GYRO_CONFIG			0x1B
#define MPU6050_RA_PWR_MGMT_1			0x6b
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

#define bias_WX   0 			//-3.302        //-3.0
#define bias_WY   0  			//1.758         //2.0
#define bias_WZ   0 			//1.59       //1.5
#define bias_AX   0    //500.0
#define bias_AY   0    //100.0
#define bias_AZ   0   //50

void imu6050_init(I2C_HandleTypeDef* hi2c1);
void imu6050_init_x69(I2C_HandleTypeDef* hi2c1);
void imu6050_write(I2C_HandleTypeDef* hi2c1,uint16_t reg_addrs, uint16_t len, uint8_t* p);
void imu6050_write_x69(I2C_HandleTypeDef* hi2c1,uint16_t reg_addrs, uint16_t len, uint8_t* p);
void imu6050_read(I2C_HandleTypeDef* hi2c1, uint8_t* x, uint16_t len);
void imu6050_read_x69(I2C_HandleTypeDef* hi2c1, uint8_t* x, uint16_t len);
void imu_process(uint8_t* x, float* y);
uint16_t imu_data_to_str(uint16_t* y, char* go);
float Q_rsqrt_6050( float number );

#define MPU6050_init        imu6050_init
#define MPU6050_init_x69    imu6050_init_x69
#define MPU6050_write       imu6050_write
#define MPU6050_write_x69   imu6050_write_x69
#define MPU6050_read        imu6050_read
#define MPU6050_read_x69    imu6050_read_x69
#define MPU6050_process     imu_process
#endif //QUA_CONTROL_MPU6050_H
