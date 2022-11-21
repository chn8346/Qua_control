//
// Created by fanchunhui on 2022/11/9.
//

#include "pre_process.h"

#include "../sensor_socket/gyro_acc/MPU6050.h"

float init_process_count = 3000;

void sensor_mean_data_process(I2C_HandleTypeDef* i2c, float acc_mean[9])
{
    printf("begin to fix mean from acc & gero...");

    // 获取数值
    uint8_t imu2_read[12];
    float imu2_data[3];
    float all_float = init_process_count;
    uint16_t ind = init_process_count;

    while(ind--)
    {
        imu6050_read(i2c, imu2_read, 12);

        // 加速度计
        imu2_data[0] = (float)(int16_t)(imu2_read[0] + (imu2_read[1] << 8));
        imu2_data[1] = (float)(int16_t)(imu2_read[2] + (imu2_read[3] << 8));
        imu2_data[2] = (float)(int16_t)(imu2_read[4] + (imu2_read[5] << 8));

        acc_mean[0] = acc_mean[0] + imu2_data[0]/all_float;
        acc_mean[1] = acc_mean[1] + imu2_data[1]/all_float;
        acc_mean[2] = acc_mean[2] + imu2_data[2]/all_float;

        // 陀螺仪
        imu2_data[0] = (float)(int16_t)(imu2_read[6] + (imu2_read[7] << 8));
        imu2_data[1] = (float)(int16_t)(imu2_read[8] + (imu2_read[9] << 8));
        imu2_data[2] = (float)(int16_t)(imu2_read[10] + (imu2_read[11] << 8));
        imu2_data[0] = imu2_data[0]/500.0;
        imu2_data[1] = imu2_data[1]/500.0;
        imu2_data[2] = imu2_data[2]/500.0;

        acc_mean[3] = acc_mean[3] + imu2_data[0]/all_float;
        acc_mean[4] = acc_mean[4] + imu2_data[1]/all_float;
        acc_mean[5] = acc_mean[5] + imu2_data[2]/all_float;
    }

    printf("mean: ax:%5.2f ay:%5.2f az:%5.2f gx:%5.2f gy:%5.2f gz:%5.2f\n",
           acc_mean[0],
           acc_mean[1],
           acc_mean[2],
           acc_mean[3],
           acc_mean[4],
           acc_mean[5]);
}

void sensor_cov_data_process(I2C_HandleTypeDef* i2c, float acc_cov[9], float acc_mean[9])
{
    printf("begin to fix P from acc&gero...");

    // 获取数值
    uint8_t imu2_read[12];
    float imu2_data[3];	// 加速度计数据
    float gyro_data[3]; // 陀螺仪数据
    float mag_data[3];  // 磁力计数据
    float all_float = init_process_count;
    uint16_t ind = init_process_count;

    while(ind--)
    {
        imu6050_read(i2c, imu2_read, 12);

        // 加速度计
        imu2_data[0] = (float)(int16_t)(imu2_read[0] + (imu2_read[1] << 8));
        imu2_data[1] = (float)(int16_t)(imu2_read[2] + (imu2_read[3] << 8));
        imu2_data[2] = (float)(int16_t)(imu2_read[4] + (imu2_read[5] << 8));

        acc_cov[0] = acc_cov[0] + (imu2_data[0] - acc_mean[0])*(imu2_data[0] - acc_mean[0])/all_float;
        acc_cov[1] = acc_cov[1] + (imu2_data[1] - acc_mean[1])*(imu2_data[1] - acc_mean[1])/all_float;
        acc_cov[2] = acc_cov[2] + (imu2_data[2] - acc_mean[2])*(imu2_data[2] - acc_mean[2])/all_float;

        // 陀螺仪
        gyro_data[0] = (float)(int16_t)(imu2_read[6] + (imu2_read[7] << 8));
        gyro_data[1] = (float)(int16_t)(imu2_read[8] + (imu2_read[9] << 8));
        gyro_data[2] = (float)(int16_t)(imu2_read[10] + (imu2_read[11] << 8));
        gyro_data[0] = gyro_data[0]/500.0;
        gyro_data[1] = gyro_data[1]/500.0;
        gyro_data[2] = gyro_data[2]/500.0;

        acc_cov[3] = acc_cov[3] + ((gyro_data[0] - acc_mean[3])*(gyro_data[0] - acc_mean[3]));
        acc_cov[4] = acc_cov[4] + ((gyro_data[1] - acc_mean[4])*(gyro_data[1] - acc_mean[4]));
        acc_cov[5] = acc_cov[5] + ((gyro_data[2] - acc_mean[5])*(gyro_data[2] - acc_mean[5]));

        // 磁罗盘


    }

    acc_cov[3] = acc_cov[3] / all_float;
    acc_cov[4] = acc_cov[4] / all_float;
    acc_cov[5] = acc_cov[5] / all_float;

    printf("mean: ax:%5.2f ay:%5.2f az:%5.2f gx:%5.6f gy:%5.6f gz:%5.6f\n",
           acc_cov[0],
           acc_cov[1],
           acc_cov[2],
           acc_cov[3],
           acc_cov[4],
           acc_cov[5]);
}