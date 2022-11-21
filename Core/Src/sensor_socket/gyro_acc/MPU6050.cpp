//
// Created by fanchunhui on 2022/11/9.
//

#include "MPU6050.h"

void imu6050_init(I2C_HandleTypeDef* hi2c1)
{
    uint8_t* d;
    *d=0x00; HAL_I2C_Mem_Write(hi2c1, imu_addr, MPU6050_RA_SMPLRT_DIV, 1, d , 1, 1000);
    *d=0x00; HAL_I2C_Mem_Write(hi2c1, imu_addr, MPU6050_RA_CONFIG, 1, d , 1, 1000);
    *d=0xe0; HAL_I2C_Mem_Write(hi2c1, imu_addr, MPU6050_RA_ACCEL_CONFIG, 1, d , 1, 1000);
    *d=0xf0; HAL_I2C_Mem_Write(hi2c1, imu_addr, MPU6050_RA_GYRO_CONFIG, 1, d , 1, 1000);
    *d=0x00; HAL_I2C_Mem_Write(hi2c1, imu_addr, MPU6050_RA_PWR_MGMT_1, 1, d , 1, 1000);
}

void imu6050_init_x69(I2C_HandleTypeDef* hi2c1)
{
    uint8_t* d;
    *d=0x00; HAL_I2C_Mem_Write(hi2c1, imu2_addr, MPU6050_RA_SMPLRT_DIV, 1, d , 1, 1000);
    *d=0x00; HAL_I2C_Mem_Write(hi2c1, imu2_addr, MPU6050_RA_CONFIG, 1, d , 1, 1000);
    *d=0xe0; HAL_I2C_Mem_Write(hi2c1, imu2_addr, MPU6050_RA_ACCEL_CONFIG, 1, d , 1, 1000);
    *d=0xf0; HAL_I2C_Mem_Write(hi2c1, imu2_addr, MPU6050_RA_GYRO_CONFIG, 1, d , 1, 1000);
    *d=0x00; HAL_I2C_Mem_Write(hi2c1, imu2_addr, MPU6050_RA_PWR_MGMT_1, 1, d , 1, 1000);
}

void imu6050_write(I2C_HandleTypeDef* hi2c1,uint16_t reg_addrs, uint16_t len, uint8_t* p)
{
    HAL_I2C_Mem_Write(hi2c1, imu_addr, reg_addrs, len, p , len, 1000);
}

void imu6050_write_x69(I2C_HandleTypeDef* hi2c1,uint16_t reg_addrs, uint16_t len, uint8_t* p)
{
    HAL_I2C_Mem_Write(hi2c1, imu2_addr, reg_addrs, len, p , len, 1000);
}

void imu6050_read(I2C_HandleTypeDef* hi2c1, uint8_t* x, uint16_t len)
{
    HAL_I2C_Mem_Read(hi2c1, imu_addr, MPU6050_RA_ACCEL_XOUT_L , 1, x, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu_addr, MPU6050_RA_ACCEL_XOUT_H , 1, x+1, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu_addr, MPU6050_RA_ACCEL_YOUT_L , 1, x+2, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu_addr, MPU6050_RA_ACCEL_YOUT_H , 1, x+3, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu_addr, MPU6050_RA_ACCEL_ZOUT_L , 1, x+4, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu_addr, MPU6050_RA_ACCEL_ZOUT_H , 1, x+5, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu_addr, MPU6050_RA_GYRO_XOUT_L , 1, x+6, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu_addr, MPU6050_RA_GYRO_XOUT_H , 1, x+7, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu_addr, MPU6050_RA_GYRO_YOUT_L , 1, x+8, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu_addr, MPU6050_RA_GYRO_YOUT_H , 1, x+9, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu_addr, MPU6050_RA_GYRO_ZOUT_L , 1, x+10, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu_addr, MPU6050_RA_GYRO_ZOUT_H , 1, x+11, 1, 1000);
}

void imu6050_read_x69(I2C_HandleTypeDef* hi2c1, uint8_t* x, uint16_t len)
{
    HAL_I2C_Mem_Read(hi2c1, imu2_addr, MPU6050_RA_ACCEL_XOUT_L , 1, x, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu2_addr, MPU6050_RA_ACCEL_XOUT_H , 1, x+1, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu2_addr, MPU6050_RA_ACCEL_YOUT_L , 1, x+2, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu2_addr, MPU6050_RA_ACCEL_YOUT_H , 1, x+3, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu2_addr, MPU6050_RA_ACCEL_ZOUT_L , 1, x+4, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu2_addr, MPU6050_RA_ACCEL_ZOUT_H , 1, x+5, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu2_addr, MPU6050_RA_GYRO_XOUT_L , 1, x+6, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu2_addr, MPU6050_RA_GYRO_XOUT_H , 1, x+7, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu2_addr, MPU6050_RA_GYRO_YOUT_L , 1, x+8, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu2_addr, MPU6050_RA_GYRO_YOUT_H , 1, x+9, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu2_addr, MPU6050_RA_GYRO_ZOUT_L , 1, x+10, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, imu2_addr, MPU6050_RA_GYRO_ZOUT_H , 1, x+11, 1, 1000);
}

void imu_process(uint8_t* x, float* y)
{
    uint16_t x16[6];
    x16[0] = x[1];
    x16[0] = (x16[0] << 8) + x[0];
    x16[1] = x[3];
    x16[1] = (x16[1] << 8) + x[2];
    x16[2] = x[5];
    x16[2] = (x16[2] << 8) + x[4];
    x16[3] = x[7];
    x16[3] = (x16[3] << 8) + x[6];
    x16[4] = x[9];
    x16[4] = (x16[4] << 8) + x[8];
    x16[5] = x[11];
    x16[5] = (x16[5] << 8) + x[10];

    int16_t x16s[6];
    x16s[0] = (int16_t)x16[0];
    x16s[1] = (int16_t)x16[1];
    x16s[2] = (int16_t)x16[2];
    x16s[3] = (int16_t)x16[3];
    x16s[4] = (int16_t)x16[4];
    x16s[5] = (int16_t)x16[5];

    y[0] = (float)x16s[0];
    y[1] = (float)x16s[1];
    y[2] = (float)x16s[2];
    y[3] = (float)x16s[3];
    y[4] = (float)x16s[4];
    y[5] = (float)x16s[5];
}


uint16_t imu_data_to_str(uint16_t* y, char* go)
{
    int i = 0;
    uint16_t index = 0,j=0,m=10000;
    for(i = 0; i < 6; i++)
    {
        for(j = 0; j < 6; j++)
        {
            go[index] = y[i]/m;
        }
    }
    return index;
}

// 快速开方倒数
float Q_rsqrt_6050( float number )
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = * ( long * ) &y;						// evil floating point bit level hacking
    i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
    y  = * ( float * ) &i;
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

    return y;
}

