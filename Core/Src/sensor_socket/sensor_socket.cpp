//
// Created by fanchunhui on 2022/11/9.
//

#include "sensor_socket.h"

/*
 *          IMU 选择
 * */

// MPU6050
#if SENSOR_SELECT_IMU == SENSOR_SELECT_IMU_MU6050
I2C_HandleTypeDef* i2c_;

void IMU_init(I2C_HandleTypeDef* i2c){
    i2c_ = i2c;
    MPU6050_init(i2c);


}

void IMU_read_data(uint8_t* data_int, float* data_f){
    MPU6050_read(i2c_, data_int, 12);
    imu_process(data_int, data_f);
}

// ICM20602
#elif SENSOR_SELECT_IMU == SENSOR_SELECT_IMU_ICM20602
void IMU_init(I2C_HandleTypeDef* i2c){
        ICM_init(i2c);
}

void IMU_read_data(uint8_t* data_int, float* data_f){
        ICM_get_6_axis_data(data_int, data_f);
}

// NO IMU
#elif SENSOR_SELECT_IMU == SENSOR_SELECT_NONE

void IMU_init(I2C_HandleTypeDef* i2c){
    ; // do nothing
}

void IMU_read_data(uint8_t* data_int, float* data_f){
    ; // do nothing
}

#endif



/*
 *          磁力计 选择
 * */

// HMC5883
#if SENSOR_SELECT_MAG == SENSOR_SELECT_MAG_HMC5883
void MAG_init(I2C_HandleTypeDef* i2c){
    HMC5883_init(i2c);
}

void MAG_read_data(uint8_t* data_int, float* data_f){
    HMC5883_read_and_process(data_int, data_f);
}

// IST8310
#elif SENSOR_SELECT_MAG == SENSOR_SELECT_MAG_IST8310
void MAG_init(I2C_HandleTypeDef* i2c){
    IST8310_init(i2c);
}

void MAG_read_data(uint8_t* data_int, float* data_f){
    IST8310_get_3d_data(data_int, data_f);
}

// MAG3110
#elif SENSOR_SELECT_MAG == SENSOR_SELECT_MAG_MAG3110
I2C_HandleTypeDef* i2c_m;

void MAG_init(I2C_HandleTypeDef* i2c){
    MAG3110_init(i2c);
    i2c_m = i2c;

}

void MAG_read_data(uint8_t* data_int, float* data_f){
    MAG3110_read(i2c_m, data_int);
    MAG3110_process3(data_int, data_f);
}

// NO MAG
#elif SENSOR_SELECT_MAG == SENSOR_SELECT_NONE
void MAG_init(I2C_HandleTypeDef* i2c){
    ; // do nothing
}

void MAG_read_data(uint8_t* data_int, float* data_f){
    ; // do nothing
}

#endif



/*
 *          GPS 选择
 * */

// GPS -- BN880
#if SENSOR_SELECT_GPS == SENSOR_SELECT_GPS_BN880
void GPS_init(I2C_HandleTypeDef* i2c){

}

void GPS_read_data(uint8_t* data_int, float* data_f){

}

// NO GPS
#elif SENSOR_SELECT_GPS == SENSOR_SELECT_NONE
void GPS_init(I2C_HandleTypeDef* i2c){
    ; // do nothing
}

void GPS_read_data(uint8_t* data_int, float* data_f){
    ; // do nothing
}
#endif



/*
 *          气压计 选择
 * */

// BMP180
#if SENSOR_SELECT_PSR == SENSOR_SELECT_PSR_BMP180
void PSR_init(I2C_HandleTypeDef* i2c){
    BMP180_init(i2c);
}

void PSR_read_data(uint8_t* data_int, float* data_f){
    BMP180_get_pressure(data_f);
}

// SPL06
#elif SENSOR_SELECT_PSR == SENSOR_SELECT_PSR_SPL06
void PSR_init(I2C_HandleTypeDef* i2c){
    SPL06_init(i2c);
}

void PSR_read_data(uint8_t* data_int, float* data_f){
    ; // 暂缺
}

// NO PSR
#elif SENSOR_SELECT_PSR == SENSOR_SELECT_NONE
void PSR_init(I2C_HandleTypeDef* i2c){
    ; // do nothing
}

void PSR_read_data(uint8_t* data_int, float* data_f){
    ; // do nothing
}

#endif



/*
 *          超声波 选择
 * */

// SONIC - 1
#if SENSOR_SELECT_SONIC == SENSOR_SELECT_SONIC_US1
void SNC_init(I2C_HandleTypeDef* i2c){
    ; // 无需初始化
}

void SNC_read_data(uint8_t* data_int, float* data_f){
    ; // 中断完成了任务
}

// NO SONIC
#elif SENSOR_SELECT_SONIC = SENSOR_SELECT_NONE
void SNC_init(I2C_HandleTypeDef* i2c){

}

void SNC_read_data(uint8_t* data_int, float* data_f){

}

#endif
