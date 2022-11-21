//
// Created by fanchunhui on 2022/11/20.
//

#include "BMP180.h"

I2C_HandleTypeDef * i2c_bmp;

void BMP180_init(I2C_HandleTypeDef* i2c){
    i2c_bmp = i2c;
    uint8_t value = 0;
    HAL_I2C_Mem_Read(i2c_bmp, BMP180_ADDR_READ, BMP180_ID_ADDR, 1, &value, 1, 500);
    if(value == BMP180_ID_KEY) {
        value = BMP180_CONTROL_KEY;
        HAL_I2C_Mem_Write(i2c_bmp, BMP180_ADDR_WRITE, BMP180_CONTROL_REG, 1, &value, 1, 500);
    }
}

void BMP180_reset(){
    uint8_t value = BMP180_SOFT_RESET_KEY;
    HAL_I2C_Mem_Write(i2c_bmp, BMP180_ADDR_WRITE, BMP180_SOFT_RESET_REG, 1, &value, 1, 500);

    for(int i = 0; i < 100000; i++); // 延时等待重置，重新写入控制
    value = BMP180_CONTROL_KEY;
    HAL_I2C_Mem_Write(i2c_bmp, BMP180_ADDR_WRITE, BMP180_CONTROL_REG, 1, &value, 1, 500);
}

void get_temperature(float* tmp){
    uint8_t value;
    int iv = 0;
    HAL_I2C_Mem_Read(i2c_bmp, BMP180_ADDR_READ, BMP180_XLSB, 1, &value, 1, 500);
    iv = value>>3;  // 高四位可用
    HAL_I2C_Mem_Read(i2c_bmp, BMP180_ADDR_READ, BMP180_LSB, 1, &value, 1, 500);
    iv = (iv << 8) + value;
    HAL_I2C_Mem_Read(i2c_bmp, BMP180_ADDR_READ, BMP180_MSB, 1, &value, 1, 500);
    iv = (iv << 8) + value;
    *tmp = (float)iv;
}

void get_pressure(float* psr){
    uint8_t value;
    int iv = 0;
    HAL_I2C_Mem_Read(i2c_bmp, BMP180_ADDR_READ, BMP180_XLSB, 1, &value, 1, 500);
    iv = value>>3;  // 高四位可用
    HAL_I2C_Mem_Read(i2c_bmp, BMP180_ADDR_READ, BMP180_LSB, 1, &value, 1, 500);
    iv = (iv << 8) + value;
    HAL_I2C_Mem_Read(i2c_bmp, BMP180_ADDR_READ, BMP180_MSB, 1, &value, 1, 500);
    iv = (iv << 8) + value;
    *psr = (float)iv;

}

// TODO 【未完成】 收尾工作还没有完成，需要等传感器到了之后测一下
void get_height(float* h){
    *h = 0;
}

