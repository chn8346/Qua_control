//
// Created by fanchunhui on 2022/11/9.
//

#include "IST8310.h"

I2C_HandleTypeDef * i2c_IST;

void IST8310_init(I2C_HandleTypeDef* i2c)
{
    i2c_IST = i2c;

    uint8_t value;
    HAL_I2C_Mem_Read(i2c_IST, IST8310_ADDR, IST8310_ID_ADDR, 1, &value, 1, 500);

    if(value != IST8310_ID_KEY)return;

    value = IST8310_CTRL_VALUE;
    HAL_I2C_Mem_Write(i2c_IST, IST8310_ADDR, IST8310_CTRL_REG, 1, &value, 1, 500);
}

void IST8310_reset()
{
    uint8_t value = IST8310_CTRL_VALUE;
    HAL_I2C_Mem_Write(i2c_IST, IST8310_ADDR, IST8310_CTRL_REG, 1, &value, 1, 500);
}

void IST8310_get_raw_data(uint8_t* data)
{
    HAL_I2C_Mem_Read(i2c_IST, IST8310_ADDR, IST8310_MAG_XL, 1, data, 1, 500);
    HAL_I2C_Mem_Read(i2c_IST, IST8310_ADDR, IST8310_MAG_XH, 1, data+1, 1, 500);
    HAL_I2C_Mem_Read(i2c_IST, IST8310_ADDR, IST8310_MAG_YL, 1, data+2, 1, 500);
    HAL_I2C_Mem_Read(i2c_IST, IST8310_ADDR, IST8310_MAG_YH, 1, data+3, 1, 500);
    HAL_I2C_Mem_Read(i2c_IST, IST8310_ADDR, IST8310_MAG_ZL, 1, data+4, 1, 500);
    HAL_I2C_Mem_Read(i2c_IST, IST8310_ADDR, IST8310_MAG_ZH, 1, data+5, 1, 500);
}

void IST8310_get_3d_data(float* data)
{
    uint8_t data8[6];
    IST8310_get_raw_data(data8);

    data[0] = (float)(data8[0] + (data8[1]<<8));
    data[1] = (float)(data8[2] + (data8[3]<<8));
    data[2] = (float)(data8[4] + (data8[5]<<8));
}


