//
// Created by fanchunhui on 2022/11/13.
//

#include "HMC5883.h"

I2C_HandleTypeDef * i2c_hmc;

void HMC5883_init()
{
    uint8_t value = HMC5883_CONF_A_V;
    HAL_I2C_Mem_Write(i2c_hmc, HMC5883_WRITE_ADDR, HMC5883_CONFIG_A,1, &value, 1, 500);

    value = HMC5883_CONF_B_V;
    HAL_I2C_Mem_Write(i2c_hmc, HMC5883_WRITE_ADDR, HMC5883_CONFIG_B,1, &value, 1, 500);

    value = HMC5883_MODE_VALUE;
    HAL_I2C_Mem_Write(i2c_hmc, HMC5883_WRITE_ADDR, HMC5883_MODE_REG,1, &value, 1, 500);
}

void HMC5883_reset()
{
    uint8_t value = HMC5883_CONF_A_V;
    HAL_I2C_Mem_Write(i2c_hmc, HMC5883_WRITE_ADDR, HMC5883_CONFIG_A,1, &value, 1, 500);

    value = HMC5883_CONF_B_V;
    HAL_I2C_Mem_Write(i2c_hmc, HMC5883_WRITE_ADDR, HMC5883_CONFIG_B,1, &value, 1, 500);

    value = HMC5883_MODE_VALUE;
    HAL_I2C_Mem_Write(i2c_hmc, HMC5883_WRITE_ADDR, HMC5883_MODE_REG,1, &value, 1, 500);

}

void HMC5883_read(uint8_t* m)
{
    HAL_I2C_Mem_Read(i2c_hmc, HMC5883_READ_ADDR, HMC5883_XH, 1, m, 1, 500);
    HAL_I2C_Mem_Read(i2c_hmc, HMC5883_READ_ADDR, HMC5883_XL, 1, m+1, 1, 500);
    HAL_I2C_Mem_Read(i2c_hmc, HMC5883_READ_ADDR, HMC5883_YH, 1, m+2, 1, 500);
    HAL_I2C_Mem_Read(i2c_hmc, HMC5883_READ_ADDR, HMC5883_YL, 1, m+3, 1, 500);
    HAL_I2C_Mem_Read(i2c_hmc, HMC5883_READ_ADDR, HMC5883_ZH, 1, m+4, 1, 500);
    HAL_I2C_Mem_Read(i2c_hmc, HMC5883_READ_ADDR, HMC5883_ZL, 1, m+5, 1, 500);
}

void HMC5883_read_and_process(float* mag)
{
    uint8_t m[6];
    HMC5883_read(m);

    uint16_t m16[3];
    m16[0] = m[0];
    m16[0] = (m16[0]<<8) + m[1];
    m16[1] = m[2];
    m16[1] = (m16[1]<<8) + m[3];
    m16[2] = m[4];
    m16[2] = (m16[2]<<8) + m[5];

    mag[0] = (float)m16[0];
    mag[1] = (float)m16[1];
    mag[2] = (float)m16[2];
}

