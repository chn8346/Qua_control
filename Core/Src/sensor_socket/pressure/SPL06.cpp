//
// Created by fanchunhui on 2022/11/9.
//

#include "SPL06.h"

I2C_HandleTypeDef* i2c_spl06;

void SPL06_init(I2C_HandleTypeDef* i2c)
{
    i2c_spl06 = i2c;

    uint8_t config = 0;
    HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_ID_ADDR,1, &config, 1, 500);

    if(config != SPL06_ID_KEY)return;

    config = SPL06_PRESSURE_CONFIG_HIGH_PERFORM;
    HAL_I2C_Mem_Write(i2c_spl06, SPL06_ADDR, SPL06_PRS_CFG, 1, &config, 1, 500);

    config = SPL06_TMP_CONFIG_HIGH_PERFORM;
    HAL_I2C_Mem_Write(i2c_spl06, SPL06_ADDR, SPL06_TMP_CFG, 1, &config, 1, 500);

    config = SPL06_CFG_REG_P_SHIFT;
    HAL_I2C_Mem_Write(i2c_spl06, SPL06_ADDR, SPL06_CFG_REG, 1, &config, 1, 500);

    config = SPL06_MODE_TP_BOTH_MEASURE;
    HAL_I2C_Mem_Write(i2c_spl06, SPL06_ADDR, SPL06_MEANS_CFG, 1, &config, 1, 500);
}

uint8_t SPL06_get_id()
{
    uint8_t key;
    key = HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_ID_ADDR,1, &key, 1, 500);
    return key;
}


void SPL06_reset()
{
    uint8_t config = SPL06_CFG_REG_P_SHIFT;
    HAL_I2C_Mem_Write(i2c_spl06, SPL06_ADDR, SPL06_CFG_REG, 1, &config, 1, 500);

    config = SPL06_MODE_TP_BOTH_MEASURE;
    HAL_I2C_Mem_Write(i2c_spl06, SPL06_ADDR, SPL06_MEANS_CFG, 1, &config, 1, 500);

}

void SPL06_get_raw_P(uint8_t* raw_data)
{
    HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_PRESSURE_0_ADDR,1,raw_data ,1,500);
    HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_PRESSURE_1_ADDR,1,raw_data+1 ,1,500);
    HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_PRESSURE_2_ADDR,1,raw_data+2 ,1,500);
}

void SPL06_get_raw_T(uint8_t* raw_data)
{
    HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_TMP_0_ADDR,1,raw_data ,1,500);
    HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_TMP_1_ADDR,1,raw_data+1 ,1,500);
    HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_TMP_2_ADDR,1,raw_data+2 ,1,500);
}

void SPL06_get_raw_data(uint8_t* raw_data)
{
    HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_PRESSURE_0_ADDR,1,raw_data ,1,500);
    HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_PRESSURE_1_ADDR,1,raw_data+1 ,1,500);
    HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_PRESSURE_2_ADDR,1,raw_data+2 ,1,500);
    HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_TMP_0_ADDR,1,raw_data+3 ,1,500);
    HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_TMP_1_ADDR,1,raw_data+4 ,1,500);
    HAL_I2C_Mem_Read(i2c_spl06, SPL06_ADDR, SPL06_TMP_2_ADDR,1,raw_data+5 ,1,500);
}

float SPL06_get_tmp()
{
    uint8_t data[3];
    SPL06_get_raw_T(data);
    float tmp = (float)(data[0]) + (float)(data[1]<<8) + (float)(data[2]<<16);
    return tmp;
}

float SPL06_get_pressure()
{
    uint8_t data[3];
    SPL06_get_raw_P(data);
    float p = (float)(data[0]) + (float)(data[1]<<8) + (float)(data[2]<<16);
    return p;
}

float SPL06_get_height()
{
    return -1;
}
