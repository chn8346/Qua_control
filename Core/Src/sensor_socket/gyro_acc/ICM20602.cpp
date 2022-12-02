//
// Created by fanchunhui on 2022/11/9.
//

#include "ICM20602.h"

I2C_HandleTypeDef * i2c_icm;

void ICM_init(I2C_HandleTypeDef * i2c)
{
    i2c_icm = i2c;

    uint8_t value;
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_ID_ADDR, 1, &value, 1, 500);

    if(value != ICM_ID_KEY)return;

    value = ICM_CONFIG_REG_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_CONFIG_REG, 1, &value, 1, 500);

    value = ICM_GYRO_CONF_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_GYRO_CONF, 1, &value, 1, 500);

    value = ICM_ACC_CONF_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_ACC_CONF, 1, &value, 1, 500);

    value = ICM_ACC_CONF2_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_ACC_CONF2, 1, &value, 1, 500);

    value = ICM_CTRL_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_CTRL, 1, &value, 1, 500);

    value = ICM_POWER_MNG_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_POWER_MNG, 1, &value, 1, 500);

    value = ICM_POWER_MNG2_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_POWER_MNG2, 1, &value, 1, 500);

}

void ICM_reset()
{
    uint8_t value = ICM_CONFIG_REG_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_CONFIG_REG, 1, &value, 1, 500);

    value = ICM_GYRO_CONF_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_GYRO_CONF, 1, &value, 1, 500);

    value = ICM_ACC_CONF_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_ACC_CONF, 1, &value, 1, 500);

    value = ICM_ACC_CONF2_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_ACC_CONF2, 1, &value, 1, 500);

    value = ICM_CTRL_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_CTRL, 1, &value, 1, 500);

    value = ICM_POWER_MNG_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_POWER_MNG, 1, &value, 1, 500);

    value = ICM_POWER_MNG2_VALUE;
    HAL_I2C_Mem_Write(i2c_icm, ICM_ADDR, ICM_POWER_MNG2, 1, &value, 1, 500);
}

void ICM_get_full_raw(uint8_t* data)
{
    // GYRO
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_GYRO_XL, 1, data, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_GYRO_XH, 1, data+1, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_GYRO_YL, 1, data+2, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_GYRO_YH, 1, data+3, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_GYRO_ZL, 1, data+4, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_GYRO_ZH, 1, data+5, 1, 500);

    // ACC
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_ACC_XL, 1, data+6, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_ACC_XH, 1, data+7, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_ACC_YL, 1, data+8, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_ACC_YH, 1, data+9, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_ACC_ZL, 1, data+10, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_ACC_ZH, 1, data+11, 1, 500);

    // T
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_TMP_L, 1, data+12, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_TMP_H, 1, data+13, 1, 500);

}

void ICM_get_raw(uint8_t* data)
{
    // GYRO
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_GYRO_XL, 1, data, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_GYRO_XH, 1, data+1, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_GYRO_YL, 1, data+2, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_GYRO_YH, 1, data+3, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_GYRO_ZL, 1, data+4, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_GYRO_ZH, 1, data+5, 1, 500);

    // ACC
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_ACC_XL, 1, data+6, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_ACC_XH, 1, data+7, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_ACC_YL, 1, data+8, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_ACC_YH, 1, data+9, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_ACC_ZL, 1, data+10, 1, 500);
    HAL_I2C_Mem_Read(i2c_icm, ICM_ADDR, ICM_ACC_YH, 1, data+11, 1, 500);

}

void Icm_process(uint8_t data[12], float* data_)
{
    data_[0] = (float)(data[0] + (data[1]<<8));
    data_[1] = (float)(data[2] + (data[3]<<8));
    data_[2] = (float)(data[4] + (data[5]<<8));
    data_[3] = (float)(data[6] + (data[7]<<8));
    data_[4] = (float)(data[8] + (data[9]<<8));
    data_[5] = (float)(data[10] + (data[11]<<8));
}

void ICM_get_6_axis_data(uint8_t *data, float* data_){

    ICM_get_raw(data);

    data_[0] = (float)(data[0] + (data[1]<<8));
    data_[1] = (float)(data[2] + (data[3]<<8));
    data_[2] = (float)(data[4] + (data[5]<<8));
    data_[3] = (float)(data[6] + (data[7]<<8));
    data_[4] = (float)(data[8] + (data[9]<<8));
    data_[5] = (float)(data[10] + (data[11]<<8));
}

void ICM_get_6_axis_data(float* data_)
{
    uint8_t data[12];

    ICM_get_raw(data);

    data_[0] = (float)(data[0] + (data[1]<<8));
    data_[1] = (float)(data[2] + (data[3]<<8));
    data_[2] = (float)(data[4] + (data[5]<<8));
    data_[3] = (float)(data[6] + (data[7]<<8));
    data_[4] = (float)(data[8] + (data[9]<<8));
    data_[5] = (float)(data[10] + (data[11]<<8));
}
