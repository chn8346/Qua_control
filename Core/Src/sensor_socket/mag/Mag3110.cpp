//
// Created by fanchunhui on 2022/11/9.
//

#include "Mag3110.h"


void MAG3110_init(I2C_HandleTypeDef* hi2c1)
{
    uint8_t d;
    d=MAG3110_CTRL_DATA;
    HAL_I2C_Mem_Write(hi2c1 ,MAG3110_ADDR ,MAG3110_CTRL_REG, 1, &d , 1, 1000);

    d = 0;
    HAL_I2C_Mem_Write(hi2c1 ,MAG3110_ADDR ,0x0a, 1, &d , 1, 1000);
    HAL_I2C_Mem_Write(hi2c1 ,MAG3110_ADDR ,0x0b, 1, &d , 1, 1000);
    HAL_I2C_Mem_Write(hi2c1 ,MAG3110_ADDR ,0x0c, 1, &d , 1, 1000);
    HAL_I2C_Mem_Write(hi2c1 ,MAG3110_ADDR ,0x0d, 1, &d , 1, 1000);
    HAL_I2C_Mem_Write(hi2c1 ,MAG3110_ADDR ,0x0e, 1, &d , 1, 1000);
    HAL_I2C_Mem_Write(hi2c1 ,MAG3110_ADDR ,0x0f, 1, &d , 1, 1000);

}

void MAG3110_read(I2C_HandleTypeDef* hi2c1, uint8_t* mdata)
{
    HAL_I2C_Mem_Read(hi2c1, MAG3110_ADDR, MAG3110_XL, 1, mdata+0, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, MAG3110_ADDR, MAG3110_XH, 1, mdata+1, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, MAG3110_ADDR, MAG3110_YL, 1, mdata+2, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, MAG3110_ADDR, MAG3110_YH, 1, mdata+3, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, MAG3110_ADDR, MAG3110_ZL, 1, mdata+4, 1, 1000);
    HAL_I2C_Mem_Read(hi2c1, MAG3110_ADDR, MAG3110_ZH, 1, mdata+5, 1, 1000);
}

void MAG3110_reset(I2C_HandleTypeDef* hi2c1)
{
    MAG3110_init(hi2c1);
}

void MAG3110_process(uint8_t* mdata, float* pdata, float* boundary, float* norm)
{
    int16_t smdata[6] = {0,0,0,0,0,0};	// 有符号磁力计数据

    // 转有符号整数
    smdata[0] = (int16_t)(mdata[0] + (mdata[1] << 8));
    smdata[1] = (int16_t)(mdata[2] + (mdata[3] << 8));
    smdata[2] = (int16_t)(mdata[4] + (mdata[5] << 8));

    // 转浮点数
    pdata[0] = (float)smdata[0];
    pdata[1] = (float)smdata[1];
    pdata[2] = (float)smdata[2];

    // 使用边界值进行校正
    if(boundary[0] < boundary[1]) norm[0] = (pdata[0] - (boundary[0] + boundary[1])/2)/(boundary[1] - boundary[0]); else norm[0] = pdata[0];
    if(boundary[2] < boundary[3]) norm[1] = (pdata[1] - (boundary[2] + boundary[3])/2)/(boundary[3] - boundary[2]); else norm[1] = pdata[1];
    if(boundary[4] < boundary[5]) norm[2] = (pdata[2] - (boundary[4] + boundary[5])/2)/(boundary[5] - boundary[4]); else norm[2] = pdata[2];
}


float mag_boundary[6] = {70000,-70000,70000,-70000,70000,-70000};

void MAG3110_process2(uint8_t* mdata, float* norm)
{

    int16_t signed_data[3];
    // 转有符号整数
    signed_data[0] = (int16_t)(mdata[0] + (mdata[1] << 8));
    signed_data[1] = (int16_t)(mdata[2] + (mdata[3] << 8));
    signed_data[2] = (int16_t)(mdata[4] + (mdata[5] << 8));

    float float_data[3];
    float_data[0] = (float)signed_data[0];
    float_data[1] = (float)signed_data[1];
    float_data[2] = (float)signed_data[2];

    mag_boundary[0] = (mag_boundary[0] > float_data[0]) ? float_data[0] : mag_boundary[0];
    mag_boundary[1] = (mag_boundary[1] < float_data[0]) ? float_data[0] : mag_boundary[1];

    mag_boundary[2] = (mag_boundary[2] > float_data[1]) ? float_data[1] : mag_boundary[2];
    mag_boundary[3] = (mag_boundary[3] < float_data[1]) ? float_data[1] : mag_boundary[3];

    mag_boundary[4] = (mag_boundary[4] > float_data[2]) ? float_data[2] : mag_boundary[4];
    mag_boundary[5] = (mag_boundary[5] < float_data[2]) ? float_data[2] : mag_boundary[5];

    if(mag_boundary[1] != mag_boundary[0])
        norm[0] = (float)1.0 - (mag_boundary[1] - float_data[0])/(mag_boundary[1] - mag_boundary[0]);
    if(mag_boundary[3] != mag_boundary[2])
        norm[1] = (float)1.0 - (mag_boundary[3] - float_data[1])/(mag_boundary[3] - mag_boundary[2]);
    if(mag_boundary[5] != mag_boundary[4])
        norm[2] = (float)1.0 - (mag_boundary[5] - float_data[2])/(mag_boundary[5] - mag_boundary[4]);
}

void MAG3110_process3(uint8_t* mdata, float* float_data)
{

    int16_t signed_data[3];
    // 转有符号整数
    signed_data[0] = (int16_t)(mdata[0] + (mdata[1] << 8));
    signed_data[1] = (int16_t)(mdata[2] + (mdata[3] << 8));
    signed_data[2] = (int16_t)(mdata[4] + (mdata[5] << 8));

    float_data[0] = (float)signed_data[0];
    float_data[1] = (float)signed_data[1];
    float_data[2] = (float)signed_data[2];
}
