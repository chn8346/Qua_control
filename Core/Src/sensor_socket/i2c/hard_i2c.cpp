//
// Created by fanchunhui on 2022/11/9.
//

#include "hard_i2c.h"
#include "../../TR/usart_plus.h"

void i2c_send(I2C_HandleTypeDef *i2c, uint16_t sensor_i2c_addr, uint16_t reg_addr, uint8_t* data, uint8_t data_size)
{
    HAL_I2C_Mem_Write(i2c,
                      sensor_i2c_addr,
                      reg_addr,
                      data_size,
                      data,
                      data_size,
                      1000);
}

// small toolkit 1
void u8_to_DEC_str_i2c(uint8_t x, uint8_t* ret)
{
    ret[0] = '0' + (uint8_t)(x/100);    x = x % 100;
    ret[1] = '0' + (uint8_t)(x/10);     x = x % 10;
    ret[2] = '0' + x;
}

// small toolkit 2
void u8_to_HEX_str_i2c(uint8_t x, uint8_t* ret)
{
    ret[0] = '0';     ret[1] = 'x';
    ret[2] = x>>4;    ret[2] = ret[2]>9? ('A' + (ret[2]-10)) : ('0' + ret[2]);
    ret[3] = x&0x0F;  ret[3] = ret[3]>9? ('A' + (ret[3]-10)) : ('0' + ret[3]);
}

void i2c_receive(I2C_HandleTypeDef *i2c, uint16_t sensor_i2c_addr, uint16_t reg_addr, uint8_t* data, uint8_t data_size)
{
    HAL_I2C_Mem_Read(i2c,
                     sensor_i2c_addr,
                     reg_addr,
                     data_size,
                     data,
                     data_size,
                     1000);
}

void i2c_read(I2C_HandleTypeDef *i2c, uint16_t sensor_i2c_addr, uint16_t reg_addr, uint8_t* data, uint8_t data_size)
{
    i2c_receive(i2c, sensor_i2c_addr, reg_addr, data, data_size);
}

uint8_t i2c_scan(I2C_HandleTypeDef *i2c, uint16_t who_am_i_addr, uint8_t who_am_i_KEY)
{
    uint8_t key = 0;
    for(int i = 0; i < 256; i++)
    {
        HAL_I2C_Mem_Read(i2c, i, who_am_i_addr, 1, &key, 1, 100);
        if(key == who_am_i_KEY)
        {
            return i;
        }
    }

    return 0;
}

void i2c_scan_print(I2C_HandleTypeDef *i2c, uint16_t who_am_i_addr, uint8_t who_am_i_KEY)
{
    uint8_t key = 0;
    for(uint8_t i = 0; i < 255; i++)
    {
        HAL_I2C_Mem_Read(i2c, i, who_am_i_addr, 1, &key, 1, 100);
        if(key == who_am_i_KEY)
        {
            uint8_t data[15];
            u8_to_HEX_str_i2c(i, data);
            data[4] = ' ';
            u8_to_HEX_str_i2c(key, data+5);
            data[9] = ' ';
            u8_to_HEX_str_i2c(0xff, data+10);
            data[14] = 0xff;
            uart_wireless_send(data, 15);
        }
        else
        {
            uint8_t data[15];
            u8_to_HEX_str_i2c(i, data);
            data[4] = ' ';
            u8_to_HEX_str_i2c(key, data+5);
            data[9] = ' ';
            u8_to_HEX_str_i2c(0x00, data+10);
            data[14] = 0xff;
            uart_wireless_send(data, 15);
        }
        key = 0;
        for(int i1 = 0; i1 < 250000; i1++); // 防止串口拥堵
    }
    uint8_t notice[] = "IF NOT TRUE, MAY BE: 0xFF";
    uart_wireless_send(notice, 25);
}
