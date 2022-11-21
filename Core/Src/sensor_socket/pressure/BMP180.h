//
// Created by fanchunhui on 2022/11/20.
//

#ifndef QUA_CONTROL_BMP180_H
#define QUA_CONTROL_BMP180_H

#include "../../base_head.h"

#define BMP180_ADDR_READ    0xEF
#define BMP180_ADDR_WRITE   0xEE
#define BMP180_ID_ADDR      0xD0
#define BMP180_ID_KEY       0x55

// 重置寄存器，写入B6传感器重置
#define BMP180_SOFT_RESET_REG  0xE0
#define BMP180_SOFT_RESET_KEY  0xB6

#define BMP180_CONTROL_REG     0xF4
#define BMP180_CONTROL_KEY     0x74  // 0x74 -->7.5ms采样时间，  0x34为4.5ms   0xB4为13.5ms  0xF4为25.5ms 温度测量为0x2E

#define BMP180_MSB             0xF6
#define BMP180_LSB             0xF7
#define BMP180_XLSB            0xF8  // 高4位可用，低3位为0

void BMP180_init(I2C_HandleTypeDef* i2c);
void BMP180_reset();
void get_temperature(float* tmp);
void get_pressure(float* psr);
void get_height(float* h);

#endif //QUA_CONTROL_BMP180_H
