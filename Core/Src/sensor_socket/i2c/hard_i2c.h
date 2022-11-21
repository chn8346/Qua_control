//
// Created by fanchunhui on 2022/11/9.
//

#ifndef QUA_CONTROL_HARD_I2C_H
#define QUA_CONTROL_HARD_I2C_H

#include "../../base_head.h"

void i2c_send(I2C_HandleTypeDef *i2c, uint16_t sensor_i2c_addr, uint16_t reg_addr, uint8_t* data, uint8_t data_size);

void i2c_receive(I2C_HandleTypeDef *i2c, uint16_t sensor_i2c_addr, uint16_t reg_addr, uint8_t* data, uint8_t data_size);

void i2c_read(I2C_HandleTypeDef *i2c, uint16_t sensor_i2c_addr, uint16_t reg_addr, uint8_t* data, uint8_t data_size);

uint8_t i2c_scan(I2C_HandleTypeDef *i2c, uint16_t who_am_i_addr, uint8_t who_am_i_KEY);

void i2c_scan_print(I2C_HandleTypeDef *i2c, uint16_t who_am_i_addr, uint8_t who_am_i_KEY);

#endif //QUA_CONTROL_HARD_I2C_H
