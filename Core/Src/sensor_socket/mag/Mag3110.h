//
// Created by fanchunhui on 2022/11/9.
//

#ifndef QUA_CONTROL_MAG3110_H
#define QUA_CONTROL_MAG3110_H

#include "../../base_head.h"

#define MAG3110_ADDR 				(0x0e)<<1
#define MAG3110_CTRL_REG		    0x10
#define MAG3110_CTRL_DATA		    0x01
#define MAG3110_XL   				0x02
#define MAG3110_XH					0x01
#define MAG3110_YL					0x04
#define MAG3110_YH					0x03
#define MAG3110_ZL					0x06
#define MAG3110_ZH					0x05

void MAG3110_init(I2C_HandleTypeDef* i2c);
void MAG3110_read(I2C_HandleTypeDef* hi2c1, uint8_t* mdata);
void MAG3110_reset(I2C_HandleTypeDef* hi2c1);
void MAG3110_process(uint8_t* mdata, float* pdata, float* boundary, float* norm);
void MAG3110_process2(uint8_t* mdata, float* norm);
void MAG3110_process3(uint8_t* mdata, float* float_data);

#endif //QUA_CONTROL_MAG3110_H
