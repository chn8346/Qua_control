//
// Created by fanchunhui on 2022/11/9.
//

#ifndef QUA_CONTROL_IST8310_H
#define QUA_CONTROL_IST8310_H

#include "../../base_head.h"


#define IST8310_ADDR_1  (0x0C<<1)
#define IST8310_ADDR_2  (0x0D<<1)
#define IST8310_ADDR_3  (0x0E<<1)
#define IST8310_ADDR_4  (0x0F<<1)
#define IST8310_ADDR    IST8310_ADDR_1

#define IST8310_WHO_AM_I_ADDR   0x00
#define IST8310_ID_ADDR         IST8310_WHO_AM_I_ADDR
#define IST8310_WHO_AM_I_KEY    0x10
#define IST8310_ID_KEY          IST8310_WHO_AM_I_KEY

#define IST8310_MAG_XL          0x03
#define IST8310_MAG_XH          0x04
#define IST8310_MAG_YL          0x05
#define IST8310_MAG_YH          0x06
#define IST8310_MAG_ZL          0x07
#define IST8310_MAG_ZH          0x08

#define IST8310_CTRL_REG        0x0A
#define IST8310_CTRL_VALUE      0x01

void IST8310_init(I2C_HandleTypeDef* i2c);
void IST8310_reset();
void IST8310_get_raw_data(uint8_t* data);
void IST8310_get_3d_data(float* data);

#endif //QUA_CONTROL_IST8310_H
