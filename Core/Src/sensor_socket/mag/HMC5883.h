//
// Created by fanchunhui on 2022/11/13.
//

#ifndef QUA_CONTROL_HMC5883_H
#define QUA_CONTROL_HMC5883_H

#include "../../base_head.h"

#define HMC5883_READ_ADDR    0x3D
#define HMC5883_WRITE_ADDR   0x3C

#define HMC5883_CONFIG_A    0x00
#define HMC5883_CONFIG_B    0x01
#define HMC5883_MODE_REG    0x02
#define HMC5883_XH          0x03
#define HMC5883_XL          0x04
#define HMC5883_YH          0x05
#define HMC5883_YL          0x06
#define HMC5883_ZH          0x07
#define HMC5883_ZL          0x08
#define HMC5883_STATE       0x09
#define HMC5883_IDA         0x0A
#define HMC5883_IDB         0x0B
#define HMC5883_IDC         0x0C

#define HMC5883_IDA_VALUE   0x48
#define HMC5883_IDB_VALUE   0x34
#define HMC5883_IDC_VALUE   0x33
#define HMC5883_MODE_VALUE  0x00
#define HMC5883_CONF_B_V    0x20
#define HMC5883_CONF_A_V    0x38

void HMC5883_init(I2C_HandleTypeDef * i2c);
void HMC5883_reset();
void HMC5883_read(uint8_t* m);
void HMC5883_read_and_process(float* mag);
void HMC5883_read_and_process(uint8_t* m, float* mag);

#endif //QUA_CONTROL_HMC5883_H
