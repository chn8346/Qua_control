//
// Created by fanchunhui on 2022/11/9.
//

#ifndef QUA_CONTROL_ICM20602_H
#define QUA_CONTROL_ICM20602_H

#include "../../base_head.h"

#define ICM_ADDR_NO_PULL    (0x68<<1)
#define ICM_ADDR_PULL       (0x69<<1)
#define ICM_ADDR            ICM_ADDR_NO_PULL

#define ICM_WHO_AM_I_ADDR   0x75
#define ICM_WHO_AM_I_KEY    0x12
#define ICM_ID_ADDR         ICM_WHO_AM_I_ADDR
#define ICM_ID_KEY          ICM_WHO_AM_I_KEY

#define ICM_POWER_MNG2          0x6C
#define ICM_POWER_MNG           0x6B
#define ICM_CTRL                0x6A
#define ICM_ACC_CONF2           0x1D
#define ICM_ACC_CONF            0x1C
#define ICM_GYRO_CONF           0x1B
#define ICM_CONFIG_REG          0x1A

#define ICM_POWER_MNG2_VALUE    0x00
#define ICM_POWER_MNG_VALUE     0x00
#define ICM_CTRL_VALUE          0x00
#define ICM_ACC_CONF2_VALUE     0x11
#define ICM_ACC_CONF_VALUE      0x08
#define ICM_GYRO_CONF_VALUE     0x10
#define ICM_CONFIG_REG_VALUE    0x01


#define ICM_GYRO_XH             0x43
#define ICM_GYRO_XL             0x44
#define ICM_GYRO_YH             0x45
#define ICM_GYRO_YL             0x46
#define ICM_GYRO_ZH             0x47
#define ICM_GYRO_ZL             0x48

#define ICM_TMP_H               0x41
#define ICM_TMP_L               0x42

#define ICM_ACC_XH              0x3B
#define ICM_ACC_XL              0x3C
#define ICM_ACC_YH              0x3D
#define ICM_ACC_YL              0x3E
#define ICM_ACC_ZH              0x3F
#define ICM_ACC_ZL              0x40


void ICM_init(I2C_HandleTypeDef * i2c);
void ICM_reset();
void ICM_get_full_raw(uint8_t* data);
void ICM_get_raw(uint8_t* data);
void ICM_get_6_axis_data(float* data_);

#endif //QUA_CONTROL_ICM20602_H
