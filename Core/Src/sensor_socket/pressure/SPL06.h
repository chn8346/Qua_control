//
// Created by fanchunhui on 2022/11/9.
//

#ifndef QUA_CONTROL_SPL06_P_H
#define QUA_CONTROL_SPL06_P_H

#include "../../base_head.h"

#define SPL06_ADDR_NO_PULL      0xEE    // 0x77<<1
#define SPL06_ADDR_PULL         0xEC    // 0x76<<1

#define SPL06_ADDR              SPL06_ADDR_NO_PULL

#define SPL06_WHO_AM_I_ADDR     0x0D
#define SPL06_WHO_AM_I_KEY      0x10
#define SPL06_ID_ADDR           SPL06_WHO_AM_I_ADDR
#define SPL06_ID_KEY            SPL06_WHO_AM_I_KEY

#define SPL06_PRESSURE_0_ADDR   0x02
#define SPL06_PRESSURE_1_ADDR   0x01
#define SPL06_PRESSURE_2_ADDR   0x00

#define SPL06_TMP_0_ADDR        0x05
#define SPL06_TMP_1_ADDR        0x04
#define SPL06_TMP_2_ADDR        0x03

#define SPL06_PRS_CFG           0x06    // psr sample config 气压采样设置
#define SPL06_TMP_CFG           0x07    // 温度采样设置
#define SPL06_MEANS_CFG         0x08    // 当前传感器状态
#define SPL06_CFG_REG           0x09    // 配置中断和位移(bias)


// 气压计采样速度
#define SPL06_PRESSURE_CONFIG_RATE_1      (0<<4)
#define SPL06_PRESSURE_CONFIG_RATE_2      (1<<4)
#define SPL06_PRESSURE_CONFIG_RATE_4      (2<<4)
#define SPL06_PRESSURE_CONFIG_RATE_8      (3<<4)
#define SPL06_PRESSURE_CONFIG_RATE_16     (4<<4)
#define SPL06_PRESSURE_CONFIG_RATE_32     (5<<4)
#define SPL06_PRESSURE_CONFIG_RATE_64     (6<<4)
#define SPL06_PRESSURE_CONFIG_RATE_128    (7<<4)

// 超采样速度
#define SPL06_PRESSURE_CONFIG_OVER_RATE_1     0
#define SPL06_PRESSURE_CONFIG_OVER_RATE_2     1
#define SPL06_PRESSURE_CONFIG_OVER_RATE_4     2
#define SPL06_PRESSURE_CONFIG_OVER_RATE_8     3
#define SPL06_PRESSURE_CONFIG_OVER_RATE_16    4
#define SPL06_PRESSURE_CONFIG_OVER_RATE_32    5
#define SPL06_PRESSURE_CONFIG_OVER_RATE_64    6
#define SPL06_PRESSURE_CONFIG_OVER_RATE_128   7

// 默认的气压计性能设置
#define SPL06_PRESSURE_CONFIG_HIGH_PERFORM     ((SPL06_PRESSURE_CONFIG_RATE_4)|(SPL06_PRESSURE_CONFIG_OVER_RATE_64))
#define SPL06_PRESSURE_CONFIG_NORMAL_PERFORM   ((SPL06_PRESSURE_CONFIG_RATE_2)|(SPL06_PRESSURE_CONFIG_OVER_RATE_16))
#define SPL06_PRESSURE_CONFIG_LOW_PERFORM      ((SPL06_PRESSURE_CONFIG_RATE_1)|(SPL06_PRESSURE_CONFIG_OVER_RATE_2))

// 气压计采样速度
#define SPL06_TMP_CONFIG_RATE_1      (0<<4)
#define SPL06_TMP_CONFIG_RATE_2      (1<<4)
#define SPL06_TMP_CONFIG_RATE_4      (2<<4)
#define SPL06_TMP_CONFIG_RATE_8      (3<<4)
#define SPL06_TMP_CONFIG_RATE_16     (4<<4)
#define SPL06_TMP_CONFIG_RATE_32     (5<<4)
#define SPL06_TMP_CONFIG_RATE_64     (6<<4)
#define SPL06_TMP_CONFIG_RATE_128    (7<<4)

// 超采样速度
#define SPL06_TMP_CONFIG_OVER_RATE_1     0
#define SPL06_TMP_CONFIG_OVER_RATE_2     1
#define SPL06_TMP_CONFIG_OVER_RATE_4     2
#define SPL06_TMP_CONFIG_OVER_RATE_8     3
#define SPL06_TMP_CONFIG_OVER_RATE_16    4
#define SPL06_TMP_CONFIG_OVER_RATE_32    5
#define SPL06_TMP_CONFIG_OVER_RATE_64    6
#define SPL06_TMP_CONFIG_OVER_RATE_128   7

// 默认的气压计性能设置
#define SPL06_TMP_CONFIG_LOW_PERFORM     ((SPL06_TMP_CONFIG_RATE_1)|(SPL06_TMP_CONFIG_OVER_RATE_1)|0x80)
#define SPL06_TMP_CONFIG_NORMAL_PERFORM   ((SPL06_TMP_CONFIG_RATE_1)|(SPL06_TMP_CONFIG_OVER_RATE_1)|0x80)
#define SPL06_TMP_CONFIG_HIGH_PERFORM      ((SPL06_TMP_CONFIG_RATE_2)|(SPL06_TMP_CONFIG_OVER_RATE_1)|0x80)


// 传感器系统设置
#define SPL06_MODE_STOP                     0
#define SPL06_MODE_P_MEASURE                5
#define SPL06_MODE_T_MEASURE                6
#define SPL06_MODE_TP_BOTH_MEASURE          7

// 设置中断和位移
#define SPL06_CFG_REG_P_SHIFT               0x04; // P偏移设置,高采样必须设置(over sample > 8) 0000 0100

// 气压计算常数

// 温度计算常数

void SPL06_init(I2C_HandleTypeDef* i2c);
uint8_t SPL06_get_id();
void SPL06_reset();
void SPL06_get_raw_P(uint8_t* raw_data);
void SPL06_get_raw_T(uint8_t* raw_data);
void SPL06_get_raw_data(uint8_t* raw_data);
float SPL06_get_tmp();
float SPL06_get_pressure();
float SPL06_get_height();


#endif //QUA_CONTROL_SPL06_P_H
