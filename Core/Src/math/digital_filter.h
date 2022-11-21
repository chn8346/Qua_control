//
// Created by fanchunhui on 2022/11/21.
//

#ifndef QUA_CONTROL_DIGITAL_FILTER_H
#define QUA_CONTROL_DIGITAL_FILTER_H

#include "../base_head.h"

// 多能数字滤波器，根据标定点进行滤波器设置
// len: 标定点长度
// freq: 标定频率
// amp_limit: 标定频率的限幅
// 程序将根据频率和限幅对数字滤波器进行生成
void digital_filter_multi_function(int len, float* freq, float* amp_limit);

#endif //QUA_CONTROL_DIGITAL_FILTER_H
