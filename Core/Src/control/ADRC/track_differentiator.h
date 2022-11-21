//
// Created by fanchunhui on 2022/11/21.
//

#ifndef QUA_CONTROL_TRACK_DIFFERENTIATOR_H
#define QUA_CONTROL_TRACK_DIFFERENTIATOR_H

// 跟踪微分器，比普通微分器的抗扰性能更好

#include "../../base_head.h"

// TD的传递函数形式为两个惯性环节相加

#define TD_DELAY 0.1    // 传递函数的delay时间，建议此项的长度大于噪声的周期时间，以获得最好的抗扰效果

void TD_get_div(float* source, float* d);

#endif //QUA_CONTROL_TRACK_DIFFERENTIATOR_H
