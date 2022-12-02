//
// Created by fanchunhui on 2022/11/26.
//

#ifndef QUA_CONTROL_ULTRA_SONIC_H
#define QUA_CONTROL_ULTRA_SONIC_H

#include "../../base_head.h"

#define DISTANCE_SIGNAL     0x55
#define TMP_SIGNAL          0x45

void sonic_requist_distance(UART_HandleTypeDef* huart);

#endif //QUA_CONTROL_ULTRA_SONIC_H
