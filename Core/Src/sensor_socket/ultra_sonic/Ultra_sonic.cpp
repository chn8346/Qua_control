//
// Created by fanchunhui on 2022/11/26.
//

#include "Ultra_sonic.h"

void sonic_requist_distance(UART_HandleTypeDef* huart){
    uint8_t tx = 0x55;
    HAL_UART_Transmit(huart, &tx, 1, 500);
}
