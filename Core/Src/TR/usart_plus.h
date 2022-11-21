//
// Created by fanchunhui on 2022/11/9.
//

#ifndef QUA_CONTROL_USART_PLUS_H
#define QUA_CONTROL_USART_PLUS_H

#include "../base_head.h"


void init_uart_to_pc(UART_HandleTypeDef* uart_in);

void Serial_Data_Send(uint8_t* nclink_databuf, uint8_t _cnt);

void uart_receive(uint8_t* data);

void uart_wireless_send(uint8_t* data, uint8_t len);

#endif //QUA_CONTROL_USART_PLUS_H
