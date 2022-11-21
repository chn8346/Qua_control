//
// Created by fanchunhui on 2022/11/9.
//

#include "usart_plus.h"

UART_HandleTypeDef* uart_toPC = NULL;

void init_uart_to_pc(UART_HandleTypeDef* uart_in)
{
    uart_toPC = uart_in;
}

void Serial_Data_Send(uint8_t* nclink_databuf, uint8_t _cnt)
{
    if(uart_toPC == NULL)
    {
        return;
    }
    else
    {
        HAL_UART_Transmit(uart_toPC, nclink_databuf, _cnt, 1000);
    }
}

void uart_receive(uint8_t* data)
{
    HAL_UART_Receive(uart_toPC, data, 1, 1000);
}

void uart_wireless_send(uint8_t* data, uint8_t len)
{
    for(int i = 0; i < len; i++)
    {
        data[i] = 0x80 | data[i];
    }

    HAL_UART_Transmit(uart_toPC, (uint8_t*)data, len, 500);
}

