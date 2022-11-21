//
// Created by fanchunhui on 2022/11/10.
//

#ifndef QUA_CONTROL_CRC16_H
#define QUA_CONTROL_CRC16_H

#include "../base_head.h"

void crc16(uint8_t * data,uint8_t data_len_pcs,uint8_t* ret_crc);
void crc16c(unsigned char*, unsigned char, unsigned char*);
void test();
#endif //QUA_CONTROL_CRC16_H
