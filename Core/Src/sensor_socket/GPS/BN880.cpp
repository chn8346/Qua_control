//
// Created by fanchunhui on 2022/11/9.
//

#include "BN880.h"

void bn880::put_byte(const uint8_t *c) {
    if(*c != '$'){
        this->data[this->data_top] = *c;
        this->data_top = this->data_top + 1;
    }
    else if(*c == ',')
    {
        if(this->state_no == 0)
        {
            this->state_no = this->state_no + 1;
            if(this->data[5] == 'A')
            {
                this->type=GPS_TYPE_GNGGA;
            }
            else if(this->data[5] == 'G')
            {
                this->type=GPS_TYPE_GNVTG;
            }
            else if(this->data[5] == 'C')
            {
                this->type=GPS_TYPE_GNRMC;
            }
        }
        else
        {
            this->state_no = this->state_no + 1;
        }
    }
    else if(*c == '*')
    {

    }
    else{
        this->phaser();
        this->data_top = 1;
        this->data[0] = *c;
    }
}

void bn880::phaser() {
    // 解析信息头
    if(this->data[5] == 'A')
    {
        ;
    }
    else if(this->data[5] == 'G')
    {
        ;
    }
    else if(this->data[5] == 'C')
    {
        ;
    }
}

void bn880::get_state(gps_info* info_) {
    info_ = &(this->info);
}

void bn880::get_state_str(char *str_info) {

}
