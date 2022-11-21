//
// Created by fanchunhui on 2022/11/9.
//

#ifndef QUA_CONTROL_BN880_H
#define QUA_CONTROL_BN880_H

#include "../../base_head.h"

typedef struct {
    // GNGGA GPS/北斗定位信息
    uint8_t time_hour_GNGGA;
    uint8_t time_min_GNGGA;
    uint8_t time_sec_GNGGA;
    uint16_t time_msec_GNGGA;
    uint8_t longitude_jd_GNGGA;
    uint8_t west_east_GNGGA;
    uint8_t latitude_wd_GNGGA;
    uint8_t north_south_GNGGA;
    uint8_t gps_state_GNGGA;
    uint8_t star_num_GNGGA;
    uint8_t hdop_GNGGA;
    uint8_t height_GNGGA;

    // GNRMC  推荐定位信息
    uint8_t locate_state_GNRMC;
    uint8_t speed_GNRMC;
    uint8_t yaw_GNRMC;
    uint8_t mag_dis_GNRMC;    // 磁偏角
    uint8_t mag_dis_dir_GNRMC;// 磁偏角方向
    uint8_t speed_get_mode_GNRMC; // 指示模式

    // GNVTG  地面速度信息
    uint8_t north_yaw_GNVTG; // 北方为基准的航向
    uint8_t mag_north_yaw_GNVTG; // 磁北为基准的航向
    uint8_t VTG_speed_GNVTG;  // 单位 节
    uint8_t VTG_speed2_GNVTG; // 单位 km/h
    uint8_t vtg_mode_GNVTG;  // 指示模式

} gps_info;

#define GPS_TYPE_NONE   0
#define GPS_TYPE_GNGGA  1
#define GPS_TYPE_GNRMC  2
#define GPS_TYPE_GNVTG  3

class bn880
{
protected:
    uint8_t data[256] = {0};
    uint16_t data_top = 0;
    gps_info info = {0};
    uint8_t state_no = 0;
    uint8_t type = GPS_TYPE_NONE;

public:
    void put_byte(const uint8_t* c);
    void get_state(gps_info* info_);
    void get_state_str(char* str_info);
private:
    void phaser();
};


#endif //QUA_CONTROL_BN880_H
