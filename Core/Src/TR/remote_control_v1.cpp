//
// Created by fanchunhui on 2022/11/26.
//

#include "remote_control_v1.h"

uint8_t zero_pos = 1; // 指向数据头的位置

// 油门临界起飞点
float force_zero = REMOTE_CONTROL_LOW_PPM_usf;// + Qua_TAKE_OFF_P*REMOTE_CONTROL_DIAMETER_PPM_usf;

void remote_control_phase(uint32_t* ppm_data, float* ppm_phased){
    // ppm摇杆数据
    int temp = zero_pos + 1;

    if(temp >= 10)temp = 1;
    ppm_phased[0] = (float)ppm_data[temp++];       // CH1

    if(temp >= 10)temp = 1;
    ppm_phased[1] = (float)ppm_data[temp++];       // CH2

    if(temp >= 10)temp = 1;
    ppm_phased[2] = (float)ppm_data[temp++];       // CH3

    if(temp >= 10)temp = 1;
    ppm_phased[3] = (float)ppm_data[temp];         // CH4

    // 解析百分比数据
    ppm_phased[PPM_CH_FORCE] =(ppm_phased[PPM_CH_FORCE] - force_zero);
//    if(ppm_phased[PPM_CH_FORCE] < REMOTE_CONTROL_DEATH_PPM_usf &&
//       ppm_phased[PPM_CH_FORCE] > -REMOTE_CONTROL_DEATH_PPM_usf){
//        ppm_phased[PPM_CH_FORCE] = 0;
//    }
    ppm_phased[PPM_CH_FORCE] = ppm_phased[PPM_CH_FORCE]/REMOTE_CONTROL_DIAMETER_PPM_usf;//REMOTE_CONTROL_RADIUS_PPM_usf;

    ppm_phased[PPM_CH_PITCH] = (ppm_phased[PPM_CH_PITCH] - REMOTE_CONTROL_MID_PPM_usf)/REMOTE_CONTROL_RADIUS_PPM_usf;

    ppm_phased[PPM_CH_YAW]   = (ppm_phased[PPM_CH_YAW]   - REMOTE_CONTROL_MID_PPM_usf);
    if(ppm_phased[PPM_CH_YAW] < REMOTE_CONTROL_DEATH_PPM_usf &&
       ppm_phased[PPM_CH_YAW] > -REMOTE_CONTROL_DEATH_PPM_usf){
        ppm_phased[PPM_CH_YAW] = 0;
    }
    ppm_phased[PPM_CH_YAW] = ppm_phased[PPM_CH_YAW]/REMOTE_CONTROL_RADIUS_PPM_usf;

    ppm_phased[PPM_CH_ROLL]  = (ppm_phased[PPM_CH_ROLL]  - REMOTE_CONTROL_MID_PPM_usf)/REMOTE_CONTROL_RADIUS_PPM_usf;
}

// 获取信息头的位置
uint16_t REMOTE_CONTROL_get_zero_index(uint32_t data[], uint16_t len){
    // 等待内八的数值最大值大于单帧最大值
    while(data[zero_pos] < REMOTE_CONTROL_FRAME_LEN_PPM_us)
        remote_max_in_list(data, len);

    return zero_pos;
}



void remote_max_in_list(uint32_t* data, uint16_t len){
    for(uint16_t i = 1; i < len; i++)
    {
        if(data[i] > data[zero_pos])
        {
            zero_pos = i;
        }
    }
}
