//
// Created by fanchunhui on 2022/11/21.
//

#ifndef QUA_CONTROL_DIGITAL_FILTER_H
#define QUA_CONTROL_DIGITAL_FILTER_H

#include "../../base_head.h"

// 描述 [非连续传递函数] 的结构体
typedef struct {
    float tf_den[10];
    uint8_t tf_den_len;
    float tf_num[10];
    uint8_t tf_num_len;
} trans_fun_disc;

#define DF_TYPE_DTF  0  // 滤波器类型：离散传递函数构成的滤波器
#define DF_TYPE_BT   1  // 滤波器类型：巴特沃斯滤波器
#define DF_TYPE_Q    2  // 滤波器类型：切比雪夫滤波器

// 数字滤波器类，用于可用的生成数字滤波器
class digital_filter{
public:
    void filter_init(uint8_t df_type_, trans_fun_disc* tf);
    void filter_go();
private:
    trans_fun_disc* tfd;  // 用于指定传递函数类型
    uint8_t df_type;      // 滤波器类型

};

// 多能数字滤波器，根据标定点进行滤波器设置
// len: 标定点长度
// freq: 标定频率
// amp_limit: 标定频率的限幅
// 程序将根据频率和限幅对数字滤波器进行生成
void digital_filter_multi_function(int len, float* freq, float* amp_limit);

// 巴特沃斯滤波器参数
#define FILTER_BT_N1  {1.0000000}
#define FILTER_BT_N2  {1.4142136}
#define FILTER_BT_N3  {2.0000000,2.0000000}
#define FILTER_BT_N4  {2.6131259,3.4142136 ,2.6131259}
#define FILTER_BT_N5  {3.2360680,5.2360680 ,5.2360680 ,3.2360680}
#define FILTER_BT_N6  {3.8637033,7.4641016 ,9.1416202 ,7.4641016 ,3.8637033}
#define FILTER_BT_N7  {4.4939592,10.0978347,14.5917939,14.5917939,10.0978347,4.4939592}
#define FILTER_BT_N8  {5.1258309,13.1370712,21.8461510,25.6883559,21.8461510,13.1370712,5.1258309}
#define FILTER_BT_N9  {5.7587705,16.5817187,31.1634375,41.9863857,41.9863857,31.1634375,16.5817187,5.7587705}
#define FILTER_BT_N10 {6.3924532,20.4317291,42.8020611,64.8823963,74.2334292,64.8823963,42.8020611,20.4317291,6.3924532}

#endif //QUA_CONTROL_DIGITAL_FILTER_H
