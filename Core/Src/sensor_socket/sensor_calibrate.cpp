//
// Created by fanchunhui on 2022/11/13.
//

#include "sensor_calibrate.h"
#include "gyro_acc/MPU6050.h"
#include "mag/Mag3110.h"

extern int send_sgn;

// 模块内部参数
calibrate_parameter cpr;
I2C_HandleTypeDef * i2c1_clb;
I2C_HandleTypeDef * i2c2_clb;

// 快速长度计算函数
uint8_t str_len_cbl(const char * x){int i=0;while(x[i])if(x[i++]=='\n')break;return i;}

void calibrate_get_i2c_spi(I2C_HandleTypeDef* i2c1,
                           I2C_HandleTypeDef* i2c2,
                           SPI_HandleTypeDef* spi1,
                           SPI_HandleTypeDef* spi2)
{
    i2c1_clb = i2c1;
    i2c2_clb = i2c2;
}

void delay_clb_n_10ms(uint16_t n_10ms)
{
    while (n_10ms > 0)
    {
        if(send_sgn != 0)
        {
            n_10ms = n_10ms - 1;
            send_sgn = 0;
        }
    }
}

#define alter_num  1

void delay_clb_alter(int n)
{
    delay_clb_n_10ms(n);
}

float invSqrt1_cbl(float x);

#define acc_fix_circle_num  600
#define acc_fix_circle_num_float  ((float)acc_fix_circle_num)

#define gyro_fix_circle_num 600
#define gyro_fix_circle_num_float  ((float)gyro_fix_circle_num)

#define mag_fix_circle_num 600
#define mag_fix_circle_num_float  ((float)mag_fix_circle_num)

void calibrate_at_init(UART_HandleTypeDef* uart) {
    for(float & i : cpr.mid)i = 0;
    for(int i = 0; i < 6; i++)cpr.mag_limit[i] = 1+(float)i;
    for(float & i : cpr.angle_gain_rad)i = 1;
    for(float & i : cpr.mag_half_range)i = 1;
    cpr.gravity_value = 0.0;

    uint8_t data[12];
    float datap[6];
    uint8_t mag_data[6];
    float mag_fdata[3];

    // 平放校准加速度（x,y）和角速度(x, y, z)
    uint8_t msg[100] = {0};
    sprintf((char *) msg, "place flat to fix acc\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char *) msg));
    for (int i = 0; i < acc_fix_circle_num; i++) {
//        MAG3110_read(i2c1_clb, mag_data);
//        MAG3110_process2(mag_data, mag_fdata);
        MAG_read_data(mag_data, mag_fdata);
        IMU_read_data(data, datap);
//        imu6050_read(i2c2_clb, data, 1);
//        imu_process(data, datap);
        // TODO 【不是废话】 传感器没有统一的数据结构封装，从软件可持续开发考虑，最好进行定义
        cpr.mid[CLB_ACCX] = cpr.mid[CLB_ACCX] + datap[0] / acc_fix_circle_num_float;
        cpr.mid[CLB_ACCY] = cpr.mid[CLB_ACCY] + datap[1] / acc_fix_circle_num_float;
        cpr.mid[CLB_ACCZ] = cpr.mid[CLB_ACCZ] + datap[2] / acc_fix_circle_num_float;
        cpr.mid[CLB_GX]   = cpr.mid[CLB_GX]   + datap[3] / acc_fix_circle_num_float;
        cpr.mid[CLB_GY]   = cpr.mid[CLB_GY]   + datap[4] / acc_fix_circle_num_float;
        cpr.mid[CLB_GZ]   = cpr.mid[CLB_GZ]   + datap[5] / acc_fix_circle_num_float;

        // 下面三行，不作数的，后面会对这些数值重测，留着只是为了延长循环实际，采样速度太快了，可能造成校准需要比较快的反应速度
        cpr.mid[CLB_MAGX] = cpr.mid[CLB_MAGX] + mag_fdata[0] / acc_fix_circle_num_float;
        cpr.mid[CLB_MAGY] = cpr.mid[CLB_MAGY] + mag_fdata[1] / acc_fix_circle_num_float;
        cpr.mid[CLB_MAGZ] = cpr.mid[CLB_MAGZ] + mag_fdata[2] / acc_fix_circle_num_float;

        // 气压计还没到位，等待 TODO 【等待】 气压计到位之后再做
        // cpr.mid[CLB_PSR]  = cpr.mid[CLB_PSR]  + mag_fdata[0]/acc_fix_circle_num_float;

        delay_clb_n_10ms(1);
    }

    // 角速度积分测试 -- 偏航
    sprintf((char *) msg, "yaw to 90 degrees\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char *) msg));
    float in_angle = 0;
    for (int i = 0; i < gyro_fix_circle_num; i++)
    {
        IMU_read_data(data, datap);
//        imu6050_read(i2c2_clb, data, 1);
//        imu_process(data, datap);
        in_angle = in_angle + datap[5]*(float)DELTA_T; // z轴角速度积分

        // 延时
        delay_clb_alter(alter_num);
    }
    // 转动pi/4 和具体的积分角度进行比值计算，得到角度增益
    cpr.angle_gain_rad[2] = ((float)(3.1415926/4))/in_angle;

    // 角速度积分测试 -- 俯仰
    sprintf((char *)msg, "place flat , IN 5 SEC\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));
    delay_clb_n_10ms(500);


    sprintf((char *)msg, "pitch to 90 degrees\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));
    in_angle = 0;
    for (int i = 0; i < gyro_fix_circle_num; i++)
    {
//        imu6050_read(i2c2_clb, data, 1);
//        imu_process(data, datap);
        IMU_read_data(data, datap);

        in_angle = in_angle + datap[4]*(float)DELTA_T; // z轴角速度积分

        // 延时
        delay_clb_alter(alter_num);
    }
    // 转动pi/4 和具体的积分角度进行比值计算，得到角度增益
    cpr.angle_gain_rad[1] = ((float)(3.1415926/4))/in_angle;

    // 角速度积分测试 -- 滚转
    sprintf((char *)msg, "place flat , IN 5 SEC\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));
    delay_clb_n_10ms(500);

    sprintf((char *)msg, "roll to 90 degrees\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));
    in_angle = 0;
    for (int i = 0; i < gyro_fix_circle_num; i++)
    {
//        imu6050_read(i2c2_clb, data, 1);
//        imu_process(data, datap);
        IMU_read_data(data, datap);
        in_angle = in_angle + datap[3]*(float)DELTA_T; // z轴角速度积分

        // 延时
        delay_clb_alter(alter_num);
    }
    // 转动pi/4 和具体的积分角度进行比值计算，得到角度增益
    cpr.angle_gain_rad[0] = ((float)(3.1415926/4))/in_angle;

    // 90度校准 z 轴加速度
    sprintf((char *)msg, "place flat , IN 5 SEC\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));
    delay_clb_n_10ms(500);

    // 90度校准 z 轴加速度 -- 同上
    sprintf((char *)msg, "roll OR pitch to 90 degrees\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));
    delay_clb_n_10ms(800);
    cpr.mid[CLB_ACCZ] = 0; // 归零，重新计算
    for (int i = 0; i < acc_fix_circle_num; i++) {
//        imu6050_read(i2c2_clb, data, 1);
//        imu_process(data, datap);
        IMU_read_data(data, datap);
        // TODO 传感器没有统一的数据结构封装
        cpr.mid[CLB_ACCZ] = cpr.mid[CLB_ACCZ] + datap[2] / acc_fix_circle_num_float;

        // 延时
        delay_clb_alter(alter_num);
    }

    // 旋转校准磁力计 & 陀螺仪
    sprintf((char *)msg, "place flat , IN 5 SEC\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));
    delay_clb_n_10ms(500);

    // 旋转校准磁力计 & 陀螺仪 -- 同上
    sprintf((char *)msg, "rot at flat surface to 360 degrees\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));
    cpr.mag_limit[0] = -70000; // x upper
    cpr.mag_limit[1] = 70000;  // x low
    cpr.mag_limit[2] = -70000; // y
    cpr.mag_limit[3] = 70000;
    cpr.mag_limit[4] = -70000; // z
    cpr.mag_limit[5] = 70000;
    for(int i = 0; i < mag_fix_circle_num; i++)
    {
//        MAG3110_read(i2c1_clb, mag_data);
//        // MAG3110_process2(mag_data, mag_fdata);
//        MAG3110_process3(mag_data, mag_fdata);
        MAG_read_data(mag_data, mag_fdata);

        cpr.mag_limit[0] = cpr.mag_limit[0] > mag_fdata[0] ?  cpr.mag_limit[0]:mag_fdata[0];
        cpr.mag_limit[1] = cpr.mag_limit[1] < mag_fdata[0] ?  cpr.mag_limit[1]:mag_fdata[0];
        cpr.mag_limit[2] = cpr.mag_limit[2] > mag_fdata[1] ?  cpr.mag_limit[2]:mag_fdata[1];
        cpr.mag_limit[3] = cpr.mag_limit[3] < mag_fdata[1] ?  cpr.mag_limit[3]:mag_fdata[1];
        cpr.mag_limit[4] = cpr.mag_limit[4] > mag_fdata[2] ?  cpr.mag_limit[4]:mag_fdata[2];
        cpr.mag_limit[5] = cpr.mag_limit[5] < mag_fdata[2] ?  cpr.mag_limit[5]:mag_fdata[2];

        // 延时
        delay_clb_alter(alter_num);
    }

    // 磁力计校准，尽可能覆盖所有可能性
    sprintf((char *)msg, "rot any angle as complete as possible ... \n");
    delay_clb_n_10ms(100);
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));
    for(int i = 0; i < mag_fix_circle_num*2; i++)
    {
//        MAG3110_read(i2c1_clb, mag_data);
//        // MAG3110_process2(mag_data, mag_fdata);
//        MAG3110_process3(mag_data, mag_fdata);
        MAG_read_data(mag_data, mag_fdata);

        cpr.mag_limit[0] = cpr.mag_limit[0] > mag_fdata[0] ?  cpr.mag_limit[0]:mag_fdata[0];
        cpr.mag_limit[1] = cpr.mag_limit[1] < mag_fdata[0] ?  cpr.mag_limit[1]:mag_fdata[0];
        cpr.mag_limit[2] = cpr.mag_limit[2] > mag_fdata[1] ?  cpr.mag_limit[2]:mag_fdata[1];
        cpr.mag_limit[3] = cpr.mag_limit[3] < mag_fdata[1] ?  cpr.mag_limit[3]:mag_fdata[1];
        cpr.mag_limit[4] = cpr.mag_limit[4] > mag_fdata[2] ?  cpr.mag_limit[4]:mag_fdata[2];
        cpr.mag_limit[5] = cpr.mag_limit[5] < mag_fdata[2] ?  cpr.mag_limit[5]:mag_fdata[2];

        // 延时
        delay_clb_alter(alter_num);
    }
    cpr.mid[CLB_MAGX] = cpr.mag_limit[0]*(float)0.5 + cpr.mag_limit[1]*(float)0.5;
    cpr.mid[CLB_MAGY] = cpr.mag_limit[2]*(float)0.5 + cpr.mag_limit[3]*(float)0.5;
    cpr.mid[CLB_MAGZ] = cpr.mag_limit[4]*(float)0.5 + cpr.mag_limit[5]*(float)0.5;
    cpr.mag_half_range[0] = cpr.mag_limit[0]*(float)0.5 - cpr.mag_limit[1]*(float)0.5;
    cpr.mag_half_range[1] = cpr.mag_limit[2]*(float)0.5 - cpr.mag_limit[3]*(float)0.5;
    cpr.mag_half_range[2] = cpr.mag_limit[4]*(float)0.5 - cpr.mag_limit[5]*(float)0.5;

#define acc_shrink_scale 100   // 为防止溢出而设置的重力加速度缩小倍数

    sprintf((char *)msg, "place flat, measuring real gravity ... start in 5 sec\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));
    delay_clb_n_10ms(500);

    float ax_temp = 0, ay_temp = 0, az_temp = 0;
    for(int i = 0; i < acc_fix_circle_num; i++)
    {
//        imu6050_read(i2c2_clb, data, 1);
//        imu_process(data, datap);
        IMU_read_data(data, datap);
        ax_temp = ax_temp + (datap[0] - cpr.mid[CLB_ACCX])/acc_shrink_scale/acc_fix_circle_num_float;
        ay_temp = ay_temp + (datap[1] - cpr.mid[CLB_ACCY])/acc_shrink_scale/acc_fix_circle_num_float;
        az_temp = az_temp + (datap[2] - cpr.mid[CLB_ACCZ])/acc_shrink_scale/acc_fix_circle_num_float;

        // 延时
        delay_clb_alter(alter_num);
    }
    cpr.gravity_value = sqrt(ax_temp*ax_temp + ay_temp*ay_temp + az_temp*az_temp);

    // 校准完成提示
    sprintf((char *)msg, "place flat, calibrate OK ... start in 5 sec\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));
    delay_clb_alter(5);

    /*
     *            ---------------  下面开始传送校准信息  ---------------
     * */

    // 校准信息开始提示
    sprintf((char *)msg, "The calibrate data below:\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));
    delay_clb_alter(5);

    // 传感器零点位置
    sprintf((char *)msg, " BIAS_ACC(xyz): %.4f, %.4f, %.4f\n", cpr.mid[0], cpr.mid[1], cpr.mid[2]);
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));delay_clb_alter(5);
    sprintf((char *)msg, "BIAS_GYRO(xyz): %.4f, %.4f, %.4f\n", cpr.mid[3], cpr.mid[4], cpr.mid[5]);
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));delay_clb_alter(5);
    sprintf((char *)msg, " BIAS_MAG(xyz): %.4f, %.4f, %.4f\n", cpr.mid[6], cpr.mid[7], cpr.mid[8]);
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));delay_clb_alter(5);

    // 陀螺仪转动增益（角速度乘dt再乘多少等于真实角度）
    sprintf((char *)msg, "Angle Rate Gain(x10000): ROLL--%.8f  ,  PITCH--%.8f  ,  YAW--%.8f\n"
            , cpr.angle_gain_rad[0]*10000.0
            , cpr.angle_gain_rad[1]*10000.0
            , cpr.angle_gain_rad[2]*10000.0);
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));delay_clb_alter(5);

    // 磁力计数值半径
    sprintf((char *)msg, "Mag Half Range(xyz): %.4f, %.4f, %.4f\n"
            , cpr.mag_half_range[0]
            , cpr.mag_half_range[1]
            , cpr.mag_half_range[2]);
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));delay_clb_alter(5);

    // 实际的重力数值（数值缩小100倍）
    sprintf((char *)msg, "Real Gravity Value(x0.01): %.4f\n", cpr.gravity_value);
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));delay_clb_alter(5);

    sprintf((char *)msg, "calibrate data above ... start in 5 sec\n");
    uart_transmit_with_next_sensor_clb(uart, msg, str_len_cbl((char*)msg));

    delay_clb_n_10ms(300);


    // TODO 1 测量加速度的高低值
    // TODO 2 测量气压传感器数据
}

void calibrate_at_init()       // 不加参数，自动使用经验参数启动
{
    // 加速度和陀螺仪零点数据
    float ga_mid[6] = CBL_MPU6050_ACC_GYRO_BIAS;
    cpr.mid[0] = ga_mid[0];
    cpr.mid[1] = ga_mid[1];
    cpr.mid[2] = ga_mid[2];
    cpr.mid[3] = ga_mid[3];
    cpr.mid[4] = ga_mid[4];
    cpr.mid[5] = ga_mid[5];

    // float m_mid[3] = CBL_MAG3110_BIAS;
    float m_mid[3] = CBL_HMC5883_BIAS;
    cpr.mid[6] = m_mid[0];
    cpr.mid[7] = m_mid[1];
    cpr.mid[8] = m_mid[2];

    float gain[3] = CBL_MPU6050_ANGLE_RATE_GAIN;
    cpr.angle_gain_rad[0] = gain[0];
    cpr.angle_gain_rad[1] = gain[1];
    cpr.angle_gain_rad[2] = gain[2];

    cpr.gravity_value = CBL_MPU6050_GRAVITY_VALUE;

    // float range[3] =CBL_MAG3110_HALF_RANGE;
    float range[3] =CBL_HMC5883_HALF_RANGE;
    cpr.mag_half_range[0] = range[0];
    cpr.mag_half_range[1] = range[1];
    cpr.mag_half_range[2] = range[2];
}

// 飞行时刻气压测量
void psr_calibrate_at_flying()
{

}

// 迭代时对数值进行校正，四元数用来校正磁力计的缺陷数值，全为0时不做修改
void calibrate_return_data(float * origin_acc, float * origin_angle, float * origin_mag, float q1, float q2, float q3, float q4)
{
    // 去偏移 & 缩小数值防止溢出
    origin_acc[CLB_ACCX] = (origin_acc[CLB_ACCX] - cpr.mid[CLB_ACCX])/acc_shrink_scale;
    origin_acc[CLB_ACCY] = (origin_acc[CLB_ACCY] - cpr.mid[CLB_ACCY])/acc_shrink_scale;
    origin_acc[CLB_ACCZ] = (origin_acc[CLB_ACCZ] - cpr.mid[CLB_ACCZ])/acc_shrink_scale;

    // 归一化增益
    float acc_inq = 1/(origin_acc[CLB_ACCX]*origin_acc[CLB_ACCX]
                     + origin_acc[CLB_ACCY]*origin_acc[CLB_ACCY]
                     + origin_acc[CLB_ACCZ]*origin_acc[CLB_ACCZ]);

    // 归一化，对于数值大于重力值，有大于一的部分
    origin_acc[CLB_ACCX] = cpr.gravity_value * acc_inq * origin_acc[CLB_ACCX];
    origin_acc[CLB_ACCY] = cpr.gravity_value * acc_inq * origin_acc[CLB_ACCY];
    origin_acc[CLB_ACCZ] = cpr.gravity_value * acc_inq * origin_acc[CLB_ACCZ];

    origin_angle[0] = (origin_angle[0] - cpr.mid[CLB_GX])*cpr.angle_gain_rad[0];
    origin_angle[1] = (origin_angle[1] - cpr.mid[CLB_GY])*cpr.angle_gain_rad[1];
    origin_angle[2] = (origin_angle[2] - cpr.mid[CLB_GZ])*cpr.angle_gain_rad[2];

    // origin_mag[0] = origin_mag[0] - cpr.mid[CLB_MAGX];
    // origin_mag[0] = (origin_mag[0] - cpr.mid[CLB_MAGX])/cpr.mag_half_range[0];
    // 对磁力计数据进行校正
    origin_mag[0] = (origin_mag[0] - cpr.mid[CLB_MAGX])/cpr.mag_half_range[0];
    origin_mag[1] = (origin_mag[1] - cpr.mid[CLB_MAGY])/cpr.mag_half_range[1];
    origin_mag[2] = (origin_mag[2] - cpr.mid[CLB_MAGZ])/cpr.mag_half_range[2];

    // 使用四元数解算角度，对磁力计数据进行纠偏
    if(q1!=0 && q2!=0 && q3!=0 && q4!=0){
        // 事实证明无需修正
    }

    // 磁力计数据归一化
    float mag_inq = invSqrt1_cbl(origin_mag[0]*origin_mag[0]
                                  + origin_mag[1]*origin_mag[1]
                                  + origin_mag[2]*origin_mag[2]);

    origin_mag[0] = mag_inq * origin_mag[0];
    origin_mag[1] = mag_inq * origin_mag[1];
    origin_mag[2] = mag_inq * origin_mag[2];
}

void uart_transmit_with_next_sensor_clb(UART_HandleTypeDef* uart, uint8_t* data, uint8_t len)
{
    for(int i = 0; i < len-1; i++)
    {
        data[i] = 0x80 | data[i];
    }
    data[len] = 0xff;
    HAL_UART_Transmit(uart, (uint8_t*)data, len, 500);
}

#define GAUE_MODE_QUAT  0
#define GAUE_MODE_ANGLE 1
#define GAUE_MODE GAUE_MODE_ANGLE

void gravity_altitude_uni_estimate(const float* gravity_norm, float* altitude_or_q, float yaw)
{
    // 假设我们用的是角度进行求解
#if GAUE_MODE == GAUE_MODE_ANGLE
    float beta[3];
    beta[0] = gravity_norm[0];
    beta[1] = gravity_norm[1];
    beta[2] = gravity_norm[2];
    // TODO 【急】 新算法还没有落地

#endif
}

// 对Q-X构型进行二次数据加工
void calibrate_QX(float * origin_acc, float * origin_angle, float * origin_mag){
//        [   cos(u),  sin(u),   0
//           -sin(u),  cos(u),   0
//              0   ,    0   ,   1  ];

//         0.7071067811865476

float t1, t2;

t1 = -origin_acc[0]*(float)0.7071068 + origin_acc[1]*(float)0.7071068;
t2 = origin_acc[0]*(float)0.7071068 + origin_acc[1]*(float)0.7071068;

origin_acc[0] = t1;
origin_acc[1] = t2;

t1 = -origin_angle[0]*(float)0.7071068 + origin_angle[1]*(float)0.7071068;
t2 = origin_angle[0]*(float)0.7071068 + origin_angle[1]*(float)0.7071068;

origin_angle[0] = t1;
origin_angle[1] = t2;

t1 = -origin_mag[0]*(float)0.7071068 + origin_mag[1]*(float)0.7071068;
t2 = origin_mag[0]*(float)0.7071068 + origin_mag[1]*(float)0.7071068;

origin_mag[0] = t1;
origin_mag[1] = t2;

}

// 求取平方根倒数
float invSqrt1_cbl(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
