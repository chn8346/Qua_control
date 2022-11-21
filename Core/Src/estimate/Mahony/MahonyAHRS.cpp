//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================


//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	100.0f			// sample frequency in Hz  采样频率
#define twoKpDef	(2.0f * 2.0f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 1.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

// 比例系数的两倍
volatile float twoKp = twoKpDef;                                            // 2 * proportional gain (Kp)

// 积分系数的两倍
volatile float twoKi = twoKiDef;                                            // 2 * integral gain (Ki)

// 传感器坐标系 向 auxiliary坐标系 转换的四元数 
volatile float q0x = 1.0f, q1x = 0.0f, q2x = 0.0f, q3x = 0.0f;                    // quaternion of sensor frame relative to auxiliary frame

// 积分误差
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki

// 误差值，用于DEBUG
volatile float erx = 0.0f, ery = 0.0f, erz = 0.0f;

//---------------------------------------------------------------------------------------------------
// Function declarations

// 求取平方根倒数
float invSqrt1(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
// gyroscope 陀螺仪			// accelerate  加速度计			// magnetic 磁力计
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;    // 临时变量，记录测量值序列的模长（正则化需要用到）
    float q0xq0x, q0xq1x, q0xq2x, q0xq3x, q1xq1x, q1xq2x, q1xq3x, q2xq2x, q2xq3x, q3xq3x;  // 四元数乘积
    float hx, hy, bx, bz;                                                //
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;                //
    float halfex, halfey, halfez;                                        //
    float qa, qb, qc;                                                    //

    // 磁力计如果没有测量值则使用下方算法                               避免NaN(not a num)数值的出现
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // 加速度计测量值可用时计算反馈									避免NaN(not a num)数值完成正则化
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 正则化【加速度】的测量值
        // Normalise accelerometer measurement
        recipNorm = (float)1.0/sqrt(ax * ax + ay * ay + az * az);    // ax,ay,az组合的范数的倒数
        // 除以范数，进行正则化
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;


        // 正则化【磁力计】的测量值
        // Normalise magnetometer measurement
        recipNorm = (float)1.0/sqrt(mx * mx + my * my + mz * mz);  // 求取序列范数
        // 除以范数，进行正则化
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        // 辅助变量：减少重复计算（转换矩阵的数值）
        q0xq0x = q0x * q0x;
        q0xq1x = q0x * q1x;
        q0xq2x = q0x * q2x;
        q0xq3x = q0x * q3x;
        q1xq1x = q1x * q1x;
        q1xq2x = q1x * q2x;
        q1xq3x = q1x * q3x;
        q2xq2x = q2x * q2x;
        q2xq3x = q2x * q3x;
        q3xq3x = q3x * q3x;

        // 地球磁场的参考方向
        //（假设是正北方，通过四元数的转换，把地球坐标系的正北方方向，转换到传感器坐标上）
        // Reference direction of Earth's magnetic field
        /**/
        hx = 2.0f * (mx * (0.5f - q2xq2x - q3xq3x) + my * (q1xq2x - q0xq3x) + mz * (q1xq3x + q0xq2x));
        hy = 2.0f * (mx * (q1xq2x + q0xq3x) + my * (0.5f - q1xq1x - q3xq3x) + mz * (q2xq3x - q0xq1x));
        // 假设飞机的x方向永远是北方，也就是说水平方向的数值通过计算膜值，全部集中到x轴上
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1xq3x - q0xq2x) + my * (q2xq3x + q0xq1x) + mz * (0.5f - q1xq1x - q2xq2x));

        // 估计重力方向和磁场方向
        // Estimated direction of gravity and magnetic field
        halfvx = q1xq3x - q0xq2x;
        halfvy = q0xq1x + q2xq3x;
        halfvz = q0xq0x - 0.5f + q3xq3x;
        halfwx = bx * (0.5f - q2xq2x - q3xq3x) + bz * (q1xq3x - q0xq2x);
        halfwy = bx * (q1xq2x - q0xq3x) + bz * (q0xq1x + q2xq3x);
        halfwz = bx * (q0xq2x + q1xq3x) + bz * (0.5f - q1xq1x - q2xq2x);

        // 误差 = [估计方位]x[测量方向]	【向量积运算】 两者越接近，这个数值越小(计算方法： value = |a| * |b| * sin<a^b>  )
        // Error is sum of cross product between estimated direction and measured direction of field vectors

#if 0
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
#else
        halfex = (ay * halfvz - az * halfvy) * 2.5;
        halfey = (az * halfvx - ax * halfvz) * 2.5;
        halfez = (mx * halfwy - my * halfwx) * 2.5;
#endif

        erx = halfex;
        ery = halfey;
        erz = halfez;

        // 根据误差，计算积分反馈
        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {        // twoKi 是积分系数
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);    // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);    // 积分误差通果Ki进行计量
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);    // 误差*积分系数*积分时间（采样时间）
            gx += integralFBx;    // apply integral feedback
            gy += integralFBy;    // gx,y,z 加入比例量
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;    // prevent integral windup
            integralFBy = 0.0f; // 积分系数为0，反馈自动归0
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        // gx,y,z 直接加入比例量
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    // 四元数的积分变化率
    gx *= (0.5f * (1.0f / sampleFreq));        // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));        // 先乘一个公共因数
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0x;                                // qa,qb,qc 均为临时变量，
    qb = q1x;                                // 防止q0x，1，2因为迭代计算冲掉上一次的迭代值
    qc = q2x;
    q0x += (-qb * gx - qc * gy - q3x * gz);    // q0x-q3x进行迭代
    q1x += (qa * gx + qc * gz - q3x * gy);
    q2x += (qa * gy - qb * gz + q3x * gx);
    q3x += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion					// 结果进行正则化
    recipNorm = invSqrt1(q0x * q0x + q1x * q1x + q2x * q2x + q3x * q3x);
    q0x *= recipNorm;
    q1x *= recipNorm;
    q2x *= recipNorm;
    q3x *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;    // 正则化的模
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 【加速度】传感器检测到数值才可继续
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        // 对加速度正则化
        recipNorm = invSqrt1(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 预估重力方向和磁通的正交矢量（理想情况下和重力方向平行，实际的姿态数值进行计算时存在误差）
        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1x * q3x - q0x * q2x;
        halfvy = q0x * q1x + q2x * q3x;
        halfvz = q0x * q0x - 0.5f + q3x * q3x;

        // 求取加速度和预估和测量的重力方向的误差（向量积计算）
        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 进行反馈
        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            // 积分部分
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);    // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;    // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        } else {
            // 如果没有积分常数，积分部分为0
            integralFBx = 0.0f;    // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 比例部分
        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));        // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0x;
    qb = q1x;
    qc = q2x;
    q0x += (-qb * gx - qc * gy - q3x * gz);
    q1x += (qa * gx + qc * gz - q3x * gy);
    q2x += (qa * gy - qb * gz + q3x * gx);
    q3x += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt1(q0x * q0x + q1x * q1x + q2x * q2x + q3x * q3x);
    q0x *= recipNorm;
    q1x *= recipNorm;
    q2x *= recipNorm;
    q3x *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root





//====================================================================================================
// END OF CODE
//====================================================================================================
