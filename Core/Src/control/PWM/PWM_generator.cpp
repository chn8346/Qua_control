//
// Created by fanchunhui on 2022/11/22.
//

#include "PWM_generator.h"

void pwm_generate_ON()     // PWM启动
{

}

void pwm_generate_OFF(TIM_HandleTypeDef* tim1, TIM_HandleTypeDef* tim2)    // PWM关闭(直接熄火，慎用)
{
    float change[4] = {0.0, 0.0, 0.0, 0.0};
    pwm_change(change, tim1, tim2);
}

void pwm_change(uint8_t index, float value)   // 调整某个电机的PWM占空比
{
    switch(index)
    {
        case 1:
            break;
        case 2:
            break;
        case 3:
            break;
        case 4:
            break;
        default:
            break;
    }
}

void pwm_change(float value[4], TIM_HandleTypeDef* tim1, TIM_HandleTypeDef* tim2) // 修改所有电机的占空比
{
    /**
     *  特别注意，pwm波形的范围由
     *  PWM_LOW_LIMIT_xxxx   占空比下限(低于这个电调丢失信号)
     *  PWM_START_xxxx       电机启动占空比(高于这个，电机启动)
     *  PWM_TOP_LIMIT_xxxx   电机满载占空比(高于这个，电调报错)
     *  PWM_RANGE_SIZE_xxxx  电调有效区间宽度(从上述参数计算得到，可减少计算量)
     *
     *  上述四个参数共同决定
     * */

    // 此处记录无人机信号分配
    // No.1 DP
    // No.2 DN
    // No.3 AN
    // No.4 AP

    int W2; // pwm 生成器数值

    // 调节value数值范围
    value[0] = PWM_LOW_LIMIT_XROTOR + value[0]*PWM_RANGE_SIZE_XROTOR;
    value[1] = PWM_LOW_LIMIT_XROTOR + value[1]*PWM_RANGE_SIZE_XROTOR;
    value[2] = PWM_LOW_LIMIT_XROTOR + value[2]*PWM_RANGE_SIZE_XROTOR;
    value[3] = PWM_LOW_LIMIT_XROTOR + value[3]*PWM_RANGE_SIZE_XROTOR;

    // CP/CN 通道暂时封闭
    // PA2 -- CP -- CH3 -- TIM2
    // value[0] = PWM_LOW_LIMIT_XROTOR + value[0]*PWM_RANGE_SIZE_XROTOR;
//    W2 = (int)(value[0]*((float)PWM_Period));
//    __HAL_TIM_SetCompare(tim2, TIM_CHANNEL_3, W2);
//
//    // PA3  -- CN -- CH4 -- TIM2
//    W2 = (int)(value[0]*((float)PWM_Period));
//    __HAL_TIM_SetCompare(tim2, TIM_CHANNEL_4, W2);

    // PA15 -- AP -- TIM2-CH1   4号电机
    // value[1] = PWM_LOW_LIMIT_XROTOR + value[1]*PWM_RANGE_SIZE_XROTOR;
    W2 = (int)(value[0]*((float)PWM_Period));
    __HAL_TIM_SetCompare(tim2, TIM_CHANNEL_1, W2);

    // PB3  -- AN -- TIM2-CH2   3号电机
    W2 = (int)(value[1]*((float)PWM_Period));
    __HAL_TIM_SetCompare(tim2, TIM_CHANNEL_2, W2);

    // BP/BN 通道暂时封闭
    // PE9  -- BP -- CH1 -- TIM1
    // value[2] = PWM_LOW_LIMIT_XROTOR + value[2]*PWM_RANGE_SIZE_XROTOR;
//    W2 = (int)(value[2]*((float)PWM_Period));
//    __HAL_TIM_SetCompare(tim1, TIM_CHANNEL_1, W2);
//
//    // PE11 -- BN -- CH2 -- TIM1
//    W2 = (int)(value[2]*((float)PWM_Period));
//    __HAL_TIM_SetCompare(tim1, TIM_CHANNEL_2, W2);

    // PE13 -- DP -- TIM1-CH3 -- 1号电机
    // value[3] = PWM_LOW_LIMIT_XROTOR + value[3]*PWM_RANGE_SIZE_XROTOR;
    W2 = (int)(value[2]*((float)PWM_Period));
    __HAL_TIM_SetCompare(tim1, TIM_CHANNEL_3, W2);

    // PE14 -- DN -- TIM1-CH4 -- 2号电机
    W2 = (int)(value[3]*((float)PWM_Period));
    __HAL_TIM_SetCompare(tim1, TIM_CHANNEL_4, W2);
}

void pwm_change(float pos1_v, float pos2_v, float pos3_v, float pos4_v) // 修改所有电机的占空比,直接给数值
{
    
}
