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
    int W2; // pwm 生成器数值

    // PA2 -- CP -- CH3 -- TIM2
    W2 = (int)(value[0]*value[0]*((float)PWM_Period));
    __HAL_TIM_SetCompare(tim2, TIM_CHANNEL_3, W2);

    // PA3  -- CN -- CH4 -- TIM2
    W2 = (int)(value[0]*value[0]*((float)PWM_Period));
    __HAL_TIM_SetCompare(tim2, TIM_CHANNEL_4, W2);

    // PA15 -- AP -- CH1 -- TIM2
    W2 = (int)(value[1]*value[1]*((float)PWM_Period));
    __HAL_TIM_SetCompare(tim2, TIM_CHANNEL_1, W2);

    // PB3  -- AN -- CH2 -- TIM2
    W2 = (int)(value[1]*value[1]*((float)PWM_Period));
    __HAL_TIM_SetCompare(tim2, TIM_CHANNEL_2, W2);

    // PE9  -- BP -- CH1 -- TIM1
    W2 = (int)(value[2]*value[2]*((float)PWM_Period));
    __HAL_TIM_SetCompare(tim1, TIM_CHANNEL_1, W2);

    // PE11 -- BN -- CH2 -- TIM1
    W2 = (int)(value[2]*value[2]*((float)PWM_Period));
    __HAL_TIM_SetCompare(tim1, TIM_CHANNEL_2, W2);

    // PE13 -- DP -- CH3 -- TIM1
    W2 = (int)(value[3]*value[3]*((float)PWM_Period));
    __HAL_TIM_SetCompare(tim1, TIM_CHANNEL_3, W2);

    // PE14 -- DN -- CH4 -- TIM1
    W2 = (int)(value[3]*value[3]*((float)PWM_Period));
    __HAL_TIM_SetCompare(tim1, TIM_CHANNEL_4, W2);
}

void pwm_change(float pos1_v, float pos2_v, float pos3_v, float pos4_v) // 修改所有电机的占空比,直接给数值
{
    
}
