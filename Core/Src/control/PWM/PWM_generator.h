//
// Created by fanchunhui on 2022/11/22.
//

#ifndef QUA_CONTROL_PWM_GENERATOR_H
#define QUA_CONTROL_PWM_GENERATOR_H

#include "../../base_head.h"

void pwm_generate_ON();     // PWM启动
void pwm_generate_OFF(TIM_HandleTypeDef* tim1, TIM_HandleTypeDef* tim2);    // PWM关闭(直接熄火，慎用)
void pwm_change(uint8_t index, float value);   // 调整某个电机的PWM占空比
void pwm_change(float value[4], TIM_HandleTypeDef* tim1, TIM_HandleTypeDef* tim2); // 修改所有电机的占空比
void pwm_change(float pos1_v, float pos2_v, float pos3_v, float pos4_v); // 修改所有电机的占空比,直接给数值

#endif //QUA_CONTROL_PWM_GENERATOR_H
