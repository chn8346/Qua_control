//
// Created by fanchunhui on 2022/11/9.

// used for base app, main don't include this

#ifndef QUA_CONTROL_BASE_HEAD_H
#define QUA_CONTROL_BASE_HEAD_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"

// 状态和目标状态的数据结构
typedef struct{
    float time_; // 轨迹规划预留
    float q[4];
    float angle[3];
    float height;
    float pos[2];
    float speed[3];
    float acc[3];
    float angle_rate[3];
    float angle_acc[3];
}qua_state;


/*
 *  TODO 系统参数  移植时【必须】重新设置
 * */

#define SAMPLE_RATE       200         // 采样速度
#define DELTA_T           0.005       // 采样时间
#define DELTA_T_float     ((float)0.005)       // 采样时间(float)

#define Pi          3.1415926

// 飞行器参数
#define Qua_MASS                2.0     // 飞行器质量
#define Qua_Kf                  0.0001   // 飞行器拉力系数
#define Qua_Km                  0.0001   // 飞行器力矩系数
#define Qua_TAKE_OFF_P          0.45    // 飞行器起飞油门
#define Qua_DIAMETER            0.3     // 飞行器桨距（电机和物理中心点距离）

// 控制器类型
#define CONTROLLER_TYPE_PID     0x0F
#define CONTROLLER_TYPE_ADRC    0xAF

// LIMIT 飞行器控制限幅
#define PITCH_LIMIT_DEGREE         10      // pitch角度限幅 -- 角度制
#define ROLL_LIMIT_DEGREE          10      // roll角度限幅  -- 角度制
// #define YAW_LIMIT_DEGREE        10      // yaw角度限幅   -- 角度制

#define PITCH_LIMIT_RAD   0.174  // 小于10度
#define ROLL_LIMIT_RAD    0.174  // 小于10度

// PWM
#define PWM_FREQ                50                // PWM输出频率(Hz)
#define PWM_RESOLVE             1000              // PWM分辨率
#define PWM_Period              10000             // 周期

// PPM
// #define PPM_nUS                 10      // PPM采样时间(us)
#define PPM_data_len_limit         10      // PPM数据长度

// 电调的相关参数
// 1. X-rotor 系列电调参数
#define PWM_LOW_LIMIT_XROTOR        0.08    // 占空比下限
#define PWM_START_XROTOR            0.1     // 电机启动占空比
#define PWM_TOP_LIMIT_XROTOR        0.22    // 电机满载占空比
#define PWM_RANGE_SIZE_XROTOR       0.12    // 电调有效区间宽度(从上述参数计算得到，可减少计算量)

// 构型设置 + x H
#define STRUCT_TYPE_plus            0x01    // ‘+’ 构型 -- 常规构型
#define STRUCT_TYPE_x               0x02    // ‘X’ 构型 -- 常规构型
#define STRUCT_TYPE_H               0x03    // ‘H’ 构型 -- 穿越机构型
#define STRUCT_TYPE_H_T             0x04    // ‘工’ 构型 -- 穿越机构型

#define STRUCT_TYPE                 STRUCT_TYPE_H

// 构型对应的电机信息
#define Motor_1_CW

/*
 *      PPM 通道
 *
 *      注意通道从0开始算，例如：通道1，数值为0
 * */


#define PPM_CH_FORCE    2       // 注意通道从0开始算，例如：通道1，数值为0
#define PPM_CH_YAW      3       // 注意通道从0开始算，例如：通道1，数值为0
#define PPM_CH_PITCH    1       // 注意通道从0开始算，例如：通道1，数值为0
#define PPM_CH_ROLL     0       // 注意通道从0开始算，例如：通道1，数值为0


/*
 *
 *  TODO parameter set
 *
 * */

/*
 * TODO ADC
 *
 * */

#define ADC_PORT        GPIOB
#define ADC_PIN         GPIO_PIN_1


/*
 * TODO I2C1 and I2C2
 *
 * */

#define I2C1_PORT       GPIOB
#define I2C1_SCL_PORT   GPIOB
#define I2C1_SCL_PIN    GPIO_PIN_6
#define I2C1_SDA_PORT   GPIOB
#define I2C1_SDA_PIN    GPIO_PIN_7


#define I2C2_PORT       GPIOB
#define I2C2_SCL_PORT   GPIOB
#define I2C2_SCL_PIN    GPIO_PIN_10
#define I2C2_SDA_PORT   GPIOB
#define I2C2_SDA_PIN    GPIO_PIN_11


/*
 *  TODO SD
 *
 * */

#define SD_D0_PORT      GPIOC
#define SD_D0_PIN       GPIO_PIN_8

#define SD_CK_PORT      GPIOC
#define SD_CK_PIN       GPIO_PIN_12

#define SD_CMD_PORT     GPIOD
#define SD_CMD_PIN      GPIO_PIN_2


/*
 * TODO SPI1 and SPI2
 *
 * */

#define SPI1_PORT        GPIOA

#define SPI1_SCK_PORT    GPIOA
#define SPI1_SCK_PIN     GPIO_PIN_5
#define SPI1_MISO_PORT   GPIOA
#define SPI1_MISO_PIN    GPIO_PIN_6
#define SPI1_MOSI_PORT   GPIOA
#define SPI1_MOSI_PIN    GPIO_PIN_7
#define SPI1_CS1_PORT    GPIOD
#define SPI1_CS1_PIN     GPIO_PIN_7
#define SPI1_CS2_PORT    GPIOD
#define SPI1_CS2_PIN     GPIO_PIN_10
#define SPI1_CS3_PORT    GPIOD
#define SPI1_CS3_PIN     GPIO_PIN_11

#define SPI2_SCK_PORT    GPIOB
#define SPI2_SCK_PIN     GPIO_PIN_13
#define SPI2_MISO_PORT   GPIOC
#define SPI2_MISO_PIN    GPIO_PIN_2
#define SPI2_MOSI_PORT   GPIOC
#define SPI2_MOSI_PIN    GPIO_PIN_3
#define SPI2_CS1_PORT    GPIOD
#define SPI2_CS1_PIN     GPIO_PIN_12
#define SPI2_CS2_PORT    GPIOD
#define SPI2_CS2_PIN     GPIO_PIN_13
#define SPI2_CS3_PORT    GPIOD
#define SPI2_CS3_PIN     GPIO_PIN_14


/*
 * TODO TIM 1
 *
 * */

// TIM1
#define TIM1_PORT           GPIOE

#define TIM1_CH1_PORT       GPIOE
#define TIM1_CH1_PIN        GPIO_PIN_9
#define PWM_BP_PORT         TIM1_CH1_PORT
#define PWM_BP_PIN          TIM1_CH1_PIN

#define TIM1_CH2_PORT       GPIOE
#define TIM1_CH2_PIN        GPIO_PIN_11
#define PWM_BN_PORT         TIM1_CH2_PORT
#define PWM_BN_PIN          TIM1_CH2_PIN

#define TIM1_CH3_PORT       GPIOE
#define TIM1_CH3_PIN        GPIO_PIN_13
#define PWM_DP_PORT         TIM1_CH3_PORT
#define PWM_DP_PIN          TIM1_CH3_PIN

#define TIM1_CH4_PORT       GPIOE
#define TIM1_CH4_PIN        GPIO_PIN_14
#define PWM_DN_PORT         TIM1_CH4_PORT
#define PWM_DN_PIN          TIM1_CH4_PIN


/*
 * TODO TIM 2
 *
 * */

// TIM2
#define TIM2_PORT           GPIOA

#define TIM2_CH1_PORT       GPIOA
#define TIM2_CH1_PIN        GPIO_PIN_15
#define PWM_AP_REAL_PORT    TIM2_CH1_PORT
#define PWM_AP_PIN          TIM2_CH1_PIN

#define TIM2_CH2_PORT       GPIOB
#define TIM2_CH2_PIN        GPIO_PIN_3
#define PWM_AN_PORT         TIM2_CH2_PORT
#define PWM_AN_PIN          TIM2_CH2_PIN

#define TIM2_CH3_PORT       GPIOA
#define TIM2_CH3_PIN        GPIO_PIN_2
#define PWM_CP_PORT         TIM2_CH3_PORT
#define PWM_CP_PIN          TIM2_CH3_PIN

#define TIM2_CH4_PORT       GPIOA
#define TIM2_CH4_PIN        GPIO_PIN_3
#define PWM_CN_PORT         TIM2_CH4_PORT
#define PWM_CN_PIN          TIM2_CH4_PIN


/*
 * TODO UART
 *
 * */

// U1
#define UART1_PORT      GPIOA
#define UART1_TX_PORT   GPIOA
#define UART1_TX_PIN    GPIO_PIN_9
#define UART1_RX_PORT   GPIOA
#define UART1_RX_PIN    GPIO_PIN_10
#define RTS_GPIO_PORT   UART1_TX_PORT
#define RTS_PIN         UART1_TX_PIN

// U2
#define UART2_PORT      GPIOD
#define UART2_TX_PORT   GPIOD
#define UART2_TX_PIN    GPIO_PIN_5
#define UART2_RX_PORT   GPIOD
#define UART2_RX_PIN    GPIO_PIN_6

// U3
#define UART3_PORT      GPIOD
#define UART3_TX_PORT   GPIOD
#define UART3_TX_PIN    GPIO_PIN_8
#define UART3_RX_PORT   GPIOD
#define UART3_RX_PIN    GPIO_PIN_9

// U4
#define UART4_PORT      GPIOA
#define UART4_TX_PORT   GPIOA
#define UART4_TX_PIN    GPIO_PIN_0
#define UART4_RX_PORT   GPIOA
#define UART4_RX_PIN    GPIO_PIN_1

// U6
#define UART6_PORT      GPIOC
#define UART6_TX_PORT   GPIOC
#define UART6_TX_PIN    GPIO_PIN_6
#define UART6_RX_PORT   GPIOC
#define UART6_RX_PIN    GPIO_PIN_7


/*
 * TODO ALTER
 *
 * */

#define OUT_NUM 0
#define IN_NUM  0

#define LED1_PORT GPIOC
#define LED1_POS  GPIO_PIN_4

#define LED2_PORT GPIOC
#define LED2_POS  GPIO_PIN_5

#define TF_DET_LED_PORT     GPIOB
#define TF_DET_LED_PIN      GPIO_PIN_9

/*
 *      自定义接口参数
 * */

#define PPM_RX_PORT         GPIOE
#define PPM_RX_PIN          GPIO_PIN_15

#define ADC_BATTERY_PORT    GPIOB
#define ADC_BATTERY_PIN     GPIO_PIN_1

/*
 *     PWM 通道
 *
 * */

#define PWM_AP_TIM_NO_  2
#define PWM_AN_TIM_NO_  2
#define PWM_BP_TIM_NO_  1
#define PWM_BN_TIM_NO_  1
#define PWM_CP_TIM_NO_  2
#define PWM_CN_TIM_NO_  2
#define PWM_DP_TIM_NO_  1
#define PWM_DN_TIM_NO_  1

#define PWM_AP_TIM_CHANNEL  TIM_CHANNEL_1
#define PWM_AN_TIM_CHANNEL  TIM_CHANNEL_2
#define PWM_BP_TIM_CHANNEL  TIM_CHANNEL_1
#define PWM_BN_TIM_CHANNEL  TIM_CHANNEL_2
#define PWM_CP_TIM_CHANNEL  TIM_CHANNEL_3
#define PWM_CN_TIM_CHANNEL  TIM_CHANNEL_4
#define PWM_DP_TIM_CHANNEL  TIM_CHANNEL_3
#define PWM_DN_TIM_CHANNEL  TIM_CHANNEL_4


#endif //QUA_CONTROL_BASE_HEAD_H
