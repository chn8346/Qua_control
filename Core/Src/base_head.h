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
 *  TODO 系统参数
 * */

#define SAMPLE_RATE 200         // 采样速度
#define DELTA_T     0.005       // 采样时间


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
 * TODO SENSOR DATA
 *
 * */

// IMU choose  -- bottom have higher right
#define MPU6050_ENABLE  1
#define ICM20602_ENABLE 0

// MAG choose  -- bottom have higher right
#define MAG3110_ENABLE 1
#define IST8310_ENABLE 0

// PRESSURE choose
#define SPL06_ENABLE 1

// GPS choose  -- bottom have higher right
#define BN880_ENABLE 1

// IMU
#if MPU6050_ENABLE
#define IMU_ADDR 0x87

#define IMU_WHO_AM_I_ADDR 0x75
#define IMU_WHO_AM_I_KEY  0x75

#define IMU_ACC_XL 0x45
#define IMU_ACC_XH 0x45
#define IMU_ACC_YL 0x45
#define IMU_ACC_YH 0x45
#define IMU_ACC_ZL 0x45
#define IMU_ACC_ZH 0x45
#define IMU_G_XL 0x45
#define IMU_G_XH 0x45
#define IMU_G_YL 0x45
#define IMU_G_YH 0x45
#define IMU_G_ZL 0x45
#define IMU_G_ZH 0x45
#endif

// IMU
#if ICM20602_ENABLE
#define IMU_ADDR 0x87

#define IMU_WHO_AM_I_ADDR 0x75
#define IMU_WHO_AM_I_KEY  0x75

#define IMU_ACC_XL 0x45
#define IMU_ACC_XH 0x45
#define IMU_ACC_YL 0x45
#define IMU_ACC_YH 0x45
#define IMU_ACC_ZL 0x45
#define IMU_ACC_ZH 0x45
#define IMU_G_XL 0x45
#define IMU_G_XH 0x45
#define IMU_G_YL 0x45
#define IMU_G_YH 0x45
#define IMU_G_ZL 0x45
#define IMU_G_ZH 0x45
#endif

// MAG
#if MAG3110_ENABLE
#define MAG_ADDR 0x87

#define MAG_WHO_AM_I_ADDR 0x75
#define MAG_WHO_AM_I_KEY  0x75

#define MAG_XL 0x45
#define MAG_XH 0x45
#define MAG_YL 0x45
#define MAG_YH 0x45
#define MAG_ZL 0x45
#define MAG_ZH 0x45
#endif

// MAG
#if IST8310_ENABLE
#define MAG_ADDR 0x87

#define MAG_WHO_AM_I_ADDR 0x75
#define MAG_WHO_AM_I_KEY  0x75

#define MAG_XL 0x45
#define MAG_XH 0x45
#define MAG_YL 0x45
#define MAG_YH 0x45
#define MAG_ZL 0x45
#define MAG_ZH 0x45
#endif

// PRESSURE
#if SPL06_ENABLE

#endif

// GPS
#if BN880_ENABLE

#endif


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

#endif //QUA_CONTROL_BASE_HEAD_H
