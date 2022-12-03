/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "head.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
volatile UART_HandleTypeDef *uart_pc;
volatile TIM_HandleTypeDef * htim1_it;
volatile UART_HandleTypeDef * uart_gps;
volatile uint8_t u1rx_buff[32];
volatile uint8_t u2rx_buff[32];
volatile uint8_t u3rx_buff[32];
volatile int     send_sgn = 0;

// 电压测量
volatile uint32_t battery_raw_value;
uint32_t times = 0;

// PPM
volatile uint8_t PPM_data_index = 0;    // 目前的ppm长度
uint32_t PPM_data[PPM_data_len_limit+1] ={5000, 5000, 5000, 5000, 5000, 5000,
                     5000, 5000, 5000, 5000, 5000}; // PPM数据
float PPM_precent_float[PPM_data_len_limit+1] = {0.0};      // PPM解析的百分比

// ultra sonic
volatile uint8_t sonic_data_length = 0;  // 长度为2时输出超声波数据
volatile uint16_t sonic_data16;
float distance_ultra_sonic = 0;

extern float q0x,q1x,q2x,q3x;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM6_Init(void); // sample
static void MX_TIM7_Init(void); // PPM
/* USER CODE BEGIN PFP */

void uart1_transmit(uint8_t* data, uint8_t len);
void uart1_transmit_with_next(uint8_t* data, uint8_t len);
void print(const char* data, uint8_t len);
void uart2_transmit(uint8_t* data, uint8_t len);
void uart2_transmit_with_next(uint8_t* data, uint8_t len);
void uart6_transmit(uint8_t* data, uint8_t len);
void uart6_transmit_with_next(uint8_t* data, uint8_t len);

// printf redirect finished.
uint8_t str_len_(const char * x){int i;while(x[i++]);return i;}

static void IT_Init(void);          // 中断启动
static void init_hard_ware(void);  // PWM生成启动


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  //MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();   // PWM
  MX_TIM2_Init();   // PWM
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();       // 采样
  MX_TIM7_Init();       // PPM计时器，
  /* USER CODE BEGIN 2 */



    /* 1. 硬件资源初始化 */
    init_hard_ware();               // 外设初始化
    IT_Init();                      // 中断设置生效
    init_uart_to_pc(&huart1);// 启动 PC端口的 uart



    /* 2. 通信功能启动 */
    // UART中断开启
    HAL_UART_Receive_IT(&huart1,(uint8_t*)u1rx_buff,1); // 开 uart1 中断 (PC 通信)
    HAL_UART_Receive_IT(&huart2,(uint8_t*)u2rx_buff,1); // 开 uart2 中断 (GPS)
    HAL_UART_Receive_IT(&huart3,(uint8_t*)u3rx_buff,1); // 开 uart3 中断 (超声波)
    LED1_OFF;    // 关闭 LED1
    LED2_OFF;    // 关闭 LED2



    /* 3. 估计器功能启动 */
    // 传感器初始化
    //MPU6050_init(&hi2c2);
    IMU_init(&hi2c2);
    MAG3110_init(&hi2c1);
    MAG_init(&hi2c1);
    // HMC5883_init(&hi2c1);
    // BMP180_init(&hi2c1);

    // GPS
    PSR_init(&hi2c1);
    gps_info main_gps_info;

    // 传感器校正
    calibrate_get_i2c_spi(&hi2c1, &hi2c2, nullptr, nullptr);
    // 校准选项
    // 1: 现场进行校准    0: 使用已有数据
#if 0
    calibrate_at_init(&huart1);       // 现场校准
#else
    calibrate_at_init();              // 使用经验数据
#endif

    // I2C设备扫描器 par1:I2C handle对象 par2:寄存器地址，一般是ID  par:期望寄存器数值，一般对应ID的KEY
    // i2c_scan_print(&hi2c1, 0x00, 0x68);

    int cnt_limit = 5;  //  采样次数达到这个数值之后通过串口回传一次状态数据
    int cnt = 0;        //  上面那个采样次数的计数值

    float rd = 180.0/3.1415926; // 1rad 代表的角度

    float quat[4] = {0,0,0,0}; // 从估计器导出的姿态值
    float pitch = 0;                          // pitch角
    float roll = 0;                           // roll角
    float yaw = 0;                            // yaw角



    /* 4. 控制器功能启动 */

    // 控制器声明
    pid_API pid_controller;

    // PID参数
    float pid_parameter[18] = {0.1, 0, 1,
                               0.1, 0, 1,
                               0.1, 0, 1,
                               0.1, 0, 1,
                               0.1, 0, 1,
                               0.1, 0, 1};

    // 这个给到每次的PID更新
    float estimate_data[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; // 状态容器，测量的姿态和位置数据从这里导入到PID_API

    // 这个目标值给到PID控制器
    float target_data[6]   = {0.0,0.0,0.0,0.0,0.0,0.0}; // 目标放置处

    // pid控制器初始化，导入pid参数和目标
    pid_controller.init_PID_API(pid_parameter,
                                pid_parameter+3,
                                pid_parameter+6,
                                pid_parameter+9,
                                pid_parameter+12,
                                pid_parameter+15,
                                target_data);

    // 这个是从更新里面取数据的变量
    float Qua_pwm_rate[4] = {0,0,0,0};                        // 占空比(也就是油门),PID控制器给出
    float Force_and_Moment[6] = {0,0,0,0,0,0};        // 力和力矩的控制输出，PID控制器给出

    // 变量
    int ss = send_sgn;
    uint8_t data[12];   // IMU
    float datap[6];     // IMU

    uint8_t mag_data[6];    // MAG
    uint8_t spl_data[10];   // SPL06
    float mag_fdata[3];     // MAG
    float mag_data_f[3] = {0,0,0};

    // 重置遥控器数据
    for(int i = 1; i < 10; i++)PPM_data[i] = 5000;

    // 这里等待遥控器连上飞控,如果飞控连接成功，在上面重置之后，这里会很快重新写上数值
    while(PPM_data[0] == 5000 ||
            PPM_data[1] == 5000 ||
            PPM_data[2] == 5000 ||
            PPM_data[3] == 5000 ||
            PPM_data[4] == 5000 ||
            PPM_data[5] == 5000 ||
            PPM_data[6] == 5000 ||
            PPM_data[7] == 5000 ||
            PPM_data[8] == 5000 ||
            PPM_data[9] == 5000);

    // 遥控器完成连接之后，延迟1秒后测量最大值
    uint16_t dy = 100; // 延时1秒
    while(dy){
        if(send_sgn != 0){
            dy = dy - 1;
            send_sgn = 0;
        }
    }

    // 这时需要遥控器内八，这样才能继续，因为这样才能测量到实际的信息头部位置
    // 注意，内部有检测的死循环，达到要求才会继续
    REMOTE_CONTROL_get_zero_index(PPM_data, 11);


    /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
      // 每当send_sgn发出，说明需要一次更新
      if(send_sgn != 0) {
          ss = send_sgn;

          send_sgn = 0;
          cnt++;

          /* 传感器读取 */

//        imu6050_read(&hi2c2, data, 1);
//        imu_process(data, datap);
          IMU_read_data(data, datap);

  //      float mag_fdata_[3];
 //       uint8_t mag_data_[6];
//        MAG3110_read(&hi2c1, mag_data_);
//        MAG3110_process3(mag_data_, mag_fdata_);
          MAG_read_data(mag_data, mag_fdata);

//        HMC5883_read_and_process(mag_data_f);
//        uint8_t hmc_data[6];
//        HMC5883_read(hmc_data);

          //float bmp_data;
          //BMP180_get_pressure(&bmp_data);

          // 超声波
          sonic_requist_distance(&huart3);

          // 电压测量 ADC 测量信号
          // hadc1.Instance->CR2 |= 0x4000;

          // 传感器校准
          calibrate_return_data(datap, datap+3, mag_fdata,0,0,0,0);

          /* 状态估计 */
          MahonyAHRSupdate(datap[3],datap[4],datap[5],
                           datap[0],datap[1],datap[2],
                           -mag_fdata[0],-mag_fdata[1],mag_fdata[2]);

          quat[0] = q0x; quat[1] = q1x; quat[2] = q2x; quat[3] = q3x;
          pitch = asin(2*(q0x*q2x-q1x*q3x));
          roll = atan(2*(q0x*q1x+q2x*q3x)/(1-2*(q1x*q1x+q2x*q2x)));
          // float yaw = atan(2*(q0x*q3x+q1x*q2x)/(1-2*(q2x*q2x+q3x*q3x)))*rd;
          yaw = atan2(2*(q0x*q3x+q1x*q2x),(1-2*(q2x*q2x+q3x*q3x)));

          /* 控制器更新 */
          // 填入estimate数值
          estimate_data[PID_API_INDEX_PITCH] = pitch;   // 注意！！！！ 控制用的是弧度
          estimate_data[PID_API_INDEX_ROLL] = roll;
          estimate_data[PID_API_INDEX_YAW] = yaw;
          pid_controller.INPUT_estimate(estimate_data);

          // 根据遥控器进行目标生成
          remote_control_phase(PPM_data, PPM_precent_float);
          PID_API_remote_control_gen_target(PPM_precent_float, target_data);
          // 填入预期
          pid_controller.INPUT_target(target_data);


          // 姿态控制器
          pid_controller.PROCESS_update();
          pid_controller.get_OUTPUT(Qua_pwm_rate, Force_and_Moment);

          /* 输出 */
          // 控制需求给到 pwm生成器，修改占空比
          pwm_change(Qua_pwm_rate, &htim1, &htim2);

          /* 串口通信 */
          if(cnt >= cnt_limit)
          {
              cnt = 0;
              LED1_TOGGLE;

              char msg[150] = {0};
#if 0       // 姿态测试
              sprintf(msg, "pitch=%f,roll=%f,yaw=%f,bt=%lu",pitch*rd, roll*rd, yaw*rd, battery_raw_value);
#elif 0     // 遥控器行程测试
              sprintf(msg, "force=%.2f,pitch=%.2f,roll=%.2f,yaw=%.2f",
                                            PPM_precent_float[PPM_CH_FORCE],
                                            PPM_precent_float[PPM_CH_PITCH],
                                            PPM_precent_float[PPM_CH_ROLL],
                                            PPM_precent_float[PPM_CH_YAW]);
#elif 0     // 遥控器生成的目标测试
              sprintf(msg, "target_z=%.2f,target_pitch=%.2f,target_roll=%.2f,target_yaw=%.2f",
                                                              target_data[PID_API_INDEX_POSZ],
                                                              target_data[PID_API_INDEX_PITCH]*rd,
                                                              target_data[PID_API_INDEX_ROLL]*rd,
                                                              target_data[PID_API_INDEX_YAW]*rd);
#elif 1     // 油门显示
              sprintf(msg, "Q1=%.5f,Q2=%.5f,Q3=%.5f,Q4=%.5f,target_z=%.2f,target_pitch=%.2f,target_roll=%.2f,target_yaw=%.2f",
                                                  Qua_pwm_rate[0]*Qua_pwm_rate[0],
                                                  Qua_pwm_rate[1]*Qua_pwm_rate[1],
                                                  Qua_pwm_rate[2]*Qua_pwm_rate[2],
                                                  Qua_pwm_rate[3]*Qua_pwm_rate[3],
                                                  target_data[PID_API_INDEX_POSZ],
                                                  target_data[PID_API_INDEX_PITCH]*rd,
                                                  target_data[PID_API_INDEX_ROLL]*rd,
                                                  target_data[PID_API_INDEX_YAW]*rd);
#elif 0     // 飞控的力和力矩显示
              sprintf(msg, "F=%.5f,Pitch=%.5f,Roll=%.5f,Yaw=%.5f,target_z=%.2f,target_pitch=%.2f,target_roll=%.2f,target_yaw=%.2f",
                                                  Force_and_Moment[PID_API_INDEX_POSZ],
                                                  Force_and_Moment[PID_API_INDEX_PITCH],
                                                  Force_and_Moment[PID_API_INDEX_ROLL],
                                                  Force_and_Moment[PID_API_INDEX_YAW],
                                                  target_data[PID_API_INDEX_POSZ],
                                                  target_data[PID_API_INDEX_PITCH]*rd,
                                                  target_data[PID_API_INDEX_ROLL]*rd,
                                                  target_data[PID_API_INDEX_YAW]*rd);
#elif 0     // 遥控器通道测试
              sprintf(msg, "PPM,CH1=%lu,CH2=%lu,CH3=%lu,CH4=%lu,CH5=%lu,CH6=%lu,CH7=%lu,CH8=%lu,CH9=%lu,CH10=%lu,SUM=%lu,",
                                                                  PPM_data[1],
                                                                  PPM_data[2],
                                                                  PPM_data[3],
                                                                  PPM_data[4],
                                                                  PPM_data[5],
                                                                  PPM_data[6],
                                                                  PPM_data[7],
                                                                  PPM_data[8],
                                                                  PPM_data[9],
                                                                  PPM_data[10],
              PPM_data[1]+PPM_data[2]+PPM_data[3]+PPM_data[4]+PPM_data[5]+PPM_data[6]+PPM_data[7]+PPM_data[8]+PPM_data[9]);
#elif 0     // 传感器测试
              sprintf(msg, "ax=%.2f,ay=%.2f,az=%.2f,gx=%.2f,gy=%.2f,gz=%.2f,mx=%.2f,my=%.2f,mz=%.2f\n",
                                                                                                    datap[0],
                                                                                                    datap[1],
                                                                                                    datap[2],
                                                                                                    datap[3],
                                                                                                    datap[4],
                                                                                                    datap[5],
                                                                                                    mag_fdata[0],
                                                                                                    mag_fdata[2],
                                                                                                    mag_fdata[1]);
#elif 0     // 磁力计单独测试
              sprintf(msg, "mx=%.2f,my=%.2f,mz=%.2f\n",mag_fdata[0],
                      mag_fdata[2],
                      mag_fdata[1]);

#elif 0     // 四元数和角度的姿态估计测试
                sprintf(msg, "Q1=%.4f, Q2=%.4f, Q3=%.4f, Q4=%.4f, pitch=%.4f, roll=%.4f, yaw=%.4f\n",
                                                                                    Qua_pwm_rate[0],
                                                                                    Qua_pwm_rate[1],
                                                                                    Qua_pwm_rate[2],
                                                                                    Qua_pwm_rate[3],
                                                                                    pitch*rd,
                                                                                    roll*rd,
                                                                                    yaw*rd);
#endif
              uart1_transmit_with_next((uint8_t *) msg, str_len_(msg));

          }
      }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  __ADC1_CLK_ENABLE();
  __ADC2_CLK_ENABLE();
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode=ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN ADC1_Init 2 */

    HAL_ADC_Start_IT(&hadc1);

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
    GPIO_InitTypeDef gi;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    gi.Pin = GPIO_PIN_6;
    gi.Mode = GPIO_MODE_AF_OD;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_HIGH;
    gi.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gi);

    gi.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &gi);
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
    __HAL_RCC_I2C1_FORCE_RESET();
            __I2C1_RELEASE_RESET();
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */
    GPIO_InitTypeDef gi;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();

    gi.Pin = GPIO_PIN_11;
    gi.Mode = GPIO_MODE_AF_OD;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_HIGH;
    gi.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &gi);

    gi.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB, &gi);
  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */
    __HAL_RCC_I2C2_FORCE_RESET();
            __I2C2_RELEASE_RESET();
  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1; // 分频到1M
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PWM_Period-1;  // 50HZ为20ms，即一个周期为20000
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 10;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
            __TIM2_CLK_ENABLE();
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = PWM_Period-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_ALL);

//            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 5000);
//            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 5000);
//            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 5000);
//            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 5000);

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1599;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100;      // 10000 = 1s   100 = 10ms
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 16;     // 1000khz -- 1us
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 5000;      // 3000us3ms  50hz的输入为3ms  多1ms是为了方便直接接收下一帧
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */
    HAL_TIM_Base_Stop(&htim7);      // 当接受信号中断之后，tim7才开始计时
    __HAL_TIM_SET_COUNTER(&htim7, 0);
  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SPI1_CS2_Pin|SPI1_CS3_Pin|SPI2_CS1_Pin|GPIO_PIN_13
                          |SPI2_CS3_Pin|SPI1_CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 IN4_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS2_Pin SPI1_CS3_Pin SPI2_CS1_Pin PD13
                           SPI2_CS3_Pin SPI1_CS1_Pin */
  GPIO_InitStruct.Pin = SPI1_CS2_Pin|SPI1_CS3_Pin|SPI2_CS1_Pin|GPIO_PIN_13
                          |SPI2_CS3_Pin|SPI1_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


    // PPM
    GPIO_InitStruct.Pin = PPM_RX_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(PPM_RX_PORT, &GPIO_InitStruct);

    // ADC
    GPIO_InitStruct.Pin = ADC_BATTERY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_BATTERY_PORT, &GPIO_InitStruct);

    // PWM
  // tim2_ch 3,4,1
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // tim2_ch 2
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // tim2_ch 1,2,3,4
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

static void IT_Init(void)
{
    uart_pc = &huart1;
    htim1_it = &htim1;
    uart_gps = &huart2;

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_RCC_TIM7_CLK_ENABLE();

    NVIC_SetPriorityGrouping(2);


    // 采样中断
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 2);
    HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    __HAL_TIM_ENABLE_IT(&htim6, TIM6_DAC_IRQn);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_TIM_Base_Start_IT(&htim6);

    // PPM-PIN中断
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    //HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // ADC中断
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);


//    // PPM接收时钟中断 -- 思路错了，应该是设置pin的 EXIT
//    HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
//    HAL_NVIC_ClearPendingIRQ(TIM7_IRQn);
//    __HAL_TIM_ENABLE_IT(&htim7, TIM7_IRQn);
//    HAL_NVIC_EnableIRQ(TIM7_IRQn);
//    HAL_TIM_Base_Start_IT(&htim7);

//    // PWM A C 通道中断
//    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 1);
//    HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
//    __HAL_TIM_ENABLE_IT(&htim1, TIM1_UP_TIM10_IRQn);
//    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
//    HAL_TIM_Base_Start_IT(&htim1);
//
//    // uart中断，不需要设置，中断也可以使用
//    HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);
//    HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
//    HAL_NVIC_EnableIRQ(USART1_IRQn);
//    __HAL_UART_ENABLE_IT(&huart1, USART1_IRQn);
//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
//    HAL_UART_Receive_IT(&huart1,(uint8_t*)u1rx_buff,1);
//
//    HAL_NVIC_SetPriority(USART2_IRQn, 1, 2);
//    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
//    HAL_NVIC_EnableIRQ(USART2_IRQn);
//    __HAL_UART_ENABLE_IT(&huart2, USART2_IRQn);
//    __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
//    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
//    HAL_UART_Receive_IT(&huart2,(uint8_t*)u2rx_buff,1);
//

}

static void init_hard_ware(void)  // 硬件启动
{

    // 电机启动
    pwm_change(0, 0, 0, 0); // 由于PWM有最低转速要求，实际0会被置为最低转速
    pwm_generate_ON();
}

void uart1_transmit(uint8_t* data, uint8_t len)
{
    for(int i = 0; i < len; i++)
    {
        data[i] = 0x80 | data[i];
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 500);
}
void uart1_transmit_with_next(uint8_t* data, uint8_t len)
{
    for(int i = 0; i < len-1; i++)
    {
        data[i] = 0x80 | data[i];
    }
    data[len] = 0xff;
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 500);
}
void print(const char* data, uint8_t len){ uart1_transmit((uint8_t*)data, len);}
void uart2_transmit(uint8_t* data, uint8_t len)
{
    for(int i = 0; i < len; i++)
    {
        data[i] = 0x80 | data[i];
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 500);
}
void uart2_transmit_with_next(uint8_t* data, uint8_t len)
{
    for(int i = 0; i < len-1; i++)
    {
        data[i] = 0x80 | data[i];
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 500);
}
void uart6_transmit(uint8_t* data, uint8_t len)
{
    for(int i = 0; i < len; i++)
    {
        data[i] = 0x80 | data[i];
    }

    HAL_UART_Transmit(&huart6, (uint8_t*)data, len, 500);
}
void uart6_transmit_with_next(uint8_t* data, uint8_t len)
{
    for(int i = 0; i < len-1; i++)
    {
        data[i] = 0x80 | data[i];
    }

    HAL_UART_Transmit(&huart6, (uint8_t*)data, len, 500);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2) {
        //HAL_UART_Transmit(&huart1,(uint8_t*)u2rx_buff,1,100);
        // uart1_transmit((uint8_t*)u2rx_buff, 1);

        LED2_TOGGLE
    }
    else if(huart -> Instance == USART1){
        uart1_transmit((uint8_t*)u2rx_buff, 1);
    }
    else if(huart -> Instance == USART3){
        sonic_data_length = sonic_data_length + 1;  // 长度为2时输出数据

        // 第一字节数据
        if(sonic_data_length == 1)
        {
            sonic_data16 = 0;
            sonic_data16 = u3rx_buff[0];
        }
            // 第二字节数据
        else if(sonic_data_length == 2)
        {
            sonic_data_length = 0;
            sonic_data16 = (sonic_data16 << 8) + u3rx_buff[0];
        }
            // 不知道啥玩意，从第一字节接受数据
        else
        {
            sonic_data_length = 0;
        }

//        uint16_t distance = u3rx_buff[0];
//        distance = (distance << 8) + u3rx_buff[1];
//        distance_ultra_sonic = (float)distance;
    }
    else if(huart->Instance == USART6){

    }
    //重新设置中断
    HAL_UART_Receive_IT(&huart1,(uint8_t*)u1rx_buff,1);     // PC
    HAL_UART_Receive_IT(&huart2,(uint8_t*)u2rx_buff,1);     // GPS
    HAL_UART_Receive_IT(&huart3,(uint8_t*)u3rx_buff,1);     // 超声波
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
