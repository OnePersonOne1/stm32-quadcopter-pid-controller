/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_MESSAGE_SIZE 256U
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
uint8_t rxBuffer[1];  // Single character buffer
uint8_t messageBuffer[MAX_MESSAGE_SIZE];  // Complete message buffer
uint16_t messageIndex = 0;  // Current position in message
uint8_t messageComplete = 0;  // Flag indicating complete message
double roll, pitch, yaw;        // IMU angle

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
    double kp, ki, kd;
    double i_term;
    double prev_err;
} PID_t;
// need experiment
static PID_t pidRoll  = { 3.0, 0.0, 30.0, 0.0, 0.0 };
static PID_t pidPitch = { 3.0, 0.0, 30.0, 0.0, 0.0 };
static PID_t pidYaw   = { 1.0, 0.0,  0.0, 0.0, 0.0 };

// sp: target, mv: sensor state(current)
static inline double pid_update(PID_t *p, double sp, double mv, double dt)
{
    double err = sp - mv;
    p->i_term += p->ki * err * dt;
    double d   = p->kd * (err - p->prev_err) / dt;
    p->prev_err = err;
    return p->kp * err + p->i_term + d;
}

/* PSC = 720-1, ARR = 1000-1 */
static inline void setMotorDuty(uint8_t channel, double dutyPercent)
/*
    channel: 0‥3 (CH1..CH4)
    dutyPercent: 0.0 – 100.0 (%)
*/
{
    if (dutyPercent < 0.0)   dutyPercent = 0.0;
    if (dutyPercent > 100.0) dutyPercent = 100.0;

    /* total ticks per period = ARR + 1 */
    uint16_t periodTicks = TIM1->ARR + 1;
    uint16_t ccrValue    = (uint16_t)((dutyPercent / 100.0) * periodTicks);

    switch (channel)
    {
        case 0: TIM1->CCR1 = ccrValue; break;   /* PA8  */
        case 1: TIM1->CCR2 = ccrValue; break;   /* PA9  */
        case 2: TIM1->CCR3 = ccrValue; break;   /* PA10 */
        default:TIM1->CCR4 = ccrValue; break;   /* PA11 */
    }
}


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
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  while (MPU6050_Init(&hi2c2) == 1);

  // Start UART reception in interrupt mode (single character)
  HAL_UART_Receive_IT(&huart2, rxBuffer, 1);
  
  // Send welcome message
  char welcomeMsg[] = "UART2 Interrupt Ready!\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)welcomeMsg, strlen(welcomeMsg), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t tick_prev = 0; // Thus static, only once called
    uint32_t tick_now = HAL_GetTick();          /* milliseconds */
    double dt = (tick_now - tick_prev) * 0.001; /* seconds      */
    if (dt < 0.001) dt = 0.001;                 /* clamp to 1 ms */
    tick_prev = tick_now;

    /* 1. Read raw sensors (driver also updates Kalman angles) */
    MPU6050_Read_All(&hi2c2, &MPU6050);

    double roll  = MPU6050.KalmanAngleX;   /* deg, double */
    double pitch = MPU6050.KalmanAngleY;   /* deg, double */
    static double yaw = 0.0;         /* yaw from gyro integration */
    yaw += MPU6050.Gz * dt;          /* Gz: deg/s */

    /* 2. PID computation (target: level = 0°) */
    double uRoll  = pid_update(&pidRoll , 0.0, roll , dt);
    double uPitch = pid_update(&pidPitch, 0.0, pitch, dt);
    double uYaw   = pid_update(&pidYaw , 0.0, yaw  , dt);

    /* 3. Mixer: +X quad (FL, FR, RR, RL) */
    double baseDuty = 45.0; // %

    setMotorDuty(0, baseDuty + uPitch - uRoll + uYaw); /* Motor 1 (PA8)  */
    setMotorDuty(1, baseDuty + uPitch + uRoll - uYaw); /* Motor 2 (PA9)  */
    setMotorDuty(2, baseDuty - uPitch + uRoll + uYaw); /* Motor 3 (PA10) */
    setMotorDuty(3, baseDuty - uPitch - uRoll - uYaw); /* Motor 4 (PA11) */

    /* 4. Loop timing: 1 kHz */
    HAL_Delay(1);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
