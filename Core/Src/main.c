/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "encoder.h"
#include <stdio.h>
#include "pid.h"
#include "protocol.h"
#include "remote_control.h"
#include "bluetooth.h"
#include "attitude.h"
#include "bmi088.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PID_OUTPUT_LIMIT 999.0f
#define PID_INTEGRAL_LIMIT 4000.0f
#define STRAIGHT_SYNC_KP 0.35f
#define STRAIGHT_SYNC_KI 0.02f
#define STRAIGHT_SYNC_KD 0.0f
#define STRAIGHT_SYNC_OUTPUT_LIMIT 6.0f
#define STRAIGHT_SYNC_INTEGRAL_LIMIT 100.0f
#define HEADING_HOLD_KP 0.18f
#define HEADING_HOLD_KI 0.002f
#define HEADING_HOLD_KD 0.0f
#define HEADING_HOLD_OUTPUT_LIMIT 5.0f
#define HEADING_HOLD_INTEGRAL_LIMIT 300.0f
#define IMU_UPDATE_PERIOD_MS 20U

PID_t pid_left;
PID_t pid_right;
PID_t pid_straight_sync;
PID_t pid_heading_hold;

volatile int16_t left_speed = 0;
volatile int16_t right_speed = 0;
volatile float straight_sync_adjust = 0.0f;
volatile int32_t imu_yaw_cdeg = 0;
volatile uint8_t imu_heading_ready = 0;

float target_left_speed = 0.0f;
float target_right_speed = 0.0f;

int left_pwm = 0;
int right_pwm = 0;

static BMI088_RawData imu_accel_raw = {0};
static BMI088_RawData imu_gyro_raw = {0};
static BMI088_RawData imu_gyro_offset = {0};
static Attitude_State imu_attitude = {0};
static uint8_t imu_enabled = 0;
static uint32_t imu_last_update_ms = 0;
static int32_t heading_target_cdeg = 0;
static uint8_t heading_hold_active = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void StopControlOutput(void)
{
    PID_Clear(&pid_left);
    PID_Clear(&pid_right);
    PID_Clear(&pid_straight_sync);
    PID_Clear(&pid_heading_hold);
    straight_sync_adjust = 0.0f;
    heading_hold_active = 0;
    left_pwm = 0;
    right_pwm = 0;
    Motor_Left_SetSignedPWM(0);
    Motor_Right_SetSignedPWM(0);
}

static float LimitTargetSpeed(float target)
{
    if (target > REMOTE_CONTROL_MAX_SPEED) target = REMOTE_CONTROL_MAX_SPEED;
    if (target < -REMOTE_CONTROL_MAX_SPEED) target = -REMOTE_CONTROL_MAX_SPEED;
    return target;
}

static void ApplyStraightSyncCorrection(const RemoteControl_State_t *remote)
{
    if (remote->turn != 0)
    {
        PID_Clear(&pid_straight_sync);
        straight_sync_adjust = 0.0f;
        return;
    }

    straight_sync_adjust = PID_Calc(&pid_straight_sync,
                                    0.0f,
                                    (float)left_speed - (float)right_speed);

    target_left_speed = LimitTargetSpeed(target_left_speed + straight_sync_adjust);
    target_right_speed = LimitTargetSpeed(target_right_speed - straight_sync_adjust);
}

static int32_t WrapYawErrorCdeg(int32_t error_cdeg)
{
    while (error_cdeg > 18000)
    {
        error_cdeg -= 36000;
    }

    while (error_cdeg < -18000)
    {
        error_cdeg += 36000;
    }

    return error_cdeg;
}

static void IMU_InitForHeadingHold(void)
{
    Attitude_Init(&imu_attitude);
    imu_heading_ready = 0;

    if (BMI088_Init() != 0)
    {
        imu_enabled = 0;
        return;
    }

    if (Attitude_CalibrateGyro(&imu_gyro_offset) != 0)
    {
        imu_enabled = 0;
        return;
    }

    imu_enabled = 1;
    imu_last_update_ms = HAL_GetTick();
}

static void IMU_Task(void)
{
    if (!imu_enabled)
    {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((now - imu_last_update_ms) < IMU_UPDATE_PERIOD_MS)
    {
        return;
    }

    imu_last_update_ms = now;

    if (BMI088_ReadRaw(&imu_accel_raw, &imu_gyro_raw) == 0)
    {
        Attitude_Update(&imu_attitude,
                        &imu_accel_raw,
                        &imu_gyro_raw,
                        &imu_gyro_offset,
                        now);
        imu_yaw_cdeg = imu_attitude.yaw_cdeg;
        imu_heading_ready = 1;
    }
}

static void ApplyHeadingHoldCorrection(const RemoteControl_State_t *remote)
{
    if (!imu_heading_ready || remote->turn != 0)
    {
        heading_hold_active = 0;
        PID_Clear(&pid_heading_hold);
        return;
    }

    if (!heading_hold_active)
    {
        heading_target_cdeg = imu_yaw_cdeg;
        heading_hold_active = 1;
        PID_Clear(&pid_heading_hold);
        return;
    }

    int32_t yaw_error_cdeg = WrapYawErrorCdeg(imu_yaw_cdeg - heading_target_cdeg);
    float heading_adjust = PID_Calc(&pid_heading_hold, 0.0f, (float)yaw_error_cdeg / 100.0f);

    target_left_speed = LimitTargetSpeed(target_left_speed - heading_adjust);
    target_right_speed = LimitTargetSpeed(target_right_speed + heading_adjust);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  Encoder_Init();
  // Motor_Right_Backward();
  // Motor_Right_SetPWM(500);
  // while (1)
  // {
  // }
  RemoteControl_Init();
  Protocol_Init();
  Bluetooth_Init();

  PID_Init(&pid_left, 25.0f, 0.2f, 0.0f,
           -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT,
           -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
  PID_Init(&pid_right, 25.0f, 0.2f, 0.0f,
           -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT,
           -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
  PID_Init(&pid_straight_sync,
           STRAIGHT_SYNC_KP, STRAIGHT_SYNC_KI, STRAIGHT_SYNC_KD,
           -STRAIGHT_SYNC_OUTPUT_LIMIT, STRAIGHT_SYNC_OUTPUT_LIMIT,
           -STRAIGHT_SYNC_INTEGRAL_LIMIT, STRAIGHT_SYNC_INTEGRAL_LIMIT);
  PID_Init(&pid_heading_hold,
           HEADING_HOLD_KP, HEADING_HOLD_KI, HEADING_HOLD_KD,
           -HEADING_HOLD_OUTPUT_LIMIT, HEADING_HOLD_OUTPUT_LIMIT,
           -HEADING_HOLD_INTEGRAL_LIMIT, HEADING_HOLD_INTEGRAL_LIMIT);
  IMU_InitForHeadingHold();
  HAL_TIM_Base_Start_IT(&htim2);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    IMU_Task();
    Protocol_Task();
    Bluetooth_Task();
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        int16_t left_raw = Encoder_GetLeftSpeed();
        int16_t right_raw = Encoder_GetRightSpeed();

        left_speed = (left_speed * 3 + left_raw) / 4;
        right_speed = (right_speed * 3 + right_raw) / 4;

        if (!RemoteControl_UpdateTargets(&target_left_speed, &target_right_speed))
        {
            StopControlOutput();
            return;
        }

        const RemoteControl_State_t *remote = RemoteControl_GetState();
        if (remote->throttle == 0 && remote->turn == 0)
        {
            target_left_speed = 0.0f;
            target_right_speed = 0.0f;
            StopControlOutput();
            return;
        }

        ApplyStraightSyncCorrection(remote);
        ApplyHeadingHoldCorrection(remote);

        left_pwm = (int)PID_Calc(&pid_left, target_left_speed, left_speed);
        right_pwm = (int)PID_Calc(&pid_right, target_right_speed, right_speed);

        Motor_Left_SetSignedPWM(left_pwm);
        Motor_Right_SetSignedPWM(right_pwm);
    }
}
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
#ifdef USE_FULL_ASSERT
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
