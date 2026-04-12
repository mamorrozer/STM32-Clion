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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "oled.h"
#include "font.h"
#include "Servo.h"
#include "pid.h"
#include "vl53l0x.h"
#include "../Inc/Serial.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_PERIOD_MS             10U
#define TELEMETRY_PERIOD_MS           100U
#define PID_ENABLE_DELAY_MS           1200U
#define MAX_CONSEC_INVALID_SAMPLES    30U

#define SENSOR_VALID_MIN_MM           40U
#define SENSOR_VALID_MAX_MM           360U
#define SENSOR_MAX_STEP_MM            60U

#define DISTANCE_CENTER_MM            160.0f
#define DEFAULT_TARGET_OFFSET_MM      0.0f

#define SERVO_CENTER_ANGLE            90.0f
#define SERVO_MIN_ANGLE               55.0f
#define SERVO_MAX_ANGLE               125.0f
#define SERVO_MAX_STEP_DEG            1.5f

#define PID_OUT_MIN_DEG               (-25.0f)
#define PID_OUT_MAX_DEG               (25.0f)
#define PID_INTEGRAL_ACTIVE_BAND_MM   45.0f
#define PID_KP                        0.20f
#define PID_KI                        0.015f
#define PID_KD                        0.12f
#define CONTROL_PERIOD_SEC            0.01f
#define PID_INTEGRAL_LIMIT            120.0f
#define INVALID_TELEMETRY_MM          (-999.0f)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
typedef enum {
  CONTROL_MODE_PD = 0,
  CONTROL_MODE_PID = 1
} ControlMode_t;

typedef enum {
  CONTROL_STATE_INIT = 0,
  CONTROL_STATE_RUN = 1,
  CONTROL_STATE_FAULT = 2
} ControlState_t;

char oled_line[24];
PID_Handle pid;
volatile uint16_t g_distance_mm = 0xFFFFU;
volatile uint16_t g_last_valid_distance_mm = 0U;
volatile float g_servo_angle = SERVO_CENTER_ANGLE;
float g_setpoint_offset_mm = DEFAULT_TARGET_OFFSET_MM;
uint32_t g_serial_last_tick = 0U;
uint32_t g_control_start_tick = 0U;
uint32_t g_last_control_tick = 0U;
uint16_t g_consecutive_invalid_samples = 0U;
bool g_has_valid_distance = false;
ControlMode_t g_control_mode = CONTROL_MODE_PD;
ControlState_t g_control_state = CONTROL_STATE_INIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float clampf(float value, float min_val, float max_val)
{
  if (value < min_val)
  {
    return min_val;
  }
  if (value > max_val)
  {
    return max_val;
  }
  return value;
}

static float apply_slew_limit(float previous, float target, float max_step)
{
  if (target > previous + max_step)
  {
    return previous + max_step;
  }
  if (target < previous - max_step)
  {
    return previous - max_step;
  }
  return target;
}

static float distance_to_offset_mm(uint16_t distance_mm)
{
  return (float)distance_mm - DISTANCE_CENTER_MM;
}

static uint16_t abs_diff_u16(uint16_t a, uint16_t b)
{
  return (a > b) ? (a - b) : (b - a);
}

/* 读取一次激光测距结果，返回单位 mm。
 * 返回 0xFFFF 表示本次测量无效。 */
uint16_t VL53L0X_GetDistance(void)
{
  VL53L0X_RangingMeasurementData_t ranging_data;
  static char range_status_buf[VL53L0X_MAX_STRING_LENGTH];

  if (vl53l0x_start_single_test(&vl53l0x_dev, &ranging_data, range_status_buf) != VL53L0X_ERROR_NONE)
  {
    return 0xFFFFU;
  }

  if (ranging_data.RangeStatus != 0U)
  {
    return 0xFFFFU;
  }

  return ranging_data.RangeMilliMeter;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  /* OLED 初始化 */
  OLED_Init();
  OLED_NewFrame();
  OLED_PrintString(0, 0, "VL53L0X Init...", &font16x16, OLED_COLOR_NORMAL);
  OLED_ShowFrame();
  /* 初始化 VL53L0X（XSHUT=PA1，GPIO1=PA2） */
  if (vl53l0x_init(&vl53l0x_dev) != VL53L0X_ERROR_NONE)
  {
    Error_Handler();
  }
  /* 设置为高频率模式 */
  if (vl53l0x_set_mode(&vl53l0x_dev, HIGH_SPEED) != VL53L0X_ERROR_NONE)
  {
    Error_Handler();
  }
  Servo_Init();
  Servo_SetAngle(SERVO_CENTER_ANGLE);
  PID_Init(&pid, PID_KP, PID_KI, PID_KD, CONTROL_PERIOD_SEC, PID_OUT_MIN_DEG, PID_OUT_MAX_DEG, PID_INTEGRAL_LIMIT, PID_INTEGRAL_ACTIVE_BAND_MM);
  pid.integral_enabled = false;
  PID_Reset(&pid, 0.0f);
  g_control_start_tick = HAL_GetTick();
  g_last_control_tick = g_control_start_tick;
  g_control_state = CONTROL_STATE_INIT;
  g_control_mode = CONTROL_MODE_PD;
  Serial_Printf("VL53L0X ready\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - g_last_control_tick) < CONTROL_PERIOD_MS)
    {
      HAL_Delay(1);
      continue;
    }

    g_last_control_tick = now;
    uint16_t sampled_distance = VL53L0X_GetDistance();
    g_distance_mm = sampled_distance;

    bool valid_sample = false;
    if ((sampled_distance >= SENSOR_VALID_MIN_MM) && (sampled_distance <= SENSOR_VALID_MAX_MM))
    {
      if (!g_has_valid_distance)
      {
        valid_sample = true;
      }
      else
      {
        uint16_t delta = abs_diff_u16(sampled_distance, g_last_valid_distance_mm);
        valid_sample = (delta <= SENSOR_MAX_STEP_MM);
      }
    }

    if (valid_sample)
    {
      g_last_valid_distance_mm = sampled_distance;
      g_has_valid_distance = true;
      g_consecutive_invalid_samples = 0U;

      float measurement_mm = distance_to_offset_mm(sampled_distance);

      if (g_control_state != CONTROL_STATE_RUN)
      {
        g_control_state = CONTROL_STATE_RUN;
        PID_Reset(&pid, measurement_mm);
      }

      if ((uint32_t)(now - g_control_start_tick) >= PID_ENABLE_DELAY_MS)
      {
        pid.integral_enabled = true;
        g_control_mode = CONTROL_MODE_PID;
      }
      else
      {
        pid.integral_enabled = false;
        g_control_mode = CONTROL_MODE_PD;
      }

      float control_output_deg = PID_Update(&pid, g_setpoint_offset_mm, measurement_mm);
      float target_servo_angle = SERVO_CENTER_ANGLE + control_output_deg;
      target_servo_angle = clampf(target_servo_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
      g_servo_angle = apply_slew_limit(g_servo_angle, target_servo_angle, SERVO_MAX_STEP_DEG);
      Servo_SetAngle(g_servo_angle);
    }
    else
    {
      g_consecutive_invalid_samples++;

      if (g_consecutive_invalid_samples > MAX_CONSEC_INVALID_SAMPLES)
      {
        g_control_state = CONTROL_STATE_FAULT;
        pid.integral_enabled = false;
        g_control_mode = CONTROL_MODE_PD;
        PID_Reset(&pid, 0.0f);
        g_servo_angle = apply_slew_limit(g_servo_angle, SERVO_CENTER_ANGLE, SERVO_MAX_STEP_DEG);
        Servo_SetAngle(g_servo_angle);
      }
    }

    /* 显示最近一次测距结果到 OLED */
    if (!valid_sample)
    {
      sprintf(oled_line, "Dis: -- mm");
    }
    else
    {
      sprintf(oled_line, "Dis:%4u mm", sampled_distance);
    }

    OLED_NewFrame();
    OLED_PrintString(0, 0, "Ball Beam Ctrl", &font16x16, OLED_COLOR_NORMAL);
    OLED_PrintString(0, 20, oled_line, &font16x16, OLED_COLOR_NORMAL);
    OLED_ShowFrame();

    if ((uint32_t)(now - g_serial_last_tick) >= TELEMETRY_PERIOD_MS)
    {
      g_serial_last_tick = now;
      float measurement_mm = g_has_valid_distance ? distance_to_offset_mm(g_last_valid_distance_mm) : INVALID_TELEMETRY_MM;
      float error_mm = g_has_valid_distance ? (g_setpoint_offset_mm - measurement_mm) : INVALID_TELEMETRY_MM;
      Serial_Printf("T=%.1f M=%.1f E=%.1f U=%.1f ANG=%.1f MODE=%u STATE=%u\r\n",
                    g_setpoint_offset_mm,
                    measurement_mm,
                    error_mm,
                    g_servo_angle - SERVO_CENTER_ANGLE,
                    g_servo_angle,
                    (unsigned int)g_control_mode,
                    (unsigned int)g_control_state);
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
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

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 333-1;  /* 10 kHz / 333 ≈ 30.03 Hz */
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 200-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PIN_VL53L0X_XSHUT_GPIO_Port, PIN_VL53L0X_XSHUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PIN_VL53L0X_XSHUT_Pin */
  GPIO_InitStruct.Pin = PIN_VL53L0X_XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_VL53L0X_XSHUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_VL53L0X_GPIO1_Pin */
  GPIO_InitStruct.Pin = PIN_VL53L0X_GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PIN_VL53L0X_GPIO1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  (void)htim;
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
