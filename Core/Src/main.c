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
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_PERIOD_DEFAULT  999         /* 1kHz PWM 주기 (0 ~ 999) */
#define ENCODER_DEADBAND    2           /* P제어 목표 도달 허용 오차 (±2 카운트) */
#define MIN_MOVE_PWR        25          /* 모터 마찰력을 이기는 최소 구동 파워 (%) */
#define BKP_MAGIC_VALUE     0xA5A55A5AUL
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile int32_t motor_power_percent  = 50;   /* 모터 최대 파워 (%) : 0 ~ 100 */
volatile int32_t inc_encoder_input    = 0;    /* 증가시킬 인코더 값 입력 */
volatile uint8_t inc_move_btn         = 0;    /* 증가 이동 실행 버튼 */
volatile uint8_t reset_btn            = 0;    /* 리셋 버튼 (1 입력 시 0으로 초기화) */
volatile int32_t target_encoder_input = 0;    /* 원하는 목표 인코더 값 입력 */
volatile uint8_t target_move_btn      = 0;    /* 목표 위치 이동 실행 버튼 */
volatile uint8_t stop_btn             = 0;    /* 정지 버튼 */

volatile int32_t current_encoder      = 0;    /* 현재 인코더 위치 */
volatile int32_t target_encoder       = 0;    /* 목표 인코더 위치 */
volatile uint8_t motor_moving         = 0;    /* 0: 정지, 1: 정방향(CW), 2: 역방향(CCW) */
volatile int32_t current_duty_percent = 0;    /* 현재 실제 출력 Duty (%) : 0 ~ 100 */
volatile int32_t current_speed_cps    = 0;    /* 현재 인코더 속도 (counts/sec, 0 이상) */

/* P 제어 비례 게인 (Kp): 오차(목표-현재)에 곱해지는 비율 */
volatile float   p_gain_kp            = 0.1f;

static uint8_t   p_control_active     = 0;    /* P제어 동작 활성화 플래그 */
static uint8_t   settled_cnt          = 0;    /* 목표 범위 안착 확인 카운터 */
static uint32_t  last_ctrl_tick       = 0;
static int32_t   prev_enc_for_speed   = 0;

static int32_t   prev_inc_input       = 0;
static int32_t   prev_target_input    = 0;

static uint8_t   uart_rx_byte         = 0;
static char      uart_cmd_buf[64];
static uint8_t   uart_cmd_idx         = 0;
static uint32_t  last_telemetry_tick  = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Motor_Stop(void);
void Motor_Brake(void);
void Motor_SetDrive(int8_t dir, int32_t power_percent);
void Encoder_Update(void);
void Encoder_Reset(void);
void Motor_StartRelativeMove(int32_t delta_counts);
void Motor_StartAbsoluteMove(int32_t target_counts);
void Motor_PositionControlStep(void);
void UART_ProcessCommand(const char *cmd);
void UART_PollRx(void);
void UART_SendTelemetry(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Encoder_SaveBackup(int32_t val)
{
  HAL_PWR_EnableBkUpAccess();
  RTC->BKP0R = (uint32_t)val;
  RTC->BKP1R = BKP_MAGIC_VALUE;
}

/**
  * @brief L298N 전자 브레이크 (IN1=HIGH, IN2=HIGH로 관성 밀림 즉시 제동)
  */
void Motor_Brake(void)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_Pin | MOTOR_IN2_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr);
  current_duty_percent = 0;
}

/**
  * @brief 모터 완전 정지 및 P제어 종료
  */
void Motor_Stop(void)
{
  HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_Pin | MOTOR_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  motor_moving = 0;
  current_duty_percent = 0;
  p_control_active = 0;
  settled_cnt = 0;
}

void Motor_SetDrive(int8_t dir, int32_t power_percent)
{
  if (power_percent <= 0 || dir == 0)
  {
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_Pin | MOTOR_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    motor_moving = 0;
    current_duty_percent = 0;
    return;
  }

  if (power_percent > 100)
  {
    power_percent = 100;
  }

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  uint32_t ccr = (uint32_t)((power_percent * arr) / 100);

  if (dir > 0)
  {
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    motor_moving = 1;
    current_duty_percent = power_percent;
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN2_Pin, GPIO_PIN_SET);
    motor_moving = 2;
    current_duty_percent = power_percent;
  }

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}

void Encoder_Update(void)
{
  int32_t now_enc = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  if (now_enc != current_encoder)
  {
    current_encoder = now_enc;
    Encoder_SaveBackup(current_encoder);
  }
}

void Encoder_Reset(void)
{
  Motor_Stop();
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  current_encoder = 0;
  target_encoder = 0;
  target_encoder_input = 0;
  prev_target_input = 0;
  inc_encoder_input = 0;
  prev_inc_input = 0;
  Encoder_SaveBackup(0);
}

void Motor_StartRelativeMove(int32_t delta_counts)
{
  if (delta_counts == 0)
  {
    return;
  }
  Motor_StartAbsoluteMove(current_encoder + delta_counts);
}

void Motor_StartAbsoluteMove(int32_t target_counts)
{
  target_encoder = target_counts;
  target_encoder_input = target_counts;
  prev_target_input = target_encoder_input;

  settled_cnt = 0;
  p_control_active = 1;
}

/**
  * @brief P 제어(비례 제어) 위치 제어기
  *        - 오차(error = target - current)에 비례해 파워를 줄여 감속
  *        - 만약 목표값을 넘어가면 error 부호가 반대가 되어 역방향으로 되돌아옴
  *        - 오차가 ±ENCODER_DEADBAND 이내에 안착하면 전자 브레이크 후 정지
  */
void Motor_PositionControlStep(void)
{
  if (!p_control_active)
  {
    return;
  }

  uint32_t now = HAL_GetTick();
  if (now - last_ctrl_tick < 5)
  {
    return;
  }
  last_ctrl_tick = now;

  if (motor_power_percent < 0)   motor_power_percent = 0;
  if (motor_power_percent > 100) motor_power_percent = 100;

  if (motor_power_percent == 0)
  {
    Motor_Stop();
    return;
  }

  int32_t error = target_encoder - current_encoder;

  /* 1. 오차가 허용 범위(±2 카운트) 안에 들어오면 전자 브레이크 걸고 안착 확인 */
  if (abs(error) <= ENCODER_DEADBAND)
  {
    Motor_Brake();
    settled_cnt++;
    if (settled_cnt >= 8) /* 약 40ms 동안 범위 안에 머물면 완전 정지 */
    {
      Motor_Stop();
    }
    return;
  }
  settled_cnt = 0;

  /* 2. P 제어 계산: 출력 파워(%) = Kp * |오차| */
  int8_t dir = (error > 0) ? 1 : -1;
  int32_t p_out = (int32_t)(p_gain_kp * (float)abs(error));

  /* 3. 모터가 멈추지 않고 움직일 수 있는 최소 파워 보장 */
  if (p_out < MIN_MOVE_PWR)
  {
    p_out = MIN_MOVE_PWR;
  }

  /* 4. 사용자가 입력한 파워(%)를 최대치로 제한 */
  if (p_out > motor_power_percent)
  {
    p_out = motor_power_percent;
  }

  Motor_SetDrive(dir, p_out);
}

void UART_ProcessCommand(const char *cmd)
{
  while (*cmd == ' ' || *cmd == '\t')
  {
    cmd++;
  }
  if (*cmd == '\0')
  {
    return;
  }

  char type = *cmd;
  const char *arg = cmd + 1;
  if (*arg == ':' || *arg == '=' || *arg == ' ')
  {
    arg++;
  }

  switch (type)
  {
    case 'P':
    case 'p':
    {
      int32_t val = (int32_t)strtol(arg, NULL, 10);
      if (val < 0)   val = 0;
      if (val > 100) val = 100;
      motor_power_percent = val;
      break;
    }
    case 'I':
    case 'i':
    {
      int32_t delta = (int32_t)strtol(arg, NULL, 10);
      inc_encoder_input = delta;
      prev_inc_input = delta;
      Motor_StartRelativeMove(delta);
      break;
    }
    case 'T':
    case 't':
    {
      int32_t target = (int32_t)strtol(arg, NULL, 10);
      Motor_StartAbsoluteMove(target);
      break;
    }
    case 'D':
    case 'd':
    {
      int dir = 0, pwr = 0;
      if (sscanf(arg, "%d,%d", &dir, &pwr) == 2)
      {
        p_control_active = 0;
        motor_power_percent = pwr;
        Motor_SetDrive((int8_t)dir, (int32_t)pwr);
      }
      break;
    }
    case 'M':
    case 'm':
    {
      int dir = 0, pwr = 0;
      long tgt = 0;
      if (sscanf(arg, "%d,%d,%ld", &dir, &pwr, &tgt) == 3)
      {
        motor_power_percent = pwr;
        Motor_StartAbsoluteMove((int32_t)tgt);
      }
      break;
    }
    case 'R':
    case 'r':
    {
      Encoder_Reset();
      break;
    }
    case 'S':
    case 's':
    {
      Motor_Stop();
      break;
    }
    default:
      break;
  }
}

static void UART_PushByte(uint8_t ch)
{
  if (ch == '\r' || ch == '\n')
  {
    if (uart_cmd_idx > 0)
    {
      uart_cmd_buf[uart_cmd_idx] = '\0';
      UART_ProcessCommand(uart_cmd_buf);
      uart_cmd_idx = 0;
    }
  }
  else if (uart_cmd_idx < sizeof(uart_cmd_buf) - 1)
  {
    uart_cmd_buf[uart_cmd_idx++] = (char)ch;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    UART_PushByte(uart_rx_byte);
    __HAL_UNLOCK(&huart2);
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UNLOCK(&huart2);
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
  }
}

void UART_PollRx(void)
{
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_OREFLAG(&huart2);
  }
  if (huart2.RxState == HAL_UART_STATE_READY)
  {
    __HAL_UNLOCK(&huart2);
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
  }
}

void UART_SendTelemetry(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t dt = now - last_telemetry_tick;
  if (dt >= 50)
  {
    last_telemetry_tick = now;
    int32_t d_enc = abs(current_encoder - prev_enc_for_speed);
    prev_enc_for_speed = current_encoder;
    current_speed_cps = (int32_t)((d_enc * 1000L) / (int32_t)dt);

    char msg[112];
    int len = snprintf(msg, sizeof(msg), "STAT:ENC=%ld,TGT=%ld,PWR=%ld,MOV=%u,DUTY=%ld,SPD=%ld\r\n",
                       (long)current_encoder,
                       (long)target_encoder,
                       (long)motor_power_percent,
                       (unsigned int)motor_moving,
                       (long)current_duty_percent,
                       (long)current_speed_cps);
    for (int i = 0; i < len; i++)
    {
      uint32_t t0 = HAL_GetTick();
      while (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE))
      {
        if (HAL_GetTick() - t0 > 5) break;
      }
      huart2.Instance->DR = (uint8_t)msg[i];
    }
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_SET_AUTORELOAD(&htim1, PWM_PERIOD_DEFAULT);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  Motor_Stop();

  /* 처음 켰을 때 0으로 리셋하지 않고 현재 모터 인코더 값을 유지/복원 */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  if (RTC->BKP1R == BKP_MAGIC_VALUE)
  {
    __HAL_TIM_SET_COUNTER(&htim2, RTC->BKP0R);
  }
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  current_encoder = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  target_encoder = current_encoder;
  target_encoder_input = current_encoder;
  prev_target_input = current_encoder;

  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Encoder_Update();
    UART_PollRx();

    if (reset_btn != 0)
    {
      reset_btn = 0;
      Encoder_Reset();
    }

    if (stop_btn != 0)
    {
      stop_btn = 0;
      Motor_Stop();
    }

    if (inc_move_btn != 0 || (inc_encoder_input != prev_inc_input && inc_encoder_input != 0))
    {
      inc_move_btn = 0;
      prev_inc_input = inc_encoder_input;
      Motor_StartRelativeMove(inc_encoder_input);
    }

    if (target_move_btn != 0 || (target_encoder_input != prev_target_input))
    {
      target_move_btn = 0;
      prev_target_input = target_encoder_input;
      Motor_StartAbsoluteMove(target_encoder_input);
    }

    Motor_PositionControlStep();
    UART_SendTelemetry();
    /* USER CODE END WHILE */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
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

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_Pin|MOTOR_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_IN1_Pin MOTOR_IN2_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN1_Pin|MOTOR_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
