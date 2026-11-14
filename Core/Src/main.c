/* USER CODE BEGIN Header */
#include "main.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "auto_tuning.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Header */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  char printMessage[100] = {'\0'};
  char rx_buffer[20] = {0};
  uint8_t rx_byte;

  // 1. Configuration initiale du PID
  PID_Controller posPID;
  float Kp = 2.5f, Ki = 0.00f, Kd = 0.0f;
  PID_Init(&posPID, Kp, Ki, Kd, 999.0f, -999.0f);

  float target_position = 10000.0f;
  float smooth_pwm = 0.0f;
  float max_step = 15.0f;

  int is_running = 0;
  int is_autotuning = 0;

  uint32_t last_pid_tick = 0;
  uint32_t last_print_tick = 0;

  Motor_Init();
  Encoder_Init();

  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // ==========================================
    // 1. LECTURE DES COMMANDES PYTHON (UART)
    // ==========================================
    if (HAL_UART_Receive(&huart2, &rx_byte, 1, 0) == HAL_OK) {
        static int idx = 0;
        if (rx_byte == '\n' || rx_byte == '\r') {
            rx_buffer[idx] = '\0';

            if (rx_buffer[0] == 'P') {
                Kp = atof(&rx_buffer[1]);
                PID_Init(&posPID, Kp, Ki, Kd, 999.0f, -999.0f);
            }
            else if (rx_buffer[0] == 'I') {
                Ki = atof(&rx_buffer[1]);
                PID_Init(&posPID, Kp, Ki, Kd, 999.0f, -999.0f);
            }
            else if (rx_buffer[0] == 'D') {
                Kd = atof(&rx_buffer[1]);
                PID_Init(&posPID, Kp, Ki, Kd, 999.0f, -999.0f);
            }
            else if (rx_buffer[0] == 'M') {
                max_step = atof(&rx_buffer[1]);
            }
            else if (rx_buffer[0] == 'S') {
                is_running = 1;
                is_autotuning = 0;
                __HAL_TIM_SET_COUNTER(&htim3, 0);
                posPID.integral = 0;
                smooth_pwm = 0;
            }
            else if (rx_buffer[0] == 'X') {
                is_running = 0;
                is_autotuning = 0;
                Motor_Stop();
            }
            else if (rx_buffer[0] == 'A') {
                is_running = 0;
                is_autotuning = 1;
                Motor_Stop();
                sprintf(printMessage, "AUTOTUNING_START\r\n");
                HAL_UART_Transmit(&huart2, (uint8_t*)printMessage, strlen(printMessage), 10);
            }
            idx = 0;
        } else {
            if (idx < 19) rx_buffer[idx++] = rx_byte;
        }
    }

    // ==========================================
    // 2. EXECUTION DE L'AUTO-TUNING
    // ==========================================
    if (is_autotuning) {
        // Lance l'auto-tuning
        AutoTuneResult tuning = AutoTune_Run(5000.0f, 600.0f, 100.0f, 6000);

        is_autotuning = 0;

        if (tuning.is_done) {
            Kp = tuning.Kp;
            Ki = tuning.Ki;
            Kd = tuning.Kd;
            PID_Init(&posPID, Kp, Ki, Kd, 999.0f, -999.0f);

            // ENVOI DU MESSAGE POUR PYTHON
            sprintf(printMessage, "TUNING_RESULT:%.2f,%.4f,%.2f\r\n", Kp, Ki, Kd);
        } else {
            Kp = 2.0f; Ki = 0.0f; Kd = 0.0f;
            PID_Init(&posPID, Kp, Ki, Kd, 999.0f, -999.0f);

            // ENVOI DU MESSAGE POUR PYTHON (Echec)
            sprintf(printMessage, "TUNING_FAILED:2.0,0.0,0.0\r\n");
        }
        HAL_UART_Transmit(&huart2, (uint8_t*)printMessage, strlen(printMessage), 100);

        __HAL_TIM_SET_COUNTER(&htim3, 0);
        posPID.integral = 0;
        smooth_pwm = 0;
    }

    // ==========================================
    // 3. BOUCLE PID NORMALE
    // ==========================================
    if (is_running && (HAL_GetTick() - last_pid_tick >= 10))
    {
        float dt = (HAL_GetTick() - last_pid_tick) / 1000.0f;
        last_pid_tick = HAL_GetTick();

        float current_position = (float)Encoder_GetCount();
        float pid_output = PID_Compute(&posPID, target_position, current_position, dt);

        if (pid_output > smooth_pwm) {
            smooth_pwm += max_step;
            if (smooth_pwm > pid_output) smooth_pwm = pid_output;
        } else if (pid_output < smooth_pwm) {
            smooth_pwm -= max_step;
            if (smooth_pwm < pid_output) smooth_pwm = pid_output;
        }

        Motor_SetPWM((int32_t)smooth_pwm);

        // Envoi à Python toutes les 100ms
        if (HAL_GetTick() - last_print_tick >= 100) {
            last_print_tick = HAL_GetTick();
            sprintf(printMessage, "%d,%d,%d\r\n",
                    (int)target_position, (int)current_position, (int)smooth_pwm);
            HAL_UART_Transmit(&huart2, (uint8_t*)printMessage, strlen(printMessage), 10);
        }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM1 Initialization Function
  */
static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }

  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM3 Initialization Function
  */
static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, en1_Pin|en2_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = en1_Pin|en2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
