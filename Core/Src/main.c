/* USER CODE BEGIN Header */
#include "main.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Header */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PV */
#define RX_BUF_SIZE   128
#define TX_BUF_SIZE   64

#define MAX_PWM        999.0f
#define MIN_PWM        100.0f   // PWM minimum pour demarrer (a ajuster)
#define DEADBAND_CM    0.20f
#define K_SYNC         25.0f    // PWM par cm d'ecart entre les roues
#define LOOP_MS        20
#define DT_S           0.020f

static uint8_t  rx_ring[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0;

static char     tx_buf[TX_BUF_SIZE];
static uint16_t tx_len = 0, tx_pos = 0;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
/*
 * UART 100% polling : AUCUNE interruption, AUCUN USART2_IRQHandler requis.
 * (Avant : HAL_UART_Receive_IT sans handler = le MCU se figeait au 1er octet recu)
 * A appeler le plus souvent possible dans la boucle principale.
 */
static void uart_service(void)
{
  // --- RX : on vide le registre dans le ring buffer ---
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
    uint8_t b = (uint8_t)(huart2.Instance->DR & 0xFF);   // lire DR efface aussi ORE/FE/NE
    uint16_t next = (uint16_t)((rx_head + 1) % RX_BUF_SIZE);
    if (next != rx_tail) { rx_ring[rx_head] = b; rx_head = next; }
  }
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
    (void)huart2.Instance->SR;
    (void)huart2.Instance->DR;
  }

  // --- TX non bloquant : 1 octet des que TXE est libre ---
  if (tx_pos < tx_len && __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE)) {
    huart2.Instance->DR = (uint8_t)tx_buf[tx_pos++];
  }
}

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  char rx_buffer[24] = {0};
  uint8_t idx = 0;

  Motor_Init();
  Encoder_Init();

  const float WHEEL_DIAMETER = 7.0f;
  const float TICKS_PER_REV  = 1635.0f;
  const float PI_F           = 3.14159f;
  const float TICKS_PER_CM   = TICKS_PER_REV / (PI_F * WHEEL_DIAMETER);

  PID_Controller pidGauche, pidDroite;
  float Kp_G = 40.0f, Ki_G = 0.5f, Kd_G = 1.5f;
  float Kp_D = 40.0f, Ki_D = 0.5f, Kd_D = 1.5f;

  PID_Init(&pidGauche, Kp_G, Ki_G, Kd_G, MAX_PWM, -MAX_PWM);
  PID_Init(&pidDroite, Kp_D, Ki_D, Kd_D, MAX_PWM, -MAX_PWM);

  float target_cm_G = 0.0f, target_cm_D = 0.0f;
  float current_cm_G = 0.0f, current_cm_D = 0.0f;
  uint32_t last_loop_tick = 0, last_print = 0;

  // Mode test moteurs en boucle ouverte : "M,pwmG,pwmD" (dure 1 s)
  uint8_t  ol_mode = 0;
  int32_t  ol_G = 0, ol_D = 0;
  uint32_t ol_until = 0;

  HAL_Delay(500);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    uart_service();

    // ========== 1. COMMANDES ==========
    while (rx_tail != rx_head) {
      uint8_t c = rx_ring[rx_tail];
      rx_tail = (uint16_t)((rx_tail + 1) % RX_BUF_SIZE);

      if (c == '\n' || c == '\r') {
        rx_buffer[idx] = '\0';
        idx = 0;
        if (rx_buffer[0] == '\0') continue;

        if (rx_buffer[0] == 'S' && rx_buffer[1] == ',') {
          int mmG = 0, mmD = 0;
          if (sscanf(rx_buffer, "S,%d,%d", &mmG, &mmD) == 2) {
            ol_mode = 0;
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            __HAL_TIM_SET_COUNTER(&htim4, 0);
            target_cm_G = mmG / 10.0f;
            target_cm_D = mmD / 10.0f;
            current_cm_G = 0.0f;
            current_cm_D = 0.0f;
            PID_Init(&pidGauche, Kp_G, Ki_G, Kd_G, MAX_PWM, -MAX_PWM);
            PID_Init(&pidDroite, Kp_D, Ki_D, Kd_D, MAX_PWM, -MAX_PWM);
          }
        }
        else if (rx_buffer[0] == 'M' && rx_buffer[1] == ',') {
          int a = 0, b = 0;
          if (sscanf(rx_buffer, "M,%d,%d", &a, &b) == 2) {
            ol_G = a; ol_D = b;
            ol_mode = 1;
            ol_until = HAL_GetTick() + 1000;
          }
        }
        else if (rx_buffer[0] == 'X') {
          ol_mode = 0;
          target_cm_G = current_cm_G;
          target_cm_D = current_cm_D;
          PID_Init(&pidGauche, Kp_G, Ki_G, Kd_G, MAX_PWM, -MAX_PWM);
          PID_Init(&pidDroite, Kp_D, Ki_D, Kd_D, MAX_PWM, -MAX_PWM);
        }
        else if (rx_buffer[1] == 'G' || rx_buffer[1] == 'D') {
          float v = (float)atof(&rx_buffer[2]);
          char k = rx_buffer[0], side = rx_buffer[1];
          if (side == 'G') {
            if (k == 'P') Kp_G = v; else if (k == 'I') Ki_G = v; else if (k == 'D') Kd_G = v;
            PID_Init(&pidGauche, Kp_G, Ki_G, Kd_G, MAX_PWM, -MAX_PWM);
          } else {
            if (k == 'P') Kp_D = v; else if (k == 'I') Ki_D = v; else if (k == 'D') Kd_D = v;
            PID_Init(&pidDroite, Kp_D, Ki_D, Kd_D, MAX_PWM, -MAX_PWM);
          }
        }
      } else if (idx < sizeof(rx_buffer) - 1) {
        rx_buffer[idx++] = (char)c;
      }
      uart_service();
    }

    // ========== 2. BOUCLE 20 ms ==========
    if (HAL_GetTick() - last_loop_tick >= LOOP_MS) {
      last_loop_tick = HAL_GetTick();

      current_cm_G = (float)Encoder_GetCount_Gauche() / TICKS_PER_CM;
      current_cm_D = (float)Encoder_GetCount_Droite() / TICKS_PER_CM;
      uart_service();

      if (ol_mode) {
        // ---- Test moteurs : PWM direct, coupe apres 1 s ----
        if ((int32_t)(HAL_GetTick() - ol_until) >= 0) {
          ol_mode = 0;
          Motor_SetSpeed_Gauche(0);
          Motor_SetSpeed_Droite(0);
          target_cm_G = current_cm_G;
          target_cm_D = current_cm_D;
          PID_Init(&pidGauche, Kp_G, Ki_G, Kd_G, MAX_PWM, -MAX_PWM);
          PID_Init(&pidDroite, Kp_D, Ki_D, Kd_D, MAX_PWM, -MAX_PWM);
        } else {
          Motor_SetSpeed_Gauche((int32_t)clampf((float)ol_G, -MAX_PWM, MAX_PWM));
          Motor_SetSpeed_Droite((int32_t)clampf((float)ol_D, -MAX_PWM, MAX_PWM));
        }
      } else {
        // ---- PID position ----
        float errG = target_cm_G - current_cm_G;
        float errD = target_cm_D - current_cm_D;

        float outG = PID_Compute(&pidGauche, target_cm_G, current_cm_G, DT_S);
        float outD = PID_Compute(&pidDroite, target_cm_D, current_cm_D, DT_S);
        uart_service();

        outG = clampf(outG, -MAX_PWM, MAX_PWM);
        outD = clampf(outD, -MAX_PWM, MAX_PWM);

        // Synchro des roues (ligne droite / rotation sur place)
        float sgnG = (target_cm_G >= 0) ? 1.0f : -1.0f;
        float sgnD = (target_cm_D >= 0) ? 1.0f : -1.0f;
        if (fabsf(target_cm_G) > 1.0f && fabsf(target_cm_D) > 1.0f &&
            fabsf(fabsf(target_cm_G) - fabsf(target_cm_D)) < 1.0f &&
            (fabsf(errG) > DEADBAND_CM || fabsf(errD) > DEADBAND_CM)) {
          float diff = (current_cm_G * sgnG) - (current_cm_D * sgnD);   // >0 : G en avance
          float corr = K_SYNC * diff;
          outG -= sgnG * corr;
          outD += sgnD * corr;
          outG = clampf(outG, -MAX_PWM, MAX_PWM);
          outD = clampf(outD, -MAX_PWM, MAX_PWM);
        }

        // Zone morte + PWM minimum
        if (fabsf(errG) < DEADBAND_CM) {
          outG = 0;
          PID_Init(&pidGauche, Kp_G, Ki_G, Kd_G, MAX_PWM, -MAX_PWM);
        } else if (fabsf(outG) < MIN_PWM) {
          outG = (errG > 0) ? MIN_PWM : -MIN_PWM;
        }
        if (fabsf(errD) < DEADBAND_CM) {
          outD = 0;
          PID_Init(&pidDroite, Kp_D, Ki_D, Kd_D, MAX_PWM, -MAX_PWM);
        } else if (fabsf(outD) < MIN_PWM) {
          outD = (errD > 0) ? MIN_PWM : -MIN_PWM;
        }

        Motor_SetSpeed_Gauche((int32_t)outG);
        Motor_SetSpeed_Droite((int32_t)outD);
      }

      // ---- Telemetrie (mm) toutes les 50 ms, non bloquante ----
      if (HAL_GetTick() - last_print >= 50 && tx_pos >= tx_len) {
        last_print = HAL_GetTick();
        int n = snprintf(tx_buf, sizeof(tx_buf), "%d,%d,%d,%d\r\n",
                         (int)lroundf(target_cm_G * 10.0f), (int)lroundf(current_cm_G * 10.0f),
                         (int)lroundf(target_cm_D * 10.0f), (int)lroundf(current_cm_D * 10.0f));
        if (n > 0) { tx_len = (uint16_t)n; tx_pos = 0; }
      }
    }
    /* USER CODE END 3 */
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84;          // identique a ton code d'origine
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

static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }

  HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM4_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

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

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
