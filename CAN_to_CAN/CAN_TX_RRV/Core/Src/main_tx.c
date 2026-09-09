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
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include "ina219.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SMARTBMS_TX_VERSION       "SmartBMS_TX_V0.2C"
#define CAN_BATTERY_STD_ID        0x123U
#define CAN_TX_CONFIRM_TIMEOUT_MS 50U
#define CAN_ALL_TX_MAILBOXES      (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)

#define ADC_VREF                  3.0f
#define ADC_MAX_COUNT             4095.0f
#define NTC_R_FIXED               10000.0f
#define NTC_R0                    10000.0f
#define NTC_T0_K                  298.15f
#define NTC_BETA                  3950.0f

#define OVER_VOLTAGE_LIMIT        4.25f
#define UNDER_VOLTAGE_LIMIT       3.00f
#define OVER_CURRENT_LIMIT        1000.0f
#define OVER_TEMP_LIMIT           60.0f

#define FAULT_OV                  (1U << 0)
#define FAULT_UV                  (1U << 1)
#define FAULT_OC                  (1U << 2)
#define FAULT_OT                  (1U << 3)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_FilterTypeDef CAN_Filter;
uint8_t TxData[8];
uint32_t TxMailbox;

uint32_t adcValue;
float ntcVoltage, resistance, temperatureC;
float batteryVoltageV, batteryCurrentmA;
uint8_t faultFlags, ina219Ready, packetCounter;
uint32_t debugCounter;
char str[180];

HAL_StatusTypeDef canAddStatus, canAbortStatus;
uint8_t canTxConfirmed, canTxTimeout, canAbortDone, canMailboxFullBeforeAbort;
uint32_t canTxErrorCode, canTxMailboxFree, canTxWaitMs, canTxQueuedMailbox;
uint32_t canTxSuccessCounter, canTxFailCounter, canAbortCounter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void UART_Print(const char *text)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

static void UART_Printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(str, sizeof(str), fmt, args);
    va_end(args);
    UART_Print(str);
}

static const char *HAL_Status_String(HAL_StatusTypeDef status)
{
    switch (status)
    {
        case HAL_OK: return "HAL_OK";
        case HAL_ERROR: return "HAL_ERROR";
        case HAL_BUSY: return "HAL_BUSY";
        case HAL_TIMEOUT: return "HAL_TIMEOUT";
        default: return "HAL_UNKNOWN";
    }
}

static const char *Can_Result_String(void)
{
    if (canAddStatus != HAL_OK) return "ADD_TX_FAILED";
    if (canTxConfirmed != 0U) return "TX_CONFIRMED_MAILBOX_EMPTY";
    if (canTxTimeout != 0U) return "TX_TIMEOUT_ABORTED_NO_ACK_OR_BUS_ISSUE";
    return "TX_UNKNOWN";
}

static float Read_Temperature_C(void)
{
    float temperatureK;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adcValue = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    ntcVoltage = ((float)adcValue * ADC_VREF) / ADC_MAX_COUNT;
    if (ntcVoltage <= 0.01f) ntcVoltage = 0.01f;
    if (ntcVoltage >= (ADC_VREF - 0.01f)) ntcVoltage = ADC_VREF - 0.01f;
    resistance = NTC_R_FIXED * (ADC_VREF - ntcVoltage) / ntcVoltage;
    if (resistance <= 1.0f) resistance = 1.0f;
    temperatureK = 1.0f / ((1.0f / NTC_T0_K) + ((1.0f / NTC_BETA) * logf(resistance / NTC_R0)));
    return temperatureK - 273.15f;
}

static uint8_t Build_Fault_Flags(float vbat, float currmA, float tempC)
{
    uint8_t flags = 0U;
    if (vbat > OVER_VOLTAGE_LIMIT) flags |= FAULT_OV;
    if (vbat < UNDER_VOLTAGE_LIMIT) flags |= FAULT_UV;
    if (fabsf(currmA) > OVER_CURRENT_LIMIT) flags |= FAULT_OC;
    if (tempC > OVER_TEMP_LIMIT) flags |= FAULT_OT;
    return flags;
}

static void Update_Outputs(uint8_t faults)
{
    if (faults != 0U)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    }
}

static void Abort_All_CAN_Tx_Mailboxes(void)
{
    canAbortStatus = HAL_CAN_AbortTxRequest(&hcan1, CAN_ALL_TX_MAILBOXES);
    canAbortDone = 1U;
    canAbortCounter++;
    canTxErrorCode = HAL_CAN_GetError(&hcan1);
}

static void Send_Battery_Frame(float tempC, float voltV, float currmA, uint8_t faults)
{
    int16_t temp_x100 = (int16_t)((tempC * 100.0f) + ((tempC >= 0.0f) ? 0.5f : -0.5f));
    uint16_t volt_x100 = (uint16_t)((voltV * 100.0f) + 0.5f);
    int16_t curr_x1 = (int16_t)(currmA + ((currmA >= 0.0f) ? 0.5f : -0.5f));
    uint32_t startTick;

    TxData[0] = (uint8_t)(temp_x100 & 0xFF);
    TxData[1] = (uint8_t)((temp_x100 >> 8) & 0xFF);
    TxData[2] = (uint8_t)(volt_x100 & 0xFF);
    TxData[3] = (uint8_t)((volt_x100 >> 8) & 0xFF);
    TxData[4] = (uint8_t)(curr_x1 & 0xFF);
    TxData[5] = (uint8_t)((curr_x1 >> 8) & 0xFF);
    TxData[6] = faults;
    TxData[7] = packetCounter;

    canAddStatus = HAL_ERROR;
    canAbortStatus = HAL_OK;
    canTxConfirmed = 0U;
    canTxTimeout = 0U;
    canAbortDone = 0U;
    canMailboxFullBeforeAbort = 0U;
    canTxWaitMs = 0U;
    canTxQueuedMailbox = 0U;
    canTxMailboxFree = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

    if (canTxMailboxFree == 0U)
    {
        canMailboxFullBeforeAbort = 1U;
        Abort_All_CAN_Tx_Mailboxes();
        canTxMailboxFree = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    }

    canAddStatus = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    canTxQueuedMailbox = TxMailbox;

    if (canAddStatus == HAL_OK)
    {
        startTick = HAL_GetTick();
        while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox) != 0U)
        {
            if ((HAL_GetTick() - startTick) >= CAN_TX_CONFIRM_TIMEOUT_MS)
            {
                canTxTimeout = 1U;
                break;
            }
        }
        canTxWaitMs = HAL_GetTick() - startTick;

        if (canTxTimeout == 0U)
        {
            canTxConfirmed = 1U;
            canTxSuccessCounter++;
        }
        else
        {
            (void)HAL_CAN_AbortTxRequest(&hcan1, TxMailbox);
            canAbortDone = 1U;
            canAbortCounter++;
            canTxFailCounter++;
        }
        packetCounter++;
    }
    else
    {
        canTxFailCounter++;
    }

    canTxErrorCode = HAL_CAN_GetError(&hcan1);
}

static void UART_Print_TX_Debug(void)
{
    UART_Print("\r\n========================================\r\n");
    UART_Printf("Version       : %s\r\n", SMARTBMS_TX_VERSION);
    UART_Printf("Debug Count   : %lu\r\n", (unsigned long)debugCounter);
    UART_Printf("Uptime        : %lu ms\r\n", (unsigned long)HAL_GetTick());
    UART_Print("----------------------------------------\r\n");
    UART_Printf("ADC Raw       : %lu counts\r\n", (unsigned long)adcValue);
    UART_Printf("NTC Voltage   : %.3f V\r\n", ntcVoltage);
    UART_Printf("NTC Resistor  : %.2f ohm\r\n", resistance);
    UART_Printf("Temperature   : %.2f C\r\n", temperatureC);
    UART_Print("----------------------------------------\r\n");
    UART_Printf("INA219 Ready  : %s\r\n", ina219Ready ? "YES" : "NO");
    UART_Printf("INA219 Status : %s\r\n", HAL_Status_String(INA219_GetLastStatus()));
    UART_Printf("INA219 BusRaw : 0x%04X\r\n", INA219_GetLastBusRaw());
    UART_Printf("INA219 ShuRaw : %d\r\n", INA219_GetLastShuntRaw());
    UART_Printf("Battery Volt  : %.2f V\r\n", batteryVoltageV);
    UART_Printf("Battery Curr  : %.2f mA\r\n", batteryCurrentmA);
    UART_Print("----------------------------------------\r\n");
    UART_Printf("Fault Flags   : 0x%02X\r\n", faultFlags);
    UART_Printf("Fault OV      : %s\r\n", (faultFlags & FAULT_OV) ? "YES" : "NO");
    UART_Printf("Fault UV      : %s\r\n", (faultFlags & FAULT_UV) ? "YES" : "NO");
    UART_Printf("Fault OC      : %s\r\n", (faultFlags & FAULT_OC) ? "YES" : "NO");
    UART_Printf("Fault OT      : %s\r\n", (faultFlags & FAULT_OT) ? "YES" : "NO");
    UART_Printf("Output        : %s\r\n", (faultFlags != 0U) ? "FAULT: RED LED ON, BUZZER ON" : "NORMAL: GREEN LED ON");
    UART_Print("----------------------------------------\r\n");
    UART_Printf("CAN StdId     : 0x%03lX\r\n", (unsigned long)TxHeader.StdId);
    UART_Printf("CAN DLC       : %lu\r\n", (unsigned long)TxHeader.DLC);
    UART_Printf("CAN Data      : %02X %02X %02X %02X %02X %02X %02X %02X\r\n", TxData[0], TxData[1], TxData[2], TxData[3], TxData[4], TxData[5], TxData[6], TxData[7]);
    UART_Printf("CAN Counter   : %u\r\n", TxData[7]);
    UART_Printf("CAN AddStatus : %s\r\n", HAL_Status_String(canAddStatus));
    UART_Printf("CAN Result    : %s\r\n", Can_Result_String());
    UART_Printf("Mailbox Full? : %s\r\n", canMailboxFullBeforeAbort ? "YES_ABORTED_BEFORE_SEND" : "NO");
    UART_Printf("CAN AbortDone : %s\r\n", canAbortDone ? "YES" : "NO");
    UART_Printf("CAN AbortStat : %s\r\n", HAL_Status_String(canAbortStatus));
    UART_Printf("CAN Mailbox   : 0x%08lX\r\n", (unsigned long)canTxQueuedMailbox);
    UART_Printf("CAN FreeBefore: %lu\r\n", (unsigned long)canTxMailboxFree);
    UART_Printf("CAN Wait      : %lu ms\r\n", (unsigned long)canTxWaitMs);
    UART_Printf("CAN ErrorCode : 0x%08lX\r\n", (unsigned long)canTxErrorCode);
    UART_Printf("CAN OK Count  : %lu\r\n", (unsigned long)canTxSuccessCounter);
    UART_Printf("CAN Fail Count: %lu\r\n", (unsigned long)canTxFailCounter);
    UART_Printf("CAN Abort Cnt : %lu\r\n", (unsigned long)canAbortCounter);
    UART_Print("========================================\r\n");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  INA219_Init(&hi2c1);

  /* USER CODE BEGIN 2 */
    TxHeader.StdId = CAN_BATTERY_STD_ID;
    TxHeader.ExtId = 0U;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8U;
    TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    UART_Print("\r\n");
    UART_Printf("%s started\r\n", SMARTBMS_TX_VERSION);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    debugCounter++;
        ina219Ready = INA219_IsDeviceReady();
        temperatureC = Read_Temperature_C();
        batteryVoltageV = INA219_GetBusVoltage_V();
        batteryCurrentmA = INA219_GetCurrent_mA();
        faultFlags = Build_Fault_Flags(batteryVoltageV, batteryCurrentmA, temperatureC);
        Update_Outputs(faultFlags);
        Send_Battery_Frame(temperatureC, batteryVoltageV, batteryCurrentmA, faultFlags);
        UART_Print_TX_Debug();
        HAL_Delay(1000);
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
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

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}
  
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}


/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
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

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to source file name
  * @param  line: assert_param line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
