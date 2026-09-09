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
#include "ina219.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SMARTBMS_TX_VERSION       "SmartBMS_TX_V0.1_USART_DEBUG"
#define CAN_BATTERY_STD_ID        0x123U

#define ADC_VREF                  3.3f
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
uint8_t TxData[8];
uint32_t TxMailbox;

uint32_t adcValue;
float ntcVoltage;
float resistance;
float temperatureC;
float batteryVoltageV;
float batteryCurrentmA;
uint8_t faultFlags;
uint8_t ina219Ready;
uint32_t debugCounter;
char str[320];


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


static int32_t To_Scaled_100(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * 100.0f) + 0.5f);
    }
    else
    {
        return (int32_t)((value * 100.0f) - 0.5f);
    }
}

static int32_t To_Scaled_1000(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * 1000.0f) + 0.5f);
    }
    else
    {
        return (int32_t)((value * 1000.0f) - 0.5f);
    }
}

static void Append_Fixed_2(char *buffer, size_t size, const char *label, float value, const char *unit)
{
    int32_t scaled = To_Scaled_100(value);
    int32_t abs_scaled = (scaled < 0) ? -scaled : scaled;

    snprintf(buffer, size,
             "%s%s%ld.%02ld %s\r\n",
             label,
             (scaled < 0) ? "-" : "",
             (long)(abs_scaled / 100),
             (long)(abs_scaled % 100),
             unit);

    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

static void Append_Fixed_3(char *buffer, size_t size, const char *label, float value, const char *unit)
{
    int32_t scaled = To_Scaled_1000(value);
    int32_t abs_scaled = (scaled < 0) ? -scaled : scaled;

    snprintf(buffer, size,
             "%s%s%ld.%03ld %s\r\n",
             label,
             (scaled < 0) ? "-" : "",
             (long)(abs_scaled / 1000),
             (long)(abs_scaled % 1000),
             unit);

    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

static const char *HAL_Status_String(HAL_StatusTypeDef status)
{
    switch (status)
    {
        case HAL_OK:      return "HAL_OK";
        case HAL_ERROR:   return "HAL_ERROR";
        case HAL_BUSY:    return "HAL_BUSY";
        case HAL_TIMEOUT: return "HAL_TIMEOUT";
        default:          return "HAL_UNKNOWN";
    }
}

static float Read_Temperature_C(void)
{
    float temperatureK;

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adcValue = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    ntcVoltage = ((float)adcValue * ADC_VREF) / ADC_MAX_COUNT;

    if (ntcVoltage <= 0.01f)
    {
        ntcVoltage = 0.01f;
    }
    else if (ntcVoltage >= (ADC_VREF - 0.01f))
    {
        ntcVoltage = ADC_VREF - 0.01f;
    }

    /* Option B divider: 3.3V -> NTC -> ADC PA0 -> 10k fixed resistor -> GND. */
    resistance = NTC_R_FIXED * (ADC_VREF - ntcVoltage) / ntcVoltage;

    if (resistance <= 1.0f)
    {
        resistance = 1.0f;
    }

    temperatureK = 1.0f / ((1.0f / NTC_T0_K) + ((1.0f / NTC_BETA) * logf(resistance / NTC_R0)));
    return temperatureK - 273.15f;
}

static uint8_t Build_Fault_Flags(float vbat, float currmA, float tempC)
{
    uint8_t flags = 0U;

    if (vbat > OVER_VOLTAGE_LIMIT)
    {
        flags |= FAULT_OV;
    }

    if (vbat < UNDER_VOLTAGE_LIMIT)
    {
        flags |= FAULT_UV;
    }

    if (fabsf(currmA) > OVER_CURRENT_LIMIT)
    {
        flags |= FAULT_OC;
    }

    if (tempC > OVER_TEMP_LIMIT)
    {
        flags |= FAULT_OT;
    }

    return flags;
}

static void Update_Outputs(uint8_t faults)
{
    if (faults != 0U)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);    /* Buzzer ON */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);  /* Green LED OFF */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);    /* Red LED ON */
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  /* Buzzer OFF */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);    /* Green LED ON */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);  /* Red LED OFF */
    }
}

static void UART_Print_TX_Debug(void)
{
    snprintf(str, sizeof(str),
             "\r\n========================================\r\n"
             "Version       : %s\r\n"
             "Debug Count   : %lu\r\n"
             "Uptime        : %lu ms\r\n"
             "----------------------------------------\r\n",
             SMARTBMS_TX_VERSION,
             (unsigned long)debugCounter,
             (unsigned long)HAL_GetTick());
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

    snprintf(str, sizeof(str), "ADC Raw       : %lu counts\r\n", (unsigned long)adcValue);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

    Append_Fixed_3(str, sizeof(str), "NTC Voltage   : ", ntcVoltage, "V");
    Append_Fixed_2(str, sizeof(str), "NTC Resistor  : ", resistance, "ohm");
    Append_Fixed_2(str, sizeof(str), "Temperature   : ", temperatureC, "C");

    snprintf(str, sizeof(str),
             "----------------------------------------\r\n"
             "INA219 Ready  : %s\r\n"
             "INA219 Status : %s\r\n"
             "INA219 BusRaw : 0x%04X\r\n"
             "INA219 ShuRaw : %d\r\n",
             ina219Ready ? "YES" : "NO",
             HAL_Status_String(INA219_GetLastStatus()),
             INA219_GetLastBusRaw(),
             INA219_GetLastShuntRaw());
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

    Append_Fixed_2(str, sizeof(str), "Battery Volt  : ", batteryVoltageV, "V");
    Append_Fixed_2(str, sizeof(str), "Battery Curr  : ", batteryCurrentmA, "mA");

    snprintf(str, sizeof(str),
             "----------------------------------------\r\n"
             "Fault Flags   : 0x%02X\r\n"
             "Fault OV      : %s  limit > %.2f V\r\n"
             "Fault UV      : %s  limit < %.2f V\r\n"
             "Fault OC      : %s  limit > %.0f mA absolute\r\n"
             "Fault OT      : %s  limit > %.0f C\r\n"
             "Output        : %s\r\n"
             "========================================\r\n",
             faultFlags,
             (faultFlags & FAULT_OV) ? "YES" : "NO", OVER_VOLTAGE_LIMIT,
             (faultFlags & FAULT_UV) ? "YES" : "NO", UNDER_VOLTAGE_LIMIT,
             (faultFlags & FAULT_OC) ? "YES" : "NO", OVER_CURRENT_LIMIT,
             (faultFlags & FAULT_OT) ? "YES" : "NO", OVER_TEMP_LIMIT,
             (faultFlags != 0U) ? "FAULT: RED LED ON, BUZZER ON" : "NORMAL: GREEN LED ON");
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

static void Send_Battery_Frame(float tempC, float voltV, float currmA, uint8_t faults)
{
    int16_t temp_x100 = (int16_t)((tempC * 100.0f) + ((tempC >= 0.0f) ? 0.5f : -0.5f));
    uint16_t volt_x100 = (uint16_t)((voltV * 100.0f) + 0.5f);
    int16_t curr_x1 = (int16_t)(currmA + ((currmA >= 0.0f) ? 0.5f : -0.5f));

    TxData[0] = (uint8_t)(temp_x100 & 0xFF);
    TxData[1] = (uint8_t)((temp_x100 >> 8) & 0xFF);
    TxData[2] = (uint8_t)(volt_x100 & 0xFF);
    TxData[3] = (uint8_t)((volt_x100 >> 8) & 0xFF);
    TxData[4] = (uint8_t)(curr_x1 & 0xFF);
    TxData[5] = (uint8_t)((curr_x1 >> 8) & 0xFF);
    TxData[6] = faults;
    TxData[7] = 0U;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
    {
        (void)HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    }
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
    
    (void)HAL_CAN_Start(&hcan1);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    snprintf(str, sizeof(str), "\r\n%s started\r\n", SMARTBMS_TX_VERSION);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

    
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
