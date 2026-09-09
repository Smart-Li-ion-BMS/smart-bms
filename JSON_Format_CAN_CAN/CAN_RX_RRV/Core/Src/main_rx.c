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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SMARTBMS_RX_VERSION       "RX_V0.2A"
#define CAN_BATTERY_STD_ID        0x123U
#define CAN_RX_TIMEOUT_MS         3000U
#define STATUS_PRINT_MS           3000U

#define FAULT_OV                  (1U << 0)
#define FAULT_UV                  (1U << 1)
#define FAULT_OC                  (1U << 2)
#define FAULT_OT                  (1U << 3)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef CAN_Filter;
uint8_t RxData[8];

float batteryVoltageV;
float batteryCurrentmA;
float temperatureC;
uint8_t faultFlags;
uint8_t packetCounter;
uint8_t lastPacketCounter;
uint8_t firstPacketReceived = 1U;
uint8_t counterOk = 1U;
uint8_t canTimeout = 0U;
uint32_t rxValidCount;
uint32_t rxIgnoredCount;
uint32_t rxErrorCount;
uint32_t rxEmptyCount;
uint32_t lastValidRxTick;
uint32_t lastStatusPrintTick;
uint32_t canErrorCode;
char str[512];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART_Print(const char *text)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

static void Configure_CAN_Filter(void)
{
    CAN_Filter.FilterBank = 0;
    CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter.FilterIdHigh = (uint16_t)(CAN_BATTERY_STD_ID << 5);
    CAN_Filter.FilterIdLow = 0x0000;
    CAN_Filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5);
    CAN_Filter.FilterMaskIdLow = 0x0000;
    CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_Filter.FilterActivation = ENABLE;
    CAN_Filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter) != HAL_OK)
    {
        Error_Handler();
    }
}

static void Decode_And_Print_JSON(void)
{
    int16_t temp_x100;
    uint16_t volt_x100;
    int16_t curr_x1;
    HAL_StatusTypeDef status;

    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0U)
    {
        rxEmptyCount++;
        return;
    }

    status = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
    if (status != HAL_OK)
    {
        rxErrorCount++;
        canErrorCode = HAL_CAN_GetError(&hcan1);
        snprintf(str, sizeof(str),
                 "{\"dev\":\"bms_rx\",\"ver\":\"%s\",\"st\":\"can_read_error\",\"err\":%lu,\"rxerr\":%lu}\r\n",
                 SMARTBMS_RX_VERSION,
                 (unsigned long)canErrorCode,
                 (unsigned long)rxErrorCount);
        UART_Print(str);
        return;
    }

    if ((RxHeader.IDE != CAN_ID_STD) ||
        (RxHeader.RTR != CAN_RTR_DATA) ||
        (RxHeader.StdId != CAN_BATTERY_STD_ID) ||
        (RxHeader.DLC != 8U))
    {
        rxIgnoredCount++;
        return;
    }

    temp_x100 = (int16_t)(((uint16_t)RxData[1] << 8) | RxData[0]);
    volt_x100 = (uint16_t)(((uint16_t)RxData[3] << 8) | RxData[2]);
    curr_x1   = (int16_t)(((uint16_t)RxData[5] << 8) | RxData[4]);

    temperatureC = ((float)temp_x100) / 100.0f;
    batteryVoltageV = ((float)volt_x100) / 100.0f;
    batteryCurrentmA = (float)curr_x1;
    faultFlags = RxData[6];
    packetCounter = RxData[7];

    if (firstPacketReceived != 0U)
    {
        counterOk = 1U;
        firstPacketReceived = 0U;
    }
    else
    {
        uint8_t expectedCounter = (uint8_t)(lastPacketCounter + 1U);
        counterOk = (packetCounter == expectedCounter) ? 1U : 0U;
    }

    lastPacketCounter = packetCounter;
    rxValidCount++;
    lastValidRxTick = HAL_GetTick();
    canTimeout = 0U;
    canErrorCode = HAL_CAN_GetError(&hcan1);

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

    /* Compact line for ESP32 and minicom. Raw CAN bytes are included for diagnosis. */
    snprintf(str, sizeof(str),
             "{\"dev\":\"bms_rx\",\"ver\":\"%s\",\"st\":\"online\",\"v\":%.2f,\"i\":%.0f,\"t\":%.2f,\"flt\":%u,\"ov\":%s,\"uv\":%s,\"oc\":%s,\"ot\":%s,\"cnt\":%u,\"cnt_ok\":%s,\"rx\":%lu,\"err\":%lu,\"d\":\"%02X %02X %02X %02X %02X %02X %02X %02X\"}\r\n",
             SMARTBMS_RX_VERSION,
             batteryVoltageV,
             batteryCurrentmA,
             temperatureC,
             faultFlags,
             ((faultFlags & FAULT_OV) != 0U) ? "true" : "false",
             ((faultFlags & FAULT_UV) != 0U) ? "true" : "false",
             ((faultFlags & FAULT_OC) != 0U) ? "true" : "false",
             ((faultFlags & FAULT_OT) != 0U) ? "true" : "false",
             packetCounter,
             counterOk ? "true" : "false",
             (unsigned long)rxValidCount,
             (unsigned long)canErrorCode,
             RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
    UART_Print(str);
}

static void Print_Status_JSON_If_Timeout(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - lastValidRxTick) > CAN_RX_TIMEOUT_MS)
    {
        canTimeout = 1U;
    }

    if ((now - lastStatusPrintTick) > STATUS_PRINT_MS)
    {
        lastStatusPrintTick = now;

        if (canTimeout != 0U)
        {
            canErrorCode = HAL_CAN_GetError(&hcan1);
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
            snprintf(str, sizeof(str),
                     "{\"dev\":\"bms_rx\",\"ver\":\"%s\",\"st\":\"waiting_for_can\",\"timeout\":true,\"rx\":%lu,\"empty\":%lu,\"ign\":%lu,\"err\":%lu,\"canerr\":%lu}\r\n",
                     SMARTBMS_RX_VERSION,
                     (unsigned long)rxValidCount,
                     (unsigned long)rxEmptyCount,
                     (unsigned long)rxIgnoredCount,
                     (unsigned long)rxErrorCount,
                     (unsigned long)canErrorCode);
            UART_Print(str);
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
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

    snprintf(str, sizeof(str),
             "{\"dev\":\"bms_rx\",\"ver\":\"%s\",\"st\":\"started\"}\r\n",
             SMARTBMS_RX_VERSION);
    UART_Print(str);

    Configure_CAN_Filter();

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        canErrorCode = HAL_CAN_GetError(&hcan1);
        snprintf(str, sizeof(str),
                 "{\"dev\":\"bms_rx\",\"ver\":\"%s\",\"st\":\"can_start_failed\",\"err\":%lu}\r\n",
                 SMARTBMS_RX_VERSION,
                 (unsigned long)canErrorCode);
        UART_Print(str);
        Error_Handler();
    }

    lastValidRxTick = HAL_GetTick();
    lastStatusPrintTick = HAL_GetTick();

    snprintf(str, sizeof(str),
             "{\"dev\":\"bms_rx\",\"ver\":\"%s\",\"st\":\"waiting_for_can\",\"id\":%u}\r\n",
             SMARTBMS_RX_VERSION,
             CAN_BATTERY_STD_ID);
    UART_Print(str);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    {
        Decode_And_Print_JSON();
        Print_Status_JSON_If_Timeout();
        HAL_Delay(10);
    }
}
  }
  /* USER CODE END WHILE */


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

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_12;
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
  	 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
        HAL_Delay(250);
  }
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
}
#endif /* USE_FULL_ASSERT */
