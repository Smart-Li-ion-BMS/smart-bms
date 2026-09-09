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
#include <stdarg.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SMARTBMS_RX_VERSION       "SmartBMS_RX_V0.1B_CAN_RECEIVE_PRINT_FIX"
#define CAN_BATTERY_STD_ID        0x123U
#define CAN_RX_TIMEOUT_MS         3000U

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
uint8_t RxData[8];
CAN_FilterTypeDef CAN_Filter;

float batteryVoltageV;
float batteryCurrentmA;
float temperatureC;
uint8_t faultFlags;
uint8_t packetCounter;
uint8_t lastPacketCounter;
uint8_t firstPacketReceived = 1U;
uint8_t counterOk = 1U;
uint8_t counterGap = 0U;
uint32_t rxValidCount;
uint32_t rxIgnoredCount;
uint32_t rxEmptyCount;
uint32_t rxErrorCount;
uint32_t lastValidRxTick;
uint32_t lastWaitingPrintTick;
uint32_t canErrorCode;
char str[160];

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
        case HAL_OK:      return "HAL_OK";
        case HAL_ERROR:   return "HAL_ERROR";
        case HAL_BUSY:    return "HAL_BUSY";
        case HAL_TIMEOUT: return "HAL_TIMEOUT";
        default:          return "HAL_UNKNOWN";
    }
}

static void Configure_CAN_Filter(void)
{
    CAN_Filter.FilterBank = 0;
    CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;

    /* Accept only standard CAN ID 0x123. StdId is placed in bits [15:5]. */
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

static void Build_Fault_Text(uint8_t flags, char *buffer, uint16_t size)
{
    if (flags == 0U)
    {
        snprintf(buffer, size, "NORMAL");
        return;
    }

    buffer[0] = '\0';
    if ((flags & FAULT_OV) != 0U) strncat(buffer, "OV ", size - strlen(buffer) - 1U);
    if ((flags & FAULT_UV) != 0U) strncat(buffer, "UV ", size - strlen(buffer) - 1U);
    if ((flags & FAULT_OC) != 0U) strncat(buffer, "OC ", size - strlen(buffer) - 1U);
    if ((flags & FAULT_OT) != 0U) strncat(buffer, "OT ", size - strlen(buffer) - 1U);
}

static void Update_Counter_Check(void)
{
    counterOk = 1U;
    counterGap = 0U;

    if (firstPacketReceived != 0U)
    {
        firstPacketReceived = 0U;
        lastPacketCounter = packetCounter;
        return;
    }

    uint8_t expectedCounter = (uint8_t)(lastPacketCounter + 1U);

    if (packetCounter != expectedCounter)
    {
        counterOk = 0U;
        counterGap = (uint8_t)(packetCounter - expectedCounter);
    }

    lastPacketCounter = packetCounter;
}

static void Print_Rx_Diagnostics_Line_By_Line(void)
{
    char faultText[40];

    Build_Fault_Text(faultFlags, faultText, sizeof(faultText));

    UART_Print("\r\n========================================\r\n");
    UART_Printf("Version       : %s\r\n", SMARTBMS_RX_VERSION);
    UART_Printf("Uptime        : %lu ms\r\n", (unsigned long)HAL_GetTick());
    UART_Print("----------------------------------------\r\n");
    UART_Print("RX Status     : VALID FRAME RECEIVED\r\n");
    UART_Printf("CAN StdId     : 0x%03lX\r\n", (unsigned long)RxHeader.StdId);
    UART_Printf("CAN IDE       : %s\r\n", (RxHeader.IDE == CAN_ID_STD) ? "STD" : "EXT");
    UART_Printf("CAN RTR       : %s\r\n", (RxHeader.RTR == CAN_RTR_DATA) ? "DATA" : "REMOTE");
    UART_Printf("CAN DLC       : %lu\r\n", (unsigned long)RxHeader.DLC);
    UART_Printf("CAN Data      : %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                 RxData[0], RxData[1], RxData[2], RxData[3],
                 RxData[4], RxData[5], RxData[6], RxData[7]);
    UART_Printf("CAN Counter   : %u\r\n", packetCounter);
    UART_Printf("Counter Check : %s\r\n", counterOk ? "OK" : "MISSED_PACKET_OR_COUNTER_JUMP");
    UART_Printf("Counter Gap   : %u\r\n", counterGap);
    UART_Print("----------------------------------------\r\n");
    UART_Printf("Temperature   : %.2f C\r\n", temperatureC);
    UART_Printf("Battery Volt  : %.2f V\r\n", batteryVoltageV);
    UART_Printf("Battery Curr  : %.2f mA\r\n", batteryCurrentmA);
    UART_Printf("Fault Flags   : 0x%02X\r\n", faultFlags);
    UART_Printf("Fault Text    : %s\r\n", faultText);
    UART_Printf("Fault OV      : %s\r\n", (faultFlags & FAULT_OV) ? "YES" : "NO");
    UART_Printf("Fault UV      : %s\r\n", (faultFlags & FAULT_UV) ? "YES" : "NO");
    UART_Printf("Fault OC      : %s\r\n", (faultFlags & FAULT_OC) ? "YES" : "NO");
    UART_Printf("Fault OT      : %s\r\n", (faultFlags & FAULT_OT) ? "YES" : "NO");
    UART_Print("----------------------------------------\r\n");
    UART_Printf("RX Valid Count: %lu\r\n", (unsigned long)rxValidCount);
    UART_Printf("RX Ignore Cnt : %lu\r\n", (unsigned long)rxIgnoredCount);
    UART_Printf("RX Empty Count: %lu\r\n", (unsigned long)rxEmptyCount);
    UART_Printf("RX Error Count: %lu\r\n", (unsigned long)rxErrorCount);
    UART_Printf("CAN ErrorCode : 0x%08lX\r\n", (unsigned long)canErrorCode);
    UART_Print("========================================\r\n");
}

static void Decode_And_Print_Frame(void)
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
        UART_Printf("RX ERROR: HAL_CAN_GetRxMessage=%s, ErrorCode=0x%08lX\r\n",
                     HAL_Status_String(status),
                     (unsigned long)canErrorCode);
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

    Update_Counter_Check();

    rxValidCount++;
    lastValidRxTick = HAL_GetTick();
    canErrorCode = HAL_CAN_GetError(&hcan1);

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    Print_Rx_Diagnostics_Line_By_Line();
}

static void Print_Waiting_Status_If_Timeout(void)
{
    uint32_t now = HAL_GetTick();

    if (((now - lastValidRxTick) > CAN_RX_TIMEOUT_MS) &&
        ((now - lastWaitingPrintTick) > CAN_RX_TIMEOUT_MS))
    {
        lastWaitingPrintTick = now;
        canErrorCode = HAL_CAN_GetError(&hcan1);
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

        UART_Print("\r\n");
        UART_Printf("[%s]\r\n", SMARTBMS_RX_VERSION);
        UART_Printf("RX Status     : WAITING FOR CAN FRAME ID 0x%03X\r\n", CAN_BATTERY_STD_ID);
        UART_Printf("RX Valid Count: %lu\r\n", (unsigned long)rxValidCount);
        UART_Printf("RX Ignore Cnt : %lu\r\n", (unsigned long)rxIgnoredCount);
        UART_Printf("RX Empty Count: %lu\r\n", (unsigned long)rxEmptyCount);
        UART_Printf("RX Error Count: %lu\r\n", (unsigned long)rxErrorCount);
        UART_Printf("CAN ErrorCode : 0x%08lX\r\n", (unsigned long)canErrorCode);
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

    UART_Print("\r\n");
    UART_Printf("%s started\r\n", SMARTBMS_RX_VERSION);

    Configure_CAN_Filter();

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        UART_Printf("ERROR: HAL_CAN_Start failed, ErrorCode=0x%08lX\r\n",
                     (unsigned long)HAL_CAN_GetError(&hcan1));
        Error_Handler();
    }

    UART_Print("CAN RX started. Waiting for ID 0x123...\r\n");
    lastValidRxTick = HAL_GetTick();
    lastWaitingPrintTick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Decode_And_Print_Frame();
        Print_Waiting_Status_If_Timeout();
        HAL_Delay(10);
  }
  /* USER CODE END WHILE */
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
