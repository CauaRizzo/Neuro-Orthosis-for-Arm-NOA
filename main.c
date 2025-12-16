/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define ADC_THRESHOLD_FILTRADO 3.5f
#define NUM_AMOSTRAS 20
#define ADC_MAX_VALUE 4095
#define VREF 3.3f
#define EWMA_ALPHA 0.05f
#define BUFFER_SIZE 5
/* USER CODE END PD */

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t adc_raw_value;
uint32_t last_detection_time;

float MES_filtrado = 0.0f;
float ewma_buffer = 0.0f;

float mes_buffer[BUFFER_SIZE];
int buffer_index = 0;
float mes_average = 0.0f;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
float readMES_STM32 (int N);
float update_mes_average(float current_mes);
/* USER CODE END PFP */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  memset(mes_buffer, 0, sizeof(mes_buffer));
  HAL_ADC_Start(&hadc1);
  last_detection_time = HAL_GetTick();

  while (1)
  {
    MES_filtrado = readMES_STM32(NUM_AMOSTRAS);
    mes_average = update_mes_average(MES_filtrado);

    if (HAL_GetTick() - last_detection_time > 200)
    {
        last_detection_time = HAL_GetTick();

        char message[100];
        int len;

        if (mes_average > ADC_THRESHOLD_FILTRADO)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            len = sprintf(message,
              "STATUS: CLOSED | AVG: %.2f | MES: %.2f | THR: %.2f\r\n",
              mes_average, MES_filtrado, ADC_THRESHOLD_FILTRADO);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            len = sprintf(message,
              "STATUS: OPEN | AVG: %.2f | MES: %.2f | THR: %.2f\r\n",
              mes_average, MES_filtrado, ADC_THRESHOLD_FILTRADO);
        }

        HAL_UART_Transmit(&huart1, (uint8_t*)message, len, 100);
    }
  }
}

float readMES_STM32 (int N)
{
    float sum = 0.0f;
    float val_volts;
    float val_filtrado;

    for (int i = 0; i < N; i++)
    {
        if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
        {
            adc_raw_value = HAL_ADC_GetValue(&hadc1);
            HAL_ADC_Start(&hadc1);
        }

        val_volts = ((float)adc_raw_value / ADC_MAX_VALUE) * VREF;

        ewma_buffer = (EWMA_ALPHA * val_volts) +
                      (1.0f - EWMA_ALPHA) * ewma_buffer;

        val_filtrado = val_volts - ewma_buffer;
        sum += fabsf(val_filtrado);

        HAL_Delay(1);
    }

    return (sum / (float)N) * 100.0f;
}

float update_mes_average(float current_mes)
{
    mes_buffer[buffer_index] = current_mes;
    buffer_index++;

    if (buffer_index >= BUFFER_SIZE)
        buffer_index = 0;

    float sum = 0.0f;
    for (int i = 0; i < BUFFER_SIZE; i++)
        sum += mes_buffer[i];

    return sum / (float)BUFFER_SIZE;
}
