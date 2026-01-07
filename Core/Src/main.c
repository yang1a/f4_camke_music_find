#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "sys.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define ADC_BUFFER_SIZE 1024u               // DMA 搬运的 uint16_t 数值个数（总数）
#define FRAME_SAMPLES (ADC_BUFFER_SIZE / 2) // 每通道一帧的采样点数（因为双通道交错）

uint16_t adc_buffer[ADC_BUFFER_SIZE];

// 去直流后的有符号音频数据（围绕 0 上下波动）
int16_t mic0[FRAME_SAMPLES]; // PA0 = ADC1_IN0
int16_t mic1[FRAME_SAMPLES]; // PA1 = ADC1_IN1

// DMA 帧完成标志
volatile uint8_t frame_ready = 0;

// 采样率统计：统计 DMA 每秒写入多少个 uint16_t 数值
volatile uint32_t sample_count_total = 0;
static uint32_t last_tick_ms = 0;

/**
 * @brief 拆分双通道交错数据，并去直流（减均值）
 * src: [ch0, ch1, ch0, ch1, ...]
 * a  : ch0 输出（PA0）
 * b  : ch1 输出（PA1）
 * n  : 每通道采样点数
 */
static void split_and_remove_dc(const uint16_t *src, int16_t *a, int16_t *b, uint32_t n)
{
  int32_t sum0 = 0, sum1 = 0;

  // 1) 计算每路均值（DC）
  for (uint32_t i = 0; i < n; i++)
  {
    uint16_t s0 = src[2u * i];
    uint16_t s1 = src[2u * i + 1u];
    sum0 += (int32_t)s0;
    sum1 += (int32_t)s1;
  }

  int16_t dc0 = (int16_t)(sum0 / (int32_t)n);
  int16_t dc1 = (int16_t)(sum1 / (int32_t)n);

  // 2) 去直流：转成围绕 0 的有符号音频
  for (uint32_t i = 0; i < n; i++)
  {
    a[i] = (int16_t)src[2u * i] - dc0;      // PA0
    b[i] = (int16_t)src[2u * i + 1u] - dc1; // PA1
  }
}

/**
 * @brief DMA 传输完成回调：只做两件事
 * 1) sample_count_total 累加
 * 2) frame_ready = 1 通知主循环处理
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    sample_count_total += ADC_BUFFER_SIZE;
    frame_ready = 1;
  }
}

// 如果你以后改成 Circular + 半满回调，可以打开这段（更实时）
// void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
// {
//     if (hadc->Instance == ADC1)
//     {
//         sample_count_total += (ADC_BUFFER_SIZE / 2u);
//         // 这里如果要处理半帧，需要再加一个 half_ready 标志并处理 adc_buffer 前半段
//     }
// }

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();

  printf("\r\n========== STM32F407 ADC+DMA (PA0/PA1) Test ==========\r\n");
  printf("ADC buffer size (total uint16): %lu\r\n", (unsigned long)ADC_BUFFER_SIZE);
  printf("Frame samples per channel: %lu\r\n", (unsigned long)FRAME_SAMPLES);

  last_tick_ms = HAL_GetTick();

  // 启动 TIM2（用于触发 ADC）
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  // 启动 ADC + DMA
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  printf("ADC+DMA started. Monitoring sampling rate + data...\r\n");

  while (1)
  {
    // 1) 每秒打印一次采样率（总样本 & 每通道）
    uint32_t now = HAL_GetTick();
    if (now - last_tick_ms >= 1000u)
    {
      uint32_t dt = now - last_tick_ms;
      last_tick_ms = now;

      uint32_t count = sample_count_total;
      sample_count_total = 0;

      // 总写入速率（uint16 数值个数/秒）
      uint32_t total_hz = (count * 1000u) / (dt ? dt : 1u);

      // 双通道时，每通道采样率约 = total_hz / 2
      uint32_t per_ch_hz = total_hz / 2u;

      printf("[Sampling Rate] Total: %lu samples/s, Per-CH: %lu Hz (Target: 48000 Hz)\r\n",
             (unsigned long)total_hz,
             (unsigned long)per_ch_hz);

      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
    }

    // 2) 处理一帧数据：拆分 + 去直流 + 打印验证
    if (frame_ready)
    {
      frame_ready = 0;

      split_and_remove_dc(adc_buffer, mic0, mic1, FRAME_SAMPLES);

      // 打印前 10 点验证：应该围绕 0 上下波动（有正有负）
      printf("mic0(PA0) first 10: ");
      for (int i = 0; i < 10; i++)
      {
        printf("%d ", mic0[i]);
      }
      printf("\r\n");

      printf("mic1(PA1) first 10: ");
      for (int i = 0; i < 10; i++)
      {
        printf("%d ", mic1[i]);
      }
      printf("\r\n");
    }

    HAL_Delay(1);
  }
}

void Error_Handler(void)
{
  __disable_irq();

  // 如果你想用串口打印错误，可以保留（前提：USART 已初始化）
  // printf("\r\n[ERROR_HANDLER]\r\n");

  while (1)
  {
    // 你有 LED 就闪烁，没有 LED 就删掉这两行
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
    HAL_Delay(200);
  }
}
