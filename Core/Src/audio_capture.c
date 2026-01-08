#include "audio_capture.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "main.h"
#include <stdint.h>

// 音频缓冲区定义
uint16_t adc_buffer[ADC_BUFFER_SIZE];
int16_t mic0[FRAME_SAMPLES];
int16_t mic1[FRAME_SAMPLES];

// 全局变量定义
volatile uint8_t frame_ready = 0;
volatile uint32_t sample_count_total = 0;

/**
 * @brief 拆分双通道交错 + 去直流
 * src: [ch0, ch1, ch0, ch1 ...]
 */
void audio_split_and_remove_dc(const uint16_t *src, int16_t *a, int16_t *b, uint32_t n)
{
    int32_t sum0 = 0, sum1 = 0;

    for (uint32_t i = 0; i < n; i++)
    {
        sum0 += (int32_t)src[2u * i];
        sum1 += (int32_t)src[2u * i + 1u];
    }

    int16_t dc0 = (int16_t)(sum0 / (int32_t)n);
    int16_t dc1 = (int16_t)(sum1 / (int32_t)n);

    for (uint32_t i = 0; i < n; i++)
    {
        a[i] = (int16_t)src[2u * i] - dc0;
        b[i] = (int16_t)src[2u * i + 1u] - dc1;
    }
}

/**
 * @brief 能量(均方,不开方)
 */
uint32_t audio_frame_energy(const int16_t *x, uint32_t n)
{
    uint64_t acc = 0;
    for (uint32_t i = 0; i < n; i++)
    {
        int32_t v = x[i];
        acc += (uint64_t)(v * v);
    }
    return (uint32_t)(acc / n);
}

/**
 * @brief 初始化音频捕获
 */
void audio_capture_init(void)
{
    // 启动 TIM2(触发 ADC)
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    // 启动 ADC + DMA
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief DMA 回调
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        sample_count_total += ADC_BUFFER_SIZE;
        frame_ready = 1;
    }
}
