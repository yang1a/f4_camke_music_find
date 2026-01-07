#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "sys.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>

// ======================= 采样参数 =======================
#define FS_HZ 48000.0f
#define ADC_BUFFER_SIZE 1024u
#define FRAME_SAMPLES (ADC_BUFFER_SIZE / 2u) // 每通道 512 点（双通道交错）

// 两麦距离 12cm（这里只用于你以后算角度，现在不打印角度）
#define MIC_DIST_M 0.12f

// 物理最大延迟：12cm / 343m/s ≈ 350us；48kHz -> 16~17 samples
#define MAX_LAG_SAMPLES 16

// 能量门控：过小认为无声，不更新舵机（按现场噪声调）
#define ENERGY_TH 60000u

// 每 N 帧打印一次，避免串口拖慢实时
#define PRINT_EVERY_NFRAMES 10u

// ======================= 舵机映射参数（满量程增益映射） =======================
// 舵机 PWM 范围（先用保守值；若觉得幅度不够可扩大，若顶死咔咔响就缩小）
#define SERVO_US_MIN 600
#define SERVO_US_MAX 2500
#define SERVO_US_CENTER ((SERVO_US_MIN + SERVO_US_MAX) / 2)

// 满量程增益映射：K = (max-min) / (2*MAX_LAG)
#define SERVO_K_US_PER_LAG (((float)(SERVO_US_MAX - SERVO_US_MIN)) / (2.0f * (float)MAX_LAG_SAMPLES))

// 死区（减少抖动）
#define LAG_DEADBAND 1

// 一阶滤波系数（0.1~0.2 常用）
#define SERVO_ALPHA 0.15f

// LED
#define LED_PORT GPIOC
#define LED_PIN GPIO_PIN_0

// ======================= 全局/静态变量 =======================
static uint16_t adc_buffer[ADC_BUFFER_SIZE];
static int16_t mic0[FRAME_SAMPLES];
static int16_t mic1[FRAME_SAMPLES];

static volatile uint8_t frame_ready = 0;
static volatile uint32_t sample_count_total = 0;

static uint32_t last_tick_ms = 0;
static uint32_t frame_cnt = 0;

// 舵机滤波后的当前输出
static float servo_us_filt = (float)SERVO_US_CENTER;

// ======================= 工具函数 =======================
static inline int clamp_int(int v, int lo, int hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static inline float clamp_f(float v, float lo, float hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

/**
 * @brief 拆分双通道交错 + 去直流
 * src: [ch0, ch1, ch0, ch1 ...]
 */
static void split_and_remove_dc(const uint16_t *src, int16_t *a, int16_t *b, uint32_t n)
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
 * @brief 能量（均方，不开方）
 */
static uint32_t frame_energy(const int16_t *x, uint32_t n)
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
 * @brief NCC 归一化互相关估计 lag（抗 MAX9814 AGC 更稳）
 */
static int32_t estimate_lag_ncc(const int16_t *x, const int16_t *y, uint32_t n, int32_t max_lag)
{
  float best = -1e9f;
  int32_t best_lag = 0;

  for (int32_t lag = -max_lag; lag <= max_lag; lag++)
  {
    int64_t sum_xy = 0;
    uint64_t sum_x2 = 0;
    uint64_t sum_y2 = 0;

    // 只累加重叠区域
    uint32_t i_start = (lag < 0) ? (uint32_t)(-lag) : 0u;
    uint32_t i_end = (lag > 0) ? (n - (uint32_t)lag) : n;

    for (uint32_t i = i_start; i < i_end; i++)
    {
      int32_t xi = x[i];
      int32_t yi = y[i + lag];
      sum_xy += (int64_t)xi * (int64_t)yi;
      sum_x2 += (uint64_t)(xi * xi);
      sum_y2 += (uint64_t)(yi * yi);
    }

    if (sum_x2 == 0 || sum_y2 == 0)
      continue;

    float denom = sqrtf((float)sum_x2 * (float)sum_y2);
    float ncc = (float)sum_xy / denom;

    if (ncc > best)
    {
      best = ncc;
      best_lag = lag;
    }
  }

  return best_lag;
}

static void servo_write_us(int us)
{
  us = clamp_int(us, SERVO_US_MIN, SERVO_US_MAX);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)us);
}

/**
 * @brief lag -> 舵机 PWM（满量程映射 + 死区 + 滤波）
 * valid=0 时不更新（保持）
 */
static void servo_track_from_lag(int32_t lag, uint8_t valid)
{
  if (!valid)
    return;

  // 物理限幅（防误峰）
  if (lag > MAX_LAG_SAMPLES)
    lag = MAX_LAG_SAMPLES;
  if (lag < -MAX_LAG_SAMPLES)
    lag = -MAX_LAG_SAMPLES;

  // 死区
  if (lag >= -LAG_DEADBAND && lag <= LAG_DEADBAND)
    lag = 0;

  // 满量程映射：中心 + lag*K
  float target = (float)SERVO_US_CENTER + (float)lag * SERVO_K_US_PER_LAG;
  target = clamp_f(target, (float)SERVO_US_MIN, (float)SERVO_US_MAX);

  // 一阶低通滤波
  servo_us_filt = servo_us_filt + SERVO_ALPHA * (target - servo_us_filt);

  servo_write_us((int)(servo_us_filt + 0.5f));
}

// ======================= DMA 回调 =======================
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    sample_count_total += ADC_BUFFER_SIZE;
    frame_ready = 1;
  }
}

// ======================= main =======================
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  printf("\r\n========== 2-Mic DOA -> Servo (Gain Mapping) ==========\r\n");
  printf("FS=%.0fHz, frame=%lu/ch, MAX_LAG=%d, micDist=%.2fm\r\n",
         FS_HZ, (unsigned long)FRAME_SAMPLES, (int)MAX_LAG_SAMPLES, (double)MIC_DIST_M);
  printf("Servo: min=%dus max=%dus center=%dus  K=%.2fus/lag  alpha=%.2f  E_TH=%lu\r\n",
         SERVO_US_MIN, SERVO_US_MAX, SERVO_US_CENTER,
         (double)SERVO_K_US_PER_LAG, (double)SERVO_ALPHA, (unsigned long)ENERGY_TH);

  // 启动舵机 PWM（TIM3_CH1 PA6）
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  servo_write_us(SERVO_US_CENTER);

  // 启动 TIM2（触发 ADC）
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  // 启动 ADC + DMA
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  last_tick_ms = HAL_GetTick();
  printf("ADC+DMA started. Tracking lag -> servo...\r\n");

  while (1)
  {
    // 每秒打印采样率
    uint32_t now = HAL_GetTick();
    if (now - last_tick_ms >= 1000u)
    {
      uint32_t dt = now - last_tick_ms;
      last_tick_ms = now;

      uint32_t count = sample_count_total;
      sample_count_total = 0;

      uint32_t total_hz = (count * 1000u) / (dt ? dt : 1u);
      uint32_t per_ch_hz = total_hz / 2u;

      printf("[Sampling] total=%lu/s, per_ch=%lu Hz\r\n",
             (unsigned long)total_hz, (unsigned long)per_ch_hz);

      HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }

    // 每帧处理：lag -> servo
    if (frame_ready)
    {
      frame_ready = 0;
      frame_cnt++;

      split_and_remove_dc(adc_buffer, mic0, mic1, FRAME_SAMPLES);

      uint32_t e0 = frame_energy(mic0, FRAME_SAMPLES);
      uint32_t e1 = frame_energy(mic1, FRAME_SAMPLES);

      uint8_t valid = (e0 > ENERGY_TH) || (e1 > ENERGY_TH);

      int32_t lag = 0;
      if (valid)
      {
        lag = estimate_lag_ncc(mic0, mic1, FRAME_SAMPLES, MAX_LAG_SAMPLES);
      }

      servo_track_from_lag(lag, valid);

      if ((frame_cnt % PRINT_EVERY_NFRAMES) == 0u)
      {
        int out_us = (int)(servo_us_filt + 0.5f);
        printf("E0=%lu E1=%lu | valid=%u | lag=%ld | pwm=%dus\r\n",
               (unsigned long)e0, (unsigned long)e1,
               (unsigned)valid, (long)lag, out_us);
      }
    }

    HAL_Delay(1);
  }
}

// ======================= Error_Handler =======================
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_Delay(200);
  }
}
