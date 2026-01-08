#include "app.h"
#include "audio_capture.h"
#include "doa.h"
#include "servo.h"
#include "gpio.h"
#include "usart.h"
#include <stdio.h>

// 静态变量
static uint32_t last_tick_ms = 0;
static uint32_t frame_cnt = 0;

/**
 * @brief 应用初始化
 */
void app_init(void)
{
    printf("\r\n========== 2-Mic DOA -> Servo (Gain Mapping) ==========\r\n");
    printf("FS=%.0fHz, frame=%lu/ch, MAX_LAG=%d, micDist=%.2fm\r\n",
           FS_HZ, (unsigned long)FRAME_SAMPLES, (int)MAX_LAG_SAMPLES, (double)MIC_DIST_M);
    printf("Servo: min=%dus max=%dus center=%dus  K=%.2fus/lag  alpha=%.2f  E_TH=%lu\r\n",
           SERVO_US_MIN, SERVO_US_MAX, SERVO_US_CENTER,
           (double)SERVO_K_US_PER_LAG, (double)SERVO_ALPHA, (unsigned long)ENERGY_TH);

    servo_init();
    audio_capture_init();

    last_tick_ms = HAL_GetTick();
    printf("ADC+DMA started. Tracking lag -> servo...\r\n");
}

/**
 * @brief 应用主循环
 */
void app_loop(void)
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

    // 每帧处理:lag -> servo
    if (frame_ready)
    {
        frame_ready = 0;
        frame_cnt++;

        audio_split_and_remove_dc(adc_buffer, mic0, mic1, FRAME_SAMPLES);

        uint32_t e0 = audio_frame_energy(mic0, FRAME_SAMPLES);
        uint32_t e1 = audio_frame_energy(mic1, FRAME_SAMPLES);

        uint8_t valid = (e0 > ENERGY_TH) || (e1 > ENERGY_TH);

        int32_t lag = 0;
        if (valid)
        {
            lag = doa_estimate_lag(mic0, mic1, FRAME_SAMPLES, MAX_LAG_SAMPLES);
        }

        servo_track_from_lag(lag, valid);

        if ((frame_cnt % PRINT_EVERY_NFRAMES) == 0u)
        {
            int out_us = servo_get_current_us();
            printf("E0=%lu E1=%lu | valid=%u | lag=%ld | pwm=%dus\r\n",
                   (unsigned long)e0, (unsigned long)e1,
                   (unsigned)valid, (long)lag, out_us);
        }
    }

    HAL_Delay(1);
}
