#ifndef __APP_H
#define __APP_H

#include <stdint.h>

// ======================= 采样参数 =======================
#define FS_HZ 48000.0f
#define ADC_BUFFER_SIZE 1024u
#define FRAME_SAMPLES (ADC_BUFFER_SIZE / 2u)

// 两麦距离
#define MIC_DIST_M 0.12f

// 物理最大延迟
#define MAX_LAG_SAMPLES 16

// 能量门控
#define ENERGY_TH 60000u

// 打印间隔
#define PRINT_EVERY_NFRAMES 10u

// LED
#define LED_PORT GPIOC
#define LED_PIN GPIO_PIN_0

// 全局变量声明
extern volatile uint8_t frame_ready;
extern volatile uint32_t sample_count_total;

// 应用初始化和主循环
void app_init(void);
void app_loop(void);

#endif /* __APP_H */
