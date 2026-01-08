#ifndef __AUDIO_CAPTURE_H
#define __AUDIO_CAPTURE_H

#include <stdint.h>
#include "app.h"

// 音频缓冲区
extern uint16_t adc_buffer[ADC_BUFFER_SIZE];
extern int16_t mic0[FRAME_SAMPLES];
extern int16_t mic1[FRAME_SAMPLES];

// 音频捕获函数
void audio_capture_init(void);
void audio_split_and_remove_dc(const uint16_t *src, int16_t *a, int16_t *b, uint32_t n);
uint32_t audio_frame_energy(const int16_t *x, uint32_t n);

#endif /* __AUDIO_CAPTURE_H */
