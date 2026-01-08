#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>
#include "app.h"

// ======================= 舵机映射参数 =======================
#define SERVO_US_MIN 600
#define SERVO_US_MAX 2500
#define SERVO_US_CENTER ((SERVO_US_MIN + SERVO_US_MAX) / 2)

// 满量程增益映射
#define SERVO_K_US_PER_LAG (((float)(SERVO_US_MAX - SERVO_US_MIN)) / (2.0f * (float)MAX_LAG_SAMPLES))

// 死区
#define LAG_DEADBAND 1

// 一阶滤波系数
#define SERVO_ALPHA 0.15f

// 舵机控制函数
void servo_init(void);
void servo_write_us(int us);
void servo_track_from_lag(int32_t lag, uint8_t valid);
int servo_get_current_us(void);

#endif /* __SERVO_H */
