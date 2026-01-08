#include "servo.h"
#include "tim.h"
#include "main.h"
#include <stdint.h>

// 舵机滤波后的当前输出
static float servo_us_filt = (float)SERVO_US_CENTER;

/**
 * @brief 工具函数 - 整数限幅
 */
static inline int clamp_int(int v, int lo, int hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

/**
 * @brief 工具函数 - 浮点数限幅
 */
static inline float clamp_f(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

/**
 * @brief 初始化舵机
 */
void servo_init(void)
{
    // 启动舵机 PWM(TIM3_CH1 PA6)
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    servo_write_us(SERVO_US_CENTER);
}

/**
 * @brief 写入舵机 PWM 值
 */
void servo_write_us(int us)
{
    us = clamp_int(us, SERVO_US_MIN, SERVO_US_MAX);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)us);
}

/**
 * @brief lag -> 舵机 PWM(满量程映射 + 死区 + 滤波)
 * valid=0 时不更新(保持)
 */
void servo_track_from_lag(int32_t lag, uint8_t valid)
{
    if (!valid)
        return;

    // 物理限幅(防误峰)
    if (lag > MAX_LAG_SAMPLES)
        lag = MAX_LAG_SAMPLES;
    if (lag < -MAX_LAG_SAMPLES)
        lag = -MAX_LAG_SAMPLES;

    // 死区
    if (lag >= -LAG_DEADBAND && lag <= LAG_DEADBAND)
        lag = 0;

    // 满量程映射:中心 + lag*K
    float target = (float)SERVO_US_CENTER + (float)lag * SERVO_K_US_PER_LAG;
    target = clamp_f(target, (float)SERVO_US_MIN, (float)SERVO_US_MAX);

    // 一阶低通滤波
    servo_us_filt = servo_us_filt + SERVO_ALPHA * (target - servo_us_filt);

    servo_write_us((int)(servo_us_filt + 0.5f));
}

/**
 * @brief 获取当前舵机输出值
 */
int servo_get_current_us(void)
{
    return (int)(servo_us_filt + 0.5f);
}
