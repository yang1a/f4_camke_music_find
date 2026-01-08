#include "doa_ncc.h"
#include <math.h>
#include <stdint.h>

/**
 * @brief NCC 归一化互相关估计 lag(抗 MAX9814 AGC 更稳)
 */
int32_t doa_estimate_lag_ncc(const int16_t *x, const int16_t *y, uint32_t n, int32_t max_lag)
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
