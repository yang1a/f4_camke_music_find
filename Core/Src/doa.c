#include "doa.h"
#include "doa_ncc.h"

/**
 * @brief DOA 估计接口 - 当前使用 NCC 方法
 */
int32_t doa_estimate_lag(const int16_t *x, const int16_t *y, uint32_t n, int32_t max_lag)
{
  return doa_estimate_lag_ncc(x, y, n, max_lag);
}
