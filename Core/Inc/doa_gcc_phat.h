#ifndef __DOA_GCC_PHAT_H
#define __DOA_GCC_PHAT_H

#include <stdint.h>

// GCC-PHAT 估计 lag (预留接口)
int32_t doa_estimate_lag_gcc_phat(const int16_t *x, const int16_t *y, uint32_t n, int32_t max_lag);

#endif /* __DOA_GCC_PHAT_H */
