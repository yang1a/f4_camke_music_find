#ifndef __DOA_NCC_H
#define __DOA_NCC_H

#include <stdint.h>

// NCC 归一化互相关估计 lag
int32_t doa_estimate_lag_ncc(const int16_t *x, const int16_t *y, uint32_t n, int32_t max_lag);

#endif /* __DOA_NCC_H */
