#ifndef __DOA_H
#define __DOA_H

#include <stdint.h>

// DOA 估计接口
int32_t doa_estimate_lag(const int16_t *x, const int16_t *y, uint32_t n, int32_t max_lag);

#endif /* __DOA_H */
