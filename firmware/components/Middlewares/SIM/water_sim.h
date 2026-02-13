#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 启动
void water_sim_start(int core_id, uint32_t stack_size, int priority);

#ifdef __cplusplus
}
#endif
