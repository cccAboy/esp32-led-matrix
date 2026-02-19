#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct FlipFluid FlipFluid;

FlipFluid* flip_create(float sim_w, float sim_h, int visible_res,
                       float fill_ratio);
void flip_destroy(FlipFluid* f);

void flip_step(FlipFluid* f, float dt, float gx, float gy);

void flip_get_led_grid(const FlipFluid* f, float* out_grid);

void flip_set_gravity_scale(FlipFluid* f, float gravity_scale);
void flip_set_solver_quality(FlipFluid* f, int push_iters, int pressure_iters,
                             float flip_ratio);

#ifdef __cplusplus
}
#endif
