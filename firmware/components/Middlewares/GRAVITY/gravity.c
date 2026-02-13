#include "gravity.h"
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
static float s_gx = 0.0f;
static float s_gy = 0.0f;
static bool s_valid = false;

static portMUX_TYPE s_gravity_mux = portMUX_INITIALIZER_UNLOCKED;

void gravity_init(void) {
    portENTER_CRITICAL(&s_gravity_mux);
    s_gx = 0.0f;
    s_gy = 0.0f;
    s_valid = false;
    portEXIT_CRITICAL(&s_gravity_mux);
}

void gravity_set(float gx, float gy)
{
    portENTER_CRITICAL(&s_gravity_mux);
    s_gx = gx;
    s_gy = gy;
    s_valid = true;
    portEXIT_CRITICAL(&s_gravity_mux);
}

gravity_xy_t gravity_get(void)
{
    gravity_xy_t out;

    portENTER_CRITICAL(&s_gravity_mux);
    out.gx = s_gx;
    out.gy = s_gy;
    out.valid = s_valid;
    portEXIT_CRITICAL(&s_gravity_mux);

    return out;
}

bool gravity_is_valid(void)
{
    bool v;
    portENTER_CRITICAL(&s_gravity_mux);
    v = s_valid;
    portEXIT_CRITICAL(&s_gravity_mux);
    return v;
}
