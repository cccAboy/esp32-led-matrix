#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gravity.h"
#include "mpu6050_gravity.h"

#include "water_sim.h"


typedef enum {
    SIM_WATER = 0,
    SIM_FIRE  = 1,
} sim_mode_t;

static const sim_mode_t g_sim_mode = SIM_WATER;

void app_main(void) {

    gravity_init();


    mpu6050_gravity_start();

    if (g_sim_mode == SIM_WATER) {
        water_sim_start(/*core_id=*/1, /*stack=*/8192, /*prio=*/5);
    } else {
        // 以后其他模拟模式
    }
}
