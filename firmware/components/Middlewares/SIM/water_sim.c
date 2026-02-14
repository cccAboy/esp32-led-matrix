#include "water_sim.h"

#include <math.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "flip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gravity.h"
#include "key.h"
#include "rgb.h"

#define W 16
#define H 16
#define LED_COUNT (W * H)

// 1=蛇形走线（每行反向），0=正常行优先
#define SERPENTINE 1

#define SIM_FPS 45

#define LED_VAL_MAX_I 20
#define LED_VAL_MAX_F 20.0f
#define LED_LEVELS (LED_VAL_MAX_I + 1)

static const char* TAG = "water_sim";

#define PAL_N 6

typedef struct {
    uint8_t r, g, b;
} rgb8_t;

static inline rgb8_t lerp_rgb(rgb8_t a, rgb8_t b, float t) {
    if (t < 0.0f)
        t = 0.0f;
    if (t > 1.0f)
        t = 1.0f;
    rgb8_t o;
    o.r = (uint8_t)lrintf((float)a.r + ((float)b.r - (float)a.r) * t);
    o.g = (uint8_t)lrintf((float)a.g + ((float)b.g - (float)a.g) * t);
    o.b = (uint8_t)lrintf((float)a.b + ((float)b.b - (float)a.b) * t);
    return o;
}

#define PALETTE_COUNT 3

static const rgb8_t PALETTES[PALETTE_COUNT][PAL_N] = {
    {
        // palette 0
        {0, 0, 0},
        {102, 255, 255},
        {0, 255, 255},
        {0, 204, 255},
        {0, 153, 255},
        {0, 102, 255},
    },
    {
        // palette 1
        {0, 0, 0},
        {255, 153, 255},
        {255, 102, 255},
        {255, 51, 204},
        {255, 51, 153},
        {255, 0, 102},
    },
    {
        {0, 0, 0},      
        {20, 255, 30},  
        {15, 255, 20},  
        {10, 255, 10},  
        {5, 255, 5},    
        {0, 255, 0},   
    },

};

static rgb8_t s_pal_lut[PALETTE_COUNT][LED_LEVELS];
static uint8_t s_lut_ready[PALETTE_COUNT];

// 构建调色板的 LUT
static void build_palette_lut(uint8_t pal_idx) {
    const rgb8_t* pal = PALETTES[pal_idx];

    for (int lv = 0; lv < LED_LEVELS; lv++) {
        float t = (LED_VAL_MAX_F > 0.0f) ? ((float)lv / LED_VAL_MAX_F)
                                         : 0.0f;  // 0..1
        float p = t * (float)(PAL_N - 1);
        int i0 = (int)p;
        int i1 = (i0 + 1 < PAL_N) ? (i0 + 1) : i0;
        float ft = p - (float)i0;

        rgb8_t c = lerp_rgb(pal[i0], pal[i1], ft);

        uint8_t m = c.r;
        if (c.g > m)
            m = c.g;
        if (c.b > m)
            m = c.b;

        if (m > 0) {
            float k = (float)lv / (float)m;
            c.r = (uint8_t)lrintf((float)c.r * k);
            c.g = (uint8_t)lrintf((float)c.g * k);
            c.b = (uint8_t)lrintf((float)c.b * k);
        } else {
            c.r = c.g = c.b = 0;
        }

        s_pal_lut[pal_idx][lv] = c;
    }

    s_lut_ready[pal_idx] = 1;
}

#define KEY_GPIO_PIN GPIO_NUM_0
#define KEY_DEBOUNCE_MS 50

static uint8_t s_palette_idx = 0;
static key_t s_key;

static inline int led_index_row_major(int x, int y) {
#if SERPENTINE
    if (y & 1)
        return y * W + (W - 1 - x);
    else
        return y * W + x;
#else
    return y * W + x;
#endif
}


static inline float grid_get(const float* grid, int x, int y) {
    return grid[x * H + y];
}

static void sim_task(void* arg) {
    (void)arg;

    // 1) 初始化灯带，和按键
    rgb_init();
    ESP_ERROR_CHECK(
        key_init(&s_key, KEY_GPIO_PIN, true /*active_low*/, KEY_DEBOUNCE_MS));
    for (int i = 0; i < PALETTE_COUNT; i++) build_palette_lut(i);

    // 2) 创建 FLIP
    FlipFluid* f = flip_create(1.0f, 1.0f, W, 0.6f);
    if (!f) {
        ESP_LOGE(TAG, "flip_create failed");
        vTaskDelete(NULL);
        return;
    }
    // 设定重力加速度:地球 9.81  月球约 1.62  火星约 3.71
    flip_set_gravity_scale(f, 9.81f);

    float grid[LED_COUNT];

    const TickType_t frame_ticks = pdMS_TO_TICKS(1000 / SIM_FPS);
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, frame_ticks);

        float dt = 1.0f / (float)SIM_FPS;

        gravity_xy_t g = gravity_get();
        float gx = g.valid ? g.gx : 0.0f;
        float gy = g.valid ? g.gy : 0.0f;

        flip_step(f, dt, gx, gy);
        flip_get_led_grid(f, grid);
        if (key_get_press(&s_key)) {
            s_palette_idx = (uint8_t)((s_palette_idx + 1) % PALETTE_COUNT);
            if (!s_lut_ready[s_palette_idx])
                build_palette_lut(s_palette_idx);
            ESP_LOGI(TAG, "palette -> %u", (unsigned)s_palette_idx);
        }

        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                float v = grid_get(grid, x, y);
                if (!isfinite(v))
                    v = 0.0f;
                if (v < 0.0f)
                    v = 0.0f;
                if (v > LED_VAL_MAX_F)
                    v = LED_VAL_MAX_F;

                int lv = (int)(v + 0.5f);
                if (lv < 0)
                    lv = 0;
                if (lv >= LED_LEVELS)
                    lv = LED_LEVELS - 1;

                rgb8_t c = s_pal_lut[s_palette_idx][lv];

                int idx = led_index_row_major(x, y);
                rgb_set((uint32_t)idx, c.r, c.g, c.b);

                // 开始的蓝色显示方案
                //  uint8_t b = (uint8_t)lrintf(v);
                //  uint8_t gg = (uint8_t)lrintf(v * (220.0f / 255.0f));

                // int idx = led_index_row_major(x, y);
                // rgb_set((uint32_t)idx, 0, gg, b);
            }
        }
        rgb_show();
    }
}

void water_sim_start(int core_id, uint32_t stack_size, int priority) {
    xTaskCreatePinnedToCore(sim_task, "water_sim", stack_size, NULL, priority,
                            NULL, core_id);
}
