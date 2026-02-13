#include "rgb.h"
#include "esp_err.h"
#include "esp_check.h"   
#include "led_strip.h"
#include "esp_rom_sys.h"
#define RGB_GPIO_PIN 10
#define RGB_LED_COUNT 256
#define RGB_RMT_RES_HZ (10 * 1000 * 1000)
#define CLAMP_MAX   20

static led_strip_handle_t s_strip = NULL;

void rgb_init(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_GPIO_PIN,
        .max_leds = RGB_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        }};

    led_strip_rmt_config_t rmt_config = {.clk_src = RMT_CLK_SRC_DEFAULT,
                                         .resolution_hz = RGB_RMT_RES_HZ,
                                         .mem_block_symbols = 64,
                                         .flags = {
                                             .with_dma = true,
                                         }};

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip)); 
    ESP_ERROR_CHECK(led_strip_clear(s_strip));
    ESP_ERROR_CHECK(led_strip_refresh(s_strip));
}

void rgb_set(uint32_t index,uint8_t r, uint8_t g, uint8_t b) {
    if (!s_strip || index >= RGB_LED_COUNT)
        return;

    uint8_t m = r;
    if (g > m) m = g;
    if (b > m) m = b;
//如果亮度超过CLAMP_MAX，截断（保底机制，在上一层已经放缩过了）
    if (m > CLAMP_MAX) {
        r = (uint16_t)r * CLAMP_MAX / m;
        g = (uint16_t)g * CLAMP_MAX / m;
        b = (uint16_t)b * CLAMP_MAX / m;
    }

    led_strip_set_pixel(s_strip, index, r, g, b);
}

void rgb_clear(void) {
    if (!s_strip)
        return;
    led_strip_clear(s_strip);
}

void rgb_set_hsv(uint32_t index, uint16_t hue,uint16_t light)
{
    if (!s_strip) return;
    if (index >= RGB_LED_COUNT) return;
    hue %= 360; // 0..359
    light = (light > CLAMP_MAX) ? CLAMP_MAX : light;
    led_strip_set_pixel_hsv(s_strip, index, hue, 255, light);
}

void rgb_show(void)
{
    if (!s_strip) return;
    esp_err_t err = led_strip_refresh(s_strip);
    if (err != ESP_OK) {
        // 不要直接 abort，先记日志/重试一次
        // ESP_LOGW("rgb", "refresh failed: %s", esp_err_to_name(err));
        led_strip_refresh(s_strip);
    }
    esp_rom_delay_us(80); 
}
