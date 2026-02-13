#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

typedef struct {
    gpio_num_t pin;
    bool active_low;
    volatile uint8_t pressed;     // ISR 置 1，任务里读取后清 0
    TickType_t last_tick;         // 去抖用
    TickType_t debounce_ticks;    // 去抖阈值（tick）
} key_t;

/**
 * @brief 初始化一个按键（支持多个实例）
 * @param key          key_t 对象指针
 * @param pin          GPIO
 * @param active_low   true=按下为低电平（常见：上拉+按下接地）
 * @param debounce_ms  去抖毫秒，例如 50
 */
esp_err_t key_init(key_t* key, gpio_num_t pin, bool active_low, uint32_t debounce_ms);

/**
 * @brief 查询是否发生“按下事件”（有则返回 true 并清除事件）
 */
bool key_get_press(key_t* key);
