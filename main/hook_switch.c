#include "hook_switch.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"

static void (*s_cb)(bool)       = NULL;
static volatile int s_last_lvl  = 1;   // 1 = трубка лежит (NC замкнут)
static esp_timer_handle_t s_tmr;

static void debounce_cb(void *arg) {
    int lvl = gpio_get_level(GPIO_HOOK_SWITCH);
    if (lvl == s_last_lvl) return;
    s_last_lvl = lvl;
    if (s_cb) s_cb(lvl == 0); // LOW = трубка поднята
}

static void IRAM_ATTR hook_isr(void *arg) {
    esp_timer_stop(s_tmr);
    esp_timer_start_once(s_tmr, 50 * 1000ULL); // 50 мс антидребезг
}

void hook_switch_init(void (*cb)(bool off_hook)) {
    s_cb = cb;
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << GPIO_HOOK_SWITCH,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&cfg);

    esp_timer_create_args_t ta = { .callback = debounce_cb, .name = "hook" };
    esp_timer_create(&ta, &s_tmr);
    gpio_isr_handler_add(GPIO_HOOK_SWITCH, hook_isr, NULL);
}
