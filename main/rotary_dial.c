#include "rotary_dial.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include <stdatomic.h>

static void (*s_digit_cb)(uint8_t)  = NULL;
static atomic_int   s_pulse_count   = 0;
static int64_t      s_last_pulse_us = 0;
static esp_timer_handle_t s_digit_tmr;

// Таймаут истёк — цифра готова
static void digit_timeout_cb(void *arg) {
    int count = atomic_exchange(&s_pulse_count, 0);
    if (count >= 1 && count <= 10) {
        uint8_t digit = (count == 10) ? 0 : (uint8_t)count;
        if (s_digit_cb) s_digit_cb(digit);
    }
}

// ISR: каждый размыкающий импульс диска
static void IRAM_ATTR pulse_isr(void *arg) {
    int64_t now   = esp_timer_get_time();
    int64_t delta = now - s_last_pulse_us;
    s_last_pulse_us = now;

    if (delta < PULSE_MIN_MS * 1000LL) return; // дребезг

    atomic_fetch_add(&s_pulse_count, 1);

    esp_timer_stop(s_digit_tmr);
    esp_timer_start_once(s_digit_tmr, DIGIT_TIMEOUT_MS * 1000ULL);
}

void rotary_dial_init(void (*digit_cb)(uint8_t digit)) {
    s_digit_cb = digit_cb;

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << GPIO_DIAL_PULSE),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    gpio_set_intr_type(GPIO_DIAL_PULSE, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(GPIO_DIAL_PULSE, pulse_isr, NULL);

    esp_timer_create_args_t ta = { .callback = digit_timeout_cb, .name = "dial" };
    esp_timer_create(&ta, &s_digit_tmr);
}
