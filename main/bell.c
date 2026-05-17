#include "bell.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <stdbool.h>

static esp_timer_handle_t s_toggle_tmr;
static esp_timer_handle_t s_pattern_tmr;
static volatile bool s_ringing  = false;
static volatile bool s_on_phase = false;
static volatile bool s_coil     = false;

// 25 Гц: переключаем катушки каждые 20 мс
static void toggle_cb(void *arg) {
    s_coil = !s_coil;
    gpio_set_level(GPIO_BELL_1,  s_coil);
    gpio_set_level(GPIO_BELL_2, !s_coil);
}

static void pattern_cb(void *arg) {
    if (s_on_phase) {
        // конец фазы ON -> выключить катушки и буст
        esp_timer_stop(s_toggle_tmr);
        gpio_set_level(GPIO_BELL_1, 0);
        gpio_set_level(GPIO_BELL_2, 0);
        gpio_set_level(GPIO_BOOST_EN, 0);  // Fix 7: буст отключается в паузе
        s_on_phase = false;
        if (s_ringing)
            esp_timer_start_once(s_pattern_tmr, BELL_OFF_MS * 1000ULL);
    } else {
        // конец фазы OFF -> включить буст и катушки
        gpio_set_level(GPIO_BOOST_EN, 1);  // Fix 7: буст включается перед звонком
        s_on_phase = true;
        s_coil = false;
        esp_timer_start_periodic(s_toggle_tmr, 1000000ULL / (BELL_FREQ_HZ * 2));
        if (s_ringing)
            esp_timer_start_once(s_pattern_tmr, BELL_ON_MS * 1000ULL);
    }
}

void bell_init(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << GPIO_BELL_1) | (1ULL << GPIO_BELL_2),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cfg);
    gpio_set_level(GPIO_BELL_1, 0);
    gpio_set_level(GPIO_BELL_2, 0);

    gpio_reset_pin(GPIO_BOOST_EN);
    gpio_set_direction(GPIO_BOOST_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_BOOST_EN, 0);

    esp_timer_create_args_t ta = { .callback = toggle_cb,  .name = "bell_tog" };
    esp_timer_create(&ta, &s_toggle_tmr);
    esp_timer_create_args_t pa = { .callback = pattern_cb, .name = "bell_pat" };
    esp_timer_create(&pa, &s_pattern_tmr);
}

void bell_start(void) {
    s_ringing  = true;
    s_on_phase = true;
    gpio_set_level(GPIO_BOOST_EN, 1);
    s_coil = false;
    esp_timer_start_periodic(s_toggle_tmr, 1000000ULL / (BELL_FREQ_HZ * 2));
    esp_timer_start_once(s_pattern_tmr, BELL_ON_MS * 1000ULL);
}

void bell_stop(void) {
    s_ringing = false;
    esp_timer_stop(s_toggle_tmr);
    esp_timer_stop(s_pattern_tmr);
    gpio_set_level(GPIO_BELL_1, 0);
    gpio_set_level(GPIO_BELL_2, 0);
    gpio_set_level(GPIO_BOOST_EN, 0);
}
