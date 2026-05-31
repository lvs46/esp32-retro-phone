#include "bell.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

static esp_timer_handle_t s_toggle_tmr;
static esp_timer_handle_t s_pattern_tmr;
static volatile bool s_ringing  = false;
static volatile bool s_on_phase = false;
static volatile bool s_coil     = false;

// Катушка обесточена: оба входа моста LOW (L293D — торможение, тока нет)
static inline void coil_off(void) {
    gpio_set_level(GPIO_BELL_IN1, 0);
    gpio_set_level(GPIO_BELL_IN2, 0);
}

// 25 Гц: переменный ток через катушку — IN1/IN2 в противофазе (H-мост L293D)
static void toggle_cb(void *arg) {
    s_coil = !s_coil;
    gpio_set_level(GPIO_BELL_IN1,  s_coil);
    gpio_set_level(GPIO_BELL_IN2, !s_coil);
}

static void pattern_cb(void *arg) {
    if (s_on_phase) {
        // конец фазы ON -> обесточить катушку и буст
        esp_timer_stop(s_toggle_tmr);
        coil_off();
        gpio_set_level(GPIO_BOOST_EN, 0);
        s_on_phase = false;
        if (s_ringing)
            esp_timer_start_once(s_pattern_tmr, BELL_OFF_MS * 1000ULL);
    } else {
        // конец фазы OFF -> включить буст и переменку
        gpio_set_level(GPIO_BOOST_EN, 1);
        s_on_phase = true;
        s_coil = false;
        esp_timer_start_periodic(s_toggle_tmr, 1000000ULL / (BELL_FREQ_HZ * 2));
        if (s_ringing)
            esp_timer_start_once(s_pattern_tmr, BELL_ON_MS * 1000ULL);
    }
}

void bell_init(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << GPIO_BELL_IN1) | (1ULL << GPIO_BELL_IN2),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cfg);
    coil_off();

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
    s_coil     = false;
    gpio_set_level(GPIO_BOOST_EN, 1);
    esp_timer_start_periodic(s_toggle_tmr, 1000000ULL / (BELL_FREQ_HZ * 2));
    esp_timer_start_once(s_pattern_tmr, BELL_ON_MS * 1000ULL);
}

void bell_stop(void) {
    s_ringing = false;
    esp_timer_stop(s_toggle_tmr);
    esp_timer_stop(s_pattern_tmr);
    coil_off();
    gpio_set_level(GPIO_BOOST_EN, 0);
}

// DEBUG: звенит ~2 с на каждой частоте из списка, между ними пауза.
// Слушаем где громче всего — это механический резонанс якоря.
void bell_sweep_test(void) {
    static const int freqs[] = { 10, 12, 14, 16, 18, 20, 22, 24, 26, 28 };
    const int n = sizeof(freqs) / sizeof(freqs[0]);

    for (int i = 0; i < n; i++) {
        int hz      = freqs[i];
        int half_ms = 1000 / (hz * 2);     // полупериод
        int toggles = 4 * hz;              // ~2 секунды звона

        ESP_LOGW("BELL", ">>> TEST %d Hz <<<", hz);
        gpio_set_level(GPIO_BOOST_EN, 1);

        int lvl = 0;
        for (int t = 0; t < toggles; t++) {
            lvl = !lvl;
            gpio_set_level(GPIO_BELL_IN1,  lvl);
            gpio_set_level(GPIO_BELL_IN2, !lvl);
            vTaskDelay(pdMS_TO_TICKS(half_ms));
        }

        coil_off();
        gpio_set_level(GPIO_BOOST_EN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));   // пауза между частотами
    }
    ESP_LOGW("BELL", ">>> SWEEP DONE <<<");
}
