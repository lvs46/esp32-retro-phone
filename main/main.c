#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "config.h"
#include "state_machine.h"
#include "audio.h"
#include "bell.h"
#include "rotary_dial.h"
#include "hook_switch.h"
#include "dial_tone.h"
#include "bt_hfp.h"

static i2s_chan_handle_t s_tx, s_rx;

static void on_hook(bool off_hook) {
    sm_dispatch(off_hook ? EVT_HOOK_OFF : EVT_HOOK_ON, 0);
}

static void on_digit(uint8_t digit) {
    sm_dispatch(EVT_DIGIT, digit);
}

void app_main(void) {
    nvs_flash_init();

    sm_init();
    audio_init(&s_tx, &s_rx);
    dial_tone_init(s_tx);
    bell_init();

    // ISR сервис — один раз для всех GPIO прерываний
    gpio_install_isr_service(0);

    hook_switch_init(on_hook);
    rotary_dial_init(on_digit);
    bt_hfp_init(s_tx, s_rx);

    ESP_LOGI("MAIN", "Retro Phone ready");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
