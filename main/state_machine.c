#include "state_machine.h"
#include "bell.h"
#include "dial_tone.h"
#include "bt_hfp.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

#define DIAL_COMPLETE_MS  3000

static const char *TAG = "SM";

static phone_state_t      s_state = STATE_IDLE;
static char               s_number[16];
static int                s_num_len;
static esp_timer_handle_t s_dial_timer;
static int                s_idle_nines = 0;   // подряд набранные «9» при положенной трубке
static int                s_idle_zeros = 0;   // подряд набранные «0» при положенной трубке

// Sweep блокирующий (~20с) — запускаем в отдельной задаче, не в контексте диспетчера
static void sweep_task(void *arg) {
    bell_sweep_test();
    vTaskDelete(NULL);
}

static void dial_complete_cb(void *arg) {
    ESP_LOGI(TAG, "dial complete: %s", s_number);
    sm_dispatch(EVT_DIAL_TIMEOUT, 0);
}

static void confirm_reset(void) {
    bell_start();
    vTaskDelay(pdMS_TO_TICKS(300));
    bell_stop();
}

static void dial_timer_restart(void) {
    esp_timer_stop(s_dial_timer);
    esp_timer_start_once(s_dial_timer, DIAL_COMPLETE_MS * 1000ULL);
}

phone_state_t sm_state(void) { return s_state; }

static void enter(phone_state_t next) {
    ESP_LOGI(TAG, "%d -> %d", s_state, next);

    // Гасим таймер при любом выходе из STATE_DIALING — до обновления s_state
    if (s_state == STATE_DIALING)
        esp_timer_stop(s_dial_timer);

    s_state = next;

    switch (next) {
    case STATE_IDLE:
        dial_tone_stop();
        bell_stop();
        break;
    case STATE_DIAL_TONE:
        s_num_len = 0;
        memset(s_number, 0, sizeof(s_number));
        if (bt_hfp_is_connected())
            dial_tone_start();
        else
            busy_tone_start();
        break;
    case STATE_DIALING:
        dial_tone_stop();
        break;
    case STATE_OUTGOING:
        bt_hfp_dial(s_number);
        break;
    case STATE_RINGING:
        bell_start();
        break;
    case STATE_IN_CALL:
        bell_stop();
        break;
    }
}

void sm_init(void) {
    s_state = STATE_IDLE;
    esp_timer_create_args_t ta = { .callback = dial_complete_cb, .name = "dial_done" };
    esp_timer_create(&ta, &s_dial_timer);
}

void sm_dispatch(phone_event_t evt, uint32_t data) {
    switch (s_state) {

    case STATE_IDLE:
        if (evt == EVT_HOOK_OFF)    { s_idle_nines = 0; s_idle_zeros = 0; enter(STATE_DIAL_TONE); }
        if (evt == EVT_BT_INCOMING) enter(STATE_RINGING);
        // Сервис-коды при положенной трубке:
        //   999 → sweep-тест звонка
        //   000 → сброс всех pairing
        if (evt == EVT_DIGIT) {
            if (data == 9) {
                s_idle_zeros = 0;
                if (++s_idle_nines >= 3) {
                    s_idle_nines = 0;
                    ESP_LOGI(TAG, "service 999 -> bell sweep");
                    xTaskCreate(sweep_task, "bell_sweep", 3072, NULL, 5, NULL);
                }
            } else if (data == 0) {
                s_idle_nines = 0;
                if (++s_idle_zeros >= 3) {
                    s_idle_zeros = 0;
                    ESP_LOGI(TAG, "service 000 -> reset pairing");
                    bt_hfp_reset_pairing();
                    confirm_reset();
                }
            } else {
                s_idle_nines = 0;
                s_idle_zeros = 0;
            }
        }
        break;

    case STATE_DIAL_TONE:
        if (evt == EVT_HOOK_ON)     enter(STATE_IDLE);
        if (evt == EVT_BT_INCOMING) enter(STATE_RINGING);
        // Fix 2: набор разрешён только при активном BT
        if (evt == EVT_DIGIT && bt_hfp_is_connected()) {
            s_number[s_num_len++] = '0' + (char)data;
            enter(STATE_DIALING);
        }
        break;

    case STATE_DIALING:
        if (evt == EVT_HOOK_ON) enter(STATE_IDLE);
        if (evt == EVT_DIGIT) {
            s_number[s_num_len++] = '0' + (char)data;

            if (s_num_len == 3 && strcmp(s_number, "000") == 0
                    && !bt_hfp_is_connected()) {
                bt_hfp_reset_pairing();
                confirm_reset();
                enter(STATE_IDLE);
                break;
            }

            dial_timer_restart();
        }
        if (evt == EVT_DIAL_TIMEOUT) enter(STATE_OUTGOING);
        break;

    case STATE_OUTGOING:
        if (evt == EVT_HOOK_ON)     { bt_hfp_hangup(); enter(STATE_IDLE); }
        if (evt == EVT_BT_ANSWERED) enter(STATE_IN_CALL);
        if (evt == EVT_BT_CALL_END) enter(STATE_DIAL_TONE);
        break;

    case STATE_RINGING:
        if (evt == EVT_HOOK_OFF)    { bt_hfp_answer(); enter(STATE_IN_CALL); }
        if (evt == EVT_BT_ANSWERED) enter(STATE_IN_CALL);   // ответили на смартфоне
        if (evt == EVT_BT_CALL_END) enter(STATE_IDLE);
        break;

    case STATE_IN_CALL:
        // Fix 1: после завершения звонка -> STATE_IDLE, трубка лежит
        if (evt == EVT_HOOK_ON)     { bt_hfp_hangup(); enter(STATE_IDLE); }
        if (evt == EVT_BT_CALL_END) enter(STATE_IDLE);
        break;
    }
}
