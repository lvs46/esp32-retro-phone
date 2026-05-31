#include "rotary_dial.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <stdatomic.h>
#include <stdio.h>

static const char *TAG = "DIAL";

static void (*s_digit_cb)(uint8_t)  = NULL;
static atomic_int   s_pulse_count   = 0;
static int64_t      s_acc_edge_us   = 0;     // время последнего УСТОЙЧИВОГО фронта
static volatile int s_acc_lvl       = 0;     // последний устойчивый уровень ИК

// Антидребезг: смена уровня засчитывается только если состояние держалось
// дольше, чем дребезг (<5 мс), но меньше реального импульса (34-75 мс)
#define PULSE_DEBOUNCE_MS  15

// DEBUG: длительности устойчивых состояний (мс): +N = разомкнут(HIGH), -N = замкнут(LOW)
#define DBG_MAX 32
static volatile int s_dbg_dt[DBG_MAX];
static volatile int s_dbg_n = 0;
static int64_t      s_active_us     = 0;     // момент подтверждённого замыкания ЗП
static volatile bool s_dial_active  = false; // диск сошёл с покоя
static volatile int  s_zp_level     = 1;     // 1 = ЗП разомкнут (покой)

static esp_timer_handle_t s_zp_tmr;          // дебаунс ЗП + финализация цифры

// Прямой ход диска и старт-глитч ИК: игнорируем импульсы первые N мс
#define DIAL_START_GUARD_MS  120

// Финализация цифры — вызывается из контекста таймера (не ISR)
static void report_digit(void) {
    int count = atomic_exchange(&s_pulse_count, 0);

    // DEBUG: дамп дельт
    int n = s_dbg_n; if (n > DBG_MAX) n = DBG_MAX;
    char buf[128]; int len = 0;
    for (int i = 0; i < n && len < (int)sizeof(buf) - 8; i++)
        len += snprintf(buf + len, sizeof(buf) - len, "%d ", s_dbg_dt[i]);
    s_dbg_n = 0;

    if (count >= 1 && count <= 10) {
        uint8_t digit = (count == 10) ? 0 : (uint8_t)count;
        ESP_LOGI(TAG, "impulses=%d -> digit=%u | dt(ms): %s", count, digit, buf);
        if (s_digit_cb) s_digit_cb(digit);
    } else if (count != 0) {
        ESP_LOGW(TAG, "impulses=%d -> отброшено | dt(ms): %s", count, buf);
    }
}

// Дебаунс ЗП: вызывается через 15 мс после фронта, перечитывает уровень
static void zp_debounce_cb(void *arg) {
    int lvl = gpio_get_level(GPIO_DIAL_ACTIVE);
    if (lvl == s_zp_level) return;
    s_zp_level = lvl;

    if (lvl == 0) {                          // ЗП замкнут → начало набора
        atomic_store(&s_pulse_count, 0);
        s_active_us   = esp_timer_get_time();
        s_acc_edge_us = s_active_us;
        s_acc_lvl     = gpio_get_level(GPIO_DIAL_PULSE); // обычно LOW (контакт замкнут)
        s_dbg_n       = 0;                   // DEBUG
        s_dial_active = true;
    } else {                                 // ЗП разомкнут → диск вернулся
        s_dial_active = false;
        report_digit();
    }
}

// ISR ЗП (синий): любой фронт перезапускает дебаунс-таймер
static void IRAM_ATTR active_isr(void *arg) {
    esp_timer_stop(s_zp_tmr);
    esp_timer_start_once(s_zp_tmr, 15 * 1000ULL);
}

// ISR ИК (жёлтый): ANYEDGE с гистерезисным антидребезгом.
// ИК нормально замкнут (LOW). Импульс = устойчивое размыкание (переход в HIGH).
static void IRAM_ATTR pulse_isr(void *arg) {
    if (!s_dial_active) return;              // вне набора — игнор (старт-глитч до active)

    int64_t now = esp_timer_get_time();
    if (now - s_active_us < DIAL_START_GUARD_MS * 1000LL)
        return;                              // прямой ход диска

    if (now - s_acc_edge_us < PULSE_DEBOUNCE_MS * 1000LL)
        return;                              // слишком рано — дребезг, игнор

    int lvl = gpio_get_level(GPIO_DIAL_PULSE);
    if (lvl == s_acc_lvl) return;            // нет реальной смены уровня

    int64_t state_dur = now - s_acc_edge_us; // длительность устойчивого состояния
    s_acc_lvl     = lvl;
    s_acc_edge_us = now;

    // DEBUG: +N = был разомкнут (HIGH) N мс, -N = был замкнут (LOW) N мс
    int idx = s_dbg_n;
    if (idx < DBG_MAX) {
        s_dbg_dt[idx] = (lvl ? 1 : -1) * (int)(state_dur / 1000);
        s_dbg_n = idx + 1;
    }

    if (lvl == 1)                            // устойчивое размыкание → импульс
        atomic_fetch_add(&s_pulse_count, 1);
}

void rotary_dial_init(void (*digit_cb)(uint8_t digit)) {
    s_digit_cb = digit_cb;

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << GPIO_DIAL_PULSE) | (1ULL << GPIO_DIAL_ACTIVE),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    gpio_set_intr_type(GPIO_DIAL_PULSE,  GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_DIAL_ACTIVE, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(GPIO_DIAL_PULSE,  pulse_isr,  NULL);
    gpio_isr_handler_add(GPIO_DIAL_ACTIVE, active_isr, NULL);

    esp_timer_create_args_t ta = { .callback = zp_debounce_cb, .name = "dial_zp" };
    esp_timer_create(&ta, &s_zp_tmr);
}
