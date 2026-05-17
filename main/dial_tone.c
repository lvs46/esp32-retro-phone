#include "dial_tone.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#define BUF_SAMPLES   160    // 20 мс при 8 кГц
#define BUSY_ON_MS    350
#define BUSY_OFF_MS   350

static i2s_chan_handle_t s_tx;
static TaskHandle_t      s_task    = NULL;
static volatile bool     s_running = false;
static volatile bool     s_busy    = false;

static void write_silence(int ms) {
    int total = SAMPLE_RATE_HZ * ms / 1000 * 2; // стерео int16
    int16_t buf[BUF_SAMPLES * 2];
    memset(buf, 0, sizeof(buf));
    size_t written;
    while (total > 0 && s_running) {
        int chunk = total < BUF_SAMPLES * 2 ? total : BUF_SAMPLES * 2;
        i2s_channel_write(s_tx, buf, chunk * sizeof(int16_t),
                          &written, pdMS_TO_TICKS(100));
        total -= chunk;
    }
}

static void tone_task(void *arg) {
    int16_t buf[BUF_SAMPLES * 2];
    float phase     = 0.0f;
    float phase_inc = 2.0f * (float)M_PI * DIAL_TONE_HZ / SAMPLE_RATE_HZ;
    size_t written;

    while (s_running) {
        // Один буфер тона (20 мс)
        for (int i = 0; i < BUF_SAMPLES; i++) {
            int16_t s      = (int16_t)(14000.0f * sinf(phase));
            buf[i * 2]     = s;
            buf[i * 2 + 1] = s;
            phase += phase_inc;
            if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
        }
        i2s_channel_write(s_tx, buf, sizeof(buf), &written, pdMS_TO_TICKS(100));

        // В режиме "занято" после BUSY_ON_MS вставляем паузу
        if (s_busy) {
            static int on_samples = 0;
            on_samples += BUF_SAMPLES;
            if (on_samples >= SAMPLE_RATE_HZ * BUSY_ON_MS / 1000) {
                on_samples = 0;
                write_silence(BUSY_OFF_MS);
            }
        }
    }

    memset(buf, 0, sizeof(buf));
    i2s_channel_write(s_tx, buf, sizeof(buf), &written, pdMS_TO_TICKS(50));
    s_task = NULL;
    vTaskDelete(NULL);
}

void dial_tone_init(i2s_chan_handle_t tx) { s_tx = tx; }

void dial_tone_start(void) {
    if (s_task) return;
    s_busy    = false;
    s_running = true;
    xTaskCreate(tone_task, "tone", 2048, NULL, 5, &s_task);
}

void busy_tone_start(void) {
    if (s_task) return;
    s_busy    = true;
    s_running = true;
    xTaskCreate(tone_task, "tone", 2048, NULL, 5, &s_task);
}

void dial_tone_stop(void) {
    s_running = false;
    // Ждём завершения задачи, чтобы следующий start() не пропустил флаг s_task
    int timeout = 150;
    while (s_task != NULL && timeout-- > 0)
        vTaskDelay(1);
}
