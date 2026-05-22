#include "bt_hfp.h"
#include "state_machine.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_hf_client_api.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "HFP";

static i2s_chan_handle_t s_tx;
static i2s_chan_handle_t s_rx;
static volatile bool s_connected  = false;
static volatile bool s_sco_active = false;

#define AUDIO_RB_BYTES  (1024 * 4)  // ~160 мс при 8 кГц стерео

static RingbufHandle_t s_spk_rb = NULL;  // BT recv → I2S TX
static RingbufHandle_t s_mic_rb = NULL;  // I2S RX → BT send

// BT callback: кладёт принятое аудио в ring buffer и сразу возвращается
static void recv_data_cb(const uint8_t *buf, uint32_t size) {
    int samples = (int)(size / 2);
    int16_t stereo[samples * 2];
    const int16_t *mono = (const int16_t *)buf;
    for (int i = 0; i < samples; i++) {
        stereo[i * 2]     = mono[i];
        stereo[i * 2 + 1] = mono[i];
    }
    xRingbufferSend(s_spk_rb, stereo, (size_t)samples * 4, 0);  // non-blocking
}

// BT callback: берёт микрофонное аудио из ring buffer и сразу возвращается
static uint32_t send_data_cb(uint8_t *buf, uint32_t size) {
    int mono_samples = (int)(size / 2);
    int16_t *out = (int16_t *)buf;
    size_t item_size;
    void *item = xRingbufferReceiveUpTo(s_mic_rb, &item_size, 0, (size_t)mono_samples * 4);
    if (item) {
        int16_t *stereo = (int16_t *)item;
        int got = (int)(item_size / 4);
        for (int i = 0; i < got; i++)
            out[i] = stereo[i * 2];
        if (got < mono_samples)
            memset(out + got, 0, (size_t)(mono_samples - got) * 2);
        vRingbufferReturnItem(s_mic_rb, item);
    } else {
        memset(buf, 0, size);  // underrun → тишина
    }
    return size;
}

// Аудиомост: блокирующий I2S вынесен из контекста BT-колбэков
static void audio_bridge_task(void *arg) {
    const int STEREO_SAMPLES = 160;  // 20 мс при 8 кГц
    int16_t stereo_buf[STEREO_SAMPLES * 2];
    size_t io_size;

    while (1) {
        if (!s_sco_active) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // Микрофон: I2S RX → ring buffer
        if (i2s_channel_read(s_rx, stereo_buf, sizeof(stereo_buf),
                             &io_size, pdMS_TO_TICKS(25)) == ESP_OK) {
            xRingbufferSend(s_mic_rb, stereo_buf, io_size, 0);
        }

        // Динамик: ring buffer → I2S TX
        void *item = xRingbufferReceiveUpTo(s_spk_rb, &io_size, 0, sizeof(stereo_buf));
        if (item) {
            i2s_channel_write(s_tx, item, io_size, NULL, pdMS_TO_TICKS(25));
            vRingbufferReturnItem(s_spk_rb, item);
        }
    }
}

static void hfp_cb(esp_hf_client_cb_event_t event,
                   esp_hf_client_cb_param_t *param) {
    switch (event) {
    case ESP_HF_CLIENT_CONNECTION_STATE_EVT:
        s_connected = (param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED ||
                       param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED);
        ESP_LOGI(TAG, "conn state: %d", param->conn_stat.state);
        break;

    case ESP_HF_CLIENT_AUDIO_STATE_EVT:
        if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
            param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
            s_sco_active = true;
            sm_dispatch(EVT_SCO_CONNECTED, 0);
        } else {
            s_sco_active = false;
            sm_dispatch(EVT_SCO_CLOSED, 0);
        }
        break;

    case ESP_HF_CLIENT_RING_IND_EVT:
        sm_dispatch(EVT_BT_INCOMING, 0);
        break;

    case ESP_HF_CLIENT_CIND_CALL_EVT:
        if (param->call.status == ESP_HF_CALL_STATUS_NO_CALLS)
            sm_dispatch(EVT_BT_CALL_END, 0);
        else
            sm_dispatch(EVT_BT_ANSWERED, 0);
        break;

    default:
        break;
    }
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    if (event == ESP_BT_GAP_PIN_REQ_EVT) {
        esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
        ESP_LOGI(TAG, "PIN reply sent");
    }
    if (event == ESP_BT_GAP_CFM_REQ_EVT) {
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        ESP_LOGI(TAG, "SSP confirm");
    }
}

bool bt_hfp_is_connected(void) { return s_connected; }

void bt_hfp_reset_pairing(void) {
    ESP_LOGI(TAG, "resetting all pairings");
    int count = esp_bt_gap_get_bond_device_num();
    if (count > 0) {
        esp_bd_addr_t *devs = malloc(count * sizeof(esp_bd_addr_t));
        if (devs) {
            esp_bt_gap_get_bond_device_list(&count, devs);
            for (int i = 0; i < count; i++)
                esp_bt_gap_remove_bond_device(devs[i]);
            free(devs);
        }
    }
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    ESP_LOGI(TAG, "ready for new pairing");
}

void bt_hfp_init(i2s_chan_handle_t tx, i2s_chan_handle_t rx) {
    s_tx = tx;
    s_rx = rx;

    s_spk_rb = xRingbufferCreate(AUDIO_RB_BYTES, RINGBUF_TYPE_BYTEBUF);
    s_mic_rb = xRingbufferCreate(AUDIO_RB_BYTES, RINGBUF_TYPE_BYTEBUF);
    xTaskCreate(audio_bridge_task, "audio_bridge", 4096, NULL, 10, NULL);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_bt_gap_set_device_name("Retro Phone");
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    esp_bt_gap_register_callback(gap_cb);

    esp_bredr_sco_datapath_set(ESP_SCO_DATA_PATH_HCI);
    esp_hf_client_register_data_callback(recv_data_cb, send_data_cb);

    esp_hf_client_register_callback(hfp_cb);
    esp_hf_client_init();
}

void bt_hfp_dial(const char *number) { esp_hf_client_dial(number);  }
void bt_hfp_answer(void)             { esp_hf_client_answer_call(); }
void bt_hfp_hangup(void)             { esp_hf_client_reject_call(); }
