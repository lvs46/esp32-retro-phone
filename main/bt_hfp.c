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
static volatile bool s_connected       = false;
static volatile bool s_sco_active      = false;
static volatile bool s_want_audio      = false;  // запрос SCO из колбека
static volatile bool s_want_hfp        = false;  // запрос активного HFP-коннекта после ACL
static esp_bd_addr_t s_peer_bda        = {0};

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

    bool  armed     = false;
    int   countdown = 0;
    int   tries     = 0;

    while (1) {
        // ACL поднялся → ждём 1.5с (вдруг телефон сам поднимет HFP),
        // и если не поднял — инициируем сами поверх уже живого ACL.
        if (s_want_hfp && !s_connected) {
            s_want_hfp = false;
            for (int i = 0; i < 75 && !s_connected; i++)
                vTaskDelay(pdMS_TO_TICKS(20));
            if (!s_connected) {
                esp_err_t r = esp_hf_client_connect(s_peer_bda);
                ESP_LOGI(TAG, "active HFP connect: %d", r);
            }
        }


        if (s_want_audio && !s_sco_active) {
            if (!armed) {
                armed     = true;
                countdown = 50;   // 50 × 20 мс = 1 с начальная задержка
                tries     = 0;
            }
            if (countdown > 0) {
                countdown--;
            } else if (tries < 10) {
                tries++;
                countdown = 250;  // 250 × 20 мс = 5 с между повторами
                // Сброс зависшего CONNECTING перед новой попыткой
                esp_hf_client_disconnect_audio(s_peer_bda);
                esp_err_t r = esp_hf_client_connect_audio(s_peer_bda);
                ESP_LOGI(TAG, "connect_audio #%d: %d", tries, r);
            }
        } else {
            armed = false; countdown = 0; tries = 0;
        }

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
        if (s_connected) {
            memcpy(s_peer_bda, param->conn_stat.remote_bda, sizeof(esp_bd_addr_t));
            s_want_hfp = false;                  // уже подключены — отменить активную попытку
        }
        if (param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED) {
            // На всякий случай повторно объявимся как connectable + discoverable,
            // чтобы телефон мог нас найти и переподключиться
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        }
        ESP_LOGI(TAG, "conn state: %d", param->conn_stat.state);
        break;

    case ESP_HF_CLIENT_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "audio_state: %d", param->audio_stat.state);
        if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
            param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
            s_sco_active = true;
            s_want_audio = false;
            sm_dispatch(EVT_SCO_CONNECTED, 0);
        } else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTING) {
            // ждём CONNECTED или DISCONNECTED, не сбрасываем s_want_audio
        } else {
            s_sco_active = false;
            sm_dispatch(EVT_SCO_CLOSED, 0);
        }
        break;

    case ESP_HF_CLIENT_RING_IND_EVT:
        sm_dispatch(EVT_BT_INCOMING, 0);
        break;

    case ESP_HF_CLIENT_CIND_CALL_EVT:
        if (param->call.status == ESP_HF_CALL_STATUS_NO_CALLS) {
            sm_dispatch(EVT_BT_CALL_END, 0);
            s_want_audio = false;
        } else {
            sm_dispatch(EVT_BT_ANSWERED, 0);
            if (!s_sco_active)
                s_want_audio = true;  // audio_bridge_task вызовет connect_audio
        }
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
    // Телефон поднял ACL → активно инициируем HFP поверх живого ACL.
    // Без этого Bluedroid через 4с разорвёт ACL по idle-timeout, т.к. HFP-сервис
    // телефон сам не открывает на переподключении.
    if (event == ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT) {
        if (param->acl_conn_cmpl_stat.stat == ESP_BT_STATUS_SUCCESS && !s_connected) {
            memcpy(s_peer_bda, param->acl_conn_cmpl_stat.bda, sizeof(esp_bd_addr_t));
            s_want_hfp = true;
            ESP_LOGI(TAG, "ACL up — arming HFP connect");
        }
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

// На старте через 5с пробуем сами поднять связь с парным телефоном
// (как делают настоящие гарнитуры при включении).
static void boot_reconnect_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    if (s_connected) { vTaskDelete(NULL); return; }

    int count = esp_bt_gap_get_bond_device_num();
    if (count <= 0) { vTaskDelete(NULL); return; }

    esp_bd_addr_t *devs = malloc(count * sizeof(esp_bd_addr_t));
    if (!devs) { vTaskDelete(NULL); return; }

    esp_bt_gap_get_bond_device_list(&count, devs);
    if (count > 0) {
        memcpy(s_peer_bda, devs[0], sizeof(esp_bd_addr_t));
        esp_err_t r = esp_hf_client_connect(s_peer_bda);
        ESP_LOGI(TAG, "boot reconnect: %d", r);
    }
    free(devs);
    vTaskDelete(NULL);
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

    // Class of Device: Audio/Video → Hands-Free Unit + Audio service.
    // Без этого Android видит ESP32 как «Uncategorized» и не делает HFP-автоконнект.
    esp_bt_cod_t cod = {0};
    cod.minor   = 0x02;   // Hands-Free
    cod.major   = 0x04;   // Audio/Video
    cod.service = 0x100;  // Audio
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_ALL);

    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    esp_bt_gap_register_callback(gap_cb);

    esp_bredr_sco_datapath_set(ESP_SCO_DATA_PATH_HCI);
    esp_hf_client_register_data_callback(recv_data_cb, send_data_cb);

    esp_hf_client_register_callback(hfp_cb);
    esp_hf_client_init();

    int bonded = esp_bt_gap_get_bond_device_num();
    ESP_LOGI(TAG, "bonded devices in NVS: %d (waiting for phone)", bonded);

    // Boot-reconnect: через 5с после старта пробуем сами достучаться до
    // последнего парного телефона. Если он рядом — поднимется ACL и HFP.
    if (bonded > 0)
        xTaskCreate(boot_reconnect_task, "bt_boot", 3072, NULL, 5, NULL);
}

void bt_hfp_dial(const char *number)  { esp_hf_client_dial(number);          }
void bt_hfp_answer(void)              { esp_hf_client_answer_call();          }
void bt_hfp_hangup(void)              { esp_hf_client_reject_call();          }
