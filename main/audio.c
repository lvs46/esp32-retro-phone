#include "audio.h"
#include "config.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "WM8960";

// WM8960: 7-бит адрес регистра + 9-бит данные -> 2 байта по I2C
static esp_err_t wm_write(uint8_t reg, uint16_t val) {
    uint8_t buf[2] = {
        (uint8_t)((reg << 1) | ((val >> 8) & 0x01)),
        (uint8_t)(val & 0xFF)
    };
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (WM8960_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "reg 0x%02X write failed: %d", reg, ret);
    return ret;
}

static void mclk_init(void) {
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = SAMPLE_RATE_HZ * 256, // 2.048 МГц
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .gpio_num   = GPIO_I2S_MCLK,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 1, // 1-bit resolution: duty=1 → 50%
        .hpoint     = 0,
    };
    ledc_channel_config(&ch);
}

static void wm8960_init(void) {
    wm_write(15, 0x000);  // программный сброс
    vTaskDelay(pdMS_TO_TICKS(50)); // Fix 5: ждём завершения сброса

    // Power Management 1: VMIDSEL=01, VREF, AINL, ADCL, MICB
    wm_write(25, 0x0FA);  // 0x0FA: бит 1 (MICB) = 1
    // Power Management 2: DACL, DACR, LOUT1, ROUT1
    wm_write(26, 0x1E0);
    // Power Management 3: LMIC (PGA), LOMIX, ROMIX
    wm_write(27, 0x00C);

    // Audio Interface: I2S, 16-бит, режим ведомого (ESP32 - мастер)
    wm_write(7, 0x002);

    // CLOCKING: SYSCLK = MCLK (без PLL), MCLKDIV=1
    wm_write(4, 0x000);

    // Левый вход PGA: LIN1, 0 дБ, IPVU
    wm_write(0,  0x117);
    // Левый ADC: 0 дБ, ADCVU
    wm_write(21, 0x1C3);

    // Вход PGA -> ADC (LIN1 -> PGA -> ADC)
    wm_write(32, 0x040);
    wm_write(34, 0x040);

    // DAC -> выходной микшер
    wm_write(38, 0x100); // LD2LO: DACL -> LOMIX
    wm_write(40, 0x100); // RD2RO: DACR -> ROMIX

    // Headphone выход: -6 дБ, HPVU
    wm_write(2, 0x170);
    wm_write(3, 0x170);

    ESP_LOGI(TAG, "init OK");
}

void audio_init(i2s_chan_handle_t *tx_out, i2s_chan_handle_t *rx_out) {
    mclk_init();

    // I2C
    i2c_config_t i2c_cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = GPIO_I2C_SDA,
        .scl_io_num       = GPIO_I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_PORT, &i2c_cfg);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

    // I2S
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, tx_out, rx_out);

    i2s_std_config_t std_cfg = {
        .clk_cfg  = {
            .sample_rate_hz = SAMPLE_RATE_HZ,
            .clk_src        = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple  = I2S_MCLK_MULTIPLE_256, // MCLK = 2.048 МГц
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT,
                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,  // MCLK через LEDC на GPIO15
            .bclk = GPIO_I2S_BCLK,
            .ws   = GPIO_I2S_LRCLK,
            .dout = GPIO_I2S_DOUT,
            .din  = GPIO_I2S_DIN,
        },
    };
    i2s_channel_init_std_mode(*tx_out, &std_cfg);
    i2s_channel_init_std_mode(*rx_out, &std_cfg);
    i2s_channel_enable(*tx_out);
    i2s_channel_enable(*rx_out);

    wm8960_init();
}
