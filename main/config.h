#pragma once
#include "driver/gpio.h"

// Целевой модуль: ESP32 DevKit (WROOM-32)

// --- GPIO ---
#define GPIO_HOOK_SWITCH    GPIO_NUM_32  // NC когда трубка лежит
#define GPIO_DIAL_PULSE     GPIO_NUM_33  // импульсы диска (ИК, жёлтый)
#define GPIO_DIAL_ACTIVE    GPIO_NUM_16  // диск активен (ЗП, синий) — замкнут при наборе
#define GPIO_BELL_IN1       GPIO_NUM_27  // L293D IN1 (мост звонка)
#define GPIO_BELL_IN2       GPIO_NUM_14  // L293D IN2 (мост звонка)
#define GPIO_BOOST_EN       GPIO_NUM_13  // EN MT3608

// --- I2S (WM8960) ---
#define I2S_PORT            I2S_NUM_0
#define GPIO_I2S_MCLK       GPIO_NUM_15  // MCLK через LEDC (GPIO0 - strapping pin BOOT)
#define GPIO_I2S_BCLK       GPIO_NUM_26
#define GPIO_I2S_LRCLK      GPIO_NUM_25
#define GPIO_I2S_DOUT       GPIO_NUM_17  // ESP32 → WM8960 DDAT
#define GPIO_I2S_DIN        GPIO_NUM_34  // WM8960 ADAT → ESP32 (input-only)

// --- I2C (WM8960 управление) ---
#define I2C_PORT            I2C_NUM_0
#define GPIO_I2C_SCL        GPIO_NUM_22
#define GPIO_I2C_SDA        GPIO_NUM_21
#define WM8960_ADDR         0x1A

// --- Аудио ---
#define SAMPLE_RATE_HZ      8000
#define DIAL_TONE_HZ        425

// --- Дисковый набор ---
#define PULSE_MIN_MS        40    // минимальная длительность импульса
#define DIGIT_TIMEOUT_MS    280   // пауза между цифрами

// --- Звонок ---
#define BELL_FREQ_HZ        18
#define BELL_ON_MS          1000
#define BELL_OFF_MS         4000
