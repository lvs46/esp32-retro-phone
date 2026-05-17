#pragma once
#include "driver/i2s_std.h"
void dial_tone_init(i2s_chan_handle_t tx);
void dial_tone_start(void);       // непрерывный 425 Гц
void busy_tone_start(void);       // 425 Гц, 0.35с / 0.35с
void dial_tone_stop(void);
