#pragma once
#include "driver/i2s_std.h"
#include <stdbool.h>
void bt_hfp_init(i2s_chan_handle_t tx, i2s_chan_handle_t rx);
void bt_hfp_dial(const char *number);
void bt_hfp_answer(void);
void bt_hfp_hangup(void);
bool bt_hfp_is_connected(void);
void bt_hfp_reset_pairing(void);
