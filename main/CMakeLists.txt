idf_component_register(
    SRCS "main.c" "state_machine.c" "bell.c" "rotary_dial.c"
         "hook_switch.c" "dial_tone.c" "audio.c" "bt_hfp.c"
    INCLUDE_DIRS "."
    REQUIRES driver bt nvs_flash esp_timer
)
