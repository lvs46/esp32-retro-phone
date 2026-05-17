#pragma once
#include <stdint.h>

typedef enum {
    STATE_IDLE,         // трубка лежит
    STATE_DIAL_TONE,    // трубка поднята, гудок
    STATE_DIALING,      // идёт набор цифр
    STATE_OUTGOING,     // ждём ответа
    STATE_RINGING,      // входящий: звонок звонит
    STATE_IN_CALL,      // разговор
} phone_state_t;

typedef enum {
    EVT_HOOK_OFF,       // трубка поднята
    EVT_HOOK_ON,        // трубка положена
    EVT_DIGIT,          // цифра набрана (data = 0-9)
    EVT_BT_INCOMING,    // входящий звонок по BT
    EVT_BT_ANSWERED,    // наш вызов принят
    EVT_BT_CALL_END,    // звонок завершён
    EVT_SCO_CONNECTED,  // аудио канал SCO открыт
    EVT_SCO_CLOSED,     // аудио канал SCO закрыт
    EVT_DIAL_TIMEOUT,   // пауза после последней цифры — номер готов
} phone_event_t;

void          sm_init(void);
void          sm_dispatch(phone_event_t evt, uint32_t data);
phone_state_t sm_state(void);
