#include "jd_services.h"
#include "interfaces/jd_pins.h"
#include "jacdac/dist/c/vibrationmotor.h"
#include "lib.h"

#define MAX_SEQUENCE 118

struct srv_state {
    SRV_COMMON;
    uint32_t now;
    uint8_t idx;
    const vibration_motor_api_t *api;
    jd_vibration_motor_vibrate_t sequence[MAX_SEQUENCE];
};

#if 1
void vibration_process(srv_t *state) {
    jd_vibration_motor_vibrate_t *curr = &state->sequence[state->idx];

    if (curr->duration == 0) {
        state->idx = (state->idx + 1) % MAX_SEQUENCE;
        return;
    }

    // each speed tick is 8 ms in duration
    // write for the next 8 ms
    state->api->write_amplitude(curr->intensity, 8);

    if (!jd_should_sample(&state->now, 8000))
        return;

    curr->duration--;
}
#endif

#if 0
void vibration_process(srv_t * state) {
    if (!jd_should_sample(&state->now, 8000))
        return;

    jd_vibration_motor_vibrate_t* curr = &state->sequence[state->idx];

    if (curr->duration == 0) {
        state->idx = (state->idx + 1) % MAX_SEQUENCE;
        return;
    }
    
    DMESG("AMP write %d %d", curr->duration, curr->speed);

    // each speed tick is 8 ms in duration
    // write for the next 8 ms
    state->api->write_amplitude(curr->speed, 8);
    curr->duration--;
}
#endif

static void handle_vibrate_cmd(srv_t *state, jd_packet_t *pkt) {
    memset(state->sequence, 0, sizeof(state->sequence));
    memcpy(state->sequence, pkt->data, min(sizeof(state->sequence), pkt->service_size));
    state->idx = 0;
}

void vibration_handle_packet(srv_t *state, jd_packet_t *pkt) {
    switch (pkt->service_command) {
    case JD_VIBRATION_MOTOR_CMD_VIBRATE:
        handle_vibrate_cmd(state, pkt);
        break;

    default:
        jd_send_not_implemented(pkt);
        break;
    }
}

SRV_DEF(vibration, JD_SERVICE_CLASS_VIBRATION_MOTOR);
void vibration_motor_init(const vibration_motor_api_t *api) {
    SRV_ALLOC(vibration);
    state->api = api;

    memset(state->sequence, 0x00, sizeof(state->sequence));

    state->api->init();
}