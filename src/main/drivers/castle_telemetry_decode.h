#pragma once

#include "platform.h"

#include "drivers/timer.h"

typedef struct castleTelemetry_s {
    uint16_t generation;
    uint16_t oneMs;
    uint16_t battVoltage;
    uint16_t rippleVoltage;
    uint16_t battCurrent;
    uint16_t throttle;
    uint16_t outputPower;
    uint16_t rpm;
    uint16_t becVoltage;
    uint16_t becCurrent;
    uint16_t linTempOrHalfMs;
    uint16_t ntcTempOrHalfMs;
} __attribute__ ((__packed__)) castleTelemetry_t;


#define CASTLE_TELEM_NFRAMES 12
#define CASTLE_PWM_HZ_MAX 100
#define CASTLE_PWM_PERIOD_MS_MIN 10
#define CASTLE_PWM_HZ_MIN 50
#define CASTLE_PWM_PERIOD_MS_MAX 20

void getCastleTelemetry(castleTelemetry_t* telem);
struct timerChannel_s;
bool castleInputConfig(const timerHardware_t* timerHardware,
                       struct timerChannel_s* timerChannel, uint32_t hz);
