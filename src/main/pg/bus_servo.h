#pragma once

#include "common/utils.h"
#include "pg/pg.h"

#define DEFAULT_BUS_SERVO_MIN     -500
#define DEFAULT_BUS_SERVO_MAX     500
#define DEFAULT_BUS_SERVO_SCALE   1000
#define BUS_SERVO_MAX_SIGNAL      2000
#define BUS_SERVO_MIN_SIGNAL      1000
#define BUS_SERVO_OFFSET          8

typedef enum {
    BUS_SERVO_SOURCE_MIXER = 0,
    BUS_SERVO_SOURCE_RX = 1
} busServoSourceType_e;

typedef struct busServoConfig_s {
    uint8_t sourceType[BUS_SERVO_CHANNELS];
} busServoConfig_t;

PG_DECLARE(busServoConfig_t, busServoConfig);

void setBusServoOutput(uint8_t channel, float value);
uint16_t getBusServoOutput(uint8_t channel);
bool hasBusServosConfigured(void);
