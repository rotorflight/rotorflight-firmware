#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "platform.h"

#if defined(USE_SBUS_OUTPUT) || defined(USE_FBUS_MASTER) || defined(USE_BUS_SERVO)

#include "io/serial.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/bus_servo.h"

PG_REGISTER_WITH_RESET_FN(busServoConfig_t, busServoConfig, PG_BUS_SERVO_CONFIG, 0);

void pgResetFn_busServoConfig(busServoConfig_t *config)
{
    for (int index = 0; index < BUS_SERVO_CHANNELS; index++)
    {
        config->sourceType[index] = (index < 8) ? BUS_SERVO_SOURCE_MIXER : BUS_SERVO_SOURCE_RX;
    }
}

bool hasBusServosConfigured(void)
{
    return findSerialPortConfig(FUNCTION_SBUS_OUT) || findSerialPortConfig(FUNCTION_FBUS_MASTER);
}

static float busServoOutput[BUS_SERVO_CHANNELS];

void setBusServoOutput(uint8_t channel, float value)
{
    if (channel < BUS_SERVO_CHANNELS)
    {
        busServoOutput[channel] = value;
    }
}

uint16_t getBusServoOutput(uint8_t channel)
{
    if (channel < BUS_SERVO_CHANNELS)
    {
        const long value = lrintf(busServoOutput[channel]);
        if (value < 0)
            return 0;
        if (value > UINT16_MAX)
            return UINT16_MAX;
        return (uint16_t)value;
    }
    return 0;
}
#endif
