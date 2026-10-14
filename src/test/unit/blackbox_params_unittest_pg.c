/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * The parameter groups of blackbox_params_unittest: what their reset functions need from modules that are not
 * in the unit test.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/timer.h"

#include "fc/rc.h"

#include "pg/battery.h"
#include "pg/rx.h"

#include "rx/rx.h"

// pg/arming.c uses BIT() without including common/utils.h
#include "pg/arming.c"

// sensors/battery.c
const char * const batteryVoltageSourceNames[VOLTAGE_METER_COUNT] = {
    [VOLTAGE_METER_NONE]    = "NONE",
    [VOLTAGE_METER_ADC]     = "ADC",
    [VOLTAGE_METER_ESC]     = "ESC",
    [VOLTAGE_METER_FBUS]    = "FBUS",
    [VOLTAGE_METER_CRSF]    = "CRSF",
};

const char * const batteryCurrentSourceNames[CURRENT_METER_COUNT] = {
    [CURRENT_METER_NONE]    = "NONE",
    [CURRENT_METER_ADC]     = "ADC",
    [CURRENT_METER_ESC]     = "ESC",
    [CURRENT_METER_FBUS]    = "FBUS",
    [CURRENT_METER_CRSF]    = "CRSF",
};

// rx/rx.c
void pgResetFn_rxFailsafeChannelConfigs(rxFailsafeChannelConfig_t *rxFailsafeChannelConfigs)
{
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfigs[i].mode = (i < CONTROL_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        rxFailsafeChannelConfigs[i].step = (i == THROTTLE)
            ? CHANNEL_VALUE_TO_RXFAIL_STEP(RX_PWM_PULSE_MIN)
            : CHANNEL_VALUE_TO_RXFAIL_STEP(RX_PWM_PULSE_MID);
    }
}

void parseRcChannels(const char *input, rxConfig_t *rxConfig)
{
    static const char rcChannelLetters[] = "AERCT12345678";

    for (int i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
        rxConfig->rcmap[i] = i;
        for (int j = 0; input[j]; j++) {
            if (rcChannelLetters[i] == input[j]) {
                rxConfig->rcmap[i] = j;
                break;
            }
        }
    }
}

// drivers/timer.c: no motor pins
ioTag_t timerioTagGetByUsage(timerUsageFlag_e usageFlag, uint8_t index)
{
    UNUSED(usageFlag);
    UNUSED(index);
    return IO_TAG_NONE;
}
