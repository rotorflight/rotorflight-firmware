/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "build/atomic.h"

#include "common/maths.h"
#include "common/time.h"

#include "config/feature.h"

#include "drivers/motor.h"
#include "drivers/timer.h"

#include "drivers/dshot_dpwm.h" // for motorDmaOutput_t, should be gone
#include "drivers/dshot_command.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h" // for PWM_TYPE_* and others

#include "rx/rx.h"

#include "dshot.h"

void dshotInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow) {
    UNUSED(motorConfig);
    UNUSED(deadbandMotor3dHigh);
    UNUSED(deadbandMotor3dLow);
    float outputLimitOffset = DSHOT_RANGE * (1 - outputLimit);
    *disarm = DSHOT_CMD_MOTOR_STOP;
    *outputLow = DSHOT_MIN_THROTTLE;
    *outputHigh = DSHOT_MAX_THROTTLE - outputLimitOffset;
}

float dshotConvertFromExternal(uint16_t externalValue)
{
    float motorValue;

    externalValue = constrain(externalValue, PWM_RANGE_MIN, PWM_RANGE_MAX);
    motorValue = (externalValue == PWM_RANGE_MIN) ? DSHOT_CMD_MOTOR_STOP : scaleRangef(externalValue, PWM_RANGE_MIN + 1, PWM_RANGE_MAX, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);

    return motorValue;
}

uint16_t dshotConvertToExternal(float motorValue)
{
    float externalValue;

    externalValue = (motorValue < DSHOT_MIN_THROTTLE) ? PWM_RANGE_MIN : scaleRangef(motorValue, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE, PWM_RANGE_MIN + 1, PWM_RANGE_MAX);

    return lrintf(externalValue);
}

FAST_CODE uint16_t prepareDshotPacket(dshotProtocolControl_t *pcb)
{
    uint16_t packet;

    ATOMIC_BLOCK(NVIC_PRIO_DSHOT_DMA) {
        packet = (pcb->value << 1) | (pcb->requestTelemetry ? 1 : 0);
        pcb->requestTelemetry = false;    // reset telemetry request to make sure it's triggered only once in a row
    }

    // compute checksum
    unsigned csum = 0;
    unsigned csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    // append checksum
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        csum = ~csum;
    }
#endif
    csum &= 0xf;
    packet = (packet << 4) | csum;

    return packet;
}

#ifdef USE_DSHOT_TELEMETRY
FAST_DATA_ZERO_INIT dshotTelemetryState_t dshotTelemetryState;

uint16_t getDshotTelemetry(uint8_t index)
{
    return dshotTelemetryState.motorState[index].telemetryValue;
}

#endif

#ifdef USE_DSHOT_TELEMETRY_STATS
FAST_DATA_ZERO_INIT dshotTelemetryQuality_t dshotTelemetryQuality[MAX_SUPPORTED_MOTORS];

void updateDshotTelemetryQuality(dshotTelemetryQuality_t *qualityStats, bool packetValid, timeMs_t currentTimeMs)
{
    uint8_t statsBucketIndex = (currentTimeMs / DSHOT_TELEMETRY_QUALITY_BUCKET_MS) % DSHOT_TELEMETRY_QUALITY_BUCKET_COUNT;
    if (statsBucketIndex != qualityStats->lastBucketIndex) {
        qualityStats->packetCountSum -= qualityStats->packetCountArray[statsBucketIndex];
        qualityStats->invalidCountSum -= qualityStats->invalidCountArray[statsBucketIndex];
        qualityStats->packetCountArray[statsBucketIndex] = 0;
        qualityStats->invalidCountArray[statsBucketIndex] = 0;
        qualityStats->lastBucketIndex = statsBucketIndex;
    }
    qualityStats->packetCountSum++;
    qualityStats->packetCountArray[statsBucketIndex]++;
    if (!packetValid) {
        qualityStats->invalidCountSum++;
        qualityStats->invalidCountArray[statsBucketIndex]++;
    }
}
#endif // USE_DSHOT_TELEMETRY_STATS

#endif // USE_DSHOT
