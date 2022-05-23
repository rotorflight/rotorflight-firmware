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

uint16_t dshotConvertToInternal(uint8_t index, uint8_t mode, float throttle)
{
    UNUSED(index);

    uint16_t value = DSHOT_CMD_MOTOR_STOP;

    if (mode == MOTOR_CONTROL_BIDIR) {
        if (throttle > 0)
            value = scaleRangef(throttle, 0, 1, DSHOT_FORWARD_MIN_THROTTLE, DSHOT_FORWARD_MAX_THROTTLE);
        else if (throttle < 0)
            value = scaleRangef(throttle, -1, 0, DSHOT_REVERSE_MAX_THROTTLE, DSHOT_REVERSE_MIN_THROTTLE);
    }
    else {
        if (throttle > 0)
            value = scaleRangef(throttle, 0, 1, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    }

    return value;
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
