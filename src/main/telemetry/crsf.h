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
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

#include "rx/crsf_protocol.h"
#include "telemetry/msp_shared.h"

enum {
    CRSF_FM_REUSE_NONE = 0,
    CRSF_FM_REUSE_GOV_STATE,
    CRSF_FM_REUSE_HEADSPEED,
    CRSF_FM_REUSE_THROTTLE,
    CRSF_FM_REUSE_ESC_TEMP,
    CRSF_FM_REUSE_MCU_TEMP,
    CRSF_FM_REUSE_MCU_LOAD,
    CRSF_FM_REUSE_SYS_LOAD,
    CRSF_FM_REUSE_RT_LOAD,
    CRSF_FM_REUSE_ADJFUNC,
    CRSF_FM_REUSE_GOV_ADJFUNC,
};

enum {
    CRSF_ATT_REUSE_NONE = 0,
    CRSF_ATT_REUSE_HEADSPEED,
    CRSF_ATT_REUSE_THROTTLE,
    CRSF_ATT_REUSE_ESC_TEMP,
    CRSF_ATT_REUSE_MCU_TEMP,
    CRSF_ATT_REUSE_MCU_LOAD,
    CRSF_ATT_REUSE_SYS_LOAD,
    CRSF_ATT_REUSE_RT_LOAD,
};

void initCrsfTelemetry(void);
uint32_t getCrsfDesiredSpeed(void);
void setCrsfDefaultSpeed(void);
bool checkCrsfTelemetryState(void);
void handleCrsfTelemetry(timeUs_t currentTimeUs);
void crsfScheduleDeviceInfoResponse(void);
void crsfScheduleMspResponse(uint8_t requestOriginID);
int getCrsfFrame(uint8_t *frame, crsfFrameType_e frameType);
void crsfProcessCommand(uint8_t *frameStart);
#if defined(USE_CRSF_CMS_TELEMETRY)
void crsfProcessDisplayPortCmd(uint8_t *frameStart);
#endif
#if defined(USE_MSP_OVER_TELEMETRY)
void initCrsfMspBuffer(void);
bool bufferCrsfMspFrame(uint8_t *frameStart, int frameLength);
bool handleCrsfMspFrameBuffer(mspResponseFnPtr responseFn);
int getCrsfMspFrame(uint8_t *frame, uint8_t *payload, const uint8_t payloadSize);
#endif
#if defined(USE_CRSF_V3)
void speedNegotiationProcess(uint32_t currentTime);
bool crsfBaudNegotiationInProgress(void);
uint32_t getCrsfCachedBaudrate(void);
#endif
