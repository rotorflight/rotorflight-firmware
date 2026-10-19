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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

#define CRSF_SENSORS_TIMEOUT_MS_DEFAULT 2000
#define CRSF_SENSORS_TIMEOUT_MS_MIN 500
#define CRSF_SENSORS_TIMEOUT_MS_MAX 10000

typedef struct crsfSensorsGpsData_s {
    int32_t latitude;
    int32_t longitude;
    uint16_t groundspeedCmS;
    uint16_t headingDeg10;
    int32_t altitudeCm;
    uint8_t satellites;
    bool valid;
    timeUs_t lastUpdateUs;
} crsfSensorsGpsData_t;

typedef struct crsfSensorsBatteryData_s {
    uint32_t voltageMv;
    uint32_t currentMa;
    uint32_t capacityMah;
    uint8_t remainingPct;
    bool valid;
    timeUs_t lastUpdateUs;
} crsfSensorsBatteryData_t;

typedef struct crsfSensorsBaroData_s {
    int32_t altitudeCm;
    int16_t verticalSpeedCmS;
    bool valid;
    timeUs_t lastUpdateUs;
} crsfSensorsBaroData_t;

// Per-cell voltage telemetry (CRSF_FRAMETYPE_CELLS, 0x0E). Distinct from the
// aggregate CRSF_FRAMETYPE_BATTERY_SENSOR (0x08) frame - some sensors only
// send this one. Used as a voltage fallback when no 0x08 frame is present.
#define CRSF_SENSORS_CELLS_MAX 12
// Sanity bound, not a real per-cell limit - filters an obviously-faulty
// channel's noise (e.g. tens of volts on one tap) out of the pack total.
#define CRSF_SENSORS_CELL_MV_MAX 6000
typedef struct crsfSensorsCellsData_s {
    uint8_t cellCount;
    uint16_t cellVoltageMv[CRSF_SENSORS_CELLS_MAX];
    uint32_t totalVoltageMv;
    bool valid;
    timeUs_t lastUpdateUs;
} crsfSensorsCellsData_t;

// RPM telemetry (CRSF_FRAMETYPE_RPM, 0x0C) - this firmware's own outbound
// telemetry/crsf.c encoder for this same frame type sends up to 2 values
// (head/tail speed); a third-party sensor accessory could report a
// different count, so this is sized generously rather than to exactly 2.
#define CRSF_SENSORS_RPM_MAX 4
typedef struct crsfSensorsRpmData_s {
    uint8_t rpmCount;
    int32_t rpmValues[CRSF_SENSORS_RPM_MAX];
    bool valid;
    timeUs_t lastUpdateUs;
} crsfSensorsRpmData_t;

void crsfSensorsInit(void);
void crsfSensorsUpdate(timeUs_t currentTimeUs);
bool crsfSensorsIsEnabled(void);

bool crsfSensorsGetGpsData(crsfSensorsGpsData_t *data);
bool crsfSensorsHasGpsData(void);

bool crsfSensorsGetBatteryData(crsfSensorsBatteryData_t *data);
bool crsfSensorsHasBatteryData(void);

bool crsfSensorsGetBaroData(crsfSensorsBaroData_t *data);
bool crsfSensorsHasBaroData(void);

bool crsfSensorsGetCellsData(crsfSensorsCellsData_t *data);
bool crsfSensorsHasCellsData(void);

bool crsfSensorsGetRpmData(crsfSensorsRpmData_t *data);
bool crsfSensorsHasRpmData(void);

void crsfSensorsSetBaroUse(bool enabled);
bool crsfSensorsGetBaroUse(void);

bool crsfSensorsGetRpmUse(void);

// Link-level rx diagnostics, surfaced to the configurator (MSP2_GET_CRSF_SENSORS_STATUS)
// as a debug page for troubleshooting sensor wiring/protocol issues - mirrors
// the FBUS/S.Port Sensors diagnostic tab's role for that bus.
#define CRSF_SENSORS_DEBUG_RAW_LEN 16
typedef struct crsfSensorsDebugStats_s {
    uint32_t rxByteCount;
    uint32_t rxSyncCount;
    uint32_t rxCrcOkCount;
    uint32_t rxCrcFailCount;
    uint8_t lastFrameType;
    uint8_t lastFrameLength;
    uint8_t rawBytes[CRSF_SENSORS_DEBUG_RAW_LEN]; // most recent bytes received, oldest first
} crsfSensorsDebugStats_t;

void crsfSensorsGetDebugStats(crsfSensorsDebugStats_t *stats);
