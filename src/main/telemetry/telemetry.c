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

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>

#include "platform.h"

#ifdef USE_TELEMETRY

#include "common/utils.h"
#include "common/unit.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/telemetry.h"
#include "pg/rx.h"

#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"

#include "io/serial.h"

#include "config/config.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky_hub.h"
#include "telemetry/hott.h"
#include "telemetry/smartport.h"
#include "telemetry/ltm.h"
#include "telemetry/jetiexbus.h"
#include "telemetry/mavlink.h"
#include "telemetry/crsf.h"
#include "telemetry/ghst.h"
#include "telemetry/srxl.h"
#include "telemetry/ibus.h"
#include "telemetry/sbus2.h"
#include "telemetry/msp_shared.h"


serialPort_t *telemetrySharedPort = NULL;


bool telemetryDetermineEnabledState(portSharing_e portSharing)
{
    bool enabled = (portSharing == PORTSHARING_NOT_SHARED);

    if (portSharing == PORTSHARING_SHARED) {
        if (isModeActivationConditionPresent(BOXTELEMETRY))
            enabled = IS_RC_MODE_ACTIVE(BOXTELEMETRY);
        else
            enabled = ARMING_FLAG(ARMED);
    }

    return enabled;
}

bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig, const SerialRXType serialrxProvider)
{
    if ((portConfig->functionMask & FUNCTION_RX_SERIAL) &&
        (portConfig->functionMask & TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK) &&
        (serialrxProvider == SERIALRX_SPEKTRUM1024 ||
         serialrxProvider == SERIALRX_SPEKTRUM2048 ||
         serialrxProvider == SERIALRX_SBUS ||
         serialrxProvider == SERIALRX_SBUS2 ||
         serialrxProvider == SERIALRX_SUMD ||
         serialrxProvider == SERIALRX_SUMH ||
         serialrxProvider == SERIALRX_XBUS_MODE_B ||
         serialrxProvider == SERIALRX_XBUS_MODE_B_RJ01 ||
         serialrxProvider == SERIALRX_IBUS)) {

        return true;
    }
#ifdef USE_TELEMETRY_IBUS
    if (portConfig->functionMask & FUNCTION_TELEMETRY_IBUS &&
        portConfig->functionMask & FUNCTION_RX_SERIAL &&
        serialrxProvider == SERIALRX_IBUS) {
        // IBUS serial RX & telemetry
        return true;
    }
#endif
    return false;
}


void telemetryProcess(timeUs_t currentTime)
{
#ifdef USE_TELEMETRY_FRSKY_HUB
    handleFrSkyHubTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_HOTT
    handleHoTTTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    handleSmartPortTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_LTM
    handleLtmTelemetry();
#endif
#ifdef USE_TELEMETRY_JETIEXBUS
    handleJetiExBusTelemetry();
#endif
#ifdef USE_TELEMETRY_MAVLINK
    handleMAVLinkTelemetry();
#endif
#ifdef USE_TELEMETRY_CRSF
    handleCrsfTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_GHST
    handleGhstTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_SRXL
    handleSrxlTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_IBUS
    handleIbusTelemetry();
#endif
#ifdef USE_TELEMETRY_SBUS2
    handleSbus2Telemetry(currentTime);
#endif
}

void telemetryCheckState(void)
{
#ifdef USE_TELEMETRY_FRSKY_HUB
    checkFrSkyHubTelemetryState();
#endif
#ifdef USE_TELEMETRY_HOTT
    checkHoTTTelemetryState();
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    checkSmartPortTelemetryState();
#endif
#ifdef USE_TELEMETRY_LTM
    checkLtmTelemetryState();
#endif
#ifdef USE_TELEMETRY_JETIEXBUS
    checkJetiExBusTelemetryState();
#endif
#ifdef USE_TELEMETRY_MAVLINK
    checkMAVLinkTelemetryState();
#endif
#ifdef USE_TELEMETRY_CRSF
    checkCrsfTelemetryState();
#endif
#ifdef USE_TELEMETRY_GHST
    checkGhstTelemetryState();
#endif
#ifdef USE_TELEMETRY_SRXL
    checkSrxlTelemetryState();
#endif
#ifdef USE_TELEMETRY_IBUS
    checkIbusTelemetryState();
#endif
#ifdef USE_TELEMETRY_SBUS2
    checkSbus2TelemetryState();
#endif
}

void INIT_CODE telemetryInit(void)
{
    legacySensorInit();

#ifdef USE_TELEMETRY_FRSKY_HUB
    initFrSkyHubTelemetry();
#endif
#ifdef USE_TELEMETRY_HOTT
    initHoTTTelemetry();
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    initSmartPortTelemetry();
#endif
#ifdef USE_TELEMETRY_LTM
    initLtmTelemetry();
#endif
#ifdef USE_TELEMETRY_JETIEXBUS
    initJetiExBusTelemetry();
#endif
#ifdef USE_TELEMETRY_MAVLINK
    initMAVLinkTelemetry();
#endif
#ifdef USE_TELEMETRY_GHST
    initGhstTelemetry();
#endif
#ifdef USE_TELEMETRY_CRSF
    initCrsfTelemetry();
#endif
#ifdef USE_TELEMETRY_SRXL
    initSrxlTelemetry();
#endif
#ifdef USE_TELEMETRY_IBUS
    initIbusTelemetry();
#endif
#ifdef USE_TELEMETRY_SBUS2
    initSbus2Telemetry();
#endif
#if defined(USE_MSP_OVER_TELEMETRY)
    initSharedMsp();
#endif

    telemetryCheckState();
}


/** Telemetry scheduling framework **/

static telemetryScheduler_t sch = INIT_ZERO;


void INIT_CODE telemetryScheduleAdd(telemetrySensor_t * sensor)
{
    if (sensor) {
        sensor->bucket = 0;
        sensor->value = 0;
        sensor->update = true;
        sensor->active = true;
    }
}

void telemetryScheduleUpdate(timeUs_t currentTime)
{
    timeDelta_t delta = cmpTimeUs(currentTime, sch.update_time);

    for (int i = 0; i < sch.sensor_count; i++) {
        telemetrySensor_t * sensor = &sch.sensors[i];
        if (sensor->active) {
            const int value = telemetrySensorValue(sensor->senid);
            sensor->update |= (value != sensor->value);
            sensor->value = value;

            const int interval = (sensor->update) ? sensor->fast_interval : sensor->slow_interval;
            sensor->bucket += delta * 1000 / interval;
            sensor->bucket = constrain(sensor->bucket, sch.min_level, sch.max_level);
        }
    }

    sch.update_time = currentTime;
}

telemetrySensor_t * telemetryScheduleNext(void)
{
    int index = sch.start_index;

    for (int i = 0; i < sch.sensor_count; i++) {
        index = (index + 1) % sch.sensor_count;
        telemetrySensor_t * sensor = &sch.sensors[index];
        if (sensor->active && sensor->bucket >= 0)
            return sensor;
    }

    if (sch.use_excess) {
        telemetrySensor_t * sensor = NULL;
        uint lowest = UINT_MAX;

        for (int i = 0; i < sch.sensor_count; i++) {
            telemetrySensor_t * iter = &sch.sensors[i];
            if (iter->active && iter->bucket < 0) {
                const uint16_t weight = (iter->update) ? iter->fast_weight : iter->slow_weight;
                if (weight) {
                    const uint32_t excess = -iter->bucket / weight;
                    if (excess < lowest) {
                        lowest = excess;
                        sensor = iter;
                    }
                }
            }
        }

        return sensor;
    }

    return NULL;
}

void telemetryScheduleCommit(telemetrySensor_t * sensor)
{
    if (sensor) {
        if (sch.use_excess && sensor->bucket < 0) {
            const uint32_t weight = (sensor->update) ? sensor->fast_weight : sensor->slow_weight;
            const uint32_t excess = -sensor->bucket / weight;
            for (int i = 0; i < sch.sensor_count; i++) {
                telemetrySensor_t * iter = &sch.sensors[i];
                if (iter->active) {
                    const uint16_t weight = (iter->update) ? iter->fast_weight : iter->slow_weight;
                    iter->bucket += excess * weight;
                }
            }
        }

        sensor->bucket = constrain(sensor->bucket - sch.quanta, sch.min_level, sch.max_level);
        sensor->update = false;

        sch.start_index = sensor->index;
    }
}

void INIT_CODE telemetryScheduleInit(telemetrySensor_t * sensors, size_t count, bool use_excess)
{
    sch.sensors = sensors;
    sch.sensor_count = count;

    sch.update_time = 0;
    sch.start_index = 0;

    sch.quanta = 1000000;
    sch.max_level = 500000;
    sch.min_level = -1500000;

    sch.use_excess = use_excess;

    for (uint i = 0; i < count; i++) {
        telemetrySensor_t * sensor = &sch.sensors[i];
        sensor->index = i;
    }
}

#endif /* USE_TELEMETRY */
