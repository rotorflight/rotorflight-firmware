/*
 * This file is part of RotorFlight.
 *
 * RotorFlight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RotorFlight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RotorFlight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#include "common/utils.h"
#include "common/time.h"
#include "common/axis.h"

#include "telemetry/telemetry.h"
#include "telemetry/sbus2.h"
#include "telemetry/sbus2_sensors.h"

#include "rx/rx.h"
#include "rx/sbus.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/adcinternal.h"

#include "io/gps.h"


#ifdef USE_ESC_SENSOR
#include "sensors/esc_sensor.h"
#include "flight/mixer.h"
#endif

#include "flight/position.h"

#ifdef USE_TELEMETRY_SBUS2

const uint8_t sbus2SlotIds[SBUS2_SLOT_COUNT] = {
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3,
    0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB,
    0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB
};

sbus2_telemetry_frame_t sbusTelemetryData[SBUS2_SLOT_COUNT] = {0};
uint8_t sbusTelemetryDataUsed[SBUS2_SLOT_COUNT] = {0};
static uint8_t currentSlot = 0;
static timeUs_t nextSlotTime = 0;

void INIT_CODE initSbus2Telemetry(void)
{
    for(int i = 0; i < SBUS2_SLOT_COUNT; ++i) {
        memset(&sbusTelemetryData[i], 0, sizeof(sbus2_telemetry_frame_t));
        sbusTelemetryDataUsed[i] = 0;
    }
}

bool checkSbus2TelemetryState(void)
{
    return rxRuntimeState.serialrxProvider == SERIALRX_SBUS2;
}

void handleSbus2Telemetry(timeUs_t currentTimeUs) 
{
    UNUSED(currentTimeUs);

    float voltage = getBatteryVoltage() * 0.01f;
    float cellVoltage =  getBatteryAverageCellVoltage() * 0.01f;
    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    float current =  getBatteryCurrent() * 0.01f;
    float capacity = getBatteryCapacityUsed();
    float temperature =  getCoreTemperatureCelsius();
    uint32_t rpm = getHeadSpeed();

    // 2 slots
    send_voltagef(1, voltage, cellVoltage);
    // 3 slots
    send_s1678_currentf(3, current, capacity, voltage);
    // 1 slot
    send_RPM(6, rpm);
    // 1 slot - esc temp
    send_SBS01T(7, temperature);

    if (escData != NULL) {
        // 8 slots, esc
        send_kontronik(8,  escData->voltage * 0.1f, escData->consumption * 100, escData->erpm, escData->current * 0.01f , escData->temperature * 10, escData->temperature2 * 10, escData->bec_current * 10, escData->pwm);
    }
}

uint8_t sbus2GetTelemetrySlot(timeUs_t elapsed)
{
    UNUSED(elapsed);
    if (elapsed < SBUS2_DEADTIME) {
        currentSlot = 0;
        nextSlotTime = 0;
        return 0xFF; // skip it
    }

    if(currentSlot < SBUS2_TELEMETRY_SLOTS) {
        return currentSlot;
    }

    return 0xFF;
}

void sbus2IncrementTelemetrySlot(timeUs_t currentTimeUs)
{
    nextSlotTime = currentTimeUs + (SBUS2_TRANSMIT_TIME + SBUS2_SLOT_DELAY);
    currentSlot++;
}

FAST_CODE void taskSendSbus2Telemetry(timeUs_t currentTimeUs)
{
    if (!telemetrySharedPort || rxRuntimeState.serialrxProvider !=  SERIALRX_SBUS2) {
        return;
    }

    timeUs_t elapsedTime = cmp32(currentTimeUs, sbusGetLastFrameTime());

    if (elapsedTime > MS2US(8)) {
        currentSlot = 0;
        nextSlotTime = 0;
        return;
    }

    if (currentTimeUs < nextSlotTime) {
        return;
    }

    uint8_t telemetryPage = sbusGetCurrentTelemetryPage();

    uint8_t slot = sbus2GetTelemetrySlot(elapsedTime);

    if(slot < SBUS2_TELEMETRY_SLOTS) {
        int slotIndex = (telemetryPage * SBUS2_TELEMETRY_SLOTS) + slot;
        if (slotIndex < SBUS2_SLOT_COUNT) {
            if (sbusTelemetryDataUsed[slotIndex] != 0) {
                // send
                serialWriteBuf(telemetrySharedPort,
                               (const uint8_t *)&sbusTelemetryData[slotIndex],
                               sizeof(sbus2_telemetry_frame_t));
            }
        }
        sbus2IncrementTelemetrySlot(currentTimeUs);
    }
}




#endif
