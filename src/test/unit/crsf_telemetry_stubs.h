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

/*
 * Stubs for the parts of telemetry/crsf.c that are not under test in the
 * CRSF telemetry unit tests: custom sensor telemetry, its scheduler, and
 * the flight/sensor values those frames report. Include once, inside
 * extern "C", in a test that links telemetry/crsf.c.
 */

#pragma once

#include "fc/rc_adjustments.h"
#include "flight/governor.h"
#include "flight/position.h"
#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "telemetry/sensors.h"
#include "telemetry/telemetry.h"

acc_t acc;

int getEstimatedVarioCms(void) { return 0; }
int getGovernorState(void) { return 0; }
float mixerGetInput(uint8_t) { return 0; }

uint8_t getBatteryCellCount(void) { return 0; }
uint16_t getBatteryCellVoltage(uint8_t) { return 0; }

const char *getAdjustmentsRangeName(void) { return NULL; }
int getAdjustmentsRangeFunc(void) { return 0; }
int getAdjustmentsRangeValue(void) { return 0; }

int telemetrySensorValue(sensor_id_e) { return 0; }
bool telemetrySensorActive(sensor_id_e) { return false; }

void telemetryScheduleInit(telemetrySensor_t *, size_t, bool) {}
void telemetryScheduleAdd(telemetrySensor_t *) {}
void telemetryScheduleUpdate(timeUs_t) {}
void telemetryScheduleCommit(telemetrySensor_t *) {}
telemetrySensor_t *telemetryScheduleNext(void) { return NULL; }
