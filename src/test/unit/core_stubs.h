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
 * No-op stubs for everything fc/core.c reaches that a test of its arming
 * logic does not look at: the flight loop subtasks, blackbox, sensors,
 * telemetry and so on. Include once in a test that links core.c, after
 * defining the stubs the test controls itself.
 */

#pragma once

extern "C" {
#include "blackbox/blackbox.h"
#include "blackbox/blackbox_fielddefs.h"
#include "build/debug.h"
#include "cli/cli.h"
#include "cms/cms.h"
#include "config/config.h"
#include "config/feature.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "flight/airborne.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/rescue.h"
#include "flight/servos.h"
#include "flight/setpoint.h"
#include "io/beeper.h"
#include "io/gps.h"
#include "io/statusindicator.h"
#include "msp/msp_serial.h"
#include "scheduler/scheduler.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "telemetry/telemetry.h"

uint32_t __timing[DEBUG_VALUE_COUNT];
int32_t debug[DEBUG_VALUE_COUNT];
uint8_t debugMode;

uint8_t activeFilterLoopDenom;
uint8_t activePidLoopDenom;
bool cliMode;
bool cmsInMenu;
struct pidProfile_s *currentPidProfile;
gpsSolutionData_t gpsSol;
gyro_t gyro;

void GPS_reset_home_position(void) {}
bool baroIsCalibrationComplete(void) { return true; }
bool compassIsCalibrationComplete(void) { return true; }
void beeperConfirmationBeeps(uint8_t) {}
void beeperSilence(void) {}
void systemBeep(bool) {}
void blackboxFlush(timeUs_t) {}
void blackboxLogEvent(FlightLogEvent, flightLogEventData_t *) {}
void blackboxUpdate(timeUs_t) {}
bool calculateRxChannelsAndUpdateFailsafe(timeUs_t) { return false; }
bool failsafeIsMonitoring(void) { return false; }
void failsafeStartMonitoring(void) {}
bool featureIsEnabled(const uint32_t) { return false; }
uint8_t getAverageCPULoadPercent(void) { return 0; }
timeDelta_t getTaskDeltaTimeUs(taskId_e) { return 0; }
void gyroFiltering(timeUs_t) {}
void gyroReadTemperature(void) {}
void gyroStartCalibration(bool) {}
void gyroUpdate(void) {}
bool isAirborne(void) { return false; }
void mixerUpdate(timeUs_t) {}
void mspSerialAllocatePorts(void) {}
void mspSerialReleaseSharedTelemetryPorts(void) {}
void pidController(const pidProfile_t *, timeUs_t) {}
void positionUpdate(void) {}
void processRcAdjustments(void) {}
void processRcStickPositions(void) {}
void rescueUpdate(void) {}
void resetMotorOverride(void) {}
void servoUpdate(void) {}
void setpointUpdate(void) {}
void telemetryCheckState(void) {}
void updateRSSI(timeUs_t) {}
void updateRcRefreshRate(timeUs_t) {}
void warningLedDisable(void) {}
void warningLedFlash(void) {}
void warningLedUpdate(void) {}
void writeEEPROMDelayed(int) {}
}
