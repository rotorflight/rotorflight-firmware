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
 * Stubs for the blackbox output device and the flight state that
 * blackbox/blackbox.c samples. The interval tests never start a log.
 */

#pragma once

extern "C" {
#include "blackbox/blackbox_io.h"
#include "build/debug.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "io/gps.h"
#include "sensors/gyro.h"

uint8_t armingFlags;
uint8_t stateFlags;
uint8_t debugMode;
boxBitmask_t rcModeActivationMask;
struct pidProfile_s *currentPidProfile;
gpsSolutionData_t gpsSol;
int32_t GPS_home[2];
gyro_t gyro;
int32_t blackboxHeaderBudget;

timeMs_t millis(void) { return 0; }
bool IS_RC_MODE_ACTIVE(boxId_e) { return false; }
bool sensors(uint32_t) { return false; }
bool featureIsEnabled(const uint32_t) { return false; }
bool adcIsEnabled(uint8_t) { return false; }

bool blackboxDeviceBeginLog(void) { return false; }
void blackboxDeviceClose(void) {}
bool blackboxDeviceEndLog(bool) { return false; }
void blackboxDeviceFlush(void) {}
bool blackboxDeviceFlushForce(void) { return false; }
bool blackboxDeviceFlushForceComplete(void) { return false; }
bool blackboxDeviceOpen(void) { return false; }
blackboxBufferReserveStatus_e blackboxDeviceReserveBufferSpace(int32_t) { return BLACKBOX_RESERVE_SUCCESS; }
void blackboxOpen(void) {}
void blackboxReplenishHeaderBudget(void) {}
void blackboxWrite(uint8_t) {}
int blackboxWriteString(const char *) { return 0; }
bool isBlackboxDeviceFull(void) { return false; }
bool isBlackboxDeviceReady(void) { return false; }

failsafePhase_e failsafePhase(void) { return FAILSAFE_IDLE; }
uint32_t getArmingBeepTimeMicros(void) { return 0; }
uint16_t getBatteryVoltageSample(void) { return 0; }
bool isBatteryCurrentConfigured(void) { return false; }
bool isBatteryVoltageConfigured(void) { return false; }
int getGovernorState(void) { return 0; }
int getRescueState(void) { return 0; }
uint8_t getMotorCount(void) { return 0; }
uint8_t getServoCount(void) { return 0; }
bool isAirborne(void) { return false; }
bool isRssiConfigured(void) { return false; }
bool rxAreFlightChannelsValid(void) { return false; }
bool rxIsReceivingSignal(void) { return false; }
}
