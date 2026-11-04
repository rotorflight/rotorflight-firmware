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
 * Stubs for everything cli/cli.c links besides the command parser and the
 * parameter groups: motors, servos, mixer, receiver, scheduler, LED strip,
 * beeper, serial and EEPROM. The cliSet() tests don't reach any of them.
 */

#pragma once

extern "C" {
#include "build/version.h"
#include "config/config.h"
#include "drivers/buf_writer.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "fc/core.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "io/beeper.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "msp/msp_box.h"
#include "rx/rx.h"
#include "scheduler/scheduler.h"
#include "sensors/battery.h"

const char * const targetName = "UNITTEST";
const char * const buildDate = "Jan 01 2000";
const char * const buildTime = "00:00:00";
const char * const shortGitRevision = "unittest";
const char * armingDisableFlagNames[ARMING_DISABLE_FLAGS_COUNT];
const uint32_t baudRates[] = { 0, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000 };
const char rcChannelLetters[] = "AERCT12345678";
uint8_t debugMode;
int32_t schedLoopStartCycles;
int32_t taskGuardCycles;

// Parameter group defaults that live in files not linked here
void pgResetFn_ledStripConfig(ledStripConfig_t *) {}
void pgResetFn_ledStripStatusModeConfig(ledStripStatusModeConfig_t *) {}
void pgResetFn_rxFailsafeChannelConfigs(rxFailsafeChannelConfig_t *) {}
void pgResetFn_serialConfig(serialConfig_t *) {}

timeMs_t millis(void) { return 0; }
void systemReset(int) {}

void adjustmentRangeReset(int) {}
void analyzeModeActivationConditions(void) {}
bool isModeActivationConditionConfigured(const modeActivationCondition_t *, const modeActivationCondition_t *) { return false; }
void changeControlRateProfile(uint8_t) {}
void changePidProfile(uint8_t) {}
uint8_t getCurrentControlRateProfileIndex(void) { return 0; }
uint8_t getCurrentPidProfileIndex(void) { return 0; }

void beeper(beeperMode_e) {}
void beeperSilence(void) {}
beeperMode_e beeperModeForTableIndex(int) { return BEEPER_SILENCE; }
uint32_t beeperModeMaskForTableIndex(int) { return 0; }
const char *beeperNameForTableIndex(int) { return NULL; }
int beeperTableEntryCount(void) { return 0; }

void bufWriterAppend(bufWriter_t *, uint8_t) {}
void bufWriterFlush(bufWriter_t *) {}
void bufWriterInit(bufWriter_t *, uint8_t *, int, bufWrite_t, void *) {}

const box_t *findBoxByBoxId(boxId_e) { return NULL; }
const box_t *findBoxByPermanentId(uint8_t) { return NULL; }

void generateLedConfig(ledConfig_t *, char *, size_t) {}
bool parseColor(int, const char *) { return false; }
bool parseLedStripConfig(int, const char *) { return false; }
bool setModeColor(ledModeIndex_e, int, int) { return false; }

armingDisableFlags_e getArmingDisableFlags(void) { return (armingDisableFlags_e)0; }
void setArmingDisabled(armingDisableFlags_e) {}
motorProtocolCause_e getMotorProtocolCause(void) { return MOTOR_PROTO_CAUSE_NONE; }

uint8_t getAverageCPULoadPercent(void) { return 0; }
timeDelta_t getTaskDeltaTimeUs(taskId_e) { return 0; }
void getTaskInfo(taskId_e, taskInfo_t *) {}
void getCheckFuncInfo(cfCheckFuncInfo_t *) {}
void schedulerResetCheckFunctionMaxExecutionTime(void) {}
void schedulerResetTaskMaxExecutionTime(taskId_e) {}

float getAverageRxRefreshRate(void) { return 0; }
uint getCurrentRxRefreshRate(void) { return 0; }
bool rxIsReceivingSignal(void) { return false; }
void parseRcChannels(const char *, struct rxConfig_s *) {}

uint8_t getBatteryCellCount(void) { return 0; }
const char *getBatteryStateString(void) { return ""; }
uint16_t getBatteryVoltage(void) { return 0; }

uint16_t getEEPROMConfigSize(void) { return 0; }
size_t getEEPROMStorageSize(void) { return 0; }
void writeEEPROM(void) {}
void writeUnmodifiedConfigToEEPROM(void) {}
void resetConfig(void) {}
mcuTypeId_e getMcuTypeId(void) { return (mcuTypeId_e)0; }
uint32_t stackHighMem(void) { return 0; }
uint32_t stackTotalSize(void) { return 0; }

uint8_t getMotorCount(void) { return 0; }
int16_t getMotorOutput(uint8_t) { return 0; }
int16_t getMotorOverride(uint8_t) { return 0; }
bool hasMotorOverride(uint8_t) { return false; }
void setMotorOverride(uint8_t, int16_t, timeDelta_t) {}
void resetMotorOverride(void) {}
uint8_t getServoCount(void) { return 0; }
uint16_t getServoOutput(uint8_t) { return 0; }
int16_t getServoOverride(uint8_t) { return 0; }
bool hasServoOverride(uint8_t) { return false; }
int16_t setServoOverride(uint8_t, int16_t) { return 0; }
float mixerGetOutput(uint8_t) { return 0; }
int16_t mixerGetOverride(uint8_t) { return 0; }
int16_t mixerSetOverride(uint8_t, int16_t) { return 0; }
void mixerUpdateMapping(void) {}

void gpsEnablePassthrough(struct serialPort_s *) {}
baudRate_e lookupBaudRateIndex(uint32_t) { return BAUD_AUTO; }
serialPortConfig_t *serialFindPortConfigurationMutable(serialPortIdentifier_e) { return NULL; }
bool serialIsPortAvailable(serialPortIdentifier_e) { return false; }
uint8_t serialRead(serialPort_t *) { return 0; }
uint32_t serialRxBytesWaiting(const serialPort_t *) { return 0; }
void serialWriteBufShim(void *, const uint8_t *, int) {}
void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
ioTag_t timerioTagGetByUsage(timerUsageFlag_e, uint8_t) { return 0; }
}
