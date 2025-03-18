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

#include <stdint.h>
#include <stdbool.h>

#include "pg/system.h"
#include "pg/pilot.h"

struct pidProfile_s;
extern struct pidProfile_s *currentPidProfile;

void initEEPROM(void);
bool resetEEPROM(bool useCustomDefaults);
bool readEEPROM(void);
void writeEEPROM(void);
void writeEEPROMDelayed(int delayUs);
void writeUnmodifiedConfigToEEPROM(void);

void saveConfigAndNotify(void);
void validateAndFixGyroConfig(void);

void setConfigDirty(void);
bool isConfigDirty(void);

uint8_t getCurrentPidProfileIndex(void);
void changePidProfile(uint8_t pidProfileIndex);

uint8_t getCurrentControlRateProfileIndex(void);
void changeControlRateProfile(uint8_t profileIndex);

uint8_t getCurrentBatteryProfileIndex(void);
void changeBatteryProfile(uint8_t batteryProfileIndex);

bool canSoftwareSerialBeUsed(void);

uint16_t getCurrentMinthrottle(void);

void resetConfig(void);
void targetConfiguration(void);
void targetValidateConfiguration(void);

bool isSystemConfigured(void);
void setRebootRequired(void);
bool getRebootRequired(void);
bool isEepromWriteInProgress(void);
