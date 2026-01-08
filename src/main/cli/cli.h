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

extern bool cliMode;

void cliProcess(void);
bool hasCustomDefaults(void);
struct serialPort_s;
void cliEnter(struct serialPort_s *serialPort);
bool resetConfigToCustomDefaults(void);

// MSP generic setting access (name-based)
// Returns false if the setting is not found or unsupported (arrays/strings/etc).
// value/min/max are returned as scaled integers; scalePow10 currently always 0 in this codebase.
bool cliMspGetSettingByName(const char *name, int32_t *value, int32_t *min, int32_t *max, uint8_t *mspType, uint8_t *flags, int8_t *scalePow10);
bool cliMspSetSettingByName(const char *name, int32_t value);

#ifdef USE_CLI_DEBUG_PRINT
void cliPrint(const char *str);
void cliPrintLinefeed(void);
void cliPrintLine(const char *str);
void cliPrintf(const char *format, ...);
void cliPrintLinef(const char *format, ...);
#endif
