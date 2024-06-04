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

#include "types.h"
#include "platform.h"

#include "build/build_config.h"
#include "common/time.h"

#include "pg/blackbox.h"

#include "blackbox_fielddefs.h"


void blackboxLogEvent(FlightLogEvent event, flightLogEventData_t *data);

void blackboxLogCustomData(const uint8_t *ptr, size_t length);
void blackboxLogCustomString(const char *ptr);

void blackboxUpdate(timeUs_t currentTimeUs);
void blackboxFlush(timeUs_t currentTimeUs);
void blackboxInit(void);

void blackboxErase(void);
bool isBlackboxErased(void);

void blackboxSetStartDateTime(const char *dateTime, timeMs_t timeNowMs);
void blackboxValidateConfig(void);
bool blackboxMayEditConfig(void);

#ifdef UNIT_TEST
STATIC_UNIT_TESTED void blackboxLogIteration(timeUs_t currentTimeUs);
STATIC_UNIT_TESTED bool blackboxShouldLogPFrame(void);
STATIC_UNIT_TESTED bool blackboxShouldLogIFrame(void);
STATIC_UNIT_TESTED bool blackboxShouldLogGpsHomeFrame(void);
STATIC_UNIT_TESTED bool writeSlowFrameIfNeeded(void);
// Called once every FC loop in order to keep track of how many FC loop iterations have passed
STATIC_UNIT_TESTED void blackboxAdvanceIterationTimers(void);
extern int32_t blackboxSInterval;
extern int32_t blackboxSlowFrameIterationTimer;
#endif
