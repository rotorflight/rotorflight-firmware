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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Parameter log (blackbox_params).
 *
 * With blackbox_params = CHANGES or FULL:
 *  - the header of each log gets a section that starts with "H Param log:1" and ends with
 *    "H param_end:<lines>,<fnv>". FULL adds a snapshot of every byte of the tracked parameter groups.
 *  - a journal records each change of a tracked parameter group while the log is open, in
 *    CUSTOM_STRING events (101) "P<type><seq> <point> ...*<crc>". The S-frame field paramSeq gives the
 *    number of the last record.
 *
 * The code that changes the configuration brackets its work in an operation (blackboxParamsOpBegin/End).
 * The capture at the end of an operation compares the groups with a shadow copy and records each
 * difference, with the old and the new value and the point in the log. The modules that read the
 * configuration into their runtime state mark each load (blackboxParamsApplied). OFF writes nothing.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

// Sources of a change
#define BBP_SRC_MSP             'm'     // MSP command (arg: the command)
#define BBP_SRC_ADJUSTMENT      'a'     // in-flight adjustment (arg: the function)
#define BBP_SRC_CMS             'k'     // CMS key
#define BBP_SRC_PROFILE         'p'     // profile change (arg: PID, 0x100 | rate, or 0x200 | battery)
#define BBP_SRC_EEPROM_LOAD     'e'
#define BBP_SRC_EEPROM_SAVE     'w'
#define BBP_SRC_STICK_TRIM      't'
#define BBP_SRC_ACC_CALIBRATION 'c'
#define BBP_SRC_TASK            's'     // asynchronous task (arg: bbpTask_e)

typedef enum {
    BBP_TASK_RX_SERIAL_TRIAL = 0,
    BBP_TASK_ESC_SENSOR_TRIAL,
    BBP_TASK_SRXL2_ESC,
} bbpTask_e;

#define BBP_ARG_NONE            0xFFFF

// Modules that read the configuration into their runtime state
typedef enum {
    BBP_LOADER_PID = 0,         // pidLoadProfile
    BBP_LOADER_GOVERNOR,        // governorInitProfile
    BBP_LOADER_RESCUE,          // rescueInitProfile
    BBP_LOADER_SETPOINT,        // setpointInitProfile
    BBP_LOADER_GYRO_FILTER,     // gyroInitFilters
    BBP_LOADER_RPM_FILTER,      // rpmFilterInit
    BBP_LOADER_MIXER,           // mixerInitConfig
    BBP_LOADER_RC_CONTROLS,     // rcControlsInit
    BBP_LOADER_ACTIVATE,        // activateConfig
    BBP_LOADER_FEATURE,         // featureInit
    BBP_LOADER_COUNT
} bbpLoader_e;

// Markers: events that explain stalls or limits of the journal
typedef enum {
    BBP_MARKER_EESAVE = 0,      // a: duration of the EEPROM write in us
    BBP_MARKER_EELOAD,
    BBP_MARKER_ESCPARAM,        // a: length, b: FNV-1 hash of the ESC parameters that were sent
    BBP_MARKER_SHADOW_RESET,
    BBP_MARKER_COUNT
} bbpMarker_e;

#ifdef USE_BLACKBOX

// blackboxInit(): find the tracked parameter groups
void blackboxParamsInit(void);

// End of init(): keep the boot values of the groups that their modules cache at boot, and set the shadow
void blackboxParamsBoot(void);

// blackboxStart(): latch blackbox_params for this log
void blackboxParamsStart(void);

// Each blackboxUpdate() from WAIT_FOR_READY to CACHE_FLUSH: compare one group with the previous log
void blackboxParamsHeaderTick(void);

// blackboxWriteSysinfo(): write the next part of the parameter section. True when it is complete.
bool blackboxParamsWriteHeader(void);

// The log became RUNNING after the header (T0)
void blackboxParamsRunning(void);

// After each logged frame, and each iteration while PAUSED: write at most one journal event
void blackboxParamsAfterFrame(void);

// Before LOG_END: true while records wait, for at most 100 ms
bool blackboxParamsHoldLogEnd(void);

// Just before LOG_END: write the end record
void blackboxParamsEnd(void);

// SHUTTING_DOWN, FULL or START_ERASE: the log ended. Records that it did not get make the shadow invalid.
void blackboxParamsStop(void);

// S-frame paramSeq: the number of the last journal record of this log (0: none)
uint32_t blackboxParamsSeq(void);

// The CLI overwrote the PG copies (the shadow)
void blackboxParamsShadowLost(void);

// Operations: a mutating entry point. They nest, and the outermost one gives the source.
void blackboxParamsOpBegin(char source, uint16_t arg);
void blackboxParamsOpEnd(void);

// MSP: an operation for each command that can write the configuration. True when the operation began.
bool blackboxParamsMspBegin(int16_t cmd);

// A loader read the configuration (slot: the PID or rate profile, or -1)
void blackboxParamsApplied(bbpLoader_e loader, int slot);

void blackboxParamsMarker(bbpMarker_e marker, uint32_t a, uint32_t b);

// The parameter log of this log is open
bool blackboxParamsActive(void);

// A number that changes when a capture finds a change of the configuration, a loader runs or a log starts. Equal
// numbers: a deterministic setter that had no effect has no effect again.
uint32_t blackboxParamsChangeCount(void);

#else

static inline void blackboxParamsBoot(void) { }
static inline void blackboxParamsShadowLost(void) { }
static inline void blackboxParamsOpBegin(char source, uint16_t arg)
{
    (void)source;
    (void)arg;
}
static inline void blackboxParamsOpEnd(void) { }
static inline bool blackboxParamsMspBegin(int16_t cmd)
{
    (void)cmd;
    return false;
}
static inline void blackboxParamsApplied(bbpLoader_e loader, int slot)
{
    (void)loader;
    (void)slot;
}
static inline void blackboxParamsMarker(bbpMarker_e marker, uint32_t a, uint32_t b)
{
    (void)marker;
    (void)a;
    (void)b;
}
static inline bool blackboxParamsActive(void)
{
    return false;
}
static inline uint32_t blackboxParamsChangeCount(void)
{
    return 0;
}

#endif
