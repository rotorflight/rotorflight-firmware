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

#pragma once

#include <stdint.h>

/*
 * Packed status words for the TELEM_SYSTEM_STATUS and TELEM_SYSTEM_CONFIG telemetry sensors.
 *
 * The radio-side Lua decodes these by bit position, so any change to the layout must ship
 * together with the matching Lua update. Bit 31 of both words stays clear because SmartPort
 * carries the value as a signed 32-bit int.
 *
 * Fields shared with Wingflight sit at the same bit positions there. Bits that are Wingflight-only
 * (backup RX, GPS navigation, autotrim, logic conditions, TV profile) carry the heli fields here,
 * or are reserved and read as 0.
 *
 * A field that is not compiled into a build (no GPS, no ACC, ...) reads as 0.
 */

/** TELEM_SYSTEM_STATUS: live state that drives radio callouts **/

#define TELEM_STATUS_ARMED                  (1u << 0)   // armingFlags ARMED
#define TELEM_STATUS_AIRBORNE               (1u << 1)   // isAirborne()
#define TELEM_STATUS_MOTORS_RUNNING         (1u << 2)   // areMotorsRunning()
#define TELEM_STATUS_RX_LINK_UP             (1u << 3)   // RX receiving

// Bits 4-5 reserved (backup RX in Wingflight).

#define TELEM_STATUS_FAILSAFE_PHASE_SHIFT   6           // failsafePhase_e, 0 = idle
#define TELEM_STATUS_FAILSAFE_PHASE_MASK    0x7u

#define TELEM_STATUS_GPS_FIX_SHIFT          9           // telemetryGpsFix_e
#define TELEM_STATUS_GPS_FIX_MASK           0x3u
#define TELEM_STATUS_GPS_HEALTHY            (1u << 11)  // GPS module is talking to the FC

#define TELEM_STATUS_SPOOLED_UP             (1u << 12)  // isSpooledUp()

// Bit 13 reserved.

#define TELEM_STATUS_BATTERY_SHIFT          14          // batteryState_e
#define TELEM_STATUS_BATTERY_MASK           0x7u

#define TELEM_STATUS_CONTROL_SATURATED      (1u << 17)  // stabilized cyclic/yaw/collective hit its mixer limit recently
#define TELEM_STATUS_GYRO_OVERFLOW          (1u << 18)  // gyroOverflowDetected()
#define TELEM_STATUS_ACC_NOT_CALIBRATED     (1u << 19)  // ACC present but never calibrated
#define TELEM_STATUS_OVERRIDE_ACTIVE        (1u << 20)  // Configurator servo/motor/mixer test override on

#define TELEM_STATUS_RESCUE_SHIFT           21          // rescueState_e
#define TELEM_STATUS_RESCUE_MASK            0x7u

#define TELEM_STATUS_BLACKBOX_LOGGING       (1u << 24)  // Blackbox is writing a log

#define TELEM_STATUS_GOVERNOR_SHIFT         25          // govState_e
#define TELEM_STATUS_GOVERNOR_MASK          0xFu

// Bits 29-30 spare, bit 31 reserved.

/** TELEM_SYSTEM_CONFIG: slow-changing configuration and hardware state **/

#define TELEM_CONFIG_PID_PROFILE_SHIFT      0           // 1-based profile number
#define TELEM_CONFIG_RATES_PROFILE_SHIFT    3
#define TELEM_CONFIG_BATTERY_PROFILE_SHIFT  6
#define TELEM_CONFIG_PROFILE_MASK           0x7u

// Bits 9-11 reserved (TV profile in Wingflight).

#define TELEM_CONFIG_DIRTY                  (1u << 12)  // settings changed but not saved
#define TELEM_CONFIG_SAVING                 (1u << 13)  // EEPROM write in progress
#define TELEM_CONFIG_REBOOT_REQUIRED        (1u << 14)
#define TELEM_CONFIG_BEEPER_ON              (1u << 15)  // beeper sounding (e.g. lost model)
#define TELEM_CONFIG_ACC_PRESENT            (1u << 16)
#define TELEM_CONFIG_BARO_PRESENT           (1u << 17)
#define TELEM_CONFIG_MAG_PRESENT            (1u << 18)
#define TELEM_CONFIG_GPS_PRESENT            (1u << 19)

// Bit 20 reserved (backup RX in Wingflight).

#define TELEM_CONFIG_BLACKBOX_FULL          (1u << 21)
#define TELEM_CONFIG_RPM_SOURCE_ACTIVE      (1u << 22)  // motor RPM telemetry is arriving

#define TELEM_CONFIG_GOVERNOR_MODE_SHIFT    23          // govMode_e
#define TELEM_CONFIG_GOVERNOR_MODE_MASK     0x7u

// Bits 26-30 spare, bit 31 reserved.

typedef enum {
    TELEM_GPS_FIX_NONE = 0,
    TELEM_GPS_FIX_OK,
    TELEM_GPS_FIX_HOME,             // fix, and home position captured
} telemetryGpsFix_e;

uint32_t telemetrySystemStatus(void);
uint32_t telemetrySystemConfig(void);
