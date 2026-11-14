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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_TELEMETRY

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "common/utils.h"

#include "config/config.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/airborne.h"
#include "flight/failsafe.h"
#include "flight/governor.h"
#include "flight/mixer.h"
#include "flight/motors.h"
#include "flight/rescue.h"
#include "flight/servos.h"

#include "io/beeper.h"
#include "io/gps.h"

#include "pg/governor.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "telemetry/status.h"

// How long TELEM_STATUS_CONTROL_SATURATED stays set after the mixer last saturated, so a brief
// hit survives until the next telemetry frame and a radio callout can't chatter.
#define CONTROL_SATURATED_HOLD_MS   500

static uint32_t field(uint32_t value, uint32_t mask, int shift)
{
    return (value & mask) << shift;
}

#ifdef USE_GPS
static telemetryGpsFix_e gpsFix(void)
{
    return STATE(GPS_FIX_HOME) ? TELEM_GPS_FIX_HOME :
        (STATE(GPS_FIX) ? TELEM_GPS_FIX_OK : TELEM_GPS_FIX_NONE);
}
#endif

static bool controlSaturated(void)
{
    static timeMs_t saturatedAt;
    static bool seen;

    const timeMs_t now = millis();

    if (mixerTakeStabilizedSaturation()) {
        saturatedAt = now;
        seen = true;
    }

    if (seen && cmp32(now, saturatedAt) >= CONTROL_SATURATED_HOLD_MS) {
        seen = false;
    }

    return seen;
}

static bool overrideActive(void)
{
    if (isServoOverrideActive() || isMixerOverrideActive()) {
        return true;
    }

    for (int i = 0; i < getMotorCount(); i++) {
        if (hasMotorOverride(i)) {
            return true;
        }
    }

    return false;
}

uint32_t telemetrySystemStatus(void)
{
    uint32_t status = 0;

    if (ARMING_FLAG(ARMED))
        status |= TELEM_STATUS_ARMED;
    if (isAirborne())
        status |= TELEM_STATUS_AIRBORNE;
    if (areMotorsRunning())
        status |= TELEM_STATUS_MOTORS_RUNNING;
    if (rxIsReceivingSignal())
        status |= TELEM_STATUS_RX_LINK_UP;

    status |= field(failsafePhase(), TELEM_STATUS_FAILSAFE_PHASE_MASK, TELEM_STATUS_FAILSAFE_PHASE_SHIFT);

#ifdef USE_GPS
    status |= field(gpsFix(), TELEM_STATUS_GPS_FIX_MASK, TELEM_STATUS_GPS_FIX_SHIFT);
    if (gpsIsHealthy())
        status |= TELEM_STATUS_GPS_HEALTHY;
#endif

    if (isSpooledUp())
        status |= TELEM_STATUS_SPOOLED_UP;

    status |= field(getBatteryState(), TELEM_STATUS_BATTERY_MASK, TELEM_STATUS_BATTERY_SHIFT);

    if (controlSaturated())
        status |= TELEM_STATUS_CONTROL_SATURATED;
    if (gyroOverflowDetected())
        status |= TELEM_STATUS_GYRO_OVERFLOW;
#ifdef USE_ACC
    if (sensors(SENSOR_ACC) && !accHasBeenCalibrated())
        status |= TELEM_STATUS_ACC_NOT_CALIBRATED;
#endif
    if (overrideActive())
        status |= TELEM_STATUS_OVERRIDE_ACTIVE;

    status |= field(getRescueState(), TELEM_STATUS_RESCUE_MASK, TELEM_STATUS_RESCUE_SHIFT);

#ifdef USE_BLACKBOX
    if (blackboxIsLogging())
        status |= TELEM_STATUS_BLACKBOX_LOGGING;
#endif

    status |= field(getGovernorState(), TELEM_STATUS_GOVERNOR_MASK, TELEM_STATUS_GOVERNOR_SHIFT);

    return status;
}

uint32_t telemetrySystemConfig(void)
{
    uint32_t config = 0;

    config |= field(getCurrentPidProfileIndex() + 1, TELEM_CONFIG_PROFILE_MASK, TELEM_CONFIG_PID_PROFILE_SHIFT);
    config |= field(getCurrentControlRateProfileIndex() + 1, TELEM_CONFIG_PROFILE_MASK, TELEM_CONFIG_RATES_PROFILE_SHIFT);
    config |= field(getCurrentBatteryProfileIndex() + 1, TELEM_CONFIG_PROFILE_MASK, TELEM_CONFIG_BATTERY_PROFILE_SHIFT);

    if (isConfigDirty())
        config |= TELEM_CONFIG_DIRTY;
    if (isEepromWriteInProgress())
        config |= TELEM_CONFIG_SAVING;
    if (getRebootRequired())
        config |= TELEM_CONFIG_REBOOT_REQUIRED;
    if (isBeeperOn())
        config |= TELEM_CONFIG_BEEPER_ON;

    if (sensors(SENSOR_ACC))
        config |= TELEM_CONFIG_ACC_PRESENT;
    if (sensors(SENSOR_BARO))
        config |= TELEM_CONFIG_BARO_PRESENT;
    if (sensors(SENSOR_MAG))
        config |= TELEM_CONFIG_MAG_PRESENT;
    if (sensors(SENSOR_GPS))
        config |= TELEM_CONFIG_GPS_PRESENT;

#ifdef USE_BLACKBOX
    if (isBlackboxDeviceFull())
        config |= TELEM_CONFIG_BLACKBOX_FULL;
#endif
    if (isRpmSourceActive())
        config |= TELEM_CONFIG_RPM_SOURCE_ACTIVE;

    config |= field(governorConfig()->gov_mode, TELEM_CONFIG_GOVERNOR_MODE_MASK, TELEM_CONFIG_GOVERNOR_MODE_SHIFT);

    return config;
}

#endif /* USE_TELEMETRY */
