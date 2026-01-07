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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_fielddefs.h"

#include "build/debug.h"

#include "cli/cli.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/light_led.h"
#include "drivers/motor.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/freq.h"
#include "drivers/sbus_output.h"

#include "fc/rc_rates.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/stats.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/wiggle.h"

#if defined(USE_DYN_NOTCH_FILTER)
#include "flight/dyn_notch_filter.h"
#endif

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/trainer.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"
#include "flight/governor.h"
#include "flight/rescue.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/serial.h"
#include "io/servos.h"
#include "io/statusindicator.h"
#include "io/vtx_control.h"
#include "io/vtx_rtc6705.h"

#include "msp/msp_serial.h"

#include "osd/osd.h"

#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"

#include "core.h"

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

enum {
    ARMING_NOT_DELAYED = 0,
    ARMING_DELAYED = 1,
};

enum {
    WIGGLE_NOT_DONE = 0,
    WIGGLE_TRIGGERED,
    WIGGLE_DONE,
};

#define MOTORS_GRACE_TIME_US 2000000

#if defined(USE_GPS) || defined(USE_MAG)
int16_t magHold;
#endif

static FAST_DATA_ZERO_INIT uint16_t pidUpdateCounter;

static timeUs_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

static int lastArmingDisabledReason = 0;
static timeUs_t lastDisarmTimeUs;

static int armingWiggle = WIGGLE_NOT_DONE;
static int armingDelayed = ARMING_NOT_DELAYED;
static int armingEnabledWiggle = WIGGLE_NOT_DONE;

static timeMs_t armingErrorWiggleTime = 0;

static bool isCalibrating(void)
{
    return (sensors(SENSOR_GYRO) && !gyroIsCalibrationComplete())
#ifdef USE_ACC
        || (sensors(SENSOR_ACC) && !accIsCalibrationComplete())
#endif
#ifdef USE_BARO
        || (sensors(SENSOR_BARO) && !baroIsCalibrationComplete())
#endif
#ifdef USE_MAG
        || (sensors(SENSOR_MAG) && !compassIsCalibrationComplete())
#endif
        ;
}

bool isTryingToArm()
{
    return (armingDelayed != ARMING_NOT_DELAYED);
}

void resetTryingToArm()
{
    armingDelayed = ARMING_NOT_DELAYED;
}

void resetArmingDisabled(void)
{
    lastArmingDisabledReason = 0;
}

#ifdef USE_ACC
static bool accNeedsCalibration(void)
{
    if (sensors(SENSOR_ACC)) {
#if 1
        // Always require ACC calibration
        return !accHasBeenCalibrated();
#else
        // Check to see if the ACC has already been calibrated
        if (accHasBeenCalibrated()) {
            return false;
        }

        // We've determined that there's a detected ACC that has not
        // yet been calibrated. Check to see if anything is using the
        // ACC that would be affected by the lack of calibration.

        // Check for any configured modes that use the ACC
        if (isModeActivationConditionPresent(BOXANGLE) ||
            isModeActivationConditionPresent(BOXHORIZON) ||
            isModeActivationConditionPresent(BOXTRAINER) ||
            isModeActivationConditionPresent(BOXRESCUE) ||
            isModeActivationConditionPresent(BOXGPSRESCUE) ||
            isModeActivationConditionPresent(BOXCALIB)) {
            return true;
        }

#ifdef USE_OSD
        // Check for any enabled OSD elements that need the ACC
        if (featureIsEnabled(FEATURE_OSD)) {
            if (osdNeedsAccelerometer()) {
                return true;
            }
        }
#endif

#ifdef USE_GPS_RESCUE
        // Check if failsafe will use GPS Rescue
        if (failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE) {
            return true;
        }
#endif
#endif
    }

    return false;
}
#endif

void updateArmingStatus(void)
{
    if (ARMING_FLAG(ARMED))
    {
        LED0_ON;
        armingErrorWiggleTime = 0;
    }
    else
    {
        // Check if the power on arming grace time has elapsed
        if ((getArmingDisableFlags() & ARMING_DISABLED_BOOT_GRACE_TIME)
            && (millis() >= armingConfig()->power_on_arming_grace_time * 1000)
#ifdef USE_DSHOT
            // We also need to prevent arming until it's possible to send DSHOT commands.
            && (!isMotorProtocolDshot() || dshotStreamingCommandsAreEnabled())
#endif
        ) {
            // If so, unset the grace time arming disable flag
            unsetArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);
        }

        // If switch is used for arming then check it is not defaulting to on when the RX link recovers from a fault
        if (!isUsingSticksForArming()) {
            static bool hadRx = false;
            const bool haveRx = rxIsReceivingSignal();

            const bool justGotRxBack = !hadRx && haveRx;

            if (justGotRxBack && IS_RC_MODE_ACTIVE(BOXARM)) {
                // If the RX has just started to receive a signal again and the arm switch is on, apply arming restriction
                setArmingDisabled(ARMING_DISABLED_BAD_RX_RECOVERY);
            } else if (haveRx && !IS_RC_MODE_ACTIVE(BOXARM)) {
                // If RX signal is OK and the arm switch is off, remove arming restriction
                unsetArmingDisabled(ARMING_DISABLED_BAD_RX_RECOVERY);
            }

            hadRx = haveRx;
        }

        if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
            setArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        }

        if (!isThrottleOff()) {
            setArmingDisabled(ARMING_DISABLED_THROTTLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_THROTTLE);
        }

        if (!isUpright()) {
            setArmingDisabled(ARMING_DISABLED_ANGLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ANGLE);
        }

        if (getMaxRealTimeLoad() > 750 || getAverageCPULoad() > 750 || getAverageSystemLoad() > 750) {
            setArmingDisabled(ARMING_DISABLED_LOAD);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_LOAD);
        }

        if (isCalibrating()) {
            setArmingDisabled(ARMING_DISABLED_CALIBRATING);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_CALIBRATING);
        }

        if (isModeActivationConditionPresent(BOXPREARM)) {
            if (IS_RC_MODE_ACTIVE(BOXPREARM) && !ARMING_FLAG(WAS_ARMED_WITH_PREARM)) {
                unsetArmingDisabled(ARMING_DISABLED_NOPREARM);
            } else {
                setArmingDisabled(ARMING_DISABLED_NOPREARM);
            }
        }

#ifdef USE_GPS_RESCUE
        if (gpsRescueIsConfigured()) {
            if (gpsRescueConfig()->allowArmingWithoutFix || STATE(GPS_FIX) || ARMING_FLAG(WAS_EVER_ARMED)) {
                unsetArmingDisabled(ARMING_DISABLED_GPS);
            } else {
                setArmingDisabled(ARMING_DISABLED_GPS);
            }
            if (IS_RC_MODE_ACTIVE(BOXGPSRESCUE)) {
                setArmingDisabled(ARMING_DISABLED_RESC);
            } else {
                unsetArmingDisabled(ARMING_DISABLED_RESC);
            }
        }
#endif

#ifdef USE_DSHOT_BITBANG
        if (isDshotBitbangActive(&motorConfig()->dev) && dshotBitbangGetStatus() != DSHOT_BITBANG_STATUS_OK) {
            setArmingDisabled(ARMING_DISABLED_DSHOT_BITBANG);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_DSHOT_BITBANG);
        }
#endif

        if (IS_RC_MODE_ACTIVE(BOXPARALYZE)) {
            setArmingDisabled(ARMING_DISABLED_PARALYZE);
        }

#ifdef USE_ACC
        if (accNeedsCalibration()) {
            setArmingDisabled(ARMING_DISABLED_ACC_CALIBRATION);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ACC_CALIBRATION);
        }
#endif

        if (!isMotorProtocolEnabled()) {
            setArmingDisabled(ARMING_DISABLED_MOTOR_PROTOCOL);
        }

        if (!isUsingSticksForArming()) {
            /* Ignore ARMING_DISABLED_CALIBRATING if we are going to calibrate gyro on first arm */
            bool ignoreGyro = armingConfig()->gyro_cal_on_first_arm
                && !(getArmingDisableFlags() & ~(ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_CALIBRATING));

            // If arming is disabled and the ARM switch is on
            if (isArmingDisabled()
                && !ignoreGyro
                && IS_RC_MODE_ACTIVE(BOXARM)) {
                setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            } else if (!IS_RC_MODE_ACTIVE(BOXARM)) {
                unsetArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            }
        }

        if (!isArmingDisabled()) {
            warningLedDisable();
            if (!ARMING_FLAG(WAS_EVER_ARMED) && wiggleEnabled(WIGGLE_READY)) {
                if (armingEnabledWiggle == WIGGLE_NOT_DONE) {
                    armingEnabledWiggle = WIGGLE_DONE;
                    wiggleTrigger(WIGGLE_READY, 0);
                }
            }
        }
        else {
            warningLedFlash();
            if (!ARMING_FLAG(WAS_EVER_ARMED) &&
                (wiggleEnabled(WIGGLE_ERROR) || wiggleEnabled(WIGGLE_FATAL)))
            {
                const bitmap_t flags = getArmingDisableFlags();
                if ((flags & ARMING_DISABLED_ARM_SWITCH) &&
                    !(flags & (
                        ARMING_DISABLED_BOOT_GRACE_TIME |
                        ARMING_DISABLED_MSP |
                        ARMING_DISABLED_CLI |
                        ARMING_DISABLED_CMS_MENU
                    )))
                {
                    const timeMs_t now = millis();
                    if (flags & (
                            ARMING_DISABLED_NO_GYRO |
                            ARMING_DISABLED_LOAD |
                            ARMING_DISABLED_GOVERNOR |
                            ARMING_DISABLED_RPMFILTER |
                            ARMING_DISABLED_REBOOT_REQUIRED |
                            ARMING_DISABLED_ACC_CALIBRATION |
                            ARMING_DISABLED_MOTOR_PROTOCOL |
                            ARMING_DISABLED_DSHOT_BITBANG
                        ))
                    {
                        if (now >= armingErrorWiggleTime + 5000) {
                            armingErrorWiggleTime = now;
                            wiggleTrigger(WIGGLE_FATAL, 2500);
                        }
                    }
                    else
                    {
                        const timeMs_t now = millis();
                        if (now >= armingErrorWiggleTime + 3000) {
                            armingErrorWiggleTime = now;
                            wiggleTrigger(WIGGLE_ERROR, 750);
                        }
                    }
                }
            }
        }

        warningLedUpdate();
    }
}

void disarm(flightLogDisarmReason_e reason)
{
    if (ARMING_FLAG(ARMED)) {
        ENABLE_ARMING_FLAG(WAS_EVER_ARMED);
        DISABLE_ARMING_FLAG(ARMED);
        lastDisarmTimeUs = micros();

        armingDelayed = ARMING_NOT_DELAYED;
        armingWiggle = WIGGLE_NOT_DONE;
        armingEnabledWiggle = WIGGLE_DONE;

#ifdef USE_BLACKBOX
        flightLogEvent_disarm_t eventData;
        eventData.reason = reason;
        blackboxLogEvent(FLIGHT_LOG_EVENT_DISARM, (flightLogEventData_t*)&eventData);
#else
        UNUSED(reason);
#endif
        BEEP_OFF;

        bool saveRequired = isConfigDirty();
#ifdef USE_PERSISTENT_STATS
        saveRequired |= statsOnDisarm();
#endif

        // let the disarming process complete and then execute the actual save
        if (saveRequired) {
            writeEEPROMDelayed(500000);
        }

        if (!getArmingDisableFlags()) {
            beeper(BEEPER_DISARMING);      // emit disarm tone
        }
    }
}

void tryArm(void)
{
    if (armingConfig()->gyro_cal_on_first_arm) {
        gyroStartCalibration(true);
    }

    updateArmingStatus();

    if (!isArmingDisabled()) {
        if (ARMING_FLAG(ARMED)) {
            return;
        }

        if (!ARMING_FLAG(WAS_EVER_ARMED) && wiggleEnabled(WIGGLE_ARMED)) {
            if (armingWiggle == WIGGLE_NOT_DONE) {
                armingDelayed = ARMING_DELAYED;
                armingWiggle = WIGGLE_TRIGGERED;
                wiggleTrigger(WIGGLE_ARMED, 0);
                return;
            }
            else if (armingWiggle == WIGGLE_TRIGGERED) {
                if (wiggleActive())
                    return;
                armingWiggle = WIGGLE_DONE;
                armingDelayed = ARMING_NOT_DELAYED;
            }
        }

        const timeUs_t currentTimeUs = micros();

#ifdef USE_DSHOT
        if (currentTimeUs - getLastDshotBeaconCommandTimeUs() < DSHOT_BEACON_GUARD_DELAY_US) {
            armingDelayed = ARMING_DELAYED;
            return;
        }
#endif

#ifdef USE_OSD
        osdSuppressStats(false);
#endif

        ENABLE_ARMING_FLAG(ARMED);

        armingDelayed = ARMING_NOT_DELAYED;
        armingWiggle = WIGGLE_NOT_DONE;
        armingEnabledWiggle = WIGGLE_DONE;

        resetMotorOverride();

#ifdef USE_ACRO_TRAINER
        acroTrainerReset();
#endif

        if (isModeActivationConditionPresent(BOXPREARM)) {
            ENABLE_ARMING_FLAG(WAS_ARMED_WITH_PREARM);
        }

#if defined(USE_DYN_NOTCH_FILTER)
        resetMaxFFT();
#endif

        disarmAt = currentTimeUs + armingConfig()->auto_disarm_delay * 1e6;   // start disarm timeout, will be extended when throttle is nonzero

        lastArmingDisabledReason = 0;

#ifdef USE_GPS
        GPS_reset_home_position();

        //beep to indicate arming
        if (featureIsEnabled(FEATURE_GPS)) {
            if (STATE(GPS_FIX) && gpsSol.numSat >= 5) {
                beeper(BEEPER_ARMING_GPS_FIX);
            } else {
                beeper(BEEPER_ARMING_GPS_NO_FIX);
            }
        } else {
            beeper(BEEPER_ARMING);
        }
#else
        beeper(BEEPER_ARMING);
#endif

#ifdef USE_PERSISTENT_STATS
        statsOnArm();
#endif
    } else {
        armingDelayed = ARMING_NOT_DELAYED;
        armingWiggle = WIGGLE_NOT_DONE;

        if (!isFirstArmingGyroCalibrationRunning()) {
            int armingDisabledReason = ffs(getArmingDisableFlags());
            if (lastArmingDisabledReason != armingDisabledReason) {
                lastArmingDisabledReason = armingDisabledReason;
                beeperWarningBeeps(armingDisabledReason);
            }
        }
    }
}

#ifdef USE_VTX_CONTROL
static bool canUpdateVTX(void)
{
#ifdef USE_VTX_RTC6705
    return vtxRTC6705CanUpdate();
#endif
    return true;
}
#endif


/*
 * processRx called from taskUpdateRxMain
 */
bool processRx(timeUs_t currentTimeUs)
{
    if (!calculateRxChannelsAndUpdateFailsafe(currentTimeUs)) {
        return false;
    }

    updateRcRefreshRate(currentTimeUs);

    updateRSSI(currentTimeUs);

    if (currentTimeUs > FAILSAFE_POWER_ON_DELAY_US && !failsafeIsMonitoring()) {
        failsafeStartMonitoring();
    }

    return true;
}

void processRxModes(timeUs_t currentTimeUs)
{
    static bool armedBeeperOn = false;
#ifdef USE_TELEMETRY
    static bool sharedPortTelemetryEnabled = false;
#endif
    const bool throttleOff = isThrottleOff();

    // When armed and motors aren't spinning, do beeps and then disarm
    // board after delay so users without buzzer won't lose fingers.
    // mixTable constrains motor commands, so checking  throttleStatus is enough
    const timeUs_t autoDisarmDelayUs = armingConfig()->auto_disarm_delay * 1e6;
    if (ARMING_FLAG(ARMED)
        && !FLIGHT_MODE(GPS_RESCUE_MODE)  // disable auto-disarm when GPS Rescue is active
    ) {
        if (isUsingSticksForArming()) {
            if (throttleOff) {
                if ((autoDisarmDelayUs > 0) && (currentTimeUs > disarmAt)) {
                    // auto-disarm configured and delay is over
                    disarm(DISARM_REASON_THROTTLE_TIMEOUT);
                    armedBeeperOn = false;
                } else {
                    // still armed; do warning beeps while armed
                    beeper(BEEPER_ARMED);
                    armedBeeperOn = true;
                }
            } else {
                // throttle is not low - extend disarm time
                disarmAt = currentTimeUs + autoDisarmDelayUs;

                if (armedBeeperOn) {
                    beeperSilence();
                    armedBeeperOn = false;
                }
            }
        } else {
            // arming is via AUX switch; beep while throttle low
            if (throttleOff) {
                beeper(BEEPER_ARMED);
                armedBeeperOn = true;
            } else if (armedBeeperOn) {
                beeperSilence();
                armedBeeperOn = false;
            }
        }
    } else {
        disarmAt = currentTimeUs + autoDisarmDelayUs;  // extend auto-disarm timer
    }

    if (!(IS_RC_MODE_ACTIVE(BOXPARALYZE) && !ARMING_FLAG(ARMED))
#ifdef USE_CMS
        && !cmsInMenu
#endif
        ) {
        processRcStickPositions();
    }

    updateActivatedModes();

    if (!cliMode &&
#ifdef USE_CMS
        !cmsInMenu &&
#endif
        !(IS_RC_MODE_ACTIVE(BOXPARALYZE) && !ARMING_FLAG(ARMED)))
    {
        processRcAdjustments();
    }

    if (sensors(SENSOR_ACC)) {
#ifdef USE_GPS_RESCUE
        if (ARMING_FLAG(ARMED) &&
            (IS_RC_MODE_ACTIVE(BOXGPSRESCUE) ||
             (failsafeIsActive() && failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE))) {
            ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
        } else {
            DISABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
        }
#endif

        if (IS_RC_MODE_ACTIVE(BOXRESCUE)) {
            ENABLE_FLIGHT_MODE(RESCUE_MODE);
        } else {
            DISABLE_FLIGHT_MODE(RESCUE_MODE);
        }

        if (IS_RC_MODE_ACTIVE(BOXANGLE)) {
            ENABLE_FLIGHT_MODE(ANGLE_MODE);
            DISABLE_FLIGHT_MODE(HORIZON_MODE);
            DISABLE_FLIGHT_MODE(TRAINER_MODE);
        }
        else if (IS_RC_MODE_ACTIVE(BOXHORIZON)) {
            DISABLE_FLIGHT_MODE(ANGLE_MODE);
            ENABLE_FLIGHT_MODE(HORIZON_MODE);
            DISABLE_FLIGHT_MODE(TRAINER_MODE);
        }
#ifdef USE_ACRO_TRAINER
        else if (IS_RC_MODE_ACTIVE(BOXTRAINER)) {
            DISABLE_FLIGHT_MODE(ANGLE_MODE);
            DISABLE_FLIGHT_MODE(HORIZON_MODE);
            ENABLE_FLIGHT_MODE(TRAINER_MODE);
        }
#endif
        else {
            DISABLE_FLIGHT_MODE(ANGLE_MODE);
            DISABLE_FLIGHT_MODE(HORIZON_MODE);
            DISABLE_FLIGHT_MODE(TRAINER_MODE);
        }
    }

#ifdef USE_ACRO_TRAINER
    acroTrainerSetState(FLIGHT_MODE(TRAINER_MODE));
#endif // USE_ACRO_TRAINER

    if (!IS_RC_MODE_ACTIVE(BOXPREARM) && ARMING_FLAG(WAS_ARMED_WITH_PREARM)) {
        DISABLE_ARMING_FLAG(WAS_ARMED_WITH_PREARM);
    }

#ifdef USE_TELEMETRY
    if (featureIsEnabled(FEATURE_TELEMETRY)) {
        bool enableSharedPortTelemetry = (!isModeActivationConditionPresent(BOXTELEMETRY) && ARMING_FLAG(ARMED)) || (isModeActivationConditionPresent(BOXTELEMETRY) && IS_RC_MODE_ACTIVE(BOXTELEMETRY));
        if (enableSharedPortTelemetry && !sharedPortTelemetryEnabled) {
            mspSerialReleaseSharedTelemetryPorts();
            telemetryCheckState();

            sharedPortTelemetryEnabled = true;
        } else if (!enableSharedPortTelemetry && sharedPortTelemetryEnabled) {
            // the telemetry state must be checked immediately so that shared serial ports are released.
            telemetryCheckState();
            mspSerialAllocatePorts();

            sharedPortTelemetryEnabled = false;
        }
    }
#endif

#ifdef USE_VTX_CONTROL
    vtxUpdateActivatedChannel();

    if (canUpdateVTX()) {
        handleVTXControlButton();
    }
#endif
}

#ifdef USE_TELEMETRY
#define GYRO_TEMP_READ_DELAY_US 3e6    // Only read the gyro temp every 3 seconds
void subTaskTelemetryPollSensors(timeUs_t currentTimeUs)
{
    static timeUs_t lastGyroTempTimeUs = 0;

    if (cmpTimeUs(currentTimeUs, lastGyroTempTimeUs) >= GYRO_TEMP_READ_DELAY_US) {
        // Read out gyro temperature if used for telemmetry
        gyroReadTemperature();
        lastGyroTempTimeUs = currentTimeUs;
    }
}
#endif

static void subTaskPosition(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    positionUpdate();
}

static void subTaskSetpoint(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    setpointUpdate();
    rescueUpdate();
}

static void subTaskPidController(timeUs_t currentTimeUs)
{
    if (debugMode == DEBUG_CYCLETIME) {
        static uint32_t previousUpdateTime;
        uint32_t startTime = micros();
        uint32_t currentDeltaTime = startTime - previousUpdateTime;
        debug[0] = getTaskDeltaTimeUs(TASK_SELF);
        debug[1] = getAverageCPULoadPercent();
        debug[2] = currentDeltaTime;
        debug[3] = currentDeltaTime - gyro.targetLooptime;
        previousUpdateTime = startTime;
    }

    pidController(currentPidProfile, currentTimeUs);

}

static void subTaskMixerUpdate(timeUs_t currentTimeUs)
{
    mixerUpdate(currentTimeUs);
}

static void subTaskMotorsServosUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (currentTimeUs > MOTORS_GRACE_TIME_US) {
#ifdef USE_SERVOS
        servoUpdate();
#endif
#ifdef USE_MOTOR
        motorUpdate(currentTimeUs);
#endif
    }
}

static void subTaskFilterUpdate(timeUs_t currentTimeUs)
{
#ifdef USE_FREQ_SENSOR
    freqUpdate();
#endif

#ifdef USE_DYN_LPF
    dynLpfUpdate(currentTimeUs);
#endif

#ifdef USE_DYN_NOTCH_FILTER
    if (isDynNotchActive())
        dynNotchUpdate();
#endif

#ifdef USE_RPM_FILTER
    rpmFilterUpdate();
#endif
}

static void subTaskBlackboxUpdate(timeUs_t currentTimeUs)
{
#ifdef USE_BLACKBOX
    if (!cliMode && blackboxConfig()->device) {
        blackboxUpdate(currentTimeUs);
    }
#else
    UNUSED(currentTimeUs);
#endif
}

static void subTaskBlackboxFlush(timeUs_t currentTimeUs)
{
#ifdef USE_BLACKBOX
    if (blackboxConfig()->device) {
        blackboxFlush(currentTimeUs);
    }
#else
    UNUSED(currentTimeUs);
#endif
}

void taskGyroSample(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    gyroUpdate();
}

bool gyroFilterReady(void)
{
    return (pidUpdateCounter % activeFilterLoopDenom == 0);
}

bool pidLoopReady(void)
{
    return true;
}

void taskFiltering(timeUs_t currentTimeUs)
{
    DEBUG_TIME_START(CYCLETIME, 5);

    if (debugMode == DEBUG_CYCLETIME) {
        static uint32_t previousUpdateTime;
        uint32_t startTime = micros();
        uint32_t currentDeltaTime = startTime - previousUpdateTime;
        debug[4] = getTaskDeltaTimeUs(TASK_SELF);
        debug[6] = currentDeltaTime;
        debug[7] = currentDeltaTime - gyro.filterLooptime;
        previousUpdateTime = startTime;
    }

    gyroFiltering(currentTimeUs);

    DEBUG_TIME_END(CYCLETIME, 5);
}

void taskMainPidLoop(timeUs_t currentTimeUs)
{
    DEBUG_TIME_START(PIDLOOP, pidUpdateCounter & 7);

    if (activePidLoopDenom == 1) {
        subTaskPosition(currentTimeUs);
        subTaskSetpoint(currentTimeUs);
        subTaskPidController(currentTimeUs);
        subTaskMixerUpdate(currentTimeUs);
        subTaskMotorsServosUpdate(currentTimeUs);
        subTaskFilterUpdate(currentTimeUs);
        subTaskBlackboxUpdate(currentTimeUs);
        subTaskBlackboxFlush(currentTimeUs);
    }
    else if (activePidLoopDenom == 2) {
        switch (pidUpdateCounter) {
            case 0:
                subTaskPosition(currentTimeUs);
                subTaskSetpoint(currentTimeUs);
                subTaskPidController(currentTimeUs);
                subTaskBlackboxFlush(currentTimeUs);
                break;
            case 1:
                subTaskMixerUpdate(currentTimeUs);
                subTaskMotorsServosUpdate(currentTimeUs);
                subTaskFilterUpdate(currentTimeUs);
                subTaskBlackboxUpdate(currentTimeUs);
                break;
        }
    }
    else if (activePidLoopDenom == 3) {
        switch (pidUpdateCounter) {
            case 0:
                subTaskPosition(currentTimeUs);
                subTaskSetpoint(currentTimeUs);
                subTaskPidController(currentTimeUs);
                subTaskMixerUpdate(currentTimeUs);
                break;
            case 1:
                subTaskMotorsServosUpdate(currentTimeUs);
                subTaskBlackboxUpdate(currentTimeUs);
                break;
            case 2:
                subTaskFilterUpdate(currentTimeUs);
                subTaskBlackboxFlush(currentTimeUs);
                break;
        }
    }
    else if (activePidLoopDenom == 4) {
        switch (pidUpdateCounter) {
            case 0:
                subTaskPosition(currentTimeUs);
                subTaskSetpoint(currentTimeUs);
                subTaskPidController(currentTimeUs);
                break;
            case 1:
                subTaskMixerUpdate(currentTimeUs);
                subTaskMotorsServosUpdate(currentTimeUs);
                break;
            case 2:
                subTaskFilterUpdate(currentTimeUs);
                subTaskBlackboxUpdate(currentTimeUs);
                break;
            case 3:
                subTaskBlackboxFlush(currentTimeUs);
                break;
        }
    }
    else if (activePidLoopDenom == 5) {
        switch (pidUpdateCounter) {
            case 0:
                subTaskPosition(currentTimeUs);
                subTaskSetpoint(currentTimeUs);
                subTaskPidController(currentTimeUs);
                break;
            case 1:
                subTaskMixerUpdate(currentTimeUs);
                break;
            case 2:
                subTaskMotorsServosUpdate(currentTimeUs);
                subTaskFilterUpdate(currentTimeUs);
                break;
            case 3:
                subTaskBlackboxUpdate(currentTimeUs);
                break;
            case 4:
                subTaskBlackboxFlush(currentTimeUs);
                break;
        }
    }
    else if (activePidLoopDenom == 6) {
        switch (pidUpdateCounter) {
            case 0:
                subTaskPosition(currentTimeUs);
                subTaskSetpoint(currentTimeUs);
                break;
            case 1:
                subTaskPidController(currentTimeUs);
                break;
            case 2:
                subTaskMixerUpdate(currentTimeUs);
                break;
            case 3:
                subTaskMotorsServosUpdate(currentTimeUs);
                subTaskFilterUpdate(currentTimeUs);
                break;
            case 4:
                subTaskBlackboxUpdate(currentTimeUs);
                break;
            case 5:
                subTaskBlackboxFlush(currentTimeUs);
                break;
        }
    }
    else if (activePidLoopDenom == 7) {
        switch (pidUpdateCounter) {
            case 0:
                subTaskPosition(currentTimeUs);
                subTaskSetpoint(currentTimeUs);
                break;
            case 1:
                subTaskPidController(currentTimeUs);
                break;
            case 2:
                subTaskMixerUpdate(currentTimeUs);
                break;
            case 3:
                subTaskMotorsServosUpdate(currentTimeUs);
                break;
            case 4:
                subTaskFilterUpdate(currentTimeUs);
                break;
            case 5:
                subTaskBlackboxUpdate(currentTimeUs);
                break;
            case 6:
                subTaskBlackboxFlush(currentTimeUs);
                break;
        }
    }
    else if (activePidLoopDenom >= 8) {
        switch (pidUpdateCounter) {
            case 0:
                subTaskPosition(currentTimeUs);
                break;
            case 1:
                subTaskSetpoint(currentTimeUs);
                break;
            case 2:
                subTaskPidController(currentTimeUs);
                break;
            case 3:
                subTaskMixerUpdate(currentTimeUs);
                break;
            case 4:
                subTaskMotorsServosUpdate(currentTimeUs);
                break;
            case 5:
                subTaskFilterUpdate(currentTimeUs);
                break;
            case 6:
                subTaskBlackboxUpdate(currentTimeUs);
                break;
            case 7:
                subTaskBlackboxFlush(currentTimeUs);
                break;
        }
    }

    DEBUG_TIME_END(PIDLOOP, pidUpdateCounter & 7);

    pidUpdateCounter = (pidUpdateCounter + 1) % activePidLoopDenom;
}

timeUs_t getLastDisarmTimeUs(void)
{
    return lastDisarmTimeUs;
}

